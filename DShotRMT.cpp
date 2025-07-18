/**
 * @file DShotRMT.cpp
 * @brief DShot signal generation using ESP32 RMT with continuous repeat and pause between frames, including BiDirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include <DShotRMT.h>

// --- DShotRMT Class ---
// This class provides an abstraction for sending and optionally receiving DShot frames.
// It uses ESP32's RMT peripheral for precise timing control, including BiDirectional RX.
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool isBidirectional)
    : _gpio(gpio), _mode(mode), _isBidirectional(isBidirectional) {}

// Sets up RMT TX and RX channels as well as encoder configuration
void DShotRMT::begin()
{
    // TX RMT Channel Configuration
    _rmt_tx_channel_config = {
        .gpio_num = _gpio,
        .clk_src = DSHOT_CLOCK_SRC_DEFAULT,
        .resolution_hz = DSHOT_RMT_RESOLUTION,
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
        // .flags = {
        // invert Signal if BiDirectional DShot Mode
        // .invert_out = _isBidirectional,
        // .with_dma = false}
    };

    rmt_new_tx_channel(&_rmt_tx_channel_config, &_rmt_tx_channel);
    rmt_enable(_rmt_tx_channel);

    // RX RMT Channel Configuration (for BiDirectional DShot)
    if (_isBidirectional)
    {
        _rmt_rx_channel_config = {
            .gpio_num = _gpio,
            .clk_src = DSHOT_CLOCK_SRC_DEFAULT,
            .resolution_hz = DSHOT_RMT_RESOLUTION,
            .mem_block_symbols = 64,
            // .flags = {
            //     .invert_in = false,
            //     .with_dma = false}
        };

        rmt_new_rx_channel(&_rmt_rx_channel_config, &_rmt_rx_channel);
        rmt_enable(_rmt_rx_channel);

        _receive_config.signal_range_min_ns = 300;
        _receive_config.signal_range_max_ns = 5000;
    }

    // Use a copy encoder to send raw symbols
    if (!_dshot_encoder)
    {
        rmt_copy_encoder_config_t enc_cfg = {};
        rmt_new_copy_encoder(&enc_cfg, &_dshot_encoder);
    }

    // Configure transmission looping
    _transmit_config.loop_count = 0;
    _transmit_config.flags.eot_level = _isBidirectional;
}

// Encodes and transmits a valid DShot Throttle value (48 - 2047)
void DShotRMT::setThrottle(uint16_t throttle)
{
    // Safety first - double check input range and 11 bit "translation"
    throttle = constrain(throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX) & 0b0000011111111111;

    _lastThrottle = throttle;

    // Convert throttle value to DShot Paket Format
    _tx_packet = assambleDShotPaket(_lastThrottle);

    // Encode RMT symbols
    size_t count = 0;
    encodeDShotTX(_tx_packet, _tx_symbols, count);

    // Send the packet
    rmt_transmit(_rmt_tx_channel, _dshot_encoder, _tx_symbols, count * sizeof(rmt_symbol_word_t), &_transmit_config);

    // Take a break
    esp_rom_delay_us(120);
}

// --- Get eRPM from ESC ---
// Receives and decodes a response frame from ESC containing eRPM info
uint32_t DShotRMT::getERPM()
{
    if (_isBidirectional)
    {
        if (_rmt_rx_channel == nullptr)
            return _last_erpm;

        // Attempt to receive a new frame
        if (!rmt_receive(_rmt_rx_channel, _rx_symbols, sizeof(_rx_symbols), &_receive_config))
            return _last_erpm;

        //
        _last_erpm = decodeDShotRX(_rx_symbols, DSHOT_BITS_PER_FRAME);
        return _last_erpm;
    }

    // Nothing to do here
    return _last_erpm;
}

// Translate eRPM value to RPM taking magnet count as parameter
uint32_t DShotRMT::getMotorRPM(uint8_t magnet_count)
{
    uint8_t pole_count = magnet_count / 2;

    if (pole_count == 0)
        pole_count = 1;

    uint32_t rpm = getERPM() / pole_count;
    return rpm;
}

// Calculate CRC for DShot Paket
uint16_t DShotRMT::calculateCRC(uint16_t dshot_packet)
{
    uint16_t _packet = (dshot_packet << 1) | (_isBidirectional ? 1 : 0);

    // Clear container before new calculation
    _packet_crc = DSHOT_NULL_PACKET;

    // CRC calculation for DShot (4 bits)
    _packet_crc = ((_packet ^ (_packet >> 4) ^ (_packet >> 8)) & 0b0000000000001111);

    // CRC is inverted for biDirectional DShot
    if (_isBidirectional)
        _packet_crc = (~_packet_crc) & 0b0000000000001111;

    return _packet_crc;
}

// Assamble DShot Paket (11 bit throttle + 1 bit telemetry request + 4 bit crc)
uint16_t DShotRMT::assambleDShotPaket(uint16_t value)
{
    // Dummy conversion to 11 bits
    uint16_t _value = value & 0b0000011111111111;

    // Clear container
    _tx_packet = DSHOT_NULL_PACKET;

    // Assemble raw DShot packet and add checksum
    _packet_crc = calculateCRC(_value);

    _tx_packet = (_value << 1) | (_isBidirectional ? 1 : 0);
    _tx_packet = (_tx_packet << 4) | _packet_crc;

    return _tx_packet;
}

// --- Encode DShot TX Frame ---
// Converts a 16-bit packet into a valid DShot Frame for RMT
void DShotRMT::encodeDShotTX(uint16_t dshot_packet, rmt_symbol_word_t *symbols, size_t &count)
{
    // Always start encoding from the top
    count = 0;

    uint32_t ticks_per_bit = 0;
    uint32_t ticks_zero_high = 0;
    uint32_t ticks_one_high = 0;

    switch (_mode)
    {
    case DSHOT150:
        ticks_per_bit = 64;
        ticks_zero_high = 24;
        ticks_one_high = 48;
        break;
    case DSHOT300:
        ticks_per_bit = 32;
        ticks_zero_high = 12;
        ticks_one_high = 24;
        break;
    case DSHOT600:
        ticks_per_bit = 16;
        ticks_zero_high = 6;
        ticks_one_high = 12;
        break;
    case DSHOT1200:
        ticks_per_bit = 8;
        ticks_zero_high = 3;
        ticks_one_high = 6;
        break;
    case DSHOT_OFF:
    default:
        ticks_per_bit = 0;
        ticks_zero_high = 0;
        ticks_one_high = 0;
        break;
    }

    uint32_t ticks_zero_low = ticks_per_bit - ticks_zero_high;
    uint32_t ticks_one_low = ticks_per_bit - ticks_one_high;

    // Fill the 16 DShot-Bits Array with selected timings
    for (int i = 15; i >= 0; i--)
    {
        bool bit = (dshot_packet >> i) & 0x01;
        if (_isBidirectional)
        {
            symbols[count].level0 = 0;
            symbols[count].duration0 = bit ? ticks_one_high : ticks_zero_high;
            symbols[count].level1 = 1;
            symbols[count].duration1 = bit ? ticks_one_low : ticks_zero_low;
        }
        else
        {
            symbols[count].level0 = 1;
            symbols[count].duration0 = bit ? ticks_one_high : ticks_zero_high;
            symbols[count].level1 = 0;
            symbols[count].duration1 = bit ? ticks_one_low : ticks_zero_low;
        }
        count++;
    }
}

// Decodes a response frame from ESC containing eRPM info
uint16_t DShotRMT::decodeDShotRX(const rmt_symbol_word_t *symbols, uint32_t count)
{
    // Container for received frame
    uint16_t _rec_frame = DSHOT_NULL_PACKET;

    // Fill the Frame bit by bit
    for (size_t i = 0; i < DSHOT_BITS_PER_FRAME && i < count; ++i)
    {
        bool bit = (symbols[i].duration0 < symbols[i].duration1);
        _rec_frame = (_rec_frame << 1) | bit;
    }

    // Store the received CRC for checking
    uint16_t _temp = _rec_frame >> 4;

    // Masking the received CRC
    uint8_t crc_recv = _rec_frame & 0x0F;

    // Calculate CRC for received frame again
    uint8_t crc_calc = (_temp ^ (_temp >> 4) ^ (_temp >> 8)) & 0b0000000000001111;

    if (_isBidirectional)
        crc_calc = (~crc_calc) & 0x0F;

    // Checking CRC
    if (crc_recv != crc_calc)
        return _last_erpm;

    // Cut "telemetric" bit leaving "raw" value
    uint16_t raw = _temp >> 1;

    return _last_erpm = raw;
}
