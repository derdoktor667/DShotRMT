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
    : _gpio(gpio), _mode(mode), _isBidirectional(isBidirectional)
{

    // Setting up fixed DShot Frame length
    switch (_mode)
    {
    case DSHOT_OFF:
        _frameLenght = 0;
        break;
    case DSHOT150:
        _frameLenght = 128;
        break;
    case DSHOT300:
        _frameLenght = 64;
        break;
    case DSHOT600:
        _frameLenght = 32;
        break;
    case DSHOT1200:
        _frameLenght = 16;
        break;
    default:
        break;
    }

    // DShot Frame length incl. DShot answer duration
    if (_isBidirectional)
    {
        _frameLenght += _frameLenght;
    }
}

// Initializes RMT TX and RX channels and encoder configuration
void DShotRMT::begin()
{
    // Configure RX RMT Channel for BiDirectional DShot
    if (_isBidirectional)
    {
        _rmt_rx_channel_config = {
            .gpio_num = _gpio,
            .clk_src = DSHOT_CLOCK_SRC_DEFAULT,
            .resolution_hz = DSHOT_RMT_RESOLUTION,
            .mem_block_symbols = 64,
        };

        if (rmt_new_rx_channel(&_rmt_rx_channel_config, &_rmt_rx_channel) != 0)
        {
            Serial.println("Failed to create RX channel");
            return;
        }
        if (rmt_enable(_rmt_rx_channel) != 0)
        {
            Serial.println("Failed to enable RX channel");
            return;
        }

        _receive_config.signal_range_min_ns = 300;
        _receive_config.signal_range_max_ns = 5000;

        _dshot_packet.telemetric_request = _isBidirectional;
    }

    // Configure TX RMT Channel
    _rmt_tx_channel_config = {
        .gpio_num = _gpio,
        .clk_src = DSHOT_CLOCK_SRC_DEFAULT,
        .resolution_hz = DSHOT_RMT_RESOLUTION,
        .mem_block_symbols = 64,
        .trans_queue_depth = 10,
    };

    // Transmission configuration
    _transmit_config.loop_count = 0;
    _transmit_config.flags.eot_level = _isBidirectional;

    if (rmt_new_tx_channel(&_rmt_tx_channel_config, &_rmt_tx_channel) != 0)
    {
        Serial.println("Failed to create TX channel");
        return;
    }
    if (rmt_enable(_rmt_tx_channel) != 0)
    {
        Serial.println("Failed to enable TX channel");
        return;
    }

    // Create copy encoder for raw symbol transmission
    if (!_dshot_encoder)
    {
        rmt_copy_encoder_config_t enc_cfg = {};
        if (rmt_new_copy_encoder(&enc_cfg, &_dshot_encoder) != 0)
        {
            Serial.println("Failed to create copy encoder");
            return;
        }
    }
}

// Encodes and transmits a valid DShot throttle value (48 - 2047)
void DShotRMT::setThrottle(uint16_t throttle)
{
    // Simple timer
    static unsigned long last_time = NULL;

    // Ensure frame lenght for compatibility
    if (micros() - last_time >= _frameLenght)
    {
        // Clamp input range for throttle value
        _dshot_packet.throttle_value = constrain(throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX) & 0b0000011111111111;

        // Calculate CRC for every throttle value
        calculateCRC(&_dshot_packet);

        // Encode RMT symbols
        size_t count = NULL;
        encodeDShotTX(&_dshot_packet, _tx_symbols, count);

        // Transmit the packet
        if (rmt_transmit(_rmt_tx_channel, _dshot_encoder, _tx_symbols, count * sizeof(rmt_symbol_word_t), &_transmit_config) != 0)
        {
            Serial.println("Failed to transmit DShot packet");
            return;
        }

        // Timestamp
        last_time = micros();
    }
}

// Receives and decodes a response frame from ESC containing eRPM info
uint32_t DShotRMT::getERPM()
{
    if (_isBidirectional)
    {
        if (_rmt_rx_channel == nullptr)
        {
            Serial.println("No bidirectional DShot support.");
            return _last_erpm;
        }

        // Try to receive a new frame
        if (!rmt_receive(_rmt_rx_channel, _rx_symbols, sizeof(_rx_symbols), &_receive_config))
        {
            Serial.println("No valid DShot frame received");
            return _last_erpm;
        }

        _last_erpm = decodeDShotRX(_rx_symbols, DSHOT_BITS_PER_FRAME);
        return _last_erpm;
    }

    // No RX possible in non-bidirectional mode
    return _last_erpm;
}

// Converts eRPM value to RPM using magnet count
uint32_t DShotRMT::getMotorRPM(uint8_t magnet_count)
{
    uint8_t pole_count = magnet_count / 2;
    if (pole_count == 0)
        pole_count = 1;

    return getERPM() / pole_count;
}

// Calculates CRC for DShot packet
void DShotRMT::calculateCRC(dshot_packet_t *dshot_packet)
{
    uint16_t packet = (dshot_packet->throttle_value << 1) | (dshot_packet->telemetric_request);

    // Reset CRC container
    dshot_packet->checksum = DSHOT_NULL_PACKET;

    // CRC calculation for DShot (4 bits)
    dshot_packet->checksum = ((packet ^ (packet >> 4) ^ (packet >> 8)) & 0b0000000000001111);

    // CRC is inverted for bidirectional DShot
    if (dshot_packet->telemetric_request)
        dshot_packet->checksum = (~dshot_packet->checksum) & 0b0000000000001111;
}

// Assembles DShot packet (11 bit throttle + 1 bit telemetry request + 4 bit CRC)
uint16_t DShotRMT::parseDShotPacket(const dshot_packet_t *dshot_packet) const
{
    uint16_t parsed_packet = ((dshot_packet->throttle_value << 1) | (dshot_packet->telemetric_request)) & 0b0000111111111111;
    return ((parsed_packet << 4) | (dshot_packet->checksum)) & 0b1111111111111111;
}

// Converts a 16-bit packet into a valid DShot frame for RMT
void DShotRMT::encodeDShotTX(dshot_packet_t *dshot_packet, rmt_symbol_word_t *symbols, size_t &count)
{
    count = 0;

    uint16_t frame_bits = parseDShotPacket(dshot_packet);

    uint32_t ticks_per_bit = 0;
    uint32_t ticks_zero_high = 0;
    uint32_t ticks_one_high = 0;

    // Select timing based on DShot mode
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
        return;
    }

    uint32_t ticks_zero_low = ticks_per_bit - ticks_zero_high;
    uint32_t ticks_one_low = ticks_per_bit - ticks_one_high;

    // Convert the parsed dshot frame to rmt_tx data
    for (int i = 15; i >= 0; i--)
    {
        bool bit = (frame_bits >> i) & 0x1;
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
    uint16_t received_frame = DSHOT_NULL_PACKET;

    // Build the frame bit by bit
    for (size_t i = 0; i < DSHOT_BITS_PER_FRAME && i < count; ++i)
    {
        bool bit = (symbols[i].duration0 < symbols[i].duration1);
        received_frame = (received_frame << 1) | bit;
    }

    // Extract CRC and payload
    uint16_t payload = received_frame >> 4;
    uint8_t crc_received = received_frame & 0b0000000000001111;

    // Calculate CRC for received frame
    uint8_t crc_calculated = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0b0000000000001111;
    if (_isBidirectional)
        crc_calculated = (~crc_calculated) & 0b0000000000001111;

    // Check CRC
    if (crc_received != crc_calculated)
    {
        Serial.println("RX - CRC check failed.");
        return _last_erpm;
    }

    // Remove telemetry bit
    return _last_erpm = payload >> 1;
}
