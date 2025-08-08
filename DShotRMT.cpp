/**
 * @file DShotRMT.cpp
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include "DShotRMT.h"

// --- DShot Timings ---
// frame_length_us, ticks_per_bit, ticks_one_high, ticks_zero_high, ticks_zero_low, ticks_one_low
constexpr dshot_timing_t DSHOT_TIMINGS[] = {
    {0, 0, 0, 0, 0, 0},        // DSHOT_OFF
    {128, 64, 48, 24, 40, 16}, // DSHOT150
    {64, 32, 24, 12, 20, 8},   // DSHOT300
    {32, 16, 12, 6, 10, 4},    // DSHOT600
    {16, 8, 6, 3, 5, 2}        // DSHOT1200
};

// --- DShot Config ---
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool is_bidirectional): 
    _gpio(gpio),
    _mode(mode),
    _is_bidirectional(is_bidirectional),
    _timing_config(DSHOT_TIMINGS[mode]),
    _rmt_tx_channel(nullptr),
    _rmt_rx_channel(nullptr),
    _dshot_encoder(nullptr),
    _last_erpm(0),
    _current_packet(0),
    _packet{0},
    _last_transmission_time(0)
{
    // Double up frame time for bidirectional mode
    if (_is_bidirectional)
    {
        _frame_time_us = (_timing_config.frame_length_us << 1) + DSHOT_SWITCH_TIME;
    }

    // Calculate frame time including switch time
    _frame_time_us = _timing_config.frame_length_us + DSHOT_SWITCH_TIME;
}

// Easy Constructor
DShotRMT::DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional):
    DShotRMT((gpio_num_t)pin_nr, 
    mode, 
    is_bidirectional)
{
}

// Init DShotRMT
bool DShotRMT::begin()
{
    // Init TX Channel
    if (!_initTXChannel())
    {
        Serial.println(DSHOT_MSG_01);
        return DSHOT_ERROR;
    }

    // Init RX Channel
    if (!_initRXChannel() && _is_bidirectional)
    {
        Serial.println(DSHOT_MSG_02);
        return DSHOT_ERROR;
    }

    // Init DShot Decoder
    if (!_initDShotEncoder())
    {
        Serial.println(DSHOT_MSG_03);
        return DSHOT_ERROR;
    }

    // All good, ready
    return DSHOT_OK;
}

// Deprecated, use "sendThrottle() instead"
bool DShotRMT::setThrottle(uint16_t throttle)
{
    return sendThrottle(throttle);
}

// Sends a valid throttle value
bool DShotRMT::sendThrottle(uint16_t throttle)
{
    static uint16_t last_throttle = DSHOT_THROTTLE_MIN;

    // Precheck throttle value
    if (throttle < DSHOT_THROTTLE_MIN || throttle > DSHOT_THROTTLE_MAX)
    {
        Serial.println(DSHOT_MSG_06);
        return DSHOT_ERROR;
    } else {
        last_throttle = throttle;
    }

    // Converts throttle value to dshot packet 
    _packet = _buildDShotPacket(last_throttle);

    return (_sendDShotFrame(_packet));
}

// Deprecated, use "sendCommand() instead"
bool DShotRMT::sendDShotCommand(uint16_t command)
{
    return sendCommand(command);
}

bool DShotRMT::sendCommand(uint16_t command)
{
    // Check for valid command
    if (command < DSHOT_CMD_MOTOR_STOP || command > DSHOT_CMD_MAX)
    {
        Serial.println(DSHOT_MSG_07);
        return DSHOT_ERROR;
    }

    _packet = _buildDShotPacket(command);

    return (_sendDShotFrame(_packet));
}

//
uint16_t DShotRMT::getERPM()
{
    if (!_is_bidirectional || !_rmt_rx_channel)
    {
        Serial.println(DSHOT_MSG_08);
        return _last_erpm;
    }

    // Try to receive telemetry data
    rmt_symbol_word_t _rx_symbols[RX_BUFFER_SIZE];
    if (!rmt_receive(_rmt_rx_channel, _rx_symbols, DSHOT_SYMBOLS_SIZE, &_receive_config))
    {
        Serial.println(DSHOT_MSG_09);
        return _last_erpm;
    }

    // Decodes the response
    uint16_t new_erpm = _decodeDShotFrame(_rx_symbols);
    if (new_erpm != 0)
    {
        _last_erpm = new_erpm;
    }

    return _last_erpm;
}

//
uint32_t DShotRMT::getMotorRPM(uint8_t magnet_count)
{
    uint8_t pole_pairs = max(1, magnet_count / 2);
    return getERPM() / pole_pairs;
}

//
bool DShotRMT::_initTXChannel()
{
    // --- RMT TX Config ---
    _tx_channel_config.gpio_num = _gpio;
    _tx_channel_config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    _tx_channel_config.resolution_hz = DSHOT_RMT_RESOLUTION;
    _tx_channel_config.mem_block_symbols = DSHOT_SYMBOLS_SIZE;
    _tx_channel_config.trans_queue_depth = TX_BUFFER_SIZE;

    // No loops, real time calculation for each frame
    _transmit_config.loop_count = 0;

    // ...it's a trap
    _transmit_config.flags.eot_level = _is_bidirectional ? 1 : 0;

    // Creates and activates RMT TX Channel
    if (rmt_new_tx_channel(&_tx_channel_config, &_rmt_tx_channel) != DSHOT_OK)
    {
        return DSHOT_ERROR;
    }

    return (rmt_enable(_rmt_tx_channel) == 0);
}

//
bool DShotRMT::_initRXChannel()
{
    // --- RMT RX Config ---
    _rx_channel_config.gpio_num = _gpio;
    _rx_channel_config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    _rx_channel_config.resolution_hz = DSHOT_RMT_RESOLUTION;
    _rx_channel_config.mem_block_symbols = DSHOT_SYMBOLS_SIZE;

    _receive_config.signal_range_min_ns = 300;
    _receive_config.signal_range_max_ns = 5000;

    // Creates and activates RMT TX Channel
    if (rmt_new_rx_channel(&_rx_channel_config, &_rmt_rx_channel) != DSHOT_OK)
    {
        return DSHOT_ERROR;
    }

    return (rmt_enable(_rmt_rx_channel) == 0);
}

//
bool DShotRMT::_initDShotEncoder()
{
    // Creates a dummy encoder
    rmt_copy_encoder_config_t encoder_config = {};
    return rmt_new_copy_encoder(&encoder_config, &_dshot_encoder) == 0;
}

// Use RMT to transmit a prepared DShot packet and returns it
bool DShotRMT::_sendDShotFrame(const dshot_packet_t &packet)
{
    // Excludes calculation from timing is more stable
    rmt_symbol_word_t tx_symbols[DSHOT_BITS_PER_FRAME];
    _encodeDShotFrame(packet, tx_symbols);

    // Checking timer signal
    if (_timer_signal())
    {
        // Triggers RMT Transmit
        rmt_transmit(_rmt_tx_channel, _dshot_encoder, tx_symbols, DSHOT_SYMBOLS_SIZE, &_transmit_config);

        // Time Stamp
        return _timer_reset();
    }

    return DSHOT_ERROR;
}

// Calculates checksum for given package
uint16_t DShotRMT::_calculateCRC(const dshot_packet_t &packet)
{
    uint16_t data = (packet.throttle_value << 1) | packet.telemetric_request;
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0b0000000000001111;

    // Inverts CRC for bidirectional DShot
    if (_is_bidirectional)
    {
        crc = (~crc) & 0b0000000000001111;
    }

    return crc;
}

// Returns bitwise parsed DShot packet
uint16_t DShotRMT::_parseDShotPacket(const dshot_packet_t &packet)
{
    uint16_t data = (packet.throttle_value << 1) | packet.telemetric_request;
    return (data << 4) | _calculateCRC(packet);
}

// Returns a "true" DShot Packet ready to roll
dshot_packet_t DShotRMT::_buildDShotPacket(const uint16_t value)
{
    // DShot Frame Container
    dshot_packet_t packet = {};

    // Creates DShot packet
    packet.throttle_value = value;
    packet.telemetric_request = 0;
    packet.checksum = _calculateCRC(packet);

    //
    return packet;
}

// Encodes DShot packet into RMT buffer
bool IRAM_ATTR DShotRMT::_encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols)
{
    // Parse actual packet into buffer
    _current_packet = _parseDShotPacket(packet);

    // Converts the parsed dshot frame to rmt_tx data
    for (int i = 0; i < DSHOT_BITS_PER_FRAME; i++)
    {
        // Encoded RMT symbols bitwise (MSB first) - tricky
        bool bit = (_current_packet >> (DSHOT_BITS_PER_FRAME - 1 - i)) & 0b0000000000000001;
        if (_is_bidirectional)
        {
            symbols[i].level0 = 0;
            symbols[i].duration0 = bit ? _timing_config.ticks_one_high : _timing_config.ticks_zero_high;
            symbols[i].level1 = 1;
            symbols[i].duration1 = bit ? _timing_config.ticks_one_low : _timing_config.ticks_zero_low;
        }
        else
        {
            symbols[i].level0 = 1;
            symbols[i].duration0 = bit ? _timing_config.ticks_one_high : _timing_config.ticks_zero_high;
            symbols[i].level1 = 0;
            symbols[i].duration1 = bit ? _timing_config.ticks_one_low : _timing_config.ticks_zero_low;
        }
    }
    return DSHOT_OK;
}

//
uint16_t DShotRMT::_decodeDShotFrame(const rmt_symbol_word_t *symbols)
{
    uint16_t received_frame = 0;

    // Decodes each symbol to reconstruct the frame
    for (size_t i = 0; i < DSHOT_BITS_PER_FRAME; ++i)
    {
        bool bit = symbols[i].duration0 < symbols[i].duration1;
        received_frame = (received_frame << 1) | bit;
    }

    // Extracts payload and CRC
    uint16_t data = received_frame >> 4;
    uint16_t received_crc = received_frame & 0b0000000000001111;

    // Calculates CRC for received frame
    uint16_t calculated_crc = (data ^ (data >> 4) ^ (data >> 8)) & 0b0000000000001111;
    if (_is_bidirectional)
    {
        calculated_crc = (~calculated_crc) & 0b0000000000001111;
    }

    // Compares CRC
    if (received_crc != calculated_crc)
    {
        Serial.println(DSHOT_MSG_04);
        return 0b0000000000000000;
    }

    // Removes telemetry bit and returns 10bit value
    return data >> 1;
}

// Timer triggered
bool DShotRMT::_timer_signal()
{
    return (micros() - _last_transmission_time) >= _frame_time_us;
}

// Updates timestamp
bool DShotRMT::_timer_reset()
{
    _last_transmission_time = micros();
    return DSHOT_OK;
}
