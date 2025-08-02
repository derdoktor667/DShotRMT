/**
 * @file DShotRMT.cpp
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include "DShotRMT.h"

// --- DShot Timings ---
const dshot_timing_t DSHOT_TIMINGS[] = {
    {0, 0, 0, 0, 0, 0},        // DSHOT_OFF
    {128, 64, 48, 24, 40, 16}, // DSHOT150
    {64, 32, 24, 12, 20, 8},   // DSHOT300
    {32, 16, 12, 6, 10, 4},    // DSHOT600
    {16, 8, 6, 3, 5, 2}        // DSHOT1200
};

//
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool is_bidirectional)
    : _gpio(gpio), _mode(mode)
    , _is_bidirectional(is_bidirectional)
    , _timing_config(DSHOT_TIMINGS[mode])
    , _rmt_tx_channel(nullptr)
    , _rmt_rx_channel(nullptr)
    , _dshot_encoder(nullptr)
    , _last_erpm(0)
    , _last_transmission_time(0)
{
    // Calculate frame time including switch time
    _frame_time_us = _timing_config.frame_length_us + DSHOT_SWITCH_TIME;

    // Double up frame time for bidirectional mode
    if (_is_bidirectional)
    {
        _frame_time_us += _frame_time_us;
    }
}

//
bool DShotRMT::begin()
{
    if (!_initTXChannel())
    {
        Serial.println("Failed to initialize TX channel");
        return DSHOT_ERROR;
    }

    if (!_initRXChannel() && _is_bidirectional)
    {
        Serial.println("Failed to initialize RX channel");
        return DSHOT_ERROR;
    }

    if (!_initDShotEncoder())
    {
        Serial.println("Failed to initialize encoder");
        return DSHOT_ERROR;
    }

    return DSHOT_OK;
}

//
bool DShotRMT::setThrottle(uint16_t throttle)
{
    // DShot Frame Container
    dshot_packet_t packet = {};

    // Create DShot packet
    packet.throttle_value = constrain(throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    packet.telemetric_request = _is_bidirectional;
    packet.checksum = _calculateCRC(packet);

    if (!_sendDShotFrame(packet))
    {
        return DSHOT_ERROR;
    }
}

//
bool DShotRMT::sendDShotCommand(uint16_t command)
{
    // DShot Frame Container
    dshot_packet_t packet = {};

    // Create DShot packet
    packet.throttle_value = constrain(command, DSHOT_CMD_MOTOR_STOP, DSHOT_CMD_MAX);
    packet.telemetric_request = _is_bidirectional;
    packet.checksum = _calculateCRC(packet);

    if (!_sendDShotFrame(packet))
    {
        return DSHOT_ERROR;
    }
}

//
uint32_t DShotRMT::getERPM()
{
    if (!_is_bidirectional || !_rmt_rx_channel)
    {
        return _last_erpm;
    }

    // Try to receive telemetry data
    if (!rmt_receive(_rmt_rx_channel, _rx_symbols, DSHOT_SYMBOLS_SIZE, &_receive_config))
    {
        return _last_erpm;
    }

    // Decode the response
    uint16_t new_erpm = _decodeDShotFrame(_rx_symbols, DSHOT_BITS_PER_FRAME);
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

    _transmit_config.loop_count = 0;

    // ...it's a trap
    _transmit_config.flags.eot_level = _is_bidirectional ? 1 : 0;

    // Creates and activate RMT TX Channel
    if (rmt_new_tx_channel(&_tx_channel_config, &_rmt_tx_channel) != DSHOT_OK)
    {
        return DSHOT_ERROR;
    }

    return rmt_enable(_rmt_tx_channel) == 0;
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

    // Create and activate RMT TX Channel
    if (rmt_new_rx_channel(&_rx_channel_config, &_rmt_rx_channel) != DSHOT_OK)
    {
        return DSHOT_ERROR;
    }

    return rmt_enable(_rmt_rx_channel) == 0;
}

//
bool DShotRMT::_initDShotEncoder()
{
    // Creates a dummy encoder
    rmt_copy_encoder_config_t encoder_config = {};
    return rmt_new_copy_encoder(&encoder_config, &_dshot_encoder) == 0;
}

//
bool DShotRMT::_sendDShotFrame(const dshot_packet_t &packet)
{
    //
    if (!_timer_signal())
    {
        return DSHOT_ERROR;
    }

    // Encodes packet directly into RMT Buffer
    rmt_symbol_word_t tx_symbols[DSHOT_BITS_PER_FRAME];
    _encodeDShotFrame(packet, tx_symbols);

    // Trigger RMT Transmit
    if (rmt_transmit(_rmt_tx_channel, _dshot_encoder, tx_symbols, DSHOT_SYMBOLS_SIZE, &_transmit_config) != 0)
    {
        Serial.println("Failed to transmit DShot packet");
        return DSHOT_ERROR;
    }

    _timer_reset();
    return DSHOT_OK;
}

//
uint16_t DShotRMT::_calculateCRC(const dshot_packet_t &packet)
{
    uint16_t data = (packet.throttle_value << 1) | packet.telemetric_request;
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0b0000000000001111;

    // Invert CRC for bidirectional DShot
    if (packet.telemetric_request)
    {
        crc = (~crc) & 0b0000000000001111;
    }

    return crc;
}

//
uint16_t DShotRMT::_assembleDShotFrame(const dshot_packet_t &packet)
{
    // Parses DShot Frame
    uint16_t data = (packet.throttle_value << 1) | packet.telemetric_request;
    return (data << 4) | packet.checksum;
}

//
void DShotRMT::_encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols)
{
    {
        // Encoding to "raw" DShot Packet
        uint16_t frame_bits = _assembleDShotFrame(packet);

        // Convert the parsed dshot frame to rmt_tx data
        for (int i = 0; i < DSHOT_BITS_PER_FRAME; i++)
        {
            // Encode RMT symbols bitwise (MSB first) - tricky
            bool bit = (frame_bits >> (DSHOT_BITS_PER_FRAME - 1 - i)) & 0b0000000000000001;
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
    }
}

//
uint16_t DShotRMT::_decodeDShotFrame(const rmt_symbol_word_t *symbols, size_t symbol_count)
{
    uint16_t received_frame = 0;

    // Decode each symbol to reconstruct the frame
    for (size_t i = 0; i < DSHOT_BITS_PER_FRAME; ++i)
    {
        bool bit = symbols[i].duration0 < symbols[i].duration1;
        received_frame = (received_frame << 1) | bit;
    }

    // Extract payload and CRC
    uint16_t data = received_frame >> 4;
    uint8_t received_crc = received_frame & 0b0000000000001111;

    // Calculate CRC for received frame
    uint8_t calculated_crc = (data ^ (data >> 4) ^ (data >> 8)) & 0b0000000000001111;
    if (_is_bidirectional)
    {
        calculated_crc = (~calculated_crc) & 0b0000000000001111;
    }

    // Compare CRC
    if (received_crc != calculated_crc)
    {
        Serial.println("RX CRC Check failed");
        return 0b0000000000000000;
    }

    // Remove telemetry bit and return eRPM
    return data >> 1;
}

//
bool DShotRMT::_timer_signal()
{
    return (micros() - _last_transmission_time) >= _frame_time_us;
}

//
void DShotRMT::_timer_reset()
{
    _last_transmission_time = micros();
}
