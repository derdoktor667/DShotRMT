/**
 * @file DShotRMT.cpp
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include "DShotRMT.h"

// Configuration Constants
static constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;
static constexpr auto DSHOT_FULL_PACKET = 0b1111111111111111;
static constexpr auto DSHOT_RX_TIMEOUT_MS = 2;
static constexpr auto DSHOT_PADDING_US = 20; // Pause between frames
static constexpr auto GCR_BITS_PER_FRAME = 21; // GCR bits in a DShot answer frame
static constexpr auto POLE_PAIRS_MIN = 1;
static constexpr auto MAGNETS_PER_POLE_PAIR = 2;
static constexpr auto NO_DSHOT_TELEMETRY = 0;
static constexpr auto DSHOT_PULSE_MIN_NS = 800;  // 0.8us minimum pulse
static constexpr auto DSHOT_PULSE_MAX_NS = 8000; // 8.0us maximum pulse
static constexpr auto DSHOT_TELEMETRY_INVALID = DSHOT_THROTTLE_MAX;
static constexpr auto DSHOT_TELEMETRY_BIT_POSITION = 11;
static constexpr auto DSHOT_CRC_BIT_SHIFT = 4;

// Command Constants
static constexpr auto DEFAULT_CMD_DELAY_US = 10;
static constexpr auto DEFAULT_CMD_REPEAT_COUNT = 1;
static constexpr auto SETTINGS_COMMAND_REPEATS = 10;
static constexpr auto SETTINGS_COMMAND_DELAY_US = 5;

// Constructor with GPIO number
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count)
    : _gpio(gpio),
      _mode(mode),
      _is_bidirectional(is_bidirectional),
      _motor_magnet_count(magnet_count),
      _dshot_timing(DSHOT_TIMING_US[_mode])
{
    // Pre-calculate timing and bit positions for performance
    _preCalculateRMTTicks();
}

// Constructor using pin number
DShotRMT::DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count)
    : DShotRMT(static_cast<gpio_num_t>(pin_nr), mode, is_bidirectional, magnet_count)
{
    // Delegates to primary constructor with type cast
}

// Destructor
DShotRMT::~DShotRMT()
{
    _cleanupRmtResources();
}

// Initialize DShotRMT
dshot_result_t DShotRMT::begin()
{
    dshot_result_t result = init_rmt_tx_channel(_gpio, &_rmt_tx_channel, _is_bidirectional);

    if (!result.success)
    {
        _cleanupRmtResources(); // Clean up any allocated resources on failure
        return result;
    }

    if (_is_bidirectional)
    {
        result = init_rmt_rx_channel(_gpio, &_rmt_rx_channel, &_rx_event_callbacks, this);
        if (!result.success)
        {
            _cleanupRmtResources(); // Clean up any allocated resources on failure
            return result;
        }
    }

    result = init_dshot_encoder(&_dshot_encoder, _rmt_ticks, _pulse_level, _idle_level);

    if (!result.success)
    {
        _cleanupRmtResources(); // Clean up any allocated resources on failure
        return result;
    }

    return {true, DSHOT_INIT_SUCCESS};
}

// Send throttle value
dshot_result_t DShotRMT::sendThrottle(uint16_t throttle)
{
    // A throttle value of 0 is a disarm command
    if (throttle == 0)
    {   
        // just to be sure
        _last_throttle = 0;
        return sendCommand(DSHOT_CMD_MOTOR_STOP);
    }

    // Constrain throttle to the valid DShot range
    _last_throttle = constrain(throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

    _packet = _buildDShotPacket(_last_throttle);
    return _sendPacket(_packet);
}

// Send throttle value as a percentage
dshot_result_t DShotRMT::sendThrottlePercent(float percent)
{
    if (percent < DSHOT_PERCENT_MIN || percent > DSHOT_PERCENT_MAX)
    {
        return {false, DSHOT_PERCENT_NOT_IN_RANGE};
    }

    // Map percent to DShot throttle range
    uint16_t throttle = static_cast<uint16_t>(DSHOT_THROTTLE_MIN + ((DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN) / DSHOT_PERCENT_MAX) * percent);
    return sendThrottle(throttle);
}

// Sends a DShot command (0-47) to the ESC by accepting an integer value.
dshot_result_t DShotRMT::sendCommand(uint16_t command_value)
{
    // Validate the integer command value before casting
    if (command_value < DSHOT_CMD_MOTOR_STOP || command_value > DSHOT_CMD_MAX_VALUE)
    {
        return {false, DSHOT_COMMAND_NOT_VALID};
    }
    return sendCommand(static_cast<dshotCommands_e>(command_value));
}

// Sends a DShot command (0-47) to the ESC.
dshot_result_t DShotRMT::sendCommand(dshotCommands_e command)
{
    uint16_t repeat_count = DEFAULT_CMD_REPEAT_COUNT;
    uint16_t delay_us = DEFAULT_CMD_DELAY_US;

    switch (command)
    {
    case DSHOT_CMD_MOTOR_STOP:
    case DSHOT_CMD_SAVE_SETTINGS:
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
        repeat_count = SETTINGS_COMMAND_REPEATS;
        delay_us = SETTINGS_COMMAND_DELAY_US;
        break;
    default:
        // For other commands, use default repeat and delay
        break;
    }

    return sendCommand(command, repeat_count, delay_us);
}

// Sends a DShot command (0-47) to the ESC with a specified repeat count and delay.
dshot_result_t DShotRMT::sendCommand(dshotCommands_e command, uint16_t repeat_count, uint16_t delay_us)
{
    dshot_result_t result = {false, DSHOT_UNKNOWN, NO_DSHOT_TELEMETRY, NO_DSHOT_TELEMETRY};

    if (!_isValidCommand(command))
    {
        result.result_code = DSHOT_INVALID_COMMAND;
        return result;
    }

    bool all_successful = true;

    // Send command multiple times with delay
    for (uint16_t i = 0; i < repeat_count; i++)
    {
        dshot_result_t single_result = _executeCommand(command);

        if (!single_result.success)
        {
            all_successful = false;
            result.result_code = single_result.result_code;
            break;
        }

        // Add delay between repetitions (except for last repetition)
        if (i < repeat_count - 1)
        {
            delayMicroseconds(delay_us);
        }
    }

    result.success = all_successful;

    if (result.success)
    {
        result.result_code = DSHOT_COMMAND_SUCCESS;
    }

    return result;
}

// Get telemetry data
dshot_result_t DShotRMT::getTelemetry()
{
    dshot_result_t result = {false, DSHOT_TELEMETRY_FAILED, NO_DSHOT_TELEMETRY, NO_DSHOT_TELEMETRY};

    if (!_is_bidirectional)
    {
        result.result_code = DSHOT_BIDIR_NOT_ENABLED;
        return result;
    }

    // Use stored magnet count if parameter is 0 (default)
    uint16_t final_magnet_count = _motor_magnet_count;

    // Check if the callback has set the flag for new data
    if (_telemetry_ready_flag_atomic)
    {
        _telemetry_ready_flag_atomic = false; // Reset the flag
        uint16_t erpm = _last_erpm_atomic;    // Read the atomic variable

        if (erpm != DSHOT_NULL_PACKET && final_magnet_count >= MAGNETS_PER_POLE_PAIR)
        {
            // Calculate motor RPM from eRPM and magnet count
            uint8_t pole_pairs = final_magnet_count / MAGNETS_PER_POLE_PAIR;
            uint32_t motor_rpm = (erpm / pole_pairs);

            result.success = true;
            result.erpm = erpm;
            result.motor_rpm = motor_rpm;
            result.result_code = DSHOT_TELEMETRY_SUCCESS;
        }
    }

    return result;
}

// Reverse motor direction directly
dshot_result_t DShotRMT::setMotorSpinDirection(bool reversed)
{
    // Use command as a yes / no switch
    dshotCommands_e command = reversed ? dshotCommands_e::DSHOT_CMD_SPIN_DIRECTION_REVERSED : dshotCommands_e::DSHOT_CMD_SPIN_DIRECTION_NORMAL;

    return sendCommand(command, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_US);
}

dshot_result_t DShotRMT::sendCustomCommand(uint16_t command_value, uint16_t repeat_count, uint16_t delay_us)
{
    // Validate the integer command value before casting
    if (command_value < DSHOT_CMD_MOTOR_STOP || command_value > DSHOT_CMD_MAX_VALUE)
    {
        return {false, DSHOT_COMMAND_NOT_VALID};
    }

    dshot_result_t result = {false, DSHOT_UNKNOWN, NO_DSHOT_TELEMETRY, NO_DSHOT_TELEMETRY};

    bool all_successful = true;

    // Send command multiple times with delay
    for (uint16_t i = 0; i < repeat_count; i++)
    {
        dshot_result_t single_result = _sendRawDshotFrame(command_value);

        if (!single_result.success)
        {
            all_successful = false;
            result.result_code = single_result.result_code;
            break;
        }

        // Add delay between repetitions (except for last repetition)
        if (i < repeat_count - 1)
        {
            delayMicroseconds(delay_us);
        }
    }

    result.success = all_successful;

    if (result.success)
    {
        result.result_code = DSHOT_COMMAND_SUCCESS;
    }

    return result;
}

// Use with caution
dshot_result_t DShotRMT::saveESCSettings()
{
    return sendCommand(dshotCommands_e::DSHOT_CMD_SAVE_SETTINGS, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_US);
}

// Simple check
bool DShotRMT::_isValidCommand(dshotCommands_e command) const
{
    return (command >= dshotCommands_e::DSHOT_CMD_MOTOR_STOP && command <= DSHOT_CMD_MAX);
}

dshot_result_t DShotRMT::_sendRawDshotFrame(uint16_t value)
{
    _packet = _buildDShotPacket(value);
    return _sendPacket(_packet);
}

// Executes a single DShot command by building and sending a DShot frame.
dshot_result_t DShotRMT::_executeCommand(dshotCommands_e command)
{
    uint64_t start_time = esp_timer_get_time();

    _packet = _buildDShotPacket(static_cast<uint16_t>(command));
    dshot_result_t result = _sendPacket(_packet);

    uint64_t end_time = esp_timer_get_time();
    _last_command_timestamp = end_time;

    return result;
}

// Private Packet Management Functions
dshot_packet_t DShotRMT::_buildDShotPacket(const uint16_t &value) const
{
    dshot_packet_t packet = {};

    packet.throttle_value = value & DSHOT_THROTTLE_MAX;
    packet.telemetric_request = _is_bidirectional ? 1 : 0;

    // The data for CRC calculation includes the 11-bit value and the 1-bit telemetry flag
    uint16_t data_for_crc = (packet.throttle_value << 1) | packet.telemetric_request;
    packet.checksum = _calculateCRC(data_for_crc);

    return packet;
}

uint16_t DShotRMT::_buildDShotFrameValue(const dshot_packet_t &packet) const
{
    // Combine throttle, telemetry bit, and CRC into a single 16-bit frame
    uint16_t data_and_telemetry = (packet.throttle_value << 1) | packet.telemetric_request;
    return (data_and_telemetry << 4) | packet.checksum;
}

uint16_t DShotRMT::_calculateCRC(const uint16_t &data) const
{
    // Standard DShot CRC calculation using XOR
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & DSHOT_CRC_MASK;

    // For bidirectional DShot, the CRC is inverted
    if (_is_bidirectional)
    {
        crc = (~crc) & DSHOT_CRC_MASK;
    }
    return crc;
}

void DShotRMT::_preCalculateRMTTicks()
{
    // Pre-calculate all timing values in RMT ticks to save CPU cycles later.
    _rmt_ticks.bit_length_ticks = static_cast<uint16_t>(_dshot_timing.bit_length_us * RMT_TICKS_PER_US);
    _rmt_ticks.t1h_ticks = static_cast<uint16_t>(_dshot_timing.t1h_lenght_us * RMT_TICKS_PER_US);
    _rmt_ticks.t0h_ticks = _rmt_ticks.t1h_ticks >> 1; // High time for a '0' bit is half of a '1' bit.
    _rmt_ticks.t1l_ticks = _rmt_ticks.bit_length_ticks - _rmt_ticks.t1h_ticks;
    _rmt_ticks.t0l_ticks = _rmt_ticks.bit_length_ticks - _rmt_ticks.t0h_ticks;

    // Calculate the minimum time required between frames.
    // Pause between frames is frame time in us, some padding and about 30 us is added by hardware.
    _frame_timer_us = (static_cast<uint64_t>(_dshot_timing.bit_length_us * DSHOT_BITS_PER_FRAME) << 1) + DSHOT_PADDING_US;

    // For bidirectional, double up.
    if (_is_bidirectional)
    {
        _frame_timer_us = (_frame_timer_us << 1);
    }
}

// Private Frame Processing Functions
dshot_result_t DShotRMT::_sendPacket(const dshot_packet_t &packet)
{
    // Ensure enough time has passed since the last transmission
    if (!_isFrameIntervalElapsed())
    {
        return {true, DSHOT_NONE};
    }

    if (_is_bidirectional)
    {
        // Start the receiver to wait for incoming telemetry data
        rmt_symbol_word_t rx_symbols[GCR_BITS_PER_FRAME];
        size_t rx_size_bytes = GCR_BITS_PER_FRAME * sizeof(rmt_symbol_word_t);

        rmt_receive_config_t rmt_rx_config = {
            .signal_range_min_ns = DSHOT_PULSE_MIN_NS,
            .signal_range_max_ns = DSHOT_PULSE_MAX_NS,
        };

        if (rmt_receive(_rmt_rx_channel, rx_symbols, rx_size_bytes, &rmt_rx_config) != DSHOT_OK)
        {
            return {false, DSHOT_RECEIVER_FAILED};
        }
    }

    // Now let's prepare the actual frame
    _encoded_frame_value = _buildDShotFrameValue(packet);

    // Byte-swap the 16-bit value for correct transmission order (ESP32 is little-endian, DShot is MSB first)
    uint16_t swapped_value = __builtin_bswap16(_encoded_frame_value);

    // The DShot frame is 16 bits, which is 2 bytes
    size_t tx_size_bytes = sizeof(swapped_value);

    rmt_transmit_config_t tx_config = { .loop_count = 0 }; // No automatic loops - real-time calculation

    // In bidirectional mode, the RMT RX channel must be disabled before transmitting.
    // This is to prevent the receiver from picking up the transmitted signal, which would cause a loopback issue.
    // The ESP32's RMT peripheral TX and RX channels on the same pin are not fully independent.
    if (_is_bidirectional)
    {
        // Disable RMT RX for sending
        if (rmt_disable(_rmt_rx_channel) != DSHOT_OK)
        {
            return {false, DSHOT_RECEIVER_FAILED};
        }
    }

    if (rmt_transmit(_rmt_tx_channel, _dshot_encoder, &swapped_value, tx_size_bytes, &tx_config) != DSHOT_OK)
    {
        return {false, DSHOT_TRANSMISSION_FAILED};
    }

    // Re-enable RMT RX
    if (_is_bidirectional)
    {
        if (rmt_enable(_rmt_rx_channel) != DSHOT_OK)
        {
            return {false, DSHOT_RECEIVER_FAILED};
        }
    }

    _recordFrameTransmissionTime(); // Reset the timer for the next frame

    return {true, DSHOT_TRANSMISSION_SUCCESS};
}

// This function needs to be fast, as it generates the RMT symbols just before sending
// Placed in IRAM for high performance, as it's called from an ISR context.
uint16_t IRAM_ATTR DShotRMT::_decodeDShotFrame(const rmt_symbol_word_t *symbols) const
{
    uint32_t gcr_value = 0;

    // Decode RMT symbols into a 21-bit GCR (Group Code Recording) value.
    // The ESC sends back a signal where the duration determines the bit value.
    for (size_t i = 0; i < GCR_BITS_PER_FRAME; ++i)
    {
        bool bit_is_one = symbols[i].duration0 > symbols[i].duration1;
        gcr_value = (gcr_value << 1) | bit_is_one;
    }

    // Perform GCR decoding (GCR = Value ^ (Value >> 1)).
    uint32_t decoded_frame = gcr_value ^ (gcr_value >> 1);

    // Extract the 16-bit DShot frame from the decoded data.
    uint16_t data_and_crc = (decoded_frame & DSHOT_FULL_PACKET);

    // Extract data and CRC from the 16-bit frame.
    uint16_t received_data = data_and_crc >> DSHOT_CRC_BIT_SHIFT;
    uint16_t received_crc = data_and_crc & DSHOT_CRC_MASK;

    // A valid response must have the telemetry request bit set to 1. This is a sanity check.
    if (!((received_data >> DSHOT_TELEMETRY_BIT_POSITION) & 1))
    {
        return DSHOT_NULL_PACKET;
    }

    // Calculate and validate CRC.
    uint16_t calculated_crc = _calculateCRC(received_data);
    if (received_crc != calculated_crc)
    {
        return DSHOT_NULL_PACKET;
    }

    // Return the eRPM value (first 11 bits).
    return received_data & DSHOT_THROTTLE_MAX;
}

// Timing Control Functions
bool IRAM_ATTR DShotRMT::_isFrameIntervalElapsed() const
{
    // Check if the minimum interval between frames has passed
    uint64_t current_time = esp_timer_get_time();
    uint64_t elapsed = current_time - _last_transmission_time_us;
    return elapsed >= _frame_timer_us;
}

void DShotRMT::_recordFrameTransmissionTime()
{
    // Record the time of the current transmission
    _last_transmission_time_us = esp_timer_get_time();
}

// Static Callback Functions
// This function is called by the RMT driver's ISR when a frame is received
bool IRAM_ATTR DShotRMT::_on_rx_done(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    DShotRMT *instance = static_cast<DShotRMT *>(user_data);

    if (edata && edata->num_symbols == GCR_BITS_PER_FRAME)
    {
        uint16_t erpm = instance->_decodeDShotFrame(edata->received_symbols);

        if (erpm != DSHOT_NULL_PACKET)
        {
            // Atomically store the new eRPM value and set the flag
            instance->_last_erpm_atomic.store(erpm);
            instance->_telemetry_ready_flag_atomic.store(true);
        }
    }

    return false;
}

void DShotRMT::_cleanupRmtResources()
{
    if (_rmt_tx_channel)
    {
        rmt_disable(_rmt_tx_channel);
        rmt_del_channel(_rmt_tx_channel);
        _rmt_tx_channel = nullptr;
    }

    if (_rmt_rx_channel)
    {
        rmt_disable(_rmt_rx_channel);
        rmt_del_channel(_rmt_rx_channel);
        _rmt_rx_channel = nullptr;
    }

    if (_dshot_encoder)
    {
        rmt_del_encoder(_dshot_encoder);
        _dshot_encoder = nullptr;
    }
}
