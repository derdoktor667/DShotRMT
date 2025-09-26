/**
 * @file DShotRMT.cpp
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include <DShotRMT.h>

// Constructors & Destructor
// Constructor with GPIO number
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count)
    : _gpio(gpio),
      _mode(mode),
      _is_bidirectional(is_bidirectional),
      _motor_magnet_count(magnet_count),
      _dshot_timing(DSHOT_TIMING_US[static_cast<int>(mode)]),
      _frame_timer_us(0),
      _rmt_ticks{0},
      _last_throttle(static_cast<uint16_t>(dshotCommands_e::DSHOT_CMD_MOTOR_STOP)),
      _last_transmission_time_us(0),
      _last_command_timestamp(0),
      _encoded_frame_value(0),
      _packet{0},
      _pulse_level(1), // DShot standard: signal is idle-low, so pulses start by going HIGH
      _idle_level(0), // DShot standard: signal returns to LOW after the high pulse
      _rmt_tx_channel(nullptr),
      _rmt_rx_channel(nullptr),
      _dshot_encoder(nullptr),
      _tx_channel_config{},
      _rx_channel_config{},
      _rmt_tx_config{},
      _rmt_rx_config{},
      _rx_event_callbacks{},
      _last_erpm_atomic(0),
      _telemetry_ready_flag_atomic(false)
{
    // Pre-calculate timing and bit positions for performance
    _preCalculateRMTTicks();

    // Activate internal pullup resistor
    // gpio_set_pull_mode(_gpio, GPIO_PULLUP_ONLY);
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
    // Cleanup TX channel
    if (_rmt_tx_channel)
    {
        if (rmt_disable(_rmt_tx_channel) == DSHOT_OK)
        {
            rmt_del_channel(_rmt_tx_channel);
            _rmt_tx_channel = nullptr;
        }
    }

    // Cleanup RX channel
    if (_rmt_rx_channel)
    {
        if (rmt_disable(_rmt_rx_channel) == DSHOT_OK)
        {
            rmt_del_channel(_rmt_rx_channel);
            _rmt_rx_channel = nullptr;
        }
    }

    // Cleanup encoder
    if (_dshot_encoder)
    {
        rmt_del_encoder(_dshot_encoder);
        _dshot_encoder = nullptr;
    }
}

// Public Core Functions
// Initialize DShotRMT
dshot_result_t DShotRMT::begin()
{
    if (!_initTXChannel().success)
    {
        return {false, dshot_msg_code_t::DSHOT_ERROR_TX_INIT_FAILED};
    }

    if (_is_bidirectional)
    {
        if (!_initRXChannel().success)
        {
            // Cleanup previously allocated TX channel on failure
            rmt_disable(_rmt_tx_channel);
            rmt_del_channel(_rmt_tx_channel);
            _rmt_tx_channel = nullptr;
            return {false, dshot_msg_code_t::DSHOT_ERROR_RX_INIT_FAILED};
        }
    }

    if (!_initDShotEncoder().success)
    {
        // Cleanup previously allocated channels on failure
        rmt_disable(_rmt_tx_channel);
        rmt_del_channel(_rmt_tx_channel);
        _rmt_tx_channel = nullptr;

        if (_rmt_rx_channel)
        {
            rmt_disable(_rmt_rx_channel);
            rmt_del_channel(_rmt_rx_channel);
            _rmt_rx_channel = nullptr;
        }

        return {false, dshot_msg_code_t::DSHOT_ERROR_ENCODER_INIT_FAILED};
    }

    return {true, dshot_msg_code_t::DSHOT_ERROR_INIT_SUCCESS};
}

// Send throttle value
dshot_result_t DShotRMT::sendThrottle(uint16_t throttle)
{
    // A throttle value of 0 is a disarm command
    if (throttle == 0)
    {
        return sendCommand(static_cast<uint16_t>(dshotCommands_e::DSHOT_CMD_MOTOR_STOP));
    }

    // Constrain throttle to the valid DShot range
    _last_throttle = constrain(throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

    _packet = _buildDShotPacket(_last_throttle);
    return _sendDShotFrame(_packet);
}

// Send throttle value as a percentage
dshot_result_t DShotRMT::sendThrottlePercent(float percent)
{
    if (percent < 0.0f || percent > 100.0f)
    {
        return {false, dshot_msg_code_t::DSHOT_ERROR_PERCENT_NOT_IN_RANGE};
    }

    // Map percent to DShot throttle range
    uint16_t throttle = static_cast<uint16_t>(DSHOT_THROTTLE_MIN + ((DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN) / 100.0f) * percent);

    return sendThrottle(throttle);
}

// Send DShot command to ESC
dshot_result_t DShotRMT::sendCommand(uint16_t command)
{
    if (command > static_cast<uint16_t>(dshotCommands_e::DSHOT_CMD_MAX))
    {
        return {false, dshot_msg_code_t::DSHOT_ERROR_COMMAND_NOT_VALID};
    }

    _packet = _buildDShotPacket(command);
    return _sendDShotFrame(_packet);
}

// Send full DShot commands for setup etc
// This is a blocking function that uses delayMicroseconds for repetitions.
dshot_result_t DShotRMT::sendCommand(dshotCommands_e dshot_command, uint16_t repeat_count, uint16_t delay_us)
{
    dshot_result_t result = {false, dshot_msg_code_t::DSHOT_ERROR_UNKNOWN};

    if (!_isValidCommand(dshot_command))
    {
        result.error_code = dshot_msg_code_t::DSHOT_ERROR_INVALID_COMMAND;
        return result;
    }

    bool all_successful = true;

    // Send command multiple times with delay
    for (uint16_t i = 0; i < repeat_count; i++)
    {
        dshot_result_t single_result = _executeCommand(dshot_command);

        if (!single_result.success)
        {
            all_successful = false;
            result.error_code = single_result.error_code;
            break;
        }

        // Add delay between repetitions (except for last repetition)
        if (i < repeat_count - 1)
        {
            delayMicroseconds(delay_us);
        }
    }

    //
    result.success = all_successful;

    if (result.success)
    {
        result.error_code = dshot_msg_code_t::DSHOT_ERROR_COMMAND_SUCCESS;
    }

    return result;
}

// Get telemetry data
dshot_result_t DShotRMT::getTelemetry(uint16_t magnet_count)
{
    dshot_result_t result = {false, dshot_msg_code_t::DSHOT_ERROR_TELEMETRY_FAILED, NO_DSHOT_TELEMETRY, NO_DSHOT_TELEMETRY};

    if (!_is_bidirectional)
    {
        result.error_code = dshot_msg_code_t::DSHOT_ERROR_BIDIR_NOT_ENABLED;
        return result;
    }

    // Use stored magnet count if parameter is 0 (default)
    uint16_t final_magnet_count = (magnet_count == 0) ? _motor_magnet_count : magnet_count;

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
            result.error_code = dshot_msg_code_t::DSHOT_ERROR_TELEMETRY_SUCCESS;
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

dshot_result_t DShotRMT::getESCInfo()
{
    return sendCommand(static_cast<uint16_t>(dshotCommands_e::DSHOT_CMD_ESC_INFO));
}

// Use with caution
dshot_result_t DShotRMT::saveESCSettings()
{
    return sendCommand(dshotCommands_e::DSHOT_CMD_SAVE_SETTINGS, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_US);
}

// Simple check
bool DShotRMT::_isValidCommand(dshotCommands_e command)
{
    return (static_cast<uint16_t>(command) >= static_cast<uint16_t>(dshotCommands_e::DSHOT_CMD_MOTOR_STOP) && static_cast<uint16_t>(command) <= static_cast<uint16_t>(dshotCommands_e::DSHOT_CMD_MAX));
}

//
dshot_result_t DShotRMT::_executeCommand(dshotCommands_e command)
{
    uint64_t start_time = esp_timer_get_time();

    // Execute the command using the DShotRMT instance
    dshot_result_t result = sendCommand(static_cast<uint16_t>(command));

    uint64_t end_time = esp_timer_get_time();
    _last_command_timestamp = end_time;

    return result;
}

// Private Initialization Functions
dshot_result_t DShotRMT::_initTXChannel()
{
    _tx_channel_config.gpio_num = _gpio;
    _tx_channel_config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    _tx_channel_config.resolution_hz = DSHOT_RMT_RESOLUTION;
    _tx_channel_config.mem_block_symbols = RMT_BUFFER_SYMBOLS;
    _tx_channel_config.trans_queue_depth = RMT_QUEUE_DEPTH;

    _rmt_tx_config.loop_count = 0; // No automatic loops - real-time calculation
    _rmt_tx_config.flags.eot_level = _is_bidirectional ? 1 : 0;

    if (rmt_new_tx_channel(&_tx_channel_config, &_rmt_tx_channel) != DSHOT_OK)
    {
        return {false, dshot_msg_code_t::DSHOT_ERROR_TX_INIT_FAILED};
    }
    if (rmt_enable(_rmt_tx_channel) != DSHOT_OK)
    {
        return {false, dshot_msg_code_t::DSHOT_ERROR_TX_INIT_FAILED};
    }

    return {true, dshot_msg_code_t::DSHOT_ERROR_TX_INIT_SUCCESS};
}

dshot_result_t DShotRMT::_initRXChannel()
{
    // Double check if bidirectional mode is enabled
    if (!_is_bidirectional)
    {
        return {true, dshot_msg_code_t::DSHOT_ERROR_NONE};
    }

    _rx_channel_config.gpio_num = _gpio;
    _rx_channel_config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    _rx_channel_config.resolution_hz = DSHOT_RMT_RESOLUTION;
    _rx_channel_config.mem_block_symbols = RMT_BUFFER_SYMBOLS;

    // Filter for pulses that are within a reasonable range for DShot telemetry
    _rmt_rx_config.signal_range_min_ns = DSHOT_PULSE_MIN;
    _rmt_rx_config.signal_range_max_ns = DSHOT_PULSE_MAX;

    if (rmt_new_rx_channel(&_rx_channel_config, &_rmt_rx_channel) != DSHOT_OK)
    {
        return {false, dshot_msg_code_t::DSHOT_ERROR_RX_INIT_FAILED};
    }

    // Register the callback function that will be triggered when a frame is received
    _rx_event_callbacks.on_recv_done = _on_rx_done;
    if (rmt_rx_register_event_callbacks(_rmt_rx_channel, &_rx_event_callbacks, this) != DSHOT_OK)
    {
        return {false, dshot_msg_code_t::DSHOT_ERROR_CALLBACK_REGISTERING_FAILED};
    }

    if (rmt_enable(_rmt_rx_channel) != DSHOT_OK)
    {
        return {false, dshot_msg_code_t::DSHOT_ERROR_RX_INIT_FAILED};
    }

    // Start the receiver to wait for incoming telemetry data
    rmt_symbol_word_t rx_symbols[GCR_BITS_PER_FRAME];
    size_t rx_size_bytes = GCR_BITS_PER_FRAME * sizeof(rmt_symbol_word_t);
    if (rmt_receive(_rmt_rx_channel, rx_symbols, rx_size_bytes, &_rmt_rx_config) != DSHOT_OK)
    {
        return {false, dshot_msg_code_t::DSHOT_ERROR_RECEIVER_FAILED};
    }

    return {true, dshot_msg_code_t::DSHOT_ERROR_RX_INIT_SUCCESS};
}

dshot_result_t DShotRMT::_initDShotEncoder()
{
    rmt_bytes_encoder_config_t encoder_config = {
        .bit0 = {
            .duration0 = _rmt_ticks.t0h_ticks,
            .level0 = _pulse_level,
            .duration1 = _rmt_ticks.t0l_ticks,
            .level1 = _idle_level,
        },
        .bit1 = {
            .duration0 = _rmt_ticks.t1h_ticks,
            .level0 = _pulse_level,
            .duration1 = _rmt_ticks.t1l_ticks,
            .level1 = _idle_level,
        },
        .flags = {
            .msb_first = 1 // DShot is MSB first
        }
    };

    if (rmt_new_bytes_encoder(&encoder_config, &_dshot_encoder) != DSHOT_OK)
    {
        return {false, dshot_msg_code_t::DSHOT_ERROR_ENCODER_INIT_FAILED};
    }

    return {true, dshot_msg_code_t::DSHOT_ERROR_ENCODER_INIT_SUCCESS};
}

// Private Packet Management Functions
dshot_packet_t DShotRMT::_buildDShotPacket(const uint16_t &value)
{
    dshot_packet_t packet = {};

    packet.throttle_value = value & DSHOT_THROTTLE_MAX;
    packet.telemetric_request = _is_bidirectional ? 1 : 0;

    // The data for CRC calculation includes the 11-bit value and the 1-bit telemetry flag
    uint16_t data_for_crc = (packet.throttle_value << 1) | packet.telemetric_request;
    packet.checksum = _calculateCRC(data_for_crc);

    return packet;
}

uint16_t DShotRMT::_buildDShotFrameValue(const dshot_packet_t &packet)
{
    // Combine throttle, telemetry bit, and CRC into a single 16-bit frame
    uint16_t data_and_telemetry = (packet.throttle_value << 1) | packet.telemetric_request;
    return (data_and_telemetry << 4) | packet.checksum;
}

uint16_t DShotRMT::_calculateCRC(const uint16_t &data)
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
    // Pre-calculate all timing values in RMT ticks to save CPU cycles later
    _rmt_ticks.bit_length_ticks = static_cast<uint16_t>(_dshot_timing.bit_length_us * RMT_TICKS_PER_US);
    _rmt_ticks.t1h_ticks = static_cast<uint16_t>(_dshot_timing.t1h_lenght_us * RMT_TICKS_PER_US);
    _rmt_ticks.t0h_ticks = _rmt_ticks.t1h_ticks >> 1; // High time for a 1 is always double of 0
    _rmt_ticks.t1l_ticks = _rmt_ticks.bit_length_ticks - _rmt_ticks.t1h_ticks;
    _rmt_ticks.t0l_ticks = _rmt_ticks.bit_length_ticks - _rmt_ticks.t0h_ticks;

    // Calculate the minimum time required between frames
    // Pause between frames is frame time in us, some padding and about 30 us is added by hardware
    _frame_timer_us = (static_cast<uint64_t>(_dshot_timing.bit_length_us * DSHOT_BITS_PER_FRAME) << 1) + DSHOT_PADDING_US;

    // For bidirectional, double up
    if (_is_bidirectional)
    {
        _frame_timer_us = (_frame_timer_us << 1);
    }
}

// Private Frame Processing Functions
dshot_result_t DShotRMT::_sendDShotFrame(const dshot_packet_t &packet)
{
    // Ensure enough time has passed since the last transmission
    if (!_isFrameIntervalElapsed())
    {
        return {true, dshot_msg_code_t::DSHOT_ERROR_NONE};
    }

    _encoded_frame_value = _buildDShotFrameValue(packet);

    // Byte-swap the 16-bit value for correct transmission order (ESP32 is little-endian, DShot is MSB first)
    uint16_t swapped_value = __builtin_bswap16(_encoded_frame_value);

    // The DShot frame is 16 bits, which is 2 bytes
    size_t tx_size_bytes = sizeof(swapped_value);

    if (rmt_transmit(_rmt_tx_channel, _dshot_encoder, &swapped_value, tx_size_bytes, &_rmt_tx_config) != DSHOT_OK)
    {
        return {false, dshot_msg_code_t::DSHOT_ERROR_TRANSMISSION_FAILED};
    }

    _recordFrameTransmissionTime(); // Reset the timer for the next frame

    return {true, dshot_msg_code_t::DSHOT_ERROR_TRANSMISSION_SUCCESS};
}

// This function needs to be fast, as it generates the RMT symbols just before sending

// Placed in IRAM for high performance, as it's called from an ISR context
uint16_t IRAM_ATTR DShotRMT::_decodeDShotFrame(const rmt_symbol_word_t *symbols)
{
    uint32_t gcr_value = 0;

    // Step 1: Decode RMT symbols into a 21-bit GCR (Group Code Recording) value.
    // The ESC sends back a signal where the duration determines the bit value.
    for (size_t i = 0; i < GCR_BITS_PER_FRAME; ++i)
    {
        bool bit_is_one = symbols[i].duration0 > symbols[i].duration1;
        gcr_value = (gcr_value << 1) | bit_is_one;
    }

    // Step 2: Perform GCR decoding (GCR = Value ^ (Value >> 1))
    uint32_t decoded_frame = gcr_value ^ (gcr_value >> 1);

    // Step 3: Extract the 16-bit DShot frame from the decoded data
    uint16_t data_and_crc = (decoded_frame & DSHOT_FULL_PACKET);

    // Step 4: Extract data and CRC from the 16-bit frame
    uint16_t received_data = data_and_crc >> DSHOT_CRC_BIT_SHIFT;
    uint16_t received_crc = data_and_crc & DSHOT_CRC_MASK;

    // Step 5: A valid response must have the telemetry request bit set to 1. This is a sanity check.
    if (!((received_data >> DSHOT_TELEMETRY_BIT_POSITION) & 1))
    {
        return DSHOT_NULL_PACKET;
    }

    // Step 6: Calculate and validate CRC
    uint16_t calculated_crc = _calculateCRC(received_data);
    if (received_crc != calculated_crc)
    {
        return DSHOT_NULL_PACKET;
    }

    // Return the eRPM value (first 11 bits).
    return received_data & DSHOT_THROTTLE_MAX;
}

// Timing Control Functions
bool IRAM_ATTR DShotRMT::_isFrameIntervalElapsed()
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

// Public Static Utility Functions
void DShotRMT::printDShotInfo(const DShotRMT &dshot_rmt, Stream &output)
{
    output.println("\n === DShot Signal Info === ");
    output.printf("Current Mode: DSHOT%d\n", dshot_rmt.getMode() == dshot_mode_t::DSHOT150 ? 150 : dshot_rmt.getMode() == dshot_mode_t::DSHOT300 ? 300
                                                                                               : dshot_rmt.getMode() == dshot_mode_t::DSHOT600   ? 600
                                                                                               : dshot_rmt.getMode() == dshot_mode_t::DSHOT1200  ? 1200
                                                                                                                                                 : 0);
    output.printf("Bidirectional: %s\n", dshot_rmt.isBidirectional() ? "YES" : "NO");
    output.printf("Current Packet: ");

    for (int i = DSHOT_BITS_PER_FRAME - 1; i >= 0; --i)
    {
        output.print((dshot_rmt.getEncodedFrameValue() >> i) & 1);
    }

    output.printf("\nCurrent Value: %u\n", dshot_rmt.getThrottleValue());
}

void DShotRMT::printCpuInfo(Stream &output)
{
    output.println("\n ===  CPU Info  === ");
    output.printf("Chip Model: %s\n", ESP.getChipModel());
    output.printf("Chip Revision: %d\n", ESP.getChipRevision());
    output.printf("CPU Freq = %lu MHz\n", ESP.getCpuFreqMHz());
    output.printf("XTAL Freq = %lu MHz\n", getXtalFrequencyMhz());
    output.printf("APB Freq = %lu Hz\n", getApbFrequency());
}