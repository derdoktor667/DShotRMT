/**
 * @file DShotRMT.cpp
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include "DShotRMT.h"
#include "dshot_init.h"
#include <cstring>
#include <algorithm>
#include <initializer_list>

DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count)
    : _gpio(gpio),
      _mode(mode),
      _is_bidirectional(is_bidirectional),
      _motor_magnet_count(magnet_count)
{
    _preCalculateTimings();
    _percent_to_throttle_ratio = (static_cast<float>(DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)) / DSHOT_PERCENT_MAX;
}

DShotRMT::DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count)
    : DShotRMT(static_cast<gpio_num_t>(pin_nr), mode, is_bidirectional, magnet_count)
{
}

DShotRMT::~DShotRMT()
{
    _cleanupRmtResources();
}

dshot_result_t DShotRMT::begin()
{
    const dshot_timing_us_t dshot_timing = DSHOT_TIMING_US[_mode];
    rmt_ticks_t rmt_ticks;
    rmt_ticks.bit_length_ticks = static_cast<uint16_t>(dshot_timing.bit_length_us * RMT_TICKS_PER_US);
    _bit_length_ticks = rmt_ticks.bit_length_ticks;
    rmt_ticks.t1h_ticks = static_cast<uint16_t>(dshot_timing.t1h_lenght_us * RMT_TICKS_PER_US);
    rmt_ticks.t0h_ticks = rmt_ticks.t1h_ticks / 2;
    rmt_ticks.t1l_ticks = rmt_ticks.bit_length_ticks - rmt_ticks.t1h_ticks;
    rmt_ticks.t0l_ticks = rmt_ticks.bit_length_ticks - rmt_ticks.t0h_ticks;
    dshot_result_t result = _init_rmt_tx_channel(_gpio, &_rmt_tx_channel, _is_bidirectional);

    if (!result.success)
    {
        _cleanupRmtResources();
        return result;
    }

    if (_is_bidirectional)
    {
        result = _init_rmt_rx_channel(_gpio, &_rmt_rx_channel, &_rx_event_callbacks, this);

        if (!result.success)
        {
            _cleanupRmtResources();
            return result;
        }
    }

    result = _init_dshot_encoder(&_dshot_encoder, rmt_ticks);

    if (!result.success)
    {
        _cleanupRmtResources();
        return result;
    }

    return dshot_result_t::create_success(DSHOT_INIT_SUCCESS);
}

dshot_result_t DShotRMT::sendThrottle(uint16_t throttle)
{
    if (throttle == 0)
    {
        _last_throttle = 0;
        return sendCommand(DSHOT_CMD_MOTOR_STOP);
    }

    _last_throttle = constrain(throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    _packet = _buildDShotPacket(_last_throttle);

    return _sendPacket(_packet);
}

dshot_result_t DShotRMT::sendThrottlePercent(float percent)
{
    if (percent < DSHOT_PERCENT_MIN || percent > DSHOT_PERCENT_MAX)
    {
        return dshot_result_t::create_error(DSHOT_PERCENT_NOT_IN_RANGE);
    }

    uint16_t throttle = static_cast<uint16_t>(DSHOT_THROTTLE_MIN + _percent_to_throttle_ratio * percent);

    return sendThrottle(throttle);
}

dshot_result_t DShotRMT::sendCommand(uint16_t command_value)
{
    if (command_value > DSHOT_CMD_MAX)
    {
        return dshot_result_t::create_error(DSHOT_COMMAND_NOT_VALID);
    }

    return sendCommand(static_cast<dshotCommands_e>(command_value));
}

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
        break;
    }

    return sendCommand(command, repeat_count, delay_us);
}

dshot_result_t DShotRMT::sendCommand(dshotCommands_e command, uint16_t repeat_count, uint16_t delay_us)
{
    if (!_isValidCommand(command))
    {

        return dshot_result_t::create_error(DSHOT_INVALID_COMMAND);
    }

    return _sendRepeatedCommand(static_cast<uint16_t>(command), repeat_count, delay_us);
}

dshot_result_t DShotRMT::getTelemetry()
{
    if (!_is_bidirectional)
    {
        return dshot_result_t::create_error(DSHOT_BIDIR_NOT_ENABLED);
    }

    if (_full_telemetry_ready_flag_atomic)
    {
        _full_telemetry_ready_flag_atomic = false;
        dshot_telemetry_data_t telemetry_data = _last_telemetry_data_atomic;
        uint16_t erpm = telemetry_data.rpm;
        uint16_t motor_rpm = _calculateMotorRpm(erpm);

        return dshot_result_t::create_success(DSHOT_TELEMETRY_DATA_AVAILABLE, erpm, motor_rpm, telemetry_data, true);
    }

    if (_telemetry_ready_flag_atomic)
    {
        _telemetry_ready_flag_atomic = false;
        uint16_t erpm = _last_erpm_atomic;

        if (erpm != DSHOT_NULL_PACKET)
        {
            uint16_t motor_rpm = _calculateMotorRpm(erpm);
            return dshot_result_t::create_success(DSHOT_TELEMETRY_SUCCESS, erpm, motor_rpm);
        }
    }

    return dshot_result_t::create_error(DSHOT_TELEMETRY_FAILED);
}

dshot_result_t DShotRMT::setMotorSpinDirection(bool reversed)
{
    dshotCommands_e command = reversed ? dshotCommands_e::DSHOT_CMD_SPIN_DIRECTION_REVERSED : dshotCommands_e::DSHOT_CMD_SPIN_DIRECTION_NORMAL;

    return sendCommand(command, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_US);
}

dshot_result_t DShotRMT::sendCustomCommand(uint16_t command_value, uint16_t repeat_count, uint16_t delay_us)
{
    if (command_value > DSHOT_CMD_MAX)
    {
        return dshot_result_t::create_error(DSHOT_COMMAND_NOT_VALID);
    }

    return _sendRepeatedCommand(command_value, repeat_count, delay_us);
}

dshot_result_t DShotRMT::saveESCSettings()
{
    return sendCommand(dshotCommands_e::DSHOT_CMD_SAVE_SETTINGS, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_US);
}

dshot_result_t DShotRMT::_sendRepeatedCommand(uint16_t value, uint16_t repeat_count, uint16_t delay_us)
{
    dshot_result_t last_result = dshot_result_t::create_success(DSHOT_COMMAND_SUCCESS);

    for (uint16_t i = 0; i < repeat_count; i++)
    {
        last_result = _sendRawDshotFrame(value);

        if (!last_result.success)
        {
            return last_result;
        }

        if (i < repeat_count - 1)
        {
            delayMicroseconds(delay_us);
        }
    }

    return last_result;
}

bool DShotRMT::_isValidCommand(dshotCommands_e command) const
{
    return (command >= dshotCommands_e::DSHOT_CMD_MOTOR_STOP && command <= DSHOT_CMD_MAX);
}

dshot_result_t DShotRMT::_sendRawDshotFrame(uint16_t value)
{
    _packet = _buildDShotPacket(value);

    return _sendPacket(_packet);
}

dshot_packet_t DShotRMT::_buildDShotPacket(const uint16_t &value) const
{
    dshot_packet_t packet = {};
    packet.throttle_value = value & DSHOT_THROTTLE_MAX;
    packet.telemetric_request = _is_bidirectional ? 1 : 0;
    uint16_t data_for_crc = (packet.throttle_value << 1) | packet.telemetric_request;
    packet.checksum = _calculateCRC(data_for_crc);

    return packet;
}

uint16_t DShotRMT::_buildDShotFrameValue(const dshot_packet_t &packet) const
{
    uint16_t data_and_telemetry = (packet.throttle_value << 1) | packet.telemetric_request;

    return (data_and_telemetry << 4) | packet.checksum;
}

uint16_t DShotRMT::_calculateCRC(const uint16_t &data) const
{
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & DSHOT_CRC_MASK;

    if (_is_bidirectional)
    {
        crc = (~crc) & DSHOT_CRC_MASK;
    }

    return crc;
}

uint8_t DShotRMT::_calculateTelemetryCRC(const uint8_t *data, size_t len) const
{
    uint8_t crc = 0;

    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ DSHOT_TELEMETRY_CRC_POLYNOMIAL;
            }

            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

void DShotRMT::_extractTelemetryData(const uint8_t *raw_telemetry_bytes, dshot_telemetry_data_t &telemetry_data) const
{
    memset(&telemetry_data, 0, sizeof(dshot_telemetry_data_t));
    telemetry_data.temperature = static_cast<int8_t>(raw_telemetry_bytes[0]);
    telemetry_data.voltage = (static_cast<uint16_t>(raw_telemetry_bytes[1]) << 8) | raw_telemetry_bytes[2];
    telemetry_data.current = (static_cast<uint16_t>(raw_telemetry_bytes[3]) << 8) | raw_telemetry_bytes[4];
    telemetry_data.consumption = (static_cast<uint16_t>(raw_telemetry_bytes[5]) << 8) | raw_telemetry_bytes[6];
    telemetry_data.rpm = (static_cast<uint16_t>(raw_telemetry_bytes[7]) << 8) | raw_telemetry_bytes[8];
}

uint16_t DShotRMT::_calculateMotorRpm(uint16_t erpm) const
{
    if (_motor_magnet_count >= MAGNETS_PER_POLE_PAIR)
    {
        uint8_t pole_pairs = _motor_magnet_count / MAGNETS_PER_POLE_PAIR;

        return erpm / pole_pairs;
    }

    return 0;
}

uint8_t DShotRMT::_decodeGcr5bTo4b(uint8_t gcr_5bit_value) const
{
    if (gcr_5bit_value < GCR_CODE_LOOKUP_TABLE_SIZE)
    {
        return GCR_DECODE_LOOKUP_TABLE[gcr_5bit_value];
    }

    return GCR_INVALID_NIBBLE;
}

dshot_result_t DShotRMT::_disableRmtRxChannel()
{
    if (_is_bidirectional)
    {
        if (rmt_disable(_rmt_rx_channel) != DSHOT_OK)
        {
            return dshot_result_t::create_error(DSHOT_RECEIVER_FAILED);
        }
    }

    return dshot_result_t::create_success(DSHOT_NONE);
}

dshot_result_t DShotRMT::_enableRmtRxChannel()
{
    if (_is_bidirectional)
    {
        if (rmt_enable(_rmt_rx_channel) != DSHOT_OK)
        {
            return dshot_result_t::create_error(DSHOT_RECEIVER_FAILED);
        }
    }

    return dshot_result_t::create_success(DSHOT_NONE);
}

void DShotRMT::_preCalculateTimings()
{
    const dshot_timing_us_t dshot_timing = DSHOT_TIMING_US[_mode];
    _frame_timer_us = (static_cast<uint64_t>(dshot_timing.bit_length_us * DSHOT_BITS_PER_FRAME) << 1) + DSHOT_PADDING_US;

    if (_is_bidirectional)
    {
        _frame_timer_us = (_frame_timer_us << 2);
        const double t1h_ns = dshot_timing.t1h_lenght_us * NANOSECONDS_PER_MICROSECOND;
        const double t1l_ns = (dshot_timing.bit_length_us - dshot_timing.t1h_lenght_us) * NANOSECONDS_PER_MICROSECOND;
        const double t0h_ns = t1h_ns / 2.0;
        const double t0l_ns = (dshot_timing.bit_length_us * NANOSECONDS_PER_MICROSECOND) - t0h_ns;
        const double shortest_pulse = std::min({t1h_ns, t1l_ns, t0h_ns, t0l_ns});
        const double longest_pulse = std::max({t1h_ns, t1l_ns, t0h_ns, t0l_ns});
        _pulse_min_ns = static_cast<uint32_t>(shortest_pulse * (1.0f - PULSE_TIMING_TOLERANCE_PERCENT));
        _pulse_max_ns = static_cast<uint32_t>(longest_pulse * (1.0f + PULSE_TIMING_TOLERANCE_PERCENT));
    }
}

dshot_result_t DShotRMT::_sendPacket(const dshot_packet_t &packet)
{
    if (!_isFrameIntervalElapsed())
    {
        return dshot_result_t::create_success(DSHOT_NONE);
    }

    _encoded_frame_value = _buildDShotFrameValue(packet);
    uint16_t swapped_value = __builtin_bswap16(_encoded_frame_value);
    rmt_transmit_config_t tx_config = {};
    dshot_result_t disable_result = _disableRmtRxChannel();

    if (!disable_result.success)
    {
        return disable_result;
    }

    if (rmt_transmit(_rmt_tx_channel, _dshot_encoder, &swapped_value, DSHOT_FRAME_SIZE_BYTES, &tx_config) != DSHOT_OK)
    {
        return dshot_result_t::create_error(DSHOT_TRANSMISSION_FAILED);
    }

    if (rmt_tx_wait_all_done(_rmt_tx_channel, DSHOT_WAIT_FOREVER) != DSHOT_OK)
    {
        return dshot_result_t::create_error(DSHOT_TRANSMISSION_FAILED);
    }

    dshot_result_t enable_result = _enableRmtRxChannel();
    if (!enable_result.success)
    {
        return enable_result;
    }

    if (_is_bidirectional)
    {
        static rmt_symbol_word_t rx_symbols[RMT_RX_BUFFER_SYMBOLS];
        rmt_receive_config_t rmt_rx_config = {};
        rmt_rx_config.signal_range_min_ns = _pulse_min_ns;
        rmt_rx_config.signal_range_max_ns = _pulse_max_ns;

        if (rmt_receive(_rmt_rx_channel, rx_symbols, sizeof(rx_symbols), &rmt_rx_config) != DSHOT_OK)
        {
            return dshot_result_t::create_error(DSHOT_RECEIVER_FAILED);
        }
    }

    _recordFrameTransmissionTime();

    return dshot_result_t::create_success(DSHOT_TRANSMISSION_SUCCESS);
}

size_t DShotRMT::_recoverBits(const rmt_symbol_word_t *symbols, size_t num_symbols, uint8_t *out_bits, size_t max_bits) const
{
    size_t bit_idx = 0;
    const uint32_t half_bit_ticks = _bit_length_ticks / 2;

    for (size_t i = 0; i < num_symbols; ++i)
    {
        const uint32_t durations[2] = {static_cast<uint32_t>(symbols[i].duration0), static_cast<uint32_t>(symbols[i].duration1)};
        const uint8_t levels[2] = {static_cast<uint8_t>(symbols[i].level0), static_cast<uint8_t>(symbols[i].level1)};

        for (int j = 0; j < 2; ++j)
        {
            if (durations[j] > 0)
            {
                int num_bits = (durations[j] + half_bit_ticks) / _bit_length_ticks;

                for (int b = 0; b < num_bits; ++b)
                {
                    if (bit_idx < max_bits)
                    {
                        out_bits[bit_idx++] = levels[j];
                    }
                }
            }
        }

        if (bit_idx >= max_bits)
            break;
    }

    return bit_idx;
}

uint16_t IRAM_ATTR DShotRMT::_decodeDShotFrame(const rmt_symbol_word_t *symbols) const
{
    uint8_t raw_bits[DSHOT_ERPM_FRAME_GCR_BITS];
    size_t recovered_count = _recoverBits(symbols, DSHOT_ERPM_FRAME_GCR_BITS, raw_bits, DSHOT_ERPM_FRAME_GCR_BITS);

    if (recovered_count < DSHOT_ERPM_FRAME_GCR_BITS)
    {
        return DSHOT_NULL_PACKET;
    }

    uint32_t raw_frame = 0;

    for (size_t i = 0; i < DSHOT_ERPM_FRAME_GCR_BITS; ++i)
    {
        raw_frame = (raw_frame << 1) | raw_bits[i];
    }

    uint32_t gcr_value = (raw_frame ^ (raw_frame >> 1)) & DSHOT_GCR_FRAME_MASK;
    uint16_t decoded_frame = 0;

    for (int i = 0; i < 4; ++i)
    {
        uint8_t gcr_nibble = (gcr_value >> (i * DSHOT_GCR_GROUP_SIZE)) & DSHOT_GCR_NIBBLE_MASK;
        uint8_t original_nibble = _decodeGcr5bTo4b(gcr_nibble);

        if (original_nibble == GCR_INVALID_NIBBLE)
        {
            return DSHOT_NULL_PACKET;
        }

        decoded_frame |= (original_nibble << (i * DSHOT_NIBBLE_SIZE));
    }

    uint16_t csum = decoded_frame;
    csum = csum ^ (csum >> 8);
    csum = csum ^ (csum >> 4);

    if ((csum & DSHOT_CRC_MASK) != DSHOT_GCR_CRC_VALID)
    {
        return DSHOT_NULL_PACKET;
    }

    uint16_t edt_value = decoded_frame >> DSHOT_CRC_BIT_SHIFT;

    if (edt_value == DSHOT_EDT_BUSY_VALUE)
    {
        return DSHOT_NULL_PACKET;
    }

    uint16_t exponent = (edt_value >> 9) & DSHOT_EDT_EXPONENT_MASK;
    uint16_t mantissa = edt_value & DSHOT_EDT_MANTISSA_MASK;
    uint32_t period_us = mantissa << exponent;

    if (period_us == 0)
    {
        return DSHOT_NULL_PACKET;
    }

    return DSHOT_MICROSECONDS_PER_MINUTE / period_us;
}

void IRAM_ATTR DShotRMT::_processFullTelemetryFrame(const rmt_symbol_word_t *symbols, size_t num_symbols)
{
    uint8_t raw_bits[DSHOT_TELEMETRY_FULL_GCR_BITS];
    size_t recovered_count = _recoverBits(symbols, num_symbols, raw_bits, DSHOT_TELEMETRY_FULL_GCR_BITS);

    if (recovered_count < DSHOT_TELEMETRY_FULL_GCR_BITS)
        return;

    uint8_t gcr_stream[DSHOT_TELEMETRY_FULL_GCR_BITS];
    uint8_t prev_level = 1;

    for (int i = 0; i < DSHOT_TELEMETRY_FULL_GCR_BITS; ++i)
    {
        gcr_stream[i] = raw_bits[i] ^ prev_level;
        prev_level = raw_bits[i];
    }

    uint8_t gcr_decoded_bytes[DSHOT_TELEMETRY_PAYLOAD_WITH_CRC_BYTES];
    memset(gcr_decoded_bytes, 0, sizeof(gcr_decoded_bytes));
    int byte_bit_idx = 0;

    for (size_t i = 0; i < DSHOT_TELEMETRY_FULL_GCR_BITS; i += DSHOT_GCR_GROUP_SIZE)
    {
        uint8_t gcr_group_5bits = 0;

        for (size_t j = 0; j < DSHOT_GCR_GROUP_SIZE; ++j)
        {
            gcr_group_5bits = (gcr_group_5bits << 1) | gcr_stream[i + j];
        }

        uint8_t decoded_nibble = _decodeGcr5bTo4b(gcr_group_5bits);

        if (decoded_nibble == GCR_INVALID_NIBBLE)
            return;

        for (int k = DSHOT_NIBBLE_SIZE - 1; k >= 0; --k)
        {
            if (byte_bit_idx < (DSHOT_TELEMETRY_FRAME_LENGTH_BITS + DSHOT_TELEMETRY_CRC_LENGTH_BITS))
            {
                size_t byte_idx = byte_bit_idx / 8;
                size_t bit_pos = byte_bit_idx % 8;
                if ((decoded_nibble >> k) & 1)
                {
                    gcr_decoded_bytes[byte_idx] |= (1 << (7 - bit_pos));
                }

                byte_bit_idx++;
            }
        }
    }

    uint8_t received_crc = gcr_decoded_bytes[DSHOT_TELEMETRY_FRAME_LENGTH_BYTES];
    uint8_t calculated_crc = _calculateTelemetryCRC(gcr_decoded_bytes, DSHOT_TELEMETRY_FRAME_LENGTH_BYTES);

    if (received_crc == calculated_crc)
    {
        dshot_telemetry_data_t telemetry_data;
        _extractTelemetryData(gcr_decoded_bytes, telemetry_data);
        _last_telemetry_data_atomic.store(telemetry_data);
        _full_telemetry_ready_flag_atomic.store(true);
    }
}

bool IRAM_ATTR DShotRMT::_on_rx_done(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    DShotRMT *instance = static_cast<DShotRMT *>(user_data);
    if (edata)
    {
        if (edata->num_symbols == DSHOT_TELEMETRY_FULL_GCR_BITS)
        {
            instance->_processFullTelemetryFrame(edata->received_symbols, edata->num_symbols);
        }

        else if (edata->num_symbols == DSHOT_ERPM_FRAME_GCR_BITS)
        {
            uint16_t erpm = instance->_decodeDShotFrame(edata->received_symbols);

            if (erpm != DSHOT_NULL_PACKET)
            {
                instance->_last_erpm_atomic.store(erpm);
                instance->_telemetry_ready_flag_atomic.store(true);
            }
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

bool IRAM_ATTR DShotRMT::_isFrameIntervalElapsed() const
{
    return (esp_timer_get_time() - _last_transmission_time_us) >= _frame_timer_us;
}

void DShotRMT::_recordFrameTransmissionTime()
{
    _last_transmission_time_us = esp_timer_get_time();
}

void DShotRMT::printDShotResult(dshot_result_t &result, Stream &output) const
{
    output.printf("Status: %s - %s", result.success ? "SUCCESS" : "FAILED", get_result_code_str(result.result_code));

    if (result.success && (result.erpm > 0 || result.motor_rpm > 0))
    {
        output.printf(" | eRPM: %u, Motor RPM: %u", result.erpm, result.motor_rpm);
    }

    output.println();
}

void DShotRMT::printDShotInfo(Stream &output)
{
    output.println("\n=== DShot Info ===");
    output.printf("Library Version: %d.%d.%d\n", DSHOTRMT_MAJOR_VERSION, DSHOTRMT_MINOR_VERSION, DSHOTRMT_PATCH_VERSION);
    output.printf("Mode: %s\n", get_dshot_mode_str(getMode()));
    output.printf("Bidirectional: %s\n", isBidirectional() ? "YES" : "NO");
    output.printf("Last Throttle: %u\n", getThrottleValue());
    output.print("Packet (binary): ");

    for (int i = DSHOT_BITS_PER_FRAME - 1; i >= 0; --i)
    {
        output.print((getEncodedFrameValue() >> i) & 1);
    }

    output.println();

    if (isBidirectional())
    {
        dshot_result_t telemetry_result = getTelemetry();
        output.print("Telemetry: ");

        if (telemetry_result.success)
        {
            output.printf("OK (%s)\n", get_result_code_str(telemetry_result.result_code));

            if (telemetry_result.erpm > 0 || telemetry_result.motor_rpm > 0)
            {
                output.printf("  eRPM: %u, Motor RPM: %u\n", telemetry_result.erpm, telemetry_result.motor_rpm);
            }

            if (telemetry_result.telemetry_available)
            {
                output.println("  --- Full Telemetry Details ---");
                output.printf("  Temp: %d C | Volt: %.2f V | Curr: %.2f A | Cons: %u mAh\n",
                              telemetry_result.telemetry_data.temperature,
                              (float)telemetry_result.telemetry_data.voltage / CONVERSION_FACTOR_MILLI_TO_UNITS,
                              (float)telemetry_result.telemetry_data.current / CONVERSION_FACTOR_MILLI_TO_UNITS,
                              telemetry_result.telemetry_data.consumption);
                output.printf("  Telemetry RPM: %u\n", telemetry_result.telemetry_data.rpm);
            }

            else
            {
                output.println("  (Full telemetry not yet available or CRC failed for full frame)");
            }
        }

        else
        {
            output.printf("FAILED (%s)\n", get_result_code_str(telemetry_result.result_code));
        }
    }

    else
    {
        output.println("Telemetry: Disabled (Bidirectional mode OFF)");
    }

    output.println("===========================\n");
}

void DShotRMT::printCpuInfo(Stream &output)
{
    output.println("\n ===  CPU Info  === ");
    output.printf("Chip Model: %s\n", ESP.getChipModel());
    output.printf("Chip Revision: %d\n", ESP.getChipRevision());
    output.printf("CPU Freq = %lu MHz\n", ESP.getCpuFreqMHz());
    output.printf("XTAL Freq = %lu Hz\n", getXtalFrequencyMhz());
    output.printf("APB Freq = %lu Hz\n", getApbFrequency());
}
