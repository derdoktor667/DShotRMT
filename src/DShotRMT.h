/**
 * @file DShotRMT.h
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#pragma once

#include <atomic>
#include <Arduino.h>

#include <driver/gpio.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>

#include "dshot_definitions.h"
#include "dshot_messages.h"

// DShotRMT Library Version
static constexpr uint8_t DSHOTRMT_MAJOR_VERSION = 0;
static constexpr uint8_t DSHOTRMT_MINOR_VERSION = 9;
static constexpr uint8_t DSHOTRMT_PATCH_VERSION = 6;

// DShot Protocol Constants
static constexpr auto DSHOT_THROTTLE_FAILSAFE = 0;

// DShotRMT class for generating DShot signals and receiving telemetry.
class DShotRMT
{
public:
    DShotRMT(gpio_num_t gpio, dshot_mode_t mode = DSHOT300, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);
    ~DShotRMT();

    dshot_result_t begin();

    dshot_result_t sendThrottle(uint16_t throttle);
    dshot_result_t sendThrottlePercent(float percent);

    dshot_result_t sendCommand(uint16_t command_value);
    dshot_result_t sendCommand(dshotCommands_e command);
    dshot_result_t sendCommand(dshotCommands_e command, uint16_t repeat_count, uint16_t delay_us);
    dshot_result_t sendCustomCommand(uint16_t command_value, uint16_t repeat_count, uint16_t delay_us);

    dshot_result_t getTelemetry();
    dshot_result_t setMotorSpinDirection(bool reversed);
    dshot_result_t saveESCSettings();

    // Getters
    dshot_mode_t getMode() const { return _mode; }
    bool isBidirectional() const { return _is_bidirectional; }
    uint16_t getThrottleValue() const { return _last_throttle; }
    uint16_t getEncodedFrameValue() const { return _encoded_frame_value; }

    // Utility functions
    void printDShotResult(dshot_result_t &result, Stream &output = Serial) const;
    void printDShotInfo(Stream &output = Serial);
    static void printCpuInfo(Stream &output = Serial);

private:
    dshot_result_t _sendRawDshotFrame(uint16_t value);
    dshot_result_t _sendPacket(const dshot_packet_t &packet);
    dshot_result_t _sendRepeatedCommand(uint16_t value, uint16_t repeat_count, uint16_t delay_us);

    void _preCalculateTimings();
    void _cleanupRmtResources();
    bool _isValidCommand(dshotCommands_e command) const;
    bool _isFrameIntervalElapsed() const;
    void _recordFrameTransmissionTime();

    dshot_packet_t _buildDShotPacket(const uint16_t &value) const;
    uint16_t _buildDShotFrameValue(const dshot_packet_t &packet) const;
    uint16_t _calculateCRC(const uint16_t &data) const;

    uint16_t _decodeDShotFrame(const rmt_symbol_word_t *symbols) const;
    void _processFullTelemetryFrame(const rmt_symbol_word_t *symbols, size_t num_symbols);
    uint8_t _calculateTelemetryCRC(const uint8_t *data, size_t len) const;
    void _extractTelemetryData(const uint8_t *raw_telemetry_bytes, dshot_telemetry_data_t &telemetry_data) const;
    uint16_t _calculateMotorRpm(uint16_t erpm) const;
    uint8_t _decodeGcr5bTo4b(uint8_t gcr_5bit_value) const;
    size_t _recoverBits(const rmt_symbol_word_t *symbols, size_t num_symbols, uint8_t *out_bits, size_t max_bits) const;

    dshot_result_t _disableRmtRxChannel();
    dshot_result_t _enableRmtRxChannel();

    // Configuration
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _is_bidirectional;
    uint16_t _motor_magnet_count;

    // RMT Hardware Handles
    rmt_channel_handle_t _rmt_tx_channel = nullptr;
    rmt_channel_handle_t _rmt_rx_channel = nullptr;
    rmt_encoder_handle_t _dshot_encoder = nullptr;

    uint32_t _pulse_min_ns = 0;
    uint32_t _pulse_max_ns = 0;
    uint32_t _bit_length_ticks = 0;

    // Timing and State
    uint64_t _last_transmission_time_us = 0;
    uint64_t _frame_timer_us = 0;
    float _percent_to_throttle_ratio = 0.0f;
    uint16_t _last_throttle = 0;
    dshot_packet_t _packet;
    uint16_t _encoded_frame_value = 0;

    // Telemetry State
    std::atomic<uint16_t> _last_erpm_atomic = 0;
    std::atomic<bool> _telemetry_ready_flag_atomic = false;
    std::atomic<dshot_telemetry_data_t> _last_telemetry_data_atomic = {};
    std::atomic<bool> _full_telemetry_ready_flag_atomic = false;

    // RMT Callback
    rmt_rx_event_callbacks_t _rx_event_callbacks = {
        .on_recv_done = _on_rx_done,
    };

    static bool _on_rx_done(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data);
};
