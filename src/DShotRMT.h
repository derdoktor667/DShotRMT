/**
 * @file DShotRMT.h
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#pragma once

#include <atomic>
#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>

#include "dshot_definitions.h"
#include "dshot_config.h"

// DShot Protocol Constants
static constexpr auto DSHOT_THROTTLE_FAILSAFE = 0;
static constexpr auto DSHOT_THROTTLE_MIN = 48;
static constexpr auto DSHOT_BITS_PER_FRAME = 16;
static constexpr auto DEFAULT_MOTOR_MAGNET_COUNT = 14;

//
class DShotRMT
{
public:
    // Constructor with GPIO number
    DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    // Constructor using pin number
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    // Destructor
    ~DShotRMT();

    // Initialize DShotRMT
    dshot_result_t begin();

    // Send throttle value
    dshot_result_t sendThrottle(uint16_t throttle);

    // Send throttle value as a percentage
    dshot_result_t sendThrottlePercent(float percent);

    // Sends a DShot command (0-47) to the ESC by accepting an integer value.
    dshot_result_t sendCommand(uint16_t command_value);

    // Sends a DShot command (0-47) to the ESC.
    dshot_result_t sendCommand(dshotCommands_e command);

    // Sends a DShot command (0-47) to the ESC with a specified repeat count and delay.
    dshot_result_t sendCommand(dshotCommands_e command, uint16_t repeat_count, uint16_t delay_us);

    // Get telemetry data
    dshot_result_t getTelemetry();

    // Reverse motor direction directly
    dshot_result_t setMotorSpinDirection(bool reversed);

    // Use with caution
    dshot_result_t saveESCSettings();

    // Getters for DShot info
    dshot_mode_t getMode() const { return _mode; }
    bool isBidirectional() const { return _is_bidirectional; }
    uint16_t getThrottleValue() const { return _last_throttle; }
    uint16_t getEncodedFrameValue() const { return _encoded_frame_value; }

private:
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _is_bidirectional;
    uint16_t _motor_magnet_count;
    dshot_timing_us_t _dshot_timing;
    rmt_channel_handle_t _rmt_tx_channel = nullptr;
    rmt_channel_handle_t _rmt_rx_channel = nullptr;
    rmt_encoder_handle_t _dshot_encoder = nullptr;
    rmt_ticks_t _rmt_ticks;
    uint16_t _pulse_level = 1; // Default to high
    uint16_t _idle_level = 0;  // Default to low
    uint64_t _last_transmission_time_us = 0;
    uint64_t _frame_timer_us = 0;
    uint16_t _last_throttle = 0;
    dshot_packet_t _packet;
    uint16_t _encoded_frame_value = 0;
    uint64_t _last_command_timestamp = 0;

    // Telemetry related
    std::atomic<uint16_t> _last_erpm_atomic = 0;
    std::atomic<bool> _telemetry_ready_flag_atomic = false;
    rmt_rx_event_callbacks_t _rx_event_callbacks = {
        .on_recv_done = _on_rx_done,
    };

    // Private helper functions
    bool _isValidCommand(dshotCommands_e command) const;
    dshot_result_t _executeCommand(dshotCommands_e command);
    dshot_packet_t _buildDShotPacket(const uint16_t &value) const;
    uint16_t _buildDShotFrameValue(const dshot_packet_t &packet) const;
    uint16_t _calculateCRC(const uint16_t &data) const;
    void _preCalculateRMTTicks();
    dshot_result_t _sendDShotFrame(const dshot_packet_t &packet);
    uint16_t IRAM_ATTR _decodeDShotFrame(const rmt_symbol_word_t *symbols) const;
    bool IRAM_ATTR _isFrameIntervalElapsed() const;
    void _recordFrameTransmissionTime();

    // Static Callback Functions
    static bool IRAM_ATTR _on_rx_done(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data);
};

#include "dshot_utils.h" // Workround for util functions