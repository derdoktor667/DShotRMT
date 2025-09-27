/**
 * @file DShotRMT.h
 * @brief Optimized DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-09-18
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <dshot_definitions.h>
#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <atomic>

// Main class for DShot signal generation and reception.
// This class provides an interface to generate DShot signals for Electronic Speed Controllers (ESCs)
// and to receive telemetry data using the ESP32's RMT peripheral.
class DShotRMT
{
public:
    // Constructor for DShotRMT with GPIO number.
    explicit DShotRMT(gpio_num_t gpio = GPIO_NUM_16, dshot_mode_t mode = dshot_mode_t::DSHOT300, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    // Constructor for DShotRMT with Arduino pin number.
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    // Destructor for DShotRMT.
    // Cleans up RMT channels and encoder resources.
    ~DShotRMT();

    // Public Core Functions
    // Initializes the DShot RMT channels and encoder.
    dshot_result_t begin();

    // Sends a DShot throttle value to the ESC.
    dshot_result_t sendThrottle(uint16_t throttle);

    // Sends a DShot throttle value as a percentage to the ESC.
    dshot_result_t sendThrottlePercent(float percent);

    // Sends a single DShot command to the ESC.
    dshot_result_t sendCommand(uint16_t command);

    // Sends a DShot command multiple times with a delay between repetitions. This is a blocking function.
    dshot_result_t sendCommand(dshotCommands_e dshot_command, uint16_t repeat_count = DEFAULT_CMD_REPEAT_COUNT, uint16_t delay_us = DEFAULT_CMD_DELAY_US);

    // Retrieves telemetry data from the ESC.
    dshot_result_t getTelemetry(uint16_t magnet_count = 0);

    // Sends a command to the ESC to request ESC information.
    dshot_result_t getESCInfo();

    // Sets the motor spin direction.
    dshot_result_t setMotorSpinDirection(bool reversed);

    // Sends a command to the ESC to save its current settings.
    // Use with caution as this writes to ESC's non-volatile memory.
    dshot_result_t saveESCSettings();

    // Public Utility & Info Functions
    // Prints detailed DShot signal information for a given DShotRMT instance.
    static void printDShotInfo(const DShotRMT &dshot_rmt, Stream &output = Serial);

    // Prints detailed CPU information.
    static void printCpuInfo(Stream &output = Serial);

    // Sets the motor magnet count for RPM calculation.
    void setMotorMagnetCount(uint16_t magnet_count);

    // Gets the current DShot mode.
    dshot_mode_t getMode() const { return _mode; }

    // Checks if bidirectional DShot is enabled.
    bool isBidirectional() const { return _is_bidirectional; }

    // Gets the last encoded DShot frame value.
    uint16_t getEncodedFrameValue() const { return _encoded_frame_value; }

    // Gets the last transmitted throttle value.
    uint16_t getThrottleValue() const { return _packet.throttle_value; }

    // Deprecated Methods
    // Deprecated. Use sendThrottle() instead.
    [[deprecated("Use sendThrottle() instead")]]
    bool setThrottle(uint16_t throttle)
    {
        auto result = sendThrottle(throttle);
        return result.success;
    }

    // Deprecated. Use sendCommand() instead.
    [[deprecated("Use sendCommand() instead")]]
    bool sendDShotCommand(uint16_t command)
    {
        auto result = sendCommand(command);
        return result.success;
    }

    // Deprecated. Use getTelemetry() instead.
    [[deprecated("Use getTelemetry() instead")]]
    uint32_t getMotorRPM(uint8_t magnet_count)
    {
        auto result = getTelemetry(magnet_count);
        return result.motor_rpm;
    }

private:
    // --- UTILITY METHODS ---
    bool _isValidCommand(dshotCommands_e command);
    dshot_result_t _executeCommand(dshotCommands_e command);

    // Core Configuration Variables
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _is_bidirectional;
    uint16_t _motor_magnet_count;
    const dshot_timing_us_t &_dshot_timing;
    uint64_t _frame_timer_us;

    // Timing & Packet Variables
    rmt_ticks_t _rmt_ticks;
    uint16_t _last_throttle;
    uint64_t _last_transmission_time_us;
    uint64_t _last_command_timestamp;
    uint16_t _encoded_frame_value;
    dshot_packet_t _packet;
    uint16_t _pulse_level; // DShot protocol: Signal is idle-low, so pulses start by going HIGH.
    uint16_t _idle_level;  // DShot protocol: Signal returns to LOW after the high pulse.

    // RMT Hardware Handles
    rmt_channel_handle_t _rmt_tx_channel;
    rmt_channel_handle_t _rmt_rx_channel;
    rmt_encoder_handle_t _dshot_encoder;

    // RMT Configuration Structures
    rmt_tx_channel_config_t _tx_channel_config;
    rmt_rx_channel_config_t _rx_channel_config;
    rmt_transmit_config_t _rmt_tx_config;
    rmt_receive_config_t _rmt_rx_config;

    // Bidirectional / Telemetry Variables
    rmt_rx_event_callbacks_t _rx_event_callbacks;
    std::atomic<uint16_t> _last_erpm_atomic;
    std::atomic<bool> _telemetry_ready_flag_atomic;

    // Private Initialization Functions
    dshot_result_t _initTXChannel();
    dshot_result_t _initRXChannel();
    dshot_result_t _initDShotEncoder();

    // Private Packet Management Functions
    dshot_packet_t _buildDShotPacket(const uint16_t &value);
    uint16_t _buildDShotFrameValue(const dshot_packet_t &packet);
    uint16_t _calculateCRC(const uint16_t &data);
    void _preCalculateRMTTicks();

    // Private Frame Processing Functions
    dshot_result_t _sendDShotFrame(const dshot_packet_t &packet);
    uint16_t _decodeDShotFrame(const rmt_symbol_word_t *symbols);

    // Private Timing Control Functions
    bool _isFrameIntervalElapsed();
    void _recordFrameTransmissionTime();

    // Static Callback Functions
    static bool _on_rx_done(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data);
};
