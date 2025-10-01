/**
 * @file DShotRMT.h
 * @brief Optimized DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-09-18
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <atomic>

#include "dshot_definitions.h"
#include <driver/rmt_encoder.h>

// DShotRMT Class Definition
class DShotRMT
{
public:
    // Constructor for DShotRMT with GPIO number.
    explicit DShotRMT(gpio_num_t gpio = GPIO_NUM_16, dshot_mode_t mode = DSHOT300, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    // Constructor for DShotRMT with Arduino pin number.
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    // Destructor
    ~DShotRMT();

    // Public Core Functions
    // Initializes the DShot RMT channels and encoder.
    dshot_result_t begin();

    // Sends a throttle value as a percentage (0.0-100.0) to the ESC.
    dshot_result_t sendThrottlePercent(float percent);

    // Sends a raw throttle value (48-2047) to the ESC. A value of 0 sends a motor stop command.
    dshot_result_t sendThrottle(uint16_t throttle);

    // Sends a DShot command (0-47) to the ESC.
    dshot_result_t sendCommand(dshotCommands_e command);

    // Retrieves telemetry data from the ESC.
    dshot_result_t getTelemetry();

    // Sets the motor spin direction. 'true' for reversed, 'false' for normal.
    dshot_result_t setMotorSpinDirection(bool reversed);

    // Sends a command to the ESC to save its current settings. Use with caution as this writes to ESC's non-volatile memory.
    dshot_result_t saveESCSettings();

    // Public Utility & Info Functions
    // Sets the motor magnet count for RPM calculation.
    void setMotorMagnetCount(uint16_t magnet_count);

    // Gets the current DShot mode.
    dshot_mode_t getMode() const { return _mode; }

    // Checks if bidirectional DShot is enabled.
    bool isBidirectional() const { return _is_bidirectional; }

    // Gets the encoded frame value.
    uint16_t getEncodedFrameValue() const { return _encoded_frame_value; }

    // Gets the last transmitted throttle value.
    uint16_t getThrottleValue() const { return _packet.throttle_value; }

    // Testing return "verbose" messages
    const char *getDShotMsg(dshot_result_t &result) const { return (_get_result_code_str(result.result_code)); }

    // Deprecated Methods
    // Deprecated. Use sendThrottle() instead.
    [[deprecated("Use sendThrottle() instead")]]
    dshot_result_t sendValue(uint16_t value) { return sendThrottle(value); }

    // Deprecated. Use sendCommand() instead.
    [[deprecated("Use sendCommand() instead")]]
    dshot_result_t sendCommand(uint16_t command) { return sendCommand(static_cast<dshotCommands_e>(command)); }

private:
    // --- UTILITY METHODS ---
    bool _isValidCommand(dshotCommands_e command) const;
    dshot_result_t _executeCommand(dshotCommands_e command);
    dshot_result_t _sendCommandInternal(dshotCommands_e dshot_command, uint16_t repeat_count, uint16_t delay_us);

    // Core Configuration Variables
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _is_bidirectional;
    uint16_t _motor_magnet_count;
    const dshot_timing_us_t &_dshot_timing;
    uint64_t _frame_timer_us = 0;

    // Timing & Packet Variables
    rmt_ticks_t _rmt_ticks{};
    uint16_t _last_throttle = dshotCommands_e::DSHOT_CMD_MOTOR_STOP;
    uint64_t _last_transmission_time_us = 0;
    uint64_t _last_command_timestamp = 0;
    uint16_t _encoded_frame_value = 0;
    dshot_packet_t _packet{};
    uint16_t _pulse_level = 1; // DShot protocol: Signal is idle-low, so pulses start by going HIGH.
    uint16_t _idle_level = 0;  // DShot protocol: Signal returns to LOW after the high pulse.

    // RMT Hardware Handles
    rmt_channel_handle_t _rmt_tx_channel = nullptr;
    rmt_channel_handle_t _rmt_rx_channel = nullptr;
    rmt_encoder_handle_t _dshot_encoder = nullptr;

    // RMT Configuration Structures
    rmt_tx_channel_config_t _tx_channel_config{};
    rmt_rx_channel_config_t _rx_channel_config{};
    rmt_transmit_config_t _rmt_tx_config{};
    rmt_receive_config_t _rmt_rx_config{};

    // Bidirectional / Telemetry Variables
    rmt_rx_event_callbacks_t _rx_event_callbacks{};
    std::atomic<uint16_t> _last_erpm_atomic{0};
    std::atomic<bool> _telemetry_ready_flag_atomic{false};

    // Private Initialization Functions
    dshot_result_t _initTXChannel();
    dshot_result_t _initRXChannel();
    dshot_result_t _initDShotEncoder();

    // Private Packet Management Functions
    dshot_packet_t _buildDShotPacket(const uint16_t &value) const;
    uint16_t _buildDShotFrameValue(const dshot_packet_t &packet) const;
    uint16_t _calculateCRC(const uint16_t &data) const;
    void _preCalculateRMTTicks();

    // Private Frame Processing Functions
    dshot_result_t _sendDShotFrame(const dshot_packet_t &packet);
    uint16_t _decodeDShotFrame(const rmt_symbol_word_t *symbols) const;

    // Private Timing Control Functions
    bool _isFrameIntervalElapsed() const;
    void _recordFrameTransmissionTime();

    // Static Callback Functions
    static bool _on_rx_done(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data);
};

// Helper to quick print DShot result codes
inline void printDShotResult(dshot_result_t &result, Stream &output = Serial)
{
    output.printf("Status: %s - %s", result.success ? "SUCCESS" : "FAILED", _get_result_code_str(result.result_code));

    // Print telemetry data if available
    if (result.success && (result.erpm > 0 || result.motor_rpm > 0))
    {
        output.printf(" | eRPM: %u, Motor RPM: %u", result.erpm, result.motor_rpm);
    }

    output.println();
}

// Helper to print DShot signal info
inline void printDShotInfo(const DShotRMT &dshot_rmt, Stream &output = Serial)
{
    output.println("\n === DShot Signal Info === ");

    uint16_t dshot_mode_val = 0;
    switch (dshot_rmt.getMode())
    {
    case DSHOT150:
        dshot_mode_val = 150;
        break;
    case DSHOT300:
        dshot_mode_val = 300;
        break;
    case DSHOT600:
        dshot_mode_val = 600;
        break;
    case DSHOT1200:
        dshot_mode_val = 1200;
        break;
    }

    output.printf("Current Mode: DSHOT%d\n", dshot_mode_val);
    output.printf("Bidirectional: %s\n", dshot_rmt.isBidirectional() ? "YES" : "NO");
    output.printf("Current Packet: ");

    for (int i = DSHOT_BITS_PER_FRAME - 1; i >= 0; --i)
    {
        output.print((dshot_rmt.getEncodedFrameValue() >> i) & 1);
    }

    output.printf("\nCurrent Value: %u\n", dshot_rmt.getThrottleValue());
}

// Helper to print CPU info
inline void printCpuInfo(Stream &output = Serial)
{
    output.println("\n ===  CPU Info  === ");
    output.printf("Chip Model: %s\n", ESP.getChipModel());
    output.printf("Chip Revision: %d\n", ESP.getChipRevision());
    output.printf("CPU Freq = %lu MHz\n", ESP.getCpuFreqMHz());
    output.printf("XTAL Freq = %lu MHz\n", getXtalFrequencyMhz());
    output.printf("APB Freq = %lu Hz\n", getApbFrequency());
}