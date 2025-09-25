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

/**
 * @brief DShotRMT Main Class for DShot signal generation and reception.
 *
 * This class provides an interface to generate DShot signals for Electronic Speed Controllers (ESCs)
 * and to receive telemetry data using the ESP32's RMT peripheral.
 */
class DShotRMT
{
public:
    /**
     * @brief Constructor for DShotRMT with GPIO number.
     * @param gpio The GPIO pin number to use for DShot communication.
     * @param mode The DShot mode (e.g., DSHOT150, DSHOT300, DSHOT600).
     * @param is_bidirectional True if bidirectional DShot is enabled, false otherwise.
     * @param magnet_count The number of magnets in the motor for RPM calculation.
     */
    explicit DShotRMT(gpio_num_t gpio = GPIO_NUM_16, dshot_mode_t mode = dshot_mode_t::DSHOT300, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    /**
     * @brief Constructor for DShotRMT with Arduino pin number.
     * @param pin_nr The Arduino pin number to use for DShot communication.
     * @param mode The DShot mode (e.g., DSHOT150, DSHOT300, DSHOT600).
     * @param is_bidirectional True if bidirectional DShot is enabled, false otherwise.
     * @param magnet_count The number of magnets in the motor for RPM calculation.
     */
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    /**
     * @brief Destructor for DShotRMT.
     * Cleans up RMT channels and encoder resources.
     */
    ~DShotRMT();

    // Public Core Functions
    /**
     * @brief Initializes the DShot RMT channels and encoder.
     * @return dshot_result_t indicating success or failure of the initialization.
     */
    dshot_result_t begin();

    /**
     * @brief Sends a DShot throttle value to the ESC.
     * @param throttle The throttle value (48-2047). A value of 0 sends a motor stop command.
     * @return dshot_result_t indicating success or failure of the transmission.
     */
    dshot_result_t sendThrottle(uint16_t throttle);

    /**
     * @brief Sends a DShot throttle value as a percentage to the ESC.
     * @param percent The throttle percentage (0.0f - 100.0f).
     * @return dshot_result_t indicating success or failure of the transmission.
     */
    dshot_result_t sendThrottlePercent(float percent);

    /**
     * @brief Sends a single DShot command to the ESC.
     * @param command The DShot command value (0-47).
     * @return dshot_result_t indicating success or failure of the transmission.
     */
    dshot_result_t sendCommand(uint16_t command);

    /**
     * @brief Sends a DShot command multiple times with a delay between repetitions. This is a blocking function.
     * @param dshot_command The DShot command to send.
     * @param repeat_count The number of times to repeat the command.
     * @param delay_us The delay in microseconds between repetitions.
     * @return dshot_result_t indicating success or failure of the transmission.
     */
    dshot_result_t sendCommand(dshotCommands_e dshot_command, uint16_t repeat_count = DEFAULT_CMD_REPEAT_COUNT, uint16_t delay_us = DEFAULT_CMD_DELAY_US);

    /**
     * @brief Retrieves telemetry data from the ESC.
     * @param magnet_count The number of magnets in the motor. If 0, uses the stored motor_magnet_count.
     * @return dshot_result_t containing telemetry data (eRPM, motor RPM) if successful.
     */
    dshot_result_t getTelemetry(uint16_t magnet_count = 0);

    /**
     * @brief Sends a command to the ESC to request ESC information.
     * @return dshot_result_t indicating success or failure of the command transmission.
     */
    dshot_result_t getESCInfo();

    /**
     * @brief Sets the motor spin direction.
     * @param reversed True for reversed direction, false for normal.
     * @return dshot_result_t indicating success or failure of the command transmission.
     */
    dshot_result_t setMotorSpinDirection(bool reversed);

    /**
     * @brief Sends a command to the ESC to save its current settings.
     * Use with caution as this writes to ESC's non-volatile memory.
     * @return dshot_result_t indicating success or failure of the command transmission.
     */
    dshot_result_t saveESCSettings();

    // Public Utility & Info Functions
    /**
     * @brief Prints detailed DShot signal information for a given DShotRMT instance.
     * @param dshot_rmt The DShotRMT instance to get information from.
     * @param output The output stream (e.g., Serial) to print to. Defaults to Serial.
     */
    static void printDShotInfo(const DShotRMT &dshot_rmt, Stream &output = Serial);

    /**
     * @brief Prints detailed CPU information.
     * @param output The output stream (e.g., Serial) to print to. Defaults to Serial.
     */
    static void printCpuInfo(Stream &output = Serial);

    /**
     * @brief Sets the motor magnet count for RPM calculation.
     * @param magnet_count The number of magnets in the motor.
     */
    void setMotorMagnetCount(uint16_t magnet_count);

    /**
     * @brief Gets the current DShot mode.
     * @return The current dshot_mode_t.
     */
    dshot_mode_t getMode() const { return _mode; }

    /**
     * @brief Checks if bidirectional DShot is enabled.
     * @return True if bidirectional DShot is enabled, false otherwise.
     */
    bool isBidirectional() const { return _is_bidirectional; }

    /**
     * @brief Gets the last encoded DShot frame value.
     * @return The 16-bit encoded DShot frame value.
     */
    uint16_t getEncodedFrameValue() const { return _encoded_frame_value; }

    /**
     * @brief Gets the last transmitted throttle value.
     * @return The last transmitted throttle value.
     */
    uint16_t getThrottleValue() const { return _packet.throttle_value; }

    // Deprecated Methods
    /**
     * @brief Deprecated. Use sendThrottle() instead.
     * @param throttle The throttle value.
     * @return True on success, false on failure.
     */
    [[deprecated("Use sendThrottle() instead")]]
    bool setThrottle(uint16_t throttle)
    {
        auto result = sendThrottle(throttle);
        return result.success;
    }

    /**
     * @brief Deprecated. Use sendCommand() instead.
     * @param command The DShot command.
     * @return True on success, false on failure.
     */
    [[deprecated("Use sendCommand() instead")]]
    bool sendDShotCommand(uint16_t command)
    {
        auto result = sendCommand(command);
        return result.success;
    }

    /**
     * @brief Deprecated. Use getTelemetry() instead.
     * @param magnet_count The number of magnets in the motor.
     * @return The motor RPM.
     */
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
    uint16_t _level0; // DShot protocol: Signal is idle-low, so pulses start by going HIGH.
    uint16_t _level1; // DShot protocol: Signal returns to LOW after the high pulse.

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
    dshot_result_t _encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols);
    uint16_t _decodeDShotFrame(const rmt_symbol_word_t *symbols);

    // Private Timing Control Functions
    bool _timer_signal();
    bool _timer_reset();

    // Static Callback Functions
    static bool _on_rx_done(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data);
};
