/*
 * DShotCommandManager.h
 * Advanced DShot command management for DShotRMT library
 * Author: Wastl Kraus
 * Date: 2025-09-04
 * License: MIT
 */

#pragma once

#include <Arduino.h>
#include <DShotRMT.h>

// Command item
typedef struct
{
    dshot_commands_t command;
    uint16_t repeat_count;
    uint32_t delay_ms;
} dshot_commandmanager_item_t;

// Advanced DShot command manager class
class DShotCommandManager
{
public:
    // Constructor
    explicit DShotCommandManager(DShotRMT &dshot_instance);

    // Initialize command manager
    dshot_result_t begin();

    void handleMenuInput(const String &input, Stream &output = Serial);

    // Send a single DShot command
    dshot_result_t sendCommand(dshot_commands_t command, uint16_t repeat_count = 1);

    // Send command with specified delay between repetitions
    dshot_result_t sendCommandWithDelay(dshot_commands_t command, uint16_t repeat_count, uint32_t delay_ms);

    // --- MOTOR CONTROL COMMANDS ---
    // Stop motor (send MOTOR_STOP command)
    dshot_result_t stopMotor();

    // Enable/disable 3D mode
    dshot_result_t set3DMode(bool enable);

    // Set motor spin direction
    dshot_result_t setSpinDirection(bool reversed);

    // Save current settings to ESC
    dshot_result_t saveSettings();

    // --- TELEMETRY COMMANDS ---
    // Enable/disable extended telemetry
    dshot_result_t setExtendedTelemetry(bool enable);

    // Request ESC information
    dshot_result_t requestESCInfo();

    // --- LED CONTROL COMMANDS (BLHeli32 only) ---

    // Control ESC LEDs (BLHeli32 only)
    dshot_result_t setLED(uint8_t led_number, bool state);

    // --- BEACON COMMANDS ---
    // Activate beacon (motor beeping)
    dshot_result_t activateBeacon(uint8_t beacon_number);

    // --- KISS ESC SPECIFIC COMMANDS ---
    // Enable/disable audio stream mode (KISS ESCs)
    dshot_result_t setAudioStreamMode(bool enable);

    // Enable/disable silent mode (KISS ESCs)
    dshot_result_t setSilentMode(bool enable);

    // --- SEQUENCE COMMANDS ---
    // Execute a sequence of DShot commands
    dshot_result_t executeSequence(const dshot_commandmanager_item_t *sequence, size_t sequence_length);

    // Execute ESC initialization sequence
    dshot_result_t executeInitSequence();

    // Execute ESC calibration sequence
    dshot_result_t executeCalibrationSequence();

    // --- UTILITY METHODS ---
    // Get command name as string
    static const char *getCommandName(dshot_commands_t command);

    // Check if command is valid
    static bool isValidCommand(dshot_commands_t command);

    // --- GETTERS ---
    // Get total number of commands sent
    uint32_t getTotalCommandCount() const { return _total_commands_sent; }

    // Get number of failed commands
    uint32_t getFailedCommandCount() const { return _failed_commands; }

    // Get reference to underlying DShotRMT instance
    DShotRMT &getDShotRMT() { return _dshot; }
    const DShotRMT &getDShotRMT() const { return _dshot; }

private:
    // --- PRIVATE MEMBERS ---
    DShotRMT &_dshot;                 // Reference to DShotRMT instance
    uint32_t _total_commands_sent;    // Total commands sent counter
    uint32_t _failed_commands;        // Failed commands counter
    uint64_t _last_command_timestamp; // Timestamp of last command

    // --- PRIVATE METHODS ---
    // Execute single command with timing
    dshot_result_t _executeCommand(dshot_commands_t command);

    // Wait for specified delay
    void _delay_ms(uint32_t delay_ms);

    // --- CONSTANTS ---
    static constexpr uint32_t DEFAULT_COMMAND_DELAY_MS = 10;
    static constexpr uint16_t DEFAULT_REPEAT_COUNT = 1;
    static constexpr uint16_t SETTINGS_COMMAND_REPEATS = 10; // Settings commands need 10 repeats
    static constexpr uint32_t SETTINGS_COMMAND_DELAY_MS = 5;
};
