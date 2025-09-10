/*
 * DShotCommandManager.cpp
 * Advanced DShot command management for DShotRMT library
 * Author: Wastl Kraus
 * Date: 2025-09-04
 * License: MIT
 */

#include <DShotCommandManager.h>

// Constructor
DShotCommandManager::DShotCommandManager(DShotRMT &dshot_instance)
    : _dshot(dshot_instance),
      _total_commands_sent(0),
      _failed_commands(0),
      _last_command_timestamp(0)
{
}

// Init command manager
dshot_result_t DShotCommandManager::begin()
{
    dshot_result_t result;
    result.success = true;
    result.msg = "Success";
    return result;
}

// --- BASIC COMMAND METHODS ---
dshot_result_t DShotCommandManager::sendCommand(dshot_commands_t command, uint16_t repeat_count)
{
    return sendCommandWithDelay(command, repeat_count, DEFAULT_COMMAND_DELAY_MS);
}

//
dshot_result_t DShotCommandManager::sendCommandWithDelay(dshot_commands_t command, uint16_t repeat_count, uint32_t delay_ms)
{
    dshot_result_t result = {false, "Unknown error"};

    if (!isValidCommand(command))
    {
        result.msg = "Invalid command";
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
            result.msg = single_result.msg;
            break;
        }

        // Add delay between repetitions (except for last repetition)
        if (i < repeat_count - 1)
        {
            _delay_ms(delay_ms);
        }
    }

    //
    result.success = all_successful;

    if (result.success)
    {
        result.msg = "Success";
    }

    return result;
}

// --- MOTOR CONTROL COMMANDS ---
dshot_result_t DShotCommandManager::stopMotor()
{
    return sendCommand(DSHOT_CMD_MOTOR_STOP);
}

//
dshot_result_t DShotCommandManager::set3DMode(bool enable)
{
    dshot_commands_t command = enable ? DSHOT_CMD_3D_MODE_ON : DSHOT_CMD_3D_MODE_OFF;
    return sendCommandWithDelay(command, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_MS);
}

//
dshot_result_t DShotCommandManager::setSpinDirection(bool reversed)
{
    dshot_commands_t command = reversed ? DSHOT_CMD_SPIN_DIRECTION_REVERSED : DSHOT_CMD_SPIN_DIRECTION_NORMAL;
    return sendCommandWithDelay(command, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_MS);
}

//
dshot_result_t DShotCommandManager::saveSettings()
{
    return sendCommandWithDelay(DSHOT_CMD_SAVE_SETTINGS, SETTINGS_COMMAND_REPEATS, SETTINGS_COMMAND_DELAY_MS);
}

// --- TELEMETRY COMMANDS ---
dshot_result_t DShotCommandManager::setExtendedTelemetry(bool enable)
{
    dshot_commands_t command = enable ? DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE : DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE;
    return sendCommand(command);
}

//
dshot_result_t DShotCommandManager::requestESCInfo()
{
    return sendCommand(DSHOT_CMD_ESC_INFO);
}

// --- LED CONTROL COMMANDS ---
dshot_result_t DShotCommandManager::setLED(uint8_t led_number, bool state)
{
    if (led_number > 3)
    {
        dshot_result_t result = {false, "Invalid LED number (0-3)"};
        return result;
    }

    dshot_commands_t command;
    if (state)
    {
        // LED ON commands
        switch (led_number)
        {
        case 0:
            command = DSHOT_CMD_LED0_ON;
            break;
        case 1:
            command = DSHOT_CMD_LED1_ON;
            break;
        case 2:
            command = DSHOT_CMD_LED2_ON;
            break;
        case 3:
            command = DSHOT_CMD_LED3_ON;
            break;
        }
    }
    else
    {
        // LED OFF commands
        switch (led_number)
        {
        case 0:
            command = DSHOT_CMD_LED0_OFF;
            break;
        case 1:
            command = DSHOT_CMD_LED1_OFF;
            break;
        case 2:
            command = DSHOT_CMD_LED2_OFF;
            break;
        case 3:
            command = DSHOT_CMD_LED3_OFF;
            break;
        }
    }

    return sendCommand(command);
}

// --- BEACON COMMANDS ---
dshot_result_t DShotCommandManager::activateBeacon(uint8_t beacon_number)
{
    if (beacon_number < 1 || beacon_number > 5)
    {
        dshot_result_t result = {false, "Invalid beacon number (1-5)"};
        return result;
    }

    dshot_commands_t command = static_cast<dshot_commands_t>(DSHOT_CMD_BEACON1 + beacon_number - 1);
    return sendCommand(command);
}

// --- KISS ESC SPECIFIC COMMANDS ---
dshot_result_t DShotCommandManager::setAudioStreamMode(bool enable)
{
    // KISS audio stream mode is a toggle command
    return sendCommand(DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF);
}

//
dshot_result_t DShotCommandManager::setSilentMode(bool enable)
{
    // KISS silent mode is a toggle command
    return sendCommand(DSHOT_CMD_SILENT_MODE_ON_OFF);
}

// --- SEQUENCE COMMANDS ---
dshot_result_t DShotCommandManager::executeSequence(const dshot_commandmanager_item_t *sequence, size_t sequence_length)
{
    dshot_result_t result = {true, "Success"};
    uint64_t total_start_time = esp_timer_get_time();

    for (size_t i = 0; i < sequence_length; i++)
    {
        dshot_result_t item_result = sendCommandWithDelay(
            sequence[i].command,
            sequence[i].repeat_count,
            DEFAULT_COMMAND_DELAY_MS);

        if (!item_result.success)
        {
            result.success = false;
            result.msg = item_result.msg;
            break;
        }

        // Add delay after command if specified
        if (sequence[i].delay_ms > 0)
        {
            _delay_ms(sequence[i].delay_ms);
        }
    }

    uint64_t total_end_time = esp_timer_get_time();

    return result;
}

//
dshot_result_t DShotCommandManager::executeInitSequence()
{
    // Basic ESC initialization sequence
    dshot_commandmanager_item_t init_sequence[] = {
        {DSHOT_CMD_MOTOR_STOP, 5, 100},               // Stop motor, repeat 5 times, wait 100ms
        {DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE, 1, 50}, // Enable telemetry, wait 50ms
        {DSHOT_CMD_ESC_INFO, 1, 100}                  // Request ESC info, wait 100ms
    };

    return executeSequence(init_sequence, sizeof(init_sequence) / sizeof(init_sequence[0]));
}

//
dshot_result_t DShotCommandManager::executeCalibrationSequence()
{
    // Basic ESC calibration sequence
    dshot_commandmanager_item_t calibration_sequence[] = {
        {DSHOT_CMD_MOTOR_STOP, 10, 500},            // Ensure motor is stopped
        {DSHOT_CMD_SPIN_DIRECTION_NORMAL, 10, 100}, // Set normal spin direction
        {DSHOT_CMD_3D_MODE_OFF, 10, 100},           // Disable 3D mode
        {DSHOT_CMD_SAVE_SETTINGS, 10, 1000},        // Save settings
        {DSHOT_CMD_MOTOR_STOP, 5, 100}              // Final stop
    };

    return executeSequence(calibration_sequence, sizeof(calibration_sequence) / sizeof(calibration_sequence[0]));
}

// --- UTILITY METHODS ---
const char *DShotCommandManager::getCommandName(dshot_commands_t command)
{
    switch (command)
    {
    case DSHOT_CMD_MOTOR_STOP:
        return "MOTOR_STOP";
    case DSHOT_CMD_BEACON1:
        return "BEACON1";
    case DSHOT_CMD_BEACON2:
        return "BEACON2";
    case DSHOT_CMD_BEACON3:
        return "BEACON3";
    case DSHOT_CMD_BEACON4:
        return "BEACON4";
    case DSHOT_CMD_BEACON5:
        return "BEACON5";
    case DSHOT_CMD_ESC_INFO:
        return "ESC_INFO";
    case DSHOT_CMD_SPIN_DIRECTION_1:
        return "SPIN_DIRECTION_1";
    case DSHOT_CMD_SPIN_DIRECTION_2:
        return "SPIN_DIRECTION_2";
    case DSHOT_CMD_3D_MODE_OFF:
        return "3D_MODE_OFF";
    case DSHOT_CMD_3D_MODE_ON:
        return "3D_MODE_ON";
    case DSHOT_CMD_SETTINGS_REQUEST:
        return "SETTINGS_REQUEST";
    case DSHOT_CMD_SAVE_SETTINGS:
        return "SAVE_SETTINGS";
    case DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE:
        return "EXTENDED_TELEMETRY_ENABLE";
    case DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE:
        return "EXTENDED_TELEMETRY_DISABLE";
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
        return "SPIN_DIRECTION_NORMAL";
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
        return "SPIN_DIRECTION_REVERSED";
    case DSHOT_CMD_LED0_ON:
        return "LED0_ON";
    case DSHOT_CMD_LED1_ON:
        return "LED1_ON";
    case DSHOT_CMD_LED2_ON:
        return "LED2_ON";
    case DSHOT_CMD_LED3_ON:
        return "LED3_ON";
    case DSHOT_CMD_LED0_OFF:
        return "LED0_OFF";
    case DSHOT_CMD_LED1_OFF:
        return "LED1_OFF";
    case DSHOT_CMD_LED2_OFF:
        return "LED2_OFF";
    case DSHOT_CMD_LED3_OFF:
        return "LED3_OFF";
    case DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF:
        return "AUDIO_STREAM_MODE_ON_OFF";
    case DSHOT_CMD_SILENT_MODE_ON_OFF:
        return "SILENT_MODE_ON_OFF";
    default:
        return "UNKNOWN";
    }
}

//
bool DShotCommandManager::isValidCommand(dshot_commands_t command)
{
    return (command >= DSHOT_CMD_MOTOR_STOP && command <= DSHOT_CMD_MAX);
}

// --- PRIVATE METHODS ---
dshot_result_t DShotCommandManager::_executeCommand(dshot_commands_t command)
{
    uint64_t start_time = esp_timer_get_time();

    // Execute the command using the DShotRMT instance
    dshot_result_t result = _dshot.sendCommand(static_cast<uint16_t>(command));

    uint64_t end_time = esp_timer_get_time();
    _last_command_timestamp = end_time;

    return result;
}

//
void DShotCommandManager::_delay_ms(uint32_t delay_ms)
{
    if (delay_ms > 0)
    {
        delay(delay_ms);
    }
}
