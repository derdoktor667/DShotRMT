/**
 * @file dshot_utils.h
 * @brief Utility functions for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-10-04
 * @license MIT
 */

#pragma once

#include <Arduino.h>

// Forward declaration of the DShotRMT class to break circular dependency
class DShotRMT;

// Error Messages
static constexpr char NONE[] = "";
static constexpr char UNKNOWN_ERROR[] = "Unknown Error!";
static constexpr char INIT_SUCCESS[] = "SignalGeneratorRMT initialized successfully";
static constexpr char INIT_FAILED[] = "SignalGeneratorRMT init failed!";
static constexpr char TX_INIT_SUCCESS[] = "TX RMT channel initialized successfully";
static constexpr char TX_INIT_FAILED[] = "TX RMT channel init failed!";
static constexpr char RX_INIT_SUCCESS[] = "RX RMT channel initialized successfully";
static constexpr char RX_INIT_FAILED[] = "RX RMT channel init failed!";
static constexpr char ENCODER_INIT_SUCCESS[] = "RMT encoder initialized successfully";
static constexpr char ENCODER_INIT_FAILED[] = "RMT encoder init failed!";
static constexpr char ENCODING_SUCCESS[] = "Packet encoded successfully";
static constexpr char TRANSMISSION_SUCCESS[] = "Transmission successfully";
static constexpr char TRANSMISSION_FAILED[] = "Transmission failed!";
static constexpr char RECEIVER_FAILED[] = "RMT receiver failed!";
static constexpr char THROTTLE_NOT_IN_RANGE[] = "Throttle not in range! (48 - 2047)";
static constexpr char PERCENT_NOT_IN_RANGE[] = "Percent not in range! (0.0 - 100.0)";
static constexpr char COMMAND_NOT_VALID[] = "Command not valid! (0 - 47)";
static constexpr char BIDIR_NOT_ENABLED[] = "Bidirectional DShot not enabled!";
static constexpr char TELEMETRY_SUCCESS[] = "Valid Telemetric Frame received!";
static constexpr char TELEMETRY_FAILED[] = "No valid Telemetric Frame received!";
static constexpr char INVALID_MAGNET_COUNT[] = "Invalid motor magnet count!";
static constexpr char TIMING_CORRECTION[] = "Timing correction!";
static constexpr char CALLBACK_REGISTERING_FAILED[] = "RMT RX Callback registering failed!";
static constexpr char INVALID_COMMAND[] = "Invalid command!";
static constexpr char COMMAND_SUCCESS[] = "DShot command sent successfully";

// Helper to get result code string
inline const char *get_result_code_str(dshot_msg_code_t code)
{
    switch (code)
    {
    case DSHOT_NONE:
        return NONE;
    case DSHOT_UNKNOWN:
        return UNKNOWN_ERROR;
    case DSHOT_TX_INIT_FAILED:
        return TX_INIT_FAILED;
    case DSHOT_RX_INIT_FAILED:
        return RX_INIT_FAILED;
    case DSHOT_ENCODER_INIT_FAILED:
        return ENCODER_INIT_FAILED;
    case DSHOT_CALLBACK_REGISTERING_FAILED:
        return CALLBACK_REGISTERING_FAILED;
    case DSHOT_RECEIVER_FAILED:
        return RECEIVER_FAILED;
    case DSHOT_TRANSMISSION_FAILED:
        return TRANSMISSION_FAILED;
    case DSHOT_THROTTLE_NOT_IN_RANGE:
        return THROTTLE_NOT_IN_RANGE;
    case DSHOT_PERCENT_NOT_IN_RANGE:
        return PERCENT_NOT_IN_RANGE;
    case DSHOT_COMMAND_NOT_VALID:
        return COMMAND_NOT_VALID;
    case DSHOT_BIDIR_NOT_ENABLED:
        return BIDIR_NOT_ENABLED;
    case DSHOT_TELEMETRY_FAILED:
        return TELEMETRY_FAILED;
    case DSHOT_INVALID_MAGNET_COUNT:
        return INVALID_MAGNET_COUNT;
    case DSHOT_INVALID_COMMAND:
        return INVALID_COMMAND;
    case DSHOT_TIMING_CORRECTION:
        return TIMING_CORRECTION;
    case DSHOT_INIT_FAILED:
        return INIT_FAILED;
    case DSHOT_INIT_SUCCESS:
        return INIT_SUCCESS;
    case DSHOT_TX_INIT_SUCCESS:
        return TX_INIT_SUCCESS;
    case DSHOT_RX_INIT_SUCCESS:
        return RX_INIT_SUCCESS;
    case DSHOT_ENCODER_INIT_SUCCESS:
        return ENCODER_INIT_SUCCESS;
    case DSHOT_ENCODING_SUCCESS:
        return ENCODING_SUCCESS;
    case DSHOT_TRANSMISSION_SUCCESS:
        return TRANSMISSION_SUCCESS;
    case DSHOT_TELEMETRY_SUCCESS:
        return TELEMETRY_SUCCESS;
    case DSHOT_COMMAND_SUCCESS:
        return COMMAND_SUCCESS;
    default:
        return UNKNOWN_ERROR;
    }
}

// Helper to quick print DShot result codes
inline void printDShotResult(dshot_result_t &result, Stream &output = Serial)
{
    output.printf("Status: %s - %s", result.success ? "SUCCESS" : "FAILED", get_result_code_str(result.result_code));

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
