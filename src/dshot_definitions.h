/**
 * @file dshot_definitions.h
 * @brief Defines DShot protocol constants, data structures, and command enums for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-10-04
 * @license MIT
 */

#pragma once

#include <cstdint>
#include <driver/rmt_common.h>

// DShot protocol definitions
static constexpr uint16_t DSHOT_FRAME_LENGTH = 16;           // 11 throttle bits + 1 telemetry bit + 4 CRC bits
static constexpr uint16_t DSHOT_BITS_PER_FRAME = 16;
static constexpr uint16_t DSHOT_THROTTLE_MAX = 2047;         // Maximum throttle value (0-2047)
static constexpr uint16_t DSHOT_THROTTLE_MIN = 48;           // Minimum throttle value for motor spin
static constexpr float DSHOT_PERCENT_MIN = 0.0f;
static constexpr float DSHOT_PERCENT_MAX = 100.0f;
static constexpr uint16_t DSHOT_CMD_MIN = 0;                 // Minimum command value
static constexpr uint16_t DSHOT_CMD_MAX = 47;                // Maximum command value
static constexpr uint16_t DSHOT_TELEMETRY_BIT_MASK = 0x0800; // Bit mask for telemetry request bit (11th bit)
static constexpr uint16_t DSHOT_CRC_MASK = 0x000F;           // Bit mask for CRC bits

// Default motor magnet count for RPM calculation
static constexpr uint16_t DEFAULT_MOTOR_MAGNET_COUNT = 14;

// Defines the available DShot communication speeds.
enum dshot_mode_t : uint8_t
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
};

// Represents the 16-bit DShot data packet sent to the ESC.
typedef struct dshot_packet
{
    uint16_t throttle_value : 11; // 11-bit throttle value or command.
    bool telemetric_request : 1;  // 1-bit telemetry request flag.
    uint16_t checksum : 4;        // 4-bit CRC checksum.
} dshot_packet_t;

// Defines the bit length and high time for a '1' bit in microseconds for each DShot mode.
typedef struct dshot_timing
{
    double bit_length_us; // Total duration of one bit in microseconds.
    double t1h_lenght_us; // High time duration for a '1' bit in microseconds.
} dshot_timing_us_t;

// Stores pre-calculated timing values in RMT ticks for efficient signal generation.
typedef struct rmt_ticks
{
    uint16_t bit_length_ticks; // Total duration of one bit in RMT ticks.
    uint16_t t1h_ticks;        // High time duration for a '1' bit in RMT ticks.
    uint16_t t1l_ticks;        // Low time duration for a '1' bit in RMT ticks.
    uint16_t t0h_ticks;        // High time duration for a '0' bit in RMT ticks.
    uint16_t t0l_ticks;        // Low time duration for a '0' bit in RMT ticks.
} rmt_ticks_t;

// Enum class for specific error and success codes
enum dshot_msg_code_t
{
    DSHOT_NONE = 0,
    DSHOT_UNKNOWN,
    DSHOT_TX_INIT_FAILED,
    DSHOT_RX_INIT_FAILED,
    DSHOT_ENCODER_INIT_FAILED,
    DSHOT_CALLBACK_REGISTERING_FAILED,
    DSHOT_RECEIVER_FAILED,
    DSHOT_TRANSMISSION_FAILED,
    DSHOT_THROTTLE_NOT_IN_RANGE,
    DSHOT_PERCENT_NOT_IN_RANGE,
    DSHOT_COMMAND_NOT_VALID,
    DSHOT_BIDIR_NOT_ENABLED,
    DSHOT_TELEMETRY_FAILED,
    DSHOT_INVALID_MAGNET_COUNT,
    DSHOT_INVALID_COMMAND,
    DSHOT_TIMING_CORRECTION,
    DSHOT_INIT_FAILED,
    DSHOT_INIT_SUCCESS,
    DSHOT_TX_INIT_SUCCESS,
    DSHOT_RX_INIT_SUCCESS,
    DSHOT_ENCODER_INIT_SUCCESS,
    DSHOT_ENCODING_SUCCESS,
    DSHOT_TRANSMISSION_SUCCESS,
    DSHOT_TELEMETRY_SUCCESS,
    DSHOT_COMMAND_SUCCESS
};

// Contains the success status, an error code, and optional telemetry data.
typedef struct dshot_result
{
    bool success;
    dshot_msg_code_t result_code; // Specific error or success code.
    uint16_t erpm;                // Electrical RPM (eRPM) if telemetry is successful.
    uint16_t motor_rpm;           // Motor RPM if telemetry is successful and magnet count is known.
} dshot_result_t;

// Standard DShot commands by "betaflight"
enum dshotCommands_e
{
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO,
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST,
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE,
    DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON,
    DSHOT_CMD_LED1_ON,
    DSHOT_CMD_LED2_ON,
    DSHOT_CMD_LED3_ON,
    DSHOT_CMD_LED0_OFF,
    DSHOT_CMD_LED1_OFF,
    DSHOT_CMD_LED2_OFF,
    DSHOT_CMD_LED3_OFF,
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31,
    DSHOT_CMD_MAX_VALUE = 47
};

// Custom status codes
static constexpr int DSHOT_OK = 0;
static constexpr int DSHOT_ERROR = 1;

// Configuration Constants
static constexpr auto DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
static constexpr auto DSHOT_RMT_RESOLUTION = 8000000;                    // 8 MHz resolution
static constexpr auto RMT_TICKS_PER_US = DSHOT_RMT_RESOLUTION / 1000000; // RMT Ticks per microsecond
static constexpr auto RMT_BUFFER_SYMBOLS = 64;
static constexpr auto RMT_QUEUE_DEPTH = 1;

// Timing parameters for each DShot mode
const dshot_timing_us_t DSHOT_TIMING_US[] = {
    {0.00, 0.00}, // DSHOT_OFF
    {6.67, 5.00}, // DSHOT150
    {3.33, 2.50}, // DSHOT300
    {1.67, 1.25}, // DSHOT600
    {0.83, 0.67}  // DSHOT1200
};