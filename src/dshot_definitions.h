#pragma once

#include <Arduino.h>
#include <driver/gpio.h> // Added for gpio_num_t
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <atomic> // Added for std::atomic

/**
 * @brief DShot Modes
 * Defines the available DShot communication speeds.
 */
enum class dshot_mode_t
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
};

/**
 * @brief DShot Packet Structure
 * Represents the 16-bit DShot data packet sent to the ESC.
 */
typedef struct dshot_packet
{
    uint16_t throttle_value : 11; ///< 11-bit throttle value or command.
    bool telemetric_request : 1;  ///< 1-bit telemetry request flag.
    uint16_t checksum : 4;        ///< 4-bit CRC checksum.
} dshot_packet_t;

/**
 * @brief DShot Timing Configuration
 * Defines the bit length and high time for a '1' bit in microseconds for each DShot mode.
 */
typedef struct dshot_timing
{
    double bit_length_us; ///< Total duration of one bit in microseconds.
    double t1h_lenght_us; ///< High time duration for a '1' bit in microseconds.
} dshot_timing_us_t;

/**
 * @brief RMT Timing Configuration
 * Stores pre-calculated timing values in RMT ticks for efficient signal generation.
 */
typedef struct rmt_ticks
{
    uint16_t bit_length_ticks; ///< Total duration of one bit in RMT ticks.
    uint16_t t1h_ticks;        ///< High time duration for a '1' bit in RMT ticks.
    uint16_t t1l_ticks;        ///< Low time duration for a '1' bit in RMT ticks.
    uint16_t t0h_ticks;        ///< High time duration for a '0' bit in RMT ticks.
    uint16_t t0l_ticks;        ///< Low time duration for a '0' bit in RMT ticks.
} rmt_ticks_t;

/**
 * @brief DShot Error Codes
 * Enum class for specific error and success codes returned by DShotRMT functions.
 */
enum class dshot_msg_code_t
{
    DSHOT_ERROR_NONE = 0,
    DSHOT_ERROR_UNKNOWN,
    DSHOT_ERROR_TX_INIT_FAILED,
    DSHOT_ERROR_RX_INIT_FAILED,
    DSHOT_ERROR_ENCODER_INIT_FAILED,
    DSHOT_ERROR_CALLBACK_REGISTERING_FAILED,
    DSHOT_ERROR_RECEIVER_FAILED,
    DSHOT_ERROR_TRANSMISSION_FAILED,
    DSHOT_ERROR_THROTTLE_NOT_IN_RANGE,
    DSHOT_ERROR_PERCENT_NOT_IN_RANGE,
    DSHOT_ERROR_COMMAND_NOT_VALID,
    DSHOT_ERROR_BIDIR_NOT_ENABLED,
    DSHOT_ERROR_TELEMETRY_FAILED,
    DSHOT_ERROR_INVALID_MAGNET_COUNT,
    DSHOT_ERROR_INVALID_COMMAND,
    DSHOT_ERROR_TIMING_CORRECTION,
    DSHOT_ERROR_INIT_FAILED,
    DSHOT_ERROR_INIT_SUCCESS,
    DSHOT_ERROR_TX_INIT_SUCCESS,
    DSHOT_ERROR_RX_INIT_SUCCESS,
    DSHOT_ERROR_ENCODER_INIT_SUCCESS,
    DSHOT_ERROR_ENCODING_SUCCESS,
    DSHOT_ERROR_TRANSMISSION_SUCCESS,
    DSHOT_ERROR_TELEMETRY_SUCCESS,
    DSHOT_ERROR_COMMAND_SUCCESS
};

/**
 * @brief Unified DShot Result Structure
 * Contains the success status, an error code, and optional telemetry data.
 */
typedef struct dshot_result
{
    bool success;
    dshot_msg_code_t error_code; ///< Specific error or success code.
    uint16_t erpm;               ///< Electrical RPM (eRPM) if telemetry is successful.
    uint16_t motor_rpm;          ///< Motor RPM if telemetry is successful and magnet count is known.
} dshot_result_t;

/**
 * @brief DShot Commands
 * Enum class for standard DShot commands that can be sent to an ESC.
 */
enum dshotCommands_e
{
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE,
    DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON,                       // BLHeli32 only
    DSHOT_CMD_LED1_ON,                       // BLHeli32 only
    DSHOT_CMD_LED2_ON,                       // BLHeli32 only
    DSHOT_CMD_LED3_ON,                       // BLHeli32 only
    DSHOT_CMD_LED0_OFF,                      // BLHeli32 only
    DSHOT_CMD_LED1_OFF,                      // BLHeli32 only
    DSHOT_CMD_LED2_OFF,                      // BLHeli32 only
    DSHOT_CMD_LED3_OFF,                      // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31,       // KISS silent Mode on/Off
    DSHOT_CMD_MAX = 47
};

/**
 * @brief DShot Command Type Enum
 * Defines how DShot commands are sent.
 */
enum class dshotCommandType_e
{
    DSHOT_CMD_TYPE_INLINE = 0, ///< Commands sent inline with motor signal (motors must be enabled).
    DSHOT_CMD_TYPE_BLOCKING    ///< Commands sent in blocking method (motors must be disabled).
};

// DShot Protocol Constants
const uint16_t DSHOT_THROTTLE_FAILSAFE = 0;
const uint16_t DSHOT_THROTTLE_MIN = 48;
const uint16_t DSHOT_THROTTLE_MAX = 2047;
const uint16_t DSHOT_BITS_PER_FRAME = 16;
const uint16_t DEFAULT_MOTOR_MAGNET_COUNT = 14;

// Custom status codes
const int DSHOT_OK = 0;
const int DSHOT_ERROR = 1;

// Configuration Constants (from DShotRMT.h private section)
const uint16_t DSHOT_NULL_PACKET = 0b0000000000000000;
const uint16_t DSHOT_FULL_PACKET = 0b1111111111111111;
const uint16_t DSHOT_CRC_MASK = 0b0000000000001111;
const rmt_clock_source_t DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
const uint32_t DSHOT_RMT_RESOLUTION = 8 * 1000 * 1000;                      // 8 MHz resolution
const uint16_t RMT_TICKS_PER_US = DSHOT_RMT_RESOLUTION / (1 * 1000 * 1000); // RMT Ticks per microsecond
const uint16_t DSHOT_RX_TIMEOUT_MS = 2;
const uint16_t DSHOT_PADDING_US = 20; // Add to pause between frames for compatibility
const uint16_t RMT_BUFFER_SYMBOLS = 64;
const uint16_t RMT_QUEUE_DEPTH = 1;
const uint16_t GCR_BITS_PER_FRAME = 21; // Number of GCR bits in a DShot answer frame
const uint16_t POLE_PAIRS_MIN = 1;
const uint16_t MAGNETS_PER_POLE_PAIR = 2;
const uint16_t NO_DSHOT_TELEMETRY = 0;
const uint16_t DSHOT_PULSE_MIN = 800;  // 0.8us minimum pulse
const uint16_t DSHOT_PULSE_MAX = 8000; // 8.0us maximum pulse
const uint16_t DSHOT_TELEMETRY_INVALID = DSHOT_THROTTLE_MAX;
const uint16_t DSHOT_TELEMETRY_BIT_POSITION = 11; // Bit position of the telemetry request flag in the DShot frame
const uint16_t DSHOT_CRC_BIT_SHIFT = 4;           // Number of bits to shift to extract data from data_and_crc

// Command Constants (from DShotRMT.h private section)
const uint16_t DEFAULT_CMD_DELAY_US = 10;
const uint16_t DEFAULT_CMD_REPEAT_COUNT = 1;
const uint16_t SETTINGS_COMMAND_REPEATS = 10; // Settings commands need 10 repeats
const uint16_t SETTINGS_COMMAND_DELAY_US = 5;

// Timing parameters for each DShot mode
// Format: {bit_length_us, t1h_length_us}
const dshot_timing_us_t DSHOT_TIMING_US[] = {
    {0.00, 0.00},  // DSHOT_OFF
    {6.67, 5.00},  // DSHOT150
    {3.33, 2.50},  // DSHOT300
    {1.67, 1.25},  // DSHOT600
    {0.83, 0.67}}; // DSHOT1200

// Error Messages
const char *const NONE = "";
const char *const UNKNOWN_ERROR = "Unknown Error!";
const char *const INIT_SUCCESS = "SignalGeneratorRMT initialized successfully";
const char *const INIT_FAILED = "SignalGeneratorRMT init failed!";
const char *const TX_INIT_SUCCESS = "TX RMT channel initialized successfully";
const char *const TX_INIT_FAILED = "TX RMT channel init failed!";
const char *const RX_INIT_SUCCESS = "RX RMT channel initialized successfully";
const char *const RX_INIT_FAILED = "RX RMT channel init failed!";
const char *const ENCODER_INIT_SUCCESS = "RMT encoder initialized successfully";
const char *const ENCODER_INIT_FAILED = "RMT encoder init failed!";
const char *const ENCODING_SUCCESS = "Packet encoded successfully";
const char *const TRANSMISSION_SUCCESS = "Transmission successfully";
const char *const TRANSMISSION_FAILED = "Transmission failed!";
const char *const RECEIVER_FAILED = "RMT receiver failed!";
const char *const THROTTLE_NOT_IN_RANGE = "Throttle not in range! (48 - 2047)";
const char *const PERCENT_NOT_IN_RANGE = "Percent not in range! (0.0 - 100.0)";
const char *const COMMAND_NOT_VALID = "Command not valid! (0 - 47)";
const char *const BIDIR_NOT_ENABLED = "Bidirectional DShot not enabled!";
const char *const TELEMETRY_SUCCESS = "Valid Telemetric Frame received!";
const char *const TELEMETRY_FAILED = "No valid Telemetric Frame received!";
const char *const INVALID_MAGNET_COUNT = "Invalid motor magnet count!";
const char *const TIMING_CORRECTION = "Timing correction!";
const char *const CALLBACK_REGISTERING_FAILED = "RMT RX Callback registering failed!";
const char *const INVALID_COMMAND = "Invalid command!";
const char *const COMMAND_SUCCESS = "DShot command sent successfully";
