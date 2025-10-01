#pragma once

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <atomic>

// Defines the available DShot communication speeds.
enum dshot_mode_t
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
enum class dshot_msg_code_t
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
    DSHOT_CMD_MAX = 47
};

// DShot Protocol Constants
static constexpr auto DSHOT_THROTTLE_FAILSAFE = 0;
static constexpr auto DSHOT_THROTTLE_MIN = 48;
static constexpr auto DSHOT_THROTTLE_MAX = 2047;
static constexpr auto DSHOT_BITS_PER_FRAME = 16;
static constexpr auto DEFAULT_MOTOR_MAGNET_COUNT = 14;

// Custom status codes
static constexpr int DSHOT_OK = 0;
static constexpr int DSHOT_ERROR = 1;

// Configuration Constants
static constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;
static constexpr auto DSHOT_FULL_PACKET = 0b1111111111111111;
static constexpr auto DSHOT_CRC_MASK = 0b0000000000001111;
static constexpr auto DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
static constexpr auto DSHOT_RMT_RESOLUTION = 8000000;                    // 8 MHz resolution
static constexpr auto RMT_TICKS_PER_US = DSHOT_RMT_RESOLUTION / 1000000; // RMT Ticks per microsecond
static constexpr auto DSHOT_RX_TIMEOUT_MS = 2;
static constexpr auto DSHOT_PADDING_US = 20; // Pause between frames
static constexpr auto RMT_BUFFER_SYMBOLS = 64;
static constexpr auto RMT_QUEUE_DEPTH = 1;
static constexpr auto GCR_BITS_PER_FRAME = 21; // GCR bits in a DShot answer frame
static constexpr auto POLE_PAIRS_MIN = 1;
static constexpr auto MAGNETS_PER_POLE_PAIR = 2;
static constexpr auto NO_DSHOT_TELEMETRY = 0;
static constexpr auto DSHOT_PULSE_MIN_NS = 800;  // 0.8us minimum pulse
static constexpr auto DSHOT_PULSE_MAX_NS = 8000; // 8.0us maximum pulse
static constexpr auto DSHOT_TELEMETRY_INVALID = DSHOT_THROTTLE_MAX;
static constexpr auto DSHOT_TELEMETRY_BIT_POSITION = 11;
static constexpr auto DSHOT_CRC_BIT_SHIFT = 4;

// Command Constants
static constexpr auto DEFAULT_CMD_DELAY_US = 10;
static constexpr auto DEFAULT_CMD_REPEAT_COUNT = 1;
static constexpr auto SETTINGS_COMMAND_REPEATS = 10;
static constexpr auto SETTINGS_COMMAND_DELAY_US = 5;

// Timing parameters for each DShot mode
const dshot_timing_us_t DSHOT_TIMING_US[] = {
    {0.00, 0.00}, // DSHOT_OFF
    {6.67, 5.00}, // DSHOT150
    {3.33, 2.50}, // DSHOT300
    {1.67, 1.25}, // DSHOT600
    {0.83, 0.67}  // DSHOT1200
};

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
inline const char *_get_result_code_str(dshot_msg_code_t code)
{
    switch (code)
    {
    case dshot_msg_code_t::DSHOT_NONE:
        return NONE;
    case dshot_msg_code_t::DSHOT_UNKNOWN:
        return UNKNOWN_ERROR;
    case dshot_msg_code_t::DSHOT_TX_INIT_FAILED:
        return TX_INIT_FAILED;
    case dshot_msg_code_t::DSHOT_RX_INIT_FAILED:
        return RX_INIT_FAILED;
    case dshot_msg_code_t::DSHOT_ENCODER_INIT_FAILED:
        return ENCODER_INIT_FAILED;
    case dshot_msg_code_t::DSHOT_CALLBACK_REGISTERING_FAILED:
        return CALLBACK_REGISTERING_FAILED;
    case dshot_msg_code_t::DSHOT_RECEIVER_FAILED:
        return RECEIVER_FAILED;
    case dshot_msg_code_t::DSHOT_TRANSMISSION_FAILED:
        return TRANSMISSION_FAILED;
    case dshot_msg_code_t::DSHOT_THROTTLE_NOT_IN_RANGE:
        return THROTTLE_NOT_IN_RANGE;
    case dshot_msg_code_t::DSHOT_PERCENT_NOT_IN_RANGE:
        return PERCENT_NOT_IN_RANGE;
    case dshot_msg_code_t::DSHOT_COMMAND_NOT_VALID:
        return COMMAND_NOT_VALID;
    case dshot_msg_code_t::DSHOT_BIDIR_NOT_ENABLED:
        return BIDIR_NOT_ENABLED;
    case dshot_msg_code_t::DSHOT_TELEMETRY_FAILED:
        return TELEMETRY_FAILED;
    case dshot_msg_code_t::DSHOT_INVALID_MAGNET_COUNT:
        return INVALID_MAGNET_COUNT;
    case dshot_msg_code_t::DSHOT_INVALID_COMMAND:
        return INVALID_COMMAND;
    case dshot_msg_code_t::DSHOT_TIMING_CORRECTION:
        return TIMING_CORRECTION;
    case dshot_msg_code_t::DSHOT_INIT_FAILED:
        return INIT_FAILED;
    case dshot_msg_code_t::DSHOT_INIT_SUCCESS:
        return INIT_SUCCESS;
    case dshot_msg_code_t::DSHOT_TX_INIT_SUCCESS:
        return TX_INIT_SUCCESS;
    case dshot_msg_code_t::DSHOT_RX_INIT_SUCCESS:
        return RX_INIT_SUCCESS;
    case dshot_msg_code_t::DSHOT_ENCODER_INIT_SUCCESS:
        return ENCODER_INIT_SUCCESS;
    case dshot_msg_code_t::DSHOT_ENCODING_SUCCESS:
        return ENCODING_SUCCESS;
    case dshot_msg_code_t::DSHOT_TRANSMISSION_SUCCESS:
        return TRANSMISSION_SUCCESS;
    case dshot_msg_code_t::DSHOT_TELEMETRY_SUCCESS:
        return TELEMETRY_SUCCESS;
    case dshot_msg_code_t::DSHOT_COMMAND_SUCCESS:
        return COMMAND_SUCCESS;
    default:
        return UNKNOWN_ERROR;
    }
}