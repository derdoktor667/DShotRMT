/**
 * @file dshot_definitions.h
 * @brief Defines DShot protocol constants, data structures, and command enums for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-11-29
 * @license MIT
 */

#pragma once

#include <cstdint>
#include <array>
#include <driver/rmt_common.h>

// --- Frame Structure ---
static constexpr uint16_t DSHOT_BITS_PER_FRAME = 16;
static constexpr uint16_t DSHOT_FRAME_SIZE_BYTES = 2;
static constexpr uint16_t DSHOT_TELEMETRY_BIT_POSITION = 11;
static constexpr uint16_t DSHOT_CRC_BIT_SHIFT = 4;
static constexpr uint16_t DSHOT_CRC_MASK = 0x000F;

// --- Signal Levels ---
static constexpr uint16_t DSHOT_PULSE_LEVEL_HIGH = 1;
static constexpr uint16_t DSHOT_PULSE_LEVEL_LOW = 0;

// --- Throttle & Command Values ---
static constexpr uint16_t DSHOT_THROTTLE_MAX = 2047;
static constexpr uint16_t DSHOT_THROTTLE_MIN = 48;
static constexpr float DSHOT_PERCENT_MIN = 0.0f;
static constexpr float DSHOT_PERCENT_MAX = 100.0f;
static constexpr uint16_t DSHOT_CMD_MIN = 0;
static constexpr uint16_t DSHOT_CMD_MAX = 47;
static constexpr uint16_t DSHOT_NULL_PACKET = 0;
static constexpr uint16_t DSHOT_FULL_PACKET = 0xFFFF;
static constexpr uint16_t DSHOT_TELEMETRY_INVALID = DSHOT_THROTTLE_MAX;

// --- Command Behavior ---
static constexpr uint16_t DEFAULT_CMD_DELAY_US = 10;
static constexpr uint16_t DEFAULT_CMD_REPEAT_COUNT = 1;
static constexpr uint16_t SETTINGS_COMMAND_REPEATS = 10;
static constexpr uint16_t SETTINGS_COMMAND_DELAY_US = 5;

// --- GCR Frame Structure ---
static constexpr uint16_t DSHOT_ERPM_FRAME_GCR_BITS = 21;
static constexpr uint16_t DSHOT_TELEMETRY_FULL_GCR_BITS = 110;
static constexpr uint8_t DSHOT_GCR_GROUP_SIZE = 5;
static constexpr uint8_t DSHOT_NIBBLE_SIZE = 4;

// --- Telemetry Payload Structure ---
static constexpr uint16_t DSHOT_TELEMETRY_FRAME_LENGTH_BITS = 80;
static constexpr uint16_t DSHOT_TELEMETRY_FRAME_LENGTH_BYTES = 10;
static constexpr uint16_t DSHOT_TELEMETRY_CRC_LENGTH_BITS = 8;
static constexpr uint16_t DSHOT_TELEMETRY_PAYLOAD_WITH_CRC_BYTES = DSHOT_TELEMETRY_FRAME_LENGTH_BYTES + 1;

// --- Telemetry CRC ---
static constexpr uint8_t DSHOT_TELEMETRY_CRC_POLYNOMIAL = 0x07;

// --- Timeout Constants ---
static constexpr int DSHOT_WAIT_FOREVER = -1;

// --- Motor Properties for RPM Calculation ---
static constexpr uint16_t DEFAULT_MOTOR_MAGNET_COUNT = 14;
static constexpr uint16_t POLE_PAIRS_MIN = 1;
static constexpr uint16_t MAGNETS_PER_POLE_PAIR = 2;

// --- GCR Decoding ---
static constexpr uint8_t GCR_INVALID_NIBBLE = 0xFF;
static constexpr size_t GCR_CODE_LOOKUP_TABLE_SIZE = 32;

// --- Timing & Conversion ---
static constexpr uint32_t DSHOT_MICROSECONDS_PER_MINUTE = 60000000;
static constexpr double NANOSECONDS_PER_MICROSECOND = 1000.0;

// --- DShot Telemetry Decoding ---
static constexpr uint32_t DSHOT_GCR_FRAME_MASK = 0xFFFFF;
static constexpr uint8_t DSHOT_GCR_NIBBLE_MASK = 0x1F;
static constexpr uint8_t DSHOT_GCR_CRC_VALID = 0xF;
static constexpr uint16_t DSHOT_EDT_BUSY_VALUE = 0x0FFF;
static constexpr uint16_t DSHOT_EDT_EXPONENT_MASK = 0x7;
static constexpr uint16_t DSHOT_EDT_MANTISSA_MASK = 0x1FF;

// Lookup table for 5-bit GCR code to 4-bit nibble conversion.
// The index is the 5-bit GCR code, the value is the decoded 4-bit nibble.
static constexpr std::array<uint8_t, GCR_CODE_LOOKUP_TABLE_SIZE> GCR_DECODE_LOOKUP_TABLE = {
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    0b0001,
    0b0100,
    0b0101,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    0b0110,
    0b0111,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    0b1000,
    0b1001,
    0b0010,
    0b0011,
    0b1010,
    0b1011,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    GCR_INVALID_NIBBLE,
    0b1100,
    0b1101,
    0b1110,
    0b1111,
};

static constexpr uint16_t DSHOT_PADDING_US = 20;

// Defines the bit length and high time for a '1' bit in microseconds for each DShot mode.
struct dshot_timing_us_t
{
    double bit_length_us;
    double t1h_lenght_us;
};

// Timing parameters for each DShot mode
static constexpr dshot_timing_us_t DSHOT_TIMING_US[] = {
    {0.00, 0.00}, // DSHOT_OFF
    {6.67, 5.00}, // DSHOT150
    {3.33, 2.50}, // DSHOT300
    {1.67, 1.25}, // DSHOT600
    {0.83, 0.67}  // DSHOT1200
};

// --- RMT Clock & Buffer Configuration ---
static constexpr auto DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
static constexpr auto DSHOT_RMT_RESOLUTION = 8000000;
static constexpr auto RMT_TICKS_PER_US = DSHOT_RMT_RESOLUTION / 1000000;
static constexpr auto RMT_TX_BUFFER_SYMBOLS = 64;
static constexpr auto RMT_RX_BUFFER_SYMBOLS = DSHOT_TELEMETRY_FULL_GCR_BITS;
static constexpr auto RMT_QUEUE_DEPTH = 1;

// --- RMT Receiver Configuration ---
// Tolerance for the RMT receiver's pulse width detection.
static constexpr float PULSE_TIMING_TOLERANCE_PERCENT = 0.20f;

// Stores pre-calculated timing values in RMT ticks for efficient signal generation.
struct rmt_ticks_t
{
    uint16_t bit_length_ticks;
    uint16_t t1h_ticks;
    uint16_t t1l_ticks;
    uint16_t t0h_ticks;
    uint16_t t0l_ticks;
};

// --- DShot Modes ---
enum dshot_mode_t : uint8_t
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
};

// --- DShot Packet Structure ---
struct dshot_packet_t
{
    uint16_t throttle_value : 11;
    bool telemetric_request : 1;
    uint16_t checksum : 4;
};

// --- Telemetry Data Structure ---
struct dshot_telemetry_data_t
{
    uint16_t rpm;
    uint16_t voltage;
    uint16_t current;
    uint16_t consumption;
    int8_t temperature;
    uint8_t errors;
};

// --- Library Result Codes & Structure ---
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
    DSHOT_TELEMETRY_DATA_AVAILABLE,
    DSHOT_COMMAND_SUCCESS
};

// Contains the success status, an error code, and optional telemetry data.
struct dshot_result_t
{
    bool success;
    dshot_msg_code_t result_code;
    uint16_t erpm;
    uint16_t motor_rpm;
    dshot_telemetry_data_t telemetry_data;
    bool telemetry_available;

    static dshot_result_t create_success(dshot_msg_code_t code, uint16_t erpm = 0, uint16_t motor_rpm = 0, dshot_telemetry_data_t telemetry = {}, bool telemetry_available = false)
    {
        return {true, code, erpm, motor_rpm, telemetry, telemetry_available};
    }

    static dshot_result_t create_error(dshot_msg_code_t code)
    {
        return {false, code, 0, 0, {}, false};
    }
};

// --- DShot Commands ---
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
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31
};

// --- General Status & Helper Constants ---
static constexpr int DSHOT_OK = 0;
static constexpr float CONVERSION_FACTOR_MILLI_TO_UNITS = 1000.0f;
