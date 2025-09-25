/**
 * @file DShotRMT.h
 * @brief Optimized DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-09-18
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <dshot_commands.h>
#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <atomic>

// DShot Protocol Constants & Types
static constexpr auto DSHOT_THROTTLE_FAILSAFE = 0;
static constexpr auto DSHOT_THROTTLE_MIN = 48;
static constexpr auto DSHOT_THROTTLE_MAX = 2047;
static constexpr auto DSHOT_BITS_PER_FRAME = 16;
static constexpr auto DEFAULT_MOTOR_MAGNET_COUNT = 14;

// Custom status codes
static constexpr auto DSHOT_OK = 0;
static constexpr auto DSHOT_ERROR = 1;

// DShot Modes
typedef enum dshot_mode
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// DShot Packet Structure
typedef struct dshot_packet
{
    uint16_t throttle_value : 11;
    bool telemetric_request : 1;
    uint16_t checksum : 4;
} dshot_packet_t;

// DShot Timing Configuration
typedef struct dshot_timing
{
    double bit_length_us;
    double t1h_lenght_us;
} dshot_timing_us_t;

// RMT Timing Configuration
typedef struct rmt_ticks
{
    uint16_t bit_length_ticks;
    uint16_t t1h_ticks;
    uint16_t t1l_ticks;
    uint16_t t0h_ticks;
    uint16_t t0l_ticks;
} rmt_ticks_t;

// Unified DShot Result Structure
typedef struct dshot_result
{
    bool success;
    const char *msg;
    uint16_t erpm;
    uint16_t motor_rpm;
} dshot_result_t;

// Command Type Alias
typedef dshotCommands_e dshot_commands_t;

// Helper Functions
void printDShotResult(dshot_result_t &result, Stream &output = Serial);

//
// DShotRMT Main Class
class DShotRMT
{
public:
    // Constructors & Destructor
    explicit DShotRMT(gpio_num_t gpio = GPIO_NUM_16, dshot_mode_t mode = DSHOT300, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    ~DShotRMT();

    // Public Core Functions
    dshot_result_t begin();
    dshot_result_t sendThrottle(uint16_t throttle);
    dshot_result_t sendThrottlePercent(float percent);
    dshot_result_t sendCommand(uint16_t command);
    dshot_result_t sendCommand(dshot_commands_t dshot_command, uint16_t repeat_count = DEFAULT_CMD_REPEAT_COUNT, uint16_t delay_us = DEFAULT_CMD_DELAY_US);
    dshot_result_t getTelemetry(uint16_t magnet_count = 0);
    dshot_result_t getESCInfo();
    dshot_result_t setMotorSpinDirection(bool reversed);
    dshot_result_t saveESCSettings();

    // Public Utility & Info Functions
    void setMotorMagnetCount(uint16_t magnet_count);
    void printDShotInfo(Stream &output = Serial) const;
    void printCpuInfo(Stream &output = Serial) const;

    // Deprecated Methods
    [[deprecated("Use sendThrottle() instead")]]
    bool setThrottle(uint16_t throttle)
    {
        auto result = sendThrottle(throttle);
        return result.success;
    }

    [[deprecated("Use sendCommand() instead")]]
    bool sendDShotCommand(uint16_t command)
    {
        auto result = sendCommand(command);
        return result.success;
    }

    [[deprecated("Use getTelemetry() instead")]]
    uint32_t getMotorRPM(uint8_t magnet_count)
    {
        auto result = getTelemetry(magnet_count);
        return result.motor_rpm;
    }

private:
    // Configuration Constants
    static constexpr auto const DSHOT_NULL_PACKET = 0b0000000000000000;
    static constexpr auto const DSHOT_FULL_PACKET = 0b1111111111111111;
    static constexpr auto const DSHOT_CRC_MASK = 0b0000000000001111;
    static constexpr auto const DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
    static constexpr auto const DSHOT_RMT_RESOLUTION = 8 * 1000 * 1000;                      // 8 MHz resolution
    static constexpr auto const RMT_TICKS_PER_US = DSHOT_RMT_RESOLUTION / (1 * 1000 * 1000); // RMT Ticks per microsecond
    static constexpr auto const DSHOT_RX_TIMEOUT_MS = 2;
    static constexpr auto const DSHOT_PADDING_US = 20; // Add to pause between frames for compatibility
    static constexpr auto const RMT_BUFFER_SYMBOLS = 128;
    static constexpr auto const RMT_QUEUE_DEPTH = 1;
    static constexpr auto const GCR_BITS_PER_FRAME = 21; // Number of GCR bits in a DShot answer frame
    static constexpr auto const POLE_PAIRS_MIN = 1;
    static constexpr auto const MAGNETS_PER_POLE_PAIR = 2;
    static constexpr auto const NO_DSHOT_TELEMETRY = 0;
    static constexpr auto const DSHOT_PULSE_MIN = 800;  // 0.8us minimum pulse
    static constexpr auto const DSHOT_PULSE_MAX = 8000; // 8.0us maximum pulse
    static constexpr auto const DSHOT_TELEMETRY_INVALID = DSHOT_THROTTLE_MAX;

    // Error Messages
    static constexpr char const *NONE = "";
    static constexpr char const *UNKNOWN_ERROR = "Unknown Error!";
    static constexpr char const *INIT_SUCCESS = "SignalGeneratorRMT initialized successfully";
    static constexpr char const *INIT_FAILED = "SignalGeneratorRMT init failed!";
    static constexpr char const *TX_INIT_SUCCESS = "TX RMT channel initialized successfully";
    static constexpr char const *TX_INIT_FAILED = "TX RMT channel init failed!";
    static constexpr char const *RX_INIT_SUCCESS = "RX RMT channel initialized successfully";
    static constexpr char const *RX_INIT_FAILED = "RX RMT channel init failed!";
    static constexpr char const *ENCODER_INIT_SUCCESS = "RMT encoder initialized successfully";
    static constexpr char const *ENCODER_INIT_FAILED = "RMT encoder init failed!";
    static constexpr char const *ENCODING_SUCCESS = "Packet encoded successfully";
    static constexpr char const *TRANSMISSION_SUCCESS = "Transmission successfully";
    static constexpr char const *TRANSMISSION_FAILED = "Transmission failed!";
    static constexpr char const *RECEIVER_FAILED = "RMT receiver failed!";
    static constexpr char const *THROTTLE_NOT_IN_RANGE = "Throttle not in range! (48 - 2047)";
    static constexpr char const *PERCENT_NOT_IN_RANGE = "Percent not in range! (0.0 - 100.0)";
    static constexpr char const *COMMAND_NOT_VALID = "Command not valid! (0 - 47)";
    static constexpr char const *BIDIR_NOT_ENABLED = "Bidirectional DShot not enabled!";
    static constexpr char const *TELEMETRY_SUCCESS = "Valid Telemetric Frame received!";
    static constexpr char const *TELEMETRY_FAILED = "No valid Telemetric Frame received!";
    static constexpr char const *INVALID_MAGNET_COUNT = "Invalid motor magnet count!";
    static constexpr char const *TIMING_CORRECTION = "Timing correction!";
    static constexpr char const *CALLBACK_REGISTERING_FAILED = "RMT RX Callback registering failed!";
    static constexpr char const *INVALID_COMMAND = "Invalid command!";
    static constexpr char const *COMMAND_SUCCESS = "DShot command sent successfully";

    // --- UTILITY METHODS ---
    bool _isValidCommand(dshot_commands_t command);
    dshot_result_t _executeCommand(dshot_commands_t command);

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
    uint8_t _bitPositions[DSHOT_BITS_PER_FRAME];
    uint16_t _level0; // Signal level for the first part of a pulse (always HIGH for DShot)
    uint16_t _level1; // Signal level for the second part of a pulse (always LOW for DShot)

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
    uint16_t _parseDShotPacket(const dshot_packet_t &packet);
    uint16_t _calculateCRC(const uint16_t &data);
    void _preCalculateRMTTicks();
    void _preCalculateBitPositions();

    // Private Frame Processing Functions
    dshot_result_t _sendDShotFrame(const dshot_packet_t &packet);
    dshot_result_t _encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols);
    uint16_t _decodeDShotFrame(const rmt_symbol_word_t *symbols);

    // Private Timing Control Functions
    bool _timer_signal();
    bool _timer_reset();

    // Static Callback Functions
    static bool _on_rx_done(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data);

    // Command Constants
    static constexpr auto DEFAULT_CMD_DELAY_US = 10;
    static constexpr auto DEFAULT_CMD_REPEAT_COUNT = 1;
    static constexpr auto SETTINGS_COMMAND_REPEATS = 10; // Settings commands need 10 repeats
    static constexpr auto SETTINGS_COMMAND_DELAY_US = 5;
};
