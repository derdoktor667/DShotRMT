/**
 * @file DShotRMT.h
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <dshot_commands.h>
#include <web_content.h>
#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>

// DShot Protocol Constants
static constexpr auto DSHOT_THROTTLE_FAILSAFE = 0;
static constexpr auto DSHOT_THROTTLE_MIN = 48;
static constexpr auto DSHOT_THROTTLE_MAX = 2047;
static constexpr auto DSHOT_BITS_PER_FRAME = 16;
static constexpr auto DSHOT_PAUSE_US = 30; // Additional frame pause time
static constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;
static constexpr auto DSHOT_FULL_PACKET = 0b1111111111111111;
static constexpr auto DSHOT_CRC_MASK = 0b0000000000001111;
static constexpr auto DSHOT_RX_TIMEOUT_MS = 2; // Never reached, just a timeeout
static constexpr auto GCR_BITS_PER_FRAME = 21; // Number of GCR bits in a DShot answer frame (1 start + 16 data + 4 CRC)
static constexpr auto DEFAULT_MOTOR_MAGNET_COUNT = 14;
static constexpr auto MAGNETS_PER_POLE_PAIR = 2;
static constexpr auto MIN_POLE_PAIRS = 1;
static constexpr auto NO_DSHOT_ERPM = 0;
static constexpr auto NO_DSHOT_RPM = 0;

// RMT Configuration Constants
constexpr auto DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
constexpr auto DSHOT_RMT_RESOLUTION = 10 * 1000 * 1000; // 10 MHz resolution
constexpr auto RMT_BUFFER_SIZE = DSHOT_BITS_PER_FRAME;
constexpr auto RMT_BUFFER_SYMBOLS = 64;
constexpr auto RMT_QUEUE_DEPTH = 1;

// Smallest pulse for DShot1200 is 2us. Largest for DShot150 is 40us.
// The range is set from 3us (3000ns) to 60us (60000ns) to be safe across all modes.
constexpr uint32_t DSHOT_PULSE_MIN = 3000;
constexpr uint32_t DSHOT_PULSE_MAX = 60000;

// DShot Modes
typedef enum
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// DShot Packet
typedef struct
{
    uint16_t throttle_value : 11;
    bool telemetric_request : 1;
    uint16_t checksum : 4;
} dshot_packet_t;

// DShot Timing Configuration
typedef struct
{
    uint32_t frame_length_us;
    uint16_t ticks_per_bit;
    uint16_t ticks_one_high;
    uint16_t ticks_one_low;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
} dshot_timing_t;

// Unified DShot result structure
typedef struct
{
    bool success;
    const char *msg;
    uint16_t erpm; 
    uint16_t motor_rpm; 
} dshot_result_t;

// Naming convention
typedef dshotCommands_e dshot_commands_t;

// --- HELPERS ---
void printDShotResult(dshot_result_t &result, Stream &output = Serial);

//
class DShotRMT
{
public:
    // Constructor with GPIO enum
    explicit DShotRMT(gpio_num_t gpio = GPIO_NUM_16, dshot_mode_t mode = DSHOT300, bool is_bidirectional = false);

    // Constructor with pin number
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional);

    // Destructor for "better" code
    ~DShotRMT();

    // Initialize the RMT module and DShot config
    dshot_result_t begin();

    // Send throttle value (48-2047)
    dshot_result_t sendThrottle(uint16_t throttle);

    // Send DShot command (0-47)
    dshot_result_t sendCommand(uint16_t command);

    // --- GETTERS ---
    gpio_num_t getGPIO() const { return _gpio; }
    uint16_t getDShotPacket() const { return _parsed_packet; }
    bool is_bidirectional() const { return _is_bidirectional; }
    dshot_mode_t getMode() const { return _mode; }
    dshot_result_t getTelemetry(uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    // --- INFO ---
    void printDShotInfo(Stream &output = Serial) const;
    void printCpuInfo(Stream &output = Serial) const;

    // --- DEPRECATED METHODS ---
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
    // --- CONFIG ---
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _is_bidirectional;
    uint32_t _frame_timer_us;
    const dshot_timing_t &_timing_config;
    uint16_t _last_throttle;

    // --- TIMING & PACKET VARIABLES ---
    uint64_t _last_transmission_time;
    uint16_t _parsed_packet;
    dshot_packet_t _packet;
    uint8_t _bitPositions[DSHOT_BITS_PER_FRAME];
    uint16_t _level0;
    uint16_t _level1;

    // --- RMT HARDWARE HANDLES ---
    rmt_channel_handle_t _rmt_tx_channel;
    rmt_channel_handle_t _rmt_rx_channel;
    rmt_encoder_handle_t _dshot_encoder;

    // --- RMT CONFIG STRUCTURES ---
    rmt_tx_channel_config_t _tx_channel_config;
    rmt_rx_channel_config_t _rx_channel_config;
    rmt_transmit_config_t _transmit_config;
    rmt_receive_config_t _receive_config;

    // --- INITS ---
    dshot_result_t _initTXChannel();
    dshot_result_t _initRXChannel();
    dshot_result_t _initDShotEncoder();

    // --- PACKET MANAGEMENT ---
    dshot_packet_t _buildDShotPacket(const uint16_t value);
    uint16_t _parseDShotPacket(const dshot_packet_t &packet);
    uint16_t _calculateCRC(const uint16_t data);
    void _preCalculateBitPositions();

    // --- FRAME PROCESSING ---
    dshot_result_t _sendDShotFrame(const dshot_packet_t &packet);
    bool IRAM_ATTR _encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols);
    uint16_t _decodeDShotFrame(const rmt_symbol_word_t *symbols);

    // --- TIMING CONTROL ---
    bool IRAM_ATTR _timer_signal();
    bool _timer_reset();

    // -- CALLBACKS ---
    rmt_rx_event_callbacks_t _rx_event_callbacks;
    volatile rmt_symbol_word_t _rx_symbols_direct[GCR_BITS_PER_FRAME];
    volatile uint16_t _last_erpm_atomic;
    volatile bool _telemetry_ready_flag;
    static bool IRAM_ATTR _rmt_rx_done_callback(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data);

    // --- DSHOT DEFAULTS ---
    static constexpr auto const DSHOT_TELEMETRY_INVALID = (0xffff);

    // --- CONSTANTS & ERROR MESSAGES ---
    static constexpr bool DSHOT_OK = 0;
    static constexpr bool DSHOT_ERROR = 1;

    static constexpr char const *NONE = "";
    static constexpr char const *UNKNOWN_ERROR = "Unknown Error!";
    static constexpr char const *INIT_SUCCESS = "SignalGeneratorRMT initialized successfully";
    static constexpr char const *INIT_FAILED = "SignalGeneratorRMT init failed!";
    static constexpr char const *TX_INIT_SUCCESS = "TX RMT channel initialized successfully";
    static constexpr char const *TX_INIT_FAILED = "TX RMT channel init failed!";
    static constexpr char const *RX_INIT_SUCCESS = "RX RMT channel initialized successfully";
    static constexpr char const *RX_INIT_FAILED = "RX RMT channel init failed!";
    static constexpr char const *RX_BUFFER_FAILED = "RX RMT buffer init failed!";
    static constexpr char const *ENCODER_INIT_SUCCESS = "RMT encoder initialized successfully";
    static constexpr char const *ENCODER_INIT_FAILED = "RMT encoder init failed!";
    static constexpr char const *TRANSMISSION_SUCCESS = "Transmission successfully";
    static constexpr char const *TRANSMISSION_FAILED = "Transmission failed!";
    static constexpr char const *RECEIVER_FAILED = "RMT receiver failed!";
    static constexpr char const *THROTTLE_NOT_IN_RANGE = "Throttle not in range! (48 - 2047)";
    static constexpr char const *COMMAND_NOT_VALID = "Command not valid! (0 - 47)";
    static constexpr char const *BIDIR_NOT_ENABLED = "Bidirectional DShot not enabled!";
    static constexpr char const *TELEMETRY_SUCCESS = "Valid Telemetric Frame received!";
    static constexpr char const *TELEMETRY_FAILED = "No valid Telemetric Frame received!";
    static constexpr char const *INVALID_MAGNET_COUNT = "Invalid motor magnet count!";
    static constexpr char const *TIMING_CORRECTION = "Timing correction!";
};
