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
#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>

// DShot Protocol Constants
constexpr auto DSHOT_THROTTLE_FAILSAFE = 0;
constexpr auto DSHOT_THROTTLE_MIN = 48;
constexpr auto DSHOT_THROTTLE_MAX = 2047;
constexpr auto DSHOT_BITS_PER_FRAME = 16;
constexpr auto DSHOT_SWITCH_TIME = 30; // Time in us between TX and RX
constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;
constexpr auto DSHOT_RX_TIMEOUT_MS = 2;

// RMT Configuration Constants
constexpr auto DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
constexpr auto DSHOT_RMT_RESOLUTION = 10 * 1000 * 1000; // 10 MHz resolution
constexpr auto RMT_BUFFER_SIZE = DSHOT_BITS_PER_FRAME;
constexpr auto RMT_BUFFER_SYMBOLS = 64;
constexpr auto RMT_TRANSMIT_QUEUE_DEPTH = 1;

// Smallest pulse for DShot1200 is 2us. Largest for DShot150 is 40us.
// The range is set from 3us (3000ns) to 60us (60000ns) to be safe across all modes.
constexpr uint32_t DSHOT_PULSE_MIN = 3000;
constexpr uint32_t DSHOT_PULSE_MAX = 60000;

// DShot Mode Enumeration
typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// DShot Packet Structure
typedef struct dshot_packet_s
{
    uint16_t throttle_value : 11;
    uint16_t telemetric_request : 1;
    uint16_t checksum : 4;
} dshot_packet_t;

// DShot Timing Configuration Structure
typedef struct dshot_timing_s
{
    uint32_t frame_length_us;
    uint16_t ticks_per_bit;
    uint16_t ticks_one_high;
    uint16_t ticks_one_low;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
} dshot_timing_t;

//
class DShotRMT
{
public:
    // Primary constructor with GPIO enum
    explicit DShotRMT(gpio_num_t gpio = GPIO_NUM_16,
                      dshot_mode_t mode = DSHOT300,
                      bool is_bidirectional = false);

    // Constructor with pin number
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional);

    // Destructor for "better" code
    ~DShotRMT();

    // Initialize the RMT module and DShot config
    uint16_t begin();

    // Send throttle value (48-2047)
    bool sendThrottle(uint16_t throttle);

    // Send DShot command (0-47)
    bool sendCommand(uint16_t command);

    // Get telemetry data (bidirectional mode only)
    uint16_t getERPM();

    // Convert eRPM to motor RPM
    uint32_t getMotorRPM(uint8_t magnet_count);

    //
    gpio_num_t getGPIO() const { return _gpio; }
    uint16_t getDShotPacket() const { return _parsed_packet; }
    bool is_bidirectional() const { return _is_bidirectional; }

    // --- INFO ---
    void printDshotInfo(Stream &output = Serial0) const;
    void printCpuInfo(Stream &output = Serial0) const;

    // --- DEPRECATED METHODS ---
    [[deprecated("Use sendThrottle() instead")]]
    bool setThrottle(uint16_t throttle) { return sendThrottle(throttle); }
    
    [[deprecated("Use sendCommand() instead")]]
    bool sendDShotCommand(uint16_t command) { return sendCommand(command); }
    
private:
    // --- CONFIG ---
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _is_bidirectional;
    uint32_t _frame_timer_us;
    const dshot_timing_t &_timing_config;

    // --- TIMING & STATE VARIABLES ---
    uint32_t _last_transmission_time;
    uint16_t _last_erpm;
    uint16_t _parsed_packet;
    dshot_packet_t _packet;

    // --- RMT HARDWARE HANDLES ---
    rmt_channel_handle_t _rmt_tx_channel;
    rmt_channel_handle_t _rmt_rx_channel;
    rmt_encoder_handle_t _dshot_encoder;

    // --- RMT CONFIG STRUCTURES ---
    rmt_tx_channel_config_t _tx_channel_config;
    rmt_rx_channel_config_t _rx_channel_config;
    rmt_transmit_config_t _transmit_config;
    rmt_receive_config_t _receive_config;

    // --- RMT DATA BUFFERS ---
    rmt_symbol_word_t _tx_symbols[RMT_BUFFER_SYMBOLS];
    rmt_symbol_word_t _rx_symbols[RMT_BUFFER_SYMBOLS];

    // --- INITS ---
    bool _initTXChannel();
    bool _initRXChannel();
    bool _initDShotEncoder();

    // --- PACKET MANAGEMENT ---
    dshot_packet_t _buildDShotPacket(const uint16_t value);
    uint16_t _parseDShotPacket(const dshot_packet_t &packet);
    uint16_t _calculateCRC(const dshot_packet_t &packet);

    // --- FRAME PROCESSING ---
    uint16_t _sendDShotFrame(const dshot_packet_t &packet);
    bool IRAM_ATTR _encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols);
    uint16_t _decodeDShotFrame(const rmt_symbol_word_t *symbols);

    // --- TIMING CONTROL ---
    bool IRAM_ATTR _timer_signal();
    bool _timer_reset();

    // -- CALLBACKS ---
    QueueHandle_t _rx_queue;
    rmt_rx_event_callbacks_t _rx_event_cbs;
    static bool IRAM_ATTR _rmt_rx_done_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data);

    // --- ERROR HANDLING & LOGGING ---
    void _dshot_log(char *msg, Stream &output = Serial0) { output.println(msg); }

    // --- CONSTANTS & ERROR MESSAGES ---
    static constexpr uint16_t DSHOT_OK = 0;
    static constexpr uint16_t DSHOT_ERROR = 1;

    static constexpr char *NEW_LINE = " ";
    static constexpr char *TX_INIT_FAILED = "Failed to initialize TX channel!";
    static constexpr char *RX_INIT_FAILED = "Failed to initialize RX channel!";
    static constexpr char *ENCODER_INIT_FAILED = "Failed to initialize DShot encoder!";
    static constexpr char *CRC_CHECK_FAILED = "RX CRC Check failed!";
    static constexpr char *THROTTLE_NOT_IN_RANGE = "Throttle value not in range (48 - 2047)!";
    static constexpr char *COMMAND_NOT_VALID = "Not a valid DShot Command (0 - 47)!";
    static constexpr char *BIDIR_NOT_ENABLED = "Bidirectional DShot support not enabled!";
    static constexpr char *RX_RMT_MODULE_ERROR = "RX RMT module failure!";
    static constexpr char *RX_RMT_RECEIVER_ERROR = "RX RMT receiver failure!";
};
