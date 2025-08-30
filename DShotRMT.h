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
constexpr auto DSHOT_SWITCH_TIME = 30;
constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;

// RMT Configuration Constants
constexpr auto DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
constexpr auto DSHOT_RMT_RESOLUTION = 10 * 1000 * 1000;       // 10 MHz resolution
constexpr auto RMT_BUFFER_SIZE = DSHOT_BITS_PER_FRAME;

constexpr auto RX_BUFFER_SIZE = 128;
constexpr auto TX_BUFFER_SIZE = 64;

// DShot Mode
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
    uint16_t throttle_value : 11;    // 11-bit throttle value
    uint16_t telemetric_request : 1; // Telemetry request bit
    uint16_t checksum : 4;           // 4-bit CRC checksum
} dshot_packet_t;

// DShot Timing Config Structure
typedef struct dshot_timing_s
{
    uint32_t frame_length_us;
    uint16_t ticks_per_bit;
    uint16_t ticks_one_high;
    uint16_t ticks_one_low;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
} dshot_timing_t;

// Timing config for DShot modes
extern const dshot_timing_t DSHOT_TIMINGS[];

// DShotRMT Class
class DShotRMT
{
public:
    // Primary constructor with GPIO enum
    explicit DShotRMT(gpio_num_t gpio = GPIO_NUM_16,
                      dshot_mode_t mode = DSHOT300,
                      bool is_bidirectional = false);

    // Alternative constructor with pin number
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional);

    // Initialize the RMT module and DShot config
    uint16_t begin();

    // Send throttle value (48-2047)
    [[deprecated("Use sendThrottle() instead")]]
    bool setThrottle(uint16_t throttle) { return sendThrottle(throttle); };
    bool sendThrottle(uint16_t throttle);

    // Send DShot command (0-47)
    [[deprecated("Use sendCommand() instead")]]
    bool sendDShotCommand(uint16_t command) { return sendCommand(command); };
    bool sendCommand(uint16_t command);

    // Get telemetry data (bidirectional mode only)
    uint16_t getERPM();
    uint32_t getMotorRPM(uint8_t magnet_count); // Convert eRPM to motor RPM

    // Tools
    gpio_num_t getGPIO() const { return _gpio; }                    // Get GPIO pin
    uint16_t getDShotPacket() const { return _parsed_packet; }     // Get raw packet
    bool is_bidirectional() const { return _is_bidirectional; }     // Check if bidirectional

    // Print DShot Info
    void printDshotInfo(Stream &output = Serial0) const;
    void printCpuInfo(Stream &output = Serial0) const;

    // Prints debug values stream
    void printDebugStream(Stream &output = Serial0) const;

private:
    // Configuration Variables
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    uint16_t _is_bidirectional;
    uint32_t _frame_timer_us;
    const dshot_timing_t &_timing_config;

    // RMT Config
    rmt_channel_handle_t _rmt_tx_channel;
    rmt_channel_handle_t _rmt_rx_channel;
    rmt_encoder_handle_t _dshot_encoder;

    // RMT Config Structures
    rmt_symbol_word_t _tx_symbols[TX_BUFFER_SIZE];
    rmt_symbol_word_t _rx_symbols[RX_BUFFER_SIZE];
    rmt_tx_channel_config_t _tx_channel_config;
    rmt_rx_channel_config_t _rx_channel_config;
    rmt_transmit_config_t _transmit_config;
    rmt_receive_config_t _receive_config;

    //
    uint16_t _last_erpm;
    uint16_t _parsed_packet;
    dshot_packet_t _packet;
    uint32_t _last_transmission_time;

    // Helpers
    bool _initTXChannel();
    bool _initRXChannel();
    bool _initDShotEncoder();

    // Utilizing RMT
    uint16_t _sendDShotFrame(const dshot_packet_t &packet);

    // Packet management
    uint16_t _calculateCRC(const dshot_packet_t &packet);
    dshot_packet_t _buildDShotPacket(const uint16_t value);
    uint16_t _parseDShotPacket(const dshot_packet_t &packet);

    // Frame processing
    bool IRAM_ATTR _encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols);
    uint16_t _decodeDShotFrame(const rmt_symbol_word_t *symbols);

    // Timer Config
    bool IRAM_ATTR _timer_signal();
    bool _timer_reset();

    // DShot Messages
    void _dshot_log(char *msg, Stream &output = Serial0) { output.println(msg); };

    // Error Codes and Messages
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
};
