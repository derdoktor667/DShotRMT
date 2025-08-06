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

// --- DShot Protocol Constants ---
static constexpr uint16_t DSHOT_THROTTLE_FAILSAFE = 0;
static constexpr uint16_t DSHOT_THROTTLE_MIN = 48;
static constexpr uint16_t DSHOT_THROTTLE_MAX = 2047;

static constexpr uint8_t DSHOT_BITS_PER_FRAME = 16;
static constexpr uint8_t DSHOT_SWITCH_TIME = 300; // 30us
static constexpr uint16_t DSHOT_NULL_PACKET = 0b0000000000000000;

// --- RMT Config Constants ---
static constexpr rmt_clock_source_t DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
static constexpr uint32_t DSHOT_RMT_RESOLUTION = 10 * 1000 * 1000; // 10 MHz
static constexpr size_t TX_BUFFER_SIZE = DSHOT_BITS_PER_FRAME;
static constexpr size_t RX_BUFFER_SIZE = 64; // debug
static constexpr size_t DSHOT_SYMBOLS_SIZE = 64;

// --- DShot Mode Select ---
typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// --- DShot Packet Structure ---
typedef struct dshot_packet_s
{
    uint16_t throttle_value : 11;
    bool telemetric_request : 1;
    uint16_t checksum : 4;
} dshot_packet_t;

// --- DShot Timing Config ---
typedef struct dshot_timing_s
{
    uint16_t frame_length_us;
    uint16_t ticks_per_bit;
    uint16_t ticks_one_high;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
    uint16_t ticks_one_low;
} dshot_timing_t;

extern const dshot_timing_t DSHOT_TIMINGS[];

// --- DShotRMT Class ---
class DShotRMT
{
public:
    // --- DShot Config ---
    DShotRMT(gpio_num_t gpio, dshot_mode_t mode = DSHOT300, bool is_bidirectional = false);
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional);

    // --- Init RMT Module ---
    bool begin();

    // Sets the throttle value and transmits
    bool setThrottle(uint16_t throttle);    // deprecated
    bool sendThrottle(uint16_t throttle);

    // Sends a DShot Command
    bool sendDShotCommand(uint16_t command);    // deprecated
    bool sendCommand(uint16_t command);

    // Gets eRPM from ESC telemetry
    uint16_t getERPM();

    // Converts eRPM to motor RPM
    uint32_t getMotorRPM(uint8_t magnet_count);

    // Returns GPIO Pin
    uint16_t getGPIO() const { return _gpio; }

    // Returns "raw" Dshot packet sent by RMT
    uint16_t getDShotPacket() { return _current_packet; }

    //
    bool is_bidirectional() const { return _is_bidirectional; }

private:
    // --- Config ---
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _is_bidirectional;
    uint16_t _frame_time_us;

    // --- DShot Timings ---
    const dshot_timing_t &_timing_config;

    // --- RMT Handles ---
    rmt_channel_handle_t _rmt_tx_channel;
    rmt_channel_handle_t _rmt_rx_channel;
    rmt_encoder_handle_t _dshot_encoder;

    // --- RMT Config ---
    rmt_tx_channel_config_t _tx_channel_config;
    rmt_rx_channel_config_t _rx_channel_config;
    rmt_transmit_config_t _transmit_config;
    rmt_receive_config_t _receive_config;

    // --- Buffers ---
    uint16_t _last_erpm;
    uint16_t _current_packet;
    dshot_packet_t _packet;
    unsigned long _last_transmission_time;

    //
    bool _initTXChannel();
    bool _initRXChannel();
    bool _initDShotEncoder();

    bool _sendDShotFrame(const dshot_packet_t &packet);
    uint16_t _calculateCRC(const dshot_packet_t &packet);
    dshot_packet_t _buildDShotPacket(const uint16_t value);
    uint16_t _parseDShotPacket(const dshot_packet_t &packet);
    bool IRAM_ATTR _encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols);
    uint16_t _decodeDShotFrame(const rmt_symbol_word_t *symbols);

    bool _timer_signal();
    bool _timer_reset();

    // Error Handling
    static constexpr bool DSHOT_OK = 0;
    static constexpr bool DSHOT_ERROR = 1;
    static constexpr char *DSHOT_MSG_01 = "Failed to initialize TX channel!";
    static constexpr char *DSHOT_MSG_02 = "Failed to initialize RX channe!l";
    static constexpr char *DSHOT_MSG_03 = "Failed to initialize encoder!";
    static constexpr char *DSHOT_MSG_04 = "RX CRC Check failed!";
    static constexpr char *DSHOT_MSG_06 = "Throttle value not in range (48 - 2047)!";
    static constexpr char *DSHOT_MSG_07 = "Not a valid DShot Command (0 - 47)!";
    static constexpr char *DSHOT_MSG_08 = "Bidirectional DShot support not enabled!";
    static constexpr char *DSHOT_MSG_09 = "RX RMT module failure!";
};
