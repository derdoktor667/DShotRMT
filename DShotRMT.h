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
static constexpr auto DSHOT_THROTTLE_FAILSAFE = 0;
static constexpr auto DSHOT_THROTTLE_MIN = 48;
static constexpr auto DSHOT_THROTTLE_MAX = 2047;

static constexpr auto DSHOT_BITS_PER_FRAME = 16;
static constexpr auto DSHOT_SWITCH_TIME = 30; // 30us
static constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;

// --- RMT Config Constants ---
static constexpr auto DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
static constexpr auto DSHOT_RMT_RESOLUTION = 10 * 1000 * 1000; // 10 MHz
static constexpr auto TX_BUFFER_SIZE = DSHOT_BITS_PER_FRAME;
static constexpr auto RX_BUFFER_SIZE = 64; // debug
static constexpr auto DSHOT_SYMBOLS_SIZE = 64;

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
    uint16_t ticks_one_low;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
} dshot_timing_t;

// Some typedef magic for DShot timing config
extern const dshot_timing_t DSHOT_TIMINGS[];

// --- DShotRMT Class ---
class DShotRMT
{
public:
    // --- DShot Config ---
    explicit DShotRMT(gpio_num_t gpio = GPIO_NUM_16, dshot_mode_t mode = DSHOT300, bool is_bidirectional = false);
    DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional);

    // --- Init RMT Module ---
    bool begin();

    // Sets the throttle value and transmits
    [[deprecated("Use sendThrottle() instead")]]
    bool setThrottle(uint16_t throttle);

    bool sendThrottle(uint16_t throttle);

    // Sends a DShot Command
    bool sendDShotCommand(uint16_t command); // deprecated
    bool sendCommand(uint16_t command);

    // Gets eRPM from ESC telemetry
    uint16_t getERPM();

    // Converts eRPM to motor RPM
    uint32_t getMotorRPM(uint8_t magnet_count);

    // Returns pin number
    uint16_t getGPIO() const { return _gpio; }

    // Debug: returns "raw" Dshot packet sent by RMT
    uint16_t getDShotPacket() const { return _current_packet; }

    //
    bool is_bidirectional() const { return _is_bidirectional; }

private:
    // --- Config ---
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _is_bidirectional;
    uint16_t _frame_timer_us;

    // --- DShot Timings ---
    const dshot_timing_t &_timing_config;

    // --- RMT Handles ---
    rmt_channel_handle_t _rmt_tx_channel;
    rmt_channel_handle_t _rmt_rx_channel;
    rmt_encoder_handle_t _dshot_encoder;

    // --- RMT Config ---
    rmt_symbol_word_t _tx_symbols[DSHOT_BITS_PER_FRAME];
    rmt_symbol_word_t _rx_symbols[RX_BUFFER_SIZE];
    rmt_tx_channel_config_t _tx_channel_config;
    rmt_rx_channel_config_t _rx_channel_config;
    rmt_transmit_config_t _transmit_config;
    rmt_receive_config_t _receive_config;

    // --- Buffers ---
    uint16_t _last_erpm;
    uint16_t _current_packet;
    dshot_packet_t _packet;
    uint32_t _last_transmission_time;

    // ---Helpers ---
    bool _initTXChannel();
    bool _initRXChannel();
    bool _initDShotEncoder();

    bool _sendDShotFrame(const dshot_packet_t &packet);
    uint16_t _calculateCRC(const dshot_packet_t &packet);
    dshot_packet_t _buildDShotPacket(const uint16_t value);
    uint16_t _parseDShotPacket(const dshot_packet_t &packet);
    bool IRAM_ATTR _encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols);
    uint16_t _decodeDShotFrame(const rmt_symbol_word_t *symbols);

    // --- Simple Timer ---
    bool IRAM_ATTR _timer_signal();
    bool _timer_reset();

    // --- Error Handling ---
    static constexpr auto DSHOT_OK = 0;
    static constexpr auto DSHOT_ERROR = 1;
    static constexpr auto *DSHOT_MSG_01 = "Failed to initialize TX channel!";
    static constexpr auto *DSHOT_MSG_02 = "Failed to initialize RX channel!";
    static constexpr auto *DSHOT_MSG_03 = "Failed to initialize DShot encoder!";
    static constexpr auto *DSHOT_MSG_04 = "RX CRC Check failed!";
    static constexpr auto *DSHOT_MSG_05 = "Throttle value not in range (48 - 2047)!";
    static constexpr auto *DSHOT_MSG_06 = "Not a valid DShot Command (0 - 47)!";
    static constexpr auto *DSHOT_MSG_07 = "Bidirectional DShot support not enabled!";
    static constexpr auto *DSHOT_MSG_08 = "RX RMT module failure!";
};
