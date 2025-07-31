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

static constexpr bool DSHOT_OK = 0;
static constexpr bool DSHOT_ERROR = 1;

// --- DShot Protocol Constants ---
static constexpr uint16_t DSHOT_THROTTLE_FAILSAFE = 0;
static constexpr uint16_t DSHOT_THROTTLE_MIN = 48;
static constexpr uint16_t DSHOT_THROTTLE_MAX = 2047;

static constexpr uint8_t DSHOT_BITS_PER_FRAME = 16;
static constexpr uint8_t DSHOT_SWITCH_TIME = 21;
static constexpr uint16_t DSHOT_NULL_PACKET = 0b0000000000000000;

// --- RMT Config Constants ---
static constexpr rmt_clock_source_t DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
static constexpr uint32_t DSHOT_RMT_RESOLUTION = 10 * 1000 * 1000; // 10 MHz
static constexpr size_t TX_BUFFER_SIZE = DSHOT_BITS_PER_FRAME;
static constexpr size_t RX_BUFFER_SIZE = 32;
static constexpr size_t DSHOT_SYMBOLS_SIZE = 64;

// --- DShot Mode Select ---
typedef enum
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// --- DShot Packet Structure ---
typedef struct
{
    uint16_t throttle_value : 11;
    bool telemetric_request : 1;
    uint16_t checksum : 4;
} dshot_packet_t;

// --- DShot Timing Config ---
typedef struct
{
    uint16_t frame_length_us;
    uint16_t ticks_per_bit;
    uint16_t ticks_one_high;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
    uint16_t ticks_one_low;
} dshot_timing_t;

// --- DShot Timing Config ---
extern const dshot_timing_t DSHOT_TIMINGS[];

class DShotRMT
{
public:
    //
    DShotRMT(gpio_num_t gpio, dshot_mode_t mode = DSHOT300, bool is_bidirectional = false);

    // --- Init RMT Module ---
    bool begin();

    // Sets the throttle value and transmits
    bool setThrottle(uint16_t throttle);

    // Sends a valid DShot Command
    bool sendDShotCommand(uint16_t command);

    // Gets eRPM from ESC telemetry
    uint32_t getERPM();

    // Converts eRPM to motor RPM
    uint32_t getMotorRPM(uint8_t magnet_count);

    //
    gpio_num_t getGPIO() const { return _gpio; }
    dshot_mode_t getDShotMode() const { return _mode; }
    bool isBidirectional() const { return _is_bidirectional; }

private:
    // --- Config ---
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _is_bidirectional;
    uint32_t _frame_time_us;

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
    rmt_symbol_word_t _rx_symbols[RX_BUFFER_SIZE];
    uint16_t _last_erpm;
    unsigned long _last_transmission_time;

    // 
    bool _initTXChannel();
    bool _initRXChannel();
    bool _initDShotEncoder();

    uint16_t _calculateCRC(const dshot_packet_t &packet);
    uint16_t _assembleDShotFrame(const dshot_packet_t &packet);
    bool _encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols);
    uint16_t _decodeDShotFrame(const rmt_symbol_word_t *symbols, size_t symbol_count);
    bool _sendDShotFrame(const dshot_packet_t &packet);

    bool _timer_signal();
    void _timer_reset();
};
