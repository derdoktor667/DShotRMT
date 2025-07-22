/**
 * @file DShotRMT.h
 * @brief DShot signal generation using ESP32 RMT with continuous repeat and pause between frames, including BiDirectional support
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
static constexpr uint16_t DSHOT_THROTTLE_FAILSAVE = 0;
static constexpr uint16_t DSHOT_THROTTLE_MIN = 48;
static constexpr uint16_t DSHOT_THROTTLE_MAX = 2047;
static constexpr uint8_t DSHOT_BITS_PER_FRAME = 16;

static constexpr uint16_t DSHOT_NULL_PACKET = 0x0000;
static constexpr uint16_t DSHOT_FULL_PACKET = 0xFFFF;
static constexpr uint16_t NO_ERPM_SIGNAL = 0;

// RMT configuration parameters
static constexpr rmt_clock_source_t DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
static constexpr uint32_t DSHOT_RMT_RESOLUTION = 10 * 1000 * 1000; // 10 MHz Clock

static constexpr size_t TX_BUFFER_SIZE = DSHOT_BITS_PER_FRAME;
static constexpr size_t RX_BUFFER_SIZE = 32; // Padding for RX decoding

// DShot Packet structure
typedef struct
{
    uint16_t throttle_value : 11;
    bool telemetric_request : 1;
    uint16_t checksum : 4;
} dshot_packet_t;

// --- DShot Mode Selection ---
typedef enum
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// --- DShotRMT Class ---
class DShotRMT
{
public:
    // Constructor: initializes configuration state
    DShotRMT(gpio_num_t gpio, dshot_mode_t mode = DSHOT300, bool isBidirectional = false, uint8_t pauseDuration = 120);

    // Initializes the RMT TX and RX channels
    void begin();

    // Sets a new throttle value (48-2047) and sends it
    void setThrottle(uint16_t throttle);

    // Receives and decodes the latest value from ESC, if available
    uint32_t getERPM();
    uint32_t getMotorRPM(uint8_t magnet_count);

    // Accessors for GPIO and DShot settings
    gpio_num_t getGPIO() const { return _gpio; }
    dshot_mode_t getDShotMode() const { return _mode; }
    uint8_t getPauseDuration() const { return _pauseDuration; }
    void setPauseDuration(uint8_t pauseDuration) { _pauseDuration = pauseDuration; }

private:
    // Calculates the checksum for a DShot packet
    uint16_t calculateCRC(uint16_t dshot_packet);

    // Assembles DShot packet (11 bit throttle + 1 bit telemetry request + 4 bit CRC)
    uint16_t assambleDShotPaket(uint16_t value);

    // Converts a 16-bit DShot packet into RMT symbols
    void encodeDShotTX(uint16_t dshot_packet, rmt_symbol_word_t *symbols, size_t &count);

    // Decodes the ESC answer
    uint16_t decodeDShotRX(const rmt_symbol_word_t *symbols, uint32_t count);

    // --- Configuration Parameters ---
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _isBidirectional;
    uint8_t _pauseDuration;

    // --- DShot Packets Container ---
    uint16_t _lastThrottle = DSHOT_NULL_PACKET;
    uint16_t _rx_packet = DSHOT_NULL_PACKET;
    uint16_t _tx_packet = DSHOT_NULL_PACKET;
    uint8_t _packet_crc = 0;

    // --- RMT Channel Handles ---
    rmt_channel_handle_t _rmt_rx_channel = nullptr;
    rmt_channel_handle_t _rmt_tx_channel = nullptr;
    rmt_rx_channel_config_t _rmt_rx_channel_config = {};
    rmt_tx_channel_config_t _rmt_tx_channel_config = {};

    // --- DShot RMT Encoder ---
    rmt_encoder_handle_t _dshot_encoder = nullptr;

    // --- RMT Configuration ---
    rmt_receive_config_t _receive_config = {};
    rmt_transmit_config_t _transmit_config = {};

    // --- RMT Symbol Buffers ---
    rmt_symbol_word_t _rx_symbols[RX_BUFFER_SIZE] = {};
    rmt_symbol_word_t _tx_symbols[TX_BUFFER_SIZE] = {};

    // Stores the last valid eRPM received from the ESC
    uint16_t _last_erpm = 0;
};
