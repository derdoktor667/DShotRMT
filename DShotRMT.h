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
#include <driver/rmt_types.h>

// --- DShot Protocol Constants ---
// Constants to define timing and encoding rules for DShot Protocol
static constexpr auto DSHOT_THROTTLE_FAILSAVE = 0;
static constexpr auto DSHOT_THROTTLE_MIN = 48;
static constexpr auto DSHOT_THROTTLE_MAX = 2047;
static constexpr auto DSHOT_BITS_PER_FRAME = 17;
static constexpr auto PAUSE_BITS = 21;

static constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;
static constexpr auto DSHOT_FULL_PACKET = 0b1111111111111111;
static constexpr auto NO_ERPM_SIGNAL = 0;

// RMT configuration parameters
static constexpr auto DSHOT_CLOCK_SRC_DEFAULT = RMT_CLK_SRC_DEFAULT;
static constexpr auto DSHOT_RMT_RESOLUTION = 10 * 1000 * 1000; // 10 MHz

static constexpr auto TX_BUFFER_SIZE = DSHOT_BITS_PER_FRAME;
static constexpr auto RX_BUFFER_SIZE = DSHOT_BITS_PER_FRAME + 4; // Padding for RX decoding

// --- DShot Mode Selection ---
// Select the appropriate bit timing for the protocol
typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// --- DShotRMT Class ---
// This class provides an abstraction for sending and optionally receiving DShot frames.
// It uses ESP32's RMT peripheral for precise timing control, including BiDirectional RX.
class DShotRMT
{
public:
    // Constructor: initializes configuration state
    DShotRMT(gpio_num_t gpio, dshot_mode_t mode = DSHOT300, bool isBidirectional = false);

    // Initializes the RMT TX and RX channels
    void begin();

    // Sets a new throttle value (48-2047) and sends it repeatedly
    void setThrottle(uint16_t throttle);

    // Receives and decodes the latest value from ESC, if available
    uint32_t getERPM();
    uint32_t getMotorRPM(uint8_t magnet_count);

    // Accessors for GPIO and DShot mode
    gpio_num_t getGPIO() const { return _gpio; }
    dshot_mode_t getDShotMode() const { return _mode; }

private:
    // Calculate the checksum for throttle value
    uint16_t calculateCRC(uint16_t dshot_packet);

    // Assamble DShot Paket (10 bit throttle + 1 bit telemetry request + 4 bit crc)
    uint16_t assambleDShotPaket(uint16_t value);

    // Converts a 16-bit DShot packet into RMT symbols and appends pause
    void encodeDShotTX(uint16_t dshot_packet, rmt_symbol_word_t *symbols, size_t &count);

    // --- Configuration Parameter ---
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _isBidirectional;

    // --- DShot Packets Container ---
    uint16_t _lastThrottle = DSHOT_FULL_PACKET;
    uint16_t _received_packet = DSHOT_NULL_PACKET;
    uint16_t _tx_packet = DSHOT_NULL_PACKET;
    uint16_t _packet_crc = 0;

    // --- RMT Channel ---
    rmt_channel_handle_t _rmt_rx_channel = nullptr;
    rmt_channel_handle_t _rmt_tx_channel = nullptr;

    // --- DShot RMT Encoder ---
    rmt_encoder_handle_t _dshot_encoder = nullptr;

    // --- RMT Configuration ---
    rmt_receive_config_t _receive_config = {};
    rmt_transmit_config_t _transmit_config = {};

    // --- RMT Symbol Buffer ---
    rmt_symbol_word_t _rx_symbols[RX_BUFFER_SIZE] = {};
    rmt_symbol_word_t _tx_symbols[TX_BUFFER_SIZE] = {};

    // Stores the last valid eRPM received from the ESC
    uint32_t _last_erpm = 0;
};
