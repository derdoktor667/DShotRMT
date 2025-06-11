/**
 * @file DShotRMT.h
 * @brief DShot signal generation using ESP32 RMT with continuous repeat and pause between frames
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <driver/rmt_tx.h>

static constexpr auto DSHOT_THROTTLE_FAILSAVE = 0;
static constexpr auto DSHOT_THROTTLE_MIN = 48;
static constexpr auto DSHOT_THROTTLE_MAX = 2047;
static constexpr auto DEFAULT_RES_HZ = 10 * 1000 * 1000; // 10 MHz resolution
static constexpr auto PAUSE_BITS = 21;
static constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;

/// DShot Mode
typedef enum dshot_mode_e
{
    DSHOT150,
    DSHOT300,
    DSHOT600
} dshot_mode_t;

class DShotRMT
{
public:
    DShotRMT(gpio_num_t gpio, dshot_mode_t mode = DSHOT300, bool isBidirectional = false);

    void begin();
    void setThrottle(uint16_t throttle, bool telemetry = false);

    gpio_num_t getGPIO() const { return _gpio; }
    dshot_mode_t getDShotMode() const { return _mode; }

private:
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _isBidirectional;

    rmt_channel_handle_t _channel = nullptr;
    rmt_encoder_handle_t _encoder = nullptr;
    rmt_transmit_config_t _tx_config = {};

    uint16_t _lastThrottle = 0;
    bool _lastTelemetry = false;

    void buildFrameSymbols(uint16_t frame, rmt_symbol_word_t *symbols, size_t &count);
};
