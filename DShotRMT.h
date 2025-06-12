/**
 * @file DShotRMT.h
 * @brief DShot signal generation using ESP32 RMT with continuous repeat and pause between frames, including BiDirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>

static constexpr auto DSHOT_THROTTLE_FAILSAVE = 0;
static constexpr auto DSHOT_THROTTLE_MIN = 48;
static constexpr auto DSHOT_THROTTLE_MAX = 2047;
static constexpr auto DEFAULT_RES_HZ = 10 * 1000 * 1000; // 10 MHz
static constexpr auto PAUSE_BITS = 21;
static constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;

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
    void setThrottle(uint16_t throttle);

    gpio_num_t getGPIO() const { return _gpio; }
    dshot_mode_t getDShotMode() const { return _mode; }

private:
    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _isBidirectional;
    uint16_t _lastThrottle = 0;
    uint16_t dshot_packet = DSHOT_NULL_PACKET;

    rmt_channel_handle_t _rmt_tx_channel = nullptr;
    rmt_channel_handle_t _rmt_rx_channel = nullptr;
    rmt_encoder_handle_t _dshot_encoder = nullptr;
    rmt_transmit_config_t _transmit_config = {};

    void buildFrameSymbols(uint16_t frame, rmt_symbol_word_t *symbols, size_t &count);
    bool decodeTelemetrySymbol(const rmt_symbol_word_t *symbols, size_t count, uint16_t &result);
};
