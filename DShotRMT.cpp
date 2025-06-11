/**
 * @file DShotRMT.cpp
 * @brief Implementation of continuous DShot signal using ESP32 RMT encoder API with pause between frames
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include <DShotRMT.h>

DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool isBidirectional)
    : _gpio(gpio), _mode(mode), _isBidirectional(isBidirectional) {}

void DShotRMT::begin()
{
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = _gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = DEFAULT_RES_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
        .flags = {
            .invert_out = _isBidirectional,
            .with_dma = false}};

    rmt_new_tx_channel(&tx_config, &_channel);
    rmt_enable(_channel);

    // Create encoder only once
    if (!_encoder)
    {
        rmt_copy_encoder_config_t enc_cfg = {};
        rmt_new_copy_encoder(&enc_cfg, &_encoder);
    }

    _tx_config.loop_count = -1; // Infinite loop
    _tx_config.flags.eot_level = 0;
}

void DShotRMT::setThrottle(uint16_t throttle, bool telemetry)
{
    // Clamp to 11 bits
    throttle &= 0x07FF;
    if (throttle == _lastThrottle && telemetry == _lastTelemetry)
        return;

    _lastThrottle = throttle;
    _lastTelemetry = telemetry;

    // Build 16-bit DShot packet
    uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);
    uint8_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    packet = (packet << 4) | crc;

    // Build symbols
    rmt_symbol_word_t symbols[32] = {};
    size_t count = 0;

    buildFrameSymbols(packet, symbols, count);

    // Transmit
    rmt_disable(_channel); // Ensure safe restart
    rmt_enable(_channel);

    rmt_transmit(_channel, _encoder, symbols, count * sizeof(rmt_symbol_word_t), &_tx_config);
}

void DShotRMT::buildFrameSymbols(uint16_t dshot_packet, rmt_symbol_word_t *symbols, size_t &count)
{
    uint32_t ticks_per_bit = 0;
    uint32_t ticks_zero_high = 0;
    uint32_t ticks_one_high = 0;

    switch (_mode)
    {
    case DSHOT150:
        ticks_per_bit = 67;
        ticks_zero_high = 25;
        ticks_one_high = 50;
        break;
    case DSHOT300:
        ticks_per_bit = 33;
        ticks_zero_high = 12;
        ticks_one_high = 25;
        break;
    case DSHOT600:
        ticks_per_bit = 17;
        ticks_zero_high = 6;
        ticks_one_high = 13;
        break;
    }

    uint32_t ticks_zero_low = ticks_per_bit - ticks_zero_high;
    uint32_t ticks_one_low = ticks_per_bit - ticks_one_high;

    // Encode 16 bits
    for (int i = 15; i >= 0; i--)
    {
        bool bit = (dshot_packet >> i) & 0x01;
        symbols[count].level0 = 1;
        symbols[count].duration0 = bit ? ticks_one_high : ticks_zero_high;
        symbols[count].level1 = 0;
        symbols[count].duration1 = bit ? ticks_one_low : ticks_zero_low;
        count++;
    }

    // Add pause
    symbols[count].level0 = 0;
    symbols[count].duration0 = ticks_per_bit * PAUSE_BITS;
    symbols[count].level1 = 0;
    symbols[count].duration1 = 0;
    count++;
}
