/**
 * @file DShotRMT.cpp
 * @brief DShot signal generation using ESP32 RMT with continuous repeat and pause between frames, including BiDirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include <DShotRMT.h>

//
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool isBidirectional)
    : _gpio(gpio), _mode(mode), _isBidirectional(isBidirectional) {}

//
void DShotRMT::begin()
{
    if (_isBidirectional)
    {
        rmt_rx_channel_config_t rmt_rx_channel_config = {
            .gpio_num = _gpio,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = DEFAULT_RES_HZ,
            .mem_block_symbols = 64,
            .flags = {
                .invert_in = false,
                .with_dma = false}};

        rmt_new_rx_channel(&rmt_rx_channel_config, &_rmt_rx_channel);
        rmt_enable(_rmt_rx_channel);
    }

    rmt_tx_channel_config_t rmt_tx_channel_config = {
        .gpio_num = _gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = DEFAULT_RES_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
        .flags = {
            // invert Signal if BiDirectional DShot Mode
            .invert_out = _isBidirectional,
            .with_dma = false}};

    rmt_new_tx_channel(&rmt_tx_channel_config, &_rmt_tx_channel);
    rmt_enable(_rmt_tx_channel);

    // Create new encoder
    if (!_dshot_encoder)
    {
        rmt_copy_encoder_config_t enc_cfg = {};
        rmt_new_copy_encoder(&enc_cfg, &_dshot_encoder);
    }

    _transmit_config.loop_count = -1;
    _transmit_config.flags.eot_level = _isBidirectional;
}

//
void DShotRMT::setThrottle(uint16_t throttle)
{
    // Fake 10 Bit transformation to be sure
    throttle = throttle & 0b0000011111111111;

    // Has Throttle really changed?
    if (throttle == _lastThrottle)
        return;

    _lastThrottle = throttle;

    // Prepare Throttle paket
    dshot_packet = (throttle << 1) | (_isBidirectional ? 1 : 0);

    // CRC Calculation
    uint16_t crc = 0;

    if (_isBidirectional)
    {
        // Calculate checksum in inverted/BiDirectional Mode
        crc = (~(dshot_packet ^ (dshot_packet >> 4) ^ (dshot_packet >> 8))) & 0x0F;
    }
    else
    {
        //
        crc = (dshot_packet ^ (dshot_packet >> 4) ^ (dshot_packet >> 8)) & 0x0F;
    }

    // attach CRC to DShot Paket
    dshot_packet = (dshot_packet << 4) | crc;

    // Encode DShot Paket
    rmt_symbol_word_t symbols[32] = {};
    size_t count = 0;

    buildFrameSymbols(dshot_packet, symbols, count);

    // Reset RMT Signnal loop before sending new value
    rmt_disable(_rmt_tx_channel);
    rmt_enable(_rmt_tx_channel);

    // Finally transmit the complete DShot Paket
    rmt_transmit(_rmt_tx_channel, _dshot_encoder, symbols, count * sizeof(rmt_symbol_word_t), &_transmit_config);
}

//
void DShotRMT::buildFrameSymbols(uint16_t dshot_packet, rmt_symbol_word_t *symbols, size_t &count)
{
    uint32_t ticks_per_bit = 0;
    uint32_t ticks_zero_high = 0;
    uint32_t ticks_one_high = 0;

    switch (_mode)
    {
    case DSHOT150:
        ticks_per_bit = 64;
        ticks_zero_high = 24;
        ticks_one_high = 48;
        break;
    case DSHOT300:
        ticks_per_bit = 32;
        ticks_zero_high = 12;
        ticks_one_high = 24;
        break;
    case DSHOT600:
        ticks_per_bit = 16;
        ticks_zero_high = 6;
        ticks_one_high = 12;
        break;
    }

    uint32_t ticks_zero_low = ticks_per_bit - ticks_zero_high;
    uint32_t ticks_one_low = ticks_per_bit - ticks_one_high;

    for (int i = 15; i >= 0; i--)
    {
        bool bit = (dshot_packet >> i) & 0x01;
        symbols[count].level0 = 1;
        symbols[count].duration0 = bit ? ticks_one_high : ticks_zero_high;
        symbols[count].level1 = 0;
        symbols[count].duration1 = bit ? ticks_one_low : ticks_zero_low;
        count++;
    }

    symbols[count].level0 = 0;
    symbols[count].duration0 = ticks_per_bit * PAUSE_BITS;
    symbols[count].level1 = 0;
    symbols[count].duration1 = 0;
    count++;
}
