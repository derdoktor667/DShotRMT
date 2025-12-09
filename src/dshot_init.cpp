/**
 * @file dshot_init.cpp
 * @brief RMT configuration and initialization functions for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-10-04
 * @license MIT
 */

#include "dshot_init.h"

// Function to initialize the RMT TX channel
dshot_result_t _init_rmt_tx_channel(gpio_num_t gpio, rmt_channel_handle_t *out_channel, bool is_bidirectional)
{
    rmt_tx_channel_config_t tx_channel_config = {};
    tx_channel_config.gpio_num = gpio;
    tx_channel_config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    tx_channel_config.resolution_hz = DSHOT_RMT_RESOLUTION;
    tx_channel_config.mem_block_symbols = RMT_TX_BUFFER_SYMBOLS;
    tx_channel_config.trans_queue_depth = RMT_QUEUE_DEPTH;
    tx_channel_config.intr_priority = 0;
    tx_channel_config.flags.invert_out = is_bidirectional ? DSHOT_PULSE_LEVEL_HIGH : DSHOT_PULSE_LEVEL_LOW;
    tx_channel_config.flags.with_dma = DSHOT_PULSE_LEVEL_LOW;
    tx_channel_config.flags.io_loop_back = is_bidirectional ? DSHOT_PULSE_LEVEL_HIGH : DSHOT_PULSE_LEVEL_LOW;

    // Deactivate internal pull-up. 
    // tx_channel_config.flags.io_od_mode = is_bidirectional ? DSHOT_PULSE_LEVEL_HIGH : DSHOT_PULSE_LEVEL_LOW;
    tx_channel_config.flags.init_level = DSHOT_PULSE_LEVEL_LOW;

    if (rmt_new_tx_channel(&tx_channel_config, out_channel) != DSHOT_OK)
    {
        return dshot_result_t::create_error(DSHOT_TX_INIT_FAILED);
    }

    if (rmt_enable(*out_channel) != DSHOT_OK)
    {
        return dshot_result_t::create_error(DSHOT_TX_INIT_FAILED);
    }

    return dshot_result_t::create_success(DSHOT_TX_INIT_SUCCESS);
}

// Function to initialize the RMT RX channel
dshot_result_t _init_rmt_rx_channel(gpio_num_t gpio, rmt_channel_handle_t *out_channel, rmt_rx_event_callbacks_t *rx_event_callbacks, void *user_data)
{
    rmt_rx_channel_config_t rx_channel_config = {};
    rx_channel_config.gpio_num = gpio;
    rx_channel_config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    rx_channel_config.resolution_hz = DSHOT_RMT_RESOLUTION;
    rx_channel_config.mem_block_symbols = RMT_RX_BUFFER_SYMBOLS;
    rx_channel_config.intr_priority = 0;

    if (rmt_new_rx_channel(&rx_channel_config, out_channel) != DSHOT_OK)
    {
        return dshot_result_t::create_error(DSHOT_RX_INIT_FAILED);
    }

    if (rmt_rx_register_event_callbacks(*out_channel, rx_event_callbacks, user_data) != DSHOT_OK)
    {
        return dshot_result_t::create_error(DSHOT_CALLBACK_REGISTERING_FAILED);
    }

    if (rmt_enable(*out_channel) != DSHOT_OK)
    {
        return dshot_result_t::create_error(DSHOT_RX_INIT_FAILED);
    }

    return dshot_result_t::create_success(DSHOT_RX_INIT_SUCCESS);
}

// Function to initialize the DShot RMT encoder
dshot_result_t _init_dshot_encoder(rmt_encoder_handle_t *out_encoder, const rmt_ticks_t &rmt_ticks)
{
    rmt_bytes_encoder_config_t encoder_config = {};
    encoder_config.bit0.duration0 = rmt_ticks.t0h_ticks;
    encoder_config.bit0.level0 = DSHOT_PULSE_LEVEL_HIGH;
    encoder_config.bit0.duration1 = rmt_ticks.t0l_ticks;
    encoder_config.bit0.level1 = DSHOT_PULSE_LEVEL_LOW;
    encoder_config.bit1.duration0 = rmt_ticks.t1h_ticks;
    encoder_config.bit1.level0 = DSHOT_PULSE_LEVEL_HIGH;
    encoder_config.bit1.duration1 = rmt_ticks.t1l_ticks;
    encoder_config.bit1.level1 = DSHOT_PULSE_LEVEL_LOW;
    encoder_config.flags.msb_first = 1;

    if (rmt_new_bytes_encoder(&encoder_config, out_encoder) != DSHOT_OK)
    {
        return dshot_result_t::create_error(DSHOT_ENCODER_INIT_FAILED);
    }

    return dshot_result_t::create_success(DSHOT_ENCODER_INIT_SUCCESS);
}