/**
 * @file dshot_init.h
 * @brief RMT configuration and initialization function declarations for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-10-04
 * @license MIT
 */

#pragma once

#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>

#include "dshot_definitions.h"

dshot_result_t _init_rmt_tx_channel(gpio_num_t gpio, rmt_channel_handle_t *out_channel, bool is_bidirectional);
dshot_result_t _init_rmt_rx_channel(gpio_num_t gpio, rmt_channel_handle_t *out_channel, rmt_rx_event_callbacks_t *rx_event_callbacks, void *user_data);
dshot_result_t _init_dshot_encoder(rmt_encoder_handle_t *out_encoder, const rmt_ticks_t &rmt_ticks);
