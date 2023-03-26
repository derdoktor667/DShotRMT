//
// Name:        DShotRMT.h
// Created: 	20.03.2021 00:49:15
// Author:  	derdoktor667
//

#pragma once

#include <Arduino.h>

// Use the RMT Module library for generating the DShot signal
#include <driver/rmt.h>

// Define library version
constexpr auto DSHOT_LIB_VERSION = "0.2.3";

// Define constants for generating the DShot signal
constexpr auto DSHOT_CLK_DIVIDER = 8;    // Slow down RMT clock to 0.1 microseconds / 100 nanoseconds per cycle
constexpr auto DSHOT_PACKET_LENGTH = 17; // Last pack is the pause

// Define constants for DShot throttle range and null packet
constexpr auto DSHOT_THROTTLE_MIN = 48;
constexpr auto DSHOT_THROTTLE_MAX = 2047;
constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;

// Define constants for DShot pause
constexpr auto DSHOT_PAUSE = 21; // 21bit is recommended, but to be sure
constexpr auto DSHOT_PAUSE_BIT = 16;

// Define constants for RMT timing
constexpr auto F_CPU_RMT = APB_CLK_FREQ;
constexpr auto RMT_CYCLES_PER_SEC = (F_CPU_RMT / DSHOT_CLK_DIVIDER);
constexpr auto RMT_CYCLES_PER_ESP_CYCLE = (F_CPU / RMT_CYCLES_PER_SEC);

// Define enumeration for DShot mode
typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// Define human-readable names for DShot modes
static const char *const dshot_mode_name[] = {
    "DSHOT_OFF",
    "DSHOT150",
    "DSHOT300",
    "DSHOT600",
    "DSHOT1200"
};

// Define type aliases for DShot name and telemetry request
typedef String dshot_name_t;
typedef enum telemetric_request_e
{
    NO_TELEMETRIC,
    ENABLE_TELEMETRIC,
} telemetric_request_t;

// Define structure for DShot packet
typedef struct dshot_packet_s
{
    uint16_t throttle_value : 11;
    telemetric_request_t telemetric_request : 1;
    uint16_t checksum : 4;
} dshot_packet_t;

// Define structure for eRPM packet
typedef struct eRPM_packet_s
{
    uint16_t eRPM_data : 12;
    uint8_t checksum : 4;
} eRPM_packet_t;

// Define structure for DShot configuration
typedef struct dshot_config_s
{
    dshot_mode_t mode;
    dshot_name_t name_str;
    bool bidirectional;
    gpio_num_t gpio_num;
    uint8_t pin_num;
    rmt_channel_t rmt_channel;
    uint8_t mem_block_num;
    uint16_t ticks_per_bit;
    uint8_t clk_div;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
    uint16_t ticks_one_high;
    uint16_t ticks_one_low;
} dshot_config_t;

// Define class for DShot generation using RMT
class DShotRMT
{
public:
    DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel, dshot_mode_t dshot_mode = DSHOT300, bool is_bidirectional = false);
    DShotRMT(uint8_t pin, uint8_t channel, dshot_mode_t dshot_mode = DSHOT300, bool is_bidirectional = false);
    ~DShotRMT();
    DShotRMT(DShotRMT const &);

    // ...safety first ...no parameters, no DShot
    void send_dshot_value(uint16_t throttle_value, telemetric_request_t telemetric_request = NO_TELEMETRIC);

    dshot_config_t *get_dshot_info();
    uint8_t *get_dshot_clock_div();

private:
    rmt_item32_t dshot_tx_rmt_item[DSHOT_PACKET_LENGTH];
    rmt_config_t dshot_tx_rmt_config;
    dshot_config_t dshot_config;

    rmt_item32_t *encode_dshot_to_rmt(uint16_t parsed_packet);
    uint16_t calc_dshot_chksum(const dshot_packet_t &dshot_packet);
    uint16_t prepare_rmt_data(const dshot_packet_t &dshot_packet);

    void install_dshot_driver(dshot_mode_t dshot_mode, bool is_bidirectional);
    void output_rmt_data(const dshot_packet_t &dshot_packet);
};
