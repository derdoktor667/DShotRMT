//------------------------------------------------------------------------------
// Name:        DShotRMT
// Date:        2025-04-14
// Author:      Wastl Kraus
// Description: ESP32 Library for controlling ESCs using the DShot protocol.
//------------------------------------------------------------------------------

#include <Arduino.h>
#include <DShotRMT.h>

// Constructor using GPIO number and RMT channel
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode)
{
    dshot_config.gpio_num = gpio;
    dshot_config.pin_num = static_cast<uint8_t>(gpio);
    dshot_config.channel_num = RMT_CHANNEL_MAX - 1;
    dshot_config.mode = mode;
}

// Constructor using pin number and channel
DShotRMT::DShotRMT(uint8_t pin, dshot_mode_t mode)
{
    dshot_config.gpio_num = static_cast<gpio_num_t>(pin);
    dshot_config.pin_num = pin;
    dshot_config.channel_num = RMT_CHANNEL_MAX - 1;
    dshot_config.mode = mode;
}

// Constructor using only pin number (default DSHOT300)
DShotRMT::DShotRMT(uint8_t pin) : DShotRMT(pin, DSHOT300) {}

// Copy constructor
DShotRMT::DShotRMT(const DShotRMT &other)
{
    dshot_config = other.dshot_config;
}

// Destructor
DShotRMT::~DShotRMT()
{
    if (rmt_channel_handle)
    {
        rmt_del_channel(rmt_channel_handle);
        rmt_channel_handle = nullptr;
    }
}

// Calculate checksum (CRC) for DShot packet
uint16_t DShotRMT::calculateCRC(const dshot_packet_t &packet)
{
    uint16_t value = (packet.throttle_value << 1) | (packet.telemetric_request & 0x01);
    uint16_t csum = 0;
    for (int i = 0; i < 3; ++i)
    {
        csum ^= (value >> (i * 4)) & 0x0F;
    }
    return csum & 0x0F;
}

// Convert dshot_packet_t into 16-bit integer
uint16_t DShotRMT::parseRmtPaket(const dshot_packet_t &packet)
{
    return ((packet.throttle_value << 5) | ((packet.telemetric_request & 0x01) << 4) | calculateCRC(packet));
}

// Initialize DShot with mode and bidirectional flag
bool DShotRMT::begin(bool is_bidirectional)
{
    dshot_config.is_bidirectional = is_bidirectional;

    switch (dshot_config.mode)
    {
    case DSHOT150:
        dshot_config.ticks_per_bit = 533; // 1.5 MBit/s → 666ns per bit
        break;
    case DSHOT300:
        dshot_config.ticks_per_bit = 267; // 3.0 MBit/s → 333ns per bit
        break;
    case DSHOT600:
        dshot_config.ticks_per_bit = 133; // 6.0 MBit/s → 167ns per bit
        break;
    case DSHOT1200:
        dshot_config.ticks_per_bit = 67; // 12.0 MBit/s → 83ns per bit
        break;
    default:
        return false;
    }

    dshot_config.ticks_one_high = static_cast<uint16_t>(round(dshot_config.ticks_per_bit * 0.75));
    dshot_config.ticks_one_low = dshot_config.ticks_per_bit - dshot_config.ticks_one_high;
    dshot_config.ticks_zero_high = static_cast<uint16_t>(round(dshot_config.ticks_per_bit * 0.375));
    dshot_config.ticks_zero_low = dshot_config.ticks_per_bit - dshot_config.ticks_zero_high;

    // Configure transmission channel
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = dshot_config.gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 80000000,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .flags = {
            .invert_out = false,
            .with_dma = false,
            .io_loop_back = false,
            .io_od_mode = false,
        }};

    if (rmt_new_tx_channel(&tx_cfg, &rmt_channel_handle) != 0)
        return false;

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0}};

    if (rmt_enable(rmt_channel_handle) != 0)
        return false;

    transmit_cfg = tx_config;

    return true;
}

// Construct one RMT symbol (32bit: high_ticks << 16 | low_ticks)
uint32_t DShotRMT::buildRmtSymbol(bool bit)
{
    uint16_t high_ticks = bit ? dshot_config.ticks_one_high : dshot_config.ticks_zero_high;
    uint16_t low_ticks = bit ? dshot_config.ticks_one_low : dshot_config.ticks_zero_low;
    return (high_ticks << 16) | low_ticks;
}

// Send parsed DShot packet via RMT
void DShotRMT::sendRmtPaket(const dshot_packet_t &packet)
{
    uint16_t parsed = parseRmtPaket(packet);

    uint32_t symbols[DSHOT_PACKET_LENGTH];
    memset(symbols, 0, sizeof(symbols));

    for (int i = 0; i < 16; ++i)
    {
        bool bit = (parsed >> (15 - i)) & 0x1;
        symbols[i] = buildRmtSymbol(bit);
    }

    // Final pause symbol
    symbols[16] = (0 << 16) | dshot_config.ticks_per_bit;

    rmt_transmit(rmt_channel_handle, nullptr, symbols, sizeof(symbols), &transmit_cfg);
    rmt_tx_wait_all_done(rmt_channel_handle, 0);
}

// Send throttle value (49–2047)
void DShotRMT::sendThrottleValue(uint16_t throttle_value)
{
    if (throttle_value < DSHOT_THROTTLE_MIN)
        throttle_value = DSHOT_THROTTLE_MIN;
    if (throttle_value > DSHOT_THROTTLE_MAX)
        throttle_value = DSHOT_THROTTLE_MAX;

    dshot_packet_t packet;
    packet.throttle_value = throttle_value;
    packet.telemetric_request = NO_TELEMETRIC;
    packet.checksum = calculateCRC(packet);

    sendRmtPaket(packet);
}
