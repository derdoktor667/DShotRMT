//------------------------------------------------------------------------------
// Name:        DShotRMT
// Date:        2025-04-14
// Author:      Wastl Kraus
// Description: ESP32 Library for controlling ESCs using the DShot protocol.
//------------------------------------------------------------------------------

#ifndef DSHOTRMT_H
#define DSHOTRMT_H

#include <driver/rmt_tx.h>

// Supported DShot modes
typedef enum dshot_mode_e
{
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// Telemetry flags
#define REQUEST_TELEMETRIC 1
#define NO_TELEMETRIC 0

// DShot packet constants
#define DSHOT_PACKET_LENGTH 17
#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047

// DShot packet structure
typedef struct dshot_packet_s
{
    uint16_t throttle_value;
    uint8_t telemetric_request;
    uint8_t checksum;
} dshot_packet_t;

// Configuration for each DShotRMT instance
typedef struct dshot_config_s
{
    dshot_mode_t mode;
    bool is_bidirectional;
    gpio_num_t gpio_num;
    uint8_t pin_num;
    uint8_t channel_num;
    uint16_t ticks_per_bit;
    uint16_t ticks_one_high;
    uint16_t ticks_one_low;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
} dshot_config_t;

// RMT Channels count
typedef enum
{
    RMT_CHANNEL_0, /*!< RMT channel number 0 */
    RMT_CHANNEL_1, /*!< RMT channel number 1 */
    RMT_CHANNEL_2, /*!< RMT channel number 2 */
    RMT_CHANNEL_3, /*!< RMT channel number 3 */
#if SOC_RMT_CHANNELS_PER_GROUP > 4
    RMT_CHANNEL_4, /*!< RMT channel number 4 */
    RMT_CHANNEL_5, /*!< RMT channel number 5 */
    RMT_CHANNEL_6, /*!< RMT channel number 6 */
    RMT_CHANNEL_7, /*!< RMT channel number 7 */
#endif
    RMT_CHANNEL_MAX /*!< Number of RMT channels */
};

//
class DShotRMT
{
public:
    // Constructors
    DShotRMT(gpio_num_t gpio, dshot_mode_t mode);
    DShotRMT(uint8_t pin, dshot_mode_t mode);
    DShotRMT(uint8_t pin); // Default to channel 0
    DShotRMT(const DShotRMT &other);

    ~DShotRMT();

    bool begin(bool is_bidirectional = false);

    void sendThrottleValue(uint16_t throttle_value);

private:
    void sendRmtPaket(const dshot_packet_t &packet);

    dshot_config_t dshot_config;
    rmt_channel_handle_t rmt_channel_handle = nullptr;
    rmt_transmit_config_t transmit_cfg;

    uint16_t calculateCRC(const dshot_packet_t &packet);
    uint16_t parseRmtPaket(const dshot_packet_t &packet);
    uint32_t buildRmtSymbol(bool bit);
};

#endif // DSHOTRMT_H
