//
// Name:        DShotRMT.h
// Created: 	20.03.2021 00:49:15
// Author:  	derdoktor667
//

#ifndef _DSHOTRMT_h
#define _DSHOTRMT_h

#include <Arduino.h>

// The RMT (Remote Control) module library is used for generating the DShot signal.
#include <driver/rmt.h>

// Defines the library version
constexpr auto DSHOT_LIB_VERSION = "0.2.4";

// Constants related to the DShot protocol
constexpr auto DSHOT_CLK_DIVIDER = 8;    // Slow down RMT clock to 0.1 microseconds / 100 nanoseconds per cycle
constexpr auto DSHOT_PACKET_LENGTH = 17; // Last pack is the pause
constexpr auto DSHOT_THROTTLE_MIN = 48;
constexpr auto DSHOT_THROTTLE_MAX = 2047;
constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;
constexpr auto DSHOT_PAUSE = 21; // 21-bit is recommended
constexpr auto DSHOT_PAUSE_BIT = 16;
constexpr auto F_CPU_RMT = APB_CLK_FREQ;
constexpr auto RMT_CYCLES_PER_SEC = (F_CPU_RMT / DSHOT_CLK_DIVIDER);
constexpr auto RMT_CYCLES_PER_ESP_CYCLE = (F_CPU / RMT_CYCLES_PER_SEC);

// Enumeration for the DShot mode
typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// Array of human-readable DShot mode names
static const char *const dshot_mode_name[] = {
    "DSHOT_OFF",
    "DSHOT150",
    "DSHOT300",
    "DSHOT600",
    "DSHOT1200"};

// Enumeration for telemetric request
typedef enum telemetric_request_e
{
    NO_TELEMETRIC,
    ENABLE_TELEMETRIC,
} telemetric_request_t;

// Structure for DShot packets
typedef struct dshot_packet_s
{
    uint16_t throttle_value : 11;
    telemetric_request_t telemetric_request : 1;
    uint16_t checksum : 4;
} dshot_packet_t;

// Structure for eRPM packets
typedef struct eRPM_packet_s
{
    uint16_t eRPM_data : 12;
    uint8_t checksum : 4;
} eRPM_packet_t;

// Structure for all settings for the DShot mode
typedef struct dshot_config_s
{
    dshot_mode_t mode;
    String name_str;
    bool is_bidirectional;
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

// The main DShotRMT class
class DShotRMT
{
public:
    // Constructor for the DShotRMT class
    DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel);
    DShotRMT(uint8_t pin, uint8_t channel);

    // Destructor for the DShotRMT class
    ~DShotRMT();

    // Copy constructor for the DShotRMT class
    DShotRMT(DShotRMT const &);

    // The begin() function initializes the DShotRMT class with
    // a given DShot mode (DSHOT_OFF, DSHOT150, DSHOT300, DSHOT600, DSHOT1200)
    // and a bidirectional flag. It returns a boolean value
    // indicating whether or not the initialization was successful.
    bool begin(dshot_mode_t dshot_mode = DSHOT_OFF, bool is_bidirectional = false);

    // The sendThrottleValue() function sends a DShot packet with a given
    // throttle value (between 0 and 2047) and an optional telemetry
    // request flag.
    // void sendThrottleValue(uint16_t throttle_value, telemetric_request_t telemetric_request = NO_TELEMETRIC);
    void sendThrottleValue(uint16_t throttle_value);

private:
    rmt_item32_t dshot_tx_rmt_item[DSHOT_PACKET_LENGTH]; // An array of RMT items used to send a DShot packet.
    rmt_config_t dshot_tx_rmt_config;                    // The RMT configuration used for sending DShot packets.
    dshot_config_t dshot_config;                         // The configuration for the DShot mode.

    rmt_item32_t *buildTxRmtItem(uint16_t parsed_packet);       // Constructs an RMT item from a parsed DShot packet.
    uint16_t calculateCRC(const dshot_packet_t &dshot_packet);  // Calculates the CRC checksum for a DShot packet.
    uint16_t parseRmtPaket(const dshot_packet_t &dshot_packet); // Parses an RMT packet to obtain a DShot packet.

    void sendRmtPaket(const dshot_packet_t &dshot_packet); // Sends a DShot packet via RMT.
};

#endif
