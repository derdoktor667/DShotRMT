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

// return states of the RPM getting function
typedef enum dshot_erpm_exit_mode_e
{
    DECODE_SUCCESS = 0,
    ERR_EMPTY_QUEUE,
    ERR_NO_PACKETS,
    ERR_CHECKSUM_FAIL,
    ERR_BIDIRECTION_DISABLED,

} dshot_erpm_exit_mode_t;

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

// The official DShot Commands
typedef enum dshot_cmd_e
{
    DSHOT_CMD_MOTOR_STOP = 0,                    // Currently not implemented - STOP Motors
    DSHOT_CMD_BEEP1,                             // Wait at least length of beep (380ms) before next command
    DSHOT_CMD_BEEP2,                             // Wait at least length of beep (380ms) before next command
    DSHOT_CMD_BEEP3,                             // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_BEEP4,                             // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_BEEP5,                             // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_ESC_INFO,                          // Currently not implemented
    DSHOT_CMD_SPIN_DIRECTION_1,                  // Need 6x, no wait required
    DSHOT_CMD_SPIN_DIRECTION_2,                  // Need 6x, no wait required
    DSHOT_CMD_3D_MODE_OFF,                       // Need 6x, no wait required
    DSHOT_CMD_3D_MODE_ON,                        // Need 6x, no wait required
    DSHOT_CMD_SETTINGS_REQUEST,                  // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,                     // Need 6x, wait at least 12ms before next command
    DSHOT_CMD_SPIN_DIRECTION_NORMAL,             // Need 6x, no wait required
    DSHOT_CMD_SPIN_DIRECTION_REVERSED,           // Need 6x, no wait required
    DSHOT_CMD_LED0_ON,                           // Currently not implemented
    DSHOT_CMD_LED1_ON,                           // Currently not implemented
    DSHOT_CMD_LED2_ON,                           // Currently not implemented
    DSHOT_CMD_LED3_ON,                           // Currently not implemented
    DSHOT_CMD_LED0_OFF,                          // Currently not implemented
    DSHOT_CMD_LED1_OFF,                          // Currently not implemented
    DSHOT_CMD_LED2_OFF,                          // Currently not implemented
    DSHOT_CMD_LED3_OFF,                          // Currently not implemented
    DSHOT_CMD_36,                                // Not yet assigned
    DSHOT_CMD_37,                                // Not yet assigned
    DSHOT_CMD_38,                                // Not yet assigned
    DSHOT_CMD_39,                                // Not yet assigned
    DSHOT_CMD_40,                                // Not yet assigned
    DSHOT_CMD_41,                                // Not yet assigned
    DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY, // No wait required
    DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY,     // No wait required
    DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY,     // No wait required
    DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY, // No wait required
    DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY,        // No wait required
    DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY, // No wait required (also command 47)
    DSHOT_CMD_MAX = 47
} dshot_cmd_t;

// ...Mapping for GCR
static const unsigned char GCR_encode[16] =
    {
        0x19, 0x1B, 0x12, 0x13,
        0x1D, 0x15, 0x16, 0x17,
        0x1A, 0x09, 0x0A, 0x0B,
        0x1E, 0x0D, 0x0E, 0x0F};

// ...shifting 5 bits > 4 bits (0xff => invalid)
static const unsigned char GCR_decode[32] =
    {
        0xFF, 0xFF, 0xFF, 0xFF, // 0 - 3
        0xFF, 0xFF, 0xFF, 0xFF, // 4 - 7
        0xFF, 9, 10, 11,        // 8 - 11
        0xFF, 13, 14, 15,       // 12 - 15

        0xFF, 0xFF, 2, 3,  // 16 - 19
        0xFF, 5, 6, 7,     // 20 - 23
        0xFF, 0, 8, 1,     // 24 - 27
        0xFF, 4, 12, 0xFF, // 28 - 31
};

// The main DShotRMT class
class DShotRMT
{
public:
    // Constructor for the DShotRMT class
    DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel);
    DShotRMT(uint8_t pin, uint8_t channel);
    DShotRMT(uint8_t pin);

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
    // throttle value (between 49 and 2047) and an optional telemetry
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
