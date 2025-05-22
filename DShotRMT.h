//------------------------------------------------------------------------------
// Name:        DShotRMT
// Date:        2025-04-14
// Author:      Wastl Kraus
// Description: ESP32 Library for controlling ESCs using the DShot protocol.
//------------------------------------------------------------------------------

#ifndef DSHOT_RMT_H
#define DSHOT_RMT_H

#include <Arduino.h>
#include <driver/rmt_tx.h>

// Constants related to the DShot protocol
constexpr auto DSHOT_CLK_DIVIDER = 8;    // Slow down RMT clock to 0.1 microseconds / 100 nanoseconds per cycle
constexpr auto DSHOT_PACKET_LENGTH = 17; // Last pack is the pause
constexpr auto DSHOT_THROTTLE_MIN = 48;
constexpr auto DSHOT_THROTTLE_MAX = 2047;
constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;
constexpr auto DSHOT_PAUSE = 21; // 21-bit is recommended
constexpr auto DSHOT_PAUSE_BIT = 16;
constexpr auto F_CPU_RMT = ( 80*1000000 );       //unit: Hz
constexpr auto RMT_CYCLES_PER_SEC = (F_CPU_RMT / DSHOT_CLK_DIVIDER);
constexpr auto RMT_CYCLES_PER_ESP_CYCLE = (F_CPU / RMT_CYCLES_PER_SEC);

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

// Enumeration for the DShot mode
typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

class DShotRMT
{
public:
    // Constructor: specify GPIO and DShot mode (DSHOT300 default)
    explicit DShotRMT(int gpio_num, dshot_mode_e mode = DSHOT300, bool bidir = 0);

    // Initializes the RMT channel
    void begin();

    // Sends a DShot command (16-bit value with telemetry flag and checksum)
    void sendDShotCommand(uint16_t command);

private:
    // Builds a 16-bit DShot frame from value and telemetry flag
    uint16_t buildDShotPacket(uint16_t value, bool telemetry);

    // Encodes the 16-bit DShot packet into RMT symbols
    void encodeDShotToRMT(uint16_t packet, rmt_symbol_word_t *symbols);

    rmt_channel_handle_t _rmt_channel; // Handle to the RMT TX channel
    int _pin;                          // GPIO pin used for DShot output
    dshot_mode_t _dshot_mode;          // DShot mode
    bool _bidir = 0;                   // Bidirectional DShot (off by default)
};

#endif // DSHOT_RMT_H
