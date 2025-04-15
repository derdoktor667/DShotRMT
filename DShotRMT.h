//------------------------------------------------------------------------------
// Name:        DShotRMT
// Date:        2025-04-14
// Author:      Wastl Kraus
// Description: ESP32 Library for controlling ESCs using the DShot protocol.
//------------------------------------------------------------------------------

#ifndef DSHOT_RMT_H
#define DSHOT_RMT_H

#include <Arduino.h>
#include "driver/rmt_tx.h"

enum DShotMode {
    DSHOT300 = 300000,
    DSHOT600 = 600000
};

class DShotRMT {
public:
    // Constructor: specify GPIO and DShot mode (DSHOT300 or DSHOT600)
    explicit DShotRMT(int gpio_num, DShotMode mode = DSHOT600);

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
    DShotMode _mode;                   // DShot mode (DSHOT300 or DSHOT600)
};

#endif // DSHOT_RMT_H
