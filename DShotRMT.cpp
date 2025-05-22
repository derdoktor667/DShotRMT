//------------------------------------------------------------------------------
// Name:        DShotRMT
// Date:        2025-04-14
// Author:      Wastl Kraus
// Description: ESP32 Library for controlling ESCs using the DShot protocol.
//------------------------------------------------------------------------------

#include "Arduino.h"
#include <DShotRMT.h>

// Constructor: store GPIO pin and mode
DShotRMT::DShotRMT(int gpio_num, dshot_mode_t mode, bool bidir)
{
    _pin = gpio_num;
    _dshot_mode = mode;
    _bidir = bidir;
    _rmt_channel = nullptr;
}

// Initializes the RMT channel with the specified settings
void DShotRMT::begin()
{
    Serial.printf("DShotRMT initialized on GPIO %d at mode DSHOT%d\n", _pin, static_cast<int>(_dshot_mode));

    rmt_tx_channel_config_t tx_config = {
        static_cast<gpio_num_t>(_pin),           // gpio_num
        RMT_CLK_SRC_DEFAULT,                     // clk_src
        64,                                      // mem_block_symbols (enough symbols for DShot600)
        static_cast<uint32_t>(_dshot_mode) * 67, // resolution_hz (67 ticks per bit)
        4,                                       // trans_queue_depth
    };

    // Create the RMT TX channel
    esp_err_t err = rmt_new_tx_channel(&tx_config, &_rmt_channel);
    if (err != ESP_OK)
    {
        Serial.println("Failed to create RMT channel");
        return;
    }

    // Enable the RMT channel
    rmt_enable(_rmt_channel);
}

// Sends a DShot command (stub)
void DShotRMT::sendDShotCommand(uint16_t command)
{
    Serial.print("Sending DShot command: ");
    Serial.println(command);
    // TODO: Encode 16-bit DShot packet and send using RMT items
}

// Builds a 16-bit DShot frame from value and telemetry flag
uint16_t DShotRMT::buildDShotPacket(uint16_t value, bool telemetry)
{
    value = (value & 0x07FF) << 1; // 11-bit value, shift left by 1
    if (telemetry)
    {
        value |= 1; // Set telemetry bit
    }

    // Compute checksum (XOR of upper 3 nibbles)
    uint16_t checksum = 0;
    uint16_t temp = value;
    for (int i = 0; i < 3; i++)
    {
        checksum ^= (temp >> (4 * i)) & 0xF;
    }

    return (value << 4) | (checksum & 0xF);
}

// Encodes the 16-bit DShot packet into RMT symbols
void DShotRMT::encodeDShotToRMT(uint16_t packet, rmt_symbol_word_t *symbols)
{
    const uint32_t total_ticks = 67; // 67 ticks per bit (constant for both DSHOT300/600 modes)
    const uint32_t t1h = (total_ticks * 2) / 3;
    const uint32_t t1l = total_ticks - t1h;
    const uint32_t t0h = total_ticks / 3;
    const uint32_t t0l = total_ticks - t0h;

    for (int i = 0; i < 16; ++i)
    {
        bool bit = (packet >> (15 - i)) & 0x01;
        if (bit)
        {
            symbols[i].level0 = 1;
            symbols[i].duration0 = t1h;
            symbols[i].level1 = 0;
            symbols[i].duration1 = t1l;
        }
        else
        {
            symbols[i].level0 = 1;
            symbols[i].duration0 = t0h;
            symbols[i].level1 = 0;
            symbols[i].duration1 = t0l;
        }
    }
}
