//
// Name:        DShotRMT.cpp
// Created: 	20.03.2021 00:49:15
// Author:  	derdoktor667
//

#include <DShotRMT.h>

// Constructor that takes gpio and rmtChannel as arguments
DShotRMT::DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel)
{
    // Initialize the dshot_config structure with the arguments passed to the constructor
    dshot_config.gpio_num = gpio;
    dshot_config.pin_num = static_cast<uint8_t>(gpio);
    dshot_config.rmt_channel = rmtChannel;
    dshot_config.mem_block_num = static_cast<uint8_t>(RMT_CHANNEL_MAX - static_cast<uint8_t>(rmtChannel));

    // Create an empty packet using the DSHOT_NULL_PACKET and the buildTxRmtItem function
    buildTxRmtItem(DSHOT_NULL_PACKET);
}

// Constructor that takes pin and channel as arguments
DShotRMT::DShotRMT(uint8_t pin, uint8_t channel)
{
    // Initialize the dshot_config structure with the arguments passed to the constructor
    dshot_config.gpio_num = static_cast<gpio_num_t>(pin);
    dshot_config.pin_num = pin;
    dshot_config.rmt_channel = static_cast<rmt_channel_t>(channel);
    dshot_config.mem_block_num = RMT_CHANNEL_MAX - channel;

    // Create an empty packet using the DSHOT_NULL_PACKET and the buildTxRmtItem function
    buildTxRmtItem(DSHOT_NULL_PACKET);
}

// ...simplest but only for testing
DShotRMT::DShotRMT(uint8_t pin)
{
    // Initialize the dshot_config structure with the arguments passed to the constructor
    dshot_config.gpio_num = static_cast<gpio_num_t>(pin);
    dshot_config.pin_num = pin;
    dshot_config.rmt_channel = static_cast<rmt_channel_t>(RMT_CHANNEL_MAX -1);
    dshot_config.mem_block_num = RMT_CHANNEL_MAX - 1;

    // Create an empty packet using the DSHOT_NULL_PACKET and the buildTxRmtItem function
    buildTxRmtItem(DSHOT_NULL_PACKET);
}

DShotRMT::~DShotRMT()
{
    // Uninstall the RMT driver
    rmt_driver_uninstall(dshot_config.rmt_channel);
}

DShotRMT::DShotRMT(DShotRMT const &)
{
    // ...write me
}

bool DShotRMT::begin(dshot_mode_t dshot_mode, bool is_bidirectional)
{
    // Set DShot configuration parameters based on input parameters
    dshot_config.mode = dshot_mode;
    dshot_config.clk_div = DSHOT_CLK_DIVIDER;
    dshot_config.name_str = dshot_mode_name[dshot_mode];
    dshot_config.is_bidirectional = is_bidirectional;

    // Set timing parameters based on selected DShot mode
    switch (dshot_config.mode)
    {
    case DSHOT150:
        dshot_config.ticks_per_bit = 64;
        dshot_config.ticks_zero_high = 24;
        dshot_config.ticks_one_high = 48;
        break;

    case DSHOT300:
        dshot_config.ticks_per_bit = 32;
        dshot_config.ticks_zero_high = 12;
        dshot_config.ticks_one_high = 24;
        break;

    case DSHOT600:
        dshot_config.ticks_per_bit = 16;
        dshot_config.ticks_zero_high = 6;
        dshot_config.ticks_one_high = 12;
        break;

    case DSHOT1200:
        dshot_config.ticks_per_bit = 8;
        dshot_config.ticks_zero_high = 3;
        dshot_config.ticks_one_high = 6;
        break;

    // Default case to handle invalid input
    default:
        dshot_config.ticks_per_bit = 0;
        dshot_config.ticks_zero_high = 0;
        dshot_config.ticks_one_high = 0;
        break;
    }

    // Calculate low signal timing
    dshot_config.ticks_zero_low = (dshot_config.ticks_per_bit - dshot_config.ticks_zero_high);
    dshot_config.ticks_one_low = (dshot_config.ticks_per_bit - dshot_config.ticks_one_high);

    // Set up RMT configuration for DShot transmission
    dshot_tx_rmt_config.rmt_mode = RMT_MODE_TX;
    dshot_tx_rmt_config.channel = dshot_config.rmt_channel;
    dshot_tx_rmt_config.gpio_num = dshot_config.gpio_num;
    dshot_tx_rmt_config.mem_block_num = dshot_config.mem_block_num;
    dshot_tx_rmt_config.clk_div = dshot_config.clk_div;
    dshot_tx_rmt_config.tx_config.loop_en = false;
    dshot_tx_rmt_config.tx_config.carrier_en = false;
    dshot_tx_rmt_config.tx_config.idle_output_en = true;

    // Set idle level for RMT transmission based on input parameter
    if (dshot_config.is_bidirectional)
    {
        dshot_tx_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    }
    else
    {
        dshot_tx_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    }

    // Set up selected DShot mode
    rmt_config(&dshot_tx_rmt_config);

    // Install RMT driver and return result
    return rmt_driver_install(dshot_tx_rmt_config.channel, 0, 0);
}

// Define a function to send a DShot command over an RMT interface to control a brushless motor's speed.
void DShotRMT::sendThrottleValue(uint16_t throttle_value)
{
    dshot_packet_t dshot_rmt_packet = {};

    // Check if the throttle value is less than the minimum allowed value for the DShot protocol.
    if (throttle_value < DSHOT_THROTTLE_MIN)
    {
        throttle_value = DSHOT_THROTTLE_MIN;
    }

    // Check if the throttle value is greater than the maximum allowed value for the DShot protocol.
    if (throttle_value > DSHOT_THROTTLE_MAX)
    {
        throttle_value = DSHOT_THROTTLE_MAX;
    }

    dshot_rmt_packet.throttle_value = throttle_value;

    // Telemetric using additional pin on the ESC is not supported.
    dshot_rmt_packet.telemetric_request = NO_TELEMETRIC;

    // Calculate the checksum for the DShot packet using the calculateCRC function.
    dshot_rmt_packet.checksum = calculateCRC(dshot_rmt_packet);

    // Send the DShot packet over the RMT interface to control the motor's speed.
    sendRmtPaket(dshot_rmt_packet);
}

// This method builds the RMT data transmission sequence for the DShot protocol
rmt_item32_t *DShotRMT::buildTxRmtItem(uint16_t parsed_packet)
{
    // Check if DShot is set to bidirectional mode
    if (dshot_config.is_bidirectional)
    {
        // If bidirectional, invert the high/low bits
        for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1)
        {
            if (parsed_packet & 0b1000000000000000)
            {
                // Set RMT item for a logic high signal
                dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_one_low;
                dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_one_high;
            }
            else
            {
                // Set RMT item for a logic low signal
                dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_zero_low;
                dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_zero_high;
            }

            // Set level of RMT item
            dshot_tx_rmt_item[i].level0 = 0;
            dshot_tx_rmt_item[i].level1 = 1;
        }
    }
    else
    {
        // If not bidirectional, set the RMT items as usual
        for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1)
        {
            if (parsed_packet & 0b1000000000000000)
            {
                // Set RMT item for a logic high signal
                dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_one_high;
                dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_one_low;
            }
            else
            {
                // Set RMT item for a logic low signal
                dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_zero_high;
                dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_zero_low;
            }

            // Set level of RMT item
            dshot_tx_rmt_item[i].level0 = 1;
            dshot_tx_rmt_item[i].level1 = 0;
        }
    }

    // Set end marker for each frame
    if (dshot_config.is_bidirectional)
    {
        dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level0 = 1;
        dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level1 = 0;
    }
    else
    {
        dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level0 = 0;
        dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level1 = 1;
    }

    // Add packet seperator aka DShot Pause.
    dshot_tx_rmt_item[DSHOT_PAUSE_BIT].duration1 = DSHOT_PAUSE;

    // Return the rmt_item
    return dshot_tx_rmt_item;
}

// Calculates a CRC value for a DShot digital control signal packet
uint16_t DShotRMT::calculateCRC(const dshot_packet_t &dshot_packet)
{
    uint16_t crc;

    // Combine the throttle value and telemetric request flag into a 16-bit packet value
    const uint16_t packet = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;

    // Calculate the CRC value using different bitwise operations depending on the DShot configuration
    if (dshot_config.is_bidirectional)
    {
        // Bidirectional configuration: perform a bitwise negation of the result of XORing the packet with its right-shifted values by 4 and 8 bits,
        // and then bitwise AND the result with 0x0F
        const uint16_t intermediate_result = packet ^ (packet >> 4) ^ (packet >> 8);
        crc = (~intermediate_result) & 0x0F;
    }
    else
    {
        // Unidirectional configuration: XOR the packet with its right-shifted values by 4 and 8 bits,
        // and then bitwise AND the result with 0x0F
        crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    }

    // Return the calculated CRC value as a 16-bit unsigned integer
    return crc;
}

uint16_t DShotRMT::parseRmtPaket(const dshot_packet_t &dshot_packet)
{
    uint16_t parsedRmtPaket = DSHOT_NULL_PACKET;
    uint16_t crc = calculateCRC(dshot_packet);

    // Complete the paket
    parsedRmtPaket = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;
    parsedRmtPaket = (parsedRmtPaket << 4) | crc;

    return parsedRmtPaket;
}

// Output using ESP32 RMT
void DShotRMT::sendRmtPaket(const dshot_packet_t &dshot_packet)
{
    buildTxRmtItem(parseRmtPaket(dshot_packet));

    rmt_write_items(dshot_tx_rmt_config.channel, dshot_tx_rmt_item, DSHOT_PACKET_LENGTH, false);
}
