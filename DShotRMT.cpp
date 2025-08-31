/**
 * @file DShotRMT.cpp
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include "DShotRMT.h"
#include <driver/rmt_tx.h>

// Timing parameters for each DShot mode
// Format: {frame_length_us, ticks_per_bit, ticks_one_high, ticks_one_low, ticks_zero_high, ticks_zero_low}
constexpr dshot_timing_t DSHOT_TIMINGS[] = {
    {0, 0, 0, 0, 0, 0},        // DSHOT_OFF
    {128, 64, 48, 16, 24, 40}, // DSHOT150
    {64, 32, 24, 8, 12, 20},   // DSHOT300
    {32, 16, 12, 4, 6, 10},    // DSHOT600
    {16, 8, 6, 2, 3, 5}        // DSHOT1200
};

// Primary constructor with GPIO number
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool is_bidirectional)
    : _gpio(gpio),
      _mode(mode),
      _is_bidirectional(is_bidirectional),
      _timing_config(DSHOT_TIMINGS[mode]),
      _rmt_tx_channel(nullptr),
      _rmt_rx_channel(nullptr),
      _dshot_encoder(nullptr),
      _last_erpm(0),
      _parsed_packet(0),
      _packet{0},
      _last_transmission_time(0),
      _rx_queue(nullptr)
{
    // Calculate frame timing including switch/pause time
    _frame_timer_us = _timing_config.frame_length_us + DSHOT_SWITCH_TIME;

    // Double frame time for bidirectional mode (includes response time)
    if (_is_bidirectional)
    {
        _frame_timer_us = (_frame_timer_us << 1);
    }
}

// Constructor using pin number
DShotRMT::DShotRMT(uint16_t pin_nr, dshot_mode_t mode, bool is_bidirectional)
    : DShotRMT((gpio_num_t)pin_nr, mode, is_bidirectional)
{
    // Delegates to primary constructor with type cast
}

// Initialize DShotRMT
uint16_t DShotRMT::begin()
{
    // Initialize TX channel
    if (!_initTXChannel())
    {
        _dshot_log(TX_INIT_FAILED);
        return DSHOT_ERROR;
    }

    // Initialize RX channel only if bidirectional mode is enabled
    if (_is_bidirectional)
    {
        if (!_initRXChannel())
        {
            _dshot_log(RX_INIT_FAILED);
            return DSHOT_ERROR;
        }
    }

    // Initialize DShot encoder
    if (_initDShotEncoder() != DSHOT_OK)
    {
        _dshot_log(ENCODER_INIT_FAILED);
        return DSHOT_ERROR;
    }

    return DSHOT_OK;
}

// Initialize RMT TX channel
bool DShotRMT::_initTXChannel()
{
    // Configure TX channel
    _tx_channel_config.gpio_num = _gpio;
    _tx_channel_config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    _tx_channel_config.resolution_hz = DSHOT_RMT_RESOLUTION;
    _tx_channel_config.mem_block_symbols = TX_BUFFER_SIZE;
    _tx_channel_config.trans_queue_depth = RMT_BUFFER_SIZE;

    // Configure transmission
    _transmit_config.loop_count = 0;                              // No automatic loops - real-time calculation
    _transmit_config.flags.eot_level = _is_bidirectional ? 1 : 0; // Telemetric Bit used as bidir flag

    // Create RMT TX channel
    if (rmt_new_tx_channel(&_tx_channel_config, &_rmt_tx_channel) != DSHOT_OK)
    {
        _dshot_log(TX_INIT_FAILED);
        return DSHOT_ERROR;
    }

    return (rmt_enable(_rmt_tx_channel) == DSHOT_OK);
}

// Initialize RMT RX channel
bool DShotRMT::_initRXChannel()
{
        // Create a queue to receive data from the RX callback
    _rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    if (_rx_queue == nullptr)
    {
        return DSHOT_ERROR;
    }
    
    // Configure RX channel parameters
    _rx_channel_config.gpio_num = _gpio;
    _rx_channel_config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    _rx_channel_config.resolution_hz = DSHOT_RMT_RESOLUTION;
    _rx_channel_config.mem_block_symbols = RX_BUFFER_SIZE;

    // Configure reception parameters
    _receive_config.signal_range_min_ns = 2;
    _receive_config.signal_range_max_ns = 200;

    // Create RMT RX channel
    if (rmt_new_rx_channel(&_rx_channel_config, &_rmt_rx_channel) != DSHOT_OK)
    {
        _dshot_log(RX_INIT_FAILED);
        return DSHOT_ERROR;
    }

    // Register callback for reception
    _rx_event_cbs.on_recv_done = _rmt_rx_done_callback;
    if (rmt_rx_register_event_callbacks(_rmt_rx_channel, &_rx_event_cbs, _rx_queue) != DSHOT_OK)
    {
        _dshot_log(RX_INIT_FAILED);
        return DSHOT_ERROR;
    }
    
    return (rmt_enable(_rmt_rx_channel) == DSHOT_OK);
}

// Callback for RMT reception completion
bool IRAM_ATTR DShotRMT::_rmt_rx_done_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    // Get the queue handle
    QueueHandle_t rx_queue = (QueueHandle_t)user_data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send the event data to the queue
    xQueueSendFromISR(rx_queue, edata, &xHigherPriorityTaskWoken);

    return (xHigherPriorityTaskWoken == pdTRUE);
}

// Initialize DShot encoder
bool DShotRMT::_initDShotEncoder()
{
    // Create copy encoder configuration
    rmt_copy_encoder_config_t encoder_config = {};

    // Create encoder instance
    if (rmt_new_copy_encoder(&encoder_config, &_dshot_encoder) != DSHOT_OK)
    {
        _dshot_log(ENCODER_INIT_FAILED);
        return DSHOT_ERROR;
    }

    return DSHOT_OK;
}

// Send throttle value
bool DShotRMT::sendThrottle(uint16_t throttle)
{
    static uint16_t last_throttle = DSHOT_CMD_MOTOR_STOP;

    // Special case: if throttle is 0, use sendCommand() instead
    if (throttle == 0)
    {
        return sendCommand(DSHOT_CMD_MOTOR_STOP);
    }

    // Log only if throttle is out of range and different from last time
    if ((throttle < DSHOT_THROTTLE_MIN || throttle > DSHOT_THROTTLE_MAX) && throttle != last_throttle)
    {
        _dshot_log(THROTTLE_NOT_IN_RANGE);
    }

    // Always store the original throttle value
    last_throttle = throttle;

    // Constrain throttle for transmission and send
    uint16_t new_throttle = constrain(throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    _packet = _buildDShotPacket(new_throttle);
    return _sendDShotFrame(_packet);
}

// Send DShot command to ESC
bool DShotRMT::sendCommand(uint16_t command)
{
    // Validate command is within DShot specification range
    if (command < DSHOT_CMD_MOTOR_STOP || command > DSHOT_CMD_MAX)
    {
        _dshot_log(COMMAND_NOT_VALID);
        return DSHOT_ERROR;
    }

    // Build packet and transmit
    _packet = _buildDShotPacket(command);
    return _sendDShotFrame(_packet);
}

// Get RPM from ESC (bidirectional mode only)
uint16_t DShotRMT::getERPM()
{
    // Check if bidirectional mode is enabled
    if (!_is_bidirectional)
    {
        _dshot_log(BIDIR_NOT_ENABLED);
        return _last_erpm;
    }

    rmt_rx_done_event_data_t rx_data;
    
    // Wait for data from the RX callback for a certain timeout
    if (xQueueReceive(_rx_queue, &rx_data, pdMS_TO_TICKS(DSHOT_RX_TIMEOUT_MS)) == pdTRUE)
    {
        // Decode the received symbols if a valid frame was received
        if (rx_data.num_symbols > 0)
        {
            _last_erpm = _decodeDShotFrame(rx_data.received_symbols);
        }
    }

    return _last_erpm;
}

// Convert eRPM to actual motor RPM
uint32_t DShotRMT::getMotorRPM(uint8_t magnet_count)
{
    uint8_t pole_pairs = max(1, magnet_count / 2);
    return getERPM() / pole_pairs;
}

// Build a complete DShot packet
dshot_packet_t DShotRMT::_buildDShotPacket(const uint16_t value)
{
    // Initialize packet structure
    dshot_packet_t packet = {};

    // Build packet
    packet.throttle_value = value;
    packet.telemetric_request = _is_bidirectional ? 1 : 0;
    packet.checksum = _calculateCRC(packet);

    return packet;
}

// Parse DShot packet into 16-bit format
uint16_t DShotRMT::_parseDShotPacket(const dshot_packet_t &packet)
{
    uint16_t data = (packet.throttle_value << 1) | packet.telemetric_request;

    // Add CRC checksum
    return (data << 4) | _calculateCRC(packet);
}

// Calculate CRC checksum
uint16_t DShotRMT::_calculateCRC(const dshot_packet_t &packet)
{
    uint16_t data = (packet.throttle_value << 1) | packet.telemetric_request;

    // DShot CRC calculation
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0b0000000000001111;

    // Invert CRC for bidirectional DShot mode
    if (_is_bidirectional)
    {
        crc = (~crc) & 0b0000000000001111;
    }

    return crc;
}

// Transmit DShot packet via RMT
uint16_t DShotRMT::_sendDShotFrame(const dshot_packet_t &packet)
{
        // Check timing requirements
    if (!_timer_signal())
    {
        return DSHOT_ERROR;
    }

    // Enable RX reception before transmission for bidirectional mode
    if (_is_bidirectional)
    {
        rmt_receive(_rmt_rx_channel, _rx_symbols, sizeof(_rx_symbols), &_receive_config);
    }

    // Encode DShot packet into RMT symbols
    _encodeDShotFrame(packet, _tx_symbols);

    // Calculate transmission data size
    size_t tx_size_bytes = DSHOT_BITS_PER_FRAME * sizeof(rmt_symbol_word_t);

    // Perform RMT transmission
    uint16_t result = rmt_transmit(_rmt_tx_channel, _dshot_encoder, _tx_symbols, tx_size_bytes, &_transmit_config);

    if (result != DSHOT_OK)
    {
        return DSHOT_ERROR;
    }
    
    // Update timestamp
    _timer_reset();

    return DSHOT_OK;
}

// Encode DShot packet into RMT symbol format (placed in IRAM for performance)
bool IRAM_ATTR DShotRMT::_encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols)
{
    // Parse packet to 16-bit format
    _parsed_packet = _parseDShotPacket(packet);

    // Convert each bit to RMT symbol
    for (int i = 0; i < DSHOT_BITS_PER_FRAME; i++)
    {
        // Extract bit from packet
        bool bit = (_parsed_packet >> (DSHOT_BITS_PER_FRAME - 1 - i)) & 0b0000000000000001;

        if (_is_bidirectional)
        {
            // Bidirectional DShot uses inverted levels - Idle HIGH
            symbols[i].level0 = 0;
            symbols[i].duration0 = bit ? _timing_config.ticks_one_high : _timing_config.ticks_zero_high;
            symbols[i].level1 = 1;
            symbols[i].duration1 = bit ? _timing_config.ticks_one_low : _timing_config.ticks_zero_low;
        }
        else
        {
            // Standard DShot levels - Idle LOW
            symbols[i].level0 = 1;
            symbols[i].duration0 = bit ? _timing_config.ticks_one_high : _timing_config.ticks_zero_high;
            symbols[i].level1 = 0;
            symbols[i].duration1 = bit ? _timing_config.ticks_one_low : _timing_config.ticks_zero_low;
        }
    }

    return DSHOT_OK;
}

// Decode received RMT symbols
uint16_t DShotRMT::_decodeDShotFrame(const rmt_symbol_word_t *symbols)
{
    uint16_t received_frame = 0;

    // Reconstruct frame from RMT symbols
    for (size_t i = 0; i < DSHOT_BITS_PER_FRAME; ++i)
    {
        // Determine bit value based on pulse duration comparison
        bool bit = symbols[i].duration0 > symbols[i].duration1;
        received_frame = (received_frame << 1) | bit;
    }

    // Extract data and CRC from received frame
    uint16_t received_crc = received_frame & 0b0000000000001111;
    uint16_t data = received_frame >> 4;

    // Calculate expected CRC
    uint16_t calculated_crc = (data ^ (data >> 4) ^ (data >> 8)) & 0b0000000000001111;

    // Validate CRC
    if (received_crc != calculated_crc)
    {
        _dshot_log(CRC_CHECK_FAILED);
        return DSHOT_NULL_PACKET;
    }

    // Remove telemetry bit and return 10-bit value
    return data >> 1;
}

// Check if enough time has passed for next transmission
bool IRAM_ATTR DShotRMT::_timer_signal()
{
    uint32_t current_time = micros();

    // Handle potential overflow
    uint32_t elapsed = current_time - _last_transmission_time;

    return elapsed >= _frame_timer_us;
}

// Reset transmission timer to current time
bool DShotRMT::_timer_reset()
{
    _last_transmission_time = micros();
    return DSHOT_OK;
}

// Print timing diagnostic information to specified stream
void DShotRMT::printDshotInfo(Stream &output) const
{
    output.println(NEW_LINE);
    output.println(" === DShot Signal Info === ");

    // Current DShot mode
    output.printf("Current Mode: DSHOT%d\n",
                  _mode == DSHOT150 ? 150 :
                  _mode == DSHOT300 ? 300 :
                  _mode == DSHOT600 ? 600 :
                  _mode == DSHOT1200  ? 1200 : 0);

    output.printf("Bidirectional: %s\n", _is_bidirectional ? "YES" : "NO");

    // Timing Info
    output.printf("Frame Length: %u us\n", _timing_config.frame_length_us);

    // Packet Info
    output.printf("Current Packet: ");

    // Print bit by bit
    for (int i = 15; i >= 0; --i)
    {
        if ((_parsed_packet >> i) & 1)
        {
            output.print("1");
        }
        else
        {
            output.print("0");
        }
    }
    output.printf("\n");

    output.printf("Current Value: %u\n", _packet.throttle_value);
}

// Print CPU information
void DShotRMT::printCpuInfo(Stream &output) const
{
    output.println(NEW_LINE);
    output.println(" ===  CPU Info  === ");
    output.printf("Chip Model: %s\n", ESP.getChipModel());
    output.printf("Chip Revision: %d\n", ESP.getChipRevision());
    output.printf("CPU Freq = %lu MHz\n", ESP.getCpuFreqMHz());
    output.printf("XTAL Freq = %lu MHz\n", getXtalFrequencyMhz());
    output.printf("APB Freq = %lu Hz\n", getApbFrequency());
}

