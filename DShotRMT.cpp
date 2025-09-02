/**
 * @file DShotRMT.cpp
 * @brief DShot signal generation using ESP32 RMT with bidirectional support
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include "DShotRMT.h"

// Timing parameters for each DShot mode
// Format: {frame_length_us, ticks_per_bit, ticks_one_high, ticks_one_low, ticks_zero_high, ticks_zero_low}
static constexpr dshot_timing_t DSHOT_TIMINGS[] = {
    {0, 0, 0, 0, 0, 0},        // DSHOT_OFF
    {128, 64, 48, 16, 24, 40}, // DSHOT150
    {64, 32, 24, 8, 12, 20},   // DSHOT300
    {32, 16, 12, 4, 6, 10},    // DSHOT600
    {16, 8, 6, 2, 3, 5}        // DSHOT1200
};

// Constructor with GPIO number
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool is_bidirectional)
    : _gpio(gpio),
      _mode(mode),
      _is_bidirectional(is_bidirectional),
      _frame_timer_us(0),
      _timing_config(DSHOT_TIMINGS[mode]),
      _last_transmission_time(0),
      _last_erpm(0),
      _parsed_packet(0),
      _packet{0},
      _rmt_tx_channel(nullptr),
      _rmt_rx_channel(nullptr),
      _dshot_encoder(nullptr),
      _tx_channel_config{},
      _rx_channel_config{},
      _transmit_config{},
      _receive_config{},
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

// Destructor for "better" code
DShotRMT::~DShotRMT()
{
    // ...kill them all
    if (_rmt_tx_channel)
    {
        rmt_disable(_rmt_tx_channel);
        rmt_del_channel(_rmt_tx_channel);
    }

    //
    if (_rmt_rx_channel)
    {
        rmt_disable(_rmt_rx_channel);
        rmt_del_channel(_rmt_rx_channel);
    }

    //
    if (_dshot_encoder)
    {
        rmt_del_encoder(_dshot_encoder);
    }

    //
    if (_rx_queue)
    {
        vQueueDelete(_rx_queue);
    }
}

// Initialize DShotRMT
uint16_t DShotRMT::begin()
{
    // Init TX channel
    if (!_initTXChannel())
    {
        _dshot_log(TX_INIT_FAILED);
        return DSHOT_ERROR;
    }

    // Init RX channel
    if (_is_bidirectional)
    {
        if (!_initRXChannel())
        {
            _dshot_log(RX_INIT_FAILED);
            return DSHOT_ERROR;
        }
    }

    // Init DShot encoder
    if (_initDShotEncoder() != DSHOT_OK)
    {
        _dshot_log(ENCODER_INIT_FAILED);
        return DSHOT_ERROR;
    }

    return DSHOT_OK;
}

// Init RMT TX channel
bool DShotRMT::_initTXChannel()
{
    // Configure TX channel
    _tx_channel_config.gpio_num = _gpio;
    _tx_channel_config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    _tx_channel_config.resolution_hz = DSHOT_RMT_RESOLUTION;
    _tx_channel_config.mem_block_symbols = RMT_BUFFER_SYMBOLS;
    _tx_channel_config.trans_queue_depth = RMT_QUEUE_DEPTH;

    // Config RMT TX
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

// Init RMT RX channel
bool DShotRMT::_initRXChannel()
{
    // Create a queue for RX callback data
    _rx_queue = xQueueCreate(RMT_QUEUE_DEPTH, sizeof(rmt_rx_done_event_data_t));
    if (_rx_queue == nullptr)
    {
        return DSHOT_ERROR;
    }

    // Config RMT RX 
    _rx_channel_config.gpio_num = _gpio;
    _rx_channel_config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    _rx_channel_config.resolution_hz = DSHOT_RMT_RESOLUTION;
    _rx_channel_config.mem_block_symbols = RMT_BUFFER_SYMBOLS;

    // Config RMT RX parameters
    _receive_config.signal_range_min_ns = DSHOT_PULSE_MIN;
    _receive_config.signal_range_max_ns = DSHOT_PULSE_MAX;

    // Create RMT RX channel
    if (rmt_new_rx_channel(&_rx_channel_config, &_rmt_rx_channel) != DSHOT_OK)
    {
        _dshot_log(RX_INIT_FAILED);
        return DSHOT_ERROR;
    }

    // Register RX callback
    _rx_event_callbacks.on_recv_done = _rmt_rx_done_callback;

    if (rmt_rx_register_event_callbacks(_rmt_rx_channel, &_rx_event_callbacks, _rx_queue) != DSHOT_OK)
    {
        _dshot_log(RX_INIT_FAILED);
        return DSHOT_ERROR;
    }

    return (rmt_enable(_rmt_rx_channel) == DSHOT_OK);
}

// Callback for RMT RX
bool IRAM_ATTR DShotRMT::_rmt_rx_done_callback(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    // Init RX buffer
    QueueHandle_t rx_queue = (QueueHandle_t)user_data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Copy callback data into RX buffer
    xQueueGenericSendFromISR(rx_queue, edata, &xHigherPriorityTaskWoken, queueSEND_TO_BACK);

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

    // RMT RX event data
    rmt_rx_done_event_data_t rx_data;

    // Wait for data from the RX callback for a certain timeout
    if (xQueueReceive(_rx_queue, &rx_data, pdMS_TO_TICKS(DSHOT_RX_TIMEOUT_MS)) == pdTRUE)
    {
        // Decode the received symbols if a valid frame was received
        if (rx_data.num_symbols > DSHOT_NULL_PACKET)
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
    // Init packet structure
    dshot_packet_t packet = {};

    // Re-check for valid value
    if (value > DSHOT_THROTTLE_MAX)
    {
        _dshot_log(PACKET_BUILD_ERROR);
        
        // Something is really wrong
        return packet;
    }

    // Build packet
    packet.throttle_value = value;
    packet.telemetric_request = _is_bidirectional ? 1 : 0;
    
    // CRC is calculated over 11bit
    uint16_t data = (packet.throttle_value << 1) | packet.telemetric_request;
    
    packet.checksum = _calculateCRC(data);

    return packet;
}

// Parse DShot packet into 16-bit format
uint16_t DShotRMT::_parseDShotPacket(const dshot_packet_t &packet)
{
    // Parse DShot frame into "raw" 16 bit value
    uint16_t data_and_telemetry = (packet.throttle_value << 1) | packet.telemetric_request;
    uint16_t parsed_packet = (data_and_telemetry << 4) | packet.checksum;

    return parsed_packet;
}

// Calculate CRC
uint16_t DShotRMT::_calculateCRC(const uint16_t data)
{
    // DShot CRC
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

    // Enable RMT RX before RMT TX 
    if (_is_bidirectional)
    {
        // Performance reasons
        rmt_symbol_word_t rx_symbols[DSHOT_BITS_PER_FRAME];

        rmt_receive(_rmt_rx_channel, rx_symbols, sizeof(rx_symbols), &_receive_config);

        // Disable RMT RX for sending
        rmt_disable(_rmt_rx_channel);
    }

    // Local for performance
    rmt_symbol_word_t tx_symbols[DSHOT_BITS_PER_FRAME];

    // Encode DShot packet into RMT symbols
    _encodeDShotFrame(packet, tx_symbols);

    // Calculate transmission data size
    size_t tx_size_bytes = DSHOT_BITS_PER_FRAME * sizeof(rmt_symbol_word_t);

    // Perform RMT transmission
    uint16_t result = rmt_transmit(_rmt_tx_channel, _dshot_encoder, tx_symbols, tx_size_bytes, &_transmit_config);

    if (result != DSHOT_OK)
    {
        return DSHOT_ERROR;
    }

    // Re-enable RMT RX
    if (_is_bidirectional)
    {
        if (rmt_enable(_rmt_rx_channel) != DSHOT_OK)
        {
            _dshot_log(RX_RMT_RECEIVER_ERROR);
        }
    }

    // Update timestamp and return success
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
    // DShot answer is GCR encoded?
    // GCR decoding: bit_N = gcr_bit_N ^ gcr_bit_(N-1)
    uint32_t raw_gcr_data = 0;

    // Construct GCR frame from RMT symbols
    for (size_t i = 0; i < DSHOT_BITS_PER_FRAME; ++i)
    {
        // Based on the DShot bidirectional protocol, a logical '1' has a longer low pulse (symbols[i].duration0) than a logical '0'.
        // The logical condition `symbols[i].duration0 > symbols[i].duration1` correctly identifies a logical '1' bit.
        bool bit_is_one = symbols[i].duration0 > symbols[i].duration1;
        raw_gcr_data = (raw_gcr_data << 1) | bit_is_one;
    }

    // GCR decoding over the "throttle" bits
    // GCR encoding rule is: bit_n = gcr_bit_n ^ gcr_bit_(n-1)
    uint16_t gcr_data = (raw_gcr_data >> 5) & 0b000000001111111111;
    uint16_t received_data = gcr_data ^ (gcr_data >> 1);

    // Extract CRC from gcr answer
    uint16_t received_crc = raw_gcr_data & 0b0000000000001111;

    // Calculate expected CRC using the new, centralized function
    uint16_t data_for_crc = (received_data << 1) | 1; // Telemetry request bit is always 1 for bidirectional
    uint16_t calculated_crc = _calculateCRC(data_for_crc);
    
    // Validate CRC
    if (received_crc != calculated_crc)
    {
        _dshot_log(CRC_CHECK_FAILED);
        return DSHOT_NULL_PACKET;
    }

    // The data is eRPM * 100
    return received_data;
}

// Check if enough time has passed for next transmission
bool IRAM_ATTR DShotRMT::_timer_signal()
{
    uint64_t current_time = esp_timer_get_time();

    // Handle potential overflow
    uint64_t elapsed = current_time - _last_transmission_time;

    return elapsed >= _frame_timer_us;
}

// Reset transmission timer to current time
bool DShotRMT::_timer_reset()
{
    _last_transmission_time = esp_timer_get_time();
    return DSHOT_OK;
}

// Print timing diagnostic information to specified stream
void DShotRMT::printDshotInfo(Stream &output) const
{
    output.println(" ");
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
    output.println(" ");
    output.println(" ===  CPU Info  === ");
    output.printf("Chip Model: %s\n", ESP.getChipModel());
    output.printf("Chip Revision: %d\n", ESP.getChipRevision());
    output.printf("CPU Freq = %lu MHz\n", ESP.getCpuFreqMHz());
    output.printf("XTAL Freq = %lu MHz\n", getXtalFrequencyMhz());
    output.printf("APB Freq = %lu Hz\n", getApbFrequency());
}
