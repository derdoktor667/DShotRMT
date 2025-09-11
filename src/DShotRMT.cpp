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
      _last_erpm_atomic(0),
      _telemetry_ready_flag(false),
      _frame_timer_us(0),
      _timing_config(DSHOT_TIMINGS[mode]),
      _last_throttle(DSHOT_CMD_MOTOR_STOP),
      _last_transmission_time(0),
      _parsed_packet(0),
      _packet{0},
      _bitPositions{0},
      _level0(_is_bidirectional ? 0 : 1),
      _level1(_is_bidirectional ? 1 : 0),
      _rmt_tx_channel(nullptr),
      _rmt_rx_channel(nullptr),
      _dshot_encoder(nullptr),
      _tx_channel_config{},
      _rx_channel_config{},
      _transmit_config{},
      _receive_config{}
{
    // Calculate frame timing including switch/pause time
    _frame_timer_us = _timing_config.frame_length_us + DSHOT_PAUSE_US;

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
    // ...TX
    if (_rmt_tx_channel)
    {
        if (rmt_disable(_rmt_tx_channel) == DSHOT_OK)
        {
            rmt_del_channel(_rmt_tx_channel);
            _rmt_tx_channel = nullptr;
        }
    }

    // ...RX
    if (_rmt_rx_channel)
    {
        if (rmt_disable(_rmt_rx_channel) == DSHOT_OK)
        {
            rmt_del_channel(_rmt_rx_channel);
            _rmt_rx_channel = nullptr;
        }
    }

    // ...Encoder
    if (_dshot_encoder)
    {
        rmt_del_encoder(_dshot_encoder);
        _dshot_encoder = nullptr;
    }
}

// Init DShotRMT
dshot_result_t DShotRMT::begin()
{
    // Init RX channel first
    if (_is_bidirectional)
    {
        if (!_initRXChannel().success)
        {
            return {false, RX_INIT_FAILED};
        }
    }

    // Init TX channel
    if (!_initTXChannel().success)
    {
        return {false, TX_INIT_FAILED};
    }

    // Init DShot encoder
    if (!_initDShotEncoder().success)
    {
        return {false, ENCODER_INIT_FAILED};
    }

    // Bit positions precalculation
    _preCalculateBitPositions();

    return {true, INIT_SUCCESS};
}

// Init RMT TX channel
dshot_result_t DShotRMT::_initTXChannel()
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
        return {false, TX_INIT_FAILED};
    }

    //
    if (rmt_enable(_rmt_tx_channel) != DSHOT_OK)
    {
        return {false, TX_INIT_FAILED};
    }

    return {true, TX_INIT_SUCCESS};
}

// Init RMT RX channel
dshot_result_t DShotRMT::_initRXChannel()
{

    // Direct RMT symbol processing - Performance optimized
    _rx_event_callbacks.on_recv_done = _rmt_rx_done_callback;

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
        return {false, RX_INIT_FAILED};
    }

    //
    if (rmt_enable(_rmt_rx_channel) != DSHOT_OK)
    {
        return {false, RX_INIT_FAILED};
    }

    return {true, RX_INIT_SUCCESS};
}

// Callback for RMT RX
bool DShotRMT::_rmt_rx_done_callback(rmt_channel_handle_t rmt_rx_channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    DShotRMT *instance = static_cast<DShotRMT *>(user_data);

    // ISR check for valid data
    if (edata && edata->num_symbols >= GCR_BITS_PER_FRAME && edata->num_symbols <= GCR_BITS_PER_FRAME)
    {

        // Direct decoding
        uint16_t erpm = instance->_decodeDShotFrame(edata->received_symbols);

        if (erpm != DSHOT_NULL_PACKET)
        {
            // Atomic writes - thread-safe
            instance->_last_erpm_atomic = erpm;
            instance->_telemetry_ready_flag = true;
        }
    }

    return false;
}

// Initialize DShot encoder
dshot_result_t DShotRMT::_initDShotEncoder()
{
    // Create copy encoder configuration
    rmt_copy_encoder_config_t encoder_config = {};

    // Create encoder instance
    if (rmt_new_copy_encoder(&encoder_config, &_dshot_encoder) != DSHOT_OK)
    {
        return {false, ENCODER_INIT_FAILED};
    }

    return {true, TX_INIT_SUCCESS};
}

// Send throttle value
dshot_result_t DShotRMT::sendThrottle(uint16_t throttle)
{
    // Special case: if throttle is 0, use sendCommand() instead
    if (throttle == 0)
    {
        return sendCommand(DSHOT_CMD_MOTOR_STOP);
    }

    // Always store the original throttle value
    _last_throttle = throttle;

    // Constrain throttle for transmission and send
    uint16_t new_throttle = constrain(throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

    _packet = _buildDShotPacket(new_throttle);

    return _sendDShotFrame(_packet);
}

// Send DShot command to ESC
dshot_result_t DShotRMT::sendCommand(uint16_t command)
{
    // Validate command is within DShot specification range
    if (command < DSHOT_CMD_MOTOR_STOP || command > DSHOT_CMD_MAX)
    {
        return {false, COMMAND_NOT_VALID};
    }

    // Build packet and transmit
    _packet = _buildDShotPacket(command);

    return _sendDShotFrame(_packet);
}

// Get telemetry data
dshot_result_t DShotRMT::getTelemetry(uint16_t magnet_count)
{
    // Result container with unified structure
    dshot_result_t result = {false, TELEMETRY_FAILED, NO_DSHOT_ERPM, NO_DSHOT_RPM};

    // Check if bidirectional mode is enabled
    if (!_is_bidirectional)
    {
        result.msg = BIDIR_NOT_ENABLED;
        return result;
    }

    //
    if (_telemetry_ready_flag)
    {
        _telemetry_ready_flag = false;

        uint16_t erpm = _last_erpm_atomic;

        //
        if (erpm != DSHOT_NULL_PACKET && magnet_count >= 1)
        {
            uint8_t pole_pairs = max(MIN_POLE_PAIRS, (magnet_count / MAGNETS_PER_POLE_PAIR));
            uint32_t motor_rpm = (erpm / pole_pairs);

            result.success = true;
            result.erpm = erpm;
            result.motor_rpm = motor_rpm;
            result.msg = TELEMETRY_SUCCESS;
        }
    }

    return result;
}

// Build a complete DShot packet
dshot_packet_t DShotRMT::_buildDShotPacket(const uint16_t value)
{
    // Init packet structure
    dshot_packet_t packet = {};

    // Re-check for valid value
    if (value > DSHOT_THROTTLE_MAX)
    {
        // Something is really wrong
        return packet;
    }

    // Build packet
    packet.throttle_value = value & 0b0000011111111111;
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
    uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & DSHOT_CRC_MASK;

    // Invert CRC for bidirectional DShot mode
    if (_is_bidirectional)
    {
        crc = (~crc) & DSHOT_CRC_MASK;
    }

    return crc;
}

// Per calculate bits - Performance optimized
void DShotRMT::_preCalculateBitPositions()
{
    for (int i = 0; i < DSHOT_BITS_PER_FRAME; ++i)
    {
        _bitPositions[i] = DSHOT_BITS_PER_FRAME - 1 - i;
    }
}

// Transmit DShot packet via RMT
dshot_result_t DShotRMT::_sendDShotFrame(const dshot_packet_t &packet)
{
    // Check timing requirements
    if (!_timer_signal())
    {
        return {false, TIMING_CORRECTION};
    }

    // Enable RMT RX before RMT TX
    if (_is_bidirectional)
    {
        // Calculate transmission data size
        size_t rx_size_bytes = GCR_BITS_PER_FRAME * sizeof(rmt_symbol_word_t);

        // Performance reasons
        rmt_symbol_word_t rx_symbols[DSHOT_BITS_PER_FRAME];

        if (rmt_receive(_rmt_rx_channel, rx_symbols, rx_size_bytes, &_receive_config) != DSHOT_OK)
        {
            return {false, RECEIVER_FAILED};
        }
    }

    // Local for performance
    rmt_symbol_word_t tx_symbols[DSHOT_BITS_PER_FRAME];

    // Encode DShot packet into RMT symbols
    _encodeDShotFrame(packet, tx_symbols);

    // Calculate transmission data size
    size_t tx_size_bytes = DSHOT_BITS_PER_FRAME * sizeof(rmt_symbol_word_t);

    // TODO: Find out, why this is needed
    if (_is_bidirectional)
    {
        // Disable RMT RX for sending
        if (rmt_disable(_rmt_rx_channel) != DSHOT_OK)
        {
            return {false, RECEIVER_FAILED};
        }
    }

    // Perform RMT transmission
    if (rmt_transmit(_rmt_tx_channel, _dshot_encoder, tx_symbols, tx_size_bytes, &_transmit_config) != DSHOT_OK)
    {
        return {false, TRANSMISSION_FAILED};
    }

    // Re-enable RMT RX
    if (_is_bidirectional)
    {
        if (rmt_enable(_rmt_rx_channel) != DSHOT_OK)
        {
            return {false, RECEIVER_FAILED};
        }
    }

    // Update timestamp and calculate execution time
    _timer_reset();

    return {true, TRANSMISSION_SUCCESS};
}

// Encode DShot packet into RMT symbol format (placed in IRAM for performance)
bool DShotRMT::_encodeDShotFrame(const dshot_packet_t &packet, rmt_symbol_word_t *symbols)
{
    _parsed_packet = _parseDShotPacket(packet);

    // Decode MSB
    for (int i = 0; i < DSHOT_BITS_PER_FRAME; ++i)
    {
        // Use precalculated bit positions - Performace optimized
        int bit_position = _bitPositions[i];

        bool bit = (_parsed_packet >> bit_position) & 0b0000000000000001;
        symbols[i].level0 = _level0;
        symbols[i].duration0 = bit ? _timing_config.ticks_one_high : _timing_config.ticks_zero_high;
        symbols[i].level1 = _level1;
        symbols[i].duration1 = bit ? _timing_config.ticks_one_low : _timing_config.ticks_zero_low;
    }

    return DSHOT_OK;
}

// Decodes a DShot telemetry frame from received RMT symbols.
uint16_t DShotRMT::_decodeDShotFrame(const rmt_symbol_word_t *symbols)
{
    uint32_t gcr_value = 0;

    // Decode GCR symbols into a 21-bit value.
    // '1' has a longer low pulse (duration0 > duration1).
    // '0' has a longer high pulse (duration1 > duration0).
    for (size_t i = 0; i < GCR_BITS_PER_FRAME; ++i)
    {
        bool bit_is_one = symbols[i].duration0 > symbols[i].duration1;
        gcr_value = (gcr_value << 1) | bit_is_one;
    }

    // Perform GCR decoding: data = gcr ^ (gcr >> 1).
    uint32_t decoded_frame = gcr_value ^ (gcr_value >> 1);

    // Extract 16 data bits and 4 CRC bits from 20-bit frame.
    // The first bit of the GCR frame is a start bit and is discarded.
    uint16_t data_and_crc = (decoded_frame & DSHOT_FULL_PACKET);

    // Cutting 4 bits?
    uint16_t received_data = data_and_crc >> 4;

    // Masking CRC
    uint16_t received_crc = data_and_crc & DSHOT_CRC_MASK;

    // Telemetry request bit has to be 1
    if (!(received_data & (1 << 11)))
    {
        return DSHOT_NULL_PACKET;
    }

    // Calculate expected CRC
    uint16_t data_for_crc = received_data;
    uint16_t calculated_crc = _calculateCRC(data_for_crc);

    // Validate CRC
    if (received_crc != calculated_crc)
    {
        return DSHOT_NULL_PACKET;
    }

    // Return the eRPM value (first 11 bits of received data).
    return received_data & DSHOT_THROTTLE_MAX;
}

// Check if enough time has passed for next transmission
bool DShotRMT::_timer_signal()
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
void DShotRMT::printDShotInfo(Stream &output) const
{
    output.println(" ");
    output.println(" === DShot Signal Info === ");

    // Current DShot mode
    output.printf("Current Mode: DSHOT%d\n", 
        _mode == DSHOT150 ? 150 : 
        _mode == DSHOT300 ? 300 : 
        _mode == DSHOT600 ? 600 : 
        _mode == DSHOT1200 ? 1200 : 0);

    output.printf("Bidirectional: %s\n", _is_bidirectional ? "YES" : "NO");

    // Packet Info
    output.printf("Current Packet: ");

    // Print bit by bit
    for (int i = DSHOT_BITS_PER_FRAME - 1; i >= 0; --i)
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

// --- HELPERS ---
void printDShotResult(dshot_result_t &result, Stream &output)
{
    output.printf("Status: %s - %s", result.success ? "SUCCESS" : "FAILED", result.msg);

    // Print telemetry data if available
    if (result.success && (result.erpm > 0 || result.motor_rpm > 0))
    {
        output.printf(" | eRPM: %u, Motor RPM: %u", result.erpm, result.motor_rpm);
    }

    output.println();
}
