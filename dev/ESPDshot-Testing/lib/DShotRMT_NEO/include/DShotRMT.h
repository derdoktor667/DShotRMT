#ifndef _DSHOTRMT_h
#define _DSHOTRMT_h

#include <Arduino.h>

// The RMT (Remote Control) module library is used for generating the DShot signal.
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>

//#include <driver/rmt.h>

// Defines the library version
constexpr auto DSHOT_LIB_VERSION = "0.3.0";

// Constants related to the DShot protocol
constexpr auto DSHOT_CLK_DIVIDER = 8;    // Slow down RMT clock to 0.1 microseconds / 100 nanoseconds per cycle
constexpr auto DSHOT_PACKET_LENGTH = 17; // Last pack is the pause
constexpr auto DSHOT_THROTTLE_MIN = 48; //the smallest value considered to be explicitly a "throttle" value (smaller values are in the enum below)
constexpr auto DSHOT_THROTTLE_MAX = 2047; //the largest value that can be passed back in a single packet
constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;
constexpr auto DSHOT_PAUSE = 21; // 21-bit is recommended
constexpr auto DSHOT_PAUSE_BIT = 16;
constexpr auto F_CPU_RMT = APB_CLK_FREQ;
constexpr auto RMT_CYCLES_PER_SEC = (F_CPU_RMT / DSHOT_CLK_DIVIDER);
constexpr auto RMT_CYCLES_PER_ESP_CYCLE = (F_CPU / RMT_CYCLES_PER_SEC); //not used ATM

constexpr auto RX_SYMBOL_MAX = 11;

// The official DShot Commands
// see https://github.com/betaflight/betaflight/blob/master/src/main/drivers/dshot_command.h //list of commands
// and https://github.com/betaflight/betaflight/blob/master/src/main/drivers/dshot_command.c#L195 //what commands need repeated/delayed
// and https://github.com/betaflight/betaflight/blob/master/src/main/drivers/dshot_command.c#L45 //delay timings
// and https://github.com/bird-sanctuary/extended-dshot-telemetry //dshot docs for bluejay ESC

/*
  DshotSettingRequest (KISS24). Spin direction, 3d and save Settings require 10 requests.. and the TLM Byte must always be high if 1-47 are used to send settings

  3D Mode:
  0 = stop
  48   (low) - 1047 (high) -> negative direction
  1048 (low) - 2047 (high) -> positive direction
 */
typedef enum dshot_cmd_e
{
    DSHOT_CMD_MOTOR_STOP = 0,                   // Stop Motors
    DSHOT_CMD_BEACON1,                          // Wait at least 100000 microseconds before next command
    DSHOT_CMD_BEACON2,                          // Wait at least 100000 microseconds before next command
    DSHOT_CMD_BEACON3,                          // Wait at least 100000 microseconds before next command
    DSHOT_CMD_BEACON4,                          // Wait at least 100000 microseconds before next command
    DSHOT_CMD_BEACON5,                          // Wait at least 100000 microseconds before next command
    DSHOT_CMD_ESC_INFO,                         // Dshot V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,                 // ESC needs >6 repeats, betaflight does x10 repeats
    DSHOT_CMD_SPIN_DIRECTION_2,                 // ESC needs >6 repeats, betaflight does x10 repeats
    DSHOT_CMD_3D_MODE_OFF,                      // ESC needs >6 repeats, betaflight does x10 repeats
    DSHOT_CMD_3D_MODE_ON,                       // ESC needs >6 repeats, betaflight does x10 repeats
    DSHOT_CMD_SETTINGS_REQUEST,                 // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,                    // ESC needs >6 repeats, betaflight does x10 repeats
    DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE,        // ESC needs >6 repeats, betaflight does x10 repeats
    DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE,       // ESC needs >6 repeats, betaflight does x10 repeats
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,       // ESC needs >6 repeats, betaflight does x10 repeats
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,     // ESC needs >6 repeats, betaflight does x10 repeats
    DSHOT_CMD_LED0_ON,                          // BLHeli32 only
    DSHOT_CMD_LED1_ON,                          // BLHeli32 only
    DSHOT_CMD_LED2_ON,                          // BLHeli32 only
    DSHOT_CMD_LED3_ON,                          // BLHeli32 only
    DSHOT_CMD_LED0_OFF,                         // BLHeli32 only
    DSHOT_CMD_LED1_OFF,                         // BLHeli32 only
    DSHOT_CMD_LED2_OFF,                         // BLHeli32 only
    DSHOT_CMD_LED3_OFF,                         // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30,    // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31,          // KISS silent Mode on/Off

    // Commands below only apply to the descrete telemetry line (NOT bidirectional DSHOT)
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE,                    // Need 6x, no wait required. Disables commands 42 to 47
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE,                     // Need 6x, no wait required. Enables commands 42 to 47
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY,            // Need 6x, no wait required. Enables commands 42 to 47, and sends erpm if normal Dshot frame
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY,     // Need 6x, no wait required. Enables commands 42 to 47, and sends erpm period if normal Dshot frame
    // Commands above are only executed when motors are stopped
    DSHOT_CMD_36,                                               // Not yet assigned 
    DSHOT_CMD_37,                                               // Not yet assigned 
    DSHOT_CMD_38,                                               // Not yet assigned 
    DSHOT_CMD_39,                                               // Not yet assigned 
    DSHOT_CMD_40,                                               // Not yet assigned 
    DSHOT_CMD_41,                                               // Not yet assigned 
    // Commands below are executed at any time
    DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY,                // No wait required 
    DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY,                    // No wait required 
    DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY,                    // No wait required 
    DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY,                // No wait required 
    DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY,                       // No wait required
    DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY,                // No wait required (also command 47)

    DSHOT_CMD_MAX = 47,
}dshot_cmd_t;

// Enumeration for the DShot mode
typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

//return states of the RPM getting function
typedef enum dshot_erpm_exit_mode_e
{
    DECODE_SUCCESS = 0,
    ERR_EMPTY_QUEUE,
    ERR_NO_PACKETS,
    ERR_CHECKSUM_FAIL,
    ERR_BIDIRECTION_DISABLED,

} dshot_erpm_exit_mode_t;

//types of data that can come back from the ESC that is not erpm
typedef enum extended_telem_type_e
{
    TYPE_ERPM = 0x1, //due to how extended telemetry works, all odd numbers are this
	TYPE_TEMPRATURE = 0x2,
	TYPE_VOLTAGE = 0x4,
	TYPE_CURRENT = 0x6,
	TYPE_DEBUG_A = 0x8,
	TYPE_DEBUG_B = 0xA,
	TYPE_STRESS_LEVEL = 0xC,
	TYPE_STATUS = 0xE,
}extended_telem_type_t;


// Array of human-readable DShot mode names
static const char *const dshot_mode_name[] =
{
    "DSHOT_OFF",
    "DSHOT150",
    "DSHOT300",
    "DSHOT600",
    "DSHOT1200"
};

typedef String dshot_name_t;


typedef enum telemetric_request_e
{
	NO_TELEMETRIC,
	ENABLE_TELEMETRIC,
} telemetric_request_t;

typedef enum bidirectional_mode_e
{
	NO_BIDIRECTION,
	ENABLE_BIDIRECTION,
} bidirectional_mode_t;


//holds everything the TX callback needs to start the RX callback
typedef struct tx_callback_datapack_s
{
    gpio_num_t gpio_num;

    rmt_channel_handle_t channel_handle; //rx_chan
    rmt_receive_config_t channel_config; //dshot_config.tx_callback_datapack
    rmt_symbol_word_t* raw_symbols; //where the gotten symbols should go
    size_t raw_sym_size; //size of the storage space for the raw symbols


} tx_callback_datapack_t;

//holds everything the RX callback needs to do its thing
typedef struct rx_callback_datapack_s
{
    //thread-safe queue object that the RX callback uses
    QueueHandle_t receive_queue;
} rx_callback_datapack_t;

//holds the actual frame data received from the ESC, this is what's passed back and forth through the xQueue
typedef struct rx_frame_data_s
{
    size_t num_symbols; //how many of the 11 frames are filled with data
    rmt_symbol_word_t received_symbols[RX_SYMBOL_MAX]; //11 frames of received symbols for 21 bits

}rx_frame_data_t;


//Type of Dshot ESC frame
typedef union {
    struct {
        uint16_t crc: 4;       /*!< CRC checksum */
        uint16_t telemetry: 1; /*!< Telemetry request */
        uint16_t throttle: 11; /*!< Throttle value */
    };
    uint16_t val;
} dshot_esc_frame_t;

//all settings for the dshot config
typedef struct dshot_config_s
{
    dshot_mode_t mode; //enum of mode (e.g. DSHOT150)
	dshot_name_t name_str; //string of ^ for user-reading
	bidirectional_mode_t bidirectional; //do we request eRPM frames from the ESC?
    uint16_t num_motor_poles; //how many magnets does the brushless motor have? (essential for calculating RPM, eRPM = x * RPM)
	gpio_num_t gpio_num; //pin the ESC is on
	//uint8_t pin_num;  //redundant?
	//uint8_t mem_block_num; //not needed with the new backend


	uint16_t ticks_per_bit; //ticks for one complete bit cycle
	//uint8_t clk_div; //not needed. we use RMT_CYCLES_PER_SEC now.

	uint16_t ticks_zero_high; //time for the "high" section
	uint16_t ticks_zero_low; //calculated from high and total
	uint16_t ticks_one_high; //ditto
	uint16_t ticks_one_low; //ditto

    uint32_t micros_per_frame; //time for one complete frame of dshot data
    uint32_t micros_per_shortest; //min time


    tx_callback_datapack_t tx_callback_datapack; //a collection of data telling the tx callback how to set up the rx session

    rx_callback_datapack_t rx_callback_datapack; //collection of settings passed to the rx callback


    //channel config (things like GPIO number send buffer, clock resolution live here)
    rmt_channel_handle_t tx_chan;
    rmt_channel_handle_t rx_chan;

    //encoder config (the part we need to push our encoded data out)
    //in the dshot RMT example on the official espidf page uses a fancier encoder, but pre-generating the data
    //and using a simple copy encoder is far easier to track
    rmt_encoder_handle_t copy_encoder;



} dshot_config_t;



class DShotRMT
{
    
    public:
    //constructors and destructors
	DShotRMT(uint8_t pin);
	~DShotRMT();


    //interface commands (with safe defaults)
	void begin(dshot_mode_t dshot_mode = DSHOT_OFF, bidirectional_mode_t is_bidirectional = NO_BIDIRECTION, uint16_t magnet_count = 14);
	void send_dshot_value(uint16_t throttle_value, bool get_onewire_telemetry = true, telemetric_request_t telemetric_request = NO_TELEMETRIC);
    
    //uint16_t get_dshot_RPM();
    //function now returns its fail state to the caller

    //peeks into the queue and gets the response from the ESC
    dshot_erpm_exit_mode_t get_dshot_packet(uint16_t* value, extended_telem_type_t* packetType);
    
    //converts a dshot packet into an equivalent voltage (as specified by dshot specs)
    float convert_packet_to_volts(uint8_t value);

    
    //ratio of passed to failed checksums
    float get_telem_success_rate();

    private:


    //thread-safe queue object that the RX callback uses
    QueueHandle_t receive_queue;

    //new data will be dumped in here from the callback (size doesn't need to be exact, we make it bigger to be safe)
    //(I think it isn't thread safe to touch this while RMT callbacks are running)
    rmt_symbol_word_t dshot_rx_rmt_item[64] = {};

    //where the TX raw frame info lives (to be sent out)
    rmt_symbol_word_t dshot_tx_rmt_item[DSHOT_PACKET_LENGTH] = {};

    //all the settings for setting up the ESC channels 
    dshot_config_t dshot_config;

    //used to determine telemetry success rate when reading values sent from the ESC
    uint32_t successful_packets = 0;
    uint32_t error_packets = 0;

    //rmt_item32_t* encode_dshot_to_rmt(uint16_t parsed_packet); //rmt_symbol_word_t
    void encode_dshot_to_rmt(uint16_t parsed_packet);
	uint16_t calc_dshot_chksum(const dshot_esc_frame_t& dshot_frame);
    uint32_t decode_eRPM_telemetry_value(uint16_t value);
    uint32_t erpmToRpm(uint16_t erpm, uint16_t motorPoleCount);
	//uint16_t prepare_rmt_data(dshot_esc_frame_t& dshot_frame);


    static void handle_error(esp_err_t);


};















#endif
