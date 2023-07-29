#ifndef _DSHOTRMT_h
#define _DSHOTRMT_h

#include <Arduino.h>

// The RMT (Remote Control) module library is used for generating the DShot signal.
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>

//#include <driver/rmt.h>

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
static const char *const dshot_mode_name[] =
{
    "DSHOT_OFF",
    "DSHOT150",
    "DSHOT300",
    "DSHOT600",
    "DSHOT1200"
};

typedef String dshot_name_t;


typedef enum telemetric_request_e {
	NO_TELEMETRIC,
	ENABLE_TELEMETRIC,
} telemetric_request_t;


//Type of Dshot ESC frame
typedef union {
    struct {
        uint16_t crc: 4;       /*!< CRC checksum */
        uint16_t telemetry: 1; /*!< Telemetry request */
        uint16_t throttle: 11; /*!< Throttle value */
    };
    uint16_t val;
} dshot_esc_frame_t;


typedef struct dshot_config_s
{
    dshot_mode_t mode; //enum of mode (e.g. DSHOT150)
	dshot_name_t name_str; //string of ^ for user-reading
	bool bidirectional; //do we request eRPM frames from the ESC?
	gpio_num_t gpio_num; //pin the ESC is on
	//uint8_t pin_num;  //redundant?
	//uint8_t mem_block_num; //not needed with the new backend


	uint16_t ticks_per_bit; //ticks for one complete bit cycle
	//uint8_t clk_div; //not needed. we use RMT_CYCLES_PER_SEC now.

	uint16_t ticks_zero_high; //time for the "high" section
	uint16_t ticks_zero_low; //calculated from high and total
	uint16_t ticks_one_high; //ditto
	uint16_t ticks_one_low; //ditto

    //channel config (things like GPIO number send buffer, clock resolution live here)
    rmt_channel_handle_t tx_chan;
    rmt_channel_handle_t rx_chan;

    //encoder config (the part we need to push our encoded data out)
    //in the dshot RMT example on the official espidf page uses a fancier encoder, but pre-generating the data
    //and using a simple copy encoder is far easier to track
    rmt_encoder_handle_t copy_encoder;



} dshot_config_t;


//in the end, I want 4 items:
//constructor(take pin and RMT channel)
    //inside this function, i create the rmt_encoder_configs
    //I install the configs i created

//begin(take dshot mode, bidirectional)
    //inside this function, I enable the channel (rmt_enable)

//sendThrottle(take throttle value, telemetry req) (does not loop)
    //I encode the throttle value
    //I send the throttle value
    //IF bidirectional, on send completed, start RX callback
    //ON RX callback, put gotten data into class storage

//sendCustom(take raw command)
    //see sendThrottle

//read(take nothing)
    //get RX data out of class storage
    //decode RX data



class DShotRMT
{
    
    public:
    //constructors and destructors
	DShotRMT(uint8_t pin);
	~DShotRMT();


    //interface commands (with safe defaults)
	bool begin(dshot_mode_t dshot_mode = DSHOT_OFF, bool is_bidirectional = false);
	void send_dshot_value(uint16_t throttle_value, telemetric_request_t telemetric_request = NO_TELEMETRIC);
    uint16_t get_dshot_RPM();

    static void handle_error(esp_err_t);

    private:

    //where raw telemetry from the ESC lives
    uint16_t bi_telem_buffer;

    //where the raw frame info lives
    rmt_symbol_word_t dshot_tx_rmt_item[DSHOT_PACKET_LENGTH];

    //all the settings for setting up the ESC channels 
    dshot_config_t dshot_config;
    


    //rmt_item32_t* encode_dshot_to_rmt(uint16_t parsed_packet); //rmt_symbol_word_t
    void encode_dshot_to_rmt(uint16_t parsed_packet);
	uint16_t calc_dshot_chksum(const dshot_esc_frame_t& dshot_frame);
	uint16_t prepare_rmt_data(dshot_esc_frame_t& dshot_frame);



};















#endif
