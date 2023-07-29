
#include <DShotRMT.h>

//#include <Arduino.h>


//callbacks need to be in C and not part of any class
extern "C"
{

//ensures that the rx callback code is always in iram, which is essential for speed
#define CONFIG_RMT_ISR_IRAM_SAFE 1
#if CONFIG_RMT_ISR_IRAM_SAFE
#define TEST_RMT_CALLBACK_ATTR IRAM_ATTR
#else
#define TEST_RMT_CALLBACK_ATTR
#endif

//callback when we get data back in
//flag the selected code to be always loaded into RAM
TEST_RMT_CALLBACK_ATTR
static bool rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
	BaseType_t high_task_wakeup = pdFALSE;
	size_t symbol_count = edata->num_symbols;
	edata->received_symbols;
	
	return false;
}


}

//populate the config struct with relavent data
DShotRMT::DShotRMT(uint8_t pin)
{
	dshot_config.gpio_num = (gpio_num_t)pin;
}

//clear constructed variables
DShotRMT::~DShotRMT()
{
	rmt_del_channel(dshot_config.tx_chan); //de-allocate channel
	rmt_del_encoder(dshot_config.copy_encoder); //de-allocate encoder
}



//apply the settings to the RMT backend
bool DShotRMT::begin(dshot_mode_t dshot_mode, bool is_bidirectional)
{

	bool bError = false;

	//populate bidirection
	dshot_config.bidirectional = is_bidirectional;

	//populate mode section
	dshot_config.mode = dshot_mode;
	dshot_config.name_str = dshot_mode_name[dshot_mode];
	//setup dshot timings
	{
		switch (dshot_config.mode)
		{
		case DSHOT150:
			dshot_config.ticks_per_bit = 64; // ...Bit Period Time 6.67 us
			dshot_config.ticks_zero_high = 24; // ...zero time 2.50 us
			dshot_config.ticks_one_high = 48; // ...one time 5.00 us
			break;

		case DSHOT300:
			dshot_config.ticks_per_bit = 32; // ...Bit Period Time 3.33 us
			dshot_config.ticks_zero_high = 12; // ...zero time 1.25 us
			dshot_config.ticks_one_high = 24; // ...one time 2.50 us
			break;

		case DSHOT600:
			dshot_config.ticks_per_bit = 16; // ...Bit Period Time 1.67 us
			dshot_config.ticks_zero_high = 6; // ...zero time 0.625 us
			dshot_config.ticks_one_high = 12; // ...one time 1.25 us
			break;

		case DSHOT1200:
			dshot_config.ticks_per_bit = 8; // ...Bit Period Time 0.83 us
			dshot_config.ticks_zero_high = 3; // ...zero time 0.313 us
			dshot_config.ticks_one_high = 6; // ...one time 0.625 us
			break;

		//play it safe, have a default setting
		default:
			dshot_config.ticks_per_bit = 0; // ...Bit Period Time endless
			dshot_config.ticks_zero_high = 0; // ...no bits, no time
			dshot_config.ticks_one_high = 0; // ......no bits, no time
			break;
		}

		//calc low signal timing
		dshot_config.ticks_zero_low = (dshot_config.ticks_per_bit - dshot_config.ticks_zero_high);
		dshot_config.ticks_one_low = (dshot_config.ticks_per_bit - dshot_config.ticks_one_high);

	}


	//holder for tx channel settings until we install them
	rmt_tx_channel_config_t tx_chan_config =
	{
        .gpio_num = dshot_config.gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution (apb)
        .resolution_hz = RMT_CYCLES_PER_SEC, // esc resolution in Hz
        .mem_block_symbols = 64, // default count per channel
        .trans_queue_depth = 10 // set the number of transactions that can be pending in the background
    };


	//we have 2 different if statements with the same condition because enabling loopback is order sensitive
	//the RX channel must be installed first
	if(dshot_config.bidirectional)
	{
		//other tx configs that need to be enabled if loopback is true:
		tx_chan_config.flags.io_od_mode = 1; //enable open drain
		tx_chan_config.flags.io_loop_back = 1; //enable loopback


		//holder for rx channel settings until we install them
		rmt_rx_channel_config_t rx_chan_config = 
		{
			.gpio_num = dshot_config.gpio_num,
			.clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution (apb)
			.resolution_hz = RMT_CYCLES_PER_SEC, // esc resolution in Hz
			.mem_block_symbols = 64, // default count per channel
			.flags =
				{
				.io_loop_back = 1,	//enable loopback			
				}
		};

		//populate channel object
		if(rmt_new_rx_channel(&rx_chan_config, &dshot_config.rx_chan) != 0)
			bError = true;

		//configure recieve callbacks
		rmt_rx_event_callbacks_t callback = 
		{
			.on_recv_done = rx_done_callback
		};
	
		//register them (unfinished)
		//if(rmt_rx_register_event_callbacks(dshot_config.rx_chan, &callback, ) != 0)
		//	bError = true;

	}



	//populate channel object
	if(rmt_new_tx_channel(&tx_chan_config, &dshot_config.tx_chan) != 0)
		bError = true;


	//additional settings if bidirectional
	if(dshot_config.bidirectional)
	{
		//register callback for the TX channel

		//enable RX channel
	}

	//enable TX channel
	rmt_enable(dshot_config.tx_chan);


	//make a copy encoder to push out the array of rmt_symbol_word_t we make when we process a dshot frame
	rmt_copy_encoder_config_t copy_encoder_config = {}; //nothing to configure
	rmt_new_copy_encoder(&copy_encoder_config, &dshot_config.copy_encoder);



	return bError;
}

//encode and send a throttle value
void DShotRMT::send_dshot_value(uint16_t throttle_value, telemetric_request_t telemetric_request)
{
	dshot_esc_frame_t dshot_frame = {};

	//keep throttle within valid range
	if (throttle_value < DSHOT_THROTTLE_MIN)
		throttle_value = DSHOT_THROTTLE_MIN;

	if (throttle_value > DSHOT_THROTTLE_MAX)
		throttle_value = DSHOT_THROTTLE_MAX;

	//setup dshot frame
	dshot_frame.throttle = throttle_value;
	dshot_frame.telemetry = telemetric_request;
	dshot_frame.crc = calc_dshot_chksum(dshot_frame);

	encode_dshot_to_rmt(dshot_frame.val); //we can pull the compiled frame out with "val"

	//may want to put this inside the begin() function...
	rmt_transmit_config_t tx_config =
	{
        //.loop_count = -1, // infinite loop
    };


	//char formattedOut[30] = {};
	//sprintf(formattedOut, "%d || %x", sizeof(rmt_symbol_word_t) * DSHOT_PACKET_LENGTH, dshot_frame.val);
	//Serial.println(formattedOut);

	//send the frame off to the pin
	rmt_transmit(dshot_config.tx_chan,
				dshot_config.copy_encoder,
				&dshot_tx_rmt_item,
				sizeof(rmt_symbol_word_t) * DSHOT_PACKET_LENGTH,
				&tx_config);




}

//read back the value in the buffer
uint16_t DShotRMT::get_dshot_RPM()
{
	return bi_telem_buffer;
}


//take dshot bits and create an array of rmt clocks
//(we don't need to return anything because we fiddle with the class-baked array, accessible everywhere)
void DShotRMT::encode_dshot_to_rmt(uint16_t parsed_packet)
{
	//shake through all bits in the dshot packet, set up the rmt item based on 1 or 0
	for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1) 
	{
		if (parsed_packet & 0b1000000000000000)
		{
			// set one
			dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_one_high;
			dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_one_low;
		}
		else
		{
			// set zero
			dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_zero_high;
			dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_zero_low;
		}
		dshot_tx_rmt_item[i].level0 = 1;
		dshot_tx_rmt_item[i].level1 = 0;
	}

	//set up pause bit (forced space between dshot frames)
	dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level0 = 0;
	dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level1 = 1;

	dshot_tx_rmt_item[DSHOT_PAUSE_BIT].duration1 = (DSHOT_PAUSE / 2);


}

// ...just returns the checksum
// DOES NOT APPEND CHECKSUM!!!
//take the dshot frame and calulate its checksum
uint16_t DShotRMT::calc_dshot_chksum(const dshot_esc_frame_t &dshot_frame)
{
    //start with two emprty containers
	uint16_t packet = DSHOT_NULL_PACKET;
	uint16_t chksum = DSHOT_NULL_PACKET;

	//same initial 12bit data for bidirectional or "normal" mode
	packet = (dshot_frame.throttle << 1) | dshot_frame.telemetry;

	if (dshot_config.bidirectional) {
		//calc the checksum "inverted" / bidirectional mode
		chksum = (~(packet ^ (packet >> 4) ^ (packet >> 8))) & 0x0F;
	} else {
		//calc the checksum "normal" mode
		chksum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
	}

	return chksum;
}


//this function is redundant thanks to the power of the union
//combine dshot componets to dshot-ready frame
uint16_t DShotRMT::prepare_rmt_data(dshot_esc_frame_t &dshot_frame)
{
	//placeholder
	uint16_t prepared_to_encode = DSHOT_NULL_PACKET;
	//get checksum from frame
	uint16_t chksum = calc_dshot_chksum(dshot_frame);

	dshot_frame.crc = chksum;
    //combine checksum and dshot frame
	//prepared_to_encode = (dshot_frame.throttle << 1) | dshot_frame.telemetry;
	//prepared_to_encode = (prepared_to_encode << 4) | chksum;
	//we don't need to do this because it is a union:
	

	return dshot_frame.val;//prepared_to_encode;
}




void DShotRMT::handle_error(esp_err_t err_code) {
	if (err_code != ESP_OK) {
		Serial.print("error: ");
		Serial.println(esp_err_to_name(err_code));
	}
}



