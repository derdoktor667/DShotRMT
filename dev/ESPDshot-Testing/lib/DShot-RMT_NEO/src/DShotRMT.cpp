
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
	//init return value
    BaseType_t high_task_wakeup = pdFALSE;
	//get pointer to freeRTOS queue handle (passed in by us when we initialized the callback)
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    //send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);

    return high_task_wakeup == pdTRUE; //return xQueueSendFromISR result (does it wake up a higher task?)
}

TEST_RMT_CALLBACK_ATTR
static bool tx_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
	BaseType_t high_task_wakeup = pdFALSE;
	tx_callback_datapack_t* config = (tx_callback_datapack_t*)user_ctx;
	//size_t symbol_count = edata->num_symbols; //we don't need to know how many symbols we sent

	//start listening for a response
	rmt_receive(config->channel_handle, config->raw_symbols, config->raw_sym_size, &config->channel_config);
	
	return high_task_wakeup; //nothing to wake up; no data needs to be send back
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
	//TODO: more RMT stuff needs to be cleaned out
}



//apply the settings to the RMT backend
bool DShotRMT::begin(dshot_mode_t dshot_mode, bool is_bidirectional)
{

	bool bError = false;

	//populate bidirection
	if(dshot_mode == DSHOT150) //dshot 150 does not support bidirection
		dshot_config.bidirectional = false;
	else
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
			//dshot_config.micros_per_frame = 106720 * 110 / 100; //dshot specs are at 106720 (110% on timing: 117392)
			//dshot_config.micros_per_shortest = dshot_config.ticks_zero_high * 90/100; //

			break;

		case DSHOT300:
			dshot_config.ticks_per_bit = 32; // ...Bit Period Time 3.33 us
			dshot_config.ticks_zero_high = 12; // ...zero time 1.25 us
			dshot_config.ticks_one_high = 24; // ...one time 2.50 us
			//dshot_config.micros_per_frame = 58608; //dshot specs are at 53280 (110% on timing: 58608)
			break;

		case DSHOT600:
			dshot_config.ticks_per_bit = 16; // ...Bit Period Time 1.67 us
			dshot_config.ticks_zero_high = 6; // ...zero time 0.625 us
			dshot_config.ticks_one_high = 12; // ...one time 1.25 us
			//dshot_config.micros_per_frame = 29392; //dshot specs are at 26720 (110% on timing: 29392)
			break;

		case DSHOT1200:
			dshot_config.ticks_per_bit = 8; // ...Bit Period Time 0.83 us
			dshot_config.ticks_zero_high = 3; // ...zero time 0.313 us
			dshot_config.ticks_one_high = 6; // ...one time 0.625 us
			//dshot_config.micros_per_frame = 14608; //dshot specs are at 13280 (110% on timing: 14608)
			break;

		//play it safe, have a default setting
		default:
			dshot_config.ticks_per_bit = 0; // ...Bit Period Time endless
			dshot_config.ticks_zero_high = 0; // ...no bits, no time
			dshot_config.ticks_one_high = 0; // ......no bits, no time
			//dshot_config.micros_per_frame = 0; // no dshot specs
			break;
		}

		//set up parameters for dshot length filtering (I went 10% beyond real vals for leniency)
		//frame count * (ticks / 10) * 1000 for ms
		dshot_config.micros_per_frame = 16 * dshot_config.ticks_per_bit * 110; //110/100: 110% longer
		//shortest length * 90% * 100
		dshot_config.micros_per_shortest = dshot_config.ticks_zero_high * 90; //90/100: 90% shoter



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
		tx_chan_config.flags.invert_out = 1; //invert output signal

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
	
		// create a thread safe queue handle
  		receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));

		//register them
		if(rmt_rx_register_event_callbacks(dshot_config.rx_chan, &callback, receive_queue) != 0)
			bError = true;


		//setup the timeouts for the reception (must change depending on DSHOT mode)



		dshot_config.receive_config.channel_config =
		{
			.signal_range_min_ns=dshot_config.micros_per_shortest,
			.signal_range_max_ns=dshot_config.micros_per_frame
		};
		//copy the pointer for the channel handle so we can start it in the TX callback
		dshot_config.receive_config.channel_handle = dshot_config.rx_chan; 
		//where the rx data should go
		dshot_config.receive_config.raw_symbols = dshot_rx_rmt_item;
		//how big this buffer is
		dshot_config.receive_config.raw_sym_size = sizeof(rmt_symbol_word_t) * DSHOT_PACKET_LENGTH;


	}



	//populate channel object
	if(rmt_new_tx_channel(&tx_chan_config, &dshot_config.tx_chan) != 0)
		bError = true;


	//additional settings if bidirectional
	if(dshot_config.bidirectional)
	{
		//register callback for the TX channel
		rmt_tx_event_callbacks_t callback = 
		{
			.on_trans_done = tx_done_callback
		};

		//tx event will be set on the TX channel, with "callback" config and fed the timing config for the RX reception
		//RX reception will be enabled as soon as the TX transmission is over
		if(rmt_tx_register_event_callbacks(dshot_config.tx_chan, &callback, &dshot_config.receive_config) != 0)
			bError = true;


		//enable RX channel
		rmt_enable(dshot_config.rx_chan);

	}

	//enable TX channel
	rmt_enable(dshot_config.tx_chan);


	//make a copy encoder to push out the array of rmt_symbol_word_t we make when we process a dshot frame
	rmt_copy_encoder_config_t copy_encoder_config = {}; //nothing to configure
	rmt_new_copy_encoder(&copy_encoder_config, &dshot_config.copy_encoder);


	//try enabling internal pullup on top of open drain mode (doesn't work, resistor value is too large 10k vs the needed 1k)
	//if(dshot_config.bidirectional)
	//	if(gpio_pullup_en(dshot_config.gpio_num) != 0)
	//		bError = true;


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

	//only process new data if we have new data waiting in the queue
	rmt_rx_done_event_data_t rx_data;
	if(xQueueReceive(receive_queue, &rx_data, 0))
	{

		if(rx_data.num_symbols > 1)
		{
			//TODO: decompile processed data
			//parse data here
			Serial.println("I got data");
			Serial.println("=========================");

			for(int i = 0; i < rx_data.num_symbols; ++i)
			{
				char outarr[100] = {};
				sprintf(outarr, "Num %d || D0: %d || D1: %d", i,
					rx_data.received_symbols[i].duration0,
					rx_data.received_symbols[i].duration1);
				Serial.println(outarr);

			}

			Serial.println("=========================");


		}
	}

	return processed_data;
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
// uint16_t DShotRMT::prepare_rmt_data(dshot_esc_frame_t &dshot_frame)
// {
// 	//placeholder
// 	uint16_t prepared_to_encode = DSHOT_NULL_PACKET;
// 	//get checksum from frame
// 	uint16_t chksum = calc_dshot_chksum(dshot_frame);
// 	dshot_frame.crc = chksum;
//     //combine checksum and dshot frame
// 	//prepared_to_encode = (dshot_frame.throttle << 1) | dshot_frame.telemetry;
// 	//prepared_to_encode = (prepared_to_encode << 4) | chksum;
// 	//we don't need to do this because it is a union:
// 	return dshot_frame.val;//prepared_to_encode;
// }





void DShotRMT::handle_error(esp_err_t err_code) {
	if (err_code != ESP_OK) {
		Serial.print("error: ");
		Serial.println(esp_err_to_name(err_code));
	}
}



