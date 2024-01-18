#include <DShotRMT.h>
#include <hal/gpio_hal.h>

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
	//high task wakeup will be true if pushing data to the queue started a task with a higher priority than this inturrupt.
    BaseType_t high_task_wakeup = pdFALSE;

	//get pointer to the settings passed in by us when we initialized the callback
	rx_callback_datapack_t* config = (rx_callback_datapack_t*)user_data;

	rx_frame_data_t out_data = {};

	//copy the edata symbols to the frame data
	size_t sym_count = edata->num_symbols > RX_SYMBOL_MAX ? RX_SYMBOL_MAX : edata->num_symbols; //cap copy size to 11
	memcpy(&out_data.received_symbols, edata->received_symbols, sym_count * sizeof(rmt_symbol_word_t));
	out_data.num_symbols = sym_count;

    //send the received RMT symbols to the parser task
	xQueueSendFromISR(config->receive_queue, &out_data, &high_task_wakeup);

    return high_task_wakeup == pdTRUE; //return xQueueSendFromISR result (does it wake up a higher task?)
}

TEST_RMT_CALLBACK_ATTR
static bool tx_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
	BaseType_t high_task_wakeup = pdFALSE;
	tx_callback_datapack_t* config = (tx_callback_datapack_t*)user_ctx;
	//size_t symbol_count = edata->num_symbols; //we don't need to know how many symbols we sent

	//restart rx channel
	rmt_enable(config->channel_handle);

	//I don't know why we need to use HAL for this...
	//we can also use gpio_ll to get the same result
	//static gpio_hal_context_t _gpio_hal = {
    //.dev = GPIO_HAL_GET_HW(GPIO_PORT_0)
	//};
	//gpio_hal_od_enable(&_gpio_hal, config->gpio_num);
	
	//enable open drain mode before listening for a response (no need for snap logic here, we only do callbacks in bidirectional mode)
	gpio_ll_od_enable(GPIO_LL_GET_HW(GPIO_PORT_0), config->gpio_num);

	//start listening for a response
	rmt_receive(config->channel_handle, config->raw_symbols, config->raw_sym_size, &config->channel_config);
	
	return high_task_wakeup; //nothing to wake up; no data needs to be send back

	//assert failed: rmt_isr_handle_rx_done rmt_rx.c:505 (offset > rx_chan->mem_off)
}

}


//i need to find a better place for these:
//nibble mapping for GCR
static const unsigned char GCR_encode[16] =
{
	0x19, 0x1B, 0x12, 0x13,
	0x1D, 0x15, 0x16, 0x17,
	0x1A, 0x09, 0x0A, 0x0B,
	0x1E, 0x0D, 0x0E, 0x0F
};

//  5 bits > 4 bits (0xff => invalid)
static const unsigned char GCR_decode[32] =
{
	0xFF, 0xFF, 0xFF, 0xFF, // 0 - 3
	0xFF, 0xFF, 0xFF, 0xFF, // 4 - 7
	0xFF, 9, 10, 11, // 8 - 11
	0xFF, 13, 14, 15, // 12 - 15

	0xFF, 0xFF, 2, 3, // 16 - 19
	0xFF, 5, 6, 7, // 20 - 23
	0xFF, 0, 8, 1, // 24 - 27
	0xFF, 4, 12, 0xFF, // 28 - 31
};


//populate the config struct with relavent data
DShotRMT::DShotRMT(uint8_t pin)
{
	dshot_config.gpio_num = (gpio_num_t)pin;
}

//clear constructed variables
DShotRMT::~DShotRMT()
{
	//TODO: more RMT stuff needs to be cleaned out

	if(dshot_config.bidirectional)
	{

		rmt_del_channel(dshot_config.rx_chan);
		vQueueDelete(receive_queue);
	}

	rmt_del_channel(dshot_config.tx_chan); //de-allocate channel
	rmt_del_encoder(dshot_config.copy_encoder); //de-allocate encoder
}



//apply the settings to the RMT backend
void DShotRMT::begin(dshot_mode_t dshot_mode, bidirectional_mode_t is_bidirectional, uint16_t magnet_count)
{

	//populate bidirection
	if(dshot_mode == DSHOT150) //dshot 150 does not support bidirection
		dshot_config.bidirectional = NO_BIDIRECTION;
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
		//note that the ESC responds with a slightly higher bitrate of 5/4, so timing issues may happen due to my nasty math here (probably not though)
		//frame count * (ticks / 10) * 1000 for ms
		dshot_config.micros_per_frame = 16 * dshot_config.ticks_per_bit * 110; //110/100: 110% longer
		//shortest length * 90% * 100
		dshot_config.micros_per_shortest = dshot_config.ticks_zero_high * 90; //90/100: 90% shoter



		//calc low signal timing
		dshot_config.ticks_zero_low = (dshot_config.ticks_per_bit - dshot_config.ticks_zero_high);
		dshot_config.ticks_one_low = (dshot_config.ticks_per_bit - dshot_config.ticks_one_high);

	}

	//pass motor pole count off to settings
	dshot_config.num_motor_poles = magnet_count;

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
				.invert_in = 1, //invert the input logic
				.io_loop_back = 1,	//enable loopback			
				}
		};

		//populate channel object
		handle_error(rmt_new_rx_channel(&rx_chan_config, &dshot_config.rx_chan));

		
		//configure recieve callbacks
		rmt_rx_event_callbacks_t callback = 
		{
			.on_recv_done = rx_done_callback
		};
	
		// create a thread safe queue handle
		receive_queue = xQueueCreate(1, sizeof(rx_frame_data_t));

		//stow settings into datapack so we can get them in the callback
		dshot_config.rx_callback_datapack.receive_queue = receive_queue;

		//register them
		handle_error(rmt_rx_register_event_callbacks(dshot_config.rx_chan, &callback, &dshot_config.rx_callback_datapack));


		//setup the timeouts for the reception (must change depending on DSHOT mode)
		dshot_config.tx_callback_datapack.channel_config =
		{
			.signal_range_min_ns=dshot_config.micros_per_shortest,
			.signal_range_max_ns=dshot_config.micros_per_frame
		};
		//copy the pointer for the channel handle so we can start it in the TX callback
		dshot_config.tx_callback_datapack.channel_handle = dshot_config.rx_chan; 
		//where the rx data should go
		dshot_config.tx_callback_datapack.raw_symbols = dshot_rx_rmt_item;
		//how big this buffer is
		dshot_config.tx_callback_datapack.raw_sym_size = sizeof(rmt_symbol_word_t) * DSHOT_PACKET_LENGTH;
		//what pin this is running on (for flip-flopping open drain mode)
		dshot_config.tx_callback_datapack.gpio_num = dshot_config.gpio_num;


	}

	//populate channel object
	handle_error(rmt_new_tx_channel(&tx_chan_config, &dshot_config.tx_chan));


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
		handle_error(rmt_tx_register_event_callbacks(dshot_config.tx_chan, &callback, &dshot_config.tx_callback_datapack));


		//enable RX channel (may need to remove this, we disable the channel right away when we go to send a packet)
		handle_error(rmt_enable(dshot_config.rx_chan));

	}

	//enable TX channel
	handle_error(rmt_enable(dshot_config.tx_chan));


	//make a copy encoder to push out the array of rmt_symbol_word_t we make when we process a dshot frame
	rmt_copy_encoder_config_t copy_encoder_config = {}; //nothing to configure
	handle_error(rmt_new_copy_encoder(&copy_encoder_config, &dshot_config.copy_encoder));

}

//encode and send a throttle value
void DShotRMT::send_dshot_value(uint16_t throttle_value, bool get_onewire_telemetry, telemetric_request_t telemetric_request)
{
	dshot_esc_frame_t dshot_frame = {};

	//keep packet size within valid range
	if (throttle_value > DSHOT_THROTTLE_MAX)
		throttle_value = DSHOT_THROTTLE_MAX;

	//setup dshot frame
	dshot_frame.throttle = throttle_value;
	dshot_frame.telemetry = telemetric_request; //NOT bidirectional telemetry (requests telemetry over second wire, which is not implemented)
	dshot_frame.crc = calc_dshot_chksum(dshot_frame);

	encode_dshot_to_rmt(dshot_frame.val); //we can pull the compiled frame out with "val"

	//disable open drain mode before sending this out
	gpio_ll_od_disable(GPIO_LL_GET_HW(GPIO_PORT_0), dshot_config.gpio_num);

	//may want to put this inside the begin() function...
	rmt_transmit_config_t tx_config =
	{
        //.loop_count = -1, // infinite loop
    };

	//shut down the rmt reciever before we send a packet out to protect against self-sniffing
	//this is what's used low-level, but rmt_rx_disable should be good enough for our needs
	//rmt_ll_rx_enable((rmt_dev_t*)hal->regs, dshot_config.rx_chan.channel_id, false); //ALTER (cast)
	if(dshot_config.bidirectional == ENABLE_BIDIRECTION)
		handle_error(rmt_disable(dshot_config.rx_chan));
	//run this in the tx_done callback
	//handle_error(rmt_enable(dshot_config.rx_chan));


	//send the frame off to the pin
	rmt_transmit(dshot_config.tx_chan,
				dshot_config.copy_encoder,
				&dshot_tx_rmt_item,
				sizeof(rmt_symbol_word_t) * DSHOT_PACKET_LENGTH,
				&tx_config);


}


//convert the dshot frame (WITHOUT checksum) into some form of erpm (not sure what units yet...)
//stolen from betaflight (src/main/drivers/dshot.c)
uint32_t DShotRMT::decode_eRPM_telemetry_value(uint16_t value)
{
    // eRPM range
    if (value == 0x0fff) {
        return 0;
    }

    // Convert value to 16 bit from the GCR telemetry format (eeem mmmm mmmm)
    value = (value & 0x01ff) << ((value & 0xfe00) >> 9);
    if (!value) {
        return 0;
    }

    // Convert period to erpm * 100
    return (1000000 * 60 / 100 + value / 2) / value;
}


//stolen from betaflight (src/main/drivers/dshot.c)
// Used with serial esc telem as well as dshot telem
uint32_t DShotRMT::erpmToRpm(uint16_t erpm, uint16_t motorPoleCount)
{
    //  rpm = (erpm * 100) / (motorConfig()->motorPoleCount / 2)
    return (erpm * 200) / motorPoleCount;
}


//read back the value in the buffer
dshot_erpm_exit_mode_t DShotRMT::get_dshot_packet(uint16_t* value, extended_telem_type_t* packetType)
{

	if(dshot_config.bidirectional != ENABLE_BIDIRECTION)
	{
		return ERR_BIDIRECTION_DISABLED;
	}

	//only process new data if we have new data waiting in the queue
	rx_frame_data_t rx_data;
	if(xQueueReceive(receive_queue, &rx_data, 0))
	{

		//only execute if we got a packet (no packet response gives us only one symbol by default)
		if (rx_data.num_symbols > 1)
		{


			// Serial.println("===============================");
			// for (int i = 0; i < rx_data.num_symbols; ++i)
			// {
			// 	char hold[100] = {};
			// 	sprintf(hold, "D0: %d L0: %d || D1: %d L1: %d",
			// 	rx_data.received_symbols[i].duration0, rx_data.received_symbols[i].level0,
			// 	rx_data.received_symbols[i].duration1, rx_data.received_symbols[i].level1);
			// 	Serial.println(hold);
			// }
			// Serial.println("===============================");
			


			int i,j;

			//experimental data shows that I get telemetry back with the following timing:
			//dshot 300 = 25 ticks per bit
			//dshot 600 = 12.5 ticks per bit
			//dshot 1200 = 6.25 ticks per bit
			//this closely matches dshot_config.ticks_one_high

			//with 600, recieved packets can have veriety that simultaneously renders the 90% reduction needed and erronious
			//(we get ticks with time 30 and time 40. Where do these go?)

			//bit time is reception bitrate * 90%
			unsigned short bitTime = dshot_config.ticks_one_high * 9 / 10;
			unsigned short bitCount0 = 0;
			unsigned short bitCount1 = 0;
			unsigned short bitShiftLevel = 20;//21 bits, including 0
			unsigned int assembledFrame = 0;

			//unsigned int* y = &assembledFrame; //for debugging on the computer
			for (i = 0; i < rx_data.num_symbols; ++i)
			{

				//for each symbol, see how many bits are in there
				bitCount0 = rx_data.received_symbols[i].duration0 / bitTime;
				bitCount1 = rx_data.received_symbols[i].duration1 / bitTime;

				//if we know the level of the first part of the symbol,
				//we know the second part must be different
				//if it is a 0, we don't have to shift bits because they are all initialized as 0
				if (rx_data.received_symbols[i].level0 == 0)
				{

					bitShiftLevel -= bitCount0;
					for (j = 0; j < bitCount1; ++j)
					{
						assembledFrame |= 1 << bitShiftLevel;
						--bitShiftLevel;
					}
				}
				else
				{
					//shift back any '1' values
					for (j = 0; j < bitCount0; ++j)
					{
						assembledFrame |= 1 << bitShiftLevel;
						--bitShiftLevel;
					}
					bitShiftLevel -= bitCount1;

				}

			}
			//this only needed to happen if the base logic level is '1'.
			//If we invert the input logic with RMT, we don't need to do this (esc response sends inverted logic, a HIGH == 0)
			//spaces come pre-filled with 0
			//fill any remaining space with '1's
			// for (i = bitShiftLevel; i >= 0; --i)
			// {
			// 	assembledFrame |= 1 << i;
			// }


			//decode the data from its transmissive state
			//it doesn't matter if we invert the input or not, this will result in the same number
			assembledFrame = (assembledFrame ^ (assembledFrame >> 1));
			//assembledFrame = 0b11010100101111010110; //test frame from dshot docs

			unsigned char nibble = 0;
			unsigned char fiveBitSubset = 0;
			unsigned int decodedFrame = 0;
			//y = &decodedFrame;
			//remove GCR encoding
			for (i = 0; i < 4; ++i)
			{
				//bitmask out the encoded quintuple
				fiveBitSubset = (assembledFrame >> (i * 5)) & 0b11111;//shift over in sets of 5

				//use a lookup table to get the corresponding nibble
				nibble = GCR_decode[fiveBitSubset];
				//append nibble to the frame
				decodedFrame |= nibble << (i * 4);


			}

			//mask out componets of the frame
			uint16_t frameData = (decodedFrame >> 4) & (0b111111111111);
			uint8_t crc = decodedFrame & (0b1111);
			uint8_t alsocrc = (~(frameData ^ (frameData >> 4) ^ (frameData >> 8))) & 0x0F;

			//stop processing if the checksum is invalid
			if(crc != alsocrc)
			{
				error_packets += 1; //for now, the only error packets we will track are the ones where the checksum fails
				//we don't update the RPM pointer that was passed to us
				return ERR_CHECKSUM_FAIL;
			}

			//test
			//frameData = 0b001001001000;

			//determine packet type
			if (frameData & 0b000100000000 || (~frameData & 0b111100000000) == 0b111100000000) //is erpm packet (4th bit is 1 or all four bits are 0)
			{
				//update internal values
				*packetType = TYPE_ERPM;

				//update output pointer
				*value = erpmToRpm(decode_eRPM_telemetry_value(frameData), dshot_config.num_motor_poles);

			}
			else //is extended telemetry packet
			{
				*packetType = (extended_telem_type_t)((frameData >> 8) & 0b1111); //we return this value anyway, so we reuse it
				*value = (frameData & 0b11111111);

				Serial.println("Other");

				//not needed, we leave this logic up to the implementer
				//uint8_t response_data = (frameData & 0b11111111);
				//switch packet type
				// switch (*packetType)
				// {
				// case TYPE_TEMPRATURE: //temprature in degrees C
				// case TYPE_VOLTAGE: //0.25 volts per step
				// case TYPE_CURRENT: //current in ANP
				// 	*value = response_data;
				// 	break;

				// default:
				// case TYPE_DEBUG_A: //do nothing for these for now
				// case TYPE_DEBUG_B:
				// case TYPE_STRESS_LEVEL:
				// case TYPE_STATUS:
				// 	break;
					
				// }
			}


		}
		else
		{
			return ERR_NO_PACKETS;
		}
	}
	else
	{
		return ERR_EMPTY_QUEUE;
	}

	successful_packets += 1;
	return DECODE_SUCCESS;
}

//converts the value recieved from the dshot packet into a voltage
float DShotRMT::convert_packet_to_volts(uint8_t value)
{
	return (float)value * 0.25;
}


//return the percent of the dshot RPM requests that succeeded
float DShotRMT::get_telem_success_rate()
{
	return (float)successful_packets / (float)(error_packets + successful_packets);
}


//private functions below:

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
    //start with two empty containers
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


