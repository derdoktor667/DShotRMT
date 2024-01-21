#include <stdio.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>


typedef union {
    struct {
        unsigned int duration0 : 15; /*!< Duration of level0 */
        unsigned int level0 : 1;     /*!< Level of the first part */
        unsigned int duration1 : 15; /*!< Duration of level1 */
        unsigned int level1 : 1;     /*!< Level of the second part */
    };
    unsigned int val; /*!< Equivalent unsigned value for the RMT symbol */
} rmt_symbol_word_t;


typedef struct {
    rmt_symbol_word_t* received_symbols; /*!< Point to the received RMT symbols */
    size_t num_symbols;                  /*!< The number of received RMT symbols */
} rmt_rx_done_event_data_t;

typedef struct rx_frame_data_s
{
	size_t num_symbols; //how many of the 11 frames are filled with data
	rmt_symbol_word_t received_symbols[11]; //11 frames of received symbols for 21 bits

}rx_frame_data_t;

typedef enum extended_telem_type_e
{
	TYPE_TEMPRATURE = 0x2,
	TYPE_VOLTAGE = 0x4,
	TYPE_CURRENT = 0x6,
	TYPE_DEBUG_A = 0x8,
	TYPE_DEBUG_B = 0xA,
	TYPE_DEBUG_C = 0xC,
	TYPE_STATE = 0xE,
}extended_telem_type_t;


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


rmt_symbol_word_t raw_data[6] =
{
//	{23, 1, 38, 0},
//	{10, 1, 11, 0},
//	{11, 1, 12, 0},
//	{23, 1, 10, 0},
//	{38, 1, 24, 0},
//	{24, 1, 0, 0},


	//{25, 1, 13, 0},
	//{13, 1, 13, 0},
	//{24, 1, 13, 0},
	//{13, 1, 13, 0},
	//{24, 1, 13, 0},
	//{13, 1, 13, 0},
	//{13, 1, 13, 0},
	//{37, 1, 13, 0},

	//{11, 0, 24, 1},
	//{24, 0, 10, 1},
	//{24, 0, 10, 1},
	//{24, 0, 11, 1},
	//{0, 0, 11, 1},
	//
	//{24, 1, 11, 0},
	//{10, 1, 24, 0},
	//{10, 1, 24, 0},
	//{11, 1, 24, 0},
	//{11, 1, 0, 0},

	{11, 1, 24, 0},
	{11, 1, 11, 0},
	{23, 1, 38, 0},
	//{23, 1, 44, 0},
	{23, 1, 38, 0},
	{11, 1, 10, 0},
	{24, 1, 0, 0},


};


//convert the dshot frame (WITHOUT checksum) into some form of erpm (not sure what units yet...)
//stolen from betaflight (src/main/drivers/dshot.c)
uint32_t decode_eRPM_telemetry_value(uint16_t value)
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
uint32_t erpmToRpm(uint16_t erpm, uint16_t motorPoleCount)
{
	//  rpm = (erpm * 100) / (motorConfig()->motorPoleCount / 2)
	return (erpm * 200) / motorPoleCount;
}

//Error Codes:
//0: exit success
//1: nothing in queue
//2: no packet in queue
//3: checksum mismatch
//4: bidirection not enabled
//5: got other packet type?


//should I run this every time I try to send out a dshot packet, or should I let the user run this function?
int main(void)
{
	unsigned short ticks_one_high = 12;
	int error_packets = 0;
    
    rmt_rx_done_event_data_t rx_data = {raw_data, sizeof(raw_data) / sizeof(rmt_symbol_word_t)};


	rx_frame_data_t test1 = {};
	int size_count = sizeof(rx_frame_data_t::received_symbols) / sizeof(rmt_symbol_word_t);

	//only execute if we got a packet (no packet response gives us only one symbol by default)
	if (rx_data.num_symbols > 1)
	{

		int i, j;

		//experimental data shows that I get telemetry back with the following timing:
		//dshot 300 = 25 ticks per bit
		//dshot 600 = 12.5 ticks per bit
		//dshot 1200 = 6.25 ticks per bit
		//this closely matches dshot_config.ticks_one_high

		//with 600, recieved packets can have veriety that simultaneously renders the 90% reduction needed and erronious

		//bit time is reception bitrate * 90%
		unsigned short bitTime = ticks_one_high * 9 / 10;
		unsigned short bitCount0 = 0;
		unsigned short bitCount1 = 0;
		unsigned short bitShiftLevel = 20;//21 bits, including 0
		unsigned int assembledFrame = 0;

		int total_symbols = 0;

		//unsigned int* y = &assembledFrame; //for debugging on the computer
		for (i = 0; i < rx_data.num_symbols; ++i)
		{

			//for each symbol, see how many bits are in there
			bitCount0 = rx_data.received_symbols[i].duration0 / bitTime;
			bitCount1 = rx_data.received_symbols[i].duration1 / bitTime;

			total_symbols += bitCount0 + bitCount1;

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
		if (crc != alsocrc)
		{

			//print the offending packet
			printf("===============================\n");
			for (int i = 0; i < rx_data.num_symbols; ++i)
			{
				char hold[100] = {};
				sprintf(hold, "D0: %d L0: %d || D1: %d L1: %d",
					rx_data.received_symbols[i].duration0, rx_data.received_symbols[i].level0,
					rx_data.received_symbols[i].duration1, rx_data.received_symbols[i].level1);
				printf("%s\n", hold);
			}
			printf("===============================\n");


			error_packets += 1; //for now, the only error packets we will track are the ones where the checksum fails
			//we don't update the RPM pointer that was passed to us
			return 3;
		}


		//determine packet type
		if (frameData & 0b000100000000 || (~frameData & 0b111100000000) == 0b111100000000) //is erpm packet
		{
			//update output pointer
			uint32_t RPM = erpmToRpm(decode_eRPM_telemetry_value(frameData), 14);
			int ano = RPM;
		}
		else //is extended telemetry packet
		{
			uint8_t response_type = (frameData >> 8) & 0b1111;
			uint8_t response_data = (frameData & 0b11111111);

			//switch packet type
			switch ((extended_telem_type_t)response_type)
			{
			case TYPE_TEMPRATURE: //temprature in degrees C
				break;
			case TYPE_VOLTAGE: //0.25 volts per step
				break;
			case TYPE_CURRENT: //current in ANP
				break;

			default:
			case TYPE_DEBUG_A: //do nothing for these for now
			case TYPE_DEBUG_B:
			case TYPE_DEBUG_C:
			case TYPE_STATE:
				break;
				
			}
		}


	}
	else
	{
		return 2;
	}
    
    return 0;
}
