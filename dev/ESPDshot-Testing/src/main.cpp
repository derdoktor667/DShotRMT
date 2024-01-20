#include <Arduino.h>

//TEST
#include <hal/gpio_hal.h>

#include <DShotRMT.h>

// USB serial port needed for this example
const int USB_SERIAL_BAUD = 115200;
#define USB_Serial Serial

// Define the GPIO pin connected to the motor and the DShot protocol used
const auto MOTOR01_PIN = 23;
const auto MOTOR02_PIN = 18;
const auto DSHOT_MODE = DSHOT600;

// Define the failsafe and initial throttle values
const auto FAILSAFE_THROTTLE = 999;
const auto INITIAL_THROTTLE = 48;

#define TRIGGER_PIN 17

DShotRMT anESC(MOTOR02_PIN);//MOTOR01_PIN
DShotRMT anotherESC(MOTOR01_PIN);//MOTOR02_PIN

void setup()
{
	USB_Serial.begin(USB_SERIAL_BAUD);
	anESC.begin(DSHOT_MODE, ENABLE_BIDIRECTION, 14);
	anotherESC.begin(DSHOT_MODE, ENABLE_BIDIRECTION, 14);

	pinMode(TRIGGER_PIN, INPUT_PULLUP);

}
int loopCount = 0;
void loop()
{
	//////////////////////////////////
	// if(Serial.available())
	// {
	// 	while (Serial.available()) Serial.read();

	// 	anESC.send_dshot_value(100);
	// 	Serial.printf("Sent dshot value.\n");
	// }
	// uint16_t rpm_1 = 0;
	// int error_a = anESC.get_dshot_RPM(&rpm_1);
	// if(error_a != 1 && error_a != 2)
	// {
	// 	Serial.printf("%10d, %10.3f, err: %d\n",
	//  		rpm_1, anESC.get_telem_success_rate(), error_a);
	// }
	//////////////////////////////////



	// static bool last_trg = false;
	// if(digitalRead(TRIGGER_PIN) == LOW)
	// {
	// 	if(!last_trg)
	// 	{
	// 		loopCount = 0;
	// 		Serial.printf("Reset Loop\n");
	// 	}
	// 	last_trg = true;
	// }
	// else
	// 	last_trg = false;




	if(loopCount < 1200)
	{
		anESC.send_dshot_value(DSHOT_THROTTLE_MIN);
	}
	else if(loopCount < 1600)
	{
		static int enable_telemm = 0;
		if(enable_telemm < 10)
		{
			anESC.send_dshot_value(DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE);
			++enable_telemm;
		}
		else
		{
			anESC.send_dshot_value(DSHOT_THROTTLE_MIN);
		}
	}
	else if(loopCount < 2400)
	{
		anESC.send_dshot_value(DSHOT_CMD_ESC_INFO);
	}
	else
	{
		anESC.send_dshot_value(DSHOT_CMD_ESC_INFO);
	}



	//if(error_a == ERR_CHECKSUM_FAIL)
	//	Serial.printf("had error %10.3f\n", anESC.get_telem_success_rate());

	//if(pack_type != TYPE_ERPM)
	//	Serial.printf("A\n");

	if(loopCount % 10 == 0)
	{

		uint16_t packet_1 = 0;
		extended_telem_type_t pack_type = TYPE_ERPM;
		int error_a = anESC.get_dshot_packet(&packet_1, &pack_type);

		//Serial.printf("Packet: %10d || Packet Type: %10d\n", packet_1, pack_type);
		//Serial.printf("%10d, %10.3f, err: %d\n",
	 	//	packet_1, anESC.get_telem_success_rate(), error_a);
		//Serial.printf("%10.3f\n", anESC.get_telem_success_rate());
	}




	delay(2);
	++loopCount;







	// if(loopCount < 700)
	// {
	// 	anESC.send_dshot_value(INITIAL_THROTTLE);
	// 	anotherESC.send_dshot_value(INITIAL_THROTTLE);
	// }
	// else if(loopCount < 1200)
	// {

    // 	anESC.send_dshot_value(100);
	// 	anotherESC.send_dshot_value(100);
	// }
	// else
	// {
	// 	anESC.send_dshot_value(100);
	// 	anotherESC.send_dshot_value(100);
	// }

	// if(loopCount % 10 == 0)
	// {
	// 	uint16_t rpm_1 = 0;
	// 	int error_a = anESC.get_dshot_RPM(&rpm_1);
	// 	uint16_t rpm_2 = 0;
	// 	int error_b = anotherESC.get_dshot_RPM(&rpm_2);



	// 	Serial.printf("%10d, %10.3f || %10d, %10.3f\n",
	// 		rpm_1, anESC.get_telem_success_rate(),
	// 		rpm_2, anotherESC.get_telem_success_rate());

	// 	//Serial.printf("%10d, %10.3f, err: %d\n",
	// 	// 	rpm_1, anESC.get_telem_success_rate(), error_a);

	// }

	// delay(2);
	// ++loopCount;

}
