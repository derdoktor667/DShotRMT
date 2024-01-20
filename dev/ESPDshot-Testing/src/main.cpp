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

DShotRMT anESC(MOTOR01_PIN);//MOTOR01_PIN
DShotRMT anotherESC(MOTOR02_PIN);//MOTOR02_PIN

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



	if(loopCount < 1600)
	{
		anESC.send_dshot_value(INITIAL_THROTTLE);
		anotherESC.send_dshot_value(INITIAL_THROTTLE);
	}
	else if(loopCount < 2000)
	{

    	anESC.send_dshot_value(200);
		anotherESC.send_dshot_value(100);
	}
	else
	{
		anESC.send_dshot_value(100);
		anotherESC.send_dshot_value(200);
	}

	if(loopCount % 10 == 0)
	{
		uint16_t rpm_1 = 0;
		extended_telem_type_t telem = TELEM_TYPE_ERPM; //telemetry argument is optional
		int error_a = anESC.get_dshot_packet(&rpm_1, &telem);
		uint16_t rpm_2 = 0;
		int error_b = anotherESC.get_dshot_packet(&rpm_2);



		Serial.printf("%10d, %10.3f || %10d, %10.3f\n",
			rpm_1, anESC.get_telem_success_rate(),
			rpm_2, anotherESC.get_telem_success_rate());


	}

	delay(2);
	++loopCount;

}
