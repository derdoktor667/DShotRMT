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


DShotRMT anESC(MOTOR01_PIN);
DShotRMT anotherESC(MOTOR02_PIN);

void setup()
{
	USB_Serial.begin(USB_SERIAL_BAUD);
	anESC.begin(DSHOT_MODE, ENABLE_BIDIRECTION, 14);
	anotherESC.begin(DSHOT_MODE, ENABLE_BIDIRECTION, 14);

}

int loopCount = 0;
void loop()
{
	if(loopCount < 700)
	{
		anESC.send_dshot_value(INITIAL_THROTTLE);
		anotherESC.send_dshot_value(INITIAL_THROTTLE);
	}
	else if(loopCount < 1200)
	{

    	anESC.send_dshot_value(500);
		anotherESC.send_dshot_value(300);
	}
	else
	{
		anESC.send_dshot_value(300);
		anotherESC.send_dshot_value(500);
	}

	if(loopCount % 10 == 0)
	{
		Serial.print(anESC.get_dshot_RPM());
		Serial.print(" || ");
		Serial.println(anotherESC.get_dshot_RPM());
	}

	delay(10);
		++loopCount;

}
