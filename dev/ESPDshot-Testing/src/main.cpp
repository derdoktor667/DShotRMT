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


DShotRMT anESC(MOTOR01_PIN);//MOTOR01_PIN
DShotRMT anotherESC(MOTOR02_PIN);//MOTOR02_PIN

void setup()
{
	USB_Serial.begin(USB_SERIAL_BAUD);
	anESC.begin(DSHOT_MODE, ENABLE_BIDIRECTION, 14);
	anotherESC.begin(DSHOT_MODE, ENABLE_BIDIRECTION, 14);

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


	if(loopCount < 700)
	{
		anESC.send_dshot_value(INITIAL_THROTTLE);
		anotherESC.send_dshot_value(INITIAL_THROTTLE);
	}
	else if(loopCount < 1200)
	{

    	anESC.send_dshot_value(100);
		anotherESC.send_dshot_value(100);
	}
	else
	{
		anESC.send_dshot_value(100);
		anotherESC.send_dshot_value(100);
	}

	if(loopCount % 10 == 0)
	{
		uint16_t rpm_1 = 0;
		int error_a = anESC.get_dshot_RPM(&rpm_1);
		uint16_t rpm_2 = 0;
		int error_b = anotherESC.get_dshot_RPM(&rpm_2);



		Serial.printf("%10d, %10.3f || %10d, %10.3f\n",
			rpm_1, anESC.get_telem_success_rate(),
			rpm_2, anotherESC.get_telem_success_rate());

		//Serial.printf("%10d, %10.3f, err: %d\n",
		// 	rpm_1, anESC.get_telem_success_rate(), error_a);

	}

	delay(2);
	++loopCount;

}
