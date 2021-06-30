// ...some very simple DShot example generating a DShot300 signal.

#include <Arduino.h>
#include "DShot_Lib.h"

// ...clearly name usb port
#ifdef SERIAL
#define USB_Serial Serial
constexpr auto USB_SERIAL_BAUD = 115200;
#endif // SERIAL

DShotRMT dshot_01(GPIO_NUM_4, RMT_CHANNEL_0);

volatile uint16_t throttle_value = 0x30; // ...sending "48", the first throttle value

void setup() {
    // ...always start the onboard usb support
	USB_Serial.begin(USB_SERIAL_BAUD);

    // ...start the dshot generation
    dshot_01.begin(DSHOT300);

    // ...stabilize settings
    vTaskDelay(500);
}

void loop() {
    dshot_01.send_dshot_value(throttle_value);
}

//
//
void read_SerialThrottle() {
	if (USB_Serial.available() > 0) {
		auto throttle_input = (USB_Serial.readStringUntil('\n')).toInt();
		throttle_value = throttle_input;
	}
}
