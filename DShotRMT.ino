// ...some very simple DShot example generating a DShot300 signal.

#include <Arduino.h>
#include "src/DShotRMT.h"

// ...clearly name usb port
#ifdef SERIAL
#define USB_Serial Serial
constexpr auto USB_SERIAL_BAUD = 115200;
#endif // SERIAL

DShotRMT dshot_01(GPIO_NUM_4, RMT_CHANNEL_0);

volatile uint16_t throttle_value = 0x30; // ...sending "48", the lowest throttle value

void setup() {
  // ...always start the onboard usb support
  USB_Serial.begin(USB_SERIAL_BAUD);

  // ...start the dshot generation
  dshot_01.begin(DSHOT300, false); // speed & bidirectional
}

void loop() {
  if (USB_Serial.available() > 0) {
    throttle_value = (USB_Serial.readStringUntil('\n')).toInt();
    USB_Serial.print("read serial value ");
    USB_Serial.println(throttle_value);
  }

  dshot_01.send_dshot_value(throttle_value);

  // ...print to console
  USB_Serial.println(throttle_value);
  delay(50); // serial.available fails if you lower this too much??
}
