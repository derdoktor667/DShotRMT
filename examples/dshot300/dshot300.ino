/*
 * Title: dshot300.ino
 * Author: derdoktor667
 * Date: 2023-04-13
 *
 * Description: A simple example of using the DShotRMT library to
 * generate a DShot300 signal for blheli_s escs.
 */

#include <Arduino.h>
#include <DShotRMT.h>

// USB serial port needed for this example
const auto USB_SERIAL_BAUD = 115200;
#define USB_Serial Serial

// Define the GPIO pin connected to the motor and the DShot protocol used
const auto MOTOR01_PIN = GPIO_NUM_4;
const auto DSHOT_MODE = DSHOT300;

// Define the failsafe and initial throttle values
const auto FAILSAFE_THROTTLE = 999;
const auto INITIAL_THROTTLE = 48;

// Initialize a DShotRMT object for the motor
DShotRMT motor01(MOTOR01_PIN, RMT_CHANNEL_0);

void setup()
{
  USB_Serial.begin(USB_SERIAL_BAUD);

  // Start generating DShot signal for the motor
  motor01.begin(DSHOT_MODE);
}

void loop()
{
  // Read the throttle value from the USB serial input
  int throttle_input = read_SerialThrottle();

  // Send the throttle value to the motor
  motor01.sendThrottleValue(throttle_input);
}

// ...just for this example
// Read the throttle value from the USB serial input
int read_SerialThrottle()
{
  if (USB_Serial.available() > 0)
  {
    auto throttle_input = (USB_Serial.readStringUntil('\n')).toInt();
    return throttle_input;
  }
  else
  {
    return FAILSAFE_THROTTLE;
  }
}
