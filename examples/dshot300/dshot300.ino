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
constexpr auto USB_SERIAL_BAUD = 115200;
constexpr auto &USB_Serial = Serial;

// Define the GPIO pin connected to the motor and the DShot protocol used
constexpr auto MOTOR01_PIN = GPIO_NUM_17;
constexpr auto DSHOT_MODE = DSHOT300;

// Define the failsafe and initial throttle values
constexpr auto FAILSAFE_THROTTLE = 999;
constexpr auto INITIAL_THROTTLE = 48;

// Initialize a DShotRMT object for the motor
DShotRMT motor01(MOTOR01_PIN, RMT_CHANNEL_0);

void setup()
{
  USB_Serial.begin(USB_SERIAL_BAUD);

  // Start generating DShot signal for the motor
  motor01.begin(DSHOT_MODE);

  Serial.println("DShotRMT Demo started.");
  Serial.println("Enter a throttle value (0–2047):");
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
  static int last_throttle = DSHOT_THROTTLE_MIN;

  if (USB_Serial.available() > 0)
  {
    String input = USB_Serial.readStringUntil('\n');
    int throttle_input = input.toInt();

    // Clamp the value to the DShot range
    throttle_input = constrain(throttle_input, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    last_throttle = throttle_input;

    USB_Serial.print("Throttle set to: ");
    USB_Serial.println(last_throttle);
    USB_Serial.println("***********************************");
    USB_Serial.println("Enter a throttle value (48–2047):");
  }

  return last_throttle;
}
