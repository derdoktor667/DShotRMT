/**
 * @file dshot300.ino
 * @brief Demo sketch for continuous DShot signal using ESP32 and DShotRMT library
 * @author Wastl Kraus
 * @date 2025-06-07
 * @license MIT
 */

#include <Arduino.h>
#include <DShotRMT.h>

// USB serial port settings
#define USB_Serial Serial0
constexpr auto USB_SERIAL_BAUD = 115200;

// Motor configuration
constexpr auto MOTOR01_PIN = GPIO_NUM_17;
constexpr auto DSHOT_MODE = DSHOT300;

// Create DShotRMT instance
DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE);

void setup()
{
  USB_Serial.begin(USB_SERIAL_BAUD);

  // Wait for serial port
  while (!USB_Serial)
    delay(10);

  USB_Serial.println("DShotRMT Demo started.");
  USB_Serial.println("Enter a throttle value (48–2047):");

  motor01.begin();

  // Arm ESC with minimum throttle
  motor01.setThrottle(DSHOT_THROTTLE_MIN);
}

void loop()
{
  // Simple as can be
  int throttle_input = readSerialThrottle();

  motor01.setThrottle(throttle_input);
}

// Reads throttle value from serial input
int readSerialThrottle()
{
  //
  static int last_throttle = DSHOT_THROTTLE_MIN;

  if (USB_Serial.available() > 0)
  {
    String input = USB_Serial.readStringUntil('\n');
    int throttle_input = input.toInt();

    // Clamp the value to the DShot range
    throttle_input = constrain(throttle_input, 48, 2047);
    last_throttle = throttle_input;

    USB_Serial.print("Throttle set to: ");
    USB_Serial.println(last_throttle);

    USB_Serial.println("Enter a throttle value (48–2047):");
  }

  return last_throttle;
}