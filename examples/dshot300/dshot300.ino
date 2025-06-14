/**
 * @file dshot300.ino
 * @brief Demo sketch for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-06-11
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

// BiDirectional DShot Support (default: false)
constexpr auto IS_BIDIRECTIONAL = true;

// Motor Magnet count for RPM calculation
constexpr auto MOTOR01_MAGNET_COUNT = 14;

// Setup Motor Pin, DShot Mode and optional BiDirectional Support
DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);

void setup()
{
  // Start the USB Serial Port
  USB_Serial.begin(USB_SERIAL_BAUD);

  // Initialize DShot Signal
  motor01.begin();

  // Arm ESC with minimum throttle
  motor01.setThrottle(DSHOT_THROTTLE_MIN);

  //
  USB_Serial.println("**********************");
  USB_Serial.println("DShotRMT Demo started.");
  USB_Serial.println("Enter a throttle value (48–2047):");
}

void loop()
{
  // Read value input from Serial
  int throttle_input = readSerialThrottle();

  // Now send the value
  motor01.setThrottle(throttle_input);

  // BiDirectional DShot: print out the received eRPMs every 2 seconds
  if (IS_BIDIRECTIONAL)
  {
    static unsigned long last_print_time = 0;
    unsigned long now = millis();

    if (now - last_print_time >= 2000)
    {
      last_print_time = now;

      uint32_t rpm = motor01.getMotorRPM(MOTOR01_MAGNET_COUNT);

      USB_Serial.print("Throttle: ");
      USB_Serial.print(throttle_input);
      USB_Serial.print(" | RPM: ");
      USB_Serial.println(rpm);
    }
  }
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
    throttle_input = constrain(throttle_input, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    last_throttle = throttle_input;

    USB_Serial.print("Throttle set to: ");
    USB_Serial.println(last_throttle);
    USB_Serial.println("***********************************");
    USB_Serial.println("Enter a throttle value (48–2047):");
  }

  return last_throttle;
}
