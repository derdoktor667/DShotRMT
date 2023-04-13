/*
 * Title: dshot300.ino
 * Author: derdoktor667
 * Date: 2023-04-13
 * Description: A simple example of using the DShotRMT library to
 * generate a DShot300 signal for a motor using an Arduino board.
 */

#include <Arduino.h>
#include "DShotRMT.h"

// USB serial port needed for this example
constexpr auto USB_SERIAL_BAUD = 115200;
#define USB_Serial Serial

// Define the GPIO pin connected to the motor and the DShot protocol used
constexpr auto MOTOR01_PIN = GPIO_NUM_4;
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
}

void loop()
{
    // Read the throttle value from the USB serial input
    auto throttle_input = readSerialThrottle();

    // Set the throttle value to either the value received from the serial input or the failsafe throttle value
    auto throttle_value = (throttle_input > 0) ? throttle_input : FAILSAFE_THROTTLE;

    // Send the throttle value to the motor
    motor01.sendThrottleValue(throttle_value);
}

// Read the throttle value from the USB serial input
uint16_t readSerialThrottle()
{
    if (USB_Serial.available() > 0)
    {
        return USB_Serial.readStringUntil('\n').toInt();
    }
}
