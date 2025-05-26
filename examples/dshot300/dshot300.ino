//------------------------------------------------------------------------------
// Name:        DShotRMT
// Date:        2025-04-14
// Author:      Wastl Kraus
// Description: A simple example of using the DShotRMT library to
//              generate a DShot300 signal for blheli_s ESCs.
//------------------------------------------------------------------------------

#include "Arduino.h"
#include "DShotRMT.h"

// USB serial port needed for this example
#define USB_Serial Serial0
const auto USB_SERIAL_BAUD = 115200;

// Define the GPIO pin connected to the motor and the DShot protocol used
const auto MOTOR01_PIN = GPIO_NUM_17;
const auto DSHOT_MODE = DSHOT300;

// Define the failsafe and initial throttle values
const auto FAILSAFE_THROTTLE = 0;
const auto INITIAL_THROTTLE = 48;

// Initialize a DShotRMT object for the motor
DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE);

void setup()
{
    USB_Serial.begin(USB_SERIAL_BAUD);
    Serial.println("DShot test begins...");
    Serial.println("Enter throttle value (48-2048):");
    motor01.begin();
}

void loop()
{
    // Read the throttle value from the USB serial input
    auto throttle_input = readSerialThrottle();

    // Set the throttle value to either the value received from the serial input or the failsafe throttle value
    auto throttle_value = (throttle_input >= 48 && throttle_input <= 2048) ? throttle_input : FAILSAFE_THROTTLE;

    // Now finally send the value
    motor01.sendThrottleValue(throttle_value);
}

// Read the throttle value from the USB serial input
uint16_t readSerialThrottle()
{
    if (USB_Serial.available() > 0)
    {
        return USB_Serial.readStringUntil('\n').toInt();
    }
    return FAILSAFE_THROTTLE;
}
