//------------------------------------------------------------------------------
// Name:        DShotRMT
// Date:        2025-04-14
// Author:      Wastl Kraus
// Description: A simple example of using the DShotRMT library to
// generate a DShot300 signal for blheli_s ESCs.
//------------------------------------------------------------------------------

#include <Arduino.h>
#include <DShotRMT.h>

// USB serial port needed for this example
const auto USB_SERIAL_BAUD = 115200;
#define USB_Serial Serial

// Define the GPIO pin connected to the motor and the DShot protocol used
const auto MOTOR01_PIN = GPIO_NUM_17;
const DShotMode DSHOT_MODE = DShotMode::DSHOT300;

// Define the failsafe and initial throttle values
const auto FAILSAFE_THROTTLE = 0;

// Initialize a DShotRMT object for the motor
DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE);

void setup()
{
    USB_Serial.begin(USB_SERIAL_BAUD);
    Serial.println("DShot test begins...");
    Serial.println("Enter throttle value (0-2000):");
}

void loop()
{
    // Read the throttle value from the USB serial input
    auto throttle_input = readSerialThrottle();

    // Set the throttle value to either the value received from the serial input or the failsafe throttle value
    auto throttle_value = (throttle_input >= 0 && throttle_input <= 2000) ? throttle_input : FAILSAFE_THROTTLE;

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
