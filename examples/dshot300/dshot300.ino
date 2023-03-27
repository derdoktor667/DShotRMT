// ...some very simple DShot example generating a DShot300 signal.
#include <Arduino.h>
#include <DShotRMT.h>

// Define USB serial port if available
#ifdef SERIAL
#define USB_Serial Serial
constexpr auto USB_SERIAL_BAUD = 115200;
#endif // SERIAL

// Define motor pin and DShot protocol
constexpr auto MOTOR_PIN = GPIO_NUM_4;
constexpr auto DSHOT_MODE = DSHOT300;

// Define failsafe and initial throttle value
constexpr auto FAILSAFE_THROTTLE = 0x3E7;
constexpr auto INITIAL_THROTTLE = 0x30;

// Initialize DShotRMT object for the motor
DShotRMT motor01(MOTOR_PIN, RMT_CHANNEL_0);

void setup()
{
    // Start USB serial port
    USB_Serial.begin(USB_SERIAL_BAUD);

    // Start DShot signal generation
    motor01.begin(DSHOT_MODE);
}

void loop()
{
    // Read throttle value from serial input
    auto throttle_input = readSerialThrottle();

    // Set the throttle value
    auto throttle_value = (throttle_input > 0) ? throttle_input : FAILSAFE_THROTTLE;

    // Send the throttle value to the motor
    motor01.sendThrottleValue(throttle_value);

    // Print the throttle value to the serial console
    USB_Serial.println(throttle_value);
}

// Read throttle value from USB serial input
uint16_t readSerialThrottle()
{
    if (USB_Serial.available() > 0)
    {
        return USB_Serial.readStringUntil('\n').toInt();
    }
    else
    {
        return 0;
    }
}
