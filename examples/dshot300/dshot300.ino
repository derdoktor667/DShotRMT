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
constexpr auto &USB_SERIAL = Serial0;
constexpr uint32_t USB_SERIAL_BAUD = 115200;

// Motor configuration
constexpr gpio_num_t MOTOR01_PIN = GPIO_NUM_17;
constexpr dshot_mode_t DSHOT_MODE = DSHOT300;

// BiDirectional DShot Support (default: false)
constexpr bool IS_BIDIRECTIONAL = false;

// Motor magnet count for RPM calculation
constexpr uint8_t MOTOR01_MAGNET_COUNT = 14;

// Setup Motor Pin, DShot Mode and optional BiDirectional Support
DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);

// Prints RPM and throttle every 2 seconds if BiDirectional is enabled
void printRPMPeriodically(uint16_t throttle);

// Reads throttle value from serial input
uint16_t readSerialThrottle();

void setup()
{
    // Start the USB Serial Port
    USB_SERIAL.begin(USB_SERIAL_BAUD);

    // Initialize DShot Signal
    motor01.begin();

    // Arm ESC with minimum throttle
    motor01.setThrottle(DSHOT_THROTTLE_MIN);

    USB_SERIAL.println("**********************");
    USB_SERIAL.println("DShotRMT Demo started.");
    USB_SERIAL.println("Enter a throttle value (48–2047):");
}

void loop()
{
    // Read value input from Serial
    uint16_t throttle_input = readSerialThrottle();

    // Send the value to the ESC
    motor01.setThrottle(throttle_input);

    // Print RPM if BiDirectional DShot is enabled
    if (IS_BIDIRECTIONAL)
    {
        printRPMPeriodically(throttle_input);
    }
}

// Reads throttle value from serial input
uint16_t readSerialThrottle()
{
    static uint16_t last_throttle = DSHOT_THROTTLE_MIN;

    if (USB_SERIAL.available() > 0)
    {
        String input = USB_SERIAL.readStringUntil('\n');
        int throttle_input = input.toInt();

        // Clamp the value to the DShot range
        throttle_input = constrain(throttle_input, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);

        if (throttle_input < DSHOT_THROTTLE_MIN || throttle_input > DSHOT_THROTTLE_MAX)
        {
            USB_SERIAL.println("Invalid input. Please enter a value between 48 and 2047.");
        }
        else
        {
            last_throttle = throttle_input;
            USB_SERIAL.print("Throttle set to: ");
            USB_SERIAL.println(last_throttle);
        }

        USB_SERIAL.println("*********************************");
        USB_SERIAL.println("Enter a throttle value (48–2047):");
    }

    return last_throttle;
}

// Prints RPM and throttle every 2 seconds
void printRPMPeriodically(uint16_t throttle)
{
    static unsigned long last_print_time = 0;
    unsigned long now = millis();

    if (now - last_print_time >= 2000)
    {
        last_print_time = now;

        uint32_t rpm = motor01.getMotorRPM(MOTOR01_MAGNET_COUNT);

        USB_SERIAL.print("Throttle: ");
        USB_SERIAL.print(throttle);
        USB_SERIAL.print(" | RPM: ");
        USB_SERIAL.println(rpm);
    }
}
