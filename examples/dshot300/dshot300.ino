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
static constexpr HardwareSerial &USB_SERIAL = Serial0;
static constexpr uint32_t USB_SERIAL_BAUD = 115200;

// Motor configuration
// static constexpr gpio_num_t MOTOR01_PIN = GPIO_NUM_17;
static constexpr uint16_t MOTOR01_PIN = 17;
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;

// BiDirectional DShot Support (default: false)
static constexpr bool IS_BIDIRECTIONAL = false;

// Motor magnet count for RPM calculation
static constexpr uint8_t MOTOR01_MAGNET_COUNT = 14;

// Setup Motor Pin, DShot Mode and optional BiDirectional Support
DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);

// Prints RPM if BiDirectional is enabled every ms
void printRPMPeriodically(uint16_t timer_ms);

// Reads throttle value from serial input
uint16_t readSerialThrottle(HardwareSerial &serial);

// Prints out the dshot packet bitwise every ms (Debug)
void printTXPacket(uint16_t timer_ms);

//
void setup()
{
    // Start the USB Serial Port
    USB_SERIAL.begin(USB_SERIAL_BAUD);

    // Initialize DShot Signal
    motor01.begin();

    // Arm ESC with minimum throttle
    // motor01.sendThrottle(DSHOT_THROTTLE_MIN);

    USB_SERIAL.println("***********************************");
    USB_SERIAL.println("  === DShotRMT Demo started. ===   ");
    USB_SERIAL.println("Enter a throttle value (48 – 2047):");
}

//
void loop()
{
    // Read value input from Serial
    uint16_t throttle_input = readSerialThrottle(USB_SERIAL);

    // Send the value to the ESC
    motor01.sendThrottle(throttle_input);

    // Prints RPM every 2 if BiDirectional DShot is enabled
    printRPMPeriodically(2000);

    // Prints out "raw" DShot packet every 2 seconds
    // printTXPacket(2000);
}

// Reads throttle value from serial input
uint16_t readSerialThrottle(HardwareSerial &serial)
{
    static uint16_t last_throttle = DSHOT_THROTTLE_MIN;

    if (serial.available() > NULL)
    {
        // Reads a value
        uint16_t throttle = (serial.readStringUntil('\n').toInt());

        // Check for valid throttle value
        if (throttle < DSHOT_THROTTLE_MIN || throttle > DSHOT_THROTTLE_MAX)
        {
            USB_SERIAL.println("Throttle value not in range (48 - 2047)!");
            return last_throttle;
        }

        last_throttle = throttle;

        USB_SERIAL.println("*********************");
        USB_SERIAL.print("Throttle set to: ");
        USB_SERIAL.println(last_throttle);
    }

    return last_throttle;
}

// Prints RPM every ms
void printRPMPeriodically(uint16_t timer_ms)
{
    if (IS_BIDIRECTIONAL)
    {
        static unsigned long last_print_time = 0;

        if (millis() - last_print_time >= timer_ms)
        {
            uint32_t rpm = motor01.getMotorRPM(MOTOR01_MAGNET_COUNT);

            USB_SERIAL.print("RPM: ");
            USB_SERIAL.println(rpm);

            last_print_time = millis();
        }
    }
}

// Prints "raw" packet every ms
void printTXPacket(uint16_t timer_ms)
{
    static unsigned long last_print_time = 0;

    if (millis() - last_print_time >= timer_ms)
    {
        uint16_t packet = motor01.getDShotPacket();

        // Print bit by bit
        for (int i = 15; i >= 0; --i)
        {
            if ((packet >> i) & 1)
            {
                USB_SERIAL.print("1");
            }
            else
            {
                USB_SERIAL.print("0");
            }
        }

        USB_SERIAL.println("");
        last_print_time = millis();
    }
}
