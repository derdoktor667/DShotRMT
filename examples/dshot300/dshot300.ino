/**
 * @file dshot300.ino
 * @brief Demo sketch for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-06-11
 * @license MIT
 */

#include <Arduino.h>
#include <DShotRMT.h>

// Debug output
static constexpr auto DEBUG = false;

// USB serial port settings
static constexpr auto &USB_SERIAL = Serial0;
static constexpr auto USB_SERIAL_BAUD = 115200;

// Motor configuration - Pin number or GPIO_PIN
// static constexpr gpio_num_t MOTOR01_PIN = GPIO_NUM_17;
static constexpr auto MOTOR01_PIN = 17;

// Supported: DSHOT150, DSHOT300, DSHOT600, (DSHOT1200)
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;

// BiDirectional DShot Support (default: false)
static constexpr auto IS_BIDIRECTIONAL = false;

// Motor magnet count for RPM calculation
static constexpr auto MOTOR01_MAGNET_COUNT = 14;

// Creates the motor instance
DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);

//
void setup()
{
    // Starts the USB Serial Port
    USB_SERIAL.begin(USB_SERIAL_BAUD);

    // Initializes DShot Signal
    motor01.begin();

    // Print CPU Info
    motor01.printCpuInfo();

    USB_SERIAL.println(" ");
    USB_SERIAL.println("***********************************");
    USB_SERIAL.println("  === DShotRMT Demo started. ===   ");
    USB_SERIAL.println("Enter a throttle value (48 – 2047):");
}

//
void loop()
{
    /// ...safety first
    static uint16_t throttle = DSHOT_CMD_MOTOR_STOP;

    // Time Measurement
    static uint32_t last_stats_print = 0;

    // Read throttle value
    if (USB_SERIAL.available() > 0)
    {
        throttle = USB_SERIAL.readStringUntil('\n').toInt();
    }

    // Send the current throttle value
    motor01.sendThrottle(throttle);

    // Debug output
    if (DEBUG)
    {
        motor01.printDebugStream();
        return;
    }
    
    // Print motor stats every 2 seconds
    if (millis() - last_stats_print >= 2000)
    {
        motor01.printDshotInfo();

        // Time Stamp
        last_stats_print = millis();
    }
}
