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
    static uint16_t throttle = DSHOT_THROTTLE_MIN;

    // Loop Time Measurement
    static uint32_t last_stats_print = 0;
    static uint32_t loop_time = 0;
    uint32_t loop_start = micros();

    //
    if (USB_SERIAL.available() > 0)
    {
        int new_throttle = USB_SERIAL.readStringUntil('\n').toInt();

        // Check for valid throttle value
        if (new_throttle >= DSHOT_THROTTLE_MIN && new_throttle <= DSHOT_THROTTLE_MAX)
        {
            throttle = new_throttle;
        }
        else
        {
            USB_SERIAL.println(" ");
            USB_SERIAL.println("NOT A VALID THROTTLE VALUE (48 - 2047)!!!");
            USB_SERIAL.println(" ");
        }
    }

    // Send the current throttle value
    motor01.sendThrottle(throttle);

    // Print motor stats every 2 seconds
    if (millis() - last_stats_print >= 3000)
    {
        motor01.printDshotInfo();

        // Loop Time Info
        USB_SERIAL.print("Loop Time: ");
        USB_SERIAL.print(loop_time);
        USB_SERIAL.println(" us");

        //
        last_stats_print = millis();
    }

    //
    loop_time = micros() - loop_start;
}
