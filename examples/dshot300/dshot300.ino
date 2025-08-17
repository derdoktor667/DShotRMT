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

// Motor configuration
// Pin number or GPIO_PIN
// static constexpr gpio_num_t MOTOR01_PIN = GPIO_NUM_17;
static constexpr auto MOTOR01_PIN = 17;

// Supported: DSHOT150, DSHOT300, DSHOT600, (DSHOT1200)
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;

// BiDirectional DShot Support (default: false)
static constexpr bool IS_BIDIRECTIONAL = false;

// Motor magnet count for RPM calculation
static constexpr auto MOTOR01_MAGNET_COUNT = 14;

//
// Create the motor instance
DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);

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
    static auto throttle = DSHOT_THROTTLE_MIN;
    
    if (USB_SERIAL.available() > NULL)
    {
        auto new_throttle = (USB_SERIAL.readStringUntil('\n').toInt());

        USB_SERIAL.println("*********************");
        USB_SERIAL.print("Throttle set to: ");
        USB_SERIAL.println(new_throttle);

        //
        throttle = new_throttle;
    }

    // Sends the value to the ESC
    motor01.sendThrottle(throttle);

    // Prints out RPM if BiDirectional DShot is enabled every 2 seconds
    // printRPMPeriodically(2000);

    // Debug: Prints out "raw" DShot packet every 2 seconds
    print_RMT_packet(2000);
}

// Prints RPM every ms
void printRPMPeriodically(auto timer_ms)
{
    if (IS_BIDIRECTIONAL)
    {
        static unsigned long last_print_time = 0;

        if (millis() - last_print_time >= timer_ms)
        {
            auto rpm = motor01.getMotorRPM(MOTOR01_MAGNET_COUNT);

            USB_SERIAL.print("RPM: ");
            USB_SERIAL.println(rpm);

            last_print_time = millis();
        }
    }
}

// Prints "raw" packet every ms
void print_RMT_packet(auto timer_ms)
{
    static unsigned long last_print_time = 0;

    if (millis() - last_print_time >= timer_ms)
    {
        auto packet = motor01.getDShotPacket();

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
