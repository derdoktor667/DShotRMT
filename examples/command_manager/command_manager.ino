/*
 * command_manager_example.ino
 * Example sketch demonstrating DShotCommandManager usage
 * Author: Wastl Kraus
 * Date: 2025-09-04
 * License: MIT
 *
 * Modified by Especiallist to support continuous throttle sending.
 */

#include <Arduino.h>
#include <DShotRMT.h>
#include <DShotCommandManager.h>

// USB serial port settings
static constexpr auto &USB_SERIAL = Serial0;
static constexpr auto USB_SERIAL_BAUD = 115200;

// Motor configuration
static constexpr auto MOTOR01_PIN = 17;
static constexpr auto IS_BIDIRECTIONAL = false;

// Motor magnet count for RPM calculation
static constexpr auto MOTOR01_MAGNET_COUNT = 14;

// Create motor and command manager instances
DShotRMT motor01(MOTOR01_PIN, DSHOT300, IS_BIDIRECTIONAL);
DShotCommandManager commandManager(motor01);

// Global variable to store the desired continuous throttle value
static volatile uint16_t throttle_now = NULL;

//
void setup()
{
    // Start USB Serial Port
    USB_SERIAL.begin(USB_SERIAL_BAUD);

    USB_SERIAL.println("=== DShotRMT Command Manager Example ===");

    // Initialize DShot
    if (motor01.begin() != 0)
    {
        USB_SERIAL.println("ERROR: Failed to initialize DShotRMT!");
        while (1)
            delay(1000);
    }

    // Init Command Manager
    if (!commandManager.begin())
    {
        USB_SERIAL.println("ERROR: Failed to initialize DShotCommandManager!");
        while (1)
            delay(1000);
    }

    USB_SERIAL.println("Initialization successful!");

    // Print Menu
    printMenu();
}

//
void loop()
{
    // Time Measurement
    static uint64_t last_stats_print = 0;

    // Check for serial input
    if (USB_SERIAL.available() > 0)
    {
        String input = USB_SERIAL.readStringUntil('\n');
        input.trim();
        handleUserInput(input);
    }

    // Continuously send the stored throttle value
    if (throttle_now != NULL)
    {
        motor01.sendThrottle(throttle_now);

        // Print motor stats every 2 seconds
        if (esp_timer_get_time() - last_stats_print >= 2000000)
        {
            motor01.printDShotInfo();

            // Get Motor RPM
            if (IS_BIDIRECTIONAL)
            {
                uint32_t rpm = motor01.getMotorRPM(MOTOR01_MAGNET_COUNT);
                USB_SERIAL.printf("Motor RPM: %u\n", rpm);
            }

            // Time Stamp
            last_stats_print = esp_timer_get_time();
        }
    }
}

//
void handleUserInput(const String &input)
{
    dshot_command_result_t result;

    if (input == "1")
    {
        // Stop motor command should also reset the continuous throttle value
        throttle_now = 0;
        USB_SERIAL.print("Stopping motor... ");
        result = commandManager.stopMotor();
        printResult(result);
    }
    else if (input == "2")
    {
        USB_SERIAL.print("Activating beacon 1... ");
        result = commandManager.activateBeacon(1);
        printResult(result);
    }
    else if (input == "3")
    {
        USB_SERIAL.print("Setting normal spin direction... ");
        result = commandManager.setSpinDirection(false);
        printResult(result);
    }
    else if (input == "4")
    {
        USB_SERIAL.print("Setting reversed spin direction... ");
        result = commandManager.setSpinDirection(true);
        printResult(result);
    }
    else if (input == "5")
    {
        USB_SERIAL.print("Enabling 3D mode... ");
        result = commandManager.set3DMode(true);
        printResult(result);
    }
    else if (input == "6")
    {
        USB_SERIAL.print("Disabling 3D mode... ");
        result = commandManager.set3DMode(false);
        printResult(result);
    }
    else if (input == "7")
    {
        USB_SERIAL.print("Saving settings... ");
        result = commandManager.saveSettings();
        printResult(result);
    }
    else if (input == "8")
    {
        USB_SERIAL.print("Turning LED 0 ON... ");
        result = commandManager.setLED(0, true);
        printResult(result);
    }
    else if (input == "9")
    {
        USB_SERIAL.print("Turning LED 0 OFF... ");
        result = commandManager.setLED(0, false);
        printResult(result);
    }
    else if (input == "i")
    {
        USB_SERIAL.print("Executing initialization sequence... ");
        result = commandManager.executeInitSequence();
        printResult(result);
    }
    else if (input == "c")
    {
        USB_SERIAL.print("Executing calibration sequence... ");
        result = commandManager.executeCalibrationSequence();
        printResult(result);
    }
    else if (input == "s")
    {
        commandManager.printStatistics();
    }
    else if (input == "r")
    {
        commandManager.resetStatistics();
        USB_SERIAL.println("Statistics reset.");
    }
    else if (input == "h")
    {
        printMenu();
    }
    else if (input == "help")
    {
        printMenu();
    }
    else if (input.startsWith("cmd "))
    {
        // Direct command execution: "cmd 5" sends command 5
        int cmd_num = input.substring(4).toInt();
        if (DShotCommandManager::isValidCommand(static_cast<dshot_commands_t>(cmd_num)))
        {
            USB_SERIAL.printf("Sending command %d (%s)... ", cmd_num,
                              DShotCommandManager::getCommandName(static_cast<dshot_commands_t>(cmd_num)));
            result = commandManager.sendCommand(static_cast<dshot_commands_t>(cmd_num));
            printResult(result);
        }
        else
        {
            USB_SERIAL.printf("Invalid command number: %d (valid range: 0-%d)\n", cmd_num, DSHOT_CMD_MAX);
        }
    }
    else if (input.startsWith("throttle "))
    {
        // Throttle control: "throttle 1000" sets throttle to 1000
        int throttle_value = input.substring(9).toInt();
        if (throttle_value >= DSHOT_THROTTLE_MIN && throttle_value <= DSHOT_THROTTLE_MAX)
        {
            throttle_now = throttle_value;
            USB_SERIAL.printf("Setting continuous throttle to %d\n", throttle_now);
        }
        else if (throttle_value == 0)
        {
            throttle_now = 0;
            USB_SERIAL.println("Continuous throttle stopped.");
        }
        else
        {
            USB_SERIAL.printf("Invalid throttle value: %d (valid range: 48-2047, use 0 to stop)\n", throttle_value);
        }
    }
    else
    {
        USB_SERIAL.println("Unknown command. Type 'h' for help.");
    }
}

//
void printResult(const dshot_command_result_t &result)
{
    if (!result.success)
    {
        USB_SERIAL.printf("SUCCESS (%u us)\n", result.execution_time_us);
    }
    else
    {
        USB_SERIAL.printf("FAILED - %s (%u us)\n", result.error_message, result.execution_time_us);
    }
}

//
void printMenu()
{
    USB_SERIAL.println("================================================");
    USB_SERIAL.println("\n=== DShot Command Manager Menu ===");
    USB_SERIAL.println("Basic Commands:");
    USB_SERIAL.println("  1 - Stop Motor");
    USB_SERIAL.println("  2 - Activate Beacon 1");
    USB_SERIAL.println("  3 - Set Normal Spin Direction");
    USB_SERIAL.println("  4 - Set Reversed Spin Direction");
    USB_SERIAL.println("  5 - Enable 3D Mode");
    USB_SERIAL.println("  6 - Disable 3D Mode");
    USB_SERIAL.println("  7 - Save Settings");
    USB_SERIAL.println("  8 - Turn LED 0 ON");
    USB_SERIAL.println("  9 - Turn LED 0 OFF");
    USB_SERIAL.println("");
    USB_SERIAL.println("Sequences:");
    USB_SERIAL.println("  i - Execute Initialization Sequence");
    USB_SERIAL.println("  c - Execute Calibration Sequence");
    USB_SERIAL.println("");
    USB_SERIAL.println("Advanced:");
    USB_SERIAL.println("  cmd <number> - Send DShot command (0 - 47)");
    USB_SERIAL.println("  throttle <value> - Set throttle (48 - 2047)");
    USB_SERIAL.println("  throttle 0 - Stop sending throttle");
    USB_SERIAL.println("");
    USB_SERIAL.println("  h - Show this Menu");
    USB_SERIAL.println("");
    USB_SERIAL.println("Examples:");
    USB_SERIAL.println("  cmd 1 - Stop Motor");
    USB_SERIAL.println("  throttle 1000 - Set throttle to 1000");
    USB_SERIAL.println("================================================");
}
