/*
 * command_manager.ino
 * Example sketch for DShotCommandManager
 * Author: Wastl Kraus
 * Date: 2025-09-04
 * License: MIT
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
static volatile uint16_t throttle_now = 0;

// Helper function to print telemetry results
void printTelemetryResult(const dshot_result_t &result)
{
    if (result.success && (result.erpm > 0 || result.motor_rpm > 0))
    {
        USB_SERIAL.printf("Telemetry: eRPM=%u, Motor RPM=%u\n", result.erpm, result.motor_rpm);
    }
    else
    {
        USB_SERIAL.printf("Telemetry: FAILED - %s\n", result.msg);
    }
}

//
void setup()
{
    // Start USB Serial Port
    USB_SERIAL.begin(USB_SERIAL_BAUD);

    // Initialize DShot
    motor01.begin();

    // Init Command Manager
    commandManager.begin();

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
    if (throttle_now != 0)
    {
        dshot_result_t result = motor01.sendThrottle(throttle_now);

        // Only print errors to avoid spam
        if (!result.success)
        {
            printResult(result);
        }

        // Print motor stats every 2 seconds
        if (esp_timer_get_time() - last_stats_print >= 2000000)
        {
            motor01.printDShotInfo();

            // Get Motor RPM
            if (IS_BIDIRECTIONAL)
            {
                dshot_result_t telem_result = motor01.getTelemetry(MOTOR01_MAGNET_COUNT);
                printTelemetryResult(telem_result);
            }

            // Time Stamp
            last_stats_print = esp_timer_get_time();
        }
    }
}

//
void handleUserInput(const String &input)
{
    dshot_result_t cmd_result;

    if (input == "1")
    {
        // Stop motor command should also reset the continuous throttle value
        throttle_now = 0;
        USB_SERIAL.print("Stopping motor... ");
        cmd_result = commandManager.stopMotor();
        printResult(cmd_result);
        return;
    }

    if (input == "2")
    {
        USB_SERIAL.print("Activating beacon 1... ");
        cmd_result = commandManager.activateBeacon(1);
        printResult(cmd_result);
        return;
    }

    if (input == "3")
    {
        USB_SERIAL.print("Setting normal spin direction... ");
        cmd_result = commandManager.setSpinDirection(false);
        printResult(cmd_result);
        return;
    }

    if (input == "4")
    {
        USB_SERIAL.print("Setting reversed spin direction... ");
        cmd_result = commandManager.setSpinDirection(true);
        printResult(cmd_result);
        return;
    }

    if (input == "5")
    {
        USB_SERIAL.print("Getting ESC Info... ");
        cmd_result = commandManager.requestESCInfo();
        printResult(cmd_result);
        return;
    }

    if (input == "6")
    {
        USB_SERIAL.print("Turning LED 0 ON... ");
        cmd_result = commandManager.setLED(0, true);
        printResult(cmd_result);
        return;
    }

    if (input == "7")
    {
        USB_SERIAL.print("Turning LED 0 OFF... ");
        cmd_result = commandManager.setLED(0, false);
        printResult(cmd_result);
        return;
    }

    if (input == "h" || input == "help")
    {
        printMenu();
        return;
    }

    if (input == "info")
    {
        motor01.printDShotInfo();
        return;
    }

    if (input == "rpm" && IS_BIDIRECTIONAL)
    {
        dshot_result_t result = motor01.getTelemetry(MOTOR01_MAGNET_COUNT);
        printTelemetryResult(result);
        return;
    }

    if (input.startsWith("cmd "))
    {
        // Direct command execution: "cmd 5" sends command 5
        int cmd_num = input.substring(4).toInt();

        if (DShotCommandManager::isValidCommand(static_cast<dshot_commands_t>(cmd_num)))
        {
            USB_SERIAL.printf("Sending command %d (%s)... ", cmd_num,
                              DShotCommandManager::getCommandName(static_cast<dshot_commands_t>(cmd_num)));
            cmd_result = commandManager.sendCommand(static_cast<dshot_commands_t>(cmd_num));
            printResult(cmd_result);
        }
        else
        {
            USB_SERIAL.printf("Invalid command number: %d (valid range: 0 - %d)\n", cmd_num, DSHOT_CMD_MAX);
        }
        return;
    }

    if (input.startsWith("throttle "))
    {
        // Throttle control: "throttle 1000" sets throttle to 1000
        int throttle_value = input.substring(9).toInt();

        if (throttle_value >= DSHOT_THROTTLE_MIN && throttle_value <= DSHOT_THROTTLE_MAX)
        {
            throttle_now = throttle_value;
            USB_SERIAL.printf("Setting continuous throttle to %d\n", throttle_now);

            // Send first throttle command and show result
            dshot_result_t result = motor01.sendThrottle(throttle_now);
            printResult(result);

            if (result.success)
            {
                USB_SERIAL.println("Continuous throttle mode enabled. Send '0' or 'throttle 0' to stop.");
            }
            return;
        }

        if (throttle_value == 0)
        {
            throttle_now = 0;
            USB_SERIAL.println("Continuous throttle stopped.");

            // Send stop command
            dshot_result_t result = motor01.sendCommand(DSHOT_CMD_MOTOR_STOP);
            printResult(result);
            return;
        }

        USB_SERIAL.printf("Invalid throttle value: %d (valid range: %d-%d, use 0 to stop)\n",
                          throttle_value, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
        return;
    }

    if (input == "0")
    {
        // Quick stop
        throttle_now = 0;
        USB_SERIAL.print("Emergency stop... ");
        dshot_result_t result = motor01.sendCommand(DSHOT_CMD_MOTOR_STOP);
        printResult(result);
        return;
    }

    if (input.startsWith("repeat "))
    {
        // Repeat command: "repeat cmd 5 count 10" - sends command 5 ten times
        String params = input.substring(7);

        if (!params.startsWith("cmd "))
        {
            USB_SERIAL.println("Usage: repeat cmd <number> count <repeat_count>");
            return;
        }

        int space_pos = params.indexOf(' ', 4);

        if (space_pos <= 0 || !params.substring(space_pos + 1).startsWith("count "))
        {
            USB_SERIAL.println("Usage: repeat cmd <number> count <repeat_count>");
            return;
        }

        int cmd_num = params.substring(4, space_pos).toInt();
        int repeat_count = params.substring(space_pos + 7).toInt();

        if (!DShotCommandManager::isValidCommand(static_cast<dshot_commands_t>(cmd_num)) ||
            repeat_count <= 0 || repeat_count > 100)
        {
            USB_SERIAL.println("Invalid command or repeat count (1-100)");
            return;
        }

        USB_SERIAL.printf("Sending command %d (%s) %d times... ", cmd_num,
                          DShotCommandManager::getCommandName(static_cast<dshot_commands_t>(cmd_num)),
                          repeat_count);
        cmd_result = commandManager.sendCommand(static_cast<dshot_commands_t>(cmd_num), repeat_count);
        printResult(cmd_result);
        return;
    }

    // Unknown command
    USB_SERIAL.printf("Unknown command: '%s'. Type 'h' or 'help' for help.\n", input.c_str());
}

//
void printResult(const dshot_result_t &result)
{
    if (result.success)
    {
        USB_SERIAL.printf("SUCCESS\n");
    }
    else
    {
        USB_SERIAL.printf("FAILED - %s \n", result.msg);
    }
}

//
void printSystemStatus()
{
    USB_SERIAL.println("\n=== System Status ===");
    USB_SERIAL.printf("Current throttle: %u\n", throttle_now);
    USB_SERIAL.printf("Continuous mode: %s\n", throttle_now > 0 ? "ACTIVE" : "INACTIVE");
    USB_SERIAL.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
    USB_SERIAL.printf("Uptime: %lu seconds\n", millis() / 1000);
}

//
void printMenu()
{
    USB_SERIAL.println("**********************************************");
    USB_SERIAL.println("          DShot Command Manager Menu          ");
    USB_SERIAL.println("**********************************************");
    USB_SERIAL.println(" 1                  - Stop Motor");
    USB_SERIAL.println(" 2                  - Activate Beacon 1");
    USB_SERIAL.println(" 3                  - Set Normal Spin");
    USB_SERIAL.println(" 4                  - Set Reversed Spin");
    USB_SERIAL.println(" 5                  - Get ESC Info");
    USB_SERIAL.println(" 6                  - Turn LED 0 ON");
    USB_SERIAL.println(" 7                  - Turn LED 0 OFF");
    USB_SERIAL.println(" 0                  - Emergency Stop");
    USB_SERIAL.println("**********************************************");
    USB_SERIAL.println(" cmd <number>       - Send Command (0 - 47)");
    USB_SERIAL.println(" throttle <value>   - Set throttle (48 - 2047)");
    USB_SERIAL.println("**********************************************");
    USB_SERIAL.println(" info               - Show DShot signal info");
    USB_SERIAL.println(" status             - Show system status");
    if (IS_BIDIRECTIONAL)
    {
        USB_SERIAL.println(" rpm                - Get telemetry data");
    }
    USB_SERIAL.println(" h / help           - Show this Menu");
    USB_SERIAL.println("**********************************************");
    USB_SERIAL.println("EXAMPLE INPUT:");
    USB_SERIAL.println(" cmd 5              - Get ESC Info");
    USB_SERIAL.println(" throttle 1000      - Set throttle to 1000");
    USB_SERIAL.println("**********************************************");
}
