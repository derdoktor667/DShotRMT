/**
 * @file web_control.ino
 * @brief Demo sketch for DShotRMT library
 * @author Wastl Kraus
 * @date 2025-09-09
 * @license MIT
 */

#include <Arduino.h>
#include <WiFi.h>

#include <DShotRMT.h>

#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Wifi Configuration
static constexpr auto *ssid = "DShotRMT Control";
static constexpr auto *password = "12345678";

IPAddress local_IP(10, 10, 10, 1);
IPAddress gateway(0, 0, 0, 0);
IPAddress subnet(255, 255, 255, 0);

// USB serial port settings
static constexpr auto &USB_SERIAL = Serial;
static constexpr auto USB_SERIAL_BAUD = 115200;

// Motor configuration - Pin number or GPIO_PIN
static constexpr auto MOTOR01_PIN = 17;

// Supported: DSHOT150, DSHOT300, DSHOT600, (DSHOT1200)
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;

// BiDirectional DShot Support (default: false)
static constexpr auto IS_BIDIRECTIONAL = false;

// Motor magnet count for RPM calculation
static constexpr auto MOTOR01_MAGNET_COUNT = 14;

// Creates the motor instance
DShotRMT motor01(MOTOR01_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);

// Web Server Configuration
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Global variables
static uint16_t throttle = DSHOT_CMD_MOTOR_STOP;
static bool isArmed = false;
static bool continuous_throttle = true;

// Helpers (forward declaration)
void printMenu();
void handleSerialInput(const String &input);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void setArmingStatus(bool armed);

//
void setup()
{
    USB_SERIAL.begin(USB_SERIAL_BAUD);

    motor01.begin();
    motor01.printCpuInfo();

    // Set IP Address
    WiFi.softAPConfig(local_IP, gateway, subnet);

    // Start Wifi Access Point
    USB_SERIAL.println("\nStarting Access Point...");
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();

    USB_SERIAL.print("Access Point IP address: ");
    USB_SERIAL.println(IP);

    // Init WebSockets and Webserver
    USB_SERIAL.println("\nStarting Webserver...");
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send_P(200, "text/html", index_html); });

    server.begin();
    USB_SERIAL.println("HTTP server started.");

    // Initialize with disarmed state
    setArmingStatus(false);

    printMenu();
}

void loop()
{
    static uint64_t last_serial_update = 0;
    static uint16_t last_sent_throttle = DSHOT_CMD_MOTOR_STOP;
    static bool last_sent_armed = false;
    static String last_sent_rpm = "N/A";

    // Handle serial input
    if (USB_SERIAL.available() > 0)
    {
        String input = USB_SERIAL.readStringUntil('\n');
        input.trim();
        if (input.length() > 0)
        {
            handleSerialInput(input);
        }
    }

    // Send throttle value only if armed and continuous mode is enabled
    if (isArmed && continuous_throttle && throttle > 0)
    {
        motor01.sendThrottle(throttle);
    }
    else if (!isArmed && continuous_throttle)
    {
        // Ensure motor is stopped when disarmed
        motor01.sendCommand(DSHOT_CMD_MOTOR_STOP);
    }

    // Print motor stats every 3 seconds in continuous mode
    if ((esp_timer_get_time() - last_serial_update >= 3000000))
    {
        motor01.printDShotInfo();

        USB_SERIAL.println(" ");

        // Get Motor RPM if bidirectional and armed
        if (IS_BIDIRECTIONAL && isArmed)
        {
            dshot_telemetry_result_t telem_result = motor01.getTelemetry(MOTOR01_MAGNET_COUNT);
            printDShotTelemetry(telem_result);
        }

        USB_SERIAL.println("Type 'help' to show Menu");

        // Time Stamp
        last_serial_update = esp_timer_get_time();
    }

    // Update JSON on data change
    String current_rpm = "N/A";

    if (IS_BIDIRECTIONAL && isArmed)
    {
        dshot_telemetry_result_t telem_result = motor01.getTelemetry(MOTOR01_MAGNET_COUNT);
        current_rpm = String(telem_result.motor_rpm);
    }

    if (throttle != last_sent_throttle || isArmed != last_sent_armed || current_rpm != last_sent_rpm)
    {
        // Generate JSON for Webserver
        JsonDocument doc;
        doc["throttle"] = isArmed ? throttle : 0;
        doc["armed"] = isArmed;
        doc["rpm"] = current_rpm;

        String json_output;
        json_output.reserve(256);
        serializeJson(doc, json_output);

        if (ws.count() > 0)
        {
            ws.textAll(json_output);
        }

        // Update last run
        last_sent_throttle = throttle;
        last_sent_armed = isArmed;
        last_sent_rpm = current_rpm;
    }

    ws.cleanupClients();
}

//
void setArmingStatus(bool armed)
{
    isArmed = armed;

    if (armed)
    {
        continuous_throttle = true;
        return;
    }

    // Safety: Stop motor and reset throttle when disarming
    throttle = 0;
    continuous_throttle = false;
    motor01.sendCommand(DSHOT_CMD_MOTOR_STOP);
    USB_SERIAL.println(" ");
    USB_SERIAL.println("=== MOTOR DISARMED - SAFETY STOP EXECUTED ===");
}

//
void printMenu()
{
    USB_SERIAL.println(" ");
    USB_SERIAL.println("***********************************************");
    USB_SERIAL.println("        --- DShotRMT Demo & Web UI ---         ");
    USB_SERIAL.println("***********************************************");
    USB_SERIAL.println("        Web Config:  http://10.10.10.1         ");
    USB_SERIAL.println("***********************************************");
    USB_SERIAL.println(" arm              - Arm motor");
    USB_SERIAL.println(" disarm           - Disarm motor (safety)");
    USB_SERIAL.println(" <value>          - Set throttle (48 – 2047)");
    USB_SERIAL.println(" 0                - Stop motor");
    USB_SERIAL.println("***********************************************");
    USB_SERIAL.println(" cmd <number>     - Send DShot command (0 - 47)");
    USB_SERIAL.println(" info             - Show motor info");
    if (IS_BIDIRECTIONAL)
    {
        USB_SERIAL.println(" rpm              - Get telemetry data");
    }
    USB_SERIAL.println("***********************************************");
    USB_SERIAL.println(" h / help         - Show this Menu");
    USB_SERIAL.println("***********************************************");
    USB_SERIAL.printf(" Current Status: %s\n", isArmed ? "ARMED" : "DISARMED");
    USB_SERIAL.println("***********************************************");
}

// Handle serial inputs and updates global variables
void handleSerialInput(const String &input)
{
    if (input == "arm")
    {
        setArmingStatus(true);
        return;
    }

    if (input == "0" || input == "disarm")
    {
        setArmingStatus(false);
        return;
    }

    if (input == "info")
    {
        motor01.printDShotInfo();
        USB_SERIAL.println(" ");
        USB_SERIAL.printf("Arming Status: %s\n", isArmed ? "ARMED" : "DISARMED");
        return;
    }

    if (input == "rpm" && IS_BIDIRECTIONAL)
    {
        if (isArmed)
        {
            dshot_telemetry_result_t result = motor01.getTelemetry(MOTOR01_MAGNET_COUNT);
            printDShotTelemetry(result);
        }
        else
        {
            USB_SERIAL.println(" ");
            USB_SERIAL.println("Cannot read RPM - Motor is DISARMED");
        }
        return;
    }

    if (input.startsWith("cmd "))
    {
        if (!isArmed)
        {
            USB_SERIAL.println(" ");
            USB_SERIAL.println("Cannot send command - Motor is DISARMED. Use 'arm' command first.");
            return;
        }

        continuous_throttle = false;
        int cmd_num = input.substring(4).toInt();

        if (cmd_num >= DSHOT_CMD_MOTOR_STOP && cmd_num <= DSHOT_CMD_MAX)
        {
            dshot_result_t result = motor01.sendCommand(cmd_num);
            printDShotResult(result);
        }
        else
        {
            USB_SERIAL.println(" ");
            USB_SERIAL.printf("Invalid command: %d (valid range: 0 - %d)\n", cmd_num, DSHOT_CMD_MAX);
        }
        return;
    }

    if (input == "h" || input == "help")
    {
        printMenu();
        return;
    }

    if (input == "status")
    {
        USB_SERIAL.println(" ");
        USB_SERIAL.printf("Arming Status: %s\n", isArmed ? "ARMED" : "DISARMED");
        USB_SERIAL.printf("Current Throttle: %u\n", throttle);
        USB_SERIAL.printf("Continuous Mode: %s\n", continuous_throttle ? "ACTIVE" : "INACTIVE");
        return;
    }

    // Handle throttle input
    int throttle_value = input.toInt();

    if (throttle_value >= DSHOT_THROTTLE_MIN && throttle_value <= DSHOT_THROTTLE_MAX)
    {
        if (!isArmed)
        {
            USB_SERIAL.println(" ");
            USB_SERIAL.println("Cannot set throttle - Motor is DISARMED. Use 'arm' command first.");
            return;
        }

        throttle = throttle_value;
        continuous_throttle = true;

        dshot_result_t result = motor01.sendThrottle(throttle);

        if (result.success)
        {
            USB_SERIAL.println(" ");
            USB_SERIAL.printf("Throttle set to %u (continuous mode active)\n", throttle);
        }
        return;
    }

    if (throttle_value == 0)
    {
        throttle = 0;
        continuous_throttle = false;
        dshot_result_t result = motor01.sendCommand(DSHOT_CMD_MOTOR_STOP);
        printDShotResult(result);
        return;
    }

    // Invalid input
    USB_SERIAL.println(" ");
    USB_SERIAL.printf("Invalid input: '%s'\n", input.c_str());
    USB_SERIAL.printf("Valid throttle range: %d - %d\n", DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    USB_SERIAL.println("Use 'arm' to enable motor control");
}

// Websocket request processing
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, data, len);

    if (error)
    {
        USB_SERIAL.print(F("deserializeJson() failed: "));
        USB_SERIAL.println(error.c_str());
        return;
    }

    bool armedFromWeb = false;

    // Handle arming status
    if (doc.containsKey("armed"))
    {
        bool armed = doc["armed"];
        setArmingStatus(armed);
        armedFromWeb = true;
    }

    // Handle throttle value (only if armed)
    if (doc.containsKey("throttle"))
    {
        if (!isArmed)
        {
            throttle = 0;
            continuous_throttle = false;
            // Ignore throttle commands when disarmed
            USB_SERIAL.println(" ");
            USB_SERIAL.println("Web throttle command ignored - Motor is DISARMED");
            return;
        }

        uint16_t web_throttle = doc["throttle"];

        if (web_throttle == 0)
        {
            throttle = 0;
            continuous_throttle = false;
            motor01.sendCommand(DSHOT_CMD_MOTOR_STOP);
        }
        else if (web_throttle >= DSHOT_THROTTLE_MIN && web_throttle <= DSHOT_THROTTLE_MAX)
        {
            throttle = web_throttle;
            continuous_throttle = true;
        }
    }

    // Webserver arms with DSHOT_THROTTLE_MIN
    if (armedFromWeb && isArmed)
    {
        continuous_throttle = true;
        motor01.sendThrottle(throttle);
        USB_SERIAL.println(" ");
        USB_SERIAL.printf("Motor armed via Web - throttle set to %i\n", throttle);
    }
}

// Websocket request handler
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        USB_SERIAL.printf("Web Client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());

        // Send current arming status to new client
        {
            JsonDocument doc;
            doc["armed"] = isArmed;
            doc["throttle"] = isArmed ? throttle : 0;
            String json_output;
            serializeJson(doc, json_output);
            client->text(json_output);
        }
        break;

    case WS_EVT_DISCONNECT:
        USB_SERIAL.printf("Web Client #%u disconnected\n", client->id());
        break;

    case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;

    case WS_EVT_PONG:
    case WS_EVT_ERROR:
        break;
    }
}
