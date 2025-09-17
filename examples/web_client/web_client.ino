/**
 * @file web_client.ino
 * @brief DShotRMT Web Control as WiFi Client
 * @author Wastl Kraus
 * @date 2025-09-11
 * @license MIT
 */

#include <Arduino.h>
#include <Update.h>
#include <WiFi.h>

#include <DShotRMT.h>
#include <ota_update.h>
#include <web_content.h>

#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// WiFi Client Configuration - UPDATE THESE VALUES!
static constexpr auto *WIFI_SSID = "YOUR_SSID";         // Enter your WiFi SSID
static constexpr auto *WIFI_PASSWORD = "YOUR_PASSWORD"; // Enter your WiFi password

// Connection timeout in milliseconds
static constexpr auto WIFI_CONNECT_TIMEOUT = 20000;

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
static bool last_sent_armed = false;
static bool continuous_throttle = true;

// WiFi status tracking
static bool wifi_connected = false;

// Helpers (forward declaration)
void printMenu();
void handleSerialInput(const String &input);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
bool connectToWiFi();
void printWiFiStatus();
void setupOTA();
void handleOTAUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);

//
void setup()
{
    USB_SERIAL.begin(USB_SERIAL_BAUD);
    USB_SERIAL.println();
    USB_SERIAL.println("DShotRMT Web Client Demo Starting...");

    motor01.begin();
    motor01.printCpuInfo();

    // Connect to WiFi network
    USB_SERIAL.println("\nConnecting to WiFi network...");
    wifi_connected = connectToWiFi();

    if (wifi_connected)
    {
        // Setup OTA first
        setupOTA();

        // Init WebSockets and Webserver
        USB_SERIAL.println("\nStarting Webserver...");

        ws.onEvent(onWsEvent);
        server.addHandler(&ws);

        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send_P(200, "text/html", index_html); });

        server.begin();
        USB_SERIAL.println("HTTP server started.");

        printWiFiStatus();
    }
    else
    {
        USB_SERIAL.println("\n*** WARNING: WiFi connection failed! ***");
        USB_SERIAL.println("*** Web interface and OTA not available ***");
        USB_SERIAL.println("*** Only serial control available ***");
    }

    // Initialize with disarmed state
    setArmingStatus(false);

    printMenu();
}

//
void loop()
{
    static uint64_t last_serial_update = 0;
    static uint16_t last_sent_throttle = DSHOT_CMD_MOTOR_STOP;
    static String last_sent_rpm = "N/A";
    static uint64_t last_wifi_check = 0;

    // Check WiFi connection every 30 seconds
    if (esp_timer_get_time() - last_wifi_check >= 30000000)
    {
        if (WiFi.status() != WL_CONNECTED && wifi_connected)
        {
            USB_SERIAL.println("\n*** WiFi connection lost! ***");
            wifi_connected = false;
        }
        else if (WiFi.status() == WL_CONNECTED && !wifi_connected)
        {
            USB_SERIAL.println("\n*** WiFi connection restored! ***");
            wifi_connected = true;
            printWiFiStatus();
        }

        last_wifi_check = esp_timer_get_time();
    }

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
            dshot_result_t telem_result = motor01.getTelemetry(MOTOR01_MAGNET_COUNT);
            printDShotResult(telem_result);
        }

        USB_SERIAL.println("Type 'help' to show Menu");

        // Time Stamp
        last_serial_update = esp_timer_get_time();
    }

    // Update JSON on data change (only if WiFi connected)
    if (wifi_connected)
    {
        String current_rpm = "N/A";

        if (IS_BIDIRECTIONAL && isArmed)
        {
            dshot_result_t telem_result = motor01.getTelemetry(MOTOR01_MAGNET_COUNT);
            if (telem_result.success && telem_result.motor_rpm > 0)
            {
                current_rpm = String(telem_result.motor_rpm);
            }
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
}

// Setup OTA Update functionality
void setupOTA()
{
    USB_SERIAL.println("Setting up OTA Update...");

    // Serve OTA update page
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send_P(200, "text/html", ota_html); });

    // Handle OTA update upload
    server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request)
              {
            bool shouldReboot = !Update.hasError();

            AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", 
                shouldReboot ? "OK" : "FAIL");

            response->addHeader("Connection", "close");
            request->send(response);
            
            if (shouldReboot) {
                USB_SERIAL.println("OTA Update successful! Rebooting...");
                delay(1000);
                ESP.restart();
            } else {
                USB_SERIAL.println("OTA Update failed!");
            } }, handleOTAUpload);

    USB_SERIAL.println("OTA Update ready at: /update");
}

// Handle OTA upload process
void handleOTAUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
    static unsigned long ota_progress_millis = 0;

    if (!index)
    {
        // Safety: Ensure motor is stopped during update
        motor01.sendCommand(DSHOT_CMD_MOTOR_STOP);
        setArmingStatus(false);

        USB_SERIAL.printf("OTA Update Start: %s\n", filename.c_str());

        if (!Update.begin(UPDATE_SIZE_UNKNOWN))
        {
            Update.printError(USB_SERIAL);
            return;
        }
    }

    if (len)
    {
        if (Update.write(data, len) != len)
        {
            Update.printError(USB_SERIAL);
            return;
        }

        // Print progress every 2 seconds to avoid spam
        if (millis() - ota_progress_millis > 2000)
        {
            size_t progress = index + len;
            USB_SERIAL.printf("OTA Progress: %zu bytes\n", progress);
            ota_progress_millis = millis();
        }
    }

    if (final)
    {
        if (Update.end(true))
        {
            USB_SERIAL.printf("OTA Update Success: %zu bytes\n", index + len);
        }
        else
        {
            Update.printError(USB_SERIAL);
        }
    }
}

// Connect to WiFi network
bool connectToWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    USB_SERIAL.printf("Connecting to '%s'", WIFI_SSID);

    uint32_t start_time = millis();

    while (WiFi.status() != WL_CONNECTED && (millis() - start_time) < WIFI_CONNECT_TIMEOUT)
    {
        delay(500);
        USB_SERIAL.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        USB_SERIAL.println(" Connected!");
        return true;
    }
    else
    {
        USB_SERIAL.println(" Failed!");
        USB_SERIAL.printf("WiFi Status: %d\n", WiFi.status());
        return false;
    }
}

// Print current WiFi status
void printWiFiStatus()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        USB_SERIAL.println();
        USB_SERIAL.println("***********************************************");
        USB_SERIAL.println("               --- WIFI INFO ---               ");
        USB_SERIAL.println("***********************************************");
        USB_SERIAL.printf("SSID: %s\n", WiFi.SSID().c_str());
        USB_SERIAL.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
        USB_SERIAL.printf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
        USB_SERIAL.printf("Subnet: %s\n", WiFi.subnetMask().toString().c_str());
        USB_SERIAL.printf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
        USB_SERIAL.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
        USB_SERIAL.printf("MAC Address: %s\n", WiFi.macAddress().c_str());
        USB_SERIAL.println("***********************************************");
        USB_SERIAL.printf("Web Interface: http://%s\n", WiFi.localIP().toString().c_str());
        USB_SERIAL.printf("OTA Update: http://%s/update\n", WiFi.localIP().toString().c_str());
        USB_SERIAL.println("***********************************************");
    }
    else
    {
        USB_SERIAL.println("WiFi not connected!");
    }
}

//
void setArmingStatus(bool armed)
{
    isArmed = armed;

    if (armed)
    {
        continuous_throttle = true;
        // dirty - force JSON update for WebSocket clients
        last_sent_armed = !armed; // Trigger JSON send in next loop
        return;
    }

    // Safety: Stop motor and reset throttle when disarming
    throttle = 0;
    continuous_throttle = false;
    motor01.sendCommand(DSHOT_CMD_MOTOR_STOP);
    USB_SERIAL.println(" ");
    USB_SERIAL.println("=== MOTOR DISARMED - SAFETY STOP EXECUTED ===");

    // Force JSON update for WebSocket clients
    last_sent_armed = !armed; // Trigger JSON send in next loop
}

//
void printMenu()
{
    USB_SERIAL.println(" ");
    USB_SERIAL.println("***********************************************");
    USB_SERIAL.println("     --- DShotRMT Web Client Demo ---         ");
    USB_SERIAL.println("***********************************************");

    if (wifi_connected)
    {
        USB_SERIAL.printf(" Web Interface: http://%s \n", WiFi.localIP().toString().c_str());
        USB_SERIAL.printf(" OTA Update: http://%s/update \n", WiFi.localIP().toString().c_str());
    }
    else
    {
        USB_SERIAL.println(" Web Interface: NOT AVAILABLE             ");
        USB_SERIAL.println(" OTA Update: NOT AVAILABLE                ");
    }

    USB_SERIAL.println("***********************************************");
    USB_SERIAL.println(" arm              - Arm motor");
    USB_SERIAL.println(" disarm           - Disarm motor (safety)");
    USB_SERIAL.println(" <value>          - Set throttle (48 – 2047)");
    USB_SERIAL.println(" 0                - Stop motor");
    USB_SERIAL.println("***********************************************");
    USB_SERIAL.println(" cmd <number>     - Send DShot command (0 - 47)");
    USB_SERIAL.println(" info             - Show motor info");
    USB_SERIAL.println(" wifi             - Show WiFi status");
    USB_SERIAL.println(" reconnect        - Reconnect to WiFi");
    USB_SERIAL.println(" ota              - Show OTA info");
    if (IS_BIDIRECTIONAL)
    {
        USB_SERIAL.println(" rpm              - Get telemetry data");
    }
    USB_SERIAL.println("***********************************************");
    USB_SERIAL.println(" h / help         - Show this Menu");
    USB_SERIAL.println("***********************************************");
    USB_SERIAL.printf(" Current Status: %s\n", isArmed ? "ARMED" : "DISARMED");
    USB_SERIAL.printf(" WiFi Status: %s\n", wifi_connected ? "CONNECTED" : "DISCONNECTED");
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

    if (input == "wifi")
    {
        printWiFiStatus();
        return;
    }

    if (input == "ota")
    {
        if (wifi_connected)
        {
            USB_SERIAL.println(" ");
            USB_SERIAL.println("=== OTA UPDATE INFO ===");
            USB_SERIAL.printf("OTA Update URL: http://%s/update\n", WiFi.localIP().toString().c_str());
            USB_SERIAL.printf("Free Sketch Space: %u bytes\n", ESP.getFreeSketchSpace());
            USB_SERIAL.printf("Sketch Size: %u bytes\n", ESP.getSketchSize());
            USB_SERIAL.println("========================");
        }
        else
        {
            USB_SERIAL.println("OTA Update not available - WiFi not connected!");
        }
        return;
    }

    if (input == "reconnect")
    {
        USB_SERIAL.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        delay(1000);
        wifi_connected = connectToWiFi();
        if (wifi_connected)
        {
            printWiFiStatus();
        }
        return;
    }

    if (input == "rpm" && IS_BIDIRECTIONAL)
    {
        if (isArmed)
        {
            dshot_result_t result = motor01.getTelemetry(MOTOR01_MAGNET_COUNT);
            printDShotResult(result);
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
        USB_SERIAL.printf("WiFi Status: %s\n", wifi_connected ? "CONNECTED" : "DISCONNECTED");
        if (wifi_connected)
        {
            USB_SERIAL.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
            USB_SERIAL.printf("OTA URL: http://%s/update\n", WiFi.localIP().toString().c_str());
        }
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
