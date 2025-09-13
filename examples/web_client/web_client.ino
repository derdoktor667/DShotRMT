/**
 * @file web_client.ino
 * @brief DShotRMT Web Control as WiFi Client
 * @author Wastl Kraus
 * @date 2025-09-11
 * @license MIT
 */

#include <Arduino.h>
#include <WiFi.h>

#include <DShotRMT.h>

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

// Web Site Content
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>DShotRMT Web Client</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
            display: flex;
            flex-direction: column;
            align-items: center;
            background-color: #2c3e50;
            color: #ecf0f1;
            margin: 0;
            height: 100dvh;
            justify-content: center;
        }

        h1 {
            font-size: 1.5em;
            font-weight: bold;
            margin-bottom: 20px;
        }

        .control-container {
            background-color: #34495e;
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
            text-align: center;
            width: 85%;
            max-width: 500px;
        }

        /* Arming Switch Styles */
        .arming-section {
            margin-bottom: 25px;
            padding: 15px;
            background-color: #2c3e50;
            border-radius: 8px;
            border: 2px solid #e74c3c;
        }

        .arming-switch {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 15px;
            margin-bottom: 10px;
        }

        .switch {
            position: relative;
            display: inline-block;
            width: 60px;
            height: 34px;
        }

        .switch input {
            opacity: 0;
            width: 0;
            height: 0;
        }

        .slider-switch {
            position: absolute;
            cursor: pointer;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background-color: #e74c3c;
            -webkit-transition: .4s;
            transition: .4s;
            border-radius: 34px;
        }

        .slider-switch:before {
            position: absolute;
            content: "";
            height: 26px;
            width: 26px;
            left: 4px;
            bottom: 4px;
            background-color: white;
            -webkit-transition: .4s;
            transition: .4s;
            border-radius: 50%;
        }

        input:checked+.slider-switch {
            background-color: #27ae60;
        }

        input:checked+.slider-switch:before {
            -webkit-transform: translateX(26px);
            -ms-transform: translateX(26px);
            transform: translateX(26px);
        }

        .arming-label {
            font-size: 1.2em;
            font-weight: bold;
        }

        .arming-status {
            font-size: 0.9em;
            margin-top: 5px;
        }

        .status-disarmed {
            color: #e74c3c;
        }

        .status-armed {
            color: #27ae60;
        }

        /* Throttle Section */
        .throttle-section {
            opacity: 0.3;
            transition: opacity 0.3s ease;
        }

        .throttle-section.armed {
            opacity: 1;
        }

        #throttleValue {
            font-size: 2.5em;
            font-weight: bold;
            color: #3498db;
            margin-bottom: 20px;
        }

        #throttleSlider {
            appearance: none;
            width: 100%;
            height: 25px;
            background: #2c3e50;
            outline: none;
            border-radius: 12px;
        }

        #throttleSlider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 40px;
            height: 40px;
            background: #3498db;
            cursor: pointer;
            border-radius: 50%;
        }

        #throttleSlider::-moz-range-thumb {
            width: 40px;
            height: 40px;
            background: #3498db;
            cursor: pointer;
            border-radius: 50%;
            border: none;
        }

        .stats {
            margin-top: 20px;
            font-size: 1.2em;
        }

        .stats span {
            font-weight: bold;
            color: #e67e22;
        }

        .warning-text {
            color: #e74c3c;
            font-size: 0.9em;
            margin-top: 10px;
            font-style: italic;
        }
    </style>
</head>

<body>
    <h1>DShotRMT Web Client</h1>
    <div class="control-container">
        <!-- Arming Section -->
        <div class="arming-section">
            <div class="arming-switch">
                <span class="arming-label">ARMING SWITCH</span>
                <label class="switch">
                    <input type="checkbox" id="armingSwitch">
                    <span class="slider-switch"></span>
                </label>
            </div>
            <div class="arming-status">
                <span id="armingStatus" class="status-disarmed">DISARMED</span>
            </div>
            <div class="warning-text">
                ⚠️ Disabled when disarmed!
            </div>
        </div>

        <!-- Throttle Section -->
        <div class="throttle-section" id="throttleSection">
            <div id="throttleValue">0</div>
            <input type="range" min="48" max="2047" value="0" id="throttleSlider" disabled>
        </div>

        <div class="stats">
            RPM: <span id="rpmValue">--</span>
        </div>
    </div>

    <script>
        const gateway = `ws://${window.location.hostname}/ws`;
        let websocket;
        let isArmed = false;

        // Init WebSocket
        window.addEventListener('load', () => {
            initWebSocket();
        });

        function initWebSocket() {
            console.log('Trying to open a WebSocket connection...');

            websocket = new WebSocket(gateway);
            websocket.onopen = onOpen;
            websocket.onclose = onClose;
            websocket.onmessage = onMessage;
        }

        function onOpen(event) {
            console.log('Connection opened');
        }

        function onClose(event) {
            console.log('Connection closed');
            setTimeout(initWebSocket, 2000);
        }

        // Getting data from sketch
        function onMessage(event) {
            try {
                const data = JSON.parse(event.data);

                if (data.rpm !== undefined) {
                    document.getElementById('rpmValue').innerText = data.rpm;
                }

                // Sync web and serial throttle inputs
                if (data.throttle !== undefined) {
                    if (isArmed) {
                        document.getElementById('throttleSlider').value = data.throttle;
                        document.getElementById('throttleValue').innerText = data.throttle;
                    }
                }

                // Sync arming status if received from ESP32
                if (data.armed !== undefined) {
                    isArmed = data.armed;
                    updateArmingUI();
                }

            } catch (e) {
                console.error("Error parsing JSON: ", e);
            }
        }

        // Elements
        const slider = document.getElementById('throttleSlider');
        const sliderValue = document.getElementById('throttleValue');
        const armingSwitch = document.getElementById('armingSwitch');
        const armingStatus = document.getElementById('armingStatus');
        const throttleSection = document.getElementById('throttleSection');

        // Arming switch event
        armingSwitch.addEventListener('change', () => {
            isArmed = armingSwitch.checked;
            updateArmingUI();

            // Send arming status to ESP32
            const message = JSON.stringify({
                "armed": isArmed,
                "throttle": isArmed ? parseInt(slider.value) : 0
            });

            console.log("Sending arming status: ", message);
            websocket.send(message);

            // If disarmed, set throttle to 0
            if (!isArmed) {
                slider.value = 0;
                sliderValue.innerText = 0;
            }
        });

        // Update UI based on arming status
        function updateArmingUI() {
            
            // Synch checkbox, as well
            armingSwitch.checked = isArmed;

            if (isArmed) {
                armingStatus.innerText = 'ARMED';
                armingStatus.className = 'status-armed';
                throttleSection.classList.add('armed');
                slider.disabled = false;
            } else {
                armingStatus.innerText = 'DISARMED';
                armingStatus.className = 'status-disarmed';
                throttleSection.classList.remove('armed');
                slider.disabled = true;
                slider.value = 0;
                sliderValue.innerText = 0;
            }
        }

        // Throttle slider event
        slider.addEventListener('input', () => {
            if (!isArmed) {
                slider.disabled = true;
                slider.value = 0;
                sliderValue.innerText = 0;
                return;
            }

            const throttle = slider.value;
            sliderValue.innerText = throttle;

            const message = JSON.stringify({
                "throttle": parseInt(throttle),
                "armed": isArmed
            });

            console.log("Sending throttle: ", message);
            websocket.send(message);
        });

        // Initialize UI
        updateArmingUI();
    </script>
</body>

</html>

)rawliteral";

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
        USB_SERIAL.println("*** Web interface not available ***");
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
    }
    else
    {
        USB_SERIAL.println(" Web Interface: NOT AVAILABLE             ");
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
