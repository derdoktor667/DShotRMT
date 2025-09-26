# DShotRMT - ESP32 RMT DShot Driver

[![Arduino CI](https://github.com/derdoktor667/DShotRMT/actions/workflows/ci.yml/badge.svg)](https://github.com/derdoktor667/DShotRMT/actions/workflows/ci.yml)

A C++ library for generating DShot signals on ESP32 microcontrollers using the **modern ESP-IDF 5 RMT encoder API** (`rmt_tx.h` / `rmt_rx.h`). It leverages the standard `rmt_bytes_encoder` to ensure an efficient, hardware-timed, and maintainable implementation. The library provides a simple way to control brushless motors in both Arduino and ESP-IDF projects. The legacy version using the old `rmt.h` API is available in the `oldAPI` branch.

## 🚀 Core Features

- **Multiple DShot Modes:** Supports DSHOT150, DSHOT300, DSHOT600, and DSHOT1200.
- **Bidirectional DShot Support:** Implemented, but note that official support is limited due to potential instability and external hardware requirements. Use with caution.
- **Hardware-Timed Signals:** Precise signal generation using the ESP32 RMT peripheral, ensuring stable and reliable motor control.
- **Simple API:** Easy-to-use C++ class with intuitive methods like `sendThrottlePercent()`.
- **Robust Error Handling:** Provides detailed feedback on operation success or failure via `dshot_result_t`.
- **Efficient and Lightweight:** The core library has no external dependencies.
- **Arduino and ESP-IDF Compatible:** Can be used in both Arduino and ESP-IDF projects.

## 📦 Installation

### Arduino IDE

1.  Open the Arduino Library Manager (`Sketch` > `Include Library` > `Manage Libraries...`).
2.  Search for "DShotRMT" and click "Install".
3.  Alternatively, you can clone this repository or download it as a ZIP file and place it in your Arduino libraries folder (`~/Arduino/libraries/DShotRMT/`).

### PlatformIO

Add the following to your `platformio.ini` file:

```ini
lib_deps = 
    https://github.com/derdoktor667/DShotRMT.git
```

## ⚡ Quick Start

Here's a basic example of how to use the `DShotRMT` library to control a motor:

```cpp
#include <Arduino.h>
#include <DShotRMT.h> // Include the DShotRMT library

// Define the GPIO pin connected to the motor ESC
const gpio_num_t MOTOR_PIN = GPIO_NUM_27;

// Create a DShotRMT instance for DSHOT300
DShotRMT motor(MOTOR_PIN, DSHOT300);

void setup() {
  Serial.begin(115200);

  // Initialize the DShot motor
  motor.begin();

  // Print CPU Info
  DShotRMT::printCpuInfo(Serial);

  Serial.println("Motor initialized. Ramping up to 25% throttle...");
  
  // Ramp up to 25% throttle over 2.5 seconds
  for (int i = 0; i <= 25; i++) {
    motor.sendThrottlePercent(i);
    delay(100);
  }
  
  Serial.println("Stopping motor.");
  motor.sendThrottlePercent(0);

  // Print DShot Info
  DShotRMT::printDShotInfo(motor, Serial);
}

void loop() {
  // Your main code here
}
```

## 🎮 Examples

The `examples` folder contains more advanced examples:

- **`throttle_percent`:** A focused example showing how to control motor speed using percentage values (0-100) via the serial monitor.
- **`dshot300`:** A more advanced example demonstrating how to send raw DShot commands and receive telemetry via the serial monitor.
- **`web_control`:** A full-featured web application for controlling a motor from a web browser. It creates a WiFi access point and serves a web page with a throttle slider and arming switch.
- **`web_client`:** A variation of the `web_control` example that connects to an existing WiFi network instead of creating its own access point.

### Dependencies for Web Examples

The `web_control` and `web_client` examples require the following additional libraries:

- [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
- [ESPAsyncWebServer](https://github.com/ESP32Async/ESPAsyncWebServer)
- [AsyncTCP](https://github.com/ESP32Async/AsyncTCP)

You can install these libraries using the Arduino Library Manager or by adding them to your `platformio.ini` file:

```ini
lib_deps = 
    https://github.com/derdoktor667/DShotRMT.git
    https://github.com/bblanchon/ArduinoJson
    https://github.com/ESP32Async/ESPAsyncWebServer
    https://github.com/ESP32Async/AsyncTCP
```

## 📚 API Reference

The main class is `DShotRMT`. Here are the most important methods:

- `DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool is_bidirectional = false, uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT)`: Constructor to create a new DShotRMT instance. (Note: Bidirectional DShot is currently not officially supported.)
- `begin()`: Initializes the RMT peripheral and the DShot encoder.
- `sendThrottlePercent(float percent)`: Sends a throttle value as a percentage (0.0-100.0).
- `sendThrottle(uint16_t throttle)`: Sends a raw throttle value (48-2047) to the motor.
- `sendCommand(uint16_t command)`: Sends a DShot command (0-47) to the motor.
- `getTelemetry(uint16_t magnet_count)`: Receives and parses telemetry data from the motor (for bidirectional DShot, which is currently not officially supported).
- `printDShotResult(dshot_result_t &result, Stream &output = Serial)`: Helper function to print DShot operation results and telemetry to a specified serial output.
- `DShotRMT::printDShotInfo(const DShotRMT &dshot_rmt, Stream &output = Serial)`: Static helper function to print detailed DShot signal information for a given DShotRMT instance.
- `DShotRMT::printCpuInfo(Stream &output = Serial)`: Static helper function to print detailed CPU information.

## 🤝 Contributing

Contributions are welcome! Please fork the repository, create a feature branch, and submit a pull request.

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.