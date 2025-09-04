[![Arduino CI](https://github.com/derdoktor667/DShotRMT/actions/workflows/ci.yml/badge.svg?event=push)](https://github.com/derdoktor667/DShotRMT/actions/workflows/ci.yml)

# DShotRMT - ESP32 Library (Rewrite for ESP-IDF 5)

A modern, robust C++ library for generating DShot signals on the ESP32 using the new ESP-IDF 5 RMT encoder API (`rmt_tx.h` / `rmt_rx.h`).  
Supports all standard DShot modes (150, 300, 600) and features continuous frame transmission with configurable pause.  
**Now with BiDirectional DShot support and advanced command management!**

> The legacy version (using the old `rmt.h` API) is still available in the `oldAPI` branch.

---

## 🚀 Features

- **All DShot Modes:** DSHOT150, DSHOT300 (default), DSHOT600, (DSHOT1200)
- **BiDirectional DShot:** Experimental support for RPM feedback
- **Advanced Command Manager:** High-level API for ESC configuration and control
- **Command Sequences:** Predefined initialization and calibration sequences
- **Continuous Frames:** Independent timed, Hardware signal generation
- **Configurable Pause:** Ensures ESCs can reliably detect frame boundaries
- **Simple API:** Easy integration into your Arduino or ESP-IDF project

---

## 📦 Installation

Clone this repository and add it to your Arduino libraries or ESP-IDF components.

```sh
git clone https://github.com/derdoktor667/DShotRMT.git
```

---

## ⚡ Quick Start

### Basic Usage (DShotRMT)

```cpp
#include <DShotRMT.h>

// Create motor instance
DShotRMT motor(17, DSHOT300);

void setup() {
    Serial.begin(115200);
    motor.begin();
}

void loop() {
    motor.sendThrottle(1000);  // Send throttle value
    delay(20);
}
```

### Advanced Usage (DShotCommandManager)

```cpp
#include <DShotRMT.h>
#include <DShotCommandManager.h>

// Create motor and command manager instances
DShotRMT motor(17, DSHOT300);
DShotCommandManager cmdManager(motor);

void setup() {
    Serial.begin(115200);
    motor.begin();
    cmdManager.begin();
    
    // Execute initialization sequence
    cmdManager.executeInitSequence();
}

void loop() {
    // Your main code here
}
```

---

## 🎛️ DShotCommandManager API

The `DShotCommandManager` provides a high-level interface for ESC control and configuration:

### Motor Control
- `stopMotor()` - Stop motor immediately
- `set3DMode(bool enable)` - Enable/disable 3D mode
- `setSpinDirection(bool reversed)` - Set motor spin direction
- `saveSettings()` - Save current settings to ESC

### LED Control (BLHeli32 only)
- `setLED(uint8_t led_number, bool state)` - Control ESC LEDs (0-3)

### Beacon Functions
- `activateBeacon(uint8_t beacon_number)` - Activate motor beeping (1-5)

### Telemetry
- `setExtendedTelemetry(bool enable)` - Enable/disable extended telemetry
- `requestESCInfo()` - Request ESC information

### Command Sequences
- `executeInitSequence()` - Basic ESC initialization
- `executeCalibrationSequence()` - ESC calibration sequence
- `executeSequence(sequence, length)` - Custom command sequences

### Utility Functions
- `getCommandName(command)` - Get command name as string
- `isValidCommand(command)` - Validate command
- `printStatistics()` - Print execution statistics
- `resetStatistics()` - Reset execution counters

---

## 📚 Examples

### 1. Basic DShot Control
Use the `dshot300.ino` example for simple throttle control.

### 2. Advanced Command Management
Use the `command_manager.ino` example for interactive ESC control:

```
=== DShot Command Manager Menu ===
Basic Commands:
  1 - Stop Motor
  2 - Activate Beacon 1
  3 - Set Normal Spin Direction
  4 - Set Reversed Spin Direction
  5 - Enable 3D Mode
  6 - Disable 3D Mode
  7 - Save Settings
  8 - Turn LED 0 ON
  9 - Turn LED 0 OFF

Sequences:
  i - Execute Initialization Sequence
  c - Execute Calibration Sequence

Advanced:
  cmd <number> - Send DShot command (0 - 47)
  throttle <value> - Set throttle (48 - 2047)
  throttle 0 - Stop sending throttle
```

---

## 📚 DShot Protocol Overview

DShot transmits 16-bit packets to brushless ESCs:

- **11 bits:** Throttle value
- **1 bit:** Telemetry request
- **4 bits:** Checksum (CRC)

Data is sent MSB-first. Pulse timing depends on the selected DShot mode.

| DSHOT | Bitrate     | TH1   | TH0    | Bit Time (µs) | Frame Time (µs) |
|-------|-------------|-------|--------|---------------|-----------------|
| 150   | 150 kbit/s  | 5.00  | 2.50   | 6.67          | ~106.72         |
| 300   | 300 kbit/s  | 2.50  | 1.25   | 3.33          | ~53.28          |
| 600   | 600 kbit/s  | 1.25  | 0.625  | 1.67          | ~26.72          |

Each frame is followed by a pause to help ESCs detect separate frames.

![DShotRMT](https://raw.githubusercontent.com/derdoktor667/DShotRMT/refs/heads/main/img/dshot300.png)

---

## 🔒 Checksum Calculation

The checksum is calculated over the first 12 bits (throttle + telemetry):

```c
crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
```

### Bidirectional DSHOT

Bidirectional DSHOT (sometimes called "inverted DSHOT") inverts the signal level:  
A logical '1' is low, and a '0' is high. This signals the ESC to send telemetry packets back.

**Bidirectional CRC:**

```c
crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
```

> **Note:** Bidirectional DShot is experimental. Further hardware testing is needed.

---

## 🛠️ ESP32 RMT Peripheral

The RMT (Remote Control) peripheral generates accurate, hardware-timed signals for controlling external devices.  
Perfect for DShot:
- Utilizes latest ESP-IDF APIs  
- Hardware-timed pulses  
- CPU-independent  
- Loop mode with inter-frame pause  
- Reliable under system load

---

## 📝 Core API Reference

### DShotRMT Class
- `DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool isBidirectional)`
- `uint16_t begin()`
- `bool sendThrottle(uint16_t throttle)`
- `bool sendCommand(uint16_t command)`
- `uint16_t getERPM()` - Get eRPM (bidirectional mode only)
- `uint32_t getMotorRPM(uint8_t magnet_count)` - Convert to motor RPM

### DShotCommandManager Class
- `DShotCommandManager(DShotRMT &dshot_instance)`
- `bool begin()`
- `dshot_command_result_t sendCommand(dshot_commands_t command, uint16_t repeat_count = 1)`
- `dshot_command_result_t sendCommandWithDelay(dshot_commands_t command, uint16_t repeat_count, uint32_t delay_ms)`

All command methods return a `dshot_command_result_t` structure containing:
- `bool success` - Command execution status
- `uint32_t execution_time_us` - Execution time in microseconds
- `const char* error_message` - Error description

---

## 🎯 DShot Commands

The library supports all standard DShot commands:

| Command | Value | Description |
|---------|-------|-------------|
| MOTOR_STOP | 0 | Stop motor |
| BEACON1-5 | 1-5 | Motor beeping |
| ESC_INFO | 6 | Request ESC information |
| SPIN_DIRECTION_1/2 | 7-8 | Set spin direction |
| 3D_MODE_OFF/ON | 9-10 | 3D mode control |
| SAVE_SETTINGS | 12 | Save settings to ESC |
| EXTENDED_TELEMETRY_ENABLE/DISABLE | 13-14 | Telemetry control |
| LED0-3_ON/OFF | 22-29 | LED control (BLHeli32) |
| AUDIO_STREAM_MODE | 30 | KISS audio mode |
| SILENT_MODE | 31 | KISS silent mode |

---

## 📖 References

- [DSHOT – the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [DSHOT in the Dark](https://dmrlawson.co.uk/index.php/2017/12/04/dshot-in-the-dark/)
- [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)

---

## 📄 License

MIT License – see [LICENSE](LICENSE)

---

## 👤 Author

**Wastl Kraus**  
GitHub: [@derdoktor667](https://github.com/derdoktor667)  
Website: [wir-sind-die-matrix.de](https://wir-sind-die-matrix.de)
