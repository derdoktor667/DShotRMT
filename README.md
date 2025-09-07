[![Arduino CI](https://github.com/derdoktor667/DShotRMT/actions/workflows/ci.yml/badge.svg?event=push)](https://github.com/derdoktor667/DShotRMT/actions/workflows/ci.yml)

# DShotRMT - ESP32 Library (Rewrite for ESP-IDF 5)

A modern, robust C++ library for generating DShot signals on the ESP32 using the new ESP-IDF 5 RMT encoder API (`rmt_tx.h` / `rmt_rx.h`).  
Supports all standard DShot modes (150, 300, 600, 1200) and features continuous frame transmission with configurable timing.  
**Now with BiDirectional DShot support and advanced command management!**

> The legacy version (using the old `rmt.h` API) is still available in the `oldAPI` branch.

---

## 🚀 Features

- **All DShot Modes:** DSHOT150, DSHOT300 (default), DSHOT600, DSHOT1200
- **BiDirectional DShot:** Full support for RPM telemetry feedback
- **Advanced Command Manager:** High-level API for ESC configuration and control
- **Command Sequences:** Predefined initialization and calibration sequences
- **Hardware-Timed Signals:** Independent, precise signal generation using ESP32 RMT peripheral
- **Configurable Timing:** Ensures ESCs can reliably detect frame boundaries
- **Error Handling:** Comprehensive result reporting with success/failure status
- **Simple API:** Easy integration into your Arduino or ESP-IDF project

---

## 📦 Installation

### Arduino IDE
1. Search "Arduino Library Manager" for "DShotRMT"

or

1. Clone this repository or download as ZIP
2. Place in your Arduino libraries folder (`~/Arduino/libraries/DShotRMT/`)
3. Restart Arduino IDE

### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps = 
    https://github.com/derdoktor667/DShotRMT.git
```

### Manual Installation
```sh
git clone https://github.com/derdoktor667/DShotRMT.git
```

---

## ⚡ Quick Start

### Basic Usage (DShotRMT)

```cpp
#include <DShotRMT.h>

// Create motor instance (GPIO 17, DSHOT300, non-bidirectional)
DShotRMT motor(17, DSHOT300, false);

void setup() {
    Serial.begin(115200);
    
    // Initialize the motor
    dshot_result_t result = motor.begin();
    if (result.success) {
        Serial.println("Motor initialized successfully");
    } else {
        Serial.printf("Motor init failed: %s\n", result.msg);
    }
}

void loop() {
    // Send throttle value (48-2047)
    dshot_result_t result = motor.sendThrottle(1000);
    if (!result.success) {
        Serial.printf("Throttle command failed: %s\n", result.msg);
    }
}
```

### Bidirectional DShot (RPM Telemetry)

```cpp
#include <DShotRMT.h>

// Enable bidirectional mode for telemetry
DShotRMT motor(17, DSHOT300, true);

void setup() {
    Serial.begin(115200);
    motor.begin();
}

void loop() {
    // Send throttle
    motor.sendThrottle(1000);
    
    // Get telemetry data
    dshot_telemetry_result_t telemetry = motor.getTelemetry(14); // 14 magnets
    if (telemetry.success) {
        Serial.printf("eRPM: %u, Motor RPM: %u\n", 
          telemetry.erpm, 
          telemetry.motor_rpm);
    }
}
```

---

## 📚 Examples

The library includes comprehensive examples:

### 1. Basic DShot Control (`dshot300.ino`)
- Simple throttle control
- Command execution
- Serial interface for testing
- Telemetry reading (if bidirectional enabled)

### 2. Advanced Command Management (`command_manager.ino`)
Interactive ESC control with full menu system:
```
=== DShot Command Manager Menu ===
 1 - Stop Motor
 2 - Activate Beacon 1
 3 - Set Normal Spin Direction
 4 - Set Reversed Spin Direction
 5 - Get ESC Info
 6 - Turn LED 0 ON
 7 - Turn LED 0 OFF
 0 - Emergency Stop

Advanced Commands:
 cmd <number>       - Send DShot command (0-47)
 throttle <value>   - Set throttle (48-2047)
 repeat cmd <num> count <count> - Repeat command
```

---

## 🔧 Hardware Configuration

### Supported DShot Modes

| Mode     | Bitrate     | Bit Time | Frame Time | Use Case |
|----------|-------------|----------|------------|----------|
| DSHOT150 | 150 kbit/s  | 6.67 µs  | ~107 µs    | Long wires, EMI-prone |
| DSHOT300 | 300 kbit/s  | 3.33 µs  | ~53 µs     | Standard (recommended) |
| DSHOT600 | 600 kbit/s  | 1.67 µs  | ~27 µs     | High performance |
| DSHOT1200| 1200 kbit/s | 0.83 µs  | ~13 µs     | Racing applications |

### GPIO Configuration
```cpp
// Using GPIO number
DShotRMT motor(17, DSHOT300);

// Using GPIO enum
DShotRMT motor(GPIO_NUM_17, DSHOT300);

// With bidirectional support
DShotRMT motor(17, DSHOT300, true);
```


---

## 🎯 DShot Commands

| Command | Value | Description | Usage |
|---------|-------|-------------|-------|
| MOTOR_STOP | 0 | Stop motor | Always available |
| BEACON1 - 5 | 1 - 5 | Motor beeping | Motor identification |
| ESC_INFO | 6 | Request ESC info | Get ESC version/settings |
| SPIN_DIRECTION_1/2 | 7 - 8 | Set spin direction | Motor configuration |
| 3D_MODE_OFF/ON | 9 - 10 | 3D mode control | Bidirectional flight |
| SAVE_SETTINGS | 12 | Save to EEPROM | Permanent configuration |
| EXTENDED_TELEMETRY_ENABLE/DISABLE | 13 - 14 | Telemetry control | Data transmission |
| SPIN_DIRECTION_NORMAL/REVERSED | 20 - 21 | Spin direction | Alias commands |
| LED0-3_ON/OFF | 22 - 29 | LED control | BLHeli32 only |
| AUDIO_STREAM_MODE | 30 | Audio mode toggle | KISS ESCs |
| SILENT_MODE | 31 | Silent mode toggle | KISS ESCs |

---

## 📚 DShot Protocol Details

![DShotRMT](https://raw.githubusercontent.com/derdoktor667/DShotRMT/refs/heads/main/img/dshot300.png)

### Packet Structure
Each DShot frame consists of 16 bits:
- **11 bits:** Throttle/command value (0-2047)
- **1 bit:** Telemetry request flag
- **4 bits:** CRC checksum

### Checksum Calculation
```cpp
// Standard DShot CRC
uint16_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;

// Bidirectional DShot (inverted CRC)
uint16_t crc = (~(data ^ (data >> 4) ^ (data >> 8))) & 0x0F;
```

### Bidirectional DShot
- **Inverted Logic:** High/low levels are inverted
- **GCR Encoding:** Telemetry uses Group Code Recording
- **21-bit Response:** 1 start + 16 data + 4 CRC bits
- **eRPM Data:** Electrical RPM transmitted back to controller

---

## 🛠️ ESP32 RMT Peripheral

The library utilizes the ESP32's RMT (Remote Control) peripheral for precise signal generation:

### Advantages
- **Hardware Timing:** No CPU intervention during transmission
- **Concurrent Operation:** Multiple channels can run simultaneously  
- **DMA Support:** Efficient memory-to-peripheral transfers

---

## 📖 References & Documentation

### DShot Protocol
- [DSHOT – the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [DSHOT in the Dark](https://dmrlawson.co.uk/index.php/2017/12/04/dshot-in-the-dark/)
- [Betaflight DShot Implementation](https://github.com/betaflight/betaflight)

### ESP32 Documentation
- [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
- [ESP-IDF RMT Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)
- [Arduino ESP32 Core](https://github.com/espressif/arduino-esp32)

---

## 🤝 Contributing

We welcome contributions! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes with tests
4. Submit a pull request

### Development Guidelines
- Follow existing code style
- Add documentation for new features
- Include examples where appropriate
- Test with real hardware when possible

### Reporting Issues
When reporting issues, please include:
- ESP32 board type and version
- Arduino/ESP-IDF version
- ESC type and firmware
- Complete error messages
- Minimal reproduction code

---

## 📄 License

MIT License – see [LICENSE](LICENSE)

---

## 👤 Author

**Wastl Kraus**  
- GitHub: [@derdoktor667](https://github.com/derdoktor667)  
- Website: [wir-sind-die-matrix.de](https://wir-sind-die-matrix.de)
