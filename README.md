[![Arduino CI](https://github.com/derdoktor667/DShotRMT/actions/workflows/ci.yml/badge.svg)](https://github.com/derdoktor667/DShotRMT/actions/workflows/ci.yml)

# DShotRMT - ESP32 Library (Rewrite for ESP-IDF 5)

A simple Arduino IDE / C++ library for generating DShot signals on the ESP32 (`fqbn: esp32:esp32:esp32`) using the ESP-IDF 5 RMT encoder API (`rmt_tx.h` / `rmt_rx.h`).  
Supports all standard DShot modes (150, 300, 600, 1200) and features signal generation and frame transmission with configurable timing.  

**Now with BiDirectional DShot support, advanced command management, and modern web control interface!**

> The legacy version (using the old `rmt.h` API) is still available in the `oldAPI` branch.

---

## 🚀 Features

- **All DShot Modes:** DSHOT150, DSHOT300 (default), DSHOT600, DSHOT1200
- **BiDirectional DShot:** Full support for RPM telemetry feedback
- **Web Control Interface:** Modern responsive web UI with WiFi access point
- **Safety Features:** Arming/disarming system with motor lockout protection
- **Dual Control Options:** Web interface and serial console control
- **Real-time Telemetry:** Live RPM monitoring and data display
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
    https://github.com/bblanchon/ArduinoJson
    https://github.com/ESP32Async/ESPAsyncWebServer
    https://github.com/ESP32Async/AsyncTCP
```

### Manual Installation
```sh
git clone https://github.com/derdoktor667/DShotRMT.git
```

### Dependencies

There are no dependencies for the main library. The extended
example sketches are using these libraries:

**Web Interface Examples (web_control.ino / web_client.ino):**
```ini
lib_deps = 
    https://github.com/derdoktor667/DShotRMT
    https://github.com/bblanchon/ArduinoJson
    https://github.com/ESP32Async/ESPAsyncWebServer
    https://github.com/ESP32Async/AsyncTCP
```

---

## ⚡ Quick Start

### Basic Usage (DShotRMT)

```cpp
// Generate "dshot300" example sketch with Arduino IDE / CLI.
```
---

## 🌐 Web Control Interface

The DShotRMT library now includes a modern web interface for wireless motor control:

### Features
- **Responsive Design:** Works on mobile phones, tablets, and desktop computers
- **WiFi Access Point:** Creates hotspot "DShotRMT Control" (Password: 12345678)
- **Safety System:** Arming/disarming switch prevents accidental motor activation
- **Real-time Control:** Instant throttle response via WebSocket communication
- **Live Telemetry:** Real-time RPM display (bidirectional mode only)
- **Auto-reconnect:** Automatically reconnects on connection loss

### Web Interface Access
1. Connect to WiFi network: **"DShotRMT Control"**
2. Password: **12345678**
3. Open browser and navigate to: **http://10.10.10.1**

### Web Client Mode
1. Setup SSID and Password in web_client.ino
2. Open serial for IP
3. Open browser, http://IP

### Safety Features
- Motor control is **disabled by default** (disarmed state)
- Toggle the **ARMING SWITCH** to enable motor control
- Throttle slider is **locked** when disarmed
- **Emergency stop** resets all values to safe state

### Technical Implementation
- **AsyncWebServer** for HTTP requests
- **WebSocket** communication for real-time data
- **JSON** message format for data exchange
- **WiFi SoftAP** mode for standalone operation
- **Automatic client cleanup** prevents memory leaks

### ⚠️ Known Issus
Make sure you are using these libraries for [ESPAsyncWebServer](https://github.com/ESP32Async/ESPAsyncWebServer) and [AsyncTCP](https://github.com/ESP32Async/AsyncTCP) to use "web_control.ino" example sketch.  

---

## 📚 Extras

### Basic DShot Control with Web Interface (`web_control.ino`)
- **Web Control Interface:** Modern responsive web UI accessible at `http://10.10.10.1`
- **WiFi Access Point:** Creates hotspot "DShotRMT Control" for wireless control
- **Safety Features:** Arming/disarming system with motor safety lockout
- **Real-time Data:** Live RPM telemetry display (bidirectional mode)
- **Dual Control:** Both web interface and serial console control
- **WebSocket Communication:** Real-time bidirectional data exchange

**Web Interface Features:**
- Responsive design optimized for mobile and desktop
- Visual arming switch with safety lockout
- Smooth throttle slider with real-time feedback
- Live RPM monitoring display
- Automatic reconnection on connection loss

---

## 🔧 Hardware Configuration

### Supported DShot Modes

| DSHOT | Bitrate     | TH1   | TH0    | Bit Time (µs) | Frame Time (µs) |
|-------|-------------|-------|--------|---------------|-----------------|
| 150   | 150 kbit/s  | 5.00  | 2.50   | 6.67          | ~106.72         |
| **300**   | **300 kbit/s**  | **2.50**  | **1.25**   | **3.33**          | **~53.28**          |
| 600   | 600 kbit/s  | 1.25  | 0.625  | 1.67          | ~26.72          |

For DShot, T1H length is always double T0H length.

### GPIO Configuration
```cpp
// Using GPIO number
DShotRMT motor(17, DSHOT300);

// Using GPIO enum
DShotRMT motor(GPIO_NUM_17, DSHOT300);

// With bidirectional support
DShotRMT motor(17, DSHOT300, true);

// Also possible, defaults (17, DSHOT300, false)
DShotRMT motor();
```


---

## 🎯 DShot Commands (experimental)

| Command | Value | Description | Usage |
|---------|-------|-------------|-------|
| MOTOR_STOP | 0 | Stop motor | Always available |
| BEACON 1 - 5 | 1 - 5 | Motor beeping | Motor identification |
| ESC_INFO | 6 | Request ESC info | Get ESC version/settings |
| SPIN_DIRECTION_1/2 | 7 - 8 | Set spin direction | Motor configuration |
| 3D_MODE_OFF/ON | 9 - 10 | 3D mode control | Bidirectional flight |
| SAVE_SETTINGS | 12 | Save to EEPROM | Permanent configuration |
| EXTENDED_TELEMETRY_ENABLE/DISABLE | 13 - 14 | Telemetry control | Data transmission |
| SPIN_DIRECTION_NORMAL/REVERSED | 20 - 21 | Spin direction | Alias commands |
| LED 0-3_ON/OFF | 22 - 29 | LED control | BLHeli32 only |
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
- **DMA Support:** Efficient, automatic memory-to-peripheral transfers

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
