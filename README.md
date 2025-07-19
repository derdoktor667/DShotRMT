[![Arduino CI](https://github.com/derdoktor667/DShotRMT/actions/workflows/esp32.yml/badge.svg?event=push)](https://github.com/derdoktor667/DShotRMT/actions/workflows/esp32.yml)

# DShotRMT - ESP32 Library (Rewrite for ESP-IDF 5)

A modern, robust C++ library for generating DShot signals on the ESP32 using the new ESP-IDF 5 RMT encoder API (`rmt_tx.h` / `rmt_rx.h`).  
Supports all standard DShot modes (150, 300, 600) and features continuous frame transmission with configurable pause.  
**Now with BiDirectional DShot support!**

> The legacy version (using the old `rmt.h` API) is still available in the `oldAPI` branch.

---

## 🚀 Features

- **All DShot Modes:** DSHOT150, DSHOT300 (default), DSHOT600
- **BiDirectional DShot:** Experimental support for telemetry and RPM feedback
- **Continuous Frames:** Hardware-timed, CPU-independent signal generation
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

```cpp
#include <DShotRMT.h>

constexpr gpio_num_t MOTOR_PIN = GPIO_NUM_17;
constexpr dshot_mode_t MODE = DSHOT300;
constexpr bool BIDIRECTIONAL = true;

DShotRMT motor(MOTOR_PIN, MODE, BIDIRECTIONAL);

void setup() {
    Serial.begin(115200);
    motor.begin();
    motor.setThrottle(1000); // Set throttle value (48–2047)
}

void loop() {
    // Optionally read RPM if bidirectional mode is enabled
    uint32_t rpm = motor.getMotorRPM(14); // 14 magnets
    Serial.println(rpm);
}
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
- Hardware-timed pulses  
- CPU-independent  
- Loop mode with inter-frame pause  
- Reliable under system load

---

## 📝 API Reference

- `DShotRMT(gpio_num_t gpio, dshot_mode_t mode, bool isBidirectional, uint8_t pauseDuration = 120)`
- `void begin()`
- `void setThrottle(uint16_t throttle)`
- `uint32_t getERPM()`
- `uint32_t getMotorRPM(uint8_t magnet_count)`

See [examples/dshot300/dshot300.ino](examples/dshot300/dshot300.ino) for a full demo.

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
