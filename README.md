[![Arduino CI](https://github.com/derdoktor667/DShotRMT/actions/workflows/esp32.yml/badge.svg?event=push)](https://github.com/derdoktor667/DShotRMT/actions/workflows/esp32.yml)

## DShotRMT - ESP32 Library (Rewrite for ESP-IDF 5)

This is a complete rewrite of the original DShotRMT library to support the new ESP-IDF 5 RMT encoder API (`rmt_tx.h`).  
The library sends continuous DShot frames with a configurable pause between them and supports all standard DShot modes (150, 300, 600).

### Now with BiDirectional DShot Support!!!

The old Version without encoding (rmt.h) is still available by using "oldAPI" Branch.

---

## The DShot Protocol

The DShot protocol transmits 16-bit packets to brushless ESCs:

- 11-bit throttle value
- 1-bit telemetry request
- 4-bit checksum

Data is transmitted MSB-first. Pulse timing depends on the selected DShot mode.

| DSHOT | Bitrate     | TH1   | TH0    | Bit Time (µs) | Frame Time (µs) |
|-------|-------------|-------|--------|---------------|-----------------|
| 150   | 150 kbit/s  | 5.00  | 2.50   | 6.67          | ~106.72         |
| 300   | 300 kbit/s  | 2.50  | 1.25   | 3.33          | ~53.28          |
| 600   | 600 kbit/s  | 1.25  | 0.625  | 1.67          | ~26.72          |

Each frame is followed by a 21-bit time pause. This helps ESCs detect separate frames.

---

## Checksum Calculation

The checksum is calculated over the first 12 bits (throttle + bit):

```c
crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
```

### Bidirectional DSHOT
Bidirectional DSHOT is also known as inverted DSHOT, because the signal level is inverted, so 1 is low and a 0 is high. This is done in order to let the ESC know, that we are operating in bidirectional mode and that it should be sending back telemetry packages.

#### Calculating the Bidirectional CRC
The calculation of the checksum is basically the same as before, but the inverted:

```c
crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
```

---

## RMT on the ESP32

The RMT (Remote Control) is a peripheral designed to generate accurate and stable signals to control external devices such as LEDs, motors, and other peripherals. It is well suited for generating the DShot signals in a high-performance and accurate way on the ESP32 platform.

### Advantages:

- Hardware-timed pulses  
- CPU-independent signal generation  
- Loop mode with inter-frame pause  
- Reliable under system load

---

## About This Library

This C++ library provides a simple class to generate DShot signals using any RMT-capable GPIO.  
It uses a `copy_encoder` to continuously send a prebuilt symbol buffer. New throttle values are applied only when they change.

### Supported Modes (optional BiDirectional):

- DSHOT150  
- DSHOT300 (default)  
- DSHOT600

### Frame Structure:

- 16-bit DShot data  
- 21-bit times worth of pause

---

## References

- [DSHOT – the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)  
- [DSHOT in the Dark](https://dmrlawson.co.uk/index.php/2017/12/04/dshot-in-the-dark/)  
- [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)

---

## License

MIT License – see LICENSE

---

## Author

Wastl Kraus  
GitHub: [@derdoktor667](https://github.com/derdoktor667)  
Website: [wir-sind-die-matrix.de](https://wir-sind-die-matrix.de)
