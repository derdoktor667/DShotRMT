## DShot ESP32 Library utilizing RMT

## What is this branch?
This branch is where I'm squashing bugs and implementing new features without potentially breaking the main release.

Known problems so far:
-	Reception reliability is around 60%, which is far too low
-	rmt_rx.c line 505, there's a bug with the RMT version I used that made an incorrect assertion:
	`assert(offset > rx_chan->mem_off);` should be `assert(offset >= rx_chan->mem_off);`
-	Starting a back transmit before the reception has been read results in an overflow because it is hearing its own voice... I think...
	This one actually doesn't happen all the time, so I don't know its cause




## Debugging error rates
pin 23 has a success rate of 37.5 % (in spot 1)
pin 18 has a success rate of 63.2 % (in spot 2)

(adding these averages together makes around 100%?) Is this a coincidence?

when switching these spots, the success rates for each pin stayed the same.
Even though the RMT for one pin was initialized before the other, there must be some set precidence for the backend

23 in spot 2 had a lower success than 18 in spot 1
spot 1 had a success of 60.8 % (18)
spot 2 had a success of 46.2 % (23)

this doesn't add as evenly into 100%...

just spot 1 with pin 23:
success of 46.9 % (considered close enough to the experiment above)

just spot 1 with pin 18:
success of 60.4 % (close enough to the experiment above)


The betaflight controller sends dshot 600 commands every 2 milliseconds
It also sends dshot 300 commands every 2 milliseconds as well.
This is also not a hard and fast 2ms. It varies based on processor load
The frequency of commands is independent of speed. The controller will perform all important operations first, then send the latest dshot command as the scheduler decides.


occasionally, we get this error:
`E rmt: hw buffer too small, received symbols truncated`
This is followed immediately by a reception error of 2 (no packet in queue, rx_data.num_symbols = 1 or 0?)



---

### What is this fork?
This fork updates the existing ESP-RMT library made by derdoktor667 to use the newer espidf RMT libraries. It adds **working Bidirectional Dshot Support** to the library.


### The DShot Protocol
The DSHOT protocol consists of transmitting 16-bit packets to the ESCs: 11-bit throttle value,  1-bit to request telemetry and a 4-bit checksum. There are three major protocol speeds: DSHOT150, DSHOT300 and DSHOT600.

| DSHOT | Bitrate   | TH1   | TH0    | Bit Time µs | Frame Time µs |
|-------|------------|-------|--------|------------|---------------|
| 150   | 150kbit/s  | 5.00  | 2.50   | 6.67       | 106.72        |
| 300   | 300kbit/s  | 2.50  | 1.25   | 3.33       | 53.28         |
| 600   | 600kbit/s  | 1.25  | 0.625  | 1.67       | 26.72         |
| 1200  | 1200kbit/s | 0.625 | 0.313  | 0.83       | 13.28         |

#### Calculating the CRC
The checksum is calculated over the throttle value and the telemetry bit, so the “first” 12 bits our value in the following example:

    crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;

### Bidirectional DSHOT
Bidirectional DSHOT is also known as inverted DSHOT, because the signal level is inverted, so 1 is low and a 0 is high. This is done in order to let the ESC know, that we are operating in bidirectional mode and that it should be sending back eRPM telemetry packages.

#### Calculating the Bidirectional CRC
The calculation of the checksum is basically the same, just before the last step the values are inverted:

    crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;

### Using RMT on ESP32
The RMT (Remote Control) is a peripheral designed to generate accurate and stable signals to control external devices such as LEDs, motors, and other peripherals. It is well suited for generating the DShot signals in a high-performance and accurate way on the ESP32 platform.

#### Advantages of using RMT
- Generates accurate signals
- Supports programmable timing
- Configurable number of channels

#### DShot RMT Library for ESP32
The DShot RMT Library for ESP32 provides a convenient way of generating DShot signals using the RMT peripheral on the ESP32 platform. The library supports all three major DShot speeds: DSHOT150, DSHOT300, and DSHOT600.

#### References
- [DSHOT - the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [DSHOT in the Dark](https://dmrlawson.co.uk/index.php/2017/12/04/dshot-in-the-dark/)
- [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
