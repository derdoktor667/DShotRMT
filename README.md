## DShot ESP32 Library utilizing RMT




### What is this fork?
This fork updates the existing ESP-RMT library made by derdoktor667 to use the newer espidf RMT libraries. It adds **working Bidirectional Dshot Support** to the library.


### Useage




To send out motor signals, first initialize a DShotRMT object with the pin number the ESC will be attached to.


Most pins can be used with the RMT module. [This thread](https://esp32.com/viewtopic.php?t=26659) outlines the peripheral pretty well.
Note that [other pin restrictions](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/) still apply.


```
DShotRMT anESC(MOTOR01_PIN);
```
Once the object has been created, install the RMT settings with the following:
```
anESC.begin(DSHOT_MODE, ENABLE_BIDIRECTION, 14);
```
Look at `DShotRMT.h` for a description of the arguments and available enums that can be used to initialize the object.


To send packets to to the ESC once it's been initialized, use `send_dshot_value()`:
```
anESC.send_dshot_value(INITIAL_THROTTLE);
```
This function takes a 16 bit unsigned integer from `0` to `DSHOT_THROTTLE_MAX`.
Values below `DSHOT_THROTTLE_MIN` correspond to special commands for the ESC, which enable things like extended telemetry, 3D mode, or motor beeping.
See `DShotRMT.h` for a complete list of commands.


*Note: To arm the ESC, a value of `INITIAL_THROTTLE` or `DSHOT_CMD_MOTOR_STOP` must be sent for a specified amount of time. It varies from firmware to firmware. For example, with BlueJay, it is 300ms.*


The library currently does **not** automate dshot value transmission, so `send_dshot_value()` must be run at least every 10 milliseconds *(Betaflight ESCs do it around every 2ms)*. It is recommended to set up a [freeRTOS task](https://www.freertos.org/taskandcr.html) to handle this process. This is preferred over an ISR because tasks are non-blocking. An ISR on a timer *may* work, but its blocking nature may result in undesirable behavior.


To get packets back from the ESC, use `get_dshot_packet()`:
```
uint16_t rpm_1 = 0;
extended_telem_type_t telem = TELEM_TYPE_ERPM; //telemetry argument is optional
int error_a = anESC.get_dshot_packet(&rpm_1, &telem);
```
This function returns an error if the last packet received was incorrect. It sets the value and packet type of the pointers fed into the argument section. If the packet type is known, the second argument is optional.
Again, types are defined in `DShotRMT.h`.


To see how well packets are being delivered, use `get_telem_success_rate()`.
This returns a float percentage of the successful packets decoded by `get_dshot_packet()`.




**Special note:** *Running `Serial.print` each time a dshot packet is sent and received plays havoc with the interrupts, stopping reception partway, or not allowing it to start until the serial message is finished being transmitted. This results in a much lower reception success rate than otherwise. I have yet to find a workaround other than calling `Serial.print` less often, like, say, every 10 packets.*




Please see the dev folder sub-project for a more complete example of the library.


---


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
Bidirectional DSHOT is also known as inverted DSHOT, because the signal level is inverted, so 1 is low and 0 is high. This is done in order to let the ESC know that we are operating in bidirectional mode and that it should be sending back eRPM telemetry packages.


#### Calculating the Bidirectional CRC
The calculation of the checksum is basically the same, just before the last step the values are inverted:


    crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;


### Using RMT on ESP32
[The RMT (Remote Control)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html) is a peripheral designed to generate accurate and stable signals to control external devices such as LEDs, motors, and other peripherals. It is well suited for generating the DShot signals in a high-performance and accurate way on the ESP32 platform.


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



