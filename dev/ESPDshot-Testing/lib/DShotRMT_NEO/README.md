## DShot ESP32 Library utilizing RMT

## What is this branch?
This branch is where I'm squashing bugs and implementing new features without potentially breaking the main release.

<details>
  <summary style="font-size:100%;"><i><b>Notes for me</b></i></summary>


Known problems so far:
-	Reception reliability is around 60%, which is far too low
-	rmt_rx.c line 505, there's a bug with the RMT version I used that made an incorrect assertion:
	`assert(offset > rx_chan->mem_off);` should be `assert(offset >= rx_chan->mem_off);`
-	Starting a back transmit before the reception has been read results in an overflow because it is hearing its own voice... I think...
	This one actually doesn't happen all the time, so I don't know its cause


The program is getting good packets, but is parsing them incorrectly?

For example, this:
===============================<\r><\n>
D0: 23 L0: 1 || D1: 38 L1: 0<\r><\n>
D0: 10 L0: 1 || D1: 11 L1: 0<\r><\n>
D0: 11 L0: 1 || D1: 12 L1: 0<\r><\n>
D0: 23 L0: 1 || D1: 10 L1: 0<\r><\n>
D0: 38 L0: 1 || D1: 24 L1: 0<\r><\n>
D0: 24 L0: 1 || D1: 0 L1: 0<\r><\n>
===============================<\r><\n>
was reported as a bad packet, even though running it through the code on a desktop PC resulted in a good packet!

This one duplicated the final frame
===============================<\r><\n>
D0: 24 L0: 1 || D1: 37 L1: 0<\r><\n>
D0: 24 L0: 1 || D1: 10 L1: 0<\r><\n>
D0: 24 L0: 1 || D1: 24 L1: 0<\r><\n>
D0: 23 L0: 1 || D1: 24 L1: 0<\r><\n>
D0: 11 L0: 1 || D1: 23 L1: 0<\r><\n>
D0: 10 L0: 1 || D1: 0 L1: 0<\r><\n>
D0: 10 L0: 1 || D1: 0 L1: 0<\r><\n> //this one caused error
===============================<\r><\n>


potential problem note:
xQueue ISR data passer sends a pointer to the data
the data that the pointer is refrencing can change with another RX event

since we know the max size that a recieved frame should be, we could probably copy the whole thing into the queue instead of just a pointer to the data. That way, if things change midway, we don't get weird OOB memory problems

the response packet is 21 bits long
each bit is either HIGH or LOW
an rmt_symbol_word_t contains a HIGH and LOW part (so it contains 2 bits)
21/2 = 10 + r=1
11 is the max number of symbols we would need
each symbol is 32 bits long (unsigned int)
11 ints is the size of the data


note: there is a 30 microsecond space minimum between dshot TX and RX


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


Changing the queue type to use a statically allocated array of rmt symbols has increased the success rate to 99% for all channels (removed channel "bias"). 
I still have problems with too frequent reads though. I think the problem is that when I go to read, I get inturrupted by an rx event, so the data I get in gets cut off.

Is the RX event cutting off the read event, or is the read event cutting off the RX event?

Noticed that whenever a serialPrint operation is sent, the next packet we get back is shorter than it should be.

</details>

---

### What is this fork?
This fork updates the existing ESP-RMT library made by derdoktor667 to use the newer espidf RMT libraries. It adds **working Bidirectional Dshot Support** to the library.

### Useage


To send out motor signals, first initialize a DShotRMT object with the pin number the ESC will be attached to.

Most pins can be used with the RMT module. [This thread](https://esp32.com/viewtopic.php?t=26659) outlines the perepheral pretty well.
Note that [other pin restrictions](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/) still apply.

```
DShotRMT anESC(MOTOR01_PIN);
```
Once the object has been created, install the RMT settings with the following:
```
anESC.begin(DSHOT_MODE, ENABLE_BIDIRECTION, 14);
```
Look at `DShotRMT.h` for a description of the arguments and avalable enums that can be used to initialize the object.

To send packets to to the ESC once it's been initialized, use `send_dshot_value()`:
```
anESC.send_dshot_value(INITIAL_THROTTLE);
```
This function takes a 16 bit unsigned integer from `0` to `DSHOT_THROTTLE_MAX`.
Values below `DSHOT_THROTTLE_MIN` correspond to special commands for the ESC, which enable things like extended telemetry, 3D mode, or motor beeping.
See `DShotRMT.h` for a complete list of commands.

*Note: To arm the ESC, a value of `INITIAL_THROTTLE` or `DSHOT_CMD_MOTOR_STOP` must be sent for a specified ammount of time. It varies from firmware to firmware. For example, with BlueJay, it is 300ms.*

The library currently does **not** automate dshot value transmission, so `send_dshot_value()` must be run at least every 10 milliseconds *(Betaflight ESCs do it around every 2ms)*. It is recommended to set up a [freeRTOS task](https://www.freertos.org/taskandcr.html) to handle this process. This is preferred over an ISR because tasks are non-blocking. An ISR on a timer *may* work, but its blocking nature may result in undesirable behavior.

To get packets back from the ESC, use `get_dshot_packet()`:
```
uint16_t rpm_1 = 0;
extended_telem_type_t telem = TELEM_TYPE_ERPM; //telemetry argument is optional
int error_a = anESC.get_dshot_packet(&rpm_1, &telem);
```
This function returns an error if the last packet recieved was incorrect. It sets the value and packet type of the pointers fed into the argument section. If the packet type is known, the second argument is optional.
Again, types are defined in `DShotRMT.h`.

To see how well packets are being delivered, use `get_telem_success_rate()`.
This returns a float percentage of the successful packets decoded by `get_dshot_packet()`.


**Special note:** *Running `Serial.print` each time a dshot packet is sent and recieved plays havoc with the inturrupts, stopping reception partway, or not allowing it to start until the serial message is finished being transmitted. This results in a much lower reception success rate than otherwise. I have yet to find a workaround other than calling `Serial.print` less often, like, say, every 10 packets.*


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
Bidirectional DSHOT is also known as inverted DSHOT, because the signal level is inverted, so 1 is low and a 0 is high. This is done in order to let the ESC know that we are operating in bidirectional mode and that it should be sending back eRPM telemetry packages.

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
