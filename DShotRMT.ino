// ...some very simple DShot example generating a DShot300 signal.

#include <Arduino.h>
#include "src/DShotRMT.h"

DShotRMT dshot_01(GPIO_NUM_4, RMT_CHANNEL_0);

volatile uint16_t throttle_value = 48; // 48 is lowest throttle value
String serialBuffer;

void setup() {
  // ...always start the onboard usb support
  Serial.begin(115200);
  Serial.setTimeout(100);

  // ...start the dshot generation
  dshot_01.begin(DSHOT300, false);  // speed & bidirectional
}

void loop() {
  while (Serial.available() > 0) {
    char recieved = Serial.read();
    if (recieved == '\n') {
      Serial.println("new serial value");
      throttle_value = serialBuffer.toInt();
      serialBuffer = "";
      Serial.println(throttle_value);
    } else {
      serialBuffer += recieved;
    }
  }

  dshot_01.send_dshot_value(throttle_value, NO_TELEMETRIC);

  // ...print to console
//  Serial.print(throttle_value);
  //  Serial.print(" ");
  //  Serial.print((dshot_01.dshot_tx_rmt_item);
//  Serial.println("");
  delay(1);
}
