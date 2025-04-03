/*
 * Name:        DShotRMT.h
 * Created:     20.03.2021 00:49:15
 * Author:      derdoktor667
 */

 #ifndef DSHOTRMT_H
 #define DSHOTRMT_H
 
 #include "Arduino.h"
 #include "driver/rmt_tx.h"
 #include "driver/rmt_rx.h"
 
 // DShot Frequenzen
 enum class DShotSpeed : uint32_t {
     DSHOT_150 = 150000,
     DSHOT_300 = 300000,
     DSHOT_600 = 600000,
     DSHOT_1200 = 1200000
 };
 
 class DShotRMT {
 public:
     DShotRMT(uint8_t pin, DShotSpeed speed);
     void send(uint16_t data);
     uint16_t receive();
     void switchToRxMode();
     void switchToTxMode();
 
 private:
     uint8_t _pin;
     DShotSpeed _speed;
     rmt_channel_handle_t _txChannel;
     rmt_channel_handle_t _rxChannel;
     void init();
 };
 
 #endif
 