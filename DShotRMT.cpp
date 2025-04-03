/*
 * Name:        DShotRMT.cpp
 * Created:     20.03.2021 00:49:15
 * Author:      derdoktor667
 */

 #include "DShotRMT.h"

 DShotRMT::DShotRMT(uint8_t pin, DShotSpeed speed) : _pin(pin), _speed(speed), _txChannel(NULL), _rxChannel(NULL) {
     init();
 }
 
 void DShotRMT::init() {
     rmt_tx_channel_config_t tx_config = {
         .gpio_num = (gpio_num_t)_pin,
         .clk_src = RMT_CLK_SRC_DEFAULT,
         .resolution_hz = static_cast<uint32_t>(_speed),
         .mem_block_symbols = 64,
         .trans_queue_depth = 4,
     };
 
     if (rmt_new_tx_channel(&tx_config, &_txChannel)) {
         Serial.println("Failed to create TX channel");
     }
 
     rmt_rx_channel_config_t rx_config = {
         .gpio_num = (gpio_num_t)_pin,
         .clk_src = RMT_CLK_SRC_DEFAULT,
         .resolution_hz = static_cast<uint32_t>(_speed),
         .mem_block_symbols = 64,
     };
 
     if (rmt_new_rx_channel(&rx_config, &_rxChannel)) {
         Serial.println("Failed to create RX channel");
     }
 }
 
 void DShotRMT::send(uint16_t data) {
     if (_txChannel == NULL) {
         Serial.println("RMT not initialized");
         return;
     }
 
     rmt_transmit_config_t tx_config = {
         .loop_count = 0,
     };
 
     if (rmt_transmit(_txChannel, NULL, &data, sizeof(data), &tx_config)) {
         Serial.println("Failed to transmit data");
     }
 
     if (rmt_tx_wait_all_done(_txChannel, portMAX_DELAY)) {
         Serial.println("Failed to wait for TX completion");
     }
 }
 
 uint16_t DShotRMT::receive() {
     if (_rxChannel == NULL) {
         Serial.println("RMT RX not initialized");
         return 0;
     }
 
     rmt_receive_config_t rx_config = {
         .signal_range_min_ns = 100,
         .signal_range_max_ns = 10000,
     };
 
     uint16_t received_data = 0;
     if (rmt_receive(_rxChannel, &received_data, sizeof(received_data), &rx_config)) {
         Serial.println("Failed to receive data");
         return 0;
     }
 
     uint16_t rpm = (received_data & 0x07FF) * 100;
     
     #ifdef DEBUG_DSHOT
     Serial.print("Received RPM: ");
     Serial.println(rpm);
     #endif
 
     return rpm;
 }
 
 void DShotRMT::switchToRxMode() {
     if (_rxChannel == NULL) {
         Serial.println("RX channel not initialized");
         return;
     }
     // Code zur Umschaltung auf RX-Modus
 }
 
 void DShotRMT::switchToTxMode() {
     if (_txChannel == NULL) {
         Serial.println("TX channel not initialized");
         return;
     }
     // Code zur Umschaltung auf TX-Modus
 }
 