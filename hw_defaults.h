/**
 * @file hw_defaults.h
 * @brief Some Shortcuts
 * @author Wastl Kraus
 * @date 2025-08-02
 * @license MIT
 */

#ifdef ESP32
// USB-Serial (UART0): GPIO1/3 in use by default

// Serial1
constexpr auto PIN_UART1_TX = GPIO_NUM_13;
constexpr auto PIN_UART1_RX = GPIO_NUM_14;

// Serial2
constexpr auto PIN_UART2_TX = GPIO_NUM_16;
constexpr auto PIN_UART2_RX = GPIO_NUM_17;

// SPI (VSPI)
constexpr auto PIN_SPI_SCLK = GPIO_NUM_18;
constexpr auto PIN_SPI_MISO = GPIO_NUM_19;
constexpr auto PIN_SPI_MOSI = GPIO_NUM_23;
constexpr auto PIN_SPI_SS = GPIO_NUM_5;

// I2C (Wire)
constexpr auto PIN_I2C_SDA = GPIO_NUM_21;
constexpr auto PIN_I2C_SCL = GPIO_NUM_22;

// RMT (5 Channels)
constexpr auto PIN_RMT_CH0 = GPIO_NUM_25;
constexpr auto PIN_RMT_CH1 = GPIO_NUM_26;
constexpr auto PIN_RMT_CH2 = GPIO_NUM_27;
constexpr auto PIN_RMT_CH3 = GPIO_NUM_32;
constexpr auto PIN_RMT_CH4 = GPIO_NUM_33;
#endif // ESP32
