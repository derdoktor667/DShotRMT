#include <Arduino.h>

//TEST
#include <hal/gpio_hal.h>

#include <DShotRMT.h>

// USB serial port needed for this example
const int USB_SERIAL_BAUD = 115200;
#define USB_Serial Serial

// Define the GPIO pin connected to the motor and the DShot protocol used
const auto MOTOR01_PIN = 23;
const auto DSHOT_MODE = DSHOT300;

// Define the failsafe and initial throttle values
const auto FAILSAFE_THROTTLE = 999;
const auto INITIAL_THROTTLE = 48;


DShotRMT anESC(23);

void setup()
{
  USB_Serial.begin(USB_SERIAL_BAUD);

  if(anESC.begin(DSHOT300, true))
    Serial.println("init error");
}

int loopCount = 0;
void loop()
{
  if(loopCount < 700)
  {
    anESC.send_dshot_value(INITIAL_THROTTLE);
  }
  else
  {

    anESC.send_dshot_value(INITIAL_THROTTLE);
  }

  if(loopCount % 100 == 0)
    anESC.get_dshot_RPM();

  delay(10);
  ++loopCount;



}