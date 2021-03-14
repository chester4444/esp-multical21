#ifndef __HWCONFIG_H__
#define __HWCONFIG_H__

#if defined(ESP8266)
// Attach CC1101 pins to ESP8266 SPI pins
// VCC   => 3V3
// GND   => GND
// CSN   => D8
// MOSI  => D7
// MISO  => D6
// SCK   => D5
// GD0   => D2  A valid interrupt pin for your platform (defined below this)
// GD2   => not connected 
  #define CC1101_GDO0         D2   // GDO0 input interrupt pin
  #define PIN_LED_BUILTIN     D4
#elif defined(ESP32)
// Attach CC1101 pins to ESP32 SPI pins
// VCC   => 3V3
// GND   => GND
// CSN   => 4
// MOSI  => 23
// MISO  => 19
// SCK   => 18
// GD0   => 32  any valid interrupt pin for your platform will do
// GD2   => not connected 

// attach CC1101 pins to ESP32 SPI pins

  #define CC1101_GDO0          32
  #define PIN_LED_BUILTIN      2
#endif

#endif //__HWCONFIG_H__