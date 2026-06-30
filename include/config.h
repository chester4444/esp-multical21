#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Arduino.h>
#include <vector>

  #define DEBUG 1
  #define ESP_NAME "esp-multical21"   // for dns resolving
  #define MQTT_PREFIX "watermeter" 
  #define MQTT_total "total"
  #define MQTT_target "target"
  #define MQTT_ftemp "flowtemp"
  #define MQTT_atemp "ambienttemp"
  #define MQTT_info "infocode"

  // ask your water supplier for your personal encryption key 
  // serial number is printed on your multical21
  constexpr char const* DRINKING_WATER_ID =  "74350917";
  constexpr char const* DRINKING_WATER_KEY = "FED0806793A0FAF6277414D1EDF6E8C8";
  constexpr char const* UTILITY_WATER_ID1 =  "74350765";
  constexpr char const* UTILITY_WATER_KEY1 = "84E96ABC017069A2A0A0F43319FD8D13";
  constexpr char const* UTILITY_WATER_ID2 =  "74027621";
  constexpr char const* UTILITY_WATER_KEY2 = "C05137278441B0A4F08FC891F6D2B1B5";
  // WIFI configuration, supports more than one WIFI, first found first served
  // if you dont use MQTT, leave broker/user/pass empty ("")
  // if you dont need user/pass for MQTT, leave it empty ("")
  struct WIFI_CREDENTIAL {
      char const* ssid;           // Wifi ssid
      char const* password;	      // Wifi password
      char const* mqtt_broker;		// MQTT broker ip address
      char const* mqtt_username;	// MQTT username
      char const* mqtt_password;	// MQTT password
  };

  //    "ssid", "wifi_passphrase", "mqtt_broker", "mqtt_username", "mqtt_password"
  std::vector<WIFI_CREDENTIAL> const wifiCredentials = {
      { "A1-42EB53", "EF1E1AF667", "10.0.0.111", "", ""}
    , { "IoT", "iot2019iot", "10.14.0.1", "", ""}
  };

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
// CSN   => 05
// MOSI  => 23
// MISO  => 19
// SCK   => 18
// GD0   => 32  any valid interrupt pin for your platform will do
// GD2   => not connected 
  #define CC1101_GDO0          32
  #define PIN_LED_BUILTIN      2
#endif
#endif // __CONFIG_H__