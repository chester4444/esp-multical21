#ifndef __CREDENTIALS_H__
#define __CREDENTIALS_H__

#include <vector>

// ask your supplier for your personal encryption key (16 bytes)
#define ENCRYPTION_KEY      0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
// serial number is printed on your multical21
#define SERIAL_NUMBER       0x12, 0x34, 0x56, 0x78

// WIFI configuration, supports more than one WIFI, first found first served
// if you dont use MQTT, leave it empty ("")
// if you dont need user/pass for MQTT, leave it empty ("")
struct CREDENTIAL {
    char const* ssid;
    char const* password;	
    char const* mqtt_broker;		// MQTT broker
    char const* mqtt_username;	// MQTT username
    char const* mqtt_password;	// MQTT password
};

CREDENTIAL currentWifi; // global to store found wifi

//    "ssid", "wifi_passphrase", "mqtt_broker", "mqtt_username", "mqtt_password"
std::vector<CREDENTIAL> const credentials = {
     { "ssid1", "********", "", "", ""}     // no MQTT - just serial output
   , { "ssid3", "********", "10.0.0.11", "", ""}  // MQTT without authentication
   , { "ssid2", "********", "10.0.0.10", "loxone", "loxy1234"}  // MQTT with user/pass
};

#endif // __CREDENTIALS_H__