/*
 Copyright (C) 2020 chester4444@wolke7.net
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266mDNS.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <ESPmDNS.h>
#endif
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "CC1101Wmbus.h"
#include "WaterMeter.h"
//#include "config.h"

WIFI_CREDENTIAL currentWifi; // global to store found wifi

uint8_t wifiConnectCounter = 0; // count retries

WiFiClient espMqttClient;
PubSubClient mqttClient(espMqttClient);
CC1101Wmbus cc1101Wmbus;
WaterMeter waterMeter(mqttClient);

char MyIp[16];
int cred = -1;
bool mqttEnabled = false;   // true, if a broker is given in credentials.h

void radioCallback(uint8_t *payload)
{
  uint length = payload[0]; // length is at index 0
  if (length == 0)
  {
    Serial.println("Received empty frame");
    return;
  }

  waterMeter.processFrame(payload);
}

bool ConnectWifi(void)
{
  int i = 0;
  bool isWifiValid = false;

  Serial.println("starting scan");
  // scan for nearby networks:
  int numSsid = WiFi.scanNetworks();

  Serial.print("scanning WIFI, found ");
  Serial.print(numSsid);
  Serial.println(" available access points:");

  if (numSsid == -1)
  {
    Serial.println("Couldn't get a wifi connection");
    return false;
  }

  for (int i = 0; i < numSsid; i++)
  {
    Serial.print(i + 1);
    Serial.print(". ");
    Serial.print(WiFi.SSID(i));
    Serial.print("  ");
    Serial.println(WiFi.RSSI(i));
  }

  // search for given credentials
  for (WIFI_CREDENTIAL credential : wifiCredentials)
  {
    for (int j = 0; j < numSsid; ++j)
    {
      if (strcmp(WiFi.SSID(j).c_str(), credential.ssid) == 0)
      {
        Serial.print("credentials found for: ");
        Serial.println(credential.ssid);
        currentWifi = credential;
        isWifiValid = true;
      }
    }
  }

  if (!isWifiValid)
  {
    Serial.println("no matching credentials");
    return false;
  }

  // try to connect
  Serial.println(WiFi.macAddress());

  // try to connect WPA
  WiFi.hostname(ESP_NAME);
  WiFi.begin(currentWifi.ssid, currentWifi.password);
  Serial.println("");
  Serial.print("Connecting to WiFi ");
  Serial.println(currentWifi.ssid);

  i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(PIN_LED_BUILTIN, LOW);
    delay(300);
    Serial.print(F("."));
    digitalWrite(PIN_LED_BUILTIN, HIGH);
    delay(300);
    if (i++ > 50)
    {
      // giving up
      ESP.restart();
      return false;   // gcc shut up
    }
  }

  return true;
}

void mqttDebug(const char* debug_str)
{
    String s=MQTT_PREFIX"/debug";
    mqttClient.publish(s.c_str(), debug_str);
}

void mqttCallback(char* topic, byte* payload, unsigned int len)
{
  // create a local copies of topic and payload
  // PubSubClient overwrites it internally
  String t(topic);
  byte *p = new byte[len];
  memcpy(p, payload, len);
  
/*  Serial.print("MQTT-RECV: ");
  Serial.print(topic);
  Serial.print(" ");
  Serial.println((char)payload[0]); // FIXME LEN
*/
  if (strstr(topic, "smarthomeNG/start"))
  {
    if (len == 4) // True
    {
      // maybe to something
    }
  }
  else if (strstr(topic, MQTT_PREFIX "/reset"))
  {
    if (len == 4) // True
    {
      // maybe to something
      // reboot
      ESP.restart();
    }
  }
  // and of course, free it
  delete[] p;
}

bool mqttConnect()
{
  bool connected=false;

  Serial.print("try to connect to MQTT server ");
  Serial.println(currentWifi.mqtt_broker);

  // use given MQTT broker
  mqttClient.setServer(currentWifi.mqtt_broker, 1883);
    
  // connect client with retainable last will message
  if (strlen(currentWifi.mqtt_username) && strlen(currentWifi.mqtt_password))
  {
    Serial.print("with user: ");
    Serial.println(currentWifi.mqtt_username);
    // connect with user/pass
    connected = mqttClient.connect( ESP_NAME
                                  , currentWifi.mqtt_username
                                  , currentWifi.mqtt_password
                                  , MQTT_PREFIX"/online"
                                  , 0
                                  , true
                                  , "False"
                                  );
  }
  else
  {
    // connect without user/pass
    connected = mqttClient.connect(ESP_NAME, MQTT_PREFIX"/online", 0, true, "False");
  }

  mqttClient.setCallback(mqttCallback);

  return connected;
}

void mqttSubscribe()
{
  // publish online status
  mqttClient.publish(MQTT_PREFIX "/online", "True", true);
//  Serial.print("MQTT-SEND: ");
//  Serial.print(s);
//  Serial.println(" True");
  
  // publish ip address
  IPAddress MyIP = WiFi.localIP();
  snprintf(MyIp, 16, "%d.%d.%d.%d", MyIP[0], MyIP[1], MyIP[2], MyIP[3]);
  mqttClient.publish(MQTT_PREFIX"/ipaddr", MyIp, true);
//  Serial.print("MQTT-SEND: ");
//  Serial.print(s);
//  Serial.print(" ");
//  Serial.println(MyIp);

  // if True -> perform an reset
  mqttClient.subscribe(MQTT_PREFIX"/reset");
}

void setupOTA()
{
    // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(ESP_NAME);

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void setup()
{
    pinMode(PIN_LED_BUILTIN, OUTPUT);

    Serial.begin(115200);

    Serial.println();
    Serial.println("Starting up... " ESP_NAME);

    uint8_t key[16];
    uint8_t id[4];
    hex2bin(DRINKING_WATER_KEY, sizeof(key), key);
    hex2bin(DRINKING_WATER_ID, sizeof(id), id);
    waterMeter.registerMeter(0, key, id);
    hex2bin(UTILITY_WATER_KEY1, sizeof(key), key);
    hex2bin(UTILITY_WATER_ID1, sizeof(id), id);
    waterMeter.registerMeter(1, key, id);
    hex2bin(UTILITY_WATER_KEY2, sizeof(key), key);
    hex2bin(UTILITY_WATER_ID2, sizeof(id), id);
    waterMeter.registerMeter(2, key, id);

    cc1101Wmbus.setCallback(radioCallback);
    if (!cc1101Wmbus.begin())
    {
      Serial.println("Failed to initialize CC1101 - check wiring");
    }
    Serial.println("Setup done...");
}

enum ControlStateType
  { StateInit
  , StateNotConnected
  , StateWifiConnect
  , StateMqttConnect
  , StateConnected
  , StateOperating
  , StateOperatingNoWifi
  };
ControlStateType ControlState = StateInit;

void loop()
{
  switch (ControlState)
  {
    case StateInit:
      //Serial.println("StateInit:");
      WiFi.mode(WIFI_STA);

      ControlState = StateNotConnected;
      break;

    case StateNotConnected:
      //Serial.println("StateNotConnected:");

      ControlState = StateWifiConnect;
      break;
      
    case StateWifiConnect:
      //Serial.println("StateWifiConnect:");
      // station mode
      if (ConnectWifi() == false)
      {
        ControlState = StateOperatingNoWifi;
        break;
      }

      delay(500);
      
      if (WiFi.status() == WL_CONNECTED)
      {
        Serial.println("");
        Serial.print("Connected to ");
        Serial.println(currentWifi.ssid);
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        setupOTA();
        
        if (strlen(currentWifi.mqtt_broker)) // MQTT is used
        {
          mqttEnabled = true;
          ControlState = StateMqttConnect;
        }
        else
        {
          // no MQTT server -> go operating
          mqttEnabled = false;
          ControlState = StateOperating;
          Serial.println(F("MQTT not enabled"));
          Serial.println(F("StateOperating:"));
        }
      }
      else
      {
        Serial.println("");
        Serial.println("Connection failed.");

        // try again
        ControlState = StateNotConnected;

        // reboot 
        ESP.restart();
      }
      break;

    case StateMqttConnect:
      Serial.println("StateMqttConnect:");
      digitalWrite(PIN_LED_BUILTIN, HIGH); // off

      waterMeter.enableMqtt(false);

      if (WiFi.status() != WL_CONNECTED)
      {
        ControlState = StateNotConnected;
        break; // exit (hopefully) switch statement
      }
      
      if (mqttEnabled)
      {
        if (mqttConnect())
        {
          ControlState = StateConnected;
          waterMeter.enableMqtt(true);
        }
        else
        {
          Serial.println("MQTT connect failed");

          delay(1000);
          // try again
        }
      }
      else
      {
        // no MQTT is used at all
        ControlState = StateConnected;
      }
      ArduinoOTA.handle();
      
      break;

    case StateConnected:
      Serial.println("StateConnected:");

      if (mqttEnabled)
      {
        if (!mqttClient.connected())
        {
          ControlState = StateMqttConnect;
          delay(1000);
        }
        else
        {
          // subscribe to given topics
          mqttSubscribe();
          
          ControlState = StateOperating;
          digitalWrite(PIN_LED_BUILTIN, LOW); // on
          Serial.println("StateOperating:");
          //mqttDebug("up and running");
        }
      }
      else
      {
        ControlState = StateOperating;
      }
      ArduinoOTA.handle();
      
      break;
    
    case StateOperating:
      //Serial.println("StateOperating:");

      if (WiFi.status() != WL_CONNECTED)
      {
        ControlState = StateWifiConnect;
        break; // exit (hopefully switch statement)
      }

      if (mqttEnabled)
      {
        if (!mqttClient.connected())
        {
          Serial.println("not connected to MQTT server");
          ControlState = StateMqttConnect;
        }

        mqttClient.loop();
      }

      // here we go
      cc1101Wmbus.loop();

      ArduinoOTA.handle();

      break;

    case StateOperatingNoWifi:

      cc1101Wmbus.loop();
      break;

    default:
      Serial.println("Error: invalid ControlState");  
  }
}