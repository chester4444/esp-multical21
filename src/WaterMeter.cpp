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

#include "WaterMeter.h"

WaterMeter::WaterMeter(PubSubClient &mqtt)
  : mqttClient  (mqtt)
  , mqttEnabled (false)
{
}

void WaterMeter::enableMqtt(bool enabled)
{
  mqttEnabled = enabled;
}

void WaterMeter::begin()
{
  radio.setCallback([this](uint8_t *payload) {
    processFrame(payload);
  });

  if (!radio.begin())
  {
    Serial.println("Failed to initialize CC1101 - check wiring");
  }
  Serial.println("CC1101 initialized successfully");
}

void WaterMeter::registerMeter(uint8_t idx, uint8_t *key, uint8_t *id)
{
  if (idx > MAX_METERS)
  {
    Serial.printf("Error: Meter index %d exceeds maximum of %d\n", idx, MAX_METERS);
    return;
  }
  // FIXME: check if meter with same id already exists
  nrOfMeters++;
  memcpy(&aesKey[idx][0], key, 16);
  memcpy(&meterId[idx][0], id, sizeof(meterId)/MAX_METERS);
  Serial.printf("Meter %d registered with id: ", idx);
  for (uint8_t i = 0; i < 4; i++)  {
    Serial.printf("%02x", meterId[idx][i]);
  } 
  Serial.println();
}

void WaterMeter::processFrame(uint8_t *payload)
{
#if 0
  Serial.printf("meterID: ");
  for (uint8_t i = 0; i < 4; i++)
  {
    Serial.printf("%02x", payload[7-i]);
  }
  // Serial.printf(" - %d", length);
  // Serial.println();
#endif

  // check meterId
  for (uint8_t i = 0; i < nrOfMeters; i++)
  {
    bool meterfound = true;
    for (uint8_t j = 0; j < 4; j++)
    {
      if (meterId[i][j] != payload[7 - j])
      {
        meterfound = false;
        break;
      }
    }
    if (meterfound)
    {
      // found our meter, process the frame
      Serial.printf("Frame received for meter %d\n\r", i);

      length = payload[0]; // length is at index 0
      memcpy(this->payload, payload, length);

    #if 0
      Serial.println("copied frame payload:");
      for (uint8_t i = 0; i <= length; i++)
      {
        Serial.printf("%02x", payload[i]);
      }
      Serial.println();
    #endif
      decryptFrame(i);
      getMeterInfo();
      publishMeterInfo(i);
    }
  }
}

void WaterMeter::getMeterInfo()
{
  // init positions for compact frame
  int pos_tt = 9;  // total consumption
  int pos_tg = 13; // target consumption
  int pos_ic = 7;  // info codes
  int pos_ft = 17; // flow temp
  int pos_at = 18; // ambient temp

  if (plaintext[2] == 0x78) // long frame
  {
    // overwrite it with long frame positions
    pos_tt = 10;
    pos_tg = 16;
    pos_ic = 6;
    pos_ft = 22;
    pos_at = 25;
  }

  totalWater = plaintext[pos_tt] + (plaintext[pos_tt + 1] << 8) + (plaintext[pos_tt + 2] << 16) + (plaintext[pos_tt + 3] << 24);

  targetWater = plaintext[pos_tg] + (plaintext[pos_tg + 1] << 8) + (plaintext[pos_tg + 2] << 16) + (plaintext[pos_tg + 3] << 24);

  flowTemp = plaintext[pos_ft];
  ambientTemp = plaintext[pos_at];
  infoCodes = plaintext[pos_ic];
}

void WaterMeter::publishMeterInfo(uint8_t idx)
{

  char total[12];
  snprintf(total, sizeof(total), "%d.%03d", totalWater/ 1000, totalWater % 1000);
  Serial.printf("total: %s m%c - ", total, 179);

  char target[12];
  snprintf(target, sizeof(target), "%d.%03d", targetWater / 1000, targetWater % 1000);
  Serial.printf("target: %s m%c - ", target, 179);

  char flow_temp[4];
  snprintf(flow_temp, sizeof(flow_temp), "%2d", flowTemp);
  Serial.printf("%s %cC - ", flow_temp, 176);

  char ambient_temp[4];
  snprintf(ambient_temp, sizeof(ambient_temp), "%2d", ambientTemp);
  Serial.printf("%s %cC - ", ambient_temp, 176);

  char info_codes[3];
  snprintf(info_codes, sizeof(info_codes), "%02x", infoCodes);
  Serial.printf("0x%s \n\r", info_codes);

  if (!mqttEnabled) return; // no MQTT broker connected, leave
  
  Serial.println("Publishing meter info to MQTT...");
  snprintf(topic_total, sizeof(topic_total), MQTT_PREFIX "/%d/" MQTT_total, idx);
  snprintf(topic_target, sizeof(topic_target), MQTT_PREFIX "/%d/" MQTT_target, idx);
  snprintf(topic_ftemp, sizeof(topic_ftemp), MQTT_PREFIX "/%d/" MQTT_ftemp, idx);
  snprintf(topic_atemp, sizeof(topic_atemp), MQTT_PREFIX "/%d/" MQTT_atemp, idx);
  snprintf(topic_info, sizeof(topic_info), MQTT_PREFIX "/%d/" MQTT_info, idx);

  // change the topics as you like
  mqttClient.publish(topic_total, total);
  mqttClient.publish(topic_target, target);
  mqttClient.loop();
  mqttClient.publish(topic_ftemp, flow_temp);
  mqttClient.publish(topic_atemp, ambient_temp);
  mqttClient.publish(topic_info, info_codes);
  mqttClient.loop();
}

void WaterMeter::decryptFrame(uint8_t idx)
{
  uint8_t cipherLength = length - 2 - 16; // cipher starts at index 16, remove 2 crc bytes
  memcpy(cipher, &payload[17], cipherLength);

  memset(iv, 0, sizeof(iv)); // padding with 0
  memcpy(iv, &payload[2], 8);
  iv[8] = payload[11];
  memcpy(&iv[9], &payload[13], 4);

#if DEBUG
  printHex(iv, sizeof(iv));
  printHex(cipher, cipherLength);
#endif

  aes128.setKey(aesKey[idx], 16);
  aes128.setIV(iv, sizeof(iv));
  aes128.decrypt(plaintext, (const uint8_t *)cipher, cipherLength);

  /*
  Serial.printf("C:     ");
  for (size_t i = 0; i < cipherLength; i++)
  {
    Serial.printf("%02X", cipher[i]);
  }
  */
  Serial.println();
  Serial.printf("P(%d): ", cipherLength);
  for (size_t i = 0; i < cipherLength; i++)
  {
    Serial.printf("%02X", plaintext[i]);
  }
  Serial.println();

  // received packet is ok
}

void WaterMeter::loop()
{
  radio.loop();
}