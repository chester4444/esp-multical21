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

#ifndef _WATERMETER_H_
#define _WATERMETER_H_

#include <Arduino.h>
#include <Crypto.h>
#include <AES.h>
#include <CTR.h>
#include <PubSubClient.h>
#include "CC1101Wmbus.h"
#include "config.h"
#include "utils.h"

class WaterMeter
{
  private:
    static constexpr uint8_t MAX_METERS = 5; // maximum number of meters that can be registered
    static const uint8_t FRAME_MAX_LENGTH = 64; // max length of receiveed frame
    static const uint8_t TOPIC_MAX_LEN = 32; // max length of MQTT topic
    uint8_t nrOfMeters = 0; // number of registered meters
    uint8_t meterId[MAX_METERS][4];
    uint8_t aesKey[MAX_METERS][16];
    CTR<AESSmall128> aes128;
    uint8_t cipher[FRAME_MAX_LENGTH];
    uint8_t plaintext[FRAME_MAX_LENGTH];
    uint8_t iv[16];
    bool isValid = false; // true, if meter information is valid for the last received frame
    uint8_t length = 0; // payload length
    uint8_t payload[FRAME_MAX_LENGTH]; // payload data
    uint32_t totalWater;
    uint32_t targetWater;
    uint32_t lastTarget=0;
    uint8_t flowTemp;
    uint8_t ambientTemp;
    uint8_t infoCodes;
    char topic_total[TOPIC_MAX_LEN];
    char topic_target[TOPIC_MAX_LEN];
    char topic_ftemp[TOPIC_MAX_LEN];
    char topic_atemp[TOPIC_MAX_LEN];
    char topic_info[TOPIC_MAX_LEN];

    CC1101Wmbus radio;
    PubSubClient &mqttClient;
    bool mqttEnabled;

    // check if frame is for this meter
    void processFrame(uint8_t *payload);

    void decryptFrame(uint8_t idx);
    void getMeterInfo();
    void publishMeterInfo(uint8_t idx);
    
  public:

    // constructor
    WaterMeter(PubSubClient &mqtt);
    
    void enableMqtt(bool enable);

    // configure AES key and meter id
    void begin();
    void registerMeter(uint8_t index, uint8_t *key, uint8_t *id);
    void loop();

};

#endif // _WATERMETER_H_

/* wmbus.cc tag 1.7.0
     else if (format_signature == 0xd2f7)
    {
        hex2bin("02FF2004134413615B5167", format_bytes);
        debug("(wmbus) using hard coded format for hash d2f7\n");

long frame 
8CEA7802FF2071000413BD7D0E004413BD7D0E00615B7F516714
6C4C79F7D2B1307100BD7D0E00BD7D0E007F14

}
    else if (format_signature == 0xdd34)
    {
        hex2bin("02FF2004134413", format_bytes);
        debug("(wmbus) using hard coded format for hash dd34\n");
    }

long frame:
CE7D7802FF2071000413229C0C004413229C0C00
C1FF7934DD3E7D7100229C0C00229C0C00
*/
