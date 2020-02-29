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

#include "WMbusFrame.h"

WMBusFrame::WMBusFrame()
{
  aes128.setKey(key, sizeof(key));
}

void WMBusFrame::check()
{
    // check meterId
    for (uint8_t i = 0; i< 4; i++)
    {
        if (meterId[i] != payload[6-i])
        {
          isValid = false;
          return;
        }
    }

    // TBD: check crc
    isValid = true;
}

void WMBusFrame::printMeterInfo(uint8_t *data, size_t len)
{
    // init positions for compact frame
  int pos_tt = 9; // total consumption
  int pos_tg = 13; // target consumption
  int pos_ic = 7; // info codes
  int pos_ft = 17; // flow temp
  int pos_at = 18; // ambient temp

  if (data[2] == 0x78) // long frame
  {
    // overwrite it with long frame positions
    pos_tt = 10;
    pos_tg = 16;
    pos_ic = 6;
    pos_ft = 22;
    pos_at = 25;
  }

  char total[10];
  uint32_t tt = data[pos_tt]
              + (data[pos_tt+1] << 8)
              + (data[pos_tt+2] << 16)
              + (data[pos_tt+3] << 24);
  snprintf(total, sizeof(total), "%d.%03d", tt/1000, tt%1000 );
  Serial.printf("total: %s m%c - ", total, 179);

  char target[10];
  uint32_t tg = data[pos_tg]
              + (data[pos_tg+1] << 8)
              + (data[pos_tg+2] << 16)
              + (data[pos_tg+3] << 24);
  snprintf(target, sizeof(target), "%d.%03d", tg/1000, tg%1000 );
  Serial.printf("target: %s m%c - ", target, 179);

  char flow_temp[3];
  snprintf(flow_temp, sizeof(flow_temp), "%2d", data[pos_ft]);
  Serial.printf("%s %cC - ", flow_temp, 176);

  char ambient_temp[3];
  snprintf(ambient_temp, sizeof(ambient_temp), "%2d", data[pos_at]);
  Serial.printf("%s %cC\n\r", ambient_temp, 176);
}

void WMBusFrame::decode()
{
  // check meterId, CRC
  check();
  if (!isValid) return;

  uint8_t cipherLength = length - 2 - 16; // cipher starts at index 16, remove 2 crc bytes
  memcpy(cipher, &payload[16], cipherLength);

  memset(iv, 0, sizeof(iv));   // padding with 0
  memcpy(iv, &payload[1], 8);
  iv[8] = payload[10];
  memcpy(&iv[9], &payload[12], 4);

  aes128.setIV(iv, sizeof(iv));
  aes128.decrypt(plaintext, (const uint8_t *) cipher, cipherLength);

/*
  Serial.printf("C:     ");
  for (size_t i = 0; i < cipherLength; i++)
  {
    Serial.printf("%02X", cipher[i]);
  }
  Serial.println();
  Serial.printf("P(%d): ", cipherLength);
  for (size_t i = 0; i < cipherLength; i++)
  {
    Serial.printf("%02X", plaintext[i]);
  }
  Serial.println();
*/

  printMeterInfo(plaintext, cipherLength);
}