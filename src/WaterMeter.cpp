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

// ChipSelect assert
inline void WaterMeter::selectCC1101(void)
{
  digitalWrite(SS, LOW);
}

// ChipSelect deassert
inline void WaterMeter::deselectCC1101(void)
{
  digitalWrite(SS, HIGH);
}

// wait for MISO pulling down
inline void WaterMeter::waitMiso(void)
{
  while (digitalRead(MISO) == HIGH)
    ;
}

// write a single register of CC1101
void WaterMeter::writeReg(uint8_t regAddr, uint8_t value)
{
  selectCC1101();
  waitMiso();            // Wait until MISO goes low
  SPI.transfer(regAddr); // Send register address
  SPI.transfer(value);   // Send value
  deselectCC1101();
}

// send a strobe command to CC1101
void WaterMeter::cmdStrobe(uint8_t cmd)
{
  selectCC1101();
  delayMicroseconds(5);
  waitMiso();        // Wait until MISO goes low
  SPI.transfer(cmd); // Send strobe command
  delayMicroseconds(5);
  deselectCC1101();
}

// read CC1101 register (status or configuration)
uint8_t WaterMeter::readReg(uint8_t regAddr, uint8_t regType)
{
  uint8_t addr, val;

  addr = regAddr | regType;
  selectCC1101();
  waitMiso();               // Wait until MISO goes low
  SPI.transfer(addr);       // Send register address
  val = SPI.transfer(0x00); // Read result
  deselectCC1101();

  return val;
}

//
void WaterMeter::readBurstReg(uint8_t *buffer, uint8_t regAddr, uint8_t len)
{
  uint8_t addr, i;

  addr = regAddr | READ_BURST;
  selectCC1101();
  delayMicroseconds(5);
  waitMiso();         // Wait until MISO goes low
  SPI.transfer(addr); // Send register address
  for (i = 0; i < len; i++)
    buffer[i] = SPI.transfer(0x00); // Read result byte by byte
  delayMicroseconds(2);
  deselectCC1101();
}

// power on reset
void WaterMeter::reset(void)
{
  deselectCC1101();
  delayMicroseconds(3);

  digitalWrite(MOSI, LOW);
  digitalWrite(SCK, HIGH); // see CC1101 datasheet 11.3

  selectCC1101();
  delayMicroseconds(3);
  deselectCC1101();
  delayMicroseconds(45); // at least 40 us

  selectCC1101();

  waitMiso();                // Wait until MISO goes low
  SPI.transfer(CC1101_SRES); // Send reset command strobe
  waitMiso();                // Wait until MISO goes low

  deselectCC1101();
}

// set IDLE state, flush FIFO and (re)start receiver
void WaterMeter::startReceiver(void)
{
  uint8_t regCount = 0;
  cmdStrobe(CC1101_SIDLE); // Enter IDLE state
  while (readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) != MARCSTATE_IDLE)
  {
    if (regCount++ > 100)
    {
      Serial.println("Enter idle state failed!\n");
      restartRadio();
    }
  }

  cmdStrobe(CC1101_SFRX); // flush receive queue
  delay(5);

  regCount = 0;
  cmdStrobe(CC1101_SRX); // Enter RX state
  delay(10);
  while (readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) != MARCSTATE_RX)
  {
    if (regCount++ > 100)
    {
      Serial.println("Enter RX state failed!\n");
      restartRadio();
    }
  }
}

// initialize all the CC1101 registers
void WaterMeter::initializeRegisters(void)
{
  writeReg(CC1101_IOCFG2, CC1101_DEFVAL_IOCFG2);
  writeReg(CC1101_IOCFG0, CC1101_DEFVAL_IOCFG0);
  writeReg(CC1101_FIFOTHR, CC1101_DEFVAL_FIFOTHR);
  writeReg(CC1101_PKTLEN, CC1101_DEFVAL_PKTLEN);
  writeReg(CC1101_PKTCTRL1, CC1101_DEFVAL_PKTCTRL1);
  writeReg(CC1101_PKTCTRL0, CC1101_DEFVAL_PKTCTRL0);
  writeReg(CC1101_SYNC1, CC1101_DEFVAL_SYNC1);
  writeReg(CC1101_SYNC0, CC1101_DEFVAL_SYNC0);
  writeReg(CC1101_ADDR, CC1101_DEFVAL_ADDR);
  writeReg(CC1101_CHANNR, CC1101_DEFVAL_CHANNR);
  writeReg(CC1101_FSCTRL1, CC1101_DEFVAL_FSCTRL1);
  writeReg(CC1101_FSCTRL0, CC1101_DEFVAL_FSCTRL0);
  writeReg(CC1101_FREQ2, CC1101_DEFVAL_FREQ2);
  writeReg(CC1101_FREQ1, CC1101_DEFVAL_FREQ1);
  writeReg(CC1101_FREQ0, CC1101_DEFVAL_FREQ0);
  writeReg(CC1101_MDMCFG4, CC1101_DEFVAL_MDMCFG4);
  writeReg(CC1101_MDMCFG3, CC1101_DEFVAL_MDMCFG3);
  writeReg(CC1101_MDMCFG2, CC1101_DEFVAL_MDMCFG2);
  writeReg(CC1101_MDMCFG1, CC1101_DEFVAL_MDMCFG1);
  writeReg(CC1101_MDMCFG0, CC1101_DEFVAL_MDMCFG0);
  writeReg(CC1101_DEVIATN, CC1101_DEFVAL_DEVIATN);
  writeReg(CC1101_MCSM1, CC1101_DEFVAL_MCSM1);
  writeReg(CC1101_MCSM0, CC1101_DEFVAL_MCSM0);
  writeReg(CC1101_FOCCFG, CC1101_DEFVAL_FOCCFG);
  writeReg(CC1101_BSCFG, CC1101_DEFVAL_BSCFG);
  writeReg(CC1101_AGCCTRL2, CC1101_DEFVAL_AGCCTRL2);
  writeReg(CC1101_AGCCTRL1, CC1101_DEFVAL_AGCCTRL1);
  writeReg(CC1101_AGCCTRL0, CC1101_DEFVAL_AGCCTRL0);
  writeReg(CC1101_FREND1, CC1101_DEFVAL_FREND1);
  writeReg(CC1101_FREND0, CC1101_DEFVAL_FREND0);
  writeReg(CC1101_FSCAL3, CC1101_DEFVAL_FSCAL3);
  writeReg(CC1101_FSCAL2, CC1101_DEFVAL_FSCAL2);
  writeReg(CC1101_FSCAL1, CC1101_DEFVAL_FSCAL1);
  writeReg(CC1101_FSCAL0, CC1101_DEFVAL_FSCAL0);
  writeReg(CC1101_FSTEST, CC1101_DEFVAL_FSTEST);
  writeReg(CC1101_TEST2, CC1101_DEFVAL_TEST2);
  writeReg(CC1101_TEST1, CC1101_DEFVAL_TEST1);
  writeReg(CC1101_TEST0, CC1101_DEFVAL_TEST0);
}

IRAM_ATTR void WaterMeter::instanceCC1101Isr()
{
  // set the flag that a package is available
  packetAvailable = true;
}

// static ISR method, that calls the right instance
IRAM_ATTR void WaterMeter::cc1101Isr(void *p)
{
  WaterMeter *ptr = (WaterMeter *)p;
  ptr->instanceCC1101Isr();
}

// should be called frequently, handles the ISR flag
// does the frame checkin and decryption
void WaterMeter::loop(void)
{
  if (packetAvailable)
  {
    // Serial.println("packet received");
    //  Disable wireless reception interrupt
    detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));

    // clear the flag
    packetAvailable = false;
    receive();

    // Enable wireless reception interrupt
    attachInterruptArg(digitalPinToInterrupt(CC1101_GDO0), cc1101Isr, this, FALLING);
  }

  if (millis() - lastFrameReceived > RECEIVE_TIMEOUT)
  {
    // workaround: reset CC1101, since it stops receiving from time to time
    restartRadio();
  }
}

// Initialize CC1101 to receive WMBus MODE C1
void WaterMeter::begin(uint8_t *key, uint8_t *id)
{
  pinMode(SS, OUTPUT);         // SS Pin -> Output
  SPI.begin();                 // Initialize SPI interface
  pinMode(CC1101_GDO0, INPUT); // Config GDO0 as input

  memcpy(aesKey, key, sizeof(aesKey));
  aes128.setKey(aesKey, sizeof(aesKey));

  memcpy(meterId, id, sizeof(meterId));

  restartRadio();
  attachInterruptArg(digitalPinToInterrupt(CC1101_GDO0), cc1101Isr, this, FALLING);
  lastFrameReceived = millis();
}

void WaterMeter::restartRadio()
{
  Serial.println("resetting CC1101");

  reset(); // power on CC1101

  // Serial.println("Setting CC1101 registers");
  initializeRegisters(); // init CC1101 registers

  cmdStrobe(CC1101_SCAL);
  delay(1);

  startReceiver();
  lastFrameReceived = millis();
}

bool WaterMeter::checkFrame(void)
{
#if DEBUG
  Serial.printf("frame serial ID: ");
  for (uint8_t i = 0; i < 4; i++)
  {
    Serial.printf("%02x", payload[7-i]);
  }
  Serial.printf(" - %d", length);
  Serial.println();
#endif

  // check meterId
  for (uint8_t i = 0; i < 4; i++)
  {
    if (meterId[i] != payload[7 - i])
    {
#if DEBUG
      Serial.println("Meter serial doesnt match!");
#endif
      return false;
    }
  }

#if DEBUG
  Serial.println("Frame payload:");
  for (uint8_t i = 0; i <= length; i++)
  {
    Serial.printf("%02x", payload[i]);
  }
  Serial.println();
#endif

  uint16_t crc = crcEN13575(payload, length - 1); // -2 (CRC) + 1 (L-field)
  if (crc != (payload[length - 1] << 8 | payload[length]))
  {
    Serial.println("CRC Error");
    Serial.printf("%04x - %02x%02x\n", crc, payload[length - 1], payload[length]);
    return false;
  }

  return true;
}

void WaterMeter::getMeterInfo(uint8_t *data, size_t len)
{
  // init positions for compact frame
  int pos_tt = 9;  // total consumption
  int pos_tg = 13; // target consumption
  int pos_ic = 7;  // info codes
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

  totalWater = data[pos_tt] + (data[pos_tt + 1] << 8) + (data[pos_tt + 2] << 16) + (data[pos_tt + 3] << 24);

  targetWater = data[pos_tg] + (data[pos_tg + 1] << 8) + (data[pos_tg + 2] << 16) + (data[pos_tg + 3] << 24);

  flowTemp = data[pos_ft];
  ambientTemp = data[pos_at];
  infoCodes = data[pos_ic];
}

void WaterMeter::publishMeterInfo()
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

  // change the topics as you like
  mqttClient.publish(MQTT_PREFIX MQTT_total, total);
  mqttClient.publish(MQTT_PREFIX MQTT_target, target);
  mqttClient.loop();
  mqttClient.publish(MQTT_PREFIX MQTT_ftemp, flow_temp);
  mqttClient.publish(MQTT_PREFIX MQTT_atemp, ambient_temp);
  mqttClient.publish(MQTT_PREFIX MQTT_info, info_codes);
  mqttClient.loop();
}

// reads a single byte from the RX fifo
uint8_t WaterMeter::readByteFromFifo(void)
{
  return readReg(CC1101_RXFIFO, CC1101_CONFIG_REGISTER);
}

// handles a received frame and restart the CC1101 receiver
void WaterMeter::receive()
{
  // read preamble, should be 0x543D
  uint8_t p1 = readByteFromFifo();
  uint8_t p2 = readByteFromFifo();
  
#if DEBUG
  Serial.printf("%02x%02x", p1, p2);
#endif

  // get length
  payload[0] = readByteFromFifo();

  // is it Mode C1, frame B and does it fit in the buffer
  if ((payload[0] < MAX_LENGTH) && (p1 == 0x54) && (p2 == 0x3D))
  {
    // 3rd byte is payload length
    length = payload[0];

#if DEBUG
    Serial.printf("%02X", length);
#endif

    // starting with 1! index 0 is lfield
    for (int i = 0; i < length; i++)
    {
      payload[i + 1] = readByteFromFifo();
    }

    // check meterId, CRC
    if (checkFrame())
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

      aes128.setIV(iv, sizeof(iv));
      aes128.decrypt(plaintext, (const uint8_t *)cipher, cipherLength);

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

      // received packet is ok
      lastPacketDecoded = millis();

      lastFrameReceived = millis();
      getMeterInfo(plaintext, cipherLength);
      publishMeterInfo();
    }
  }

  // flush RX fifo and restart receiver
  startReceiver();
  // Serial.printf("rxStatus: 0x%02x\n\r", readStatusReg(CC1101_RXBYTES));
}