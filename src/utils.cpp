
#include "utils.h"

void printHex(uint8_t *buf, size_t len)
{
  for (size_t i = 0; i < len; i++)
  {
    Serial.printf("%02X ", buf[i]);
    if ((i+1) % 16 == 0) Serial.println();
  }
  Serial.println();
}

uint16_t crcX25(uint8_t *payload, uint16_t length)
{
   return crcInternal(payload, length, 0x1021, 0xffff, true, true);
}

uint16_t crcEN13575(uint8_t *payload, uint16_t length)
{
   return crcInternal(payload, length, 0x3D65, 0x0000, false, false);
}

uint16_t mirror(uint16_t crc, uint8_t bitnum)
{
  // mirrors the lower 'bitnum' bits of 'crc'

  uint16_t i, j = 1, crcout = 0;

  for (i = (uint16_t)1 << (bitnum - 1); i; i >>= 1)
  {
    if (crc & i)
    {
      crcout |= j;
    }
    j <<= 1;
  }
  return crcout;
}

uint16_t crcInternal(uint8_t *p, uint16_t len, uint16_t poly, uint16_t init, bool revIn, bool revOut)
{
    uint16_t i, j, c, bit, crc;

    crc = init;
    for (i = 0; i < 16; i++)
    {
      bit = crc & 1;
      if (bit) crc ^= poly;
      crc >>= 1;
      if (bit) crc |= 0x8000;
    }

    // bit by bit algorithm with augmented zero bytes.
    // does not use lookup table, suited for polynom orders between 1...32.

    for (i = 0; i < len; i++)
    {
      c = (uint16_t)*p++;
      if (revIn) c = mirror(c, 8);

      for (j = 0x80; j; j >>= 1)
      {
        bit = crc & 0x8000;
        crc <<= 1;
        if (c & j) crc |= 1;
        if (bit) crc ^= poly;
      }
    }

    for (i = 0; i < 16; i++)
    {
        bit = crc & 0x8000;
        crc <<= 1;
        if (bit) crc ^= poly;
    }

    if (revOut) crc = mirror(crc, 16);
    crc ^= 0xffff;  // crcxor

    return crc;
}


// convert _in_ to _len_ hex numbers stored in _out_
// _in_ "EF01" to 2 hex numbers: 0xEf, 0x01
void hex2bin(const char *in, size_t len, uint8_t *out)
{
    const char *pos = in;

    for(size_t count = 0; count < len; count++)
    {
        char buf[5] = {'0', 'x', pos[0], pos[1], 0};
        out[count] = strtol(buf, NULL, 0);
        pos += 2 * sizeof(char);
    }
}

void bin2hex(char *xp, uint8_t *bb, int n) 
{
    const char xx[]= "0123456789ABCDEF";
    while (--n >= 0) xp[n] = xx[(bb[n>>1] >> ((1 - (n&1)) << 2)) & 0xF];
}