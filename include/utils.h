#ifndef __UTILS_H__
#define __UTILS_H__

#include <Arduino.h>
#include <inttypes.h>

void printHex(uint8_t * buf, size_t len);

uint16_t crcEN13575(uint8_t *payload, uint16_t length);
uint16_t mirror(uint16_t crc, uint8_t bitnum);
uint16_t crcInternal(uint8_t *p, uint16_t len, uint16_t poly, uint16_t init, bool revIn, bool revOut);
void bin2hex(char *xp, uint8_t *bb, int n);
void hex2bin(const char *in, size_t len, uint8_t *out);

#endif //__UTILS_H__