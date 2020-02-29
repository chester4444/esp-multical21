#ifndef __WMBUS_FRAME__
#define __WMBUS_FRAME__

#include <Arduino.h>
#include <Crypto.h>
#include <AES.h>
#include <CTR.h>
#include "credentials.h"

class WMBusFrame
{
  public:
    static const uint8_t MAX_LENGTH = 64;
  private:
    CTR<AESSmall128> aes128;
    const uint8_t meterId[4] = { W_SERIAL_NUMBER }; // Multical21 serial number
    const uint8_t key[16] = { W_ENCRYPTION_KEY }; // AES-128 key
    uint8_t cipher[MAX_LENGTH];
    uint8_t plaintext[MAX_LENGTH];
    uint8_t iv[16];
    void check(void);
    void printMeterInfo(uint8_t *data, size_t len);

  public:
    // check frame and decrypt it
    void decode(void);

    // true, if meter information is valid for the last received frame
    bool isValid = false;

    // payload length
    uint8_t length = 0;

    // payload data
    uint8_t payload[MAX_LENGTH];

    // constructor
    WMBusFrame();
};

#endif // __WMBUS_FRAME__
