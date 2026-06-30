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

#ifndef _CC1101WMBUS_H_
#define _CC1101WMBUS_H_

#include <Arduino.h>
#include <SPI.h>
#if 0
#include <Crypto.h>
#include <AES.h>
#include <CTR.h>
#include <PubSubClient.h>
#include "config.h"
#include "utils.h"
#endif

class CC1101Wmbus
{
  private:
    static constexpr uint8_t CC1101_MARCSTATE_IDLE = 0x01;
    static constexpr uint8_t CC1101_MARCSTATE_RX = 0x0D;
    static constexpr uint8_t CC1101_READ_SINGLE = 0x80;
    static constexpr uint8_t CC1101_READ_BURST = 0xC0;
    static constexpr uint8_t CC1101_RXFIFO = 0x3F;     // RX FIFO address
    static constexpr uint8_t CC1101_SRES = 0x30;       // Reset CC1101 chip
    static constexpr uint8_t CC1101_SCAL = 0x33;       // Calibrate frequency synthesizer and turn it off 
    static constexpr uint8_t CC1101_SRX = 0x34;        // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
    static constexpr uint8_t CC1101_SIDLE = 0x36;      // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
    static constexpr uint8_t CC1101_SFRX = 0x3A;       // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
    static constexpr uint8_t CC1101_IOCFG2 = 0x00;     // GDO2 Output Pin Configuration
    static constexpr uint8_t CC1101_IOCFG1 = 0x01;     // GDO1 Output Pin Configuration
    static constexpr uint8_t CC1101_IOCFG0 = 0x02;     // GDO0 Output Pin Configuration
    static constexpr uint8_t CC1101_FIFOTHR = 0x03;    // RX FIFO and TX FIFO Thresholds
    static constexpr uint8_t CC1101_SYNC1 = 0x04;      // Sync Word, High Byte
    static constexpr uint8_t CC1101_SYNC0 = 0x05;      // Sync Word, Low Byte
    static constexpr uint8_t CC1101_PKTLEN = 0x06;     // Packet Length
    static constexpr uint8_t CC1101_PKTCTRL1 = 0x07;   // Packet Automation Control
    static constexpr uint8_t CC1101_PKTCTRL0 = 0x08;   // Packet Automation Control
    static constexpr uint8_t CC1101_ADDR = 0x09;       // Device Address
    static constexpr uint8_t CC1101_CHANNR = 0x0A;     // Channel Number
    static constexpr uint8_t CC1101_FSCTRL1 = 0x0B;    // Frequency Synthesizer Control
    static constexpr uint8_t CC1101_FSCTRL0 = 0x0C;    // Frequency Synthesizer Control
    static constexpr uint8_t CC1101_FREQ2 = 0x0D;      // Frequency Control Word, High Byte
    static constexpr uint8_t CC1101_FREQ1 = 0x0E;      // Frequency Control Word, Middle Byte
    static constexpr uint8_t CC1101_FREQ0 = 0x0F;      // Frequency Control Word, Low Byte
    static constexpr uint8_t CC1101_MDMCFG4 = 0x10;    // Modem Configuration
    static constexpr uint8_t CC1101_MDMCFG3 = 0x11;    // Modem Configuration
    static constexpr uint8_t CC1101_MDMCFG2 = 0x12;    // Modem Configuration
    static constexpr uint8_t CC1101_MDMCFG1 = 0x13;    // Modem Configuration
    static constexpr uint8_t CC1101_MDMCFG0 = 0x14;    // Modem Configuration
    static constexpr uint8_t CC1101_DEVIATN = 0x15;    // Modem Deviation Setting
    static constexpr uint8_t CC1101_MCSM2 = 0x16;      // Main Radio Control State Machine Configuration
    static constexpr uint8_t CC1101_MCSM1 = 0x17;      // Main Radio Control State Machine Configuration
    static constexpr uint8_t CC1101_MCSM0 = 0x18;      // Main Radio Control State Machine Configuration
    static constexpr uint8_t CC1101_FOCCFG = 0x19;     // Frequency Offset Compensation Configuration
    static constexpr uint8_t CC1101_BSCFG = 0x1A;      // Bit Synchronization Configuration
    static constexpr uint8_t CC1101_AGCCTRL2 = 0x1B;   // AGC Control
    static constexpr uint8_t CC1101_AGCCTRL1 = 0x1C;   // AGC Control
    static constexpr uint8_t CC1101_AGCCTRL0 = 0x1D;   // AGC Control
    static constexpr uint8_t CC1101_FREND1 = 0x21;     // Front End RX Configuration
    static constexpr uint8_t CC1101_FREND0 = 0x22;     // Front End TX Configuration
    static constexpr uint8_t CC1101_FSCAL3 = 0x23;     // Frequency Synthesizer Calibration
    static constexpr uint8_t CC1101_FSCAL2 = 0x24;     // Frequency Synthesizer Calibration
    static constexpr uint8_t CC1101_FSCAL1 = 0x25;     // Frequency Synthesizer Calibration
    static constexpr uint8_t CC1101_FSCAL0 = 0x26;     // Frequency Synthesizer Calibration
    static constexpr uint8_t CC1101_RCCTRL1 = 0x27;    // RC Oscillator Configuration
    static constexpr uint8_t CC1101_RCCTRL0 = 0x28;    // RC Oscillator Configuration
    static constexpr uint8_t CC1101_FSTEST = 0x29;     // Frequency Synthesizer Calibration Control
    static constexpr uint8_t CC1101_PTEST = 0x2A;      // Production Test
    static constexpr uint8_t CC1101_AGCTEST = 0x2B;    // AGC Test
    static constexpr uint8_t CC1101_TEST2 = 0x2C;      // Various Test Settings
    static constexpr uint8_t CC1101_TEST1 = 0x2D;      // Various Test Settings
    static constexpr uint8_t CC1101_TEST0 = 0x2E;      // Various Test Settings
  
    static constexpr uint8_t CC1101_PARTNUM = 0x30;        // Chip ID
    static constexpr uint8_t CC1101_VERSION = 0x31;        // Chip ID
    static constexpr uint8_t CC1101_FREQEST = 0x32;        // Frequency Offset Estimate from Demodulator
    static constexpr uint8_t CC1101_LQI = 0x33;            // Demodulator Estimate for Link Quality
    static constexpr uint8_t CC1101_RSSI = 0x34;           // Received Signal Strength Indication
    static constexpr uint8_t CC1101_MARCSTATE = 0x35;       // Main Radio Control State Machine State
    static constexpr uint8_t CC1101_PKTSTATUS = 0x38;       // Current GDOx Status and Packet Status
    static constexpr uint8_t CC1101_RXBYTES = 0x3B;         // Overflow and Number of Bytes
    static constexpr uint8_t CC1101_RCCTRL1_STATUS = 0x3C;  // Last RC Oscillator Calibration Result
    static constexpr uint8_t CC1101_RCCTRL0_STATUS = 0x3D;  // Last RC Oscillator Calibration Result 

    static constexpr uint8_t CC1101_DEFVAL_SYNC1 = 0x54;    // Synchronization word, high byte
    static constexpr uint8_t CC1101_DEFVAL_SYNC0 = 0x3D;    // Synchronization word, low byte
    static constexpr uint8_t CC1101_DEFVAL_MCSM1 = 0x00;    // Main Radio Control State Machine Configuration

    static constexpr uint8_t CC1101_DEFVAL_IOCFG2 = 0x2E;   // GDO2 Output Pin Configuration
    static constexpr uint8_t CC1101_DEFVAL_IOCFG0 = 0x06;   // GDO0 Output Pin Configuration

    static constexpr uint8_t CC1101_DEFVAL_FSCTRL1 = 0x08;  // Frequency Synthesizer Control
    static constexpr uint8_t CC1101_DEFVAL_FSCTRL0 = 0x00;  // Frequency Synthesizer Control
    static constexpr uint8_t CC1101_DEFVAL_FREQ2 = 0x21;    // Frequency Control Word, High Byte
    static constexpr uint8_t CC1101_DEFVAL_FREQ1 = 0x6B;    // Frequency Control Word, Middle Byte
    static constexpr uint8_t CC1101_DEFVAL_FREQ0 = 0xD0;    // Frequency Control Word, Low Byte
    static constexpr uint8_t CC1101_DEFVAL_MDMCFG4 = 0x5C;  // Modem configuration. Speed = 103 Kbps
    static constexpr uint8_t CC1101_DEFVAL_MDMCFG3 = 0x04;  // Modem Configuration
    static constexpr uint8_t CC1101_DEFVAL_MDMCFG2 = 0x06;  // Modem Configuration
    static constexpr uint8_t CC1101_DEFVAL_MDMCFG1 = 0x22;  // Modem Configuration
    static constexpr uint8_t CC1101_DEFVAL_MDMCFG0 = 0xF8;  // Modem Configuration
    static constexpr uint8_t CC1101_DEFVAL_CHANNR = 0x00;   // Channel Number
    static constexpr uint8_t CC1101_DEFVAL_DEVIATN = 0x44;  // Modem Deviation Setting
    static constexpr uint8_t CC1101_DEFVAL_FREND1 = 0xB6;   // Front End RX Configuration
    static constexpr uint8_t CC1101_DEFVAL_FREND0 = 0x10;   // Front End TX Configuration
    static constexpr uint8_t CC1101_DEFVAL_MCSM0 = 0x18;    // Main Radio Control State Machine Configuration
    static constexpr uint8_t CC1101_DEFVAL_FOCCFG = 0x2E;   // Frequency Offset Compensation Configuration
    static constexpr uint8_t CC1101_DEFVAL_BSCFG = 0xBF;    // Bit Synchronization Configuration
    static constexpr uint8_t CC1101_DEFVAL_AGCCTRL2 = 0x43; // AGC Control
    static constexpr uint8_t CC1101_DEFVAL_AGCCTRL1 = 0x09; // AGC Control
    static constexpr uint8_t CC1101_DEFVAL_AGCCTRL0 = 0xB5; // AGC Control
    static constexpr uint8_t CC1101_DEFVAL_FSCAL3 = 0xEA;   // Frequency Synthesizer Calibration
    static constexpr uint8_t CC1101_DEFVAL_FSCAL2 = 0x2A;   // Frequency Synthesizer Calibration
    static constexpr uint8_t CC1101_DEFVAL_FSCAL1 = 0x00;   // Frequency Synthesizer Calibration
    static constexpr uint8_t CC1101_DEFVAL_FSCAL0 = 0x1F;   // Frequency Synthesizer Calibration
    static constexpr uint8_t CC1101_DEFVAL_FSTEST = 0x59;   // Frequency Synthesizer Calibration Control
    static constexpr uint8_t CC1101_DEFVAL_TEST2 = 0x81;    // Various Test Settings
    static constexpr uint8_t CC1101_DEFVAL_TEST1 = 0x35;    // Various Test Settings
    static constexpr uint8_t CC1101_DEFVAL_TEST0 = 0x09;    // Various Test Settings
    static constexpr uint8_t CC1101_DEFVAL_PKTCTRL1 = 0x00; // Packet Automation Control
    static constexpr uint8_t CC1101_DEFVAL_PKTCTRL0 = 0x02; // 2 - infinite length 
    static constexpr uint8_t CC1101_DEFVAL_ADDR = 0x00;     // Device Address
    static constexpr uint8_t CC1101_DEFVAL_PKTLEN = 0x30;   // Packet Length
    static constexpr uint8_t CC1101_DEFVAL_FIFOTHR = 0x00;  // RX 4 bytes and TX 61 bytes Thresholds

    const uint32_t RECEIVE_TIMEOUT = 300000UL;  // 5 min
    uint32_t lastFrameReceived = 0;
    volatile boolean packetAvailable = false;
    inline void selectCC1101(void);
    inline void deselectCC1101(void);
    inline void waitMiso(void);
    bool frameReceived = false;     // wmbus frame c1 detected
    static const uint8_t MAX_LENGTH = 64;
    uint8_t length = 0; // payload length
    uint8_t payload[MAX_LENGTH]; // payload data

 // reset HW and restart receiver
    void restartRadio(void);

    // flush fifo and (re)start receiver
    void startReceiver(void);

    //void writeBurstReg(uint8_t regaddr, uint8_t* buffer, uint8_t len);
    void readBurstReg(uint8_t * buffer, uint8_t regaddr, uint8_t len);
    void cmdStrobe(uint8_t cmd);
    uint8_t readReg(uint8_t regaddr, uint8_t regtype);
    uint8_t readByteFromFifo(void);
    void writeReg(uint8_t regaddr, uint8_t value);
    void initializeRegisters(void);
    void reset(void);

    // static ISR calls instanceISR via this pointer
    IRAM_ATTR static void cc1101Isr(void *p);

//    void CC1101_ISR();

    // receive a wmbus frame 
    void receive(void); // read frame from CC1101
    bool checkFrame(void);  // check id, CRC
    
    using Callback = std::function<void(uint8_t*)>;
    Callback callback;

  public:

    // constructor
    CC1101Wmbus();
    
    // setup CC1101 for receiving wmbus mode c 
    bool begin();

    // must be called frequently
    void loop(void);

    //void setCallback(void (*callback)(uint8_t *payload));
    void setCallback(Callback cb);

    IRAM_ATTR void instanceCC1101Isr();
};

#endif // _CC1101WMBUS_H_
