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
#include <SPI.h>
#include <Crypto.h>
#include <AES.h>
#include <CTR.h>
#include <PubSubClient.h>
#include "config.h"
#include "utils.h"

#define MARCSTATE_SLEEP            0x00
#define MARCSTATE_IDLE             0x01
#define MARCSTATE_XOFF             0x02
#define MARCSTATE_VCOON_MC         0x03
#define MARCSTATE_REGON_MC         0x04
#define MARCSTATE_MANCAL           0x05
#define MARCSTATE_VCOON            0x06
#define MARCSTATE_REGON            0x07
#define MARCSTATE_STARTCAL         0x08
#define MARCSTATE_BWBOOST          0x09
#define MARCSTATE_FS_LOCK          0x0A
#define MARCSTATE_IFADCON          0x0B
#define MARCSTATE_ENDCAL           0x0C
#define MARCSTATE_RX               0x0D
#define MARCSTATE_RX_END           0x0E
#define MARCSTATE_RX_RST           0x0F
#define MARCSTATE_TXRX_SWITCH      0x10
#define MARCSTATE_RXFIFO_OVERFLOW  0x11
#define MARCSTATE_FSTXON           0x12
#define MARCSTATE_TX               0x13
#define MARCSTATE_TX_END           0x14
#define MARCSTATE_RXTX_SWITCH      0x15
#define MARCSTATE_TXFIFO_UNDERFLOW 0x16

#define WRITE_BURST              0x40
#define READ_SINGLE              0x80
#define READ_BURST               0xC0

#define CC1101_CONFIG_REGISTER   READ_SINGLE
#define CC1101_STATUS_REGISTER   READ_BURST

#define CC1101_PATABLE           0x3E        // PATABLE address
#define CC1101_TXFIFO            0x3F        // TX FIFO address
#define CC1101_RXFIFO            0x3F        // RX FIFO address

#define CC1101_SRES              0x30        // Reset CC1101 chip
#define CC1101_SFSTXON           0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA):
                                             // Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
#define CC1101_SXOFF             0x32        // Turn off crystal oscillator
#define CC1101_SCAL              0x33        // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without
                                             // setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define CC1101_SRX               0x34        // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1
#define CC1101_STX               0x35        // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1.
                                             // If in RX state and CCA is enabled: Only go to TX if channel is clear
#define CC1101_SIDLE             0x36        // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
#define CC1101_SWOR              0x38        // Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if
                                             // WORCTRL.RC_PD=0
#define CC1101_SPWD              0x39        // Enter power down mode when CSn goes high
#define CC1101_SFRX              0x3A        // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC1101_SFTX              0x3B        // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC1101_SWORRST           0x3C        // Reset real time clock to Event1 value
#define CC1101_SNOP              0x3D        // No operation. May be used to get access to the chip status byte

#define CC1101_IOCFG2            0x00        // GDO2 Output Pin Configuration
#define CC1101_IOCFG1            0x01        // GDO1 Output Pin Configuration
#define CC1101_IOCFG0            0x02        // GDO0 Output Pin Configuration
#define CC1101_FIFOTHR           0x03        // RX FIFO and TX FIFO Thresholds
#define CC1101_SYNC1             0x04        // Sync Word, High Byte
#define CC1101_SYNC0             0x05        // Sync Word, Low Byte
#define CC1101_PKTLEN            0x06        // Packet Length
#define CC1101_PKTCTRL1          0x07        // Packet Automation Control
#define CC1101_PKTCTRL0          0x08        // Packet Automation Control
#define CC1101_ADDR              0x09        // Device Address
#define CC1101_CHANNR            0x0A        // Channel Number
#define CC1101_FSCTRL1           0x0B        // Frequency Synthesizer Control
#define CC1101_FSCTRL0           0x0C        // Frequency Synthesizer Control
#define CC1101_FREQ2             0x0D        // Frequency Control Word, High Byte
#define CC1101_FREQ1             0x0E        // Frequency Control Word, Middle Byte
#define CC1101_FREQ0             0x0F        // Frequency Control Word, Low Byte
#define CC1101_MDMCFG4           0x10        // Modem Configuration
#define CC1101_MDMCFG3           0x11        // Modem Configuration
#define CC1101_MDMCFG2           0x12        // Modem Configuration
#define CC1101_MDMCFG1           0x13        // Modem Configuration
#define CC1101_MDMCFG0           0x14        // Modem Configuration
#define CC1101_DEVIATN           0x15        // Modem Deviation Setting
#define CC1101_MCSM2             0x16        // Main Radio Control State Machine Configuration
#define CC1101_MCSM1             0x17        // Main Radio Control State Machine Configuration
#define CC1101_MCSM0             0x18        // Main Radio Control State Machine Configuration
#define CC1101_FOCCFG            0x19        // Frequency Offset Compensation Configuration
#define CC1101_BSCFG             0x1A        // Bit Synchronization Configuration
#define CC1101_AGCCTRL2          0x1B        // AGC Control
#define CC1101_AGCCTRL1          0x1C        // AGC Control
#define CC1101_AGCCTRL0          0x1D        // AGC Control
#define CC1101_WOREVT1           0x1E        // High Byte Event0 Timeout
#define CC1101_WOREVT0           0x1F        // Low Byte Event0 Timeout
#define CC1101_WORCTRL           0x20        // Wake On Radio Control
#define CC1101_FREND1            0x21        // Front End RX Configuration
#define CC1101_FREND0            0x22        // Front End TX Configuration
#define CC1101_FSCAL3            0x23        // Frequency Synthesizer Calibration
#define CC1101_FSCAL2            0x24        // Frequency Synthesizer Calibration
#define CC1101_FSCAL1            0x25        // Frequency Synthesizer Calibration
#define CC1101_FSCAL0            0x26        // Frequency Synthesizer Calibration
#define CC1101_RCCTRL1           0x27        // RC Oscillator Configuration
#define CC1101_RCCTRL0           0x28        // RC Oscillator Configuration
#define CC1101_FSTEST            0x29        // Frequency Synthesizer Calibration Control
#define CC1101_PTEST             0x2A        // Production Test
#define CC1101_AGCTEST           0x2B        // AGC Test
#define CC1101_TEST2             0x2C        // Various Test Settings
#define CC1101_TEST1             0x2D        // Various Test Settings
#define CC1101_TEST0             0x2E        // Various Test Settings

#define CC1101_PARTNUM           0x30        // Chip ID
#define CC1101_VERSION           0x31        // Chip ID
#define CC1101_FREQEST           0x32        // Frequency Offset Estimate from Demodulator
#define CC1101_LQI               0x33        // Demodulator Estimate for Link Quality
#define CC1101_RSSI              0x34        // Received Signal Strength Indication
#define CC1101_MARCSTATE         0x35        // Main Radio Control State Machine State
#define CC1101_WORTIME1          0x36        // High Byte of WOR Time
#define CC1101_WORTIME0          0x37        // Low Byte of WOR Time
#define CC1101_PKTSTATUS         0x38        // Current GDOx Status and Packet Status
#define CC1101_VCO_VC_DAC        0x39        // Current Setting from PLL Calibration Module
#define CC1101_TXBYTES           0x3A        // Underflow and Number of Bytes
#define CC1101_RXBYTES           0x3B        // Overflow and Number of Bytes
#define CC1101_RCCTRL1_STATUS    0x3C        // Last RC Oscillator Calibration Result
#define CC1101_RCCTRL0_STATUS    0x3D        // Last RC Oscillator Calibration Result 

#define CC1101_DEFVAL_SYNC1      0x54        // Synchronization word, high byte
#define CC1101_DEFVAL_SYNC0      0x3D        // Synchronization word, low byte
#define CC1101_DEFVAL_MCSM1      0x00        // Main Radio Control State Machine Configuration

#define CC1101_DEFVAL_IOCFG2     0x2E        // GDO2 Output Pin Configuration
#define CC1101_DEFVAL_IOCFG0     0x06        // GDO0 Output Pin Configuration

#define CC1101_DEFVAL_FSCTRL1    0x08        // Frequency Synthesizer Control
#define CC1101_DEFVAL_FSCTRL0    0x00        // Frequency Synthesizer Control
#define CC1101_DEFVAL_FREQ2      0x21        // Frequency Control Word, High Byte
#define CC1101_DEFVAL_FREQ1      0x6B        // Frequency Control Word, Middle Byte
#define CC1101_DEFVAL_FREQ0      0xD0        // Frequency Control Word, Low Byte
#define CC1101_DEFVAL_MDMCFG4    0x5C        // Modem configuration. Speed = 103 Kbps
#define CC1101_DEFVAL_MDMCFG3    0x04        // Modem Configuration
#define CC1101_DEFVAL_MDMCFG2    0x06        // Modem Configuration
#define CC1101_DEFVAL_MDMCFG1    0x22        // Modem Configuration
#define CC1101_DEFVAL_MDMCFG0    0xF8        // Modem Configuration
#define CC1101_DEFVAL_CHANNR     0x00        // Channel Number
#define CC1101_DEFVAL_DEVIATN    0x44        // Modem Deviation Setting
#define CC1101_DEFVAL_FREND1     0xB6        // Front End RX Configuration
#define CC1101_DEFVAL_FREND0     0x10        // Front End TX Configuration
#define CC1101_DEFVAL_MCSM0      0x18        // Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_FOCCFG     0x2E      // Frequency Offset Compensation Configuration
#define CC1101_DEFVAL_BSCFG      0xBF        // Bit Synchronization Configuration
#define CC1101_DEFVAL_AGCCTRL2   0x43        // AGC Control
#define CC1101_DEFVAL_AGCCTRL1   0x09        // AGC Control
#define CC1101_DEFVAL_AGCCTRL0   0xB5        // AGC Control
#define CC1101_DEFVAL_FSCAL3     0xEA        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL2     0x2A        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL1     0x00        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL0     0x1F        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSTEST     0x59        // Frequency Synthesizer Calibration Control
#define CC1101_DEFVAL_TEST2      0x81        // Various Test Settings
#define CC1101_DEFVAL_TEST1      0x35        // Various Test Settings
#define CC1101_DEFVAL_TEST0      0x09        // Various Test Settings
#define CC1101_DEFVAL_PKTCTRL1   0x00        // Packet Automation Control
#define CC1101_DEFVAL_PKTCTRL0   0x02        // 2 - infinite length 
#define CC1101_DEFVAL_ADDR       0x00        // Device Address
#define CC1101_DEFVAL_PKTLEN     0x30        // Packet Length
#define CC1101_DEFVAL_FIFOTHR    0x00        // RX 4 bytes and TX 61 bytes Thresholds

class WaterMeter
{
  private:
    const uint32_t RECEIVE_TIMEOUT = 300000UL;  // in millis
    const uint32_t PACKET_TIMEOUT = 180000UL; // in seconds
    uint32_t lastPacketDecoded = -PACKET_TIMEOUT;
    uint32_t lastFrameReceived = 0;
    volatile boolean packetAvailable = false;
    uint8_t meterId[4];
    uint8_t aesKey[16];
    inline void selectCC1101(void);
    inline void deselectCC1101(void);
    inline void waitMiso(void);
    static const uint8_t MAX_LENGTH = 64;
    CTR<AESSmall128> aes128;
    uint8_t cipher[MAX_LENGTH];
    uint8_t plaintext[MAX_LENGTH];
    uint8_t iv[16];
    bool isValid = false; // true, if meter information is valid for the last received frame
    uint8_t length = 0; // payload length
    uint8_t payload[MAX_LENGTH]; // payload data
    uint32_t totalWater;
    uint32_t targetWater;
    uint32_t lastTarget=0;
    uint8_t flowTemp;
    uint8_t ambientTemp;
    uint8_t infoCodes;

    PubSubClient &mqttClient;
    bool mqttEnabled;

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

    // receive a wmbus frame 
    void receive(void); // read frame from CC1101
    bool checkFrame(void);  // check id, CRC
    void getMeterInfo(uint8_t *data, size_t len);
    void publishMeterInfo();
    
  public:

    // constructor
    WaterMeter(PubSubClient &mqtt);
    
    void enableMqtt(bool enable);

    // startup CC1101 for receiving wmbus mode c 
    void begin(uint8_t *key, uint8_t *id);

    // must be called frequently
    void loop(void);

    IRAM_ATTR void instanceCC1101Isr();
};

#endif // _WATERMETER_H_
