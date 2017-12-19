#ifndef __DRV_CC1101_H__
#define __DRV_CC1101_H__

#include <Arduino.h>
#include "drv_CC1101_Reg.h"


#define TRUE  1
#define FALSE 0
const uint8_t PaTabel[] = { 0xc0, 0xC8, 0x84, 0x60, 0x68, 0x34, 0x1D, 0x0E, 0x12};//9

static const uint8_t CC1101InitData[ 47 ][ 2 ] =
{
  { CC1101_IOCFG2,      0x06 },
  { CC1101_IOCFG1,      0x2E },
  { CC1101_IOCFG0,      0x80 },
  { CC1101_FIFOTHR,     0x07 },
  { CC1101_SYNC1,       0x57 },
  { CC1101_SYNC0,       0x43 },
  { CC1101_PKTLEN,      0x3E },
  { CC1101_PKTCTRL1,    0x0E },  
  { CC1101_PKTCTRL0,    0x45 },
  { CC1101_ADDR,        0xFF }, 
  { CC1101_CHANNR,      0x00 },  
//  { CC1101_FSCTRL1,     0x08 },
  { CC1101_FSCTRL0,     0x00 },
 // { CC1101_FREQ2,       0x21 },
 // { CC1101_FREQ1,       0x65 },
//  { CC1101_FREQ0,       0x6A },
//  { CC1101_MDMCFG4,     0xF5 },
//  { CC1101_MDMCFG3,     0x83 },
 // { CC1101_MDMCFG2,     0x13 },
  { CC1101_MDMCFG1,     0xA0 },
  { CC1101_MDMCFG0,     0xF8 },
 // { CC1101_DEVIATN,     0x15 },
  { CC1101_MCSM2,       0x07 },
  { CC1101_MCSM1,       0x0C },
//  { CC1101_MCSM0,       0x19 },
//  { CC1101_FOCCFG,      0x16 },
  { CC1101_BSCFG,       0x6C },
  { CC1101_AGCCTRL2,    0x03 },
  { CC1101_AGCCTRL1,    0x40 },
  { CC1101_AGCCTRL0,    0x91 },
  { CC1101_WOREVT1,     0x02 },
  { CC1101_WOREVT0,     0x26 },
  { CC1101_WORCTRL,     0x09 },
  { CC1101_FREND1,      0x56 },
  { CC1101_FREND0,      0x17 },       
 // { CC1101_FSCAL3,      0xA9 },
 // { CC1101_FSCAL2,      0x0A },
 // { CC1101_FSCAL1,      0x00 },
//  { CC1101_FSCAL0,      0x11 },
  { CC1101_RCCTRL1,     0x41 },
  { CC1101_RCCTRL0,     0x00 },
  { CC1101_FSTEST,      0x59 },
  { CC1101_PTEST,       0x7F },
  { CC1101_AGCTEST,     0x3F },
  { CC1101_TEST2,       0x81 },
  { CC1101_TEST1,       0x3F },
  { CC1101_TEST0,       0x0B },
  

  {CC1101_FSCTRL1,     0x06},
  {CC1101_FREQ2,       0x10},
  {CC1101_FREQ1,       0xA7},
  {CC1101_FREQ0,       0x62},
  {CC1101_MDMCFG4,     0xF8},
  {CC1101_MDMCFG3,     0x93},
  {CC1101_MDMCFG2,     0x03},
  {CC1101_DEVIATN,     0x15},
  {CC1101_MCSM0,       0x18},
  {CC1101_FOCCFG,      0x16},

  {CC1101_FSCAL3,      0xE9},
  {CC1101_FSCAL2,      0x2A},
  {CC1101_FSCAL1,      0x00},
  {CC1101_FSCAL0,      0x1F},

};


/** 发送模式定义 */
enum
{
	TX_MODE_1 = 0,		//发送模式1，发送固定的字符串
	TX_MODE_2			//发送模式2，发送串口接收到的数据
};

enum
{
	SEND = 0,		//发送模式1，发送固定的字符串
	ACK			//发送模式2，发送串口接收到的数据
};

#define RSSI_OFFSET 0x4E
#define PA_TABLE 						{0xc2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,}
const int slaveSelectPin = 10;
const int CC1101_GDO0_GPIO_PIN = 8;
const int CC1101_GDO2_GPIO_PIN = 3;
const int SPISCK = 13;
const int MOSI_PIN = 11;
/** CC1101硬件接口定义 */
#define CC1101_CSN_GPIO_PIN				slaveSelectPin
#define SPI_NSS_GPIO_PIN				slaveSelectPin
                       

/** 口线操作函数定义 */
#define CC1101_SET_CSN_HIGH()			digitalWrite(SPI_NSS_GPIO_PIN, HIGH)
#define CC1101_SET_CSN_LOW()			digitalWrite(SPI_NSS_GPIO_PIN, LOW)

#define CC1101_GET_GDO0_STATUS()		digitalRead(CC1101_GDO0_GPIO_PIN)
#define CC1101_GET_GDO2_STATUS()		digitalRead(CC1101_GDO2_GPIO_PIN)

/** 枚举量定义 */
typedef enum 
{ 
	TX_MODE, 
	RX_MODE 
}CC1101_ModeType;

typedef enum 
{ 
	BROAD_ALL, 
	BROAD_NO, 
	BROAD_0, 
	BROAD_0AND255 
}CC1101_AddrModeType;

typedef enum 
{ 
	BROADCAST, 
	ADDRESS_CHECK
} CC1101_TxDataModeType;


void CC1101_Write_Cmd( uint8_t Command );
void CC1101_Write_Reg( uint8_t Addr, uint8_t WriteValue );
void CC1101_Write_Multi_Reg( uint8_t Addr, uint8_t *pWriteBuff, uint8_t WriteSize );
uint8_t CC1101_Read_Reg( uint8_t Addr );
void CC1101_Read_Multi_Reg( uint8_t Addr, uint8_t *pReadBuff, uint8_t ReadSize );
uint8_t CC1101_Read_Status( uint8_t Addr );
void CC1101_Set_Mode( CC1101_ModeType Mode );
void CC1101_Set_Idle_Mode( void );
void C1101_WOR_Init( void );
void CC1101_Set_Address( uint8_t Address, CC1101_AddrModeType AddressMode);
void CC1101_Set_Sync( uint16_t Sync );
void CC1101_Clear_TxBuffer( void );
void CC1101_Clear_RxBuffer( void );
void CC1101_Tx_Packet( uint8_t *pTxBuff, uint8_t TxSize, CC1101_TxDataModeType DataMode );
uint8_t CC1101_Get_RxCounter( void );
uint8_t CC1101_Rx_Packet( uint8_t *RxBuff );
void CC1101_Reset( void );
void CC1101_Init( void );


#endif
