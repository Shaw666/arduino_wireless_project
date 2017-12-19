#include <EEPROM.h>
#include <drv_CC1101.h>
#include <drv_CC1101_Reg.h>
#include <SPI.h>

const char *g_Ashining = "1";
//78 69 61 6F 68 75 61 6E 70 65 6E 67
uint8_t g_RF24L01RxBuffer[ 32 ] = { 0 }; 
    uint8_t* sendtest;

    
void setup() {
  uint8_t i;
  pinMode(slaveSelectPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SPISCK, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  CC1101_Init();
  for( i = 0; i < 3; i++ )    //模块初始化完成，LED灯闪烁3个周期
  {
    led_red_flashing( );
  }

}
int incomingByte;
void loop() {
//  send_Loop();
//recive_Loop();
serial_cmd_Loop();
}







int addr = 0,value=0;
void eeprom_Loop(){
  EEPROM.write(addr, addr);
  if (addr == 255)
  addr = 0;
}

void send_Loop(){
  while( 1 )  
  {
    CC1101_Tx_Packet( (uint8_t *)g_Ashining, 12 , ADDRESS_CHECK );    
    //模式1发送固定字符,1S一包
    delay(1000); 
  }
}

void recive_Loop(){
  int i = 0;
    CC1101_Clear_RxBuffer( );
    CC1101_Set_Mode( RX_MODE );
    i = CC1101_Rx_Packet( g_RF24L01RxBuffer );    //接收字节
    if( 0 != i )
    {
      if(g_RF24L01RxBuffer[0] == sendtest[0]){
//          CC1101_Tx_Packet( (uint8_t *)g_Ashining, 1 , ADDRESS_CHECK );
          i++;
          EEPROM.write(i, g_Ashining[i]);  
        }
      drv_uart_tx_bytes( g_RF24L01RxBuffer, i );  //输出接收到的字节
    }
}

void send_test(){
    int i = 0;
    for(i=0;i<1000;i++){
    sendtest[0] = i; 
    CC1101_Tx_Packet( (uint8_t *)sendtest, 1 , ADDRESS_CHECK );
      EEPROM.write(i, g_Ashining[i]);    
    //模式1发送固定字符,1S一包
    delay(200); 
  }
}

  uint8_t rec_ptr;
  uint8_t cmd_buf[8];
void serial_cmd_process(){
    if(cmd_buf[0] == 'h'){
        Serial.println("no help");
      }
    else if(cmd_buf[0] == 't'){
        Serial.println("send test");
      }
  }

void serial_cmd_Loop(){
  int rec_data;
  if(Serial.available() > 0){
  rec_data = Serial.read();
  if(rec_data == 0x0D){
      drv_uart_tx_bytes(cmd_buf, rec_ptr);
      serial_cmd_process();
      rec_ptr = 0;
    }
    else{
        cmd_buf[rec_ptr] = rec_data;
        rec_ptr++;
      }
  }
}

void drv_uart_tx_bytes( uint8_t* TxBuffer, uint8_t Length )
{
  while( Length-- )
  {
    Serial.print(*TxBuffer,HEX);
    TxBuffer++;
  }
}

/*-------------------------------------------------------------------------
 * led driver
 */
void led_red_flashing()
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second  
}






















/*--------------------------------------------------------------------------
spi driver
    receivedVal = SPI.transfer(val)
    receivedVal16 = SPI.transfer16(val16)
    SPI.transfer(buffer, size)
 */
uint8_t drv_spi_read_write_byte(uint8_t TxByte){
    return SPI.transfer(TxByte);
}

//---------------------------------------------------------------------------
//si664x driver

//10, 7, 5, 0, -5, -10, -15, -20, dbm output power, 0x12 == -30dbm
const uint8_t PaTabel[ ] = { 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0};
static const uint8_t CC1101InitData[ 22 ][ 2 ]= 
{
  { CC1101_IOCFG0,      0x06 },
  { CC1101_FIFOTHR,     0x47 },
  { CC1101_PKTCTRL0,    0x05 },
  { CC1101_CHANNR,      0x96 },  //430M
  { CC1101_FSCTRL1,     0x06 },
  { CC1101_FREQ2,       0x0F },
  { CC1101_FREQ1,       0x62 },
  { CC1101_FREQ0,       0x76 },
  { CC1101_MDMCFG4,     0xF6 },
  { CC1101_MDMCFG3,     0x43 },
  { CC1101_MDMCFG2,     0x13 },
  { CC1101_DEVIATN,     0x15 },
  { CC1101_MCSM0,       0x18 },
  { CC1101_FOCCFG,      0x16 },
  { CC1101_WORCTRL,     0xFB },
  { CC1101_FSCAL3,      0xE9 },
  { CC1101_FSCAL2,      0x2A },
  { CC1101_FSCAL1,      0x00 },
  { CC1101_FSCAL0,      0x1F },
  { CC1101_TEST2,       0x81 },
  { CC1101_TEST1,       0x35 },
  { CC1101_MCSM1,       0x3B },
};


/**
  * @brief :CC1101写命令
  * @param :
  *     @Command：命令
  * @note  :无
  * @retval:无
  */
void CC1101_Write_Cmd( uint8_t Command )
{
    CC1101_SET_CSN_LOW( );          //SPI片选，本工程中该函数都是用作SPI片选
  
    drv_spi_read_write_byte( Command );   //写命令
  
    CC1101_SET_CSN_HIGH( );         //SPI取消片选，本工程中该函数都是用作取消SPI片选          
}

/**
  * @brief :CC1101写寄存器
  * @param :
  *     @Addr：地址
  *     @WriteValue：写入的数据字节
  * @note  :无
  * @retval:无
  */
void CC1101_Write_Reg( uint8_t Addr, uint8_t WriteValue )
{
  CC1101_SET_CSN_LOW( );          
  
    drv_spi_read_write_byte( Addr );    //写地址
    drv_spi_read_write_byte( WriteValue );  //写数据
  
    CC1101_SET_CSN_HIGH( );         
}

/**
  * @brief :CC1101连续写寄存器
  * @param :
  *     @Addr：地址
  *     @pWriteBuff：写入的数据串首地址
  *     @WriteSize：写入的数据个数
  * @note  :无
  * @retval:无
  */
void CC1101_Write_Multi_Reg( uint8_t Addr, uint8_t *pWriteBuff, uint8_t WriteSize )
{
    uint8_t i;
  
    CC1101_SET_CSN_LOW( );          
  
    drv_spi_read_write_byte( Addr | WRITE_BURST );  //连续写命令 及首地址
    for( i = 0; i < WriteSize; i ++ )
    {
        drv_spi_read_write_byte( *( pWriteBuff + i ) ); //连续写入数据
    }
  
    CC1101_SET_CSN_HIGH( );         
}

/**
  * @brief :CC1101读寄存器
  * @param :
  *     @Addr：地址
  * @note  :无
  * @retval:寄存器值
  */
uint8_t CC1101_Read_Reg( uint8_t Addr )
{
    uint8_t l_RegValue = 0;
  
    CC1101_SET_CSN_LOW( );
  
    drv_spi_read_write_byte( Addr | READ_SINGLE );  //单独读命令 及地址
    l_RegValue = drv_spi_read_write_byte( 0xFF ); //读取寄存器
  
    CC1101_SET_CSN_HIGH( );
  
    return l_RegValue;
}

/**
  * @brief :CC1101读一个寄存器状态
  * @param :
  *     @Addr：地址
  * @note  :无
  * @retval:寄存器状态
  */
uint8_t CC1101_Read_Status( uint8_t Addr )
{
    uint8_t l_RegStatus = 0;
  
    CC1101_SET_CSN_LOW( );
  
    drv_spi_read_write_byte( Addr | READ_BURST ); //连续读命令 及地址
    l_RegStatus = drv_spi_read_write_byte( 0xFF );  //读取状态
  
    CC1101_SET_CSN_HIGH( );
  
    return l_RegStatus;
}

/**
  * @brief :CC1101连续读寄存器
  * @param :
  *     @Addr：地址
  *     @pReadBuff：读取数据存放首地址
  *     @ReadSize：读取数据的个数
  * @note  :无
  * @retval:无
  */
void CC1101_Read_Multi_Reg( uint8_t Addr, uint8_t *pReadBuff, uint8_t ReadSize )
{
    uint8_t i = 0, j = 0;
  
    CC1101_SET_CSN_LOW( );
  
    drv_spi_read_write_byte( Addr | READ_BURST);  //连续读命令 及首地址
    for( i = 0; i < ReadSize; i ++ )
    {
        for( j = 0; j < 20; j ++ );
        *( pReadBuff + i ) = drv_spi_read_write_byte( 0xFF ); //连续读取数据
    }
  
    CC1101_SET_CSN_HIGH( );
}

/**
  * @brief :CC1101发送接收模式设置
  * @param :
  *     @Mode：TX_MODE，发送模式 RX_MODE，接收模式
  * @note  :无
  * @retval:寄存器状态
  */
void CC1101_Set_Mode( CC1101_ModeType Mode )
{
    if( Mode == TX_MODE )     //发送模式
    {
        CC1101_Write_Reg( CC1101_IOCFG0,0x46 );
        CC1101_Write_Cmd( CC1101_STX );   
    }
    else if( Mode == RX_MODE )    //接收模式
    {
        CC1101_Write_Reg(CC1101_IOCFG0,0x46);
        CC1101_Write_Cmd( CC1101_SRX );
    }
  
  while( 0 != CC1101_GET_GDO0_STATUS( ));   //等待发送 或 接收开始
}

/**
  * @brief :CC1101进入空闲模式
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void CC1101_Set_Idle_Mode( void )
{
    CC1101_Write_Cmd( CC1101_SIDLE );
}

/**
  * @brief :CC1101初始化WOR功能
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void C1101_WOR_Init( void )
{
    CC1101_Write_Reg(CC1101_MCSM0,0x18);    
    CC1101_Write_Reg(CC1101_WORCTRL,0x78); 
    CC1101_Write_Reg(CC1101_MCSM2,0x00);
    CC1101_Write_Reg(CC1101_WOREVT1,0x8C);
    CC1101_Write_Reg(CC1101_WOREVT0,0xA0);
  CC1101_Write_Cmd( CC1101_SWORRST );   //写入WOR命令
}

/**
  * @brief :CC1101设置地址
  * @param :
  *     @Address：设置的设备地址值
  *     @AddressMode：地址检测模式
  * @note  :无
  * @retval:无
  */
void CC1101_Set_Address( uint8_t Address, CC1101_AddrModeType AddressMode)
{
    uint8_t btmp = 0;
  
  btmp = CC1101_Read_Reg( CC1101_PKTCTRL1 ) & ~0x03;  //读取CC1101_PKTCTRL1寄存器初始值
    CC1101_Write_Reg( CC1101_ADDR, Address );     //设置设备地址
  
    if( AddressMode == BROAD_ALL )     { }        //不检测地址
    else if( AddressMode == BROAD_NO  )
  { 
    btmp |= 0x01;                 //检测地址 但是不带广播
  }
    else if( AddressMode == BROAD_0   )
  { 
    btmp |= 0x02;                 //0x00为广播
  }
    else if( AddressMode == BROAD_0AND255 ) 
  {
    btmp |= 0x03;                   //0x00 0xFF为广播
  } 

  CC1101_Write_Reg( CC1101_PKTCTRL1, btmp);     //写入地址模式  
}

/**
  * @brief :CC1101设置同步字段
  * @param :无
  * @note  :无
  * @retval:无
  */
void CC1101_Set_Sync( uint16_t Sync )
{
    CC1101_Write_Reg( CC1101_SYNC1, 0xFF & ( Sync >> 8 ) );
    CC1101_Write_Reg( CC1101_SYNC0, 0xFF & Sync );  //写入同步字段 16Bit
}

/**
  * @brief :CC1101清空发送缓冲区
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void CC1101_Clear_TxBuffer( void )
{
    CC1101_Set_Idle_Mode( );          //首先进入IDLE模式
    CC1101_Write_Cmd( CC1101_SFTX );      //写入清发送缓冲区命令    
}

/**
  * @brief :CC1101清空接收缓冲区
  * @param :无
  * @note  :无
  * @retval:无
  */
void CC1101_Clear_RxBuffer( void )
{
    CC1101_Set_Idle_Mode();           //首先进入IDLE模式
    CC1101_Write_Cmd( CC1101_SFRX );      //写入清接收缓冲区命令
}

/**
  * @brief :CC1101发送数据包
  * @param :
  *     @pTxBuff：发送数据缓冲区
  *     @TxSize：发送数据长度
  *     @DataMode：数据模式
  * @note  :无
  * @retval:无
  */ 
void CC1101_Tx_Packet( uint8_t *pTxBuff, uint8_t TxSize, CC1101_TxDataModeType DataMode )
{
    uint8_t Address;
  uint16_t l_RxWaitTimeout = 0;
  
    if( DataMode == BROADCAST )             
  {
    Address = 0; 
  }
    else if( DataMode == ADDRESS_CHECK )    
  { 
    Address = CC1101_Read_Reg( CC1101_ADDR ); 
  }

    CC1101_Clear_TxBuffer( );
    
    if(( CC1101_Read_Reg( CC1101_PKTCTRL1 ) & 0x03 ) != 0 ) 
    {
        CC1101_Write_Reg( CC1101_TXFIFO, TxSize + 1 );    
        CC1101_Write_Reg( CC1101_TXFIFO, Address );     //写入长度和地址 由于多一个字节地址此时长度应该加1
    }
    else
    {
        CC1101_Write_Reg( CC1101_TXFIFO, TxSize );      //只写长度 不带地址
    }

    CC1101_Write_Multi_Reg( CC1101_TXFIFO, pTxBuff, TxSize ); //写入数据
    CC1101_Set_Mode( TX_MODE );               //发送模式
  
  while( 0 == CC1101_GET_GDO0_STATUS( ))    //等待发送完成
  {
    delay( 1 );
    if( 1000 == l_RxWaitTimeout++ )
    {
      l_RxWaitTimeout = 0;
      CC1101_Init( );
      break; 
    } 
  }
  
}

/**
  * @brief :CC1101读取接收到的字节数
  * @param :无
  * @note  :无
  * @retval:接收到的数据个数
  */
uint8_t CC1101_Get_RxCounter( void )
{
    return ( CC1101_Read_Status( CC1101_RXBYTES ) & BYTES_IN_RXFIFO );  
}

/**
  * @brief :CC1101接收数据包
  * @param :
  *     @RxBuff：发送数据缓冲区
  * @note  :无
  * @retval：接收到的字节数，0表示无数据
  */
uint8_t CC1101_Rx_Packet( uint8_t *RxBuff )
{
  uint8_t l_PktLen = 0;
    uint8_t l_Status[ 2 ] = { 0 };
  uint16_t l_RxWaitTimeout = 0;

  while( 0 == CC1101_GET_GDO0_STATUS( ))    //等待接收完成
  {
    delay( 1 );
    if( 3000 == l_RxWaitTimeout++ )
    {
      l_RxWaitTimeout = 0;
      CC1101_Init( );
      break; 
    } 
  }

    if( 0 != CC1101_Get_RxCounter( ))
    {
        l_PktLen = CC1101_Read_Reg( CC1101_RXFIFO );           // 获取长度信息
    
    if( ( CC1101_Read_Reg( CC1101_PKTCTRL1 ) & 0x03 ) != 0 )
        {
           CC1101_Read_Reg( CC1101_RXFIFO );          //如果数据包中包含地址信息 ，则读取地址信息
        }
        if( l_PktLen == 0 )           
    {
      return 0;     //无数据
    }
        else 
    {
      l_PktLen--;     //减去一个地址字节
    }
        CC1101_Read_Multi_Reg( CC1101_RXFIFO, RxBuff, l_PktLen );   //读取数据
        CC1101_Read_Multi_Reg( CC1101_RXFIFO, l_Status, 2 );    //读取数据包最后两个额外字节，后一个为CRC标志位

        CC1101_Clear_RxBuffer( );

        if( l_Status[ 1 ] & CRC_OK )
    {   
      return l_PktLen; 
    }
        else
    {   
      return 0; 
    }
    }
    else   
  {  
    return 0; 
  }                              
}

/**
  * @brief :CC1101复位
  * @param :无
  * @note  :无
  * @retval:无
  */
void CC1101_Reset( void )
{
  CC1101_SET_CSN_HIGH( );
  CC1101_SET_CSN_LOW( );
  CC1101_SET_CSN_HIGH( );
  delayMicroseconds( 40 );       
  CC1101_Write_Cmd( CC1101_SRES );
}

/**
  * @brief :CC1101引脚初始化
  * @param :无
  * @note  :无
  * @retval:无
  */ 
static void CC1101_Gpio_Init( void )
{  
  pinMode(CC1101_GDO0_GPIO_PIN,INPUT_PULLUP);
  pinMode(CC1101_GDO2_GPIO_PIN,INPUT_PULLUP);
  //GDO0 GDO2配置为上拉输入
}

/**
  * @brief :CC1101初始化
  * @param :无
  * @note  :无
  * @retval:无
  */
void CC1101_Init( void )
{
  uint8_t i = 0;

  CC1101_Gpio_Init( );    //引脚初始化
  CC1101_Reset( );        //模块复位

  for( i = 0; i < 22; i++ )
  {
    CC1101_Write_Reg( CC1101InitData[i][0], CC1101InitData[i][1] ); //写入配置参数
  }
  CC1101_Set_Address( 0x05, BROAD_0AND255 );    //写入设备地址 和地址模式
  CC1101_Set_Sync( 0x7899 );            //写入同步字段
  CC1101_Write_Reg(CC1101_MDMCFG1, 0x72 );      //调制解调器配置

  CC1101_Write_Multi_Reg( CC1101_PATABLE, (uint8_t*)PaTabel, 8 ); 
}

