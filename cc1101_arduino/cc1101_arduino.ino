#include <PinChangeInt.h>
#include <EEPROM.h>
#include <drv_CC1101.h>
#include <drv_CC1101_Reg.h>
#include <SPI.h>

#define SEND_TEST_COUNTER 1000
//10, 7, 5, 0, -5, -10, -15, -20, dbm output power, 0x12 == -30dbm
//const uint8_t PaTabel[] = { 0xc0, 0x84, 0x60, 0x68, 0x34, 0x1D, 0x0E, 0x12};//9
const char *g_Ashining = "xiaohuanpeng";
uint8_t Signal_strength = 0;//最强信号 20dbm
//Patable index: -30  -20 -15  -10   0    5    7    10 dBm
char str_Signal_strength[][8] = {"10dbm", "5dbm", "0dbm", "-5dbm", "-10dbm", "-15dbm", "-20dbm", "-30dnm"};
uint8_t receive_response_status;
//78 69 61 6F 68 75 61 6E 70 65 6E 67
uint8_t g_CC1101RxBuffer[ 32 ] = { 0 };

void setup() {
  uint16_t i;
  pinMode(slaveSelectPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  receive_response_status = SEND;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  //  CC1101_Init();
  Serial.println("init OK");
}


void loop() {
  //  send_Loop();
  //  recive_Loop();
  serial_cmd_Loop();
  //    receive_test(0);
  //    recive_test();
}

void send_Loop() {
  detachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN);
  Serial.println("send");
  CC1101_Tx_Packet( (uint8_t *)g_Ashining, 2 , ADDRESS_CHECK );
  attachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN, rf_available_int, HIGH);      //enable pin change interrupt
  //模式1发送固定字符,1S一包
  delay(1000);
}

//void recive_Loop() {
//  int i = 0;
//  if (cc1101_packet_available()) {
//    i = CC1101_Rx_Packet( g_CC1101RxBuffer );    //接收字节
//    if ( 0 != i )
//    {
//      drv_uart_tx_bytes( g_CC1101RxBuffer, i );  //输出接收到的字节
//    }
//    CC1101_Clear_RxBuffer( );
//    CC1101_Set_Mode( RX_MODE );
//  }
//}
uint8_t sendtest[2];
uint16_t response_success_count = 0;
uint16_t j, receive_count = 0, recive_eeprom_val = 0xFF;
uint16_t recive_eeprom_addr = 0;

void rf_available_int(void)
{
  detachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN);
  sei();
  Serial.println("rf_available_int");

  if (cc1101_packet_available() == TRUE) {
    receive_count = CC1101_Rx_Packet( g_CC1101RxBuffer );    //接收字节
    if ( 0 != receive_count )
    {
      j = 0;
      Serial.print(g_CC1101RxBuffer[0], DEC);  //输出接收到的字节
      Serial.println(g_CC1101RxBuffer[1], DEC);  //输出接收到的字节

      if (receive_response_status == SEND) {
        if ((g_CC1101RxBuffer[0] == sendtest[0]) && (g_CC1101RxBuffer[1] == sendtest[1])) {
          recive_eeprom_addr = ((g_CC1101RxBuffer[0] << 8) | (g_CC1101RxBuffer[1]));
          response_success_count++;
          EEPROM.write(recive_eeprom_addr, 0x00);
        }
      }
      else if (receive_response_status == ACK) {
        recive_eeprom_addr = ((g_CC1101RxBuffer[0] << 8) | (g_CC1101RxBuffer[1]));
        recive_eeprom_val = 0x00;;
        EEPROM.write(recive_eeprom_addr, recive_eeprom_val);
        CC1101_Tx_Packet(g_CC1101RxBuffer, 2 , ADDRESS_CHECK );
        Serial.print(recive_eeprom_addr);
        Serial.println("send ACK");
      }
      else {

      }
    }
  }
  CC1101_Clear_RxBuffer( );
  CC1101_Set_Mode( RX_MODE );
  attachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN, rf_available_int, HIGH);
}


void send_test(uint8_t division_strength) {
    detachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN);
  int i, j, reponse_count, response_eeprom_val = 0xFF;
  uint16_t response_eeprom_addr, response_time_out = 200;
  if (division_strength > 8) {
    division_strength = 7;
  }
  Signal_strength = division_strength;
  CC1101_Init();
  EEPROM.write(1023, Signal_strength);
  receive_response_status = SEND;
  response_success_count = 0;

  for (i = 0; i < SEND_TEST_COUNTER; i++) {
    Serial.println("send");
    sendtest[0] = (uint8_t)(i >> 8);
    sendtest[1] = i;
        detachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN);
    CC1101_Tx_Packet(sendtest, 2 , ADDRESS_CHECK );
    CC1101_Clear_RxBuffer( );
    CC1101_Set_Mode( RX_MODE );
    attachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN, rf_available_int, HIGH);
    delay(300);
  }

  Serial.println("send test OK");
  Serial.print("Signal strength:  ");  Serial.println(str_Signal_strength[Signal_strength]);
  Serial.println(PaTabel[Signal_strength], DEC);
  Serial.print("Test Packet: "); Serial.print(SEND_TEST_COUNTER, DEC);
  Serial.print("  Loss Packet: ");
  Serial.println(SEND_TEST_COUNTER - response_success_count, DEC);
}


void receive_test(uint8_t division_strength) {

  if (division_strength > 8) {
    division_strength = 7;
  }
  Signal_strength = division_strength;
  detachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN);
  CC1101_Init();
  receive_response_status = ACK;
  EEPROM.write(1023, Signal_strength);
  attachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN, rf_available_int, HIGH);
  while (1) {
    j++;
    delay(1);
    if (j % 2000 == 0) {
      detachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN);
      CC1101_Init();
      CC1101_Clear_RxBuffer( );
      CC1101_Set_Mode( RX_MODE );
      attachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN, rf_available_int, HIGH);
      Serial.println("hanve some problem ,reset modem");
    }
    if ( j == 20000 ) {
      j = 0;
      detachPinChangeInterrupt(CC1101_GDO2_GPIO_PIN);
      Serial.println("Recive Test Timeout Over,System back idle");
      break;
    }
  }
}

uint8_t rec_ptr;
uint8_t cmd_buf[8];
void serial_cmd_process() {
  if (cmd_buf[0] == 'h') {
    Serial.println("----------------------------------------------------------------");
    Serial.println("{10dbm, 5dbm, 0dbm, -5dbm, -10dbm, -15dbm, -20dbm, -30dnm}");
    Serial.println("txx t--send test,xx--single_strength=0(-30dbm)/xx");
    Serial.println("rxx r--receive test,xx--single_strength=0(-30dbm)/xx");
    Serial.println("e1 read eeprom info");
    Serial.println("e2 erease eeprom");
    Serial.println("----------------------------------------------------------------");
  }
  else if (cmd_buf[0] == 't') {
    Serial.print("send test eraseing...");
    if ((cmd_buf[1] - 0x30) < 0 || (cmd_buf[1] - 0x30) > 9 || (cmd_buf[2] - 0x30) < 0 || (cmd_buf[2] - 0x30) > 9) {
      Serial.print("Parameter Error");
      return ;
    }
    send_test(10 * (cmd_buf[1] - 0x30) + (cmd_buf[2] - 0x30));
  }
  else if (cmd_buf[0] == 'r') {
    Serial.print("recive test eraseing...");
    if ((cmd_buf[1] - 0x30) < 0 || (cmd_buf[1] - 0x30) > 9 || (cmd_buf[2] - 0x30) < 0 || (cmd_buf[2] - 0x30) > 9) {
      Serial.print("Parameter Error");
      return ;
    }
    receive_test(10 * (cmd_buf[1] - 0x30) + (cmd_buf[2] - 0x30));
  }
  else if (cmd_buf[0] == 'e') {
    if (cmd_buf[1] == '1') {
      Serial.println("eeprom readAll");
      eeprom_readAll();
    }
    else if (cmd_buf[1] == '2') {
      erase_eeprom();
    }
  }
}

uint16_t timecount = 0;
void serial_cmd_Loop() {
  uint8_t rec_data;
  if (Serial.available() > 0) {
    rec_data = Serial.read();
    cmd_buf[rec_ptr] = rec_data;
    rec_ptr++;
    if (rec_data == 0x0A) {
      Serial.write(cmd_buf, rec_ptr);
      serial_cmd_process();
      rec_ptr = 0;
    }
  } else
  {
    timecount++;
    if (timecount >= 3000) {
      timecount = 0;
      Serial.println("no command,get commad pelease input h endwith enter");
    }
    delay(1);
  }
}

void eeprom_readAll() {
  int i; uint8_t dataeeprom;
  uint16_t success_00_count = 0;
  for (i = 0; i < EEPROM.length(); i++) {
    dataeeprom = EEPROM.read(i);
    Serial.write(dataeeprom);
    if (dataeeprom == 0x00) {
      success_00_count++;
    }
  }
  Serial.print("A->B success:");
  Serial.println(success_00_count);
  if(dataeeprom<8){
  Serial.print("Signal strength(dbm): ");  Serial.println(str_Signal_strength[dataeeprom]);
  Serial.println(PaTabel[dataeeprom], DEC);
  }
}

void erase_eeprom() {
  int i;
  Serial.println("erase eeprom...");
  for (i = 0 ; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0xff);
  }
  Serial.println("erase eeprom OK");
}

void drv_uart_tx_bytes( uint8_t* TxBuffer, uint8_t Length )
{
  while ( Length-- )
  {
    Serial.write(*TxBuffer);
    TxBuffer++;
  }
}

/*-------------------------------------------------------------------------
   led driver
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
uint8_t drv_spi_read_write_byte(uint8_t TxByte) {
  return SPI.transfer(TxByte);
}

//---------------------------------------------------------------------------
//cc1101 driver

/**
    @brief :CC1101写命令
    @param :
        @Command：命令
    @note  :无
    @retval:无
*/
void CC1101_Write_Cmd( uint8_t Command )
{
  CC1101_SET_CSN_LOW( );
  while (digitalRead(MOSI_PIN) == 0); // Wait until MOSI_PIN becomes LOW
  drv_spi_read_write_byte( Command );   //写命令

  CC1101_SET_CSN_HIGH( );         //SPI取消片选，本工程中该函数都是用作取消SPI片选
}

/**
    @brief :CC1101写寄存器
    @param :
        @Addr：地址
        @WriteValue：写入的数据字节
    @note  :无
    @retval:无
*/
void CC1101_Write_Reg( uint8_t Addr, uint8_t WriteValue )
{
  CC1101_SET_CSN_LOW( );
  while (digitalRead(MOSI_PIN) == 0); //Wait until MOSI_PIN becomes LOW
  drv_spi_read_write_byte( Addr );    //写地址
  drv_spi_read_write_byte( WriteValue );  //写数据

  CC1101_SET_CSN_HIGH( );
}

/**
    @brief :CC1101连续写寄存器
    @param :
        @Addr：地址
        @pWriteBuff：写入的数据串首地址
        @WriteSize：写入的数据个数
    @note  :无
    @retval:无
*/
void CC1101_Write_Multi_Reg( uint8_t Addr, uint8_t *pWriteBuff, uint8_t WriteSize )
{
  uint8_t i;

  CC1101_SET_CSN_LOW( );
  while (digitalRead(MOSI_PIN) == 0); //Wait until MOSI_PIN becomes LOW
  drv_spi_read_write_byte( Addr | WRITE_BURST );  //连续写命令 及首地址
  for ( i = 0; i < WriteSize; i ++ )
  {
    drv_spi_read_write_byte( *( pWriteBuff + i ) ); //连续写入数据
  }

  CC1101_SET_CSN_HIGH( );
  //  delayMicroseconds( 10 );
}

/**
    @brief :CC1101读寄存器
    @param :
        @Addr：地址
    @note  :无
    @retval:寄存器值
*/
uint8_t CC1101_Read_Reg( uint8_t Addr )
{
  uint8_t l_RegValue = 0;

  CC1101_SET_CSN_LOW( );
  while (digitalRead(MOSI_PIN) == 0); // Wait until MOSI_PIN becomes LOW
  drv_spi_read_write_byte( Addr | READ_SINGLE );  //单独读命令 及地址
  l_RegValue = drv_spi_read_write_byte( 0xFF ); //读取寄存器

  CC1101_SET_CSN_HIGH( );

  return l_RegValue;
}

/**
    @brief :CC1101读一个寄存器状态
    @param :
        @Addr：地址
    @note  :无
    @retval:寄存器状态
*/
uint8_t CC1101_Read_Status( uint8_t Addr )
{
  uint8_t l_RegStatus = 0;

  CC1101_SET_CSN_LOW( );
  while (digitalRead(MOSI_PIN) == 0); //Wait until MOSI_PIN becomes LOW
  drv_spi_read_write_byte( Addr | READ_BURST ); //连续读命令 及地址
  l_RegStatus = drv_spi_read_write_byte( 0xFF );  //读取状态

  CC1101_SET_CSN_HIGH( );

  return l_RegStatus;
}

/**
    @brief :CC1101连续读寄存器
    @param :
        @Addr：地址
        @pReadBuff：读取数据存放首地址
        @ReadSize：读取数据的个数
    @note  :无
    @retval:无
*/
void CC1101_Read_Multi_Reg( uint8_t Addr, uint8_t *pReadBuff, uint8_t ReadSize )
{
  uint8_t i = 0, j = 0;

  CC1101_SET_CSN_LOW( );
  while (digitalRead(MOSI_PIN) == 0); //Wait until MOSI_PIN becomes LOW
  drv_spi_read_write_byte( Addr | READ_BURST);  //连续读命令 及首地址
  for ( i = 0; i < ReadSize; i ++ )
  {
    for ( j = 0; j < 20; j ++ );
    *( pReadBuff + i ) = drv_spi_read_write_byte( 0xFF ); //连续读取数据
  }

  CC1101_SET_CSN_HIGH( );
}

/**
    @brief :CC1101发送接收模式设置
    @param :
        @Mode：TX_MODE，发送模式 RX_MODE，接收模式
    @note  :无
    @retval:寄存器状态
*/
void CC1101_Set_Mode( CC1101_ModeType Mode )
{
  uint8_t marcstate, i;
  CC1101_Set_Idle_Mode();
  delay(1);
  marcstate = 0xFF;
  if ( Mode == TX_MODE )     //发送模式
  {
    CC1101_Write_Cmd( CC1101_STX );
    while (marcstate != 0x01)             //0x01 = ILDE after sending data
    {
      marcstate = (CC1101_Read_Reg(CC1101_MARCSTATE) & 0x1F); //read out state of cc1100 to be sure in IDLE and TX is finished
      //      Serial.println(marcstate);
    }
  }
  else if ( Mode == RX_MODE )   //接收模式
  {
    CC1101_Write_Cmd( CC1101_SRX );
    while (marcstate != 0x0D)             //0x0D = RX
    {
      CC1101_Write_Cmd( CC1101_SRX );
      delay(1);
      marcstate = (CC1101_Read_Reg(CC1101_MARCSTATE) & 0x1F); //read out state of cc1100 to be sure in RX
      //      Serial.println(marcstate);
    }
  }

}

/**
    @brief :CC1101进入空闲模式
    @param :无
    @note  :无
    @retval:无
*/
void CC1101_Set_Idle_Mode( void )
{
  CC1101_Write_Cmd( CC1101_SIDLE );
}

/**
    @brief :CC1101初始化WOR功能
    @param :无
    @note  :无
    @retval:无
*/
void C1101_WOR_Init( void )
{
  CC1101_Write_Reg(CC1101_MCSM0, 0x18);
  CC1101_Write_Reg(CC1101_WORCTRL, 0x78);
  CC1101_Write_Reg(CC1101_MCSM2, 0x00);
  CC1101_Write_Reg(CC1101_WOREVT1, 0x8C);
  CC1101_Write_Reg(CC1101_WOREVT0, 0xA0);
  CC1101_Write_Cmd( CC1101_SWORRST );   //写入WOR命令
}

/**
    @brief :CC1101设置地址
    @param :
        @Address：设置的设备地址值
        @AddressMode：地址检测模式
    @note  :无
    @retval:无
*/
void CC1101_Set_Address( uint8_t Address, CC1101_AddrModeType AddressMode)
{
  uint8_t btmp = 0;

  btmp = CC1101_Read_Reg( CC1101_PKTCTRL1 ) & ~0x03;  //读取CC1101_PKTCTRL1寄存器初始值
  CC1101_Write_Reg( CC1101_ADDR, Address );     //设置设备地址

  if ( AddressMode == BROAD_ALL )     { }       //不检测地址
  else if ( AddressMode == BROAD_NO  )
  {
    btmp |= 0x01;                 //检测地址 但是不带广播
  }
  else if ( AddressMode == BROAD_0   )
  {
    btmp |= 0x02;                 //0x00为广播
  }
  else if ( AddressMode == BROAD_0AND255 )
  {
    btmp |= 0x03;                   //0x00 0xFF为广播
  }

  CC1101_Write_Reg( CC1101_PKTCTRL1, btmp);     //写入地址模式
}

/**
    @brief :CC1101设置同步字段
    @param :无
    @note  :无
    @retval:无
*/
void CC1101_Set_Sync( uint16_t Sync )
{
  CC1101_Write_Reg( CC1101_SYNC1, 0xFF & ( Sync >> 8 ) );
  CC1101_Write_Reg( CC1101_SYNC0, 0xFF & Sync );  //写入同步字段 16Bit
}

/**
    @brief :CC1101清空发送缓冲区
    @param :无
    @note  :无
    @retval:无
*/
void CC1101_Clear_TxBuffer( void )
{
  CC1101_Set_Idle_Mode( );          //首先进入IDLE模式
  CC1101_Write_Cmd( CC1101_SFTX );      //写入清发送缓冲区命令
}

/**
    @brief :CC1101清空接收缓冲区
    @param :无
    @note  :无
    @retval:无
*/
void CC1101_Clear_RxBuffer( void )
{
  CC1101_Set_Idle_Mode();           //首先进入IDLE模式
  CC1101_Write_Cmd( CC1101_SFRX );      //写入清接收缓冲区命令
  delayMicroseconds(100);
}

/**
    @brief :CC1101发送数据包
    @param :
        @pTxBuff：发送数据缓冲区
        @TxSize：发送数据长度
        @DataMode：数据模式
    @note  :无
    @retval:无
*/
void CC1101_Tx_Packet( uint8_t *pTxBuff, uint8_t TxSize, CC1101_TxDataModeType DataMode )
{
  uint8_t Address;
  uint16_t l_RxWaitTimeout = 0;

  if ( DataMode == BROADCAST )
  {
    Address = 0;
  }
  else if ( DataMode == ADDRESS_CHECK )
  {
    Address = CC1101_Read_Reg( CC1101_ADDR );
  }
  //  Serial.println(Address);
  CC1101_Clear_TxBuffer( );

  if (( CC1101_Read_Reg( CC1101_PKTCTRL1 ) & 0x03 ) != 0 )
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
}

/**
    @brief :CC1101读取接收到的字节数
    @param :无
    @note  :无
    @retval:接收到的数据个数
*/
uint8_t CC1101_Get_RxCounter( void )
{
  uint8_t NUM_RXBYTES = 0;
  NUM_RXBYTES = CC1101_Read_Status( CC1101_RXBYTES ) & BYTES_IN_RXFIFO;
  Serial.println(NUM_RXBYTES);
  return NUM_RXBYTES;
}

/**
    @brief :CC1101接收数据包
    @param :
        @RxBuff：发送数据缓冲区
    @note  :无
    @retval：接收到的字节数，0表示无数据
*/
uint8_t CC1101_Rx_Packet( uint8_t *RxBuff )
{
  uint8_t l_PktLen = 0;
  uint8_t l_Status[ 2 ] = { 0 };
  uint16_t l_RxWaitTimeout = 0;
  int8_t RSSI_DBM;

  uint8_t bytes_in_RXFIFO = CC1101_Read_Reg(CC1101_RXBYTES);
  //  Serial.println(bytes_in_RXFIFO);//接收的个数及标识
  if ( bytes_in_RXFIFO & 0x7F && !(bytes_in_RXFIFO & 0x80))
  {
    l_PktLen = CC1101_Read_Reg( CC1101_RXFIFO );           // 获取长度信息

    if ( ( CC1101_Read_Reg( CC1101_PKTCTRL1 ) & 0x03 ) != 0 )
    {
      CC1101_Read_Reg( CC1101_RXFIFO );          //如果数据包中包含地址信息 ，则读取地址信息
    }
    if ( l_PktLen == 0 )
    {
      return 0;     //无数据
    }
    else
    {
      l_PktLen--;     //减去一个地址字节
    }
    //data address rssi lqi crc
    CC1101_Read_Multi_Reg( CC1101_RXFIFO, RxBuff, l_PktLen );   //读取数据
    CC1101_Read_Multi_Reg( CC1101_RXFIFO, l_Status, 2 );    //读取数据包最后两个额外字节，后一个为CRC标志位
    RSSI_DBM = cc1101_rssi_convert(l_Status[0]);
    Serial.print("RSSI: "); Serial.println(RSSI_DBM);
    CC1101_Clear_RxBuffer( );

    if ( l_Status[ 1 ] & CRC_OK )
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



uint8_t cc1101_packet_available()
{
  if (digitalRead(CC1101_GDO2_GPIO_PIN) == TRUE)                              //if RF package received
  {
    //Serial.println("GDO2!");
    if (CC1101_Read_Reg(CC1101_IOCFG2) == 0x06)              //if sync word detect mode is used
    {
      while (digitalRead(CC1101_GDO2_GPIO_PIN) == TRUE) {             //for sync word receive
      };
    }
    return TRUE;
  }
  return FALSE;
}
int8_t cc1101_rssi_convert(uint8_t Rssi_hex)
{
  int8_t rssi_dbm;
  int16_t Rssi_dec;
  Rssi_dec = Rssi_hex;        //convert unsigned to signed
  if (Rssi_dec >= 128) {
    rssi_dbm = ((Rssi_dec - 256) / 2) - RSSI_OFFSET;
  }
  else {
    if (Rssi_dec < 128) {
      rssi_dbm = ((Rssi_dec) / 2) - RSSI_OFFSET;
    }
  }
  return rssi_dbm;
}

void CC1100_set_output_power_level(int8_t dBm)
{
  uint8_t pa = 0x00;
  pa = dBm;
  //  if      (dBm <= -30) pa = 0x00;
  //  else if (dBm <= -20) pa = 0x01;
  //  else if (dBm <= -15) pa = 0x02;
  //  else if (dBm <= -10) pa = 0x03;
  //  else if (dBm <= 0)   pa = 0x04;
  //  else if (dBm <= 5)   pa = 0x05;
  //  else if (dBm <= 7)   pa = 0x06;
  //  else if (dBm <= 10)  pa = 0x07;

  CC1101_Write_Reg(CC1101_FREND0, pa);
}














/**
    @brief :CC1101复位
    @param :无
    @note  :无
    @retval:无
*/
void CC1101_Reset( void )
{
  CC1101_SET_CSN_LOW( );
  delayMicroseconds(10);
  CC1101_SET_CSN_HIGH( );
  delayMicroseconds( 40 );

  CC1101_Write_Cmd( CC1101_SRES );
}

/**
    @brief :CC1101引脚初始化
    @param :无
    @note  :无
    @retval:无
*/
static void CC1101_Gpio_Init( void )
{
  pinMode(CC1101_GDO0_GPIO_PIN, INPUT);
  pinMode(CC1101_GDO2_GPIO_PIN, INPUT);
  //GDO0 GDO2配置为上拉输入
}

/**
    @brief :CC1101初始化
    @param :无
    @note  :无
    @retval:无
*/
void CC1101_Init( void )
{
  uint8_t i = 0;
  uint8_t freq2, freq1, freq0;
  //433.92MHz
  freq2 = 0x10;
  freq1 = 0xB0;
  freq0 = 0x71;

  CC1101_Gpio_Init( );    //引脚初始化
  CC1101_Reset( );        //模块复位

  CC1101_Clear_RxBuffer();
  delayMicroseconds(100);
  CC1101_Clear_TxBuffer();
  delayMicroseconds(100);
  CC1101_Set_Idle_Mode();
  for ( i = 0; i < 47; i++ )
  {
    CC1101_Write_Reg( CC1101InitData[i][0], CC1101InitData[i][1] ); //写入配置参数
  }   //写入设备地址 和地址模式
  //    CC1101_Set_Sync( 0x7899 );            //写入同步字段
  //  CC1101_Write_Reg(CC1101_MDMCFG1, 0x72 );
  //  //调制解调器配置

  //  CC1101_Write_Reg(CC1101_FREQ2, freq2);
  //  CC1101_Write_Reg(CC1101_FREQ1, freq1);
  //  CC1101_Write_Reg(CC1101_FREQ0, freq0);
  CC1101_Write_Multi_Reg( CC1101_PATABLE, (uint8_t*)PaTabel, 8 );
  //Patable index: -30  -20 -15  -10   0    5    7    10 dBm
  //  CC1100_set_output_power_level(Signal_strength);
  CC1101_Write_Reg(CC1101_CHANNR, 0x01);
  CC1101_Write_Reg(CC1101_FREND0, Signal_strength);
  CC1101_Set_Address( 0x05, BROAD_0AND255 );
  CC1101_Write_Reg(CC1101_IOCFG2, 0x06); //set module in sync mode detection mode
  //  CC1101_Write_Reg(CC1101_FREND0, 0x10);

}

