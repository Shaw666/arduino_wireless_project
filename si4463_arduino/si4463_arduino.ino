#include <EEPROM.h>
#include <SPI.h>
#include <drv_SI446x.h>
#include <drv_Si446x_Config_30M.h>

#define SEND_TEST_COUNTER 1000

const char *g_Ashining = "xiaohuanpeng";
//20db 10dbm 5 0 -10 -15 -20 - 30
char str_Signal_strength[][8] = { "20dbm", "10dbm", "5dbm", "0dbm", "-10dbm", "-15dbm", "-20dbm", "-30dbm"};
uint8_t PA_TABLE[] = {127,20,10,7,4,3,2,1};
uint8_t Signal_strength = 0;//最强信号 20dbm
//78 69 61 6F 68 75 61 6E 70 65 6E 67
void setup() {
  uint16_t i;
  // set the slaveSelectPin as an output:
  pinMode(slaveSelectPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SPISCK, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);

  // initialize SPI:
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  SI446x_Init();
  //  for ( i = 0; i < 3; i++ )   //模块初始化完成，LED灯闪烁3个周期
  //  {
  //    led_red_flashing( );
  //  }
  Serial.println("init OK");
}

uint8_t g_UartRxBuffer[ 64 ] = { 0 };
uint8_t g_SI4463ItStatus[ 9 ] = { 0 };
uint8_t g_SI4463RxBuffer[ 64 ] = { 0 };
uint8_t g_SI4463ModemStatus[ 9 ] = { 0 };

void loop() {
  //        sendloop();
  //    reciveloop();
  serial_cmd_Loop();
  //  receive_test(0);
}


void sendloop() {
  while (1) {
    SI446x_Send_Packet( (uint8_t *)g_Ashining, 2, 0, 0 );
    Serial.println("send");
    delay(200);
    //    while((g_SI4463ItStatus[4]&0x20) != 0x20){
    //      SI446x_Interrupt_Status( g_SI4463ItStatus );
    //      }
  }
}

int responseOK = 0;
uint8_t sendtest[2];

void reciveloop() {
  int i;
  SI446x_Interrupt_Status( g_SI4463ItStatus );    //读中断状态
  //      drv_uart_tx_bytes(g_SI4463ItStatus, 9);

  if ((g_SI4463ItStatus[6] & 0x01) == 0x01) {
    SI446x_Get_Modem_Status(g_SI4463ModemStatus);
    //    Serial.println(g_SI4463ModemStatus[4], HEX);
  }
  //  if ((g_SI4463ItStatus[ 3 ] & 0x40) == 0x40) {
  //    SI446x_Reset_RxFifo( );
  //  }

  if ( (g_SI4463ItStatus[ 4 ] & 0x10) == 0x10)
  {
    i = SI446x_Read_Packet( g_SI4463RxBuffer );   //读FIFO数据
    if ( i != 0 )
    {
      if ((g_SI4463RxBuffer[0] == sendtest[0]) && (g_SI4463RxBuffer[1] == sendtest[1])) {
        Serial.print(g_SI4463RxBuffer[0], DEC);  //输出接收到的字节
        Serial.println(g_SI4463RxBuffer[1], DEC);  //输出接收到的字节
        responseOK = 1;
      }
    }
    SI446x_Change_Status( 6 );
    while ( 6 != SI446x_Get_Device_Status( ));
    SI446x_Start_Rx(  0, 0, PACKET_LENGTH, 0, 0, 3 );
  }
  else
  {
    if ( 3000 == i++ )
    {
      i = 0;
      SI446x_Init( );
    }
    delay( 1 );
  }
}
//------------------------------------------------------------------------------------
void send_test(uint8_t division_strength) {
  uint16_t i, response_eeprom_val = 0xFF;
  uint16_t response_eeprom_addr = 0, response_time_out = 200;
  uint16_t response_success_count = 0;

  if (division_strength > 8) {
    division_strength = 7;
  }
  Signal_strength = division_strength;
  SI446x_Init( );
  EEPROM.write(1023, Signal_strength);//将信号强度写在最末尾

  for (i =  0; i < SEND_TEST_COUNTER ; i++) {
    responseOK = 0;
    sendtest[0] = (i >> 8);
    sendtest[1] = i;
    Serial.print("send");
    SI446x_Send_Packet(sendtest, 2, 0, 0 );
    //等待响应
    SI446x_Change_Status( 3 );
    SI446x_Change_Status( 6 );
    while ( 6 != SI446x_Get_Device_Status( ));
    SI446x_Start_Rx(  0, 0, PACKET_LENGTH, 0, 0, 3 );
    response_time_out = 1000;
    while (1) {
      response_time_out--;
      reciveloop();
      delay(1);
      if (responseOK == 1) {
        response_success_count++;
        break;
      }
      if (response_time_out == 0) {
        break;
      }
    }
    response_eeprom_addr = i;

    if (responseOK == 1) {
      EEPROM.write(response_eeprom_addr, 0x00);
    }
    delay(100);
  }

  Serial.println("send test OK");
  Serial.print("Signal strength(dbm): ");  Serial.println(str_Signal_strength[Signal_strength]);
  Serial.println(PA_TABLE[Signal_strength], DEC);
  Serial.print("Test Packet: "); Serial.print(SEND_TEST_COUNTER, DEC);
  Serial.print("  Loss Packet: ");
  Serial.println(SEND_TEST_COUNTER - response_success_count, DEC);
}


void receive_test(uint8_t division_strength) {
  uint16_t i = 0, j = 0, recive_count = 0, recive_eeprom_val = 0xFF;
  uint16_t recive_eeprom_addr = 0;

  if (division_strength > 8) {
    division_strength = 7;
  }

  Signal_strength = division_strength;
  SI446x_Init( );
    Serial.print("Signal strength(dbm): ");  Serial.println(str_Signal_strength[Signal_strength]);
  EEPROM.write(1023, Signal_strength);//在最后写入信号强度
  while (1) {

    SI446x_Interrupt_Status( g_SI4463ItStatus );    //读中断状态
    //    drv_uart_tx_bytes(g_SI4463ItStatus, 9);
    if ((g_SI4463ItStatus[6] & 0x01) == 0x01) {
      //      SI446x_Get_Modem_Status(g_SI4463ModemStatus);
      //      drv_uart_tx_bytes( &g_SI4463ModemStatus[3],2);
    }
    //    if ((g_SI4463ItStatus[ 3 ] & 0x40) == 0x40) {
    //      SI446x_Reset_RxFifo( );
    //    }
    if ( (g_SI4463ItStatus[ 4 ] & 0x10) == 0x10)
    {
      recive_count = SI446x_Read_Packet( g_SI4463RxBuffer );   //读FIFO数据
      if ( recive_count != 0 )
      {
        i = 0;
        Serial.print(g_SI4463RxBuffer[0], DEC);  //输出接收到的字节
        Serial.println(g_SI4463RxBuffer[1], DEC);  //输出接收到的字节
        recive_eeprom_addr = ((g_SI4463RxBuffer[0] << 8) + (g_SI4463RxBuffer[1]));
        EEPROM.write(recive_eeprom_addr, 0x00);
        Serial.print(recive_eeprom_addr);
        SI446x_Send_Packet(g_SI4463RxBuffer, 2 , 0, 0 );
        Serial.println("ACK Send");  //输出接收到的字节
      }
      SI446x_Change_Status( 3 );
      SI446x_Change_Status( 6 );
      while ( 6 != SI446x_Get_Device_Status( )) {
//              SI446x_Change_Status( 6 );
        j++;
        if (j > 0x3fff) {
          j = 0;
          break;
        }
      }
      SI446x_Start_Rx(  0, 0, PACKET_LENGTH, 0, 0, 3 );

    }
    else
    {
      i++;
      if ( i % 1000 == 0)
      {
        SI446x_Init( );
        Serial.println("have some problem,reset modem");
      }
      if (i >= 10000)
      {
        Serial.println("Recive Test Timeout Over,System back idle");
        break;
      }
      delay( 1 );
    }

  }
}

uint8_t rec_ptr;
uint8_t cmd_buf[8];
void serial_cmd_process() {
  if (cmd_buf[0] == 'h') {
    Serial.println("----------------------------------------------------------------");
    Serial.println("{ 20dbm, 10dbm,5dbm, 0dbm, -10dbm, -15dbm, -20dbm, -30dbm}");
    Serial.println("txx t--send test,xx--single_strength=7F(20dbm)/xx");
    Serial.println("rxx r--receive test,xx--single_strength=7F(20dbm)/xx");
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
    //    Serial.println(10 * (cmd_buf[1] - 0x30) + (cmd_buf[2] - 0x30), DEC);
    send_test(10 * (cmd_buf[1] - 0x30) + (cmd_buf[2] - 0x30));
  }
  else if (cmd_buf[0] == 'r') {
    Serial.print("recive test eraseing...");
    if ((cmd_buf[1] - 0x30) < 0 || (cmd_buf[1] - 0x30) > 9 || (cmd_buf[2] - 0x30) < 0 || (cmd_buf[2] - 0x30) > 9) {
      Serial.print("Parameter Error");
      return ;
    }
    //    erase_eeprom();
    //    Serial.println(10 * (cmd_buf[1] - 0x30) + (cmd_buf[2] - 0x30), DEC);
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
      Serial.println("no command,get help pelease input h endwith enter");
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
    if ((dataeeprom == 0x00)&&(i<1000)) {
      success_00_count++;
    }
  }
  Serial.print("success receive:");
  Serial.println(success_00_count);
  if(dataeeprom<8){
  Serial.print("Signal strength(dbm): ");  Serial.println(str_Signal_strength[dataeeprom]);
  Serial.println(PA_TABLE[dataeeprom], DEC);
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
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(500);                       // wait for a second
}
/*--------------------------------------------------------------------------
  spi driver
    receivedVal = SPI.transfer(val)
    receivedVal16 = SPI.transfer16(val16)
    SPI.transfer(buffer, size)
*/
uint8_t drv_spi_read_write_byte( uint8_t TxByte )
{
  return SPI.transfer(TxByte);
}

//---------------------------------------------------------------------------
//si664x driver
/**
    @brief :SI446x等待CTS状态
    @param :无
    @note  :无
    @retval:无
*/
const static uint8_t config_table[ ] = RADIO_CONFIGURATION_DATA_ARRAY;

void SI446x_Wait_Cts( void )
{
  uint8_t l_Cts;
  uint16_t l_ReadCtsTimes = 0;

  do
  {
    SI_SET_CSN_LOW( );    //SPI片选

    //读CTS状态
    drv_spi_read_write_byte( READ_CMD_BUFF );
    l_Cts = drv_spi_read_write_byte( 0xFF );

    SI_SET_CSN_HIGH( );   //取消SPI片选

    if ( 1000 == l_ReadCtsTimes++ )
    {
      SI446x_Init( );
      break;
    }

  } while ( l_Cts != 0xFF ); //直到读CTS的返回值等于0xFF
}

/**
    @brief :SI446x写命令
    @param :
        @pCmd:命令首地址
        @CmdNumber：命令个数
    @note  :无
    @retval:无
*/
void SI446x_Write_Cmds( uint8_t *pCmd, uint8_t CmdNumber )
{
  SI446x_Wait_Cts( );     //查询CTS状态

  SI_SET_CSN_LOW( );      //SPI片选

  while ( CmdNumber -- )
  {
    drv_spi_read_write_byte( *pCmd ); //发送命令
    pCmd++;
  }

  SI_SET_CSN_HIGH( );     //取消SPI片选
}

/**
    @brief :SI446x POWER_UP
    @param :
        @Xo_Freq:晶振频率
    @note  :SI446x复位之后需要调用
    @retval:无
*/
void SI446x_Power_Up( uint32_t Xo_Freq )
{
  uint8_t l_Cmd[7] = { 0 };

  l_Cmd[0] = POWER_UP;    //Power_Up命令
  l_Cmd[1] = 0x01;
  l_Cmd[2] = 0x00;
  l_Cmd[3] = Xo_Freq >> 24;
  l_Cmd[4] = Xo_Freq >> 16;
  l_Cmd[5] = Xo_Freq >> 8;
  l_Cmd[6] = Xo_Freq;
  SI446x_Write_Cmds( l_Cmd, 7 );  //写命令
}

/**
    @brief :SI446x读CTS和命令应答
    @param :
        @pRead:返回数据首地址
        @Length:长度
    @note  :无
    @retval:无
*/
void SI446x_Read_Response( uint8_t *pRead, uint8_t Length )
{
  SI446x_Wait_Cts( );   //查询CTS状态
  SI_SET_CSN_LOW( );    //SPI片选

  drv_spi_read_write_byte( READ_CMD_BUFF ); //发送读命令
  while ( Length-- )
  {
    *pRead = drv_spi_read_write_byte( 0xFF ); //交换数据
    pRead++;
  }

  SI_SET_CSN_HIGH( );   //SPI取消片选

}

/**
    @brief :SI446x空操作
    @param :无
    @note  :无
    @retval:无
*/
uint8_t SI446x_Nop( void )
{
  uint8_t l_Cts;

  SI_SET_CSN_LOW( );    //SPI片选

  l_Cts = drv_spi_read_write_byte( NOP ); //空操作命令

  SI_SET_CSN_HIGH( );   //SPI取消片选

  return l_Cts;
}

/**
    @brief :SI446x读设备基本信息
    @param :
        @pRead:返回数据首地址
    @note  :无
    @retval:无
*/
void SI446x_Get_Part_Informatoin( uint8_t *pRead )
{
  uint8_t l_Cmd = PART_INFO;

  SI446x_Write_Cmds( &l_Cmd, 1 );   //命令
  SI446x_Read_Response( pRead, 8 ); //读设备基本信息

}

/**
    @brief :SI446x读设备功能版本信息
    @param :
        @pRead:返回数据首地址
    @note  :无
    @retval:无
*/
void SI446x_Get_Fun_Informatoin( uint8_t *pRead )
{
  uint8_t l_Cmd = FUNC_INFO;

  SI446x_Write_Cmds( &l_Cmd, 1 );   //命令
  SI446x_Read_Response( pRead, 7 ); //读设备功能版本信息
}

/**
    @brief :SI446x读中断状态
    @param :
        @pRead:返回数据首地址
    @note  :无
    @retval:无
*/
void SI446x_Interrupt_Status( uint8_t *pRead )
{
  uint8_t l_Cmd[ 4 ] = { 0 };

  l_Cmd[0] = GET_INT_STATUS;
  l_Cmd[1] = 0;
  l_Cmd[2] = 0;
  l_Cmd[3] = 0;

  SI446x_Write_Cmds( l_Cmd, 4 );    //发送中断读取命令
  SI446x_Read_Response( pRead, 9 ); //读取状态
}

/**
    @brief :SI446x读取属性值
    @param :
        @Group_Num:属性组(参考SI446X_PROPERTY)
        @Num_Props:读取的属性个数
        @pRead:返回数据首地址
    @note  :无
    @retval:无
*/
void SI446x_Get_Property( SI446X_PROPERTY Group_Num, uint8_t Num_Props, uint8_t *pRead  )
{
  uint8_t l_Cmd[ 4 ] = { 0 };

  l_Cmd[ 0 ] = GET_PROPERTY;
  l_Cmd[ 1 ] = Group_Num >> 8;
  l_Cmd[ 2 ] = Num_Props;
  l_Cmd[ 3 ] = Group_Num;

  SI446x_Write_Cmds( l_Cmd, 4 );    //发送读取属性命令
  SI446x_Read_Response( pRead, Num_Props + 1 ); //读属性
}

/**
    @brief :SI446x设置属性
    @param :
        @Group_Num:属性组(参考SI446X_PROPERTY)
        @Num_Props:设置的属性个数
        @pWrite:写地址设备
    @note  :无
    @retval:无
*/
void SI446x_Set_Property( SI446X_PROPERTY Group_Num, uint8_t Num_Props, uint8_t *pWrite )
{
  uint8_t l_Cmd[ 20 ] = { 0 }, i = 0;

  if ( Num_Props >= 16 )
  {
    return;   //数量不大于16
  }

  l_Cmd[ i++ ] = SET_PROPERTY;    //设置属性命令
  l_Cmd[ i++ ] = Group_Num >> 8;
  l_Cmd[ i++ ] = Num_Props;
  l_Cmd[ i++ ] = Group_Num;

  while ( Num_Props-- )
  {
    l_Cmd[ i++ ] = *pWrite;
    pWrite++;
  }
  SI446x_Write_Cmds( l_Cmd, i );    //发送命令及数据
}

/**
    @brief :SI446x设置属性组1属性
    @param :
        @Group_Num:属性组
        @Start_Prop:开始设置的属性号(参考SI446X_PROPERTY)
    @note  :无
    @retval:无
*/
void SI446x_Set_Property_1( SI446X_PROPERTY Group_Num, uint8_t Start_Prop )
{
  uint8_t l_Cmd[ 5 ] = { 0 };

  l_Cmd[ 0 ] = SET_PROPERTY;    //命令
  l_Cmd[ 1 ] = Group_Num >> 8;
  l_Cmd[ 2 ] = 1;
  l_Cmd[ 3 ] = Group_Num;
  l_Cmd[ 4 ] = Start_Prop;

  SI446x_Write_Cmds( l_Cmd, 5 );  //发送命令设置属性
}

/**
    @brief :SI446x读取属性组1属性
    @param :
        @Group_Num:开始的属性号(参考SI446X_PROPERTY)
    @note  :无
    @retval:无
*/
uint8_t SI446x_Get_Property_1( SI446X_PROPERTY Group_Num )
{
  uint8_t l_Cmd[ 4 ] = { 0 };

  l_Cmd[ 0 ] = GET_PROPERTY;
  l_Cmd[ 1 ] = Group_Num >> 8;
  l_Cmd[ 2 ] = 1;
  l_Cmd[ 3 ] = Group_Num;
  SI446x_Write_Cmds( l_Cmd, 4 );    //发送命令

  SI446x_Read_Response( l_Cmd, 2 ); //读取属性

  return l_Cmd[ 1 ];
}

/**
    @brief :SI446x复位
    @param :无
    @note  :无
    @retval:无
*/
void SI446x_Reset( void )
{
  SI_SET_SDN_HIGH( );   //关设备
  delayMicroseconds(20);   //延时 等待设备完全断电
  SI_SET_SDN_LOW( );    //开设备
  SI_SET_CSN_HIGH( );   //取消SPI片选
  delay(15);
}

/**
    @brief :SI446x配置GPIO
    @param :无
    @note  :无
    @retval:无
*/
void SI446x_Config_Gpio( uint8_t Gpio_0, uint8_t Gpio_1, uint8_t Gpio_2, uint8_t Gpio_3, uint8_t Irq, uint8_t Sdo, uint8_t Gen_Config )
{
  uint8_t l_Cmd[ 10] = { 0 };
  // SI446x_Config_Gpio( 0, 0, 33 | 0x40, 32 | 0x40, 0, 0, 0 ); //4463才需要配置
  l_Cmd[ 0 ] = GPIO_PIN_CFG;
  l_Cmd[ 1 ] = Gpio_0;
  l_Cmd[ 2 ] = Gpio_1;
  l_Cmd[ 3 ] = Gpio_2;
  l_Cmd[ 4 ] = Gpio_3;
  l_Cmd[ 5 ] = Irq;
  l_Cmd[ 6 ] = Sdo;
  l_Cmd[ 7 ] = Gen_Config;

  SI446x_Write_Cmds( l_Cmd, 8 );    //写配置
  SI446x_Read_Response( l_Cmd, 8 ); //读配置
}

/**
    @brief :SI446x模块配置
    @param :无
    @note  :无
    @retval:无
*/
void SI446x_Config_Init( void )
{
  uint8_t i;
  uint16_t j = 0;
  uint8_t retopt[3] = {0x01, 0xFF, 0x40};

  while ( ( i = config_table[j] ) != 0 )
  {
    j += 1;
    SI446x_Write_Cmds( (uint8_t *)config_table + j, i );
    j += i;
  }

  //设置匹配字 一个 地址为0x01

#if PACKET_LENGTH > 0           //固定数据长度

  SI446x_Set_Property_1( PKT_FIELD_1_LENGTH_7_0, PACKET_LENGTH );

#else                           //动态数据长度

  SI446x_Set_Property_1( PKT_CONFIG1, 0x00 );
  SI446x_Set_Property_1( PKT_CRC_CONFIG, 0x00 );
  SI446x_Set_Property_1( PKT_LEN_FIELD_SOURCE, 0x01 );
  SI446x_Set_Property_1( PKT_LEN, 0x2A );
  SI446x_Set_Property_1( PKT_LEN_ADJUST, 0x00 );
  SI446x_Set_Property_1( PKT_FIELD_1_LENGTH_12_8, 0x00 );
  SI446x_Set_Property_1( PKT_FIELD_1_LENGTH_7_0, 0x01 );
  SI446x_Set_Property_1( PKT_FIELD_1_CONFIG, 0x00 );
  SI446x_Set_Property_1( PKT_FIELD_1_CRC_CONFIG, 0x00 );
  SI446x_Set_Property_1( PKT_FIELD_2_LENGTH_12_8, 0x00 );
  SI446x_Set_Property_1( PKT_FIELD_2_LENGTH_7_0, 0x20 );
  SI446x_Set_Property_1( PKT_FIELD_2_CONFIG, 0x00 );
  SI446x_Set_Property_1( PKT_FIELD_2_CRC_CONFIG, 0x00 );

#endif
  //4463 的GDO2 GDO3控制射频开关 33 32
  // 发射：GDO2 = 0, GDO3 = 1
  // 接收：GDO2 = 1, GDO3 = 0
  SI446x_Config_Gpio( 0, 0, 33 | 0x40, 32 | 0x40, 0, 0, 0 ); //4463才需要配置
}

/**
    @brief :SI446x写TX FIFO
    @param :
        @pWriteData：写数据首地址
        @Length：数据长度
    @note  :无
    @retval:无
*/
void SI446x_Write_TxFifo( uint8_t *pWriteData, uint8_t Length )
{
  SI_SET_CSN_LOW( );
  drv_spi_read_write_byte( WRITE_TX_FIFO );   //写命令
  while ( Length-- )
  {
    drv_spi_read_write_byte( *pWriteData++ );   //写数据
  }
  SI_SET_CSN_HIGH( );
}

/**
    @brief :SI446x 复位RX FIFO
    @param :无
    @note  :无
    @retval:无
*/
void SI446x_Reset_RxFifo( void )
{
  uint8_t l_Cmd[ 2 ] = { 0 };

  l_Cmd[ 0 ] = FIFO_INFO;
  l_Cmd[ 1 ] = 0x02;
  SI446x_Write_Cmds( l_Cmd, 2 );
}

/**
    @brief :SI446x 复位TX FIFO
    @param :无
    @note  :无
    @retval:无
*/
void SI446x_Reset_TxFifo( void )
{
  uint8_t l_Cmd[ 2 ] = { 0 };

  l_Cmd[0] = FIFO_INFO;
  l_Cmd[1] = 0x02;
  SI446x_Write_Cmds( l_Cmd, 2 );
}

/**
    @brief :SI446x发送数据包
    @param :
        @pTxData：发送数据首地址
        @Length：数据长度
        @Channel：通道
        @Condition：发送状况选择
    @note  :无
    @retval:无
*/
void SI446x_Send_Packet( uint8_t *pTxData, uint8_t Length, uint8_t Channel, uint8_t Condition )
{
  uint8_t l_Cmd[ 5 ] = { 0 };
  uint8_t tx_len = Length;

  SI446x_Reset_TxFifo( );   //清空TX FIFO

  SI_SET_CSN_LOW( );

  drv_spi_read_write_byte( WRITE_TX_FIFO ); //写TX FIFO命令

#if PACKET_LENGTH == 0      //动态数据长度

  tx_len ++;
  drv_spi_read_write_byte( Length );

#endif

  while ( Length-- )
  {
    drv_spi_read_write_byte( *pTxData++ );  //写数据到TX FIFO
  }

  SI_SET_CSN_HIGH( );

  l_Cmd[ 0 ] = START_TX;
  l_Cmd[ 1 ] = Channel;
  l_Cmd[ 2 ] = Condition;
  l_Cmd[ 3 ] = 0;
  l_Cmd[ 4 ] = tx_len;

  SI446x_Write_Cmds( l_Cmd, 5 );    //发送数据包
  while (SI_GET_IRQ_STATUS());
}

/**
    @brief :SI446x启动发送
    @param :
        @Length：数据长度
        @Channel：通道
        @Condition：发送状况选择
    @note  :无
    @retval:无
*/
void SI446x_Start_Tx( uint8_t Channel, uint8_t Condition, uint16_t Length )
{
  uint8_t l_Cmd[5] = { 0 };

  l_Cmd[ 0 ] = START_TX;
  l_Cmd[ 1 ] = Channel;
  l_Cmd[ 2 ] = Condition;
  l_Cmd[ 3 ] = Length >> 8;
  l_Cmd[ 4 ] = Length;

  SI446x_Write_Cmds( l_Cmd, 5 );
}

/**
    @brief :SI446x读RX FIFO数据
    @param :
        @pRxData：数据首地址
    @note  :无
    @retval:数据个数
*/
uint8_t SI446x_Read_Packet( uint8_t *pRxData )
{
  uint8_t length = 0, i = 0;

  SI446x_Wait_Cts( );
  SI_SET_CSN_LOW( );

  drv_spi_read_write_byte( READ_RX_FIFO );  //读FIFO命令

#if PACKET_LENGTH == 0

  length = drv_spi_read_write_byte( 0xFF ); //读数据长度

#else

  length = PACKET_LENGTH;

#endif
  i = length;

  while ( length -- )
  {
    *pRxData++ = drv_spi_read_write_byte( 0xFF ); //读数据
  }

  SI_SET_CSN_HIGH( );   //返回数据个数

  return i;
}

/**
    @brief :SI446x启动接收
    @param :
        @Channel：通道
        @Condition：开始接收状态
        @Length：接收长度
        @Next_State1：下一个状态1
        @Next_State2：下一个状态2
        @Next_State3：下一个状态3
    @note  :无
    @retval:无
*/
void SI446x_Start_Rx( uint8_t Channel, uint8_t Condition, uint16_t Length, uint8_t Next_State1, uint8_t Next_State2, uint8_t Next_State3 )
{
  uint8_t l_Cmd[ 8 ] = { 0 };

  SI446x_Reset_RxFifo( );
  SI446x_Reset_TxFifo( );

  l_Cmd[ 0 ] = START_RX;
  l_Cmd[ 1 ] = Channel;
  l_Cmd[ 2 ] = Condition;
  l_Cmd[ 3 ] = Length >> 8;
  l_Cmd[ 4 ] = Length;
  l_Cmd[ 5 ] = Next_State1;
  l_Cmd[ 6 ] = Next_State2;
  l_Cmd[ 7 ] = Next_State3;

  SI446x_Write_Cmds( l_Cmd, 8 );
}

/**
    @brief :SI446x读取当前数据包信息
    @param :
        @pReadData：数据存放地址
        @FieldNumMask：掩码域
        @Length：长度
        @DiffLen：不同长度
    @note  :无
    @retval:无
*/
void SI446x_Get_Packet_Information( uint8_t *pReadData, uint8_t FieldNumMask, uint16_t Length, uint16_t DiffLen )
{
  uint8_t l_Cmd[ 6 ] = { 0 };

  l_Cmd[ 0 ] = PACKET_INFO;
  l_Cmd[ 1 ] = FieldNumMask;
  l_Cmd[ 2 ] = Length >> 8;
  l_Cmd[ 3 ] = Length;
  l_Cmd[ 4 ] = DiffLen >> 8;
  l_Cmd[ 5 ] = DiffLen;

  SI446x_Write_Cmds( l_Cmd, 6 );
  SI446x_Read_Response( pReadData, 3 );
}

/**
    @brief :SI446x读取FIFO状态
    @param :
        @pReadData：数据存放地址
    @note  :无
    @retval:无
*/
void SI446x_Get_Fifo_Information( uint8_t *pReadData )
{
  uint8_t l_Cmd[ 2 ] = { 0 };

  l_Cmd[ 0 ] = FIFO_INFO;
  l_Cmd[ 1 ] = 0x03;

  SI446x_Write_Cmds( l_Cmd, 2 );
  SI446x_Read_Response( pReadData, 3 );
}

/**
    @brief :SI446x状态切换
    @param :
        @NextStatus：下一个状态
    @note  :无
    @retval:无
*/
void SI446x_Change_Status( uint8_t NextStatus )
{
  uint8_t l_Cmd[ 2 ] = { 0 };

  l_Cmd[ 0 ] = CHANGE_STATE;
  l_Cmd[ 1 ] = NextStatus;

  SI446x_Write_Cmds( l_Cmd, 2 );
}

/**
    @brief :SI446x获取设备当前状态
    @param :
    @note  :无
    @retval:设备当前状态
*/
uint8_t SI446x_Get_Device_Status( void )
{
  uint8_t l_Cmd[ 3 ] = { 0 };

  l_Cmd [ 0 ] = REQUEST_DEVICE_STATE;

  SI446x_Write_Cmds( l_Cmd, 1 );
  SI446x_Read_Response( l_Cmd, 3 );

  return l_Cmd[ 1 ] & 0x0F;
}

/**
    @brief :SI446x功率设置
    @param :
        @PowerLevel：数据存放地址
    @note  :无
    @retval:设备当前状态
*/
void SI446x_Set_Power( uint8_t PowerLevel )
{
  SI446x_Set_Property_1( PA_PWR_LVL, PowerLevel );
}

/**
    @brief :SI446x引脚初始化
    @param :无
    @note  :无
    @retval:无
*/
void SI446x_Gpio_Init( void )
{
  pinMode(SI4463_SDN_PIN, OUTPUT);
  pinMode(SI4463_IRQ_PIN, INPUT);
  digitalWrite(SI4463_SDN_PIN, LOW);
}

/**
    @brief :SI446x快速跳频
    @param :
         @PowerLevel：数据存放地址
    @note  :无
    @retval:设备当前状态
*/
void SI446x_Fast_RX_Hop( void )
{
  uint8_t l_Cmd[ 7 ] = { 0 };

  l_Cmd[ 0 ] = RX_HOP;
  l_Cmd[ 1 ] = INTE;
  l_Cmd[ 2 ] = FRAC2;
  l_Cmd[ 3 ] = FRAC1;
  l_Cmd[ 4 ] = FRAC0;
  l_Cmd[ 5 ] = VCO_CNT1;
  l_Cmd[ 6 ] = VCO_CNT0;

  SI446x_Write_Cmds( l_Cmd, 7 );
  drv_spi_read_write_byte( 0xFF );//响应CTS
}
/**
    @brief :SI446x获取SI446x当前modem status
    @param :
    @note  :无
    @retval:当前状态
*/
void SI446x_Get_Modem_Status(uint8_t *pRead )
{
  uint8_t l_Cmd[ 1 ] = { 0 };

  l_Cmd [ 0 ] = GET_MODEM_STATUS;

  SI446x_Write_Cmds( l_Cmd, 1 );
  SI446x_Read_Response( pRead, 9 );
}
/**
    @brief :SI446x初始化
    @param :无
    @note  :无
    @retval:无
*/
// send 1  recive 0
uint8_t is_send_or_recive_complete(uint8_t whichStatus) {
  SI446x_Interrupt_Status( g_SI4463ItStatus );
  Serial.write(g_SI4463ItStatus, 9);
  if (whichStatus) {
    if ((g_SI4463ItStatus[4] & 0x20) == 0x20) {
      return 1;
    }
  } else {
    if ((g_SI4463ItStatus[4] & 0x10) == 0x10)
      return 1;
  }
  return 0;
}
void SI446x_Init( void )
{
  uint8_t bufftemp[8];

  SI446x_Gpio_Init( );    //SI4463引脚初始化
  SI446x_Reset( );      //SI4463复位
  SI446x_Power_Up( 30000000 );//reset 后需要Power up设备 晶振30MHz
  SI446x_Config_Init( );    //SI4463模块初始化
  SI446x_Set_Power( PA_TABLE[Signal_strength] ); //功率设置
  SI446x_Change_Status( 6 );  //切换到RX状态
  SI446x_Get_Part_Informatoin(bufftemp);
  while ( 6 != SI446x_Get_Device_Status( ));
  SI446x_Start_Rx( 0, 0, PACKET_LENGTH, 0, 0, 3 );

}

