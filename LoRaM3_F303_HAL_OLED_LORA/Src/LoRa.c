#include <LoRa.h>

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80
#define RFO                      0x70
// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define REG_FDEVMSB                                 0x04
#define REG_FDEVLSB                                 0x05
#define REG_OPMODE                                  0x01
#define REG_BITRATEMSB                              0x02
#define REG_BITRATELSB                              0x03
#define REG_PREAMBLEMSB                             0x25
#define REG_PREAMBLELSB                             0x26
#define REG_DIOMAPPING1                             0x40
#define REG_DIOMAPPING2                             0x41
#define REG_PACKETCONFIG1                           0x30
#define REG_PACKETCONFIG2                           0x31
#define RF_PACKETCONFIG1_CRC_MASK                   0xEF
#define RF_PACKETCONFIG1_CRC_ON                     0x10  // Default
#define RF_PACKETCONFIG1_CRC_OFF                    0x00
#define RF_PACKETCONFIG1_PACKETFORMAT_MASK          0x7F
#define RF_PACKETCONFIG1_PACKETFORMAT_FIXED         0x00
#define RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE      0x80  // Default
#define RF_DIOMAPPING1_DIO0_10                      0x80
#define RF_DIOMAPPING1_DIO0_11                      0xC0
#define RF_DIOMAPPING2_DIO5_10                      0x20
#define RF_DIOMAPPING2_DIO5_11                      0x30
#define RF_DIOMAPPING2_DIO4_10                      0x80
#define RF_DIOMAPPING2_DIO4_11                      0xC0
#define RF_DIOMAPPING1_DIO1_11                      0x30
#define RF_OPMODE_TRANSMITTER                       0x03
#define RF_PACKETCONFIG2_DATAMODE_MASK              0xBF
#define RF_PACKETCONFIG2_DATAMODE_PACKET            0x40  // Default
#define RFLR_OPMODE_LONGRANGEMODE_MASK 				0x7F
#define RFLR_OPMODE_LONGRANGEMODE_ON 				0x80
#define RF_OPMODE_SLEEP                             0x00
#define RF_OPMODE_MASK                              0xF8
#define RFLR_OPMODE_LONGRANGEMODE_OFF 				0x00
#define RF_DIOMAPPING1_DIO0_MASK                    0x3F
#define RF_DIOMAPPING2_MAP_MASK                     0xFE
#define RF_DIOMAPPING2_DIO4_MASK                    0x3F
#define RF_DIOMAPPING1_DIO1_01                      0x10
#define RF_DIOMAPPING1_DIO2_MASK                    0xF3
#define RF_DIOMAPPING1_DIO1_MASK                    0xCF


#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625
#define RX_BUFFER_SIZE                              256
#define RadioModems_t int
#define MODEM_FSK     1
#define MODEM_LORA    2
#define MAX_PKT_LENGTH           255
extern SPI_HandleTypeDef hspi1;
unsigned short  SPI1_Tx_Data  = 0xD000;
unsigned short  SPI1_Rx_Data  = 0;

// --------HAL--------
#define  OUTPUT  	5
#define  LOW  		5
#define  HIGH  		5
#define  RISING 	5
#define  delay       HAL_Delay
void pinMode(int a,int b){

}
void digitalWrite(int a,int b){

}
void SPI_begin(void){

}
void SPI_end(void){

}
void LoRa_attachInterrupt(int a, void* b,int c){

}
void LoRa_detachInterrupt(int a){

}
//void SPI_beginTransaction(){
//
//}
//void SPI_endTransaction(){
//
//}
//uint8_t SPI_transfer(uint8_t value){
//	SPI1_Tx_Data=value;
//	HAL_SPI_TransmitReceive_IT(&hspi1, (unsigned char *) &SPI1_Tx_Data, (unsigned char *) &SPI1_Rx_Data, 1);
//	return SPI1_Rx_Data;
//}
int digitalPinToInterrupt(int a){
	return 5;
}
// --------HAL--------
void sx1276SetTx(){

	LoRa_writeRegister( REG_DIOMAPPING1, ( LoRa_readRegister( REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                           RF_DIOMAPPING1_DIO1_MASK &
                                                                           RF_DIOMAPPING1_DIO2_MASK ) |
                                                                           RF_DIOMAPPING1_DIO1_01 );
    LoRa_writeRegister( REG_DIOMAPPING2, ( LoRa_readRegister( REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                    RF_DIOMAPPING2_MAP_MASK ) );
    //SX1276.Settings.FskPacketHandler.FifoThresh = SX1276Read( REG_FIFOTHRESH ) & 0x3F;
    //SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
    //while((LoRa_readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);
}
int LoRa_begin(long frequency,bool PABOOST){

  // setup pins
  pinMode(_ss, OUTPUT);
  pinMode(_reset, OUTPUT);
  // perform reset
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //digitalWrite(_reset, HIGH);
  delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); //digitalWrite(_reset, LOW);
  delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //digitalWrite(_reset, HIGH);
  delay(10);
  // set SS high
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //digitalWrite(_ss, HIGH);
  // start SPI
  SPI_begin();
  // check version
  uint8_t version = LoRa_readRegister(REG_VERSION);
  if (version != 0x12) { return 0; }
  // put in sleep mode
  LoRa_sleep();
  // set frequency
  LoRa_setFrequency(frequency);
  // set base addresses
  LoRa_writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  LoRa_writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
  // set LNA boost
  LoRa_writeRegister(REG_LNA, LoRa_readRegister(REG_LNA) | 0x03);
  // set auto AGC
  LoRa_writeRegister(REG_MODEM_CONFIG_3, 0x04);
  // set output power to 17 dBm
  LoRa_setTxPower(17,PABOOST);  //rfo
  // put in standby mode
  LoRa_idle();
  return 1;
}
void LoRa_end()
{
  // put in sleep mode
	LoRa_sleep();
  // stop SPI
  SPI_end();
}
int LoRa_beginPacket(int implicitHeader)
{
	//default implicitHeader = false
  // put in standby mode
	LoRa_idle();
  if (implicitHeader) {
	  LoRa_implicitHeaderMode();
  } else {
	  LoRa_explicitHeaderMode();
  }
  // reset FIFO address and paload length
  LoRa_writeRegister(REG_FIFO_ADDR_PTR, 0);
  LoRa_writeRegister(REG_PAYLOAD_LENGTH, 0);
  return 1;
}
int LoRa_endPacket()
{
  // put in TX mode
  LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  // wait for TX done
  while((LoRa_readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);
  // clear IRQ's
  LoRa_writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  return 1;
}
int parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = LoRa_readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
	  LoRa_implicitHeaderMode();
    LoRa_writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
	  LoRa_explicitHeaderMode();
  }

  // clear IRQ's
  LoRa_writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;
    // read packet length
    if (_implicitHeaderMode) {
      packetLength = LoRa_readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = LoRa_readRegister(REG_RX_NB_BYTES);
    }
    // set FIFO address to current RX address
    LoRa_writeRegister(REG_FIFO_ADDR_PTR, LoRa_readRegister(REG_FIFO_RX_CURRENT_ADDR));
    // put in standby mode
    LoRa_idle();
  }
  else if (LoRa_readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode
    // reset FIFO address
    LoRa_writeRegister(REG_FIFO_ADDR_PTR, 0);
    // put in single RX mode
    LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }
  return packetLength;
}
int packetRssi()
{
  return (LoRa_readRegister(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}
float LoRa_packetSnr()
{
  return ((int8_t)LoRa_readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}
size_t LoRa_writebfr(const uint8_t *buffer, size_t size)
{
  int currentLength = LoRa_readRegister(REG_PAYLOAD_LENGTH);
  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }
  // write data
  for (size_t i = 0; i < size; i++) {
    LoRa_writeRegister(REG_FIFO, buffer[i]);
  }
  // update length
  LoRa_writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
  return size;
}
size_t LoRa_write(uint8_t byte)
{
  return writebfr(&byte, sizeof(byte));
}
size_t LoRa_print(char* str){

	return LoRa_writebfr(&str, sizeof(str));
}
int LoRa_available()
{
  return (LoRa_readRegister(REG_RX_NB_BYTES) - _packetIndex);
}
int LoRa_read()
{
  if (!available()) { return -1; }
  _packetIndex++;
  return LoRa_readRegister(REG_FIFO);
}
int LoRa_peek()
{
  if (!available()) { return -1; }
  // store current FIFO address
  int currentAddress = LoRa_readRegister(REG_FIFO_ADDR_PTR);
  // read
  uint8_t b = LoRa_readRegister(REG_FIFO);
  // restore FIFO address
  LoRa_writeRegister(REG_FIFO_ADDR_PTR, currentAddress);
  return b;
}
void LoRa_flush()
{
}
void LoRa_onReceive(void(*callback)(int)){

  _onReceive = callback;

  if (callback) {
    LoRa_writeRegister(REG_DIO_MAPPING_1, 0x00);
    LoRa_attachInterrupt(digitalPinToInterrupt(_dio0), LoRa_onDio0Rise, RISING);
  } else {
	LoRa_detachInterrupt(digitalPinToInterrupt(_dio0));
  }
}
void LoRa_receive(int size)
{
  if (size > 0) {
	  LoRa_implicitHeaderMode();
    LoRa_writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
	  LoRa_explicitHeaderMode();
  }

  LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}
void LoRa_idle()
{
  LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}
void LoRa_sleep()
{
  LoRa_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}
void LoRa_setTxPower(int level, int outputPin)
{
	// default  = PA_OUTPUT_PA_BOOST_PIN
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0)      { level = 0; }
    else if (level > 14){ level = 14; }
    LoRa_writeRegister(REG_PA_CONFIG, RFO | (level + 1));
    //spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | (power + 1));
  } else {
    // PA BOOST
    if (level < 2) { level = 2; }
    //else if (level > 17) { level = 17; }
    LoRa_writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}
void LoRa_setFrequency(long frequency){

  _frequency = frequency;
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000+70;
  LoRa_writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  LoRa_writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  LoRa_writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}
void LoRa_setSpreadingFactor(int sf)
{
  if (sf < 6) { sf = 6; }
  else if (sf > 12) { sf = 12; }
  if (sf == 6) {
    LoRa_writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    LoRa_writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    LoRa_writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    LoRa_writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }
  LoRa_writeRegister(REG_MODEM_CONFIG_2, (LoRa_readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}
void LoRa_setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) { bw = 0; }
  else if (sbw <= 10.4E3) { bw = 1; }
  else if (sbw <= 15.6E3) { bw = 2; }
  else if (sbw <= 20.8E3) { bw = 3; }
  else if (sbw <= 31.25E3) { bw = 4; }
  else if (sbw <= 41.7E3) { bw = 5; }
  else if (sbw <= 62.5E3) { bw = 6; }
  else if (sbw <= 125E3) { bw = 7; }
  else if (sbw <= 250E3) { bw = 8; }
  else /*if (sbw <= 250E3)*/ { bw = 9; }
  LoRa_writeRegister(REG_MODEM_CONFIG_1,(LoRa_readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}
void LoRa_setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }
  int cr = denominator - 4;
  LoRa_writeRegister(REG_MODEM_CONFIG_1, (LoRa_readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}
void LoRa_setPreambleLength(long length)
{
  LoRa_writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  LoRa_writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}
void LoRa_setSyncWord(int sw)
{
  LoRa_writeRegister(REG_SYNC_WORD, sw);
}
void LoRa_enableCrc()
{
  LoRa_writeRegister(REG_MODEM_CONFIG_2, LoRa_readRegister(REG_MODEM_CONFIG_2) | 0x04);
}
void LoRa_disableCrc()
{
  LoRa_writeRegister(REG_MODEM_CONFIG_2, LoRa_readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}
uint8_t LoRa_random()
{
  return LoRa_readRegister(REG_RSSI_WIDEBAND);
}
void LoRa_setPins(int ss, int reset, int dio0)
{
	//int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}
void LoRa_setSPIFrequency(uint32_t frequency)
{
  //_spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}
//void dumpRegisters(Stream& out)
//{
//  for (int i = 0; i < 128; i++) {
//    out.print("0x");
//    out.print(i, HEX);
//    out.print(": 0x");
//    out.println(LoRa_readRegister(i), HEX);
//  }
//}
void LoRa_explicitHeaderMode()
{
  _implicitHeaderMode = 0;
  LoRa_writeRegister(REG_MODEM_CONFIG_1, LoRa_readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}
void LoRa_implicitHeaderMode()
{
  _implicitHeaderMode = 1;
  LoRa_writeRegister(REG_MODEM_CONFIG_1, LoRa_readRegister(REG_MODEM_CONFIG_1) | 0x01);
}
void LoRa_handleDio0Rise(){

  int irqFlags = LoRa_readRegister(REG_IRQ_FLAGS);
  // clear IRQ's
  LoRa_writeRegister(REG_IRQ_FLAGS, irqFlags);
  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;
    // read packet length
    int packetLength = _implicitHeaderMode ? LoRa_readRegister(REG_PAYLOAD_LENGTH) : LoRa_readRegister(REG_RX_NB_BYTES);
    // set FIFO address to current RX address
    LoRa_writeRegister(REG_FIFO_ADDR_PTR, LoRa_readRegister(REG_FIFO_RX_CURRENT_ADDR));
    if (_onReceive) { _onReceive(packetLength); }
    // reset FIFO address
    LoRa_writeRegister(REG_FIFO_ADDR_PTR, 0);
  }
}
uint8_t LoRa_readRegister(uint8_t address){

  return LoRa_singleTransfer(address & 0x7f, 0x00);
}
void LoRa_writeRegister(uint8_t address, uint8_t value)
{
  LoRa_singleTransfer(address | 0x80, value);
}
uint8_t LoRa_singleTransfer(uint8_t address, uint8_t value){

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //digitalWrite(_ss, LOW);
  HAL_SPI_TransmitReceive(&hspi1, (unsigned char *) &address, (unsigned char *) &SPI1_Rx_Data, 1,50);
  HAL_SPI_TransmitReceive(&hspi1, (unsigned char *) &value, (unsigned char *) &SPI1_Rx_Data, 1,50);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //digitalWrite(_ss, HIGH);
  return SPI1_Rx_Data;
}
void LoRa_onDio0Rise()
{
	LoRa_handleDio0Rise();
}

//+++++++++++++++++++++++++++
void SX1276SetOpMode( uint8_t opMode )
{
//    if( opMode == RF_OPMODE_SLEEP )
//    {
//        SX1276SetAntSwLowPower( true );
//    }
//    else
//    {
//        SX1276SetAntSwLowPower( false );
//        if( opMode == RF_OPMODE_TRANSMITTER )
//        {
//             SX1276SetAntSw( 1 );
//        }
//        else
//        {
//             SX1276SetAntSw( 0 );
//        }
//    }
    if( opMode == RF_OPMODE_SLEEP )
    {
    	LoRa_writeRegister( REG_OPMODE, ( LoRa_readRegister( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
    }
    else LoRa_writeRegister( REG_OPMODE, ( LoRa_readRegister( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}
void SX1276SetModem( RadioModems_t modem )
{
//    if( ( SX1276Read( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_ON ) != 0 )
//    {
//        SX1276.Settings.Modem = MODEM_LORA;
//    }
//    else
//    {
//        SX1276.Settings.Modem = MODEM_FSK;
//    }
//    if( SX1276.Settings.Modem == modem )
//    {
//        return;
//    }
//    SX1276.Settings.Modem = modem;
    switch( modem )
    {
    default:
    case MODEM_FSK:
    	LoRa_sleep();
        LoRa_writeRegister( REG_OPMODE, (LoRa_readRegister( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );
        LoRa_writeRegister( REG_DIOMAPPING1, 0x00 );
        LoRa_writeRegister( REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
    	LoRa_sleep();
        LoRa_writeRegister( REG_OPMODE, ( LoRa_readRegister( REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );
        LoRa_writeRegister( REG_DIOMAPPING1, 0x00 );
        LoRa_writeRegister( REG_DIOMAPPING2, 0x00 );
        break;
    }
}
void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    SX1276SetModem( modem );
    LoRa_setTxPower(17, PA_BOOST); //SX1276SetRfTxPower( power );

        {
            //SX1276.Settings.Fsk.Power = power;
//            SX1276.Settings.Fsk.Fdev = fdev;
//            SX1276.Settings.Fsk.Bandwidth = bandwidth;
//            SX1276.Settings.Fsk.Datarate = datarate;
//            SX1276.Settings.Fsk.PreambleLen = preambleLen;
//            SX1276.Settings.Fsk.FixLen = fixLen;
//            SX1276.Settings.Fsk.CrcOn = crcOn;
//            SX1276.Settings.Fsk.IqInverted = iqInverted;
//            SX1276.Settings.Fsk.TxTimeout = timeout;

            fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
            LoRa_writeRegister( REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
            LoRa_writeRegister( REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            LoRa_writeRegister( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            LoRa_writeRegister( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            LoRa_writeRegister( REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            LoRa_writeRegister( REG_PREAMBLELSB, preambleLen & 0xFF );

            LoRa_writeRegister( REG_PACKETCONFIG1,
                         ( LoRa_readRegister( REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
            LoRa_writeRegister( REG_PACKETCONFIG2, ( LoRa_readRegister( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );
        }

}
void LoRa_SetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time ){

    uint32_t timeout = ( uint32_t )( time * 1e6 );
    LoRa_setFrequency( freq );
    SX1276SetTxConfig( MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout );
    LoRa_writeRegister( REG_PACKETCONFIG2, ( LoRa_readRegister( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    // Disable radio interrupts
    LoRa_writeRegister( REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11 );
    LoRa_writeRegister( REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10 );
    //this->settings.State = RF_TX_RUNNING;
    //txTimeoutTimer.attach_us( mbed::callback( this, &SX1276::OnTimeoutIrq ), timeout );
    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
}
//+++++++++++++++++++++++++++
//+++++++++++++++++++++++++
#define	Funtion_No				0
#define	Funtion_CarryWave		1
#define	Funtion_Receive			2
#define	Funtion_PackSend		3
#define	Funtion_PackRev			4
#define	Funtion_Sleep			5
#define	u8	unsigned char
#define	u16 unsigned short int
#define b_changemode			(1<<0)
#define LR_RegOpMode                                0x0100
const u8  testData[] = {"HPD 11/12/13 1234567890"};
u8 test_RxData[32];
u8 test_TxData[32];
unsigned short  Tx_Data  = 0xD000;
unsigned short  Rx_Data  = 0;
unsigned char HPD1X_Para = 0x00;
unsigned char  KeyValue;
unsigned char  FuntionMode;
unsigned char  ModeFlag;
u8 HPD11Step = 0,LinkStat,Temp1,LinkStatus;
//+++++++++++++++++++++++
#define SetIO2 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define ChkIO0 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)
void Delay_ms(unsigned short int Cnt){

	unsigned short int i,j;
	for(i=0;i<Cnt;i++) { for(j=0;j<900;j++); }
}
void IO2_INPUT() {

	//PA0
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void IO2_OUTPUT() {

	//PA0
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
//+++++++++++++++++++++++
uint8_t spiRead8(uint8_t dat){

	dat = dat & 0x7f;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //digitalWrite(_ss, LOW);
    HAL_SPI_TransmitReceive(&hspi1, (unsigned char *) &dat, (unsigned char *) &Rx_Data, 1,50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //digitalWrite(_ss, HIGH);
    return Rx_Data;
}
uint8_t spiTransfer(uint8_t address, uint8_t value){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //digitalWrite(_ss, LOW);
    HAL_SPI_TransmitReceive(&hspi1, (unsigned char *) &address, (unsigned char *) &Rx_Data, 1,50);
    HAL_SPI_TransmitReceive(&hspi1, (unsigned char *) &value, (unsigned char *) &Rx_Data, 1,50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //digitalWrite(_ss, HIGH);
    return Rx_Data;
}
uint8_t spiWrite16(uint16_t data){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //digitalWrite(_ss, LOW);
	u8 lo = (data & 0xff);
	u8 hi = ((data>>8) & 0xff) | 0x80;
    HAL_SPI_TransmitReceive(&hspi1, (unsigned char *) &hi, (unsigned char *) &Rx_Data, 1,50);
    HAL_SPI_TransmitReceive(&hspi1, (unsigned char *) &lo, (unsigned char *) &Rx_Data, 1,50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //digitalWrite(_ss, HIGH);
    return Rx_Data;
}
uint8_t spiWriteBuf(uint8_t addr,uint8_t* data,uint16_t len){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //digitalWrite(_ss, LOW);
    HAL_SPI_TransmitReceive(&hspi1, (unsigned char *) (addr|0x80) , (unsigned char *) &Rx_Data, 1,50);
    HAL_SPI_TransmitReceive(&hspi1, (unsigned char *) (data) , (unsigned char *) &Rx_Data, len,50);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //digitalWrite(_ss, HIGH);
    return Rx_Data;
}
uint8_t spiReadBuf(uint8_t addr,uint8_t* data,uint16_t len){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //digitalWrite(_ss, LOW);
    HAL_SPI_TransmitReceive(&hspi1, (unsigned char *) (addr) , (unsigned char *) &Rx_Data, 1,50);
    HAL_SPI_TransmitReceive(&hspi1, (unsigned char *) (0) , (unsigned char *) &test_RxData, len,50);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //digitalWrite(_ss, HIGH);
    return Rx_Data;
}
//+++++++++++++++++++++++
u8 FSKTxWaitStable(void){

	Delay_ms(5);
	u8 Temp1 = spiRead8(0x3E);
	if ((Temp1 & 0xA0) == 0xA0) return 1;
	else return 0;
}
u8 FSKRxWaitStable(void) {

	Delay_ms(5);
	Temp1 = spiRead8(0x3E);
	if ((Temp1 & 0xC0) == 0xC0)
		return 1;
	else
		return 0;
}
void SX1276Standby() {

	spiWrite16(LR_RegOpMode + 0x01 + HPD1X_Para);
}
void SX1276Sleep(void) {

	spiWrite16(LR_RegOpMode + 0x00 + HPD1X_Para);         //Sleep
}
void sx1276initTest() {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //digitalWrite(_reset, HIGH);
	delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); //digitalWrite(_reset, LOW);
	delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //digitalWrite(_reset, HIGH);
	delay(10);
	SX1276Sleep();                  //Change modem mode Must in Sleep mode
	delay(10);

	//setting frequency parameter
	spiWrite16(0x06D9);         //868	*2^14 = 14221312 = D90000
	spiWrite16(0x0700);
	spiWrite16(0x0800);
	//setting rf rate parameter
	spiWrite16(0x0268);         //BR=1.2Kbps
	spiWrite16(0x032B);
	//Setting output power parameter
	spiWrite16(0x09FF);         //20dbm
	//setting base parameter
	spiWrite16(0x0402);	//RegFdevMsb 35KHz
	spiWrite16(0x053D);	//RegFdevLsb
	spiWrite16(0x0B0B);	//RegOcp  Close Ocp
	spiWrite16(0x0C23);	//RegLNA  High & LNA Enable
	spiWrite16(0x1212);	//RegRxBw 83KHz
	spiWrite16(0x1FA0);	//RegPreambleDet  Enable 2Byte
	spiWrite16(0x2500);	//RegPreambleMsb
	spiWrite16(0x2600);	//RegPreambleLsb  0Byte Preamble
	spiWrite16(0x2100);	//RegPacketConfig2  Continuous Mode
	// standby
	SX1276Standby();
}
void sx1276initFSK() {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); //digitalWrite(_reset, LOW);
	delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); //digitalWrite(_reset, HIGH);
	delay(10);
	SX1276Sleep();                  //Change modem mode Must in Sleep mode
	delay(10);

	//setting frequency parameter
	spiWrite16(0x06D9);	//868	*2^14 = 14221312 = D90000
	spiWrite16(0x0700);
	spiWrite16(0x0800);

	//Setting output power parameter
	spiWrite16(0x09FF);	//20dbm

	//setting base parameter
	spiWrite16(0x0402);      //RegFdevMsb  35KHz
	spiWrite16(0x053D);		//RegFdevLsb
	spiWrite16(0x0B0B);		//RegOcp  Close Ocp
	//0x0C20,           //RegLNA  High & LNA Disable
	spiWrite16(0x0C23);		//RegLNA  High & LNA Enable
	spiWrite16(0x1212);		//RegRxBw   83KHz
	spiWrite16(0x1FA0);		//RegPreambleDet  Enable 2Byte
	//0x1F20,           //RegPreambleDet  Disable
	spiWrite16(0x2500);		//RegPreambleMsb
	spiWrite16(0x2606);		//RegPreambleLsb  6Byte Preamble
	spiWrite16(0x2792);		//RegSyncConfig Sync 2+1=3bytes
	spiWrite16(0x2800 + 0xAA);	//SyncWord = aa2dd4
	spiWrite16(0x2900 + 0x2D);	//
	spiWrite16(0x2A00 + 0xD4);	//
	spiWrite16(0x3000);		//RegPacketConfig1  Disable CRC��NRZ
	spiWrite16(0x3140);		//RegPacketConfig2  Packet Mode
	spiWrite16(0x3215);		//RegPayloadLength  21bytes Fixed
	spiWrite16(0x3595);		//RegFiFoThresh   21bytes

	// standby
	SX1276Standby();
}
void FskClearFIFO(void) {

	spiWrite16(0x0101);            //Standby
	spiWrite16(0x0105 + HPD1X_Para); //entry RxMode
}
void FskEntryTx(void) {

	sx1276initFSK();
	//setting rf rate parameter
	spiWrite16(0x0268); //BR=1.2Kbps
	spiWrite16(0x032B);
	//Define to Tx mode
	spiWrite16(0x4000); //DIO0 Mapping for IRQ / DIO2 for RxData
	spiWrite16(0x4100); //...
	spiWrite16(0x4D87); //DIO0 Mapping for IRQ / DIO2 for RxData
	spiWrite16(0x0103 + HPD1X_Para);
}
void sleepTest(void) {

	sx1276initFSK();
	// init for FSK
	sx1276initFSK();
	// IO2 as input
	IO2_INPUT();
	// set sleep
	SX1276Sleep();
	// loop
	while (1) { if(ModeFlag & b_changemode) break; }
	// standby
	SX1276Standby();
}
void carryWaveTest(){

	unsigned char flag = 0,Step=0;

	// IO2 as output
	IO2_OUTPUT();
	// init for test
	sx1276initTest();

	while (1) {
		switch (Step) {
			case 0:
				//Define to Tx mode
				spiWrite16(0x4000); //DIO0 Mapping for IRQ / DIO2 for RxData
				spiWrite16(0x4100); // "
				spiWrite16(0x4D87); //20dBm Tx
				//...
				spiWrite16(0x0100 + 0x20 + HPD1X_Para + 0x03);
				Step = 1;
			break;
			case 1:
				if ( FSKTxWaitStable() != 0) Step = 2;
			break;
			case 2:
				SetIO2; Step = 3; //IO2
			break;
			case 3:
				if (ModeFlag & b_changemode) flag = 1;
			break;
			default:
			break;
		}
		if (flag != 0) break;
	}
	// standby
	SX1276Standby();
	// IO2 as input
	IO2_INPUT();
}
void FSKPackageTxSampleTest(void){

	unsigned char flag = 0,Step = 0;
	LinkStatus = 1;

	while (1) {
		switch (Step) {
			case 0:
				FskEntryTx();					//FSK TX PACKAGE mode
				Step = 1;
			break;
			case 1:
				if (FSKTxWaitStable() != 0)	Step = 2; //WAIT TX
				else {
					SX1276Standby();
					spiWrite16(0x0103 + HPD1X_Para);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				}
			break;
			case 2:
				spiWriteBuf(0x00, (u8 *) testData, 21);  //д
				Step = 3;
			break;
			case 3:
				if (ChkIO0){                     //Packet send Finish
					LinkStatus ^= 1;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
					SX1276Standby();             //Entry Standby mode
					Step = 4;
				}
			break;
			case 4:
				if (ModeFlag & b_changemode) { flag = 1; break; }
				spiWrite16(0x0103 + HPD1X_Para);
				Step = 1;
			break;
			default:
			break;
		}
		if (flag != 0) break;
	}
	// standby
	SX1276Standby();
}
void FSKPackageRxSampleTest(void) {

	unsigned char i;
	unsigned char flag = 0;

	LinkStatus = 1;
	u8 Step = 0;
	sx1276initFSK();

	while (1) {
		switch (Step) {
			case 0:
				FskEntryTx();				//FSK
				Step = 1;
			break;
			case 1:
				if (FSKRxWaitStable() != 0)Step = 2;		//̬ OK
			break;
			case 2:
				if (ChkIO0){								//IO0 HIGH PLUS
					for (i = 0; i < 32; i++) test_RxData[i] = 0x00;
					spiReadBuf(0x00, test_RxData, 21);
					FskClearFIFO();
					for (i = 0; i < 17; i++) {
						if (test_RxData[i] != testData[i]) break;
					}
					if (i >= 17) LinkStatus ^= 1;  //Rx success LED
					Step = 3;
				}
				if (ModeFlag & b_changemode) flag = 1;		//
			break;
			case 3: Step = 2; break;
			default: break;
		}
		if (flag != 0) break;
	}
	// standby
	SX1276Standby();
}
void DirectRxModeTest(void) {

	unsigned char flag = 0;
	HPD11Step = 0;
	SX1276Standby();
	while (1) {
		switch (HPD11Step) {
			case 0:
				//IO2 as inpput
				IO2_INPUT(); //IO2
				// init for test
				sx1276initTest();
				//Define to Rx mode
				spiWrite16(0x090F);//RFIO Pin
				spiWrite16(0x400C);//DIO0 Mapping for IRQ / DIO2 for RxData
				spiWrite16(0x4100);//...
				spiWrite16(0x4D84);//Normal and Rx

				spiWrite16(0x0105 + HPD1X_Para);
				HPD11Step = 1;
			break;
			case 1:
				if (FSKRxWaitStable() != 0) HPD11Step = 2;
			break;
			case 2:
				if (ModeFlag & b_changemode) flag = 1;
			break;
			default: break;
		}
		if (flag != 0) break;
	}
	SX1276Standby();
}
void HPDTest(){

	while (1) {
		if (ModeFlag & b_changemode) {
		    ModeFlag &= ~b_changemode; //change mode
			if (FuntionMode == Funtion_CarryWave) {
				carryWaveTest();
			}
			else if (FuntionMode == Funtion_Receive) {
				DirectRxModeTest();
			}
			else if (FuntionMode == Funtion_PackSend) {
				FSKPackageTxSampleTest();
			}
			else if (FuntionMode == Funtion_PackRev) {
				FSKPackageRxSampleTest();
			}
			else if (FuntionMode == Funtion_Sleep) {
				sleepTest();
			}
		}
	}
}
