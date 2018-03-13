#ifndef LORA_H
#define LORA_H

#include "stm32f3xx_hal.h"
#include <stdbool.h>
//#include <Arduino.h>
//#include <SPI.h>

#define LORA_DEFAULT_SS_PIN     10
#define LORA_DEFAULT_RESET_PIN  9
#define LORA_DEFAULT_DIO0_PIN   2

#define PA_OUTPUT_RFO_PIN       0
#define PA_OUTPUT_PA_BOOST_PIN  1

#if defined (__STM32F1__) || defined (STM32GENERIC)
inline unsigned char  digitalPinToInterrupt(unsigned char Interrupt_pin) { return Interrupt_pin; } //This isn't included in the stm32duino libs (yet)
#define portOutputRegister(port) (volatile byte *)( &(port->regs->ODR) ) //These are defined in STM32F1/variants/generic_stm32f103c/variant.h but return a non byte* value
#define portInputRegister(port) (volatile byte *)( &(port->regs->IDR) ) //These are defined in STM32F1/variants/generic_stm32f103c/variant.h but return a non byte* value
#endif

extern unsigned char  ModeFlag;
//class LoRaClass : public Stream {
//public:
//LoRaClass();

  int LoRa_begin(long frequency,bool PABOOST);
  void LoRa_end();

  int LoRa_beginPacket(int implicitHeader);
  int LoRa_endPacket();

  int LoRa_parsePacket(int size);
  int LoRa_packetRssi();
  float LoRa_packetSnr();

  // from Print
  size_t write(uint8_t byte);
  size_t writebfr(const uint8_t *buffer, size_t size);

  // from Stream
  int  available();
  int  read();
  int  peek();
  void flush();
  size_t LoRa_print(char* str);
  void LoRa_onReceive(void(*callback)(int));

  void LoRa_receive(int size);
  void LoRa_idle();
  void LoRa_sleep();

  void LoRa_setTxPower(int level, int outputPin);
  void LoRa_setFrequency(long frequency);
  void LoRa_setSpreadingFactor(int sf);
  void LoRa_setSignalBandwidth(long sbw);
  void LoRa_setCodingRate4(int denominator);
  void LoRa_setPreambleLength(long length);
  void LoRa_setSyncWord(int sw);
  void LoRa_enableCrc();
  void LoRa_disableCrc();
  void sx1276SetTx();
  // deprecated
  //void LoRa_crc() { LoRa_enableCrc(); }
  //void LoRa_noCrc() { LoRa_disableCrc(); }

  uint8_t LoRa_random();

  void LoRa_setPins(int ss , int reset, int dio0);
  void LoRa_setSPIFrequency(uint32_t frequency);

  //void dumpRegisters(Stream& out);

//private:
  void LoRa_explicitHeaderMode();
  void LoRa_implicitHeaderMode();

  void LoRa_handleDio0Rise();

  uint8_t LoRa_readRegister(uint8_t address);
  void LoRa_writeRegister(uint8_t address, uint8_t value);
  uint8_t LoRa_singleTransfer(uint8_t address, uint8_t value);

  static void LoRa_onDio0Rise();

//private:
  //SPISettings _spiSettings;
  int _ss;
  int _reset;
  int _dio0;
  int _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);
//};

//extern LoRaClass LoRa;

#endif
