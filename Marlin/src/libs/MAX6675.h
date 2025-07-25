#pragma once
//
//    FILE: MAX6675.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.3.3
// PURPOSE: Arduino library for MAX6675 chip for K type thermocouple
//    DATE: 2022-01-12
//     URL: https://github.com/RobTillaart/MAX6675


//  TODO Breakout board
//
//       +---------+
//   Vin | o       |
//   3V3 | o       |
//   GND | o     O | Thermocouple
//    D0 | o     O | Thermocouple
//    CS | o       |
//   CLK | o       |
//       +---------+

#include "Arduino.h"
#include "SPI.h"
#include "digitalWriteFast.h"
#include "../inc/MarlinConfig.h"
#include "../HAL/shared/Delay.h"

#define MAX6675_LIB_VERSION               (F("0.3.3"))


#ifndef __SPI_CLASS__
  //  MBED must be tested before RP2040
  #if defined(ARDUINO_ARCH_MBED)
  #define __SPI_CLASS__   SPIClass
  #elif defined(ARDUINO_ARCH_RP2040)
  #define __SPI_CLASS__   SPIClassRP2040
  #else
  #define __SPI_CLASS__   SPIClass
  #endif
#endif


#define MAX6675_NO_TEMPERATURE            -999

//  STATE constants returned by read()
//  TODO check
#define STATUS_OK                         0x00
#define STATUS_ERROR                      0x04
#define STATUS_NOREAD                     0x80
#define STATUS_NO_COMMUNICATION           0x81


//  Thermocouples working is based upon Seebeck effect.
//  Different TC have a different Seebeck Coefficient  (µV/°C)
//  See http://www.analog.com/library/analogDialogue/archives/44-10/thermocouple.html
//
//  K_TC == 41.276 µV/°C


class MAX6675
{
public:
  // HW SPI
  MAX6675(uint8_t select, __SPI_CLASS__ * mySPI);
  // SW SPI
  MAX6675(int32_t spi_cs, int32_t spi_mosi, int32_t spi_miso,
             int32_t spi_clk);

  void     begin();

  //       returns state - bit field: 0 = STATUS_OK
  uint8_t  read();
  float    getCelsius(void)      { return _temperature + _offset; };
  float    getFahrenheit(void)   { return getCelsius() * 1.8 + 32; };

  uint8_t  getStatus(void) const { return _status; };

  //       use offset to calibrate the TC.
  //       offset is in degrees Celsius.
  void     setOffset(const float  t)   { _offset = t; };
  float    getOffset() const           { return _offset; };

  uint32_t lastRead()    { return _lastTimeRead; };
  uint16_t getRawData()  { return _rawData;};

  //       speed in Hz
  void     setSPIspeed(uint32_t speed);
  uint32_t getSPIspeed() { return _SPIspeed; };
  void     setSWSPIdelay(uint16_t del = 0)  { _swSPIdelay = del; };
  uint16_t getSWSPIdelay() { return _swSPIdelay; };

  //  obsolete in future.
  float    getTemperature(void)  { return _temperature + _offset; };

private:
  uint32_t _read();

  uint8_t  _status;
  float    _temperature;
  float    _offset;
  uint32_t _lastTimeRead;
  uint16_t _rawData;
  bool     _hwSPI;

  uint32_t  _clock;
  uint32_t  _miso;
  uint32_t  _select;

  uint16_t    _swSPIdelay = 0;
  uint32_t    _SPIspeed;
  __SPI_CLASS__ * _mySPI;
  SPISettings     _spi_settings;
};


//  -- END OF FILE --

