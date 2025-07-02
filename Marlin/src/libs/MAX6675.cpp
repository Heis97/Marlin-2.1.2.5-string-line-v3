//
//    FILE: MAX6675.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.3.3
// PURPOSE: Arduino library for MAX6675 chip for K type thermocouple
//    DATE: 2022-01-11
//     URL: https://github.com/RobTillaart/MAX6675


#include "MAX6675.h"


//  HW SPI
MAX6675::MAX6675(uint8_t select, __SPI_CLASS__ * mySPI)
{
  _select = select;
  _miso   = 255;
  _clock  = 255;
  _mySPI  = mySPI;
  _hwSPI  = true;
}


// SW SPI
MAX6675::MAX6675(int32_t spi_cs, int32_t spi_mosi, int32_t spi_miso,
             int32_t spi_clk)
{
  _select = spi_cs;
  _miso   = spi_miso;
  _clock  = spi_clk;

  _mySPI  = NULL;
  _hwSPI  = false;
}


void MAX6675::begin()
{
  _lastTimeRead = 0;
  _offset       = 0;
  _status       = STATUS_NOREAD;
  _temperature  = MAX6675_NO_TEMPERATURE;
  _rawData      = 0;

  setSPIspeed(1000000);

  pinMode(_select, OUTPUT);
  digitalWrite(_select, HIGH);


    pinMode(_clock, OUTPUT);
    digitalWrite(_clock, LOW);
    pinMode(_miso, INPUT);


  
}


void MAX6675::setSPIspeed(uint32_t speed)
{
  _SPIspeed = speed;
  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);
};


uint8_t MAX6675::read()
{
  //  return value of _read()  page 5 datasheet
  //  BITS       DESCRIPTION
  //  ------------------------------
  //       00    three state ?
  //       01    device ID ?
  //       02    INPUT OPEN
  //  03 - 14    TEMPERATURE (RAW)
  //       15    SIGN
  uint16_t value = _read();

  //  needs a pull up on MISO pin to work properly!
  if (value == 0xFFFF)
  {
    _status = STATUS_NO_COMMUNICATION;
    return _status;
  }

  _lastTimeRead = millis();

  //  process status bit 2
  _status = value & 0x04;

   value >>= 3;

  //  process temperature bits
  _temperature = (value & 0x1FFF) * 0.25;
  //  dummy negative flag set ?
  //  if (value & 0x2000)
  return _status;
}


///////////////////////////////////////////////////
//
//  PRIVATE
//
uint32_t MAX6675::_read(void)
{
  _rawData = 0;
  //  DATA TRANSFER


     digitalWrite(_select, LOW);
    //delayMicroseconds(2);
    DELAY_NS_VAR(800);
    for (int8_t i = 15; i >= 0; i--)
    {
      _rawData <<= 1;
      digitalWrite(_clock, HIGH);
      DELAY_NS_VAR(200);
      //for(int j=0; j<1;j++)  __asm__ __volatile__ ("nop");
      if (digitalRead(_miso)) _rawData++;
      DELAY_NS_VAR(200);
      //for(int j=0; j<1;j++)  __asm__ __volatile__ ("nop");
      digitalWrite(_clock, LOW);
      DELAY_NS_VAR(400);
      //for(int j=0; j<2;j++)  __asm__ __volatile__ ("nop");
    }
    DELAY_NS_VAR(800);
    digitalWrite(_select, HIGH);
  

  return _rawData;
}


//  -- END OF FILE --

