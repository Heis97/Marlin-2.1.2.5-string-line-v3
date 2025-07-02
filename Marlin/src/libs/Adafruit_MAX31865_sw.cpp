/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_MAX31865_sw.h"

#include <stdlib.h>
#define MAX31865_SPI_TIMING_TCC    400    // CS to SCLK setup
#define MAX31865_SPI_TIMING_TDC     35    // Data to SCLK setup
#define MAX31865_SPI_TIMING_TCL    100    // SCK half period
#define MAX31865_SPI_TIMING_TCCH   100    // SCK to CS hold
#define MAX31865_SPI_TIMING_TCWH   400    // CS inactive time (min)
/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI
    @param spi_cs the SPI CS pin to use
    @param spi_mosi the SPI MOSI pin to use
    @param spi_miso the SPI MISO pin to use
    @param spi_clk the SPI clock pin to use
*/
/**************************************************************************/
//
Adafruit_MAX31865_sw::Adafruit_MAX31865_sw(int32_t spi_cs, int32_t spi_mosi, int32_t spi_miso,
             int32_t spi_clk)
  {
    cselPin = spi_cs;
    mosiPin = spi_mosi;
    misoPin = spi_miso;
    sclkPin = spi_clk;
   }

/*!
    @brief Initialize the SPI interface and set the number of RTD wires used
    @param wires The number of wires in enum format. Can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @return True
*/
/**************************************************************************/
bool Adafruit_MAX31865_sw::begin(max31865_numwires_t wires) {
 // spi_dev.begin();
   pinMode(cselPin, OUTPUT);
  digitalWrite(cselPin, HIGH);

  softSpiInit();
  setWires(wires);
  enableBias(false);
  autoConvert(false);
  setThresholds(0, 0xFFFF);
  clearFault();

  // Serial.print("config: ");
  // Serial.println(readRegister8(MAX31865_CONFIG_REG), HEX);
  return true;
}

/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @param fault_cycle The fault cycle type to run. Can be MAX31865_FAULT_NONE,
   MAX31865_FAULT_AUTO, MAX31865_FAULT_MANUAL_RUN, or
   MAX31865_FAULT_MANUAL_FINISH
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t Adafruit_MAX31865_sw::readFault(max31865_fault_cycle_t fault_cycle) {
  if (fault_cycle) {
    uint8_t cfg_reg = readRegister8(MAX31865_CONFIG_REG);
    cfg_reg &= 0x11; // mask out wire and filter bits
    switch (fault_cycle) {
    case MAX31865_FAULT_AUTO:
      writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10000100));
      delay(1);
      break;
    case MAX31865_FAULT_MANUAL_RUN:
      writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10001000));
      return 0;
    case MAX31865_FAULT_MANUAL_FINISH:
      writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10001100));
      return 0;
    case MAX31865_FAULT_NONE:
    default:
      break;
    }
  }
  return readRegister8(MAX31865_FAULTSTAT_REG);
}

/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void Adafruit_MAX31865_sw::clearFault(void) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31865_CONFIG_FAULTSTAT;
  writeRegister8(MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void Adafruit_MAX31865_sw::enableBias(bool b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31865_CONFIG_BIAS; // disable bias
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param b If true, auto conversion is enabled
*/
/**************************************************************************/
void Adafruit_MAX31865_sw::autoConvert(bool b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_MODEAUTO; // enable autoconvert
  } else {
    t &= ~MAX31865_CONFIG_MODEAUTO; // disable autoconvert
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Whether we want filter out 50Hz noise or 60Hz noise
    @param b If true, 50Hz noise is filtered, else 60Hz(default)
*/
/**************************************************************************/

void Adafruit_MAX31865_sw::enable50Hz(bool b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_FILT50HZ;
  } else {
    t &= ~MAX31865_CONFIG_FILT50HZ;
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Write the lower and upper values into the threshold fault
    register to values as returned by readRTD()
    @param lower raw lower threshold
    @param upper raw upper threshold
*/
/**************************************************************************/
void Adafruit_MAX31865_sw::setThresholds(uint16_t lower, uint16_t upper) {
  writeRegister8(MAX31865_LFAULTLSB_REG, lower & 0xFF);
  writeRegister8(MAX31865_LFAULTMSB_REG, lower >> 8);
  writeRegister8(MAX31865_HFAULTLSB_REG, upper & 0xFF);
  writeRegister8(MAX31865_HFAULTMSB_REG, upper >> 8);
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit lower threshold value
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t Adafruit_MAX31865_sw::getLowerThreshold(void) {
  return readRegister16(MAX31865_LFAULTMSB_REG);
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit lower threshold value
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t Adafruit_MAX31865_sw::getUpperThreshold(void) {
  return readRegister16(MAX31865_HFAULTMSB_REG);
}

/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
void Adafruit_MAX31865_sw::setWires(max31865_numwires_t wires) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    t |= MAX31865_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    t &= ~MAX31865_CONFIG_3WIRE;
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float Adafruit_MAX31865_sw::temperature(float RTDnominal, float refResistor) {
  return calculateTemperature(readRTD(), RTDnominal, refResistor);
}
/**************************************************************************/
/*!
    @brief Calculate the temperature in C from the RTD through calculation of
   the resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param RTDraw The raw 16-bit value from the RTD_REG
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float Adafruit_MAX31865_sw::calculateTemperature(uint16_t RTDraw, float RTDnominal,
                                              float refResistor) {
  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = RTDraw;
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t Adafruit_MAX31865_sw::readRTD(void) {
  clearFault();
  enableBias(true);
  delay(2);
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  t |= MAX31865_CONFIG_1SHOT;
  writeRegister8(MAX31865_CONFIG_REG, t);
  delay(63);

  uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG);

  enableBias(false); // Disable bias current again to reduce selfheating.

  // remove fault
  rtd >>= 1;

  return rtd;
}

/**********************************************/

uint8_t Adafruit_MAX31865_sw::readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t Adafruit_MAX31865_sw::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

void Adafruit_MAX31865_sw::readRegisterN(uint8_t addr, uint8_t buffer[],
                                      uint8_t n) {
  addr &= 0x7F; // make sure top bit is not set

  spiBeginTransaction();
  spiTransfer(addr);

  while (n--) {
    buffer[0] = spiTransfer(0xFF);
    buffer++;
  }

  spiEndTransaction();
}

void Adafruit_MAX31865_sw::writeRegister8(uint8_t addr, uint8_t data) {
  spiBeginTransaction();
  spiTransfer(addr | 0x80); // make sure top bit is set
  spiTransfer(data);
  spiEndTransaction();
}


void Adafruit_MAX31865_sw::spiBeginTransaction() {
  digitalWrite(sclkPin, LOW); // ensure CPOL0
  DELAY_NS_VAR(MAX31865_SPI_TIMING_TCWH); // ensure minimum time of CS inactivity after previous operation
  digitalWrite(cselPin, LOW);
  DELAY_NS_VAR(MAX31865_SPI_TIMING_TCC);

  digitalWrite(sclkPin, HIGH);
}

void Adafruit_MAX31865_sw::spiEndTransaction() {

  digitalWrite(sclkPin, LOW);

  DELAY_NS_VAR(MAX31865_SPI_TIMING_TCCH);

  digitalWrite(cselPin, HIGH);
}

/**
 * Transfer SPI data +x+ and read the response. From the datasheet...
 * Input data (SDI) is latched on the internal strobe edge and output data (SDO) is
 * shifted out on the shift edge. There is one clock for each bit transferred.
 * Address and data bits are transferred in groups of eight, MSB first.
 *
 * @param  x  an 8-bit chunk of data to write
 * @return    the 8-bit response
 */
uint8_t Adafruit_MAX31865_sw::spiTransfer(uint8_t x) {

  uint8_t reply = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(mosiPin, x & _BV(i));
    DELAY_NS_VAR(MAX31865_SPI_TIMING_TDC);
    digitalWrite(sclkPin, LOW);
    DELAY_NS_VAR(MAX31865_SPI_TIMING_TCL - MAX31865_SPI_TIMING_TDC);
    reply <<= 1;
    if (digitalRead(misoPin)) reply |= 1;
    DELAY_NS_VAR(MAX31865_SPI_TIMING_TDC);
    digitalWrite(sclkPin, HIGH);
    DELAY_NS_VAR(MAX31865_SPI_TIMING_TCL - MAX31865_SPI_TIMING_TDC);
  }
  return reply;
}

void Adafruit_MAX31865_sw::softSpiInit() {
  pinMode(sclkPin, OUTPUT);
  digitalWrite(sclkPin, LOW);
  pinMode(mosiPin, OUTPUT);
  pinMode(misoPin, INPUT);
}
