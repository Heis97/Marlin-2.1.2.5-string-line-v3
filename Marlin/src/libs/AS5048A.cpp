

#include "AS5048A.h"
#include <stdlib.h>
#define AS5048A_resolution  0.02197399
float AS5048A_resolution_in_rad = AS5048A_resolution * 0.01745329 ; 

#define spi_multipl 1
#define AS5048A_SPI_TIMING_TCC    400 *spi_multipl   // CS to SCLK setup
#define AS5048A_SPI_TIMING_TDC    35 *spi_multipl   // Data to SCLK setup
#define AS5048A_SPI_TIMING_TCL    100 *spi_multipl   // SCK half period
#define AS5048A_SPI_TIMING_TCCH   100 *spi_multipl   // SCK to CS hold
#define AS5048A_SPI_TIMING_TCWH   400 *spi_multipl   // CS inactive time (min)

/// globals 
  uint16_t old_result_AS5048A ;
  uint32_t previousMillis_AS5048A ;  
  int16_t diff_AS5048A;
  int DIR_AS5048A;
  int16_t res_info_AS5048A;
  
  
  
  
  
AS5048A::AS5048A(int32_t spi_cs, int32_t spi_mosi, int32_t spi_miso,
             int32_t spi_clk ){
  cselPin = spi_cs;
    mosiPin = spi_mosi;
    misoPin = spi_miso;
    sclkPin = spi_clk;
  
}


/// sets up our SPI communication parameters
void AS5048A::SPI_setup(){  
  
  softSpiInit();
}



/// ends SPI communication
void AS5048A::end_SPI(){
	SPI.end();
}



/// tells us position .
/// does not depend on any other function .
/// call this if you dont care about direction and speed or you want to calculate it yourself.
uint16_t AS5048A::get_raw(){
  
uint16_t result ;
//result = readRegister16(0);
result = _read();
//result &= 0b0011111111111111;  
res_info_AS5048A = result;
return result;

}



/// samples position every sample_all time
/// using that we can get position, direction and speed
void AS5048A::get_info(int sample){
 
 
  sample_all = sample ;
  uint32_t currentMillis = millis();
 
  
// We take sample every sample_all . Time is in ms 
if(currentMillis-previousMillis_AS5048A >= sample_all ){
  previousMillis_AS5048A=currentMillis;
  res_info_AS5048A = AS5048A::get_raw();
  diff_AS5048A=res_info_AS5048A-old_result_AS5048A;

 
 if (diff_AS5048A==0){
  //do nothing
}else if (abs(diff_AS5048A) < 8192 ){  // if angle changes less then 180 deg
 // if diff_AS5048A is positive we are rotating clockwise
 // if diff_AS5048A is negative we are rotating counterclockwise

     if (diff_AS5048A > 0)
        DIR_AS5048A = 1;     // rotating clockwise
     else
        DIR_AS5048A = 0;    // rotating counterclockwise
}
old_result_AS5048A=res_info_AS5048A; 

}
}



/// tells us direction 1 clockwise, 0 counterclockwise
/// call this only if you have  "get_info" in your main loop
bool AS5048A::get_DIR(){
bool val;
val = DIR_AS5048A;
return val;
}



/// tells us position , same as "get_raw" but it is always updated by "get_info"
/// if you want to know position in that exact moment in code call "get_raw"
/// call this only if you have  "get_info" in your main loop
uint16_t AS5048A::get_pos(){
	uint16_t pos;
	pos = res_info_AS5048A;
	return pos;
}



/// calculates speed in rad/s 
/// call this only if you have "get_info" in your main loop
float AS5048A::get_speed(){
	float speed;
	speed = ((abs(diff_AS5048A)*AS5048A_resolution_in_rad)/(sample_all / 1000.0));	// in rad/s ??
    Serial.println(AS5048A_resolution_in_rad,8);
	return speed;

}

uint16_t AS5048A::_read_ns()
{
  //spiBeginTransaction();
  _rawData = 0;
  spiBeginTransaction();
  for (int8_t i = 15; i >= 0; i--)
  {
    _rawData <<= 1;
    DELAY_NS_VAR(AS5048A_SPI_TIMING_TDC/2);
    digitalWrite(sclkPin, HIGH);
    DELAY_NS_VAR(AS5048A_SPI_TIMING_TCL/2);
    if ( digitalRead(misoPin) ) _rawData++;
    DELAY_NS_VAR(AS5048A_SPI_TIMING_TDC/2);
    digitalWrite(sclkPin, LOW);
    DELAY_NS_VAR(AS5048A_SPI_TIMING_TDC/2);
  }
  spiEndTransaction();
  
  //spiEndTransaction();
  return _rawData;
}

uint8_t AS5048A::readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);
  return ret;
}

/**
 * Read two bytes: 1 from the specified register address, and 1 from the next address.
 *
 * @param  addr  the first register address
 * @return       both register contents as a single 16-bit int
 */
uint16_t AS5048A::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = { 0 };
  readRegisterN(addr, buffer, 2);
  return uint16_t(buffer[0]) << 8 | buffer[1];
}

/**
 * Read +n+ bytes from a specified address into +buffer+. Set D7 to 0 to specify a read.
 *
 * @param addr    the first register address
 * @param buffer  storage for the read bytes
 * @param n       the number of bytes to read
 */
void AS5048A::readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n) {

  addr &= 0x7F; // make sure top bit is not set

  spiBeginTransaction();
  spiTransfer(addr);

  while (n--) {
    buffer[0] = spiTransfer(0xFF);
    buffer++;
  }

  spiEndTransaction();
}

void AS5048A::writeRegister16(uint8_t addr, uint16_t data) {
  spiBeginTransaction();
  spiTransfer(addr | 0x80); // make sure top bit is set
  spiTransfer(data >> 8);
  spiTransfer(data & 0xFF);
  spiEndTransaction();
}

/**
 * Write an 8-bit value to a register. Set D7 to 1 to specify a write.
 *
 * @param addr  the address to write to
 * @param data  the data to write
 */
void AS5048A::writeRegister8(uint8_t addr, uint8_t data) {
  spiBeginTransaction();
  spiTransfer(addr | 0x80); // make sure top bit is set
  spiTransfer(data);
  spiEndTransaction();
}

void AS5048A::spiBeginTransaction() {
  digitalWrite(sclkPin, LOW); // ensure CPOL0
  DELAY_NS_VAR(AS5048A_SPI_TIMING_TCWH); // ensure minimum time of CS inactivity after previous operation
  digitalWrite(cselPin, LOW);
  DELAY_NS_VAR(AS5048A_SPI_TIMING_TCC);

 // if (sclkPin == TERN(LARGE_PINMAP, -1UL, 255))
  //  SPI.beginTransaction(spiConfig);
 // else
    digitalWrite(sclkPin, HIGH);
}

void AS5048A::spiEndTransaction() {
 // if (sclkPin == TERN(LARGE_PINMAP, -1UL, 255))
   // SPI.endTransaction();
  //else
    digitalWrite(sclkPin, LOW);

  DELAY_NS_VAR(AS5048A_SPI_TIMING_TCCH);

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
uint8_t AS5048A::spiTransfer(uint8_t x) {
 // if (sclkPin == TERN(LARGE_PINMAP, -1UL, 255))
  //  return SPI.transfer(x);

  uint8_t reply = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(mosiPin, x & _BV(i));
    DELAY_NS_VAR(AS5048A_SPI_TIMING_TDC);
    digitalWrite(sclkPin, LOW);
    DELAY_NS_VAR(AS5048A_SPI_TIMING_TCL - AS5048A_SPI_TIMING_TDC);
    reply <<= 1;
    if (digitalRead(misoPin)) reply |= 1;
    DELAY_NS_VAR(AS5048A_SPI_TIMING_TDC);
    digitalWrite(sclkPin, HIGH);
    DELAY_NS_VAR(AS5048A_SPI_TIMING_TCL - AS5048A_SPI_TIMING_TDC);
  }
  return reply;
}

void AS5048A::softSpiInit() {
   pinMode(cselPin, OUTPUT);
  digitalWrite(cselPin, HIGH);

  pinMode(sclkPin, OUTPUT);
  digitalWrite(sclkPin, LOW);
  pinMode(mosiPin, OUTPUT);
  pinMode(misoPin, INPUT);
}

uint32_t AS5048A::_read(void)
{
  _rawData = 0;

     digitalWrite(cselPin, LOW);
    DELAY_NS_VAR(800);
    for (int8_t i = 15; i >= 0; i--)
    {
      _rawData <<= 1;
      digitalWrite(sclkPin, HIGH);
      DELAY_NS_VAR(200);
      if (digitalRead(misoPin)) _rawData++;
      DELAY_NS_VAR(200);
      digitalWrite(sclkPin, LOW);
      DELAY_NS_VAR(400);
    }
    DELAY_NS_VAR(800);
    digitalWrite(cselPin, HIGH);
  

  return _rawData;
}