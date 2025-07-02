//stm32 (Blue Pill) wiring :
// CS to PA4 , CLK to PA5 , MOSI to 3.3V , MISO to PA6
//UNO wiring :
//CS to PIN 10 , CLK to PIN 13 , MOSI to 5v , MISO to PIN 12


#include "Arduino.h"


#include "../inc/MarlinConfig.h"
#include "../HAL/shared/Delay.h"
#include HAL_PATH(.., MarlinSPI.h)
/// globals
extern  uint16_t old_result_AS5048A ;
extern	uint32_t previousMillis_AS5048A ;
extern  int16_t diff_AS5048A;
extern  int DIR_AS5048A;
extern  int16_t res_info_AS5048A;


class AS5048A{
	
	public:
	
		// Constructor
		AS5048A(int32_t spi_cs, int32_t spi_mosi, int32_t spi_miso,
             int32_t spi_clk);
		// Methods
		void get_info(int sample);
		void SPI_setup();
		void end_SPI();
		uint16_t get_raw();
		uint16_t get_pos();
		bool get_DIR();
		float get_speed();
		uint16_t _read_ns();
		uint32_t _read();
		
	 private:
	 
		int sample_all;
		int value;
		uint16_t _rawData;
		uint32_t sclkPin, misoPin, mosiPin, cselPin;
		void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
		uint8_t readRegister8(uint8_t addr);
		uint16_t readRegister16(uint8_t addr);

		void writeRegister8(uint8_t addr, uint8_t reg);
		void writeRegister16(uint8_t addr, uint16_t reg);

		void softSpiInit();
		void spiBeginTransaction();
		uint8_t spiTransfer(uint8_t addr);
		void spiEndTransaction();
      	
    
};


