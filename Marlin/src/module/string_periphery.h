#pragma once


#include "../inc/MarlinConfig.h"
//#include "../libs/Adafruit_MAX31865_sw.h"
#include "../libs/MAX6675.h"
//#include "../libs/MAX31865.h"
//#include "../libs/AS5048A.h"
#include "../libs/AS5600.h"
#include "../feature/twibus.h"
#include "../libs/MCP4725.h"
#include "../libs/ADS1X15.h"
#include "../libs/SparkFun_I2C_Mux_Arduino_Library.h"

/*
#define TEMP_0_CS_PIN                     PF8   // Max31865 CS
  #define TEMP_0_SCK_PIN                    PA5
  #define TEMP_0_MISO_PIN                   PA6
  #define TEMP_0_MOSI_PIN                   PA7
*/


class StringPeriphery {

public:

MCP4725 mcp4725_hv_i = MCP4725(0x60);
MCP4725 mcp4725_hv_v = MCP4725(0x61);
MCP4725 mcp4725_press = MCP4725(0x60);
ADS1015 analog_inp = ADS1015(0x48);
QWIICMUX mux_iic = QWIICMUX();
//Adafruit_MAX31865_sw max_test1 = Adafruit_MAX31865_sw(TEMP_0_CS_PIN ,TEMP_0_MOSI_PIN,TEMP_0_MISO_PIN,TEMP_0_SCK_PIN);
//AS5048A a5048_lin = AS5048A(ENC_LIN_CS_PIN ,TEMP_0_MOSI_PIN,TEMP_0_MISO_PIN,TEMP_0_SCK_PIN);
AS5600 as5600_lin = AS5600();
MAX6675 max6675_temp_cam_ext = MAX6675(TEMP_0_CS_PIN ,TEMP_0_MOSI_PIN,TEMP_0_MISO_PIN,TEMP_0_SCK_PIN);
MAX6675 max6675_temp_cam_intern_1 = MAX6675(TEMP_1_CS_PIN ,TEMP_0_MOSI_PIN,TEMP_0_MISO_PIN,TEMP_0_SCK_PIN);
MAX6675 max6675_temp_cam_intern_2 = MAX6675(TEMP_2_CS_PIN ,TEMP_0_MOSI_PIN,TEMP_0_MISO_PIN,TEMP_0_SCK_PIN);
//MAX31865 max_test1 = MAX31865(TEMP_0_CS_PIN ,TEMP_0_MOSI_PIN,TEMP_0_MISO_PIN,TEMP_0_SCK_PIN);


void init();
void idle();

//-----------------M576
float get_vel();

int16_t get_pos();
void set_vel_lin(float v);
void set_vel_gateway(float v);
void set_vel_powder(float v);
void set_turbine(float v);

//-----------------M577
float get_v_hv();
float get_a_hv();
void set_hv_v(uint16_t v);
void set_hv_i(uint16_t v);

//-----------------M578
float get_v_press();
void set_press(uint16_t v);

//-----------------M579

float get_temp_cam_ext();
float get_temp_cam_intern1();
float get_temp_cam_intern2();

void set_reley_1(int v);
void set_reley_2(int v);
void set_reley_heater(int ind,int v);

void set_reley_HV(int v);
void set_reley_press(int v);

void set_heater_2(int v);
void set_heater_3(int v);

void set_heaters_enable(int v);
void set_heaters_ind(int v);
void set_heaters_temp(float v);

void manage_heat();
void manage_heat_duty();
int manage_heat_duty_single(int ind, float temp, float kp);
void heat_pwm_control();
void heat_pwm_control_single(int ind, int counter,int duty);

void report_state();
void set_reporting(bool state);
long string_lenght = 0;
float kp_1 = 0.5;
float kp_2 = 0.5;
private:

unsigned long time_measure,time_measure_enc,time_measure_temp;
float temp_val_int1= 0;
float temp_val_int2= 0;
float temp_val_ext= 0;
int reley_1,reley_2,reley_HV,reley_press, turbo, string_cur_pos, string_last_pos,moves_planned;//

float pressure = 0;//
float HV = 0;//

float temp_dest = 0;
int ind_sensor = 0;
int heater_en = 0;
float temp_hyst = 3;
bool reporting = true;

bool heating_1,heating_2;
float duty_1,duty_2;

int duty_time_1 = 0 ; 
int duty_time_2 = 0 ; 
int cycle_time = 50;
int duty_counter = 0;
};
extern StringPeriphery string_manager;
