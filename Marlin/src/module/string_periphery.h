#pragma once


#include "../inc/MarlinConfig.h"
//#include "../libs/Adafruit_MAX31865_sw.h"
#include "../feature/twibus.h"
#include "../libs/MAX6675.h"
//#include "../libs/MAX31865.h"
//#include "../libs/AS5048A.h"
#include "../libs/AS5600.h" 

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

//M580----------------------------------- 

void manage_motion();
int manage_axis(AxisEnum Axis, int vibr, int k, int k_m,int dir,int vibr_a, uint16_t step_mot,uint16_t dir_mot);



//-----------------------------------
long string_lenght = 0;
float kp_1 = 0.05;
float kp_2 = 0.05;
int cycle_time = 50;
int period_manage_ms = 200;

int gateway_move = 0;
int feed_pound_move = 0;
int recuperator_move = 0;
int karet_move = 0;
int string_move = 0;
int string_move_second = 0;

float dist_m = 0.05;
int buff_m = 11;

int k_m_x = 2;
int vibr_x = 0;
int vibr_a_x = 0;
int k_x = 0;
int dir_x = 1;

int k_m_y = 2;
int vibr_y = 0;
int vibr_a_y = 0;
int k_y = 0;
int dir_y = 1;

int k_m_z = 2;
int vibr_z = 0;
int vibr_a_z = 0;
int k_z = 0;
int dir_z = 1;

int k_m_e = 2;
int vibr_e = 0;
int vibr_a_e = 0;
int k_e = 0;
int dir_e = 1;

int k_m_a = 2;
int vibr_a = 0;
int vibr_a_a = 0;
int k_a = 0;
int dir_a = 1;

int k_m_b = 2;
int vibr_b = 0;
int vibr_a_b = 0;
int k_b = 0;
int dir_b = 1;


int step_switch = 0;
int dir_switch = 0;

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
int duty_1 = 0;
int duty_2 = 0;

int duty_time_1 = 0 ; 
int duty_time_2 = 0 ; 


int duty_counter = 0;


};
extern StringPeriphery string_manager;
