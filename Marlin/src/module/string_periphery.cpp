#include "../MarlinCore.h"
#include "string_periphery.h"
#include "stepper.h"
#include "planner.h"
/*
#define TEMP_0_CS_PIN                     PF8   // Max31865 CS
  #define TEMP_0_SCK_PIN                    PA5
  #define TEMP_0_MISO_PIN                   PA6
  #define TEMP_0_MOSI_PIN                   PA7
*/
StringPeriphery  string_manager;

void StringPeriphery::init()
{
    mux_iic.begin();

    
  mcp4725_hv_v.begin();
  mcp4725_hv_v.setValue(0);

  mux_iic.setPort(7);
  mcp4725_hv_i.begin();
  mcp4725_hv_i.setValue(4095);
  mux_iic.disablePort(7);

  mux_iic.setPort(6);
  mcp4725_press.begin();
  mcp4725_press.setValue(0);
  mux_iic.disablePort(6);

  max6675_temp_cam_ext.begin();
  max6675_temp_cam_intern_1.begin();
  max6675_temp_cam_intern_2.begin();
  as5600_lin.begin(ENC_LIN_CS_PIN);


  pinMode(RELAY_0_PIN,OUTPUT);
  pinMode(RELAY_1_PIN,OUTPUT);
  pinMode(RELAY_2_PIN,OUTPUT);
  pinMode(RELAY_3_PIN,OUTPUT);

  digitalWrite(RELAY_0_PIN,1);
  digitalWrite(RELAY_1_PIN,1);
  digitalWrite(RELAY_2_PIN,1);
  digitalWrite(RELAY_3_PIN,1);

  analog_inp.setGain(0);

  mux_iic.begin();
  //a5048_test.SPI_setup();
};


float StringPeriphery::get_vel(){return 0;};

int16_t StringPeriphery::get_pos()
{
     int16_t pos  = as5600_lin.readAngle();
   //  Serial.println(pos);
    return pos;
};
void StringPeriphery::set_vel_lin(float v){};
void StringPeriphery::set_vel_powder(float v){};
void StringPeriphery::set_vel_gateway(float v){};
void StringPeriphery::set_turbine(float v){};
float StringPeriphery::get_temp_cam_ext()
{
    float val = max6675_temp_cam_ext.getTemperature();
    Serial.println(val);
    return 0;
};

float StringPeriphery::get_temp_cam_intern1()
{
    float val = max6675_temp_cam_intern_1.getTemperature();
    return 0;
};

float StringPeriphery::get_temp_cam_intern2()
{
    float val = max6675_temp_cam_intern_2.getTemperature();

    return 0;
};
float StringPeriphery::get_v_hv()
{
    uint16_t val = analog_inp.readADC(1);
    return val;
};
float StringPeriphery::get_a_hv()
{
    uint16_t val = analog_inp.readADC(2);
    return val;
};
float StringPeriphery::get_v_press()
{
    uint16_t val = analog_inp.readADC(0);
    return val;
};
void StringPeriphery::idle()
{
    unsigned long cur_time = millis();
    unsigned long dt = (cur_time- time_measure);
    //SERIAL_ECHOLNPGM("dt",dt);
   
    if(dt>20)
    {
        time_measure = cur_time;
        moves_planned =  planner.movesplanned();
        report_state();
        //Serial.println("string_manager.idle()");
    }

    //--------------------------------------------------------
    unsigned long dt_temp = (cur_time- time_measure_temp);
    if(dt_temp>200)
    {
        pressure = get_v_press();
        HV = get_v_hv();// mcp4725_hv_v.getValue();
        max6675_temp_cam_ext.read(); 
        max6675_temp_cam_intern_1.read(); 
        max6675_temp_cam_intern_2.read(); 
        temp_val_int1 = max6675_temp_cam_intern_1.getTemperature();
        temp_val_int2 = max6675_temp_cam_intern_2.getTemperature();
        temp_val_ext = max6675_temp_cam_ext.getTemperature();
        time_measure_temp = cur_time;

        manage_heat();
    }
    //--------------------------------------------------------
     unsigned long dt_enc = (cur_time- time_measure_enc);
     if(dt_enc>5)
     {
        string_cur_pos = get_pos();
        int d_pos = string_last_pos - string_cur_pos;
        if(d_pos>2000) d_pos = (4096 - string_last_pos )+ string_cur_pos;
        else if(d_pos<(-2000)) d_pos = (4096 + string_last_pos )- string_cur_pos;
        string_lenght = string_lenght + d_pos;

        string_last_pos = string_cur_pos;
        time_measure_enc = cur_time;
     }

     
};
void StringPeriphery::set_hv_v(uint16_t v)
{
    mcp4725_hv_v.setValue(v);
    
};
void StringPeriphery::set_hv_i(uint16_t v)
{
    mux_iic.setPort(7);
    mcp4725_hv_i.setValue(v);
    mux_iic.disablePort(7);
};
void StringPeriphery::set_press(uint16_t v)
{
     mux_iic.setPort(6);
     mcp4725_press.setValue(v);
     mux_iic.disablePort(6);

};

void StringPeriphery::set_reley_1(int v)
{
    int v_set = 0;
    if(v==0) v_set = 1;
    else v_set = 0;
    digitalWrite(RELAY_0_PIN,v_set);
    reley_1 = v;
    
};
void StringPeriphery::set_reley_2(int v)
{
   int v_set = 0;
    if(v==0) v_set = 1;
    else v_set = 0;
    digitalWrite(RELAY_1_PIN,v_set);
    reley_2 = v;
};

void StringPeriphery::set_reley_heater(int ind, int v)
{
    if(ind == 0) set_reley_1(v);
    else if(ind == 1) set_reley_2(v);
};

void StringPeriphery::set_reley_HV(int v)
{
    int v_set = 0;
    if(v==0) v_set = 1;
    else v_set = 0;
    digitalWrite(RELAY_2_PIN,v_set);
    reley_HV = v;
};
void StringPeriphery::set_reley_press(int v)
{
   int v_set = 0;
    if(v==0) v_set = 1;
    else v_set = 0;
    digitalWrite(RELAY_3_PIN,v_set);
    reley_press = v;
};

void StringPeriphery::set_heater_2(int v)
{
    digitalWrite(HEATER_2_PIN,v);
    turbo = v;
};
void StringPeriphery::set_heater_3(int v)
{
    digitalWrite(HEATER_3_PIN,v);
};

void StringPeriphery::report_state()
{
    Serial.print("string: ");
    Serial.print(temp_val_int1);
    Serial.print(" ");
    Serial.print(temp_val_int2);
    Serial.print(" ");
    Serial.print(temp_val_ext);
    Serial.print(" ");
    Serial.print(reley_1);
    Serial.print(" ");
    Serial.print(reley_2);
    Serial.print(" ");
    Serial.print(reley_HV);
    Serial.print(" ");
    Serial.print(reley_press);
    Serial.print(" ");
    Serial.print(string_lenght);
    Serial.print(" ");
    Serial.print(pressure);
    Serial.print(" ");
    Serial.print(HV);
    Serial.print(" ");
    Serial.print(turbo);
    Serial.print(" ");
    Serial.print(moves_planned);
    Serial.print(" ");
    Serial.print(time_measure);
    Serial.println(" ");
};


//float temp_dest = 0;
//int ind_sensor = 0;
//int heater_en = 0;

void StringPeriphery::manage_heat()
{
    if(heater_en == 1)
    {
        float cur_temp = 0;
        if(ind_sensor==0)
        {
            cur_temp = temp_val_ext;
            
        }
        else if(ind_sensor==1)
        {
            cur_temp = temp_val_int1;
            
        }
        else if(ind_sensor==2)
        {
            cur_temp = temp_val_int2;
        }
        else
        {
            set_reley_1(0);
            set_reley_2(0);
            return;
        }
        if(cur_temp<temp_dest+temp_hyst)
        {
            set_reley_1(1);
            set_reley_2(1);
        }
        else if(cur_temp>temp_dest-temp_hyst)
        {
            set_reley_1(0);
            set_reley_2(0);
        }
    }
    else
    {
        set_reley_1(0);
        set_reley_2(0);
    }


}
void StringPeriphery::manage_heat_duty()
{
    float temp_1 = temp_val_int1;
    float temp_2 = temp_val_int2;
    if(ind_sensor == 0)
    {
        temp_1 = temp_val_ext;
        temp_2 = temp_val_ext;  
    }
    if(heater_en == 1)
    {
        duty_1 = manage_heat_duty_single(0, temp_1,kp_1);
        duty_2 = manage_heat_duty_single(1, temp_2,kp_2);

        if(duty_1 < 0) heating_1 = false; else heating_1 = true;
        if(duty_2 < 0) heating_2 = false; else heating_2 = true;
    }
    else
    {
        set_reley_1(0);
        set_reley_2(0);
    }

    heat_pwm_control();
}

int StringPeriphery::manage_heat_duty_single(int ind, float temp,float kp)
{    
    bool heating = false;
    int duty = 0;
    if(temp<temp_dest+temp_hyst)
    {
        
        heating = true;
    }
    else if(temp>temp_dest-temp_hyst)
    {
        heating = false;
    }  
    else
    {
        heating = false;
    }

    if(!heating)
    {
        return -1;
    }

    duty = kp* abs(temp_dest-temp);
    if(duty>cycle_time-5) duty = cycle_time-5;
    

    return duty;
}
void StringPeriphery::heat_pwm_control()
{
    if(heating_1) heat_pwm_control_single(0, duty_counter, duty_1); else  set_reley_1(0);
    if(heating_2) heat_pwm_control_single(1, duty_counter, duty_2); else  set_reley_2(0);
    
    duty_counter++;
    if(duty_counter>cycle_time) 
    {
        duty_counter = 0;       
    }
};
void StringPeriphery::heat_pwm_control_single(int ind, int counter, int duty)
{
    if(counter==0)
    {
        set_reley_heater(ind,1);
    }
    else if(counter == duty)
    {
        set_reley_heater(ind,0);
    }

};



void StringPeriphery::set_heaters_enable(int v)
{
    heater_en = v;
};
void StringPeriphery::set_heaters_temp(float v)
{
    temp_dest = v;
};
void StringPeriphery::set_heaters_ind(int v)
{
    ind_sensor = v;
};
void StringPeriphery::set_reporting(bool state){
    reporting = state;
};


