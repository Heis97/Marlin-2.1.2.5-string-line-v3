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
    mux_iic.begin(112U,Wire);
   
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
  as5600_lin  = AS5600(&Wire);
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
  // pinMode(DT_HX711_PIN,INPUT);
   //pinMode(CL_HX711_PIN,OUTPUT);
    //tensosensor.begin(DT_HX711_PIN,CL_HX711_PIN);
    //tensosensor.set_scale(127.15);
    //tensosensor.set_raw_mode();
    //tensosensor.tare();
};


float StringPeriphery::get_vel(){return 0;};

int16_t StringPeriphery::get_pos()
{
    int16_t pos  = as5600_lin.readAngle();
    //Serial.println(pos);
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
bool swap = false;
void StringPeriphery::idle()
{
    manage_motion();
    unsigned long cur_time = millis();
    unsigned long dt = (cur_time- time_measure);
    //SERIAL_ECHOLNPGM("dt",dt);
    unsigned long dt_enc = (cur_time- time_measure_enc);
    unsigned long dt_temp = (cur_time- time_measure_temp);
    if(dt>20)
    {
        time_measure = cur_time;
        moves_planned =  planner.movesplanned();
        
        report_state();
        
        /*if(swap){
            digitalWrite(PB8,1);
            
            swap = false;
        } 
        else
        {
            digitalWrite(PB8,0);
            swap = true;
        }*/
        //Serial.println("string_manager.idle()");
    }
    if(dt_enc>50)
     {
        //string_cur_pos = get_pos();
        //int d_pos = string_last_pos - string_cur_pos;
        //if(d_pos>2000) d_pos = (4096 - string_last_pos )+ string_cur_pos;
        //else if(d_pos<(-2000)) d_pos = (4096 + string_last_pos )- string_cur_pos;
        //string_lenght = string_lenght + d_pos;

        string_last_pos = string_cur_pos;
        
       time_measure_enc = cur_time;
     }

    //--------------------------------------------------------
    
    if(dt_temp>period_manage_ms)
    {
        //get_request_i2c(70);
        pressure = get_v_press();
        HV = get_v_hv();// mcp4725_hv_v.getValue();
        max6675_temp_cam_ext.read(); 
        max6675_temp_cam_intern_1.read(); 
        max6675_temp_cam_intern_2.read(); 
        temp_val_int1 = max6675_temp_cam_intern_1.getTemperature();
        temp_val_int2 = max6675_temp_cam_intern_2.getTemperature();
        temp_val_ext = max6675_temp_cam_ext.getTemperature();
       
        manage_heat_duty();
         time_measure_temp = cur_time;
    }


    //--------------------------------------------------------
     
     

     
};
int i2c_req_divider = 100;
int i2c_req_count = 100;
char rec_i2c[10]; 
void StringPeriphery::get_request_i2c(int adr)
{
  //if(i2c_req_count>i2c_req_divider)
  {
    char rec_i2c_int[10];// = "0000000000";
    //rec_i2c[10];
    int i_m =0; 
    int i_st =0; 
    int i_int =0; 
    bool st_done = false;
    Wire.requestFrom(adr,10);
    while(Wire.available() && i_m<10) {  // Read Received Datat From Slave Device
      int b = Wire.read();
      i_m++;
      if(b<254)
      {
        if(!st_done )
        {
          i_st = i_m;
          st_done = true;
        }
        rec_i2c_int[i_m] = (char)b;
        i_int++;
        /*Serial.print(b);
        Serial.print(" ");
        Serial.println(rec_i2c_int[i_m] );*/
      }
    }
    int ki=0;
    for(int i=i_st; i<i_int;i++)
    {
      rec_i2c[ki] = rec_i2c_int[i];ki++;
    }
    
    //Serial.println( rec_i2c);
    //Serial.println(i_m);
    i2c_req_count = 0;
    }
  i2c_req_count++;
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
    Serial.print(" ");
    Serial.print(duty_1);
    Serial.print(" ");
    Serial.print(duty_2);
    Serial.print(" ");
    Serial.print(rec_i2c);
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

        heat_pwm_control();
    }
    else
    {
        set_reley_1(0);
        set_reley_2(0);
    }

    
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

    duty = kp * abs(temp_dest-temp);
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


void StringPeriphery::manage_motion()
{
    int move = 0;
    feedrate_mm_s = 10;
    
    
    if(step_switch == 0 ) 
    {
        step_switch = 1;
        if(dir_switch == 0 ) dir_switch = 1;
        else dir_switch = 0;
    }    
    else step_switch = 0;

    if(planner.movesplanned()<buff_m)
    {
        if(recuperator_move == 1) { k_x = manage_axis(X_AXIS,vibr_x,k_x,k_m_x,dir_x,vibr_a_x);    move++;  };
        if(karet_move == 1) { k_y =manage_axis(Y_AXIS,vibr_y,k_y,k_m_y,dir_y,vibr_a_y);    move++;  };
        if(gateway_move == 1) {k_z = manage_axis(Z_AXIS,vibr_z,k_z,k_m_z,dir_z,vibr_a_z);    move++;  };
        if(string_move == 1) { k_e =manage_axis(E_AXIS,vibr_e,k_e,k_m_e,dir_e,vibr_a_e);    move++;  };
        if(feed_pound_move == 1) {k_a = manage_axis(I_AXIS,vibr_a,k_a,k_m_a,dir_a,vibr_a_a);    move++;  };
        if(string_vibro == 1) { k_b =manage_axis(J_AXIS,vibr_b,k_b,k_m_b,dir_b,vibr_a_b);    move++;  };
        if(string_move_second == 1) { k_c =manage_axis(K_AXIS,vibr_c,k_c,k_m_c,dir_c,vibr_a_c);    move++;  };

        if(move>0){
            prepare_line_to_destination(); 
            //planner.synchronize();
        };
    }
    //}
};

int StringPeriphery::manage_axis(AxisEnum Axis,  int vibr, int k, int k_m,int dir,int _vibr_a){
    int sign = 1;

    if(vibr==1) 
    {
        if(k_m - k <= _vibr_a)
        { 
            sign=-1; 
            k = 0;
        };
        //digitalWrite(step_mot,step_switch);
       // digitalWrite(dir_mot,dir_switch);
    }
    /*Serial.print(vibr);
    Serial.print(" ");
    Serial.print(k);
    Serial.print(" ");
    Serial.print(k_m);
    Serial.print(" ");
    Serial.print(dir);
    Serial.print(" ");
    Serial.print(_vibr_a);
    Serial.print(" ");
    Serial.print(sign);
    Serial.println(" ");*/
    destination[Axis] = current_position[Axis] + dir*sign*dist_m; 
    k++;
    return k;
};




