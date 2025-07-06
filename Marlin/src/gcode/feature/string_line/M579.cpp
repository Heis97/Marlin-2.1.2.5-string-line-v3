//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M579() {
  if (parser.seen('S') && parser.seen('I')) 
  {
    //string_manager.set_hv(parser.floatval('S'));
    switch (parser.intval('I'))
    {
    case 0: string_manager.set_reley_1(parser.intval('S')); break;
    case 1: string_manager.set_reley_2(parser.intval('S')); break;
    case 2: string_manager.set_reley_HV(parser.intval('S')); break;
    case 3: string_manager.set_reley_press(parser.intval('S')); break;    
    case 4: string_manager.set_heater_2(parser.intval('S')); break; 
    case 5: string_manager.set_heater_3(parser.intval('S')); break; 
    default:
      break;
    }
  } 
  if (parser.seen('G')) 
  {
    if(parser.intval('G')==0)
    {
      string_manager.get_temp_cam_ext(); 
    }
    else if(parser.intval('G')==1)
    {
      string_manager.get_temp_cam_intern1();
    }
    else if(parser.intval('G')==2)
    {
      string_manager.get_temp_cam_intern2();
    }
  }
  
  if (parser.seen('T')) 
  {
      string_manager.set_heaters_temp(parser.floatval('T'));
  }

  if (parser.seen('E')) 
  {
    string_manager.set_heaters_enable(parser.floatval('E'));
  }

  if (parser.seen('N')) 
  {
    string_manager.set_heaters_ind(parser.intval('N'));
  }

  if (parser.seen('K') ) 
  {
    string_manager.kp_1 = parser.floatval('K');
  }
  if (parser.seen('P') ) 
  {
    string_manager.kp_2 = parser.floatval('P');
  }

    
  if (parser.seen('L')) 
  {
      float time_s = parser.floatval('L');
      string_manager.cycle_time = (int)((1000*time_s) / (float)string_manager.period_manage_ms);
  }
}
