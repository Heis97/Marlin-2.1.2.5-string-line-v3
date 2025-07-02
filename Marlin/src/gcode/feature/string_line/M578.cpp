//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M578() {
  if (parser.seen('S')) 
  {
    string_manager.set_press(parser.intval('S'));
    //misis_i2c.handle_gcode(parser.floatval('V'), parser.seen('E'));
  } 
  if (parser.seen('G')) 
  {
    string_manager.get_v_press();
    //misis_i2c.handle_gcode(parser.floatval('V'), parser.seen('E'));
  } 
}
