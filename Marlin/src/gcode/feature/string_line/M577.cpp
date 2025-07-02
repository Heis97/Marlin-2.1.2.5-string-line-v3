//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M577() {
  if (parser.seen('V')) 
  {
    string_manager.set_hv_v(parser.intval('V'));
    //misis_i2c.handle_gcode(parser.floatval('V'), parser.seen('E'));
  } 
  if (parser.seen('I')) 
  {
    string_manager.set_hv_i(parser.intval('I'));
    //misis_i2c.handle_gcode(parser.floatval('V'), parser.seen('E'));
  } 
  if (parser.seen('G')) 
  {
    string_manager.get_v_hv();
    //misis_i2c.handle_gcode(parser.floatval('V'), parser.seen('E'));
  } 
}
