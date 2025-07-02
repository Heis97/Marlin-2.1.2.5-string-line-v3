//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M580() {
  if (parser.seen('S')) 
  {
   // string_manager.set_hv_v(parser.floatval('S'));
    //misis_i2c.handle_gcode(parser.floatval('V'), parser.seen('E'));
  } 
  if (parser.seen('G')) 
  {
   // string_manager.get_v_hv();
    //misis_i2c.handle_gcode(parser.floatval('V'), parser.seen('E'));
  } 
}
