//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M581() {
  if (parser.seen('X')) 
  {
        string_manager.k_m_x = parser.intval('X');
  } 

  if (parser.seen('B')) 
  {
        string_manager.k_m_b = parser.intval('B');
  } 

   if (parser.seen('D')) 
  {
        string_manager.dist_m = parser.floatval('D');
  } 

   if (parser.seen('L')) 
  {
        string_manager.buff_m = parser.intval('L');
  } 


  if (parser.seen('A')) 
  {
        string_manager.feed_pound_move = parser.seen('A');
  } 
  if (parser.seen('G')) 
  {
   // string_manager.get_v_hv();
    //misis_i2c.handle_gcode(parser.floatval('V'), parser.seen('E'));
  } 
}
