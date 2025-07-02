//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M576() {
  if (parser.seen('L')) 
  {
    string_manager.set_vel_lin(parser.floatval('L'));
  } 
  if (parser.seen('S')) 
  {
    string_manager.string_lenght = 0;
  } 
  if (parser.seen('P')) 
  {
    string_manager.set_vel_powder(parser.floatval('P'));
  } 
  if (parser.seen('G')) 
  {
    string_manager.set_vel_gateway(parser.floatval('G'));
  } 

  if (parser.seen('I')) 
  {
    string_manager.init();
  } 

  if (parser.seen('R'))
  {
    if(parser.seenval('R')==0)  string_manager.set_reporting(false);
    if(parser.seenval('R')==1)  string_manager.set_reporting(true);
  }  
}
