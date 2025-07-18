//M576 linear speed control
#include "../../../inc/MarlinConfig.h"
#include "../../gcode.h"
#include "../../../module/planner.h"
#include "../../../module/string_periphery.h"

void GcodeSuite::M580() {
  if (parser.seen('X')) 
  {
        string_manager.recuperator_move = parser.intval('X');
  } 

  if (parser.seen('X') && parser.seen('V')) 
  {
        string_manager.vibr_x = parser.intval('V');
  } 
 if (parser.seen('X') && parser.seen('D')) 
  {
        string_manager.dir_x = parser.intval('D');
  } 

  if (parser.seen('X') && parser.seen('W')) 
  {
        string_manager.vibr_a_x = parser.intval('W');
  } 

//-------------------Y-----------------------
  if (parser.seen('Y')) 
  {
        string_manager.karet_move = parser.intval('Y');
  } 

  if (parser.seen('Y') && parser.seen('V')) 
  {
        string_manager.vibr_y = parser.intval('V');
  } 
 if (parser.seen('Y') && parser.seen('D')) 
  {
        string_manager.dir_y = parser.intval('D');
  } 

//-------------------Z-----------------------
  if (parser.seen('Z')) 
  {
        string_manager.gateway_move = parser.intval('Z');
  } 

  if (parser.seen('Z') && parser.seen('V')) 
  {
        string_manager.vibr_z = parser.intval('V');
  } 
 if (parser.seen('Z') && parser.seen('D')) 
  {
        string_manager.dir_z = parser.intval('D');
  } 
//-------------------E-----------------------
  if (parser.seen('E')) 
  {
        string_manager.string_move = parser.intval('E');
  } 

  if (parser.seen('E') && parser.seen('V')) 
  {
        string_manager.vibr_e = parser.intval('V');
  } 
 if (parser.seen('E') && parser.seen('D')) 
  {
        string_manager.dir_e = parser.intval('D');
  } 

//-------------------A-----------------------
  if (parser.seen('A')) 
  {
        string_manager.feed_pound_move = parser.intval('A');
  } 

  if (parser.seen('A') && parser.seen('V')) 
  {
        string_manager.vibr_a = parser.intval('V');
  } 
 if (parser.seen('A') && parser.seen('D')) 
  {
        string_manager.dir_a = parser.intval('D');
  } 
//-------------------B-----------------------
  if (parser.seen('B')) 
  {
        string_manager.string_vibro = parser.intval('B');
  } 

  if (parser.seen('B') && parser.seen('V')) 
  {
        string_manager.vibr_b = parser.intval('V');
  } 
 if (parser.seen('B') && parser.seen('D')) 
  {
        string_manager.dir_b = parser.intval('D');
  } 
//-------------------C-----------------------
  if (parser.seen('C')) 
  {
        string_manager.string_move_second = parser.intval('C');
  } 

  if (parser.seen('C') && parser.seen('V')) 
  {
        string_manager.vibr_c = parser.intval('V');
  } 
 if (parser.seen('C') && parser.seen('D')) 
  {
        string_manager.dir_c = parser.intval('D');
  } 
}
