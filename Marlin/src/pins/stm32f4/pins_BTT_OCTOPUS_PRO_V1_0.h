/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#define BOARD_INFO_NAME "BTT OCTOPUS PRO V1.0"

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PF4   // TH0

//#if TEMP_SENSOR_0_IS_MAX31865
#define ENC_LIN_CS_PIN                     PB2 
  #define TEMP_0_CS_PIN                     PA4   // Max31865 CS//   PF8 internal
  #define TEMP_1_CS_PIN                     PC15   // Max31865 CS//   PF8 internal
  #define TEMP_2_CS_PIN                     PB1   // Max31865 CS//   PF8 internal
  #define TEMP_0_SCK_PIN                    PA5
  #define TEMP_0_MISO_PIN                   PA6
  #define TEMP_0_MOSI_PIN                   PA7
  #define SOFTWARE_SPI                            // Max31865 and LCD SD share a set of SPIs, Set SD to softwareSPI for Max31865
  #define FORCE_SOFT_SPI

//#endif

#if !defined(Z_MIN_PROBE_PIN) && DISABLED(BLTOUCH)
  #define Z_MIN_PROBE_PIN                   PC5   // Probe (Proximity switch) port
#endif



#include "pins_BTT_OCTOPUS_V1_common_I.h"

#define RELAY_0_PIN  PA9
#define RELAY_1_PIN  PA10
#define RELAY_2_PIN  PB6
#define RELAY_3_PIN  PB7