/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../inc/MarlinConfig.h"

template <typename Helper>
struct AutoReporter {
  millis_t next_report_ms;
  uint8_t report_interval;
  #if HAS_MULTI_SERIAL
    SerialMask report_port_mask;
    AutoReporter() : report_port_mask(SerialMask::All) {}
  #endif

  inline void set_interval(uint16_t milliseconds, const uint16_t limit=60) {
    report_interval = milliseconds;
    next_report_ms = millis() + milliseconds;
  }

  inline void tick() {
    if (!report_interval) return;
    const millis_t ms = millis();
    if (ELAPSED(ms, next_report_ms)) {
      next_report_ms = ms + report_interval;
      PORT_REDIRECT(report_port_mask);
      Helper::report();
      PORT_RESTORE();
    }
  }
};
