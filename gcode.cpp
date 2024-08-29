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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * gcode.cpp - Temporary container for all gcode handlers
 *             Most will migrate to classes, by feature.
 */

#include "gcode.h"
GcodeSuite gcode;

#if ENABLED(WIFI_CUSTOM_COMMAND)
  extern bool wifi_custom_command(char * const command_ptr);
#endif

#include "parser.h"
#include "queue.h"
#include "../module/motion.h"

#if ENABLED(PRINTCOUNTER)
  #include "../module/printcounter.h"
#endif

#if ENABLED(HOST_PROMPT_SUPPORT)
  #include "../feature/host_actions.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../sd/cardreader.h"
  #include "../feature/powerloss.h"
#endif

#if ENABLED(CANCEL_OBJECTS)
  #include "../feature/cancel_object.h"
#endif

#include "../MarlinCore.h" // for idle()

millis_t GcodeSuite::previous_move_ms;

// Relative motion mode for each logical axis
static constexpr xyze_bool_t ar_init = AXIS_RELATIVE_MODES;
uint8_t GcodeSuite::axis_relative = (
    (ar_init.x ? _BV(REL_X) : 0)
  | (ar_init.y ? _BV(REL_Y) : 0)
  | (ar_init.z ? _BV(REL_Z) : 0)
  | (ar_init.e ? _BV(REL_E) : 0)
);

#if HAS_AUTO_REPORTING || ENABLED(HOST_KEEPALIVE_FEATURE)
  bool GcodeSuite::autoreport_paused; // = false
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  GcodeSuite::MarlinBusyState GcodeSuite::busy_state = NOT_BUSY;
  uint8_t GcodeSuite::host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
#endif

#if ENABLED(CNC_WORKSPACE_PLANES)
  GcodeSuite::WorkspacePlane GcodeSuite::workspace_plane = PLANE_XY;
#endif

#if ENABLED(CNC_COORDINATE_SYSTEMS)
  int8_t GcodeSuite::active_coordinate_system = -1; // machine space
  xyz_pos_t GcodeSuite::coordinate_system[MAX_COORDINATE_SYSTEMS];
#endif

/**
 * Get the target extruder from the T parameter or the active_extruder
 * Return -1 if the T parameter is out of range
 */
int8_t GcodeSuite::get_target_extruder_from_command() {
  if (parser.seenval('T')) {
    const int8_t e = parser.value_byte();
    if (e < EXTRUDERS) return e;
    SERIAL_ECHO_START();
    SERIAL_CHAR('M'); SERIAL_ECHO(parser.codenum);
    SERIAL_ECHOLNPAIR(" " STR_INVALID_EXTRUDER " ", int(e));
    return -1;
  }
  return active_extruder;
}

/**
 * Get the target e stepper from the T parameter
 * Return -1 if the T parameter is out of range or unspecified
 */
int8_t GcodeSuite::get_target_e_stepper_from_command() {
  const int8_t e = parser.intval('T', -1);
  if (WITHIN(e, 0, E_STEPPERS - 1)) return e;

  SERIAL_ECHO_START();
  SERIAL_CHAR('M'); SERIAL_ECHO(parser.codenum);
  if (e == -1)
    SERIAL_ECHOLNPGM(" " STR_E_STEPPER_NOT_SPECIFIED);
  else
    SERIAL_ECHOLNPAIR(" " STR_INVALID_E_STEPPER " ", int(e));
  return -1;
}

/**
 * Set XYZE destination and feedrate from the current GCode command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate, if included
 */
void GcodeSuite::get_destination_from_command() {
  xyze_bool_t seen = { false, false, false, false };

  #if ENABLED(CANCEL_OBJECTS)
    const bool &skip_move = cancelable.skipping;
  #else
    constexpr bool skip_move = false;
  #endif

  // Get new XYZ position, whether absolute or relative
  LOOP_XYZ(i) {
    if ( (seen[i] = parser.seenval(XYZ_CHAR(i))) ) {
      const float v = parser.value_axis_units((AxisEnum)i);
      if (skip_move)
        destination[i] = current_position[i];
      else
        destination[i] = axis_is_relative(AxisEnum(i)) ? current_position[i] + v : LOGICAL_TO_NATIVE(v, i);
    }
    else
      destination[i] = current_position[i];
  }

  // Get new E position, whether absolute or relative
  if ( (seen.e = parser.seenval('E')) ) {
    const float v = parser.value_axis_units(E_AXIS);
    destination.e = axis_is_relative(E_AXIS) ? current_position.e + v : v;
  }
  else
    destination.e = current_position.e;

  #if ENABLED(POWER_LOSS_RECOVERY) && !PIN_EXISTS(POWER_LOSS)
    // Only update power loss recovery on moves with E
    if (recovery.enabled && IS_SD_PRINTING() && seen.e && (seen.x || seen.y))
      recovery.save();
  #endif

  if (parser.linearval('F') > 0)
    feedrate_mm_s = parser.value_feedrate();

  #if ENABLED(PRINTCOUNTER)
    if (!DEBUGGING(DRYRUN) && !skip_move)
      print_job_timer.incFilamentUsed(destination.e - current_position.e);
  #endif

  // Get ABCDHI mixing factors
  #if BOTH(MIXING_EXTRUDER, DIRECT_MIXING_IN_G1)
    M165();
  #endif
}

/**
 * Dwell waits immediately. It does not synchronize. Use M400 instead of G4
 */
void GcodeSuite::dwell(millis_t time) {
  time += millis();
  while (PENDING(millis(), time)) idle();
}

/**
 * When G29_RETRY_AND_RECOVER is enabled, call G29() in
 * a loop with recovery and retry handling.
 */
#if HAS_LEVELING && ENABLED(G29_RETRY_AND_RECOVER)

  #ifndef G29_MAX_RETRIES
    #define G29_MAX_RETRIES 0
  #endif

  void GcodeSuite::G29_with_retry() {
    uint8_t retries = G29_MAX_RETRIES;
    while (G29()) { // G29 should return true for failed probes ONLY
      if (retries--) event_probe_recover();
      else {
        event_probe_failure();
        return;
      }
    }

    #if ENABLED(HOST_PROMPT_SUPPORT)
      host_action_prompt_end();
    #endif

    #ifdef G29_SUCCESS_COMMANDS
      process_subcommands_now_P(PSTR(G29_SUCCESS_COMMANDS));
    #endif
  }

#endif // HAS_LEVELING && G29_RETRY_AND_RECOVER

//
// Placeholders for non-migrated codes
//
#if ENABLED(M100_FREE_MEMORY_WATCHER)
  extern void M100_dump_routine(PGM_P const title, const char * const start, const char * const end);
#endif

/**
 * Process the parsed command and dispatch it to its handler
 */
bool GcodeSuite::process_parsed_command(const bool no_ok/*=false*/) {
  KEEPALIVE_STATE(IN_HANDLER);

  // Handle a known G, M, or T
  switch (parser.command_letter) {
    case 'G': switch (parser.codenum) {

      case 0: case 1: G0_G1(                                      // G0: Fast Move, G1: Linear Move
                        #if IS_SCARA || IS_DEXARM || defined(G0_FEEDRATE)
                          parser.codenum == 0
                        #endif
                      );
                      break;

      #if ENABLED(ARC_SUPPORT) && DISABLED(SCARA)
        case 2: case 3: G2_G3(parser.codenum == 2); break;        // G2: CW ARC, G3: CCW ARC
      #endif

      case 4: G4(); break;                                        // G4: Dwell

      #if ENABLED(BEZIER_CURVE_SUPPORT)
        case 5: G5(); break;                                      // G5: Cubic B_spline
      #endif

      case 6: G6();  break;   // relative pipette aspirate/dispense. Parameter: Volume in µl
      case 7: G7();  break;   // discard pipette tip
      case 8: G8();  break;   // home pipette

      #if ENABLED(NOZZLE_PARK_FEATURE)
        case 27: G27(); break;                                    // G27: Nozzle Park
      #endif

      case 28: G28(); break;                                      // G28: Home one or more axes

      #if HAS_LEVELING
        case 29:                                                  // G29: Bed leveling calibration
          #if ENABLED(G29_RETRY_AND_RECOVER)
            G29_with_retry();
          #else
            G29();
          #endif
          break;
      #endif // HAS_LEVELING

      #if ENABLED(CNC_COORDINATE_SYSTEMS)
        case 53: G53(); break;                                    // G53: (prefix) Apply native workspace
        case 54: G54(); break;                                    // G54: Switch to Workspace 1
        case 55: G55(); break;                                    // G55: Switch to Workspace 2
        case 56: G56(); break;                                    // G56: Switch to Workspace 3
        case 57: G57(); break;                                    // G57: Switch to Workspace 4
        case 58: G58(); break;                                    // G58: Switch to Workspace 5
        case 59: G59(); break;                                    // G59.0 - G59.3: Switch to Workspace 6-9
      #endif

      case 90: set_relative_mode(false); break;                   // G90: Absolute Mode
      case 91: set_relative_mode(true);  break;                   // G91: Relative Mode

      case 92: G92(); break;                                      // G92: Set current axis position(s)

      #if HAS_MESH
        case 42: G42(); break;                                    // G42: Coordinated move to a mesh point
      #endif

      default: return false;//parser.unknown_command_warning(); break;
    }
    break;

    case 'M': switch (parser.codenum) {

      #if HAS_RESUME_CONTINUE
        case 0:                                                   // M0: Unconditional stop - Wait for user button press on LCD
        case 1: M0_M1(); break;                                   // M1: Conditional stop - Wait for user button press on LCD
      #endif
      case 2:  M2(); break;                                              // M2: Sync positions between encoder and stepper

      case 17: M17(); break;                                      // M17: Enable all stepper motors

      case 31: M31(); break;                                      // M31: Report time since the start of SD print or last M109
      case 42: M42(); break;                                      // M42: Change pin state

      #if ENABLED(PINS_DEBUGGING)
        case 43: M43(); break;                                    // M43: Read pin state
      #endif

      case 75: M75(); break;                                      // M75: Start print timer
      case 76: M76(); break;                                      // M76: Pause print timer
      case 77: M77(); break;                                      // M77: Stop print timer

      case 101: M101(); break;                                    // M101: Emergency Stop

      #if EXTRUDERS
        case 104: M104(); break;                                  // M104: Set hot end temperature
        case 109: M109(); break;                                  // M109: Wait for hotend temperature to reach target
      #endif

      #if FAN_COUNT > 0
        case 106: M106(); break;                                  // M106: Fan On
        case 107: M107(); break;                                  // M107: Fan Off
      #endif

      case 110: M110(); break;                                    // M110: Set Current Line Number
      case 111: M111(); break;                                    // M111: Set debug level

      #if DISABLED(EMERGENCY_PARSER)
        case 108: M108(); break;                                  // M108: Cancel Waiting
        case 112: M112(); break;                                  // M112: Full Shutdown
        case 410: M410(); break;                                  // M410: Quickstop - Abort all the planned moves.
        #if ENABLED(HOST_PROMPT_SUPPORT)
          case 876: M876(); break;                                // M876: Handle Host prompt responses
        #endif
      #else
        case 108: case 112: case 410:
        #if ENABLED(HOST_PROMPT_SUPPORT)
          case 876:
        #endif
        break;
      #endif

      #if ENABLED(HOST_KEEPALIVE_FEATURE)
        case 113: M113(); break;                                  // M113: Set Host Keepalive interval
      #endif

      #if ENABLED(AUTO_REPORT_TEMPERATURES) && HAS_TEMP_SENSOR
        case 155: M155(); break;                                  // M155: Set temperature auto-report interval
      #endif

      #if ENABLED(PARK_HEAD_ON_PAUSE)
        case 125: M125(); break;                                  // M125: Store current position and move to filament change position
      #endif

      case 81: M81(); break;                                      // M81: Turn off Power, including Power Supply, if possible

      case 82: M82(); break;                                      // M82: Set E axis normal mode (same as other axes)
      case 83: M83(); break;                                      // M83: Set E axis relative mode
      case 18: case 84: M18_M84(); break;                         // M18/M84: Disable Steppers / Set Timeout
      case 85: M85(); break;                                      // M85: Set inactivity stepper shutdown timeout
      case 92: M92(); break;                                      // M92: Set the steps-per-unit for one or more axes
      case 114: M114(); break;                                    // M114: Report current position
      case 115: M115(); break;                                    // M115: Report capabilities
      case 117: M117(); break;                                    // M117: Set LCD message text, if possible
      case 118: M118(); break;                                    // M118: Display a message in the host console
      case 119: M119(); break;                                    // M119: Report endstop states
      case 120: M120(); break;                                    // M120: Enable endstops
      case 121: M121(); break;                                    // M121: Disable endstops

      case 888: M888(); break;
      case 889: M889(); break;                                    // M889
      case 890: M890(); break;                                    // M890
      case 891: M891(); break;                                    // M891
      case 892: M892(); break;                                    // M892
      case 893: M893(); break;                                    // M893
      case 894: M894(); break;                                    // M894
      case 895: M895(); break;                                    // M895
      case 896: M896(); break;                                    // M896
      case 897: M897(); break;                                    // M897
      case 898: M898(); break;                                    // M898 get encoder status register
      case 899: M899(); break;                                    // M899 get encoder conf register
      //case 1000: M1000(); break;                                    // M1000
      //case 1001: M1001(); break;                                    // M1001
      case 1111: M1111(); break;                                    // M1111
      case 1112: M1112(); break;                                    // M1112
      //case 1113: M1113(); break;                                    // M1113
      case 1114: M1114(); break;                                    // M1114
      case 1115: M1115(); break;                                    // M1115
      case 1116: M1116(); break;                                    // M1116
      case 1117: M1117(); break;                                    // M1117
      case 1118: M1118(); break;                                    // M1118
      case 1119: M1119(); break;                                    // M1119
      case 2000: M2000(); break;                                    // M2000
      case 2001: M2001(); break;                                    // M2001
      case 2002: M2002(); break;                                    // M2002
      case 2003: M2003(); break;                                    // M2003
      case 2004: M2004(); break;                                    // M2004
      //case 2005: M2005(); break;                                    // M2005
      // case 2006: M2006(); break;                                    // M2006
      case 2007: M2007(); break;                                    // M2007
      case 2010: M2010(); break;                                    // M2010
      case 2011: M2011(); break;                                    // M2011
	    // case 2012: M2012(); break;                                    // M2012
      // case 2100: M2100(); break;                                    // M2100
      // case 2101: M2101(); break;                                    // M2101
      // case 2102: M2102(); break;                                    // M2102
      // case 2103: M2103(); break;                                    // M2103
      case 5201314: M5201314(); break;                              // M5010000
      case 5010000: M5010000(); break;                              // M5010000


      #if DISABLED(NO_VOLUMETRICS)
        case 200: M200(); break;                                  // M200: Set filament diameter, E to cubic units
      #endif

      case 201: M201(); break;                                    // M201: Set max acceleration for print moves (units/s^2)
      case 203: M203(); break;                                    // M203: Set max feedrate (units/sec)
      case 204: M204(); break;                                    // M204: Set acceleration
      case 205: M205(); break;                                    // M205: Set advanced settings

      #if HAS_SOFTWARE_ENDSTOPS
        case 211: M211(); break;                                  // M211: Enable, Disable, and/or Report software endstops
      #endif

      case 220: M220(); break;                                    // M220: Set Feedrate Percentage: S<percent> ("FR" on your LCD)

      #if EXTRUDERS
        case 221: M221(); break;                                  // M221: Set Flow Percentage
      #endif

      case 226: M226(); break;                                    // M226: Wait until a pin reaches a state

      #if ENABLED(PIDTEMP)
        case 301: M301(); break;                                  // M301: Set hotend PID parameters
      #endif

      #if HAS_PID_HEATING
        case 303: M303(); break;                                  // M303: PID autotune
      #endif

      case 400: M400(); break;                                    // M400: Finish all moves

      case 401: M401(); break;
      case 402: M402(); break;

      #if HAS_LEVELING
        case 420: M420(); break;                                  // M420: Enable/Disable Bed Leveling
      #endif

      #if HAS_MESH
        case 421: M421(); break;                                  // M421: Set a Mesh Bed Leveling Z coordinate
      #endif

      case 500: M500(); break;                                    // M500: Store settings in EEPROM
      case 501: M501(); break;                                    // M501: Read settings from EEPROM
      case 502: M502(); break;                                    // M502: Revert to default settings
      #if DISABLED(DISABLE_M503)
        case 503: M503(); break;                                  // M503: print settings currently in memory
      #endif
      #if ENABLED(EEPROM_SETTINGS)
        case 504: M504(); break;                                  // M504: Validate EEPROM contents
      #endif

      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case 600: M600(); break;                                  // M600: Pause for Filament Change
        case 603: M603(); break;                                  // M603: Configure Filament Change
      #endif

      #if HAS_TRINAMIC_CONFIG
        case 122: M122(); break;                                  // M122: Report driver configuration and status
        case 222: M222(); break;                                  // M222: report stall guard value
        case 906: M906(); break;                                  // M906: Set motor current in milliamps using axis codes X, Y, Z, E
        #if HAS_STEALTHCHOP
          case 569: M569(); break;                                // M569: Enable stealthChop on an axis.
        #endif

        #if ENABLED(MONITOR_DRIVER_STATUS)
          case 911: M911(); break;                                // M911: Report TMC2130 prewarn triggered flags
          case 912: M912(); break;                                // M912: Clear TMC2130 prewarn triggered flags
        #endif

        #if USE_SENSORLESS
          case 914: M914(); break;                                // M914: Set StallGuard sensitivity.
        #endif
      #endif

      #if IS_DEXARM
        case 2012: M2012(); break;                                // M2012: DexArm: Get device ID
        case 2013: M2013(); break;                                // M2013: retreive pipette ID
        case 2014: M2014(); break;                                // M2014: retreive pipette max volume in ul
        case 2015: M2015(); break;                                // M2015: set pump calibration values and/or store them in pipette EEPROM
        case 2016: M2016(); break;                                // M2016: retrieve pump calibration parameters
        case 2914: M2914(); break;                                // M2914: DexArm: Set StallGuard sensitivity for Homing (H), Acceleration (A), Deceleration (D), and Cruise (C).
      #endif

      case 997: M997(); break;                                  // M997: Perform in-application firmware update
      case 999: M999(); break;                                    // M999: Restart after being Stopped

      default: return false;//parser.unknown_command_warning(); break;
    }
    break;

    case 'T': T(parser.codenum); break;                           // Tn: Tool Change

    default:
      return false;//parser.unknown_command_warning();
  }

  
  // if we found the command, return true
  return true;

  // if (!no_ok) {
  //   SERIAL_ECHO_START();
  //   SERIAL_ECHOLN("TEST MESSAGE BEFORE OK");
  //   queue.ok_to_send();
  // }
}

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void GcodeSuite::process_next_command() {
  char * const current_command = queue.command_buffer[queue.index_r];

  PORT_REDIRECT(queue.port[queue.index_r]);

  #if ENABLED(POWER_LOSS_RECOVERY)
    recovery.queue_index_r = queue.index_r;
  #endif

  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLN(current_command);
    #if ENABLED(M100_FREE_MEMORY_DUMPER)
      SERIAL_ECHOPAIR("slot:", queue.index_r);
      M100_dump_routine(PSTR("   Command Queue:"), &queue.command_buffer[0][0], &queue.command_buffer[BUFSIZE - 1][MAX_CMD_SIZE - 1]);
    #endif
  }

  // Parse the next command in the queue
  parser.parse(current_command);
  if (process_parsed_command()) {
    queue.response_pending = true;
  }
}

/**
 * Run a series of commands, bypassing the command queue to allow
 * G-code "macros" to be called from within other G-code handlers.
 */

void GcodeSuite::process_subcommands_now_P(PGM_P pgcode) {
  char * const saved_cmd = parser.command_ptr;        // Save the parser state
  for (;;) {
    PGM_P const delim = strchr_P(pgcode, '\n');       // Get address of next newline
    const size_t len = delim ? delim - pgcode : strlen_P(pgcode); // Get the command length
    char cmd[len + 1];                                // Allocate a stack buffer
    strncpy_P(cmd, pgcode, len);                      // Copy the command to the stack
    cmd[len] = '\0';                                  // End with a nul
    parser.parse(cmd);                                // Parse the command
    process_parsed_command(true);                     // Process it
    if (!delim) break;                                // Last command?
    pgcode = delim + 1;                               // Get the next command
  }
  parser.parse(saved_cmd);                            // Restore the parser state
}

void GcodeSuite::process_subcommands_now(char * gcode) {
  char * const saved_cmd = parser.command_ptr;        // Save the parser state
  for (;;) {
    char * const delim = strchr(gcode, '\n');         // Get address of next newline
    if (delim) *delim = '\0';                         // Replace with nul
    parser.parse(gcode);                              // Parse the current command
    if (delim) *delim = '\n';                         // Put back the newline
    process_parsed_command(true);                     // Process it
    if (!delim) break;                                // Last command?
    gcode = delim + 1;                                // Get the next command
  }
  parser.parse(saved_cmd);                            // Restore the parser state
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting commands.
   */
  void GcodeSuite::host_keepalive() {
    const millis_t ms = millis();
    static millis_t next_busy_signal_ms = 0;
    if (!autoreport_paused && host_keepalive_interval && busy_state != NOT_BUSY) {
      if (PENDING(ms, next_busy_signal_ms)) return;
      switch (busy_state) {
        case IN_HANDLER:
        case IN_PROCESS:
          SERIAL_ECHO_MSG(STR_BUSY_PROCESSING);
          // SERIAL_ECHOLN(STR_BUSY_PROCESSING);    // goodbot - Martin L. - 2023-07-04 - tested printing via SERIAL_PORT_2
          break;
        case PAUSED_FOR_USER:
          SERIAL_ECHO_MSG(STR_BUSY_PAUSED_FOR_USER);
          break;
        case PAUSED_FOR_INPUT:
          SERIAL_ECHO_MSG(STR_BUSY_PAUSED_FOR_INPUT);
          break;
        default:
          break;
      }
    }
    next_busy_signal_ms = ms + host_keepalive_interval * 1000UL;
  }

#endif // HOST_KEEPALIVE_FEATURE
