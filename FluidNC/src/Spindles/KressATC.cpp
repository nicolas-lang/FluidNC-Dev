// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
Example config section

kress_atc:
    atc_valve_pin: gpio.4
    atc_dustoff_pin: gpio.16
    ets_dustoff_pin: gpio.27
    ets_mpos_mm: 157.00 142.00 -31.00
    tool1_mpos_mm: 197.0 142.0 -26.0
    tool2_mpos_mm: 237.0 142.0 -26.0
    tool3_mpos_mm: 277.0 142.0 -26.0
    tool4_mpos_mm: 317.0 142.0 -26.0
    direction_pin: NO_PIN
    output_pin: gpio.26
    enable_pin: NO_PIN
    disable_with_s0: false
    s0_with_disable: true
    spinup_ms: 3000
    spindown_ms: 4000
    tool_num: 0
    speed_map: 0=0.000% 0=100.000% 1=100.000%


TO DO
    Turn off soft limits during tool changes.
    This would allow the rack to be placed outside of soft limit zone
    This would prevent user from damaging the rack

    Need to fail and quit if no probe defined at the time of probing
    change pickup logic to macro-file

Limitations
    all code assumes a XYZ coordinate system
    right now, the code just assumes a vertical pickup/dropoff which prevents using tools with a diameter larger then the ATC-collet
*/

#include "KressATC.h"
#include "../Protocol.h"
#include "../GCode.h"
#include "../Uart.h"
#include "../Machine/MachineConfig.h"
#include "../Limits.h"  // limitsMaxPosition

// ========================= KressATC ==================================

namespace Spindles {

    void KressATC::init() {
        OnOff::init();
        _atc_ok = atc_init();
    }

    bool KressATC::atc_init() {
        // Spindle spindown delay is required for safety and to prevent ATC damage
        if (spindle->_spindown_ms == 0) {
            log_error("ATC operation requires a Spindle spindown > 0ms");
            return false;
        }

        _atc_valve_pin.setAttr(Pin::Attr::Output);
        _atc_dustoff_pin.setAttr(Pin::Attr::Output);
        _toolsetter_dustoff.setAttr(Pin::Attr::Output);

        // the atc valve must be defined
        if (!_atc_valve_pin.defined()) {
            log_error("ATC: " << _atc_valve_pin.name() << " must be defined");
            return false;
        }

        log_info("ATC Valve:" << _atc_valve_pin.name());
        log_info("ATC Dustoff Valve:" << _atc_dustoff_pin.name());
        log_info("ATC Toolsetter Dustoff Valve:" << _toolsetter_dustoff.name());

        // determine top of z for safest XY travel above things
        auto axisConfig = config->_axes->_axis[Z_AXIS];
        top_of_z        = limitsMaxPosition(Z_AXIS) - axisConfig->_motors[0]->_pulloff;

        //parse locations
        if (_ets_mpos.size() != 3) {
            log_error("ATC ETS mpos wrong");
            return false;  // failed
        }

        tool[ETS_INDEX].mpos[X_AXIS] = _ets_mpos.at(0);
        tool[ETS_INDEX].mpos[Y_AXIS] = _ets_mpos.at(1);
        tool[ETS_INDEX].mpos[Z_AXIS] = _ets_mpos.at(2);

        for (int i = 0; i < TOOL_COUNT; i++) {
            if (_tool_mpos[i].size() != 3) {
                log_error("ATC Tool mpos wrong. Tool:" << i + 1);
                return false;  // failed
            }
            tool[i + 1].mpos[X_AXIS] = _tool_mpos[i].at(0);
            tool[i + 1].mpos[Y_AXIS] = _tool_mpos[i].at(1);
            tool[i + 1].mpos[Z_AXIS] = _tool_mpos[i].at(2);
        }
        return true;
    }

    void KressATC::tool_change_manual(uint8_t new_tool) {
        log_info("Manual tool change: Toggle ATC");
        set_ATC_state(true);
        gc_exec_linef(true, "G4 P2.0");
        set_ATC_state(false);
        current_tool = new_tool;
    }

    void KressATC::tool_preselect(uint8_t new_tool) {
        log_warn("Tool preselect not implemented:" << new_tool);
    }

    bool KressATC::tool_change(uint8_t new_tool, bool pre_select) {
        log_debug(name() << " tool change to:" << new_tool << " From:" << current_tool << " Preselect:" << pre_select);

        if (!is_ATC_ok()) {
            log_error("ATC not initialized, toolchange failed");
            return false;
        }

        if (new_tool > MANUAL_CHG) {
            log_error(name() << ":invalid tool number:" << new_tool);
            return false;
        }

        if (pre_select) {
            tool_preselect(new_tool);
            return true;
        }

        // wait for all previous moves to complete
        protocol_buffer_synchronize();
        // Save State & Position before the tool change
        bool  was_incremental_mode   = (gc_state.modal.distance == Distance::Incremental);
        bool  spindle_was_on         = (gc_state.modal.spindle != SpindleState::Disable);
        bool  coolant_state_flood    = (gc_state.modal.coolant.Flood);
        bool  coolant_state_mist     = (gc_state.modal.coolant.Mist);
        float saved_mpos[MAX_N_AXIS] = {};
        motor_steps_to_mpos(saved_mpos, get_motor_steps());

        //Handle manual change using the ATC
        if (current_tool == MANUAL_CHG || new_tool == MANUAL_CHG) {
            if (spindle_was_on) {
                log_error("Spindle must not be active for a manual change");
                return false;
            }
            if ((current_tool != NO_TOOL && new_tool != NO_TOOL) && (current_tool != MANUAL_CHG && new_tool != MANUAL_CHG)) {
                log_error("MANUAL_CHG sequences can only change from or to (NO_TOOL, MANUAL_CHG)");
                return false;
            }
            tool_change_manual(new_tool);
            return true;
        }

        // ============= Start of automated tool change ====================
        if (coolant_state_flood || coolant_state_mist) {
            gc_exec_linef(true, "M9");
        }
        if (spindle_was_on) {
            gc_exec_linef(true, "M5");
        }

        goto_top_of_z();

        // returnTool (if there is one).
        if (current_tool != NO_TOOL) {
            return_tool(current_tool);
        }

        // PickupTool (if there is one)
        if (new_tool != NO_TOOL) {
            take_tool(new_tool);
        }

        // check the length of the tool
        if (!atc_toolsetter_probe()) {
            return false;
        }

        // ================== return old states ===================
        // return to saved mpos in XY
        gc_exec_linef(false, "G53 G0 X%0.3f Y%0.3f Z%0.3f", saved_mpos[X_AXIS], saved_mpos[Y_AXIS], top_of_z);
        // If the spindle was on before we started, we need to turn it back on.
        if (spindle_was_on) {
            gc_exec_linef(false, "M3");  // spindle should handle spinup delay
        }
        if (coolant_state_mist) {
            gc_exec_linef(true, "M7");
        }
        if (coolant_state_flood) {
            gc_exec_linef(true, "M8");
        }

        // return to saved mpos in Z if it is not outside of work area.
        gc_exec_linef(false, "G53 G0 Z%0.3f", saved_mpos[Z_AXIS] + gc_state.tool_length_offset);

        //return G90/91 to saved state
        if ((gc_state.modal.distance == Distance::Incremental) && was_incremental_mode == false)
            gc_exec_linef(false, "G90");
        if ((gc_state.modal.distance != Distance::Incremental) && was_incremental_mode == true)
            gc_exec_linef(false, "G91");

        return true;
    }

    bool KressATC::take_tool(uint8_t tool_num) {
        log_debug("Get tool: " << tool_num);
        go_above_tool(tool_num);
        set_ATC_state(true);  // open ATC
        gc_exec_linef(true, "G4 P%0.2f", 0.25);
        gc_exec_linef(true, "G53 G0 Z%0.3f", tool[tool_num].mpos[Z_AXIS]);  // drop down to tool
        gc_exec_linef(true, "G4 P%0.2f", 0.25);
        set_ATC_state(false);                              // Close ATC
        gc_exec_linef(true, "G4 P%0.2f", TOOL_GRAB_TIME);  // wait for grab to complete and settle
        current_tool = tool_num;
        goto_top_of_z();
        return true;
    }

    bool KressATC::return_tool(uint8_t tool_num) {
        log_debug("Return tool: " << tool_num);
        go_above_tool(tool_num);
        gc_exec_linef(true, "G53 G0 Z%0.3f", tool[tool_num].mpos[Z_AXIS]);  // drop down to tool
        set_ATC_state(true);
        gc_exec_linef(true, "G53 G0 Z%0.3f", _empty_safe_z);
        set_ATC_state(false);  // close ATC
        current_tool = NO_TOOL;
        return true;
    }

    void KressATC::go_above_tool(uint8_t tool_num) {
        goto_top_of_z();
        gc_exec_linef(false, "G53 G0 X%0.3f Y%0.3f", tool[tool_num].mpos[X_AXIS], tool[tool_num].mpos[Y_AXIS]);
    }

    bool KressATC::set_ATC_state(bool open) {
        if (gc_state.modal.spindle != SpindleState::Disable) {
            log_error("Spindle active when trying to operate ATC");
            return false;
        }
        _atc_valve_pin.synchronousWrite(open);
        return true;
    }

    void KressATC::atc_ETS_dustoff() {
        _atc_dustoff_pin.synchronousWrite(true);
        gc_exec_linef(true, "G4 P%0.2f", 0.5);
        _atc_dustoff_pin.synchronousWrite(false);
    }

    bool KressATC::atc_toolsetter_probe() {
        float probe_to;  // Calculated work position
        float probe_position[MAX_N_AXIS];

        atc_ETS_dustoff();

        goto_top_of_z();
        gc_exec_linef(true, "G53 G0 X%0.3f Y%0.3f", tool[ETS_INDEX].mpos[X_AXIS], tool[ETS_INDEX].mpos[Y_AXIS]);
        float wco = gc_state.coord_system[Z_AXIS] + gc_state.coord_offset[Z_AXIS] + gc_state.tool_length_offset;
        probe_to  = tool[ETS_INDEX].mpos[Z_AXIS] - wco;

        // https://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G38-probe
        tool_setter_probing = true;
        gc_exec_linef(true, "G38.2 F%0.3f Z%0.3f", PROBE_FEEDRATE, probe_to);
        tool_setter_probing = false;

        // Was probe successful?
        if (sys.state == State::Alarm) {
            auto msg = (lastAlarm == ExecAlarm::ProbeFailInitial) ? "ATC Probe Switch Error" : "ATC Probe Missing Tool Error";
            log_error(msg);
            return false;
        }

        motor_steps_to_mpos(probe_position, probe_steps);
        tool[current_tool].offset[Z_AXIS] = probe_position[Z_AXIS];  // Get the Z height ...

        if (zeroed_tool_index != 0) {
            float tlo = tool[current_tool].offset[Z_AXIS] - tool[zeroed_tool_index].offset[Z_AXIS];
            log_info("ATC Tool No:" << current_tool << " TLO:" << tlo);
            // https://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G43_1
            gc_exec_linef(false, "G43.1 Z%0.3f", tlo);  // raise up
        }
        goto_top_of_z();
        return true;
    }

    bool KressATC::is_ATC_ok() {
        return _atc_ok;
    }

    void KressATC::goto_top_of_z() {
        gc_exec_linef(true, "G53 G0 Z%0.3f", top_of_z);  // Go to top of Z travel
    }

    void KressATC::probe_notification() {
        //only handle successfull external probings
        if (sys.state == State::Alarm || tool_setter_probing) {
            return;
        }
        zeroed_tool_index = current_tool;
    }

    void KressATC::deactivate() {
        log_debug("Deactivating ATC spindle:" << current_tool);
        tool_change(0, false);  // return any tool we have

        float probe_position[MAX_N_AXIS];
        motor_steps_to_mpos(probe_position, probe_steps);
        log_info("ETS:" << tool[zeroed_tool_index].offset[Z_AXIS]);
        log_info("Surface:" << gc_state.coord_system[Z_AXIS]);
        log_info("Delta:" << tool[zeroed_tool_index].offset[Z_AXIS] - gc_state.coord_system[Z_AXIS]);

        // set G92 Z to the zeroed tool probe height for reference of the next spindle
        gc_state.coord_offset[Z_AXIS] = tool[zeroed_tool_index].offset[Z_AXIS];

        Spindle::deactivate();  // call base function
    }

    void KressATC::activate() {
        log_debug("Activating ATC spindle:" << current_tool);
        Spindle::activate();
    }

    // Configuration registration    namespace {
    SpindleFactory::InstanceBuilder<KressATC> registration("kress_atc");
}
