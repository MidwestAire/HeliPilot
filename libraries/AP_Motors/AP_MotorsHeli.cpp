/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsHeli.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsHeli.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli::var_info[] = {

    // Indices 1-2 deprecated, do not use

    // @Param: COL_MIN
    // @DisplayName: Collective Pitch Minimum
    // @Description: Lowest possible servo position in PWM microseconds for the swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MIN", 3, AP_MotorsHeli, _collective_min, AP_MOTORS_HELI_COLLECTIVE_MIN),

    // @Param: COL_MAX
    // @DisplayName: Collective Pitch Maximum
    // @Description: Highest possible servo position in PWM microseconds for the swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MAX", 4, AP_MotorsHeli, _collective_max, AP_MOTORS_HELI_COLLECTIVE_MAX),

    // @Param: COL_MID
    // @DisplayName: Collective Pitch Mid-Point
    // @Description: Swash servo position in PWM microseconds corresponding to zero collective pitch (or zero lift for Asymmetrical blades)
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MID", 5, AP_MotorsHeli, _collective_mid, AP_MOTORS_HELI_COLLECTIVE_MID),

    // @Param: COL_SETUP
    // @DisplayName: Swash Setup Mode
    // @Description: Manual servo override for swash setup only
    // @Values: 0:Disabled,1:Passthrough,2:Max collective,3:Mid collective,4:Min collective
    // @User: Standard
    AP_GROUPINFO("COL_SETUP",  6, AP_MotorsHeli, _servo_mode, SERVO_CONTROL_MODE_AUTOMATED),

    // Index 7 deprecated, do not use

    // @Param: THROTTLE_MODE
    // @DisplayName: Throttle Control Mode
    // @Description: Throttle Curve controls throttle with the collective by using the five throttle curve settings. Governor uses the throttle curve with built-in governor to control rotor speed. Use a flat throttle curve for electric ESC governors
    // @Values: 1:Throttle Curve,2:Governor
    // @User: Standard
    AP_GROUPINFO("THROTTLE_MODE", 8, AP_MotorsHeli, _rsc_mode, ROTOR_CONTROL_MODE_THROTTLE_CURVE),

    // Index 9 deprecated, do not use 

    // @Param: THROTTLE_RAMP
    // @DisplayName: Throttle Ramp Time
    // @Description: Time in seconds for throttle to ramp from ground idle to flight idle power when throttle hold is released. This setting is used primarily by piston and turbine engines to smoothly engage the transmission clutch. However, it can also be used for electric ESC's that do not have an internal soft-start. If used with electric ESC with soft-start it is recommended to set this to 1 second so as to not confuse the ESC's soft-start function
    // @Range: 0 60
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("THROTTLE_RAMP", 10, AP_MotorsHeli, _rsc_ramp_time, AP_MOTORS_HELI_RSC_RAMP_TIME),

    // @Param: ROTOR_RUNUP
    // @DisplayName: Rotor Runup Time
    // @Description: Actual time in seconds for the main rotor to reach full speed after throttle hold is released. Must be at least one second longer than the Throttle Ramp Time
    // @Range: 0 60
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("ROTOR_RUNUP", 11, AP_MotorsHeli, _rsc_runup_time, AP_MOTORS_HELI_RSC_RUNUP_TIME),

    // @Param: ROTOR_CRITICAL
    // @DisplayName: Critical Rotor Speed
    // @Description: Percentage of normal rotor speed where entry to autorotation becomes dangerous. For helicopters with rotor speed sensor should be set to a percentage of the rotor rpm setting. Even if governor is not used when a speed sensor is installed, set the rotor rpm to normal headspeed then set critical to a percentage of normal rpm (usually 90%). This can be considered the bottom of the green arc for autorotation. For helicopters without speed sensor should be set to the throttle percentage where flight is no longer possible. With no speed sensor critical should be lower than normal in-flight throttle percentage
    // @Range: 0 90
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ROTOR_CRITICAL", 12, AP_MotorsHeli, _rsc_critical, AP_MOTORS_HELI_RSC_CRITICAL),

    // @Param: THROTTLE_IDLE
    // @DisplayName: Engine Ground Idle Setting
    // @Description: FOR COMBUSTION ENGINES. Sets the engine ground idle throttle percentage with clutch disengaged. This must be set to zero for electric helicopters under most situations. If the ESC has an autorotation window this can be set to keep the autorotation window open in the ESC. Consult the operating manual for your ESC to set it properly for this purpose
    // @Range: 0 50
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_IDLE", 13, AP_MotorsHeli, _rsc_idle_output, AP_MOTORS_HELI_RSC_IDLE_DEFAULT),

    // Indices 14-15 deprecated, do not use

    // @Param: CYCLIC_DEG
    // @DisplayName: Cyclic Degrees Setting
    // @Description: Cyclic tilt angle of the swashplate. Normally set this to whatever it takes to get 8 degrees of cyclic pitch
    // @Range: 0 45
    // @Units: deg
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CYCLIC_DEG", 16, AP_MotorsHeli, _cyclic_max, AP_MOTORS_HELI_SWASH_CYCLIC_MAX),

    // Indices 17-19 deprecated, do not use

    // @Param: THROTTLE_P1
    // @DisplayName: Throttle at 0% collective
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate all the way to its maximum negative collective pitch position
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_P1", 20, AP_MotorsHeli, _rsc_thrcrv[0], 0),

    // @Param: THROTTLE_P2
    // @DisplayName: Throttle at 25% collective
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 25% of it's full collective travel.This may or may not correspond to 25% position of the collective stick, depending on the range of negative pitch in the setup. Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 25% of 12 degrees is 3 degrees, so this setting would correspond to +1 degree of positive pitch.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_P2", 21, AP_MotorsHeli, _rsc_thrcrv[1], 0),

    // @Param: THROTTLE_P3
    // @DisplayName: Throttle at 50% collective
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 50% of it's full collective travel.This may or may not correspond to 50% position of the collective stick, depending on the range of negative pitch in the setup. Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 50% of 12 degrees is 6 degrees, so this setting would correspond to +4 degrees of positive pitch.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_P3", 22, AP_MotorsHeli, _rsc_thrcrv[2], 0),

    // @Param: THROTTLE_P4
    // @DisplayName: Throttle at 75% collective
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 75% of it's full collective travel.This may or may not correspond to 75% position of the collective stick, depending on the range of negative pitch in the setup. Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 75% of 12 degrees is 9 degrees, so this setting would correspond to +7 degrees of positive pitch.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_P4", 23, AP_MotorsHeli, _rsc_thrcrv[3], 0),

    // @Param: THROTTLE_P5
    // @DisplayName: Throttle at 100% collective
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 100% of it's full collective travel, which is maximum positive pitch.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_P5", 24, AP_MotorsHeli, _rsc_thrcrv[4], 0),

    // @Param: ROTOR_RPM
    // @DisplayName: Headspeed RPM setting
    // @Description: Set to the rotor rpm your helicopter runs in flight. When a speed sensor is installed the rotor governor maintains this speed. Also used for autorotation and for runup. For governor operation this should be set 10 rpm higher than the actual desired headspeed to allow for governor droop
    // @Range: 800 3500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ROTOR_RPM", 25, AP_MotorsHeli, _rsc_governor_reference, AP_MOTORS_HELI_RSC_RPM_DEFAULT),

    // @Param: GOV_DISENGAGE
    // @DisplayName: Throttle Percentage for Governor Disengage
    // @Description: Percentage of throttle where the governor will disengage to allow return to flight idle power. Typically should be set to the same value as flight idle throttle (the very lowest throttle setting on your throttle curve). The governor disengage can be disabled by setting this value to zero and using the pull-down from the governor TCGAIN to reduce power to flight idle with the collective at it's lowest throttle setting on the throttle curve.
    // @Range: 0 50
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GOV_DISENGAGE", 26, AP_MotorsHeli, _rsc_governor_disengage, AP_MOTORS_HELI_RSC_GOVERNOR_DISENGAGE_DEFAULT),

    // @Param: GOV_DROOP
    // @DisplayName: Governor Droop Response Setting
    // @Description: Governor droop response under load, normal settings of 0-100%. Higher value is quicker response but may cause surging. Setting to zero disables the governor. Adjust this to be as aggressive as possible without getting surging or over-run on headspeed when the governor engages. Setting over 100% is allowable for some two-stage turbine engines to provide scheduling of the gas generator for proper torque response of the N2 spool
    // @Range: 0 150
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GOV_DROOP", 27, AP_MotorsHeli, _rsc_governor_droop_response, AP_MOTORS_HELI_RSC_GOVERNOR_DROOP_DEFAULT),

    // @Param: GOV_TCGAIN
    // @DisplayName: Governor Throttle Curve Gain
    // @Description: Percentage of throttle curve gain in governor output. This provides a type of feedforward response to sudden loading or unloading of the engine. If headspeed drops excessively during sudden heavy load, increase the throttle curve gain. If the governor runs with excessive droop more than 15 rpm lower than the speed setting, increase this setting until the governor runs at 8-10 rpm droop from the speed setting. The throttle curve must be properly tuned to fly the helicopter without the governor for this setting to work properly
    // @Range: 50 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GOV_TCGAIN", 28, AP_MotorsHeli, _rsc_governor_tcgain, AP_MOTORS_HELI_RSC_GOVERNOR_TCGAIN_DEFAULT),
    
    // @Param: GOV_RANGE
    // @DisplayName: Governor Operational Range
    // @Description: RPM range +/- governor rpm reference setting where governor is operational. If speed sensor fails or rpm falls outside of this range, the governor will disengage and return to throttle curve. Recommended range is 100 rpm
    // @Range: 50 200
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("GOV_RANGE", 29, AP_MotorsHeli, _rsc_governor_range, AP_MOTORS_HELI_RSC_GOVERNOR_RANGE_DEFAULT),

    AP_GROUPEND
};

//
// public methods
//

// init
void AP_MotorsHeli::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remember frame type
    _frame_type = frame_type;

    // set update rate
    set_update_rate(_speed_hz);

    // ensure inputs are not passed through to servos on start-up
    _servo_mode = SERVO_CONTROL_MODE_AUTOMATED;

    // initialise radio passthrough for collective to middle
    _throttle_radio_passthrough = 0.5f;

    // initialise Servo/PWM ranges and endpoints
    if (!init_outputs()) {
        // don't set initialised_ok
        return;
    }

    // calculate all scalars
    calculate_scalars();

    // record successful initialisation if what we setup was the desired frame_class
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_HELI);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsHeli::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_HELI);
}

// output_min - sets servos to neutral point with motors stopped
void AP_MotorsHeli::output_min()
{
    // move swash to mid
    move_actuators(0.0f,0.0f,0.5f,0.0f);

    update_motor_control(ROTOR_CONTROL_STOP);

    // override limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = false;
}

// output - sends commands to the servos
void AP_MotorsHeli::output()
{
    // update throttle filter
    update_throttle_filter();

    if (_flags.armed) {
        calculate_armed_scalars();
        if (!_flags.interlock) {
            output_armed_zero_throttle();
        } else {
            output_armed_stabilizing();
        }
    } else {
        output_disarmed();
    }
};

// sends commands to the motors
void AP_MotorsHeli::output_armed_stabilizing()
{
    // if manual override active after arming, deactivate it and reinitialize servos
    if (_servo_mode != SERVO_CONTROL_MODE_AUTOMATED) {
        reset_flight_controls();
    }

    move_actuators(_roll_in, _pitch_in, get_throttle(), _yaw_in);

    update_motor_control(ROTOR_CONTROL_ACTIVE);
}

// output_armed_zero_throttle - sends commands to the motors
void AP_MotorsHeli::output_armed_zero_throttle()
{
    // if manual override active after arming, deactivate it and reinitialize servos
    if (_servo_mode != SERVO_CONTROL_MODE_AUTOMATED) {
        reset_flight_controls();
    }

    move_actuators(_roll_in, _pitch_in, get_throttle(), _yaw_in);

    update_motor_control(ROTOR_CONTROL_IDLE);
}

// output_disarmed - sends commands to the motors
void AP_MotorsHeli::output_disarmed()
{
    // manual override (i.e. when setting up swash)
    switch (_servo_mode) {
        case SERVO_CONTROL_MODE_MANUAL_PASSTHROUGH:
            // pass pilot commands straight through to swash
            _roll_in = _roll_radio_passthrough;
            _pitch_in = _pitch_radio_passthrough;
            _throttle_filter.reset(_throttle_radio_passthrough);
            _yaw_in = _yaw_radio_passthrough;
            break;
        case SERVO_CONTROL_MODE_MANUAL_CENTER:
            // fixate mid collective
            _roll_in = 0.0f;
            _pitch_in = 0.0f;
            _throttle_filter.reset(_collective_mid_pct);
            _yaw_in = 0.0f;
            break;
        case SERVO_CONTROL_MODE_MANUAL_MAX:
            // fixate max collective
            _roll_in = 0.0f;
            _pitch_in = 0.0f;
            _throttle_filter.reset(1.0f);
            _yaw_in = 1.0f;
            break;
        case SERVO_CONTROL_MODE_MANUAL_MIN:
            // fixate min collective
            _roll_in = 0.0;
            _pitch_in = 0.0f;
            _throttle_filter.reset(0.0f);
            _yaw_in = -1.0f;
            break;
        default:
            // no manual override
            break;
        }

    // ensure swash servo endpoints haven't been moved
    init_outputs();

    // continuously recalculate scalars to allow setup
    calculate_scalars();

    // helicopters always run stabilizing flight controls
    move_actuators(_roll_in, _pitch_in, get_throttle(), _yaw_in);

    update_motor_control(ROTOR_CONTROL_STOP);
}

// parameter_check - check if helicopter specific parameters are sensible
bool AP_MotorsHeli::parameter_check(bool display_msg) const
{
    // returns false if RSC Mode is not set to a valid control mode
    if (_rsc_mode <= (int8_t)ROTOR_CONTROL_MODE_DISABLED || _rsc_mode > (int8_t)ROTOR_CONTROL_MODE_GOVERNOR) {
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Throttle mode invalid");
        }
        return false;
    }

    // returns false if RSC Runup Time is less than Ramp time as this could cause undesired behaviour of rotor speed estimate when no speed sensor is used
    if (_rsc_runup_time <= _rsc_ramp_time){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Throttle ramp exceeds runup time");
        }
        return false;
    }

    // returns false if idle output is higher than critical rotor speed percentage
    if ( _rsc_idle_output >=  _rsc_critical){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Engine idle and critical speed misconfigured");
        }
        return false;
    }

    // returns false if cyclic pitch is out of range
    if (_cyclic_max > 45){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Cyclic exceeds 45 degrees");
        }
        return false;
    }

    // returns false if cyclic pitch is out of range
    if (_rsc_critical > 90){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Rotor critical speed exceeds 90 percent");
        }
        return false;
    }

    // all other cases parameters are OK
    return true;
}

// reset_swash_servo
void AP_MotorsHeli::reset_swash_servo(SRV_Channel::Aux_servo_function_t function)
{
    // outputs are defined on a -500 to 500 range for swash servos
    SRV_Channels::set_range(function, 1000);

    // swash servos always use full endpoints as restricting them would lead to scaling errors
    SRV_Channels::set_output_min_max(function, 1000, 2000);
}

// update the throttle input filter
void AP_MotorsHeli::update_throttle_filter()
{
    _throttle_filter.apply(_throttle_in, 1.0f/_loop_rate);

    // constrain filtered throttle
    if (_throttle_filter.get() < 0.0f) {
        _throttle_filter.reset(0.0f);
    }
    if (_throttle_filter.get() > 1.0f) {
        _throttle_filter.reset(1.0f);
    }
}

// reset_flight_controls - resets all controls and scalars to flight status
void AP_MotorsHeli::reset_flight_controls()
{
    _servo_mode = SERVO_CONTROL_MODE_AUTOMATED;
    init_outputs();
    calculate_scalars();
}

// convert input in -1 to +1 range to pwm output for swashplate servo.
// The value 0 corresponds to the trim value of the servo. Swashplate
// servo travel range is fixed to 1000 pwm and therefore the input is
// multiplied by 500 to get PWM output.
void AP_MotorsHeli::rc_write_swash(uint8_t chan, float swash_in)
{
    uint16_t pwm = (uint16_t)(1500 + 500 * swash_in);
    SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(chan);
    SRV_Channels::set_output_pwm_trimmed(function, pwm);
}
