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

    // 1 was ROL_MAX which has been replaced by CYC_MAX

    // 2 was PIT_MAX which has been replaced by CYC_MAX

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

    // @Param: SV_MAN
    // @DisplayName: Manual Servo Mode
    // @Description: Manual servo override for swash set-up. Do not set this manually!
    // @Values: 0:Disabled,1:Passthrough,2:Max collective,3:Mid collective,4:Min collective
    // @User: Standard
    AP_GROUPINFO("SV_MAN",  6, AP_MotorsHeli, _servo_mode, SERVO_CONTROL_MODE_AUTOMATED),

    // @Param: RSC_SETPOINT
    // @DisplayName: Electric ESC Throttle Setting
    // @Description: Throttle signal percent for electric helicopters when a governor is used in the ESC
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_SETPOINT", 7, AP_MotorsHeli, _rsc_setpoint, AP_MOTORS_HELI_RSC_SETPOINT),

    // @Param: RSC_MODE
    // @DisplayName: Throttle Control Mode
    // @Description: Selects the type of throttle control that will be used
    // @Values: 1:RC Passthru,2:Electric ESC Governor,3:Throttle Curve,4:Rotor Governor
    // @User: Standard
    AP_GROUPINFO("RSC_MODE", 8, AP_MotorsHeli, _rsc_mode, ROTOR_CONTROL_MODE_SPEED_SETPOINT),

    // @Param: LAND_COL_MIN
    // @DisplayName: Landing Collective Minimum
    // @Description: Minimum collective position in PWM microseconds while landed or landing
    // @Range: 0 500
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LAND_COL_MIN", 9, AP_MotorsHeli, _land_collective_min, AP_MOTORS_HELI_LAND_COLLECTIVE_MIN),

    // @Param: RSC_RAMP_TIME
    // @DisplayName: Throttle Ramp Time
    // @Description: Time in seconds for throttle to ramp from ground idle to flight idle power when throttle hold is released. This setting is used primarily by piston and turbine engines to smoothly engage the transmission clutch. However, it can also be used for electric ESC's that do not have an internal soft-start. If used with electric ESC with soft-start it is recommended to set this to 1 second so as to not confuse the ESC's soft-start function
    // @Range: 0 60
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("RSC_RAMP_TIME", 10, AP_MotorsHeli, _rsc_ramp_time, AP_MOTORS_HELI_RSC_RAMP_TIME),

    // @Param: RSC_RUNUP_TIME
    // @DisplayName: Rotor Runup Time
    // @Description: Actual time in seconds for the main rotor to reach full speed after throttle hold is released. Must be at least one second longer than the Throttle Ramp Time that is set with RSC_RAMP_TIME.
    // @Range: 0 60
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("RSC_RUNUP_TIME", 11, AP_MotorsHeli, _rsc_runup_time, AP_MOTORS_HELI_RSC_RUNUP_TIME),

    // @Param: RSC_CRITICAL
    // @DisplayName: Critical Rotor Speed
    // @Description: Percentage of normal rotor speed where entry to autorotation becomes dangerous. For helicopters with rotor speed sensor should be set to the percentage of the governor rpm setting used. Even if governor is not used when a speed sensor is installed, set the governor rpm to normal headspeed then set critical to a percentage of normal rpm (usually 90%). This can be considered the bottom of the green arc for autorotation. For helicopters without speed sensor should be set to the throttle percentage where flight is no longer possible. With no speed sensor critical should be lower than electric ESC throttle setting for ESC's with governor, or lower than normal in-flight throttle percentage when the throttle curve or RC Passthru is used.
    // @Range: 0 1000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_CRITICAL", 12, AP_MotorsHeli, _rsc_critical, AP_MOTORS_HELI_RSC_CRITICAL),

    // @Param: RSC_IDLE
    // @DisplayName: Engine Ground Idle Setting
    // @Description: FOR COMBUSTION ENGINES. Sets the engine ground idle throttle percentage with clutch disengaged. This must be set to zero for electric helicopters under most situations. If the ESC has an autorotation window this can be set to keep the autorotation window open in the ESC. Consult the operating manual for your ESC to set it properly for this purpose
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_IDLE", 13, AP_MotorsHeli, _rsc_idle_output, AP_MOTORS_HELI_RSC_IDLE_DEFAULT),

    // index 14 was RSC_POWER_LOW. Do not use this index in the future.

    // index 15 was RSC_POWER_HIGH. Do not use this index in the future.

    // @Param: CYC_MAX
    // @DisplayName: Cyclic Pitch Angle Max
    // @Description: Maximum pitch angle of the swash plate
    // @Range: 0 18000
    // @Units: cdeg
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("CYC_MAX", 16, AP_MotorsHeli, _cyclic_max, AP_MOTORS_HELI_SWASH_CYCLIC_MAX),

    // @Param: SV_TEST
    // @DisplayName: Boot-up Servo Test Cycles
    // @Description: Number of cycles to run servo test on boot-up
    // @Range: 0 10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SV_TEST",  17, AP_MotorsHeli, _servo_test, 0),

    // index 18 was RSC_POWER_NEGC. Do not use this index in the future.

    // @Param: RSC_SLEWRATE
    // @DisplayName: Throttle servo slew rate
    // @Description: This controls the maximum rate at which the throttle output can change, as a percentage per second. A value of 100 means the throttle can change over its full range in one second. A value of zero gives unlimited slew rate.
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_SLEWRATE", 19, AP_MotorsHeli, _rsc_slewrate, 0),

    // @Param: RSC_THRCRV_0
    // @DisplayName: Throttle Servo Position for 0 percent collective
    // @Description: Throttle Servo Position for 0 percent collective. This is on a scale from 0 to 1000, where 1000 is full throttle and 0 is zero throttle. Actual PWM values are controlled by SERVOX_MIN and SERVOX_MAX. The 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.
    // @Range: 0 1000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_THRCRV_0", 20, AP_MotorsHeli, _rsc_thrcrv[0], AP_MOTORS_HELI_RSC_THRCRV_0_DEFAULT),

    // @Param: RSC_THRCRV_25
    // @DisplayName: Throttle Servo Position for 25 percent collective
    // @Description: Throttle Servo Position for 25 percent collective. This is on a scale from 0 to 1000, where 1000 is full throttle and 0 is zero throttle. Actual PWM values are controlled by SERVOX_MIN and SERVOX_MAX. The 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.
    // @Range: 0 1000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_THRCRV_25", 21, AP_MotorsHeli, _rsc_thrcrv[1], AP_MOTORS_HELI_RSC_THRCRV_25_DEFAULT),

    // @Param: RSC_THRCRV_50
    // @DisplayName: Throttle Servo Position for 50 percent collective
    // @Description: Throttle Servo Position for 50 percent collective. This is on a scale from 0 to 1000, where 1000 is full throttle and 0 is zero throttle. Actual PWM values are controlled by SERVOX_MIN and SERVOX_MAX. The 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.
    // @Range: 0 1000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_THRCRV_50", 22, AP_MotorsHeli, _rsc_thrcrv[2], AP_MOTORS_HELI_RSC_THRCRV_50_DEFAULT),

    // @Param: RSC_THRCRV_75
    // @DisplayName: Throttle Servo Position for 75 percent collective
    // @Description: Throttle Servo Position for 75 percent collective. This is on a scale from 0 to 1000, where 1000 is full throttle and 0 is zero throttle. Actual PWM values are controlled by SERVOX_MIN and SERVOX_MAX. The 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.
    // @Range: 0 1000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_THRCRV_75", 23, AP_MotorsHeli, _rsc_thrcrv[3], AP_MOTORS_HELI_RSC_THRCRV_75_DEFAULT),

    // @Param: RSC_THRCRV_100
    // @DisplayName: Throttle Servo Position for 100 percent collective
    // @Description: Throttle Servo Position for 100 percent collective. This is on a scale from 0 to 1000, where 1000 is full throttle and 0 is zero throttle. Actual PWM values are controlled by SERVOX_MIN and SERVOX_MAX. The 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.
    // @Range: 0 1000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_THRCRV_100", 24, AP_MotorsHeli, _rsc_thrcrv[4], AP_MOTORS_HELI_RSC_THRCRV_100_DEFAULT),

    // @Param: RSC_HEADSPEED
    // @DisplayName: Headspeed RPM setting
    // @Description: Set to the rotor rpm your helicopter runs in flight. When a speed sensor is installed the rotor governor maintains this speed. Also used for autorotation and for runup. For governor operation this should be set 10 rpm higher than the actual desired headspeed to allow for governor droop
    // @Range: 800 3500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_HEADSPEED", 25, AP_MotorsHeli, _rsc_governor_reference, AP_MOTORS_HELI_RSC_RPM_DEFAULT),

    // @Param: RSC_GOV_DISGAG
    // @DisplayName: Throttle Percentage for Governor Disengage
    // @Description: Percentage of throttle where the governor will disenage to allow return to flight idle power. Typically should be set to a value slightly below flight idle throttle. The governor disengage can be disabled by setting this value to zero, requiring throttle hold to shut down the governor
    // @Range: 0 50
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RSC_GOV_DISGAG", 26, AP_MotorsHeli, _rsc_governor_disengage, AP_MOTORS_HELI_RSC_GOVERNOR_DISENGAGE_DEFAULT),

    // @Param: RSC_GOV_DROOP
    // @DisplayName: Governor Droop Response Setting
    // @Description: Governor droop response under load, normal settings of 0-100%. Higher value is quicker response but may cause surging. Setting to zero disables the governor. Adjust this to be as aggressive as possible without getting surging or over-run on headspeed when the governor engages. Setting over 100% is allowable for some two-stage turbine engines to provide scheduling of the gas generator for proper torque response of the N2 spool
    // @Range: 0 150
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RSC_GOV_DROOP", 27, AP_MotorsHeli, _rsc_governor_droop_response, AP_MOTORS_HELI_RSC_GOVERNOR_DROOP_DEFAULT),

    // @Param: RSC_GOV_TCGAIN
    // @DisplayName: Governor Throttle Curve Gain
    // @Description: Percentage of throttle curve gain in governor output. If the governor runs with excessive droop more than 15 rpm lower than the speed setting, increase this setting until the governor runs at 8-10 rpm droop from the speed setting. The throttle curve must be properly tuned to fly the helicopter without the governor before this setting will work properly
    // @Range: 50 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RSC_GOV_TCGAIN", 28, AP_MotorsHeli, _rsc_governor_tcgain, AP_MOTORS_HELI_RSC_GOVERNOR_TCGAIN_DEFAULT),
    
    // @Param: RSC_GOV_RANGE
    // @DisplayName: Governor Operational Range
    // @Description: RPM range +/- governor rpm reference setting where governor is operational. If speed sensor fails or rpm falls outside of this range, the governor will disengage and return to throttle curve. Recommended range is 100 rpm
    // @Range: 50 200
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_GOV_RANGE", 29, AP_MotorsHeli, _rsc_governor_range, AP_MOTORS_HELI_RSC_GOVERNOR_RANGE_DEFAULT),

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

    // load boot-up servo test cycles into counter to be consumed
    _servo_test_cycle_counter = _servo_test;

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
    if (_servo_test_cycle_counter > 0){
        // perform boot-up servo test cycle if enabled
        servo_test();
    } else {
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
                _roll_in = 0.0f;
                _pitch_in = 0.0f;
                _throttle_filter.reset(0.0f);
                _yaw_in = -1.0f;
                break;
            case SERVO_CONTROL_MODE_MANUAL_OSCILLATE:
                // use servo_test function from child classes
                servo_test();
                break;
            default:
                // no manual override
                break;
        }
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
    // returns false if _rsc_setpoint is not higher than _rsc_critical as this would not allow rotor_runup_complete to ever return true for electric helicopters with ESC governor
    if (_rsc_critical >= _rsc_setpoint) {
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: H_RSC_CRITICAL too large");
        }
        return false;
    }

    // returns false if RSC Mode is not set to a valid control mode
    if (_rsc_mode <= (int8_t)ROTOR_CONTROL_MODE_DISABLED || _rsc_mode > (int8_t)ROTOR_CONTROL_MODE_CLOSED_LOOP_POWER_OUTPUT) {
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Throttle mode invalid");
        }
        return false;
    }

    // returns false if RSC Runup Time is less than Ramp time as this could cause undesired behaviour of rotor speed estimate when no speed sensor is used
    if (_rsc_runup_time <= _rsc_ramp_time){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Runup time too small");
        }
        return false;
    }

    // returns false if idle output is higher than critical rotor speed percentage
    if ( _rsc_idle_output >=  _rsc_critical){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Engine idle speed too high");
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
