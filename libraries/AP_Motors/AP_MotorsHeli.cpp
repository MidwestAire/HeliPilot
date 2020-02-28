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

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsHeli.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli::var_info[] = {

    // @Param: COL_MID
    // @DisplayName: Collective Zero Thrust
    // @Description: Swash servo position corresponding to zero collective pitch (or zero thrust for asymmetric blades)
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MID", 1, AP_MotorsHeli, _collective_mid, AP_MOTORS_HELI_COLLECTIVE_MID),

    // @Param: COL_MIN
    // @DisplayName: Collective Minimum
    // @Description: Minimum blade pitch. Must be set for proper autorotation performance at best autorotation airspeed
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MIN", 2, AP_MotorsHeli, _collective_min, AP_MOTORS_HELI_COLLECTIVE_MIN),

    // @Param: COL_MAX
    // @DisplayName: Collective Maximum
    // @Description: Maximum blade pitch. Must be set to correspond with maximum available engine torque at full collective climb
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MAX", 3, AP_MotorsHeli, _collective_max, AP_MOTORS_HELI_COLLECTIVE_MAX),

    // @Param: COL_YAW
    // @DisplayName: Collective-Yaw Mixing
    // @Description: Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
    // @Range: -10 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("COL_YAW", 4,  AP_MotorsHeli, _collective_yaw_effect, 0),

    // @Param: CYCLIC_DEG
    // @DisplayName: Cyclic Degrees
    // @Description: Cyclic tilt angle of the swashplate. Normally set this to whatever it takes to get 7-8 degrees of cyclic pitch
    // @Range: 0 45
    // @Units: deg
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CYCLIC_DEG", 5, AP_MotorsHeli, _cyclic_max, AP_MOTORS_HELI_SWASH_CYCLIC_MAX),

    // @Param: GOV_DROOP
    // @DisplayName: Engine #1 Droop Response
    // @Description: AutoThrottle governor droop response under load, normal settings of 0-100%. Higher value is quicker response but may cause surging. Adjust this to be as aggressive as possible without getting surging or Rrpm over-run when the governor engages. For twin-engine helicopters this will normally be tuned in static hover, adjusting the droop response higher on each engine until governor hunting is noted, then reduce the setting to where the governor is stable
    // @Range: 10 50
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GOV_DROOP", 6, AP_MotorsHeli, _governor_droop_response, 25),

    // @Param: GOV_TORQUE
    // @DisplayName: Governor Torque Limiter
    // @Description: Adjusts the engine's torque rise percent on AutoThrottle during governor ramp-up to full engage speed. The torque rise will determine how fast the rotor speed will ramp up when the governor is turned on. 30% torque rise is a good starting setting to adjust the governor ramp-in for piston and turbine engines
    // @Range: 10 60
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GOV_TORQUE", 7, AP_MotorsHeli, _governor_torque, 30),

    // @Param: GOV_TCGAIN
    // @DisplayName: Engine #1 TCGain
    // @Description: Percentage of throttle curve in governor output. This provides a feed-forward response to sudden loading or unloading of the rotor system. If Rrpm drops below Rrpm Low Warning during full collective climb increase the throttle curve gain
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GOV_TCGAIN", 8, AP_MotorsHeli, _governor_tcgain, 50),

    // @Param: ROTOR_CRITICAL
    // @DisplayName: Critical Rotor Speed
    // @Description: Percentage of normal rotor speed where entry to autorotation becomes dangerous. For helicopters with rotor speed sensor should be set to a percentage of the rotor rpm setting. Even if governor is not used when a speed sensor is installed, set the rotor rpm to normal headspeed then set critical to a percentage of normal rpm (usually 90%). This can be considered the bottom of the warning arc for autorotation. For helicopters without rotor speed sensor leave at 90%. Lack of a speed sensor results in using an estimated rotor speed instead of actual and is only marginally accurate
    // @Range: 50 95
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ROTOR_CRITICAL", 9, AP_MotorsHeli, _rotor_critical, AP_MOTORS_HELI_ROTOR_CRITICAL),

    // @Param: ROTOR_RPM
    // @DisplayName: Headspeed RPM
    // @Description: Set to the rotor rpm your helicopter runs in flight. When a speed sensor is installed the rotor governor maintains this speed. Also used for autorotation and for runup
    // @Range: 500 2500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ROTOR_RPM", 10, AP_MotorsHeli, _governor_reference, 1500),

    // @Param: ROTOR_RUNUP
    // @DisplayName: Rotor Runup Time
    // @Description: Time in seconds for the main rotor to reach full speed on AutoThrottle. SET TO ZERO TO USE ROTOR SPEED SENSOR FOR RUNUP (recommended). If not using rotor speed sensor the rotor runup must be at least 1 second longer than the throttle ramp time.!WARNING! - when measured rotor speed is not used to determine runup, setting rotor runup time to an excessively high value can cause rapid power recovery from manual throttle, resulting in blade lag and potential rotor imbalance. With all electric helicopters it is recommended to set rotor runup one second longer than throttle ramp time. Piston and turbine helicopters must use a rotor speed sensor with this setting set to zero
    // @Range: 0 60
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("ROTOR_RUNUP", 11, AP_MotorsHeli, _rotor_runup_time, AP_MOTORS_HELI_ROTOR_RUNUP_TIME),

    // @Param: SWASH_SETUP
    // @DisplayName: Swashplate Setup
    // @Description: Manual servo override for swashplate setup only
    // @Values: 0:Disabled,1:Passthrough,2:Max collective,3:Mid collective,4:Min collective
    // @User: Standard
    AP_GROUPINFO("SWASH_SETUP", 12, AP_MotorsHeli, _servo_mode, SERVO_CONTROL_MODE_AUTOMATED),

    // @Param: THROTTLE_IDLE
    // @DisplayName: Engine Ground Idle
    // @Description: For piston or turbine engines in single-engine helicopters only. Use of manual throttle to set engine idle speed is recommended instead of using this setting. For twin-engine helicopters always set the engine ground idle speeds with the RC radio manual throttles as using this setting will not result in the same idle speed for both engines. At ground idle the engines should idle with clutch disengaged, and for engine start. !!WARNING!! Using this setting requires disarm of the flight control to shut down engines on either single or twin-engine helicopters. Using this setting for electric helicopters could result in motor start when the flight control system is armed. This is a legacy setting, it will be deprecated in future versions of HeliPilot
    // @Range: 0 55
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_IDLE", 13, AP_MotorsHeli, _throttle_idle_output, 0),

    // @Param: THROTTLE_P1
    // @DisplayName: Engine #1 P1
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate all the way to its maximum negative or low collective pitch position. This setting, combined with THROTTLE_P2 corresponds to engine flight idle, rotor turning at rated speed with no engine load
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_P1", 14, AP_MotorsHeli, _throttlecurve[0], 20),

    // @Param: THROTTLE_P2
    // @DisplayName: Engine #1 P2
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 25% of it's full collective travel.This may or may not correspond to 25% position of the collective stick, depending on the range of negative pitch in the setup. Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 25% of 12 degrees is 3 degrees, so this setting would correspond to +1 degree of positive pitch. This setting, combined with THROTTLE_P1 corresponds to engine flight idle, rotors turning at rated speed with no engine load
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_P2", 15, AP_MotorsHeli, _throttlecurve[1], 40),

    // @Param: THROTTLE_P3
    // @DisplayName: Engine #1 P3
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 50% of it's full collective travel.This may or may not correspond to 50% position of the collective stick, depending on the range of negative pitch in the setup. Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 50% of 12 degrees is 6 degrees, so this setting would correspond to +4 degrees of positive pitch
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_P3", 16, AP_MotorsHeli, _throttlecurve[2], 60),

    // @Param: THROTTLE_P4
    // @DisplayName: Engine #1 P4
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 75% of it's full collective travel.This may or may not correspond to 75% position of the collective stick, depending on the range of negative pitch in the setup. Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 75% of 12 degrees is 9 degrees, so this setting would correspond to +7 degrees of positive pitch
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_P4", 17, AP_MotorsHeli, _throttlecurve[3], 80),

    // @Param: THROTTLE_P5
    // @DisplayName: Engine #1 P5
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 100% of it's full collective travel, which is maximum positive pitch
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE_P5", 18, AP_MotorsHeli, _throttlecurve[4], 100),

    // @Param: THROTTLE_RAMP
    // @DisplayName: Throttle Ramp Time
    // @Description: Time in seconds for throttle to ramp from ground idle to AutoThrottle when manual throttle(s) are suddenly advanced from fuel cutoff to current throttle curve position. This setting is used primarily by piston and turbine engines to smoothly engage the transmission clutch. However, it can also be used for electric ESC's that do not have an internal soft-start. If used with electric ESC with soft-start it is recommended to set this to 1 second so as to not confuse the ESC's soft-start function
    // @Range: 0 60
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("THROTTLE_RAMP", 19, AP_MotorsHeli, _throttle_ramp_time, AP_MOTORS_HELI_THROTTLE_RAMP_TIME),

    // Index 20 is used for swashplate library. Do not use

    // @Param: NUM_ENGINES
    // @DisplayName: Number of Engines
    // @Description: Set number of active throttle controls for either single-engine or twin-engine helicopters. If twin-engine is selected RC7 input is used for engine #2 throttle control. RC8 input is used for engine #1 throttle control on both single and twin-engine helicopters. One of the SERVO outputs must be set to HeliThrottle2 to enable the throttle servo output for the second engine
    // @Values: 1:Single Engine,2:Twin Engine
    // @User: Standard
    AP_GROUPINFO("NUM_ENGINES", 21, AP_MotorsHeli, _throttle_mode, THROTTLE_CONTROL_SINGLE),

    // @Param: GOV2_DROOP
    // @DisplayName: Engine #2 Droop Response
    // @Description: AutoThrottle governor droop response under load for engine #2, normal settings of 10-100%. Higher value is quicker response but may cause surging. Adjust this to be as aggressive as possible without getting surging or Rrpm over-run when the governor engages. For twin-engine helicopters this will normally be tuned in static hover, adjusting the droop response higher on each engine until governor hunting is noted, then reduce the setting to where the governor is stable
    // @Range: 10 50
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GOV2_DROOP", 22, AP_MotorsHeli, _governor2_droop_response, 25),

    // @Param: GOV2_TCGAIN
    // @DisplayName: Engine #2 TCGain
    // @Description: Percentage of throttle curve in governor output for engine #2. This provides a feed-forward response to sudden loading or unloading of the rotor system. If Rrpm drops below Rrpm Low Warning during full collective climb increase the throttle curve gain
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("GOV2_TCGAIN", 23, AP_MotorsHeli, _governor2_tcgain, 50),

    // @Param: THROTTLE2_P1
    // @DisplayName: Engine #2 P1
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate all the way to its maximum negative or low collective pitch position for engine #2. This setting, combined with THROTTLE2_P2 corresponds to engine flight idle, rotors turning at rated speed with no engine load
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE2_P1", 24, AP_MotorsHeli, _throttlecurve2[0], 20),

    // @Param: THROTTLE2_P2
    // @DisplayName: Engine #2 P2
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 25% of it's full collective travel for engine #2.This may or may not correspond to 25% position of the collective stick, depending on the range of negative pitch in the setup. Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 25% of 12 degrees is 3 degrees, so this setting would correspond to +1 degree of positive pitch. This setting, combined with THROTTLE2_P1 corresponds to engine flight idle, rotors turning at rated speed with no engine load
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE2_P2", 25, AP_MotorsHeli, _throttlecurve2[1], 40),

    // @Param: THROTTLE2_P3
    // @DisplayName: Engine #2 P3
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 50% of it's full collective travel for engine #2.This may or may not correspond to 50% position of the collective stick, depending on the range of negative pitch in the setup. Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 50% of 12 degrees is 6 degrees, so this setting would correspond to +4 degrees of positive pitch
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE2_P3", 26, AP_MotorsHeli, _throttlecurve2[2], 60),

    // @Param: THROTTLE2_P4
    // @DisplayName: Engine #2 P4
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 75% of it's full collective travel for engine #2.This may or may not correspond to 75% position of the collective stick, depending on the range of negative pitch in the setup. Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 75% of 12 degrees is 9 degrees, so this setting would correspond to +7 degrees of positive pitch
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE2_P4", 27, AP_MotorsHeli, _throttlecurve2[3], 80),

    // @Param: THROTTLE2_P5
    // @DisplayName: Engine #2 P5
    // @Description: Sets the engine's throttle percent for the throttle curve with the swashplate at 100% of it's full collective travel for engine #2, which is maximum positive pitch
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THROTTLE2_P5", 28, AP_MotorsHeli, _throttlecurve2[4], 100),

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

// set frame class
void AP_MotorsHeli::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_HELI);
}

// output_min - sets servos to neutral point with engine(s) stopped
void AP_MotorsHeli::output_min()
{
    // move swash to mid
    move_actuators(0.0f,0.0f,0.5f,0.0f);

    update_engine_control(ENGINE_CONTROL_STOP);

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

// sends commands to the servos
void AP_MotorsHeli::output_armed_stabilizing()
{
    // if swash setup manual override active after arming, deactivate it and reinitialize servos
    if (_servo_mode != SERVO_CONTROL_MODE_AUTOMATED) {
        reset_flight_controls();
    }

    move_actuators(_roll_in, _pitch_in, get_throttle(), _yaw_in);

    update_engine_control(ENGINE_CONTROL_AUTOTHROTTLE);
}

// output_armed_zero_throttle - sends commands to the servos
void AP_MotorsHeli::output_armed_zero_throttle()
{
    // if swash setup manual override active after arming, deactivate it and reinitialize servos
    if (_servo_mode != SERVO_CONTROL_MODE_AUTOMATED) {
        reset_flight_controls();
    }

    move_actuators(_roll_in, _pitch_in, get_throttle(), _yaw_in);

    update_engine_control(ENGINE_CONTROL_IDLE);
}

// output_disarmed - sends commands to the servos
void AP_MotorsHeli::output_disarmed()
{
    // manual override (i.e. when setting up swash)
    //TODO replace reference to throttle with collective
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

    update_engine_control(ENGINE_CONTROL_STOP);
}

// parameter_check - check if helicopter specific parameters are sensible
bool AP_MotorsHeli::parameter_check(bool display_msg) const
{
    // returns false if cyclic pitch is out of range
    if (_cyclic_max > 45){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Cyclic exceeds 45 degrees");
        }
        return false;
    }

    // returns false if critical speed exceeds 95% of rated rotor speed
    if (_rotor_critical > 95){
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Rotor critical speed over 95 percent");
        }
        return false;
    }

    // returns false if collective yaw compensation out of range
    if ((_collective_yaw_effect > 10) || (_collective_yaw_effect < -10)) {
        if (display_msg) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Collective yaw compensation out of range");
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
//TODO replace reference to throttle with collective
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
