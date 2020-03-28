/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsHeli_Single.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Single::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli, 0),

    // @Group: SWASH
    // @Path: AP_MotorsHeli_Swash.cpp
    AP_SUBGROUPINFO(_swashplate, "SWASH_", 20, AP_MotorsHeli_Single, AP_MotorsHeli_Swash),

    AP_GROUPEND
};

#define YAW_SERVO_MAX_ANGLE 4500

// set update rate to servos - a value in hertz
void AP_MotorsHeli_Single::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // setup fast channels
    uint32_t mask = 
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4;
    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << (AP_MOTORS_MOT_5);
    }
    rc_set_freq(mask, _speed_hz);
}

// init_outputs - initialise Servo/PWM ranges and endpoints
bool AP_MotorsHeli_Single::init_outputs()
{
    if (!_flags.initialised_ok) {
        // map primary swash servos
        for (uint8_t i=0; i<AP_MOTORS_HELI_SINGLE_NUM_SWASHPLATE_SERVOS; i++) {
            add_motor_num(CH_1+i);
        }
        if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
            add_motor_num(CH_5);
        }

        // yaw servo
        add_motor_num(CH_4);

        // initialize throttle servos
        _main_rotor.init_servo();
        if (_throttle_mode == THROTTLE_CONTROL_TWIN) {
            _main_rotor.init_servo_2();
        }
    }

    // reset swash servo range and endpoints
    for (uint8_t i=0; i<AP_MOTORS_HELI_SINGLE_NUM_SWASHPLATE_SERVOS; i++) {
        reset_swash_servo(SRV_Channels::get_motor_function(i));
    }
    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        reset_swash_servo(SRV_Channels::get_motor_function(4));
    }

    // yaw servo is an angle from -4500 to 4500
    SRV_Channels::set_angle(SRV_Channel::k_motor4, YAW_SERVO_MAX_ANGLE);

    _flags.initialised_ok = true;

    return true;
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsHeli_Single::output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // swash servo 1
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // swash servo 2
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // swash servo 3
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // tail servo 4
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 5:
            // main rotor
            rc_write(AP_MOTORS_HELI_SINGLE_THROTTLE, pwm);
            rc_write(AP_MOTORS_HELI_SINGLE_THROTTLE2, pwm);
            break;
        default:
            // do nothing
            break;
    }
}

// set_desired_rotor_speed - manual throttle Engine #1
void AP_MotorsHeli_Single::set_desired_rotor_speed(float throttle_input)
{
    _main_rotor.set_throttle_input(throttle_input);
}

// set_desired_rotor_speed_2 - manual throttle Engine #2
void AP_MotorsHeli_Single::set_desired_rotor_speed_2(float throttle2_input)
{
    _main_rotor.set_throttle2_input(throttle2_input);
}

// set_rotor_rpm - used for governor with speed sensor
void AP_MotorsHeli_Single::set_rpm(float rotor_rpm)
{
    _main_rotor.set_rotor_rpm(rotor_rpm);
}

// calculate_scalars - recalculates various scalers used.
void AP_MotorsHeli_Single::calculate_armed_scalars()
{
    float throttlecurve[5];
    for (uint8_t i = 0; i < 5; i++) {
        throttlecurve[i] = _throttlecurve[i]*0.01f;
    }

    // throttle curve for engine #2
    float throttlecurve2[5];
    for (uint8_t i = 0; i < 5; i++) {
        throttlecurve2[i] = _throttlecurve2[i]*0.01f;
    }

    _main_rotor.set_ramp_time(_throttle_ramp_time);
    _main_rotor.set_runup_time(_rotor_runup_time);
    _main_rotor.set_critical_speed(_rotor_critical*0.01f);
    _main_rotor.set_idle_output(_throttle_idle_output*0.01f);
    _main_rotor.set_throttle_curve(throttlecurve);
    _main_rotor.set_governor_droop_response(_governor_droop_response*0.0001f);
    _main_rotor.set_governor_reference(_governor_reference);
    _main_rotor.set_governor_torque(_governor_torque*.01f);
    _main_rotor.set_governor_tcgain(_governor_tcgain*0.01f);
    // set variables for twin-engine heli's, engine #2
    _main_rotor.set_governor2_tcgain(_governor2_tcgain*0.01f);
    _main_rotor.set_governor2_droop_response(_governor2_droop_response*0.0001f);
    _main_rotor.set_throttle_curve2(throttlecurve2);

    if (_heliflags.governor_on) {
        _main_rotor.set_governor_on(true);
    } else {
        _main_rotor.set_governor_on(false);
    }
}

// calculate_scalars - recalculates various scalers used.
void AP_MotorsHeli_Single::calculate_scalars()
{
    // range check collective min, max and mid
    if( _collective_min >= _collective_max ) {
        _collective_min = AP_MOTORS_HELI_COLLECTIVE_MIN;
        _collective_max = AP_MOTORS_HELI_COLLECTIVE_MAX;
    }
    _collective_mid = constrain_int16(_collective_mid, _collective_min, _collective_max);

    // calculate collective mid point as a number from 0 to 1
    _collective_mid_pct = ((float)(_collective_mid-_collective_min))/((float)(_collective_max-_collective_min));

    // configure swashplate and update scalars
    _swashplate.configure();
    _swashplate.calculate_roll_pitch_collective_factors();

    // send setpoints to main rotor controller and trigger recalculation of scalars
    _main_rotor.set_control_mode(static_cast<ThrottleControl>(_throttle_mode.get()));
    calculate_armed_scalars();
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsHeli_Single::get_motor_mask()
{
    // heli uses channels 1,2,3,4 and 8
    // setup fast channels
    uint32_t mask = 1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << AP_MOTORS_HELI_SINGLE_THROTTLE;

    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        mask |= 1U << 4;
    }

//    if (_throttle_mode == THROTTLE_CONTROL_TWIN) {
//        mask |= 1U << AP_MOTORS_HELI_SINGLE_THROTTLE2;
//    }

    return rc_map_mask(mask);
}

// update_engine_controls - sends commands to servo controllers
void AP_MotorsHeli_Single::update_engine_control(EngineControlState state)
{
    // Send state update to servos
    _main_rotor.output(state);

    if (state == ENGINE_CONTROL_STOP){
        // set engine run enable aux output to not run position to kill engine when disarmed
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
    } else {
        // else if armed, set engine run enable output to run position
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::SRV_CHANNEL_LIMIT_MAX);
    }

    // Check if rotor is ran up
    _heliflags.rotor_runup_complete = ( _main_rotor.is_runup_complete());
}

// move_actuators - moves swash plate and tail servos
//                 - expected ranges:
//                       roll : -1 ~ +1
//                       pitch: -1 ~ +1
//                       collective: 0 ~ 1
//                       yaw:   -1 ~ +1
//
void AP_MotorsHeli_Single::move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)
{
    float yaw_offset = 0.0f;

    // initialize limits flag
    limit.roll = false;
    limit.pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // rescale roll_out and pitch_out into the min and max ranges to provide linear motion
    // across the input range instead of stopping when the input hits the constrain value
    // these calculations are based on an assumption of the user specified cyclic_max
    // coming into this equation at 45 deg or less
    float total_out = norm(pitch_out, roll_out);

    if (total_out > (_cyclic_max/45.0f)) {
        float ratio = (float)(_cyclic_max/45.0f) / total_out;
        roll_out *= ratio;
        pitch_out *= ratio;
        limit.roll = true;
        limit.pitch = true;
    }

    // constrain collective input
    float collective_out = coll_in;
    if (collective_out <= 0.0f) {
        collective_out = 0.0f;
        limit.throttle_lower = true;
    }
    if (collective_out >= 1.0f) {
        collective_out = 1.0f;
        limit.throttle_upper = true;
    }

    // ensure not below landed/landing collective
    if (_heliflags.landing_collective && collective_out < _collective_mid_pct) {
        collective_out = _collective_mid_pct;
        limit.throttle_lower = true;
    }

    // if servo output not in manual mode, process pre-compensation factors
    if (_servo_mode == SERVO_CONTROL_MODE_AUTOMATED) {
        // rudder feed forward based on collective
        //TODO This does not work properly - needs to be looked at
        if (_main_rotor.get_throttle_output() > _main_rotor.get_idle_output()) {
            // the 4.5 scaling factor is to bring the values in line with previous releases
            yaw_offset = _collective_yaw_effect * fabsf(collective_out - _collective_mid_pct) / 4.5f;
        }
    } else {
        yaw_offset = 0.0f;
    }

    // feed power estimate into main rotor controller
    _main_rotor.set_collective(fabsf(collective_out));

    // scale collective pitch for swashplate servos
    float collective_scalar = ((float)(_collective_max-_collective_min))*0.001f;
    float collective_out_scaled = collective_out * collective_scalar + (_collective_min - 1000)*0.001f;

    // get servo positions from swashplate library
    _servo1_out = _swashplate.get_servo_out(CH_1,pitch_out,roll_out,collective_out_scaled);
    _servo2_out = _swashplate.get_servo_out(CH_2,pitch_out,roll_out,collective_out_scaled);
    _servo3_out = _swashplate.get_servo_out(CH_3,pitch_out,roll_out,collective_out_scaled);
    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        _servo5_out = _swashplate.get_servo_out(CH_4,pitch_out,roll_out,collective_out_scaled);
    }

    // update the yaw rate using the tail rotor/servo
    move_yaw(yaw_out + yaw_offset);
}

// move_yaw
void AP_MotorsHeli_Single::move_yaw(float yaw_out)
{
    // sanity check yaw_out
    if (yaw_out < -1.0f) {
        yaw_out = -1.0f;
        limit.yaw = true;
    }
    if (yaw_out > 1.0f) {
        yaw_out = 1.0f;
        limit.yaw = true;
    }

    _servo4_out = yaw_out;
}

void AP_MotorsHeli_Single::output_to_motors()
{
    if (!_flags.initialised_ok) {
        return;
    }

    // actually move the servos.  PWM is sent based on nominal 1500 center.  servo output shifts center based on trim value.
    rc_write_swash(AP_MOTORS_MOT_1, _servo1_out);
    rc_write_swash(AP_MOTORS_MOT_2, _servo2_out);
    rc_write_swash(AP_MOTORS_MOT_3, _servo3_out);
    // get servo positions from swashplate library and write to servo for 4 servo of 4 servo swashplate
    if (_swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_90 || _swashplate.get_swash_type() == SWASHPLATE_TYPE_H4_45) {
        rc_write_swash(AP_MOTORS_MOT_5, _servo5_out);
    }
    rc_write_angle(AP_MOTORS_MOT_4, _servo4_out * YAW_SERVO_MAX_ANGLE);

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            update_engine_control(ENGINE_CONTROL_STOP);
            break;
        case SpoolState::GROUND_IDLE:
            update_engine_control(ENGINE_CONTROL_IDLE);
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
            update_engine_control(ENGINE_CONTROL_AUTOTHROTTLE);
            break;
        case SpoolState::SPOOLING_DOWN:
            update_engine_control(ENGINE_CONTROL_IDLE);
            break;

    }
}

// parameter_check - check if helicopter specific parameters are sensible
bool AP_MotorsHeli_Single::parameter_check(bool display_msg) const
{
    // check parent class parameters
    return AP_MotorsHeli::parameter_check(display_msg);
}
