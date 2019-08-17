/// @file	AP_MotorsHeli_Single.h
/// @brief	Motor control class for traditional heli
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"
#include "AP_MotorsHeli_Swash.h"

// rsc and extgyro function output channels. 
#define AP_MOTORS_HELI_SINGLE_RSC                              CH_8

// maximum number of swashplate servos
#define AP_MOTORS_HELI_SINGLE_NUM_SWASHPLATE_SERVOS            3

/// @class      AP_MotorsHeli_Single
class AP_MotorsHeli_Single : public AP_MotorsHeli {
public:
    // constructor
    AP_MotorsHeli_Single(uint16_t       loop_rate,
                         uint16_t       speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(loop_rate, speed_hz),
        _main_rotor(SRV_Channel::k_heli_rsc, AP_MOTORS_HELI_SINGLE_RSC),
        _swashplate()
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // set update rate to motors - a value in hertz
    void set_update_rate(uint16_t speed_hz) override;

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    void output_test(uint8_t motor_seq, int16_t pwm) override;

    // set_desired_rotor_speed - sets target rotor speed as a number from 0 ~ 1
    void set_desired_rotor_speed(float desired_speed) override;

    // set_rpm - for rotor speed governor
    void set_rpm(float rotor_rpm) override;

    // get_main_rotor_speed - estimated rotor speed when no speed sensor or governor is used
    float get_main_rotor_speed() const  override { return _main_rotor.get_rotor_speed(); }

    // get_desired_rotor_speed - gets target rotor speed as a number from 0 ~ 1
    float get_desired_rotor_speed() const  override { return _main_rotor.get_desired_speed(); }

    // rotor_speed_above_critical - return true if rotor speed is above that critical for flight
    bool rotor_speed_above_critical() const  override { return _main_rotor.get_rotor_speed() > _main_rotor.get_critical_speed(); }
    
    // get_governor_output
    float get_governor_output() const override { return _main_rotor.get_governor_output(); }
    
    // get_control_output
    float get_control_output() const override{ return _main_rotor.get_control_output(); }

    // calculate_scalars - recalculates various scalars used
    void calculate_scalars() override;

    // calculate_armed_scalars - recalculates scalars that can change while armed
    void calculate_armed_scalars() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint16_t get_motor_mask() override;

    // has_flybar - returns true if we have a mechical flybar
    bool has_flybar() const  override { return _flybar_mode; }

    // parameter_check - returns true if helicopter specific parameters are sensible, used for pre-arm check
    bool parameter_check(bool display_msg) const override;

    // var_info
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // init_outputs - initialise Servo/PWM ranges and endpoints
    bool init_outputs() override;

    // update_motor_controls - sends commands to motor controllers
    void update_motor_control(RotorControlState state) override;

    // heli_move_actuators - moves swash plate and tail servo
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out) override;

    // move_yaw - moves the yaw servo
    void move_yaw(float yaw_out);

    // external objects we depend upon
    AP_MotorsHeli_RSC   _main_rotor;            // main rotor
    AP_MotorsHeli_Swash _swashplate;            // swashplate

    // internal variables
    float _servo1_out = 0.0f;                   // output value sent to motor
    float _servo2_out = 0.0f;                   // output value sent to motor
    float _servo3_out = 0.0f;                   // output value sent to motor
    float _servo5_out = 0.0f;                   // output value sent to motor

    // parameters
    AP_Int8         _flybar_mode;               // Compile for flybar helicopter
};
