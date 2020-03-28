/// @file	AP_MotorsHeli.h
/// @brief	Motor control class for Traditional Heli
#pragma once

#include <inttypes.h>

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_Motors_Class.h"
#include "AP_MotorsHeli_Throttle.h"

// servo output rates
#define AP_MOTORS_HELI_SPEED_DEFAULT              125     // default servo update rate for helicopters

// default swash min and max angles and positions
#define AP_MOTORS_HELI_SWASH_CYCLIC_MAX           25
#define AP_MOTORS_HELI_COLLECTIVE_MIN             1450
#define AP_MOTORS_HELI_COLLECTIVE_MAX             1650
#define AP_MOTORS_HELI_COLLECTIVE_MID             1500

// default main rotor critical speed
#define AP_MOTORS_HELI_ROTOR_CRITICAL             90

// default main rotor ramp up time in seconds
#define AP_MOTORS_HELI_THROTTLE_RAMP_TIME         5
#define AP_MOTORS_HELI_ROTOR_RUNUP_TIME           10

// flybar types
#define AP_MOTORS_HELI_NOFLYBAR                   0       // set to 1 to compile for flybar helicopter

class AP_HeliControls;

/// @class      AP_MotorsHeli
class AP_MotorsHeli : public AP_Motors {
public:

    /// Constructor
    AP_MotorsHeli( uint16_t         loop_rate,
                   uint16_t         speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_Motors(loop_rate, speed_hz)
    {
        AP_Param::setup_object_defaults(this, var_info);

        // initialise flags
        _heliflags.landing_collective = 0;
        _heliflags.rotor_runup_complete = 0;
    };

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to servos - a value in hertz
    virtual void set_update_rate( uint16_t speed_hz ) override = 0;

    // output_min - sets servos to neutral point with engine(s) stopped
    void output_min() override;

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void output_test_seq(uint8_t motor_seq, int16_t pwm) override = 0;

    //
    // heli specific methods
    //

    // parameter_check - returns true if helicopter specific parameters are sensible, used for pre-arm check
    virtual bool parameter_check(bool display_msg) const;

    // has_flybar - returns true if we have a mechanical flybar
    virtual bool has_flybar() const { return AP_MOTORS_HELI_NOFLYBAR; }

    // set_collective_for_landing - limits collective from going too low if we know we are landed
    void set_collective_for_landing(bool landing) { _heliflags.landing_collective = landing; }

    // get_throttle_mode - gets the throttle control method
    uint8_t get_throttle_mode() const { return _throttle_mode; }

    // set_rpm - for rotor speed governor
    virtual void set_rpm(float rotor_rpm) = 0;

    // set_governor_on - enables/disables governor
    void set_governor(bool governor) { _heliflags.governor_on = governor; }

    // set_desired_rotor_speed for engine manual throttles
    virtual void set_desired_rotor_speed(float throttle_input) = 0;
    virtual void set_desired_rotor_speed_2(float throttle2_input) = 0;

    // get_desired_rotor_speed for engine manual throttles
    virtual float get_desired_rotor_speed() const = 0;
    virtual float get_desired_rotor_speed_2() const = 0;

    // get_main_rotor_speed - estimated rotor speed when no governor or speed sensor used
    virtual float get_main_rotor_speed() const = 0;

    // return true if the main rotor is up to speed
    bool rotor_runup_complete() const { return _heliflags.rotor_runup_complete; }

    // rotor_speed_above_critical - return true if rotor speed is above that critical for flight
    virtual bool rotor_speed_above_critical() const = 0;

    //get AutoThrottle governor outputs
    virtual float get_governor_output() const = 0;
    virtual float get_governor2_output() const = 0;

    //get engine throttle outputs
    virtual float get_throttle_output() const = 0;
    virtual float get_throttle2_output() const = 0;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t get_motor_mask() override = 0;

    // output - sends commands to the servos
    void output() override;

    // supports_yaw_passthrough
    virtual bool supports_yaw_passthrough() const { return false; }

    float get_throttle_hover() const override { return 0.5f; }

    // support passing init_targets_on_arming flag to greater code
    bool init_targets_on_arming() const override { return _heliflags.init_targets_on_arming; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // manual servo modes (used for setup)
    enum ServoControlModes {
        SERVO_CONTROL_MODE_AUTOMATED = 0,
        SERVO_CONTROL_MODE_MANUAL_PASSTHROUGH,
        SERVO_CONTROL_MODE_MANUAL_MAX,
        SERVO_CONTROL_MODE_MANUAL_CENTER,
        SERVO_CONTROL_MODE_MANUAL_MIN,
    };

    // output - sends commands to the servos
    void output_armed_stabilizing() override;
    void output_armed_zero_throttle();
    void output_disarmed();

    // update_engine_controls - sends commands to servo controllers
    virtual void update_engine_control(EngineControlState state) = 0;

    // run spool logic
    void                output_logic();

    // output_to_motors - sends commands to the motors
    virtual void        output_to_motors() = 0;

    // reset_flight_controls - resets all controls and scalars to flight status
    void reset_flight_controls();

    // update the throttle input filter
    void update_throttle_filter() override;

    // move_actuators - moves swash plate and tail rotor
    virtual void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out) = 0;

    // reset_swash_servo - free up swash servo for maximum movement
    void reset_swash_servo(SRV_Channel::Aux_servo_function_t function);

    // init_outputs - initialise Servo/PWM ranges and endpoints
    virtual bool init_outputs() = 0;

    // calculate_armed_scalars - must be implemented by child classes
    virtual void calculate_armed_scalars() = 0;

    // calculate_scalars - must be implemented by child classes
    virtual void calculate_scalars() = 0;

    // write to a swash servo. output value is pwm
    void rc_write_swash(uint8_t chan, float swash_in);

    // flags bitmask
    struct heliflags_type {
        uint8_t landing_collective      : 1;      // true if collective is setup for landing which has much higher minimum
        uint8_t rotor_runup_complete    : 1;      // true if the rotors have had enough time to wind up
        uint8_t governor_on             : 1;      // true for governor on
        uint8_t init_targets_on_arming  : 1;      // 0 if targets were initialized, 1 if targets were not initialized after arming
    } _heliflags;

    // parameters
    AP_Int16        _cyclic_max;                  // Maximum cyclic angle of the swash plate
    AP_Int16        _collective_min;              // Lowest possible servo position for the swashplate
    AP_Int16        _collective_max;              // Highest possible servo position for the swashplate
    AP_Int16        _collective_mid;              // Swash servo position corresponding to zero thrust
    AP_Float        _collective_yaw_effect;       // Feed-forward compensation for collective to yaw
    AP_Int8         _servo_mode;                  // Pass radio inputs directly to servos during set-up
    AP_Int16        _governor_reference;          // sets headspeed for rotor governor, autorotation and runup
    AP_Float        _governor_droop_response;     // governor response to droop under load
    AP_Float        _governor2_droop_response;    // governor response for engine #2
    AP_Float        _governor_tcgain;             // governor throttle curve gain, range 50-100%
    AP_Float        _governor2_tcgain;            // governor throttle curve gain, engine #2
    AP_Float        _governor_torque;             // governor torque limiter variable
    AP_Int8         _rotor_runup_time;            // time in seconds for the main rotor to reach full speed
    AP_Int16        _rotor_critical;              // minimum safe rotor speed
    AP_Int8         _throttle_mode;               // default throttle control variable
    AP_Int8         _throttle_ramp_time;          // time in seconds to ramp throttle from ground idle to flight idle
    AP_Int16        _throttle_idle_output;        // Combustion engine idle speed setting
    AP_Int16        _throttlecurve[5];            // throttle curve, engine #1
    AP_Int16        _throttlecurve2[5];           // throttle curve, engine #2

    // internal variables
    float           _collective_mid_pct = 0.0f;   // collective mid parameter value converted to 0 ~ 1 range

    motor_frame_type _frame_type;
    motor_frame_class _frame_class;
};
