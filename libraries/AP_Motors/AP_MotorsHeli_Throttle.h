#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

// engine throttle controller states
enum EngineControlState {
    ENGINE_CONTROL_STOP = 0,
    ENGINE_CONTROL_IDLE,
    ENGINE_CONTROL_AUTOTHROTTLE
};

// throttle control modes
enum ThrottleControl {
    THROTTLE_CONTROL_DISABLED = 0,
    THROTTLE_CONTROL_SINGLE,
    THROTTLE_CONTROL_TWIN
};

class AP_MotorsHeli_Throttle {
public:
    friend class AP_MotorsHeli_Single;

    AP_MotorsHeli_Throttle(SRV_Channel::Aux_servo_function_t aux_fn, uint8_t default_channel,
                      SRV_Channel::Aux_servo_function_t aux_fn_2, uint8_t default_channel_2) :
        _aux_fn(aux_fn),
        _default_channel(default_channel),
        _aux_fn_2(aux_fn_2),
        _default_channel_2(default_channel_2)
    {};

    // init_servo - servo initialization on start-up
    void        init_servo();
    void        init_servo_2();

    // set_control_mode - sets control mode
    void        set_control_mode(ThrottleControl mode) { _control_mode = mode; }

    // set_critical_speed
    void        set_critical_speed(float critical_speed) { _critical_speed = critical_speed; }

    // get_critical_speed
    float       get_critical_speed() const { return _critical_speed; }

    // set_idle_output
    float       get_idle_output() { return _idle_output; }
    void        set_idle_output(float idle_output) { _idle_output = idle_output; }

    // set AutoThrottle governor parameters
    void        set_governor_on(bool governor_on) {_governor_on = (bool)governor_on; }
    void        set_governor_droop_response(float governor_droop_response) { _governor_droop_response = governor_droop_response; }
    void        set_governor_output(float governor_output) {_governor_output = governor_output; }
    void        set_governor_reference(float governor_reference) { _governor_reference = governor_reference; }
    void        set_governor_torque(float governor_torque) { _governor_torque = governor_torque; }
    void        set_governor_tcgain(float governor_tcgain) {_governor_tcgain = governor_tcgain; }
    // set AutoThrottle governor parameters for engine #2
    void        set_governor2_droop_response(float governor2_droop_response) { _governor2_droop_response = governor2_droop_response; }
    void        set_governor2_output(float governor2_output) {_governor2_output = governor2_output; }
    void        set_governor2_tcgain(float governor2_tcgain) {_governor2_tcgain = governor2_tcgain; }

    // get_throttle_input for manual throttles
    float       get_throttle_input() const { return _throttle_input; }
    float       get_throttle2_input() const { return _throttle2_input; }

    // set_throttle_input for manual throttles
    void        set_throttle_input(float throttle_input) { _throttle_input = throttle_input; }
    void        set_throttle2_input(float throttle2_input) { _throttle2_input = throttle2_input; }

    // get engine throttle outputs
    float       get_throttle_output() const { return _throttle_output; }
    float       get_throttle2_output() const { return _throttle2_output; }

    // get_rotor_speed - estimated rotor speed when no governor or rpm sensor is used
    float       get_rotor_speed() const;

    // set_rotor_rpm - when speed sensor is available for governor
    void        set_rotor_rpm(float rotor_rpm) {_rotor_rpm = (float)rotor_rpm; }

    // get AutoThrottle governor_outputs
    float       get_governor_output() const { return _governor_output; }
    float       get_governor2_output() const { return _governor2_output; }

    // is_runup_complete
    bool        is_runup_complete() const { return _runup_complete; }

    // set_ramp_time
    void        set_ramp_time(int8_t ramp_time) { _ramp_time = ramp_time; }

    // set_runup_time
    void        set_runup_time(int8_t runup_time) { _runup_time = runup_time; }

    // set_throttle_curve
    void        set_throttle_curve(float throttlecurve[5]);
    void        set_throttle_curve2(float throttlecurve2[5]);

    // set_collective for throttle curve calculation
    void        set_collective(float collective) { _collective_in = collective; }

    // output - update value to send to throttle servo
    void        output(EngineControlState state);

    // calculate autothrottle output
    void        engine_1_autothrottle_run();
    void        engine_2_autothrottle_run();

private:
    uint64_t        _last_update_us;

    // channel setup for aux function
    SRV_Channel::Aux_servo_function_t _aux_fn;
    uint8_t         _default_channel;
    SRV_Channel::Aux_servo_function_t _aux_fn_2;
    uint8_t         _default_channel_2;

    // internal variables
    ThrottleControl _control_mode = THROTTLE_CONTROL_DISABLED;   // throttle control mode
    float           _critical_speed;              // rotor speed below which flight is not possible
    float           _idle_output;                 // engine idle speed output
    float           _throttle_input;              // latest manual throttle input, Engine #1
    float           _throttle2_input;             // latest manual throttle input, Engine #2
    float           _throttle_output;             // AutoThrottle output, Engine #1
    float           _throttle2_output;            // AutoThrottle output, Engine #2
    float           _throttle_torque_reference;   // throttle torque reference, Engine #1
    float           _throttle2_torque_reference;  // throttle torque reference. Engine #2
    bool            _autothrottle_on;             // AutoThrottle status flag, Engine #1
    bool            _autothrottle2_on;            // AutoThrottle status flag, Engine #2
    float           _throttle_ramp_output;        // scalar to ramp throttle from _throttle_idle_output (0.0-1.0f)
    float           _engine_runup_output;         // scalar used to store status of engine run-up time (0.0-1.0f)
    int8_t          _ramp_time;                   // time in seconds to ramp throttle output
    int8_t          _runup_time;                  // time in seconds for the main rotor to reach full speed
    bool            _runup_complete;              // flag for determining if runup is complete
    float           _throttlecurve_poly[4][4];    // spline polynomials for engine #1 throttle curve
    float           _throttlecurve2_poly[4][4];   // spline polynomials for engine #1 throttle curve
    float           _collective_in;               // collective in for throttle curve calculation, range 0-1.0f
    float           _rotor_rpm;                   // rotor rpm from rotor speed sensor
    bool            _governor_on;                 // flag for governor on/off switch
    bool            _governor_fault;              // flag for governor fault detection, Engine #1
    bool            _governor2_fault;             // flag for governor fault detection, Engine #2
    float           _governor_output;             // governor output for engine #1
    float           _governor2_output;            // governor output for engine #2
    float           _governor_torque;             // governor torque limiter variable
    float           _governor_reference;          // sets rotor speed for governor
    float           _governor_droop_response;     // governor response, engine #1
    float           _governor2_droop_response;    // governor response, engine #2
    bool            _governor_engage;             // governor status flag engine #1
    bool            _governor2_engage;            // governor status flag engine #2
    float           _governor_tcgain;             // governor throttle curve gain, engine #1
    float           _governor2_tcgain;            // governor throttle curve gain, engine #2
    float           _governor_fault_timer;        // timer variable used to trigger a governor fault

    // update_throttle_ramp - slews output scalar between 0 and 1, outputs float scalar to _throttle_ramp_output
    void            update_throttle_ramp(float throttle_ramp_input, float dt);

    // update_engine_runup - function to slew runup scalar, outputs float scalar to _engine_runup_output
    void            update_engine_runup(float dt);

    // write_throttle - outputs pwm onto output throttle channel. servo_out parameter is of the range 0 ~ 1
    void            write_throttle(SRV_Channel::Aux_servo_function_t aux_function, float servo_out);

    // calculate_throttlecurve for both engines
    float           calculate_throttlecurve(float collective_in);
    float           calculate_throttlecurve2(float collective_in);
};

