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
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsHeli_Throttle.h"

extern const AP_HAL::HAL& hal;

// init_servo - initialize engine #1 throttle on startup
void AP_MotorsHeli_Throttle::init_servo()
{
    // setup throttle on specified channel by default
    SRV_Channels::set_aux_channel_default(_aux_fn, _default_channel);

    // set servo range
    SRV_Channels::set_range(SRV_Channels::get_motor_function(_aux_fn), 1000);

    if (_control_mode == THROTTLE_CONTROL_TWIN) {
        init_servo_2();
    }
}

// init_servo_2 - initialize engine #2 throttle on startup
void AP_MotorsHeli_Throttle::init_servo_2()
{
    // setup throttle on specified channel by default
    SRV_Channels::set_aux_channel_default(_aux_fn_2, _default_channel_2);

    // set servo range
    SRV_Channels::set_range(SRV_Channels::get_motor_function(_aux_fn_2), 1000);
}

// Throttle curve calculations for engine #1
void AP_MotorsHeli_Throttle::set_throttle_curve(float throttlecurve[5])
{
    // Ensure user inputs are within parameter limits
    for (uint8_t i = 0; i < 5; i++) {
        throttlecurve[i] = constrain_float(throttlecurve[i], 0.0f, 1.0f);
    }
    // Calculate the spline polynomials for the throttle curve
    splinterp5(throttlecurve, _throttlecurve_poly);
}

// Throttle curve calculations for engine #2
void AP_MotorsHeli_Throttle::set_throttle_curve2(float throttlecurve2[5])
{
    // Ensure user inputs are within parameter limits
    for (uint8_t i = 0; i < 5; i++) {
        throttlecurve2[i] = constrain_float(throttlecurve2[i], 0.0f, 1.0f);
    }
    // Calculate the spline polynomials for the throttle curve
    splinterp5(throttlecurve2, _throttlecurve2_poly);
}

// output - update value to send to throttle servo
void AP_MotorsHeli_Throttle::output(EngineControlState state)
{
    float dt;
    uint64_t now = AP_HAL::micros64();

    if (_last_update_us == 0) {
        _last_update_us = now;
        dt = 0.001f;
    } else {
        dt = 1.0e-6f * (now - _last_update_us);
        _last_update_us = now;
    }

    switch (state){
        case ENGINE_CONTROL_STOP:
            // set throttle ramp to decrease speed to zero, this happens instantly inside update_throttle_ramp()
            update_throttle_ramp(0.0f, dt);

            // throttle control output forced to zero
            _throttle_output = 0.0f;
            _throttle2_output = 0.0f;
            break;

        case ENGINE_CONTROL_IDLE:
            // set throttle ramp to decrease speed to zero
            update_throttle_ramp(0.0f, dt);

            // set throttle control to idle speed parameter, this happens instantly and ignore ramping
            _throttle_output = _idle_output;
            _throttle2_output = _idle_output;
            break;

        case ENGINE_CONTROL_AUTOTHROTTLE:
            // set throttle ramp to increase to full speed
            update_throttle_ramp(1.0f, dt);

            // single-engine throttle controls
            if (_control_mode == THROTTLE_CONTROL_SINGLE) {
                engine_1_autothrottle_run();

            // twin-engine throttle controls
            } else if (_control_mode == THROTTLE_CONTROL_TWIN) {
                engine_1_autothrottle_run();
                engine_2_autothrottle_run();
            }
            break;
    }

    // update run-up estimate
    update_engine_runup(dt);

    // write throttle outputs to servos
    write_throttle(_aux_fn, _throttle_output);
    if (_control_mode == THROTTLE_CONTROL_TWIN) {
        write_throttle(_aux_fn_2, _throttle2_output);
    }
}

// update_throttle_ramp - slews rotor output scalar between 0 and 1, outputs float scalar to _throttle_ramp_output
void AP_MotorsHeli_Throttle::update_throttle_ramp(float throttle_ramp_input, float dt)
{
    // sanity check ramp time
    if (_ramp_time <= 0) {
        _ramp_time = 1;
    }

    // ramp output upwards towards target
    if (_throttle_ramp_output < throttle_ramp_input) {
        // allow control output to jump to estimated speed
        if (_throttle_ramp_output < _engine_runup_output) {
            _throttle_ramp_output = _engine_runup_output;
        }
        // ramp up slowly to target
        _throttle_ramp_output += (dt / _ramp_time);
        if (_throttle_ramp_output > throttle_ramp_input) {
            _throttle_ramp_output = throttle_ramp_input;
        }
    }else{
        // ramping down happens instantly
        _throttle_ramp_output = throttle_ramp_input;
    }
}

// update_engine_runup - function to slew the runup scalar, outputs float scalar to _rotor_runup_ouptut
void AP_MotorsHeli_Throttle::update_engine_runup(float dt)
{
    // if control mode is disabled, then run-up complete always returns true
    if ( _control_mode == THROTTLE_CONTROL_DISABLED ){
        _runup_complete = true;
        return;
    }

    // if _runup_time is set to zero then rotor speed sensor is being used
    if (_runup_time <= 0) {
        // update run-up complete flag using measured rotor rpm
        if (!_runup_complete && (_rotor_rpm > (_governor_reference * _critical_speed))) {
                _runup_complete = true;
            }
        if (_runup_complete && (_rotor_rpm <= (_governor_reference * _critical_speed))) {
                _runup_complete = false;
        }
    } else {
        // measured rotor speed is not used, estimate rotor speed based on engine runup scalar
        // check rotor runup setting and correct it if misconfigured
        if (_runup_time < _ramp_time) {
            _runup_time = _ramp_time;
        }
        float runup_increment = dt / _runup_time;
        if (_engine_runup_output < _throttle_ramp_output) {
            _engine_runup_output += runup_increment;
            if (_engine_runup_output > _throttle_ramp_output) {
                _engine_runup_output = _throttle_ramp_output;
            }
        } else {
            _engine_runup_output -= runup_increment;
            if (_engine_runup_output < _throttle_ramp_output) {
                _engine_runup_output = _throttle_ramp_output;
            }
        }
        // update run-up complete flag using runup timer
        if (!_runup_complete && (_throttle_ramp_output >= 1.0f) && (_engine_runup_output >= 1.0f)) {
            _runup_complete = true;
        }
        if (_runup_complete && (get_rotor_speed() <= _critical_speed)) {
            _runup_complete = false;
        }
    }
}

// get_rotor_speed - gets rotor speed as an estimate when no speed sensor is installed or has failed
float AP_MotorsHeli_Throttle::get_rotor_speed() const
{
    return _engine_runup_output;
}

// write_throttle - outputs pwm onto output throttle channel
// servo_out parameter is of the range 0 ~ 1
void AP_MotorsHeli_Throttle::write_throttle(SRV_Channel::Aux_servo_function_t aux_function, float servo_out)
{
    if (_control_mode == THROTTLE_CONTROL_DISABLED){
        // do not do servo output to avoid conflicting with other output on the channel
        return;
    } else {
        SRV_Channels::set_output_scaled(aux_function, (uint16_t) (servo_out * 1000));
    }
}

// calculate_throttlecurve - uses throttle curve and collective input to determine throttle setting
float AP_MotorsHeli_Throttle::calculate_throttlecurve(float collective_in)
{
    const float inpt = collective_in * 4.0f + 1.0f;
    uint8_t idx = constrain_int16(int8_t(collective_in * 4), 0, 3);
    const float a = inpt - (idx + 1.0f);
    const float b = (idx + 1.0f) - inpt + 1.0f;
    float throttle = _throttlecurve_poly[idx][0] * a + _throttlecurve_poly[idx][1] * b + _throttlecurve_poly[idx][2] * (powf(a,3.0f) - a) / 6.0f + _throttlecurve_poly[idx][3] * (powf(b,3.0f) - b) / 6.0f;

    throttle = constrain_float(throttle, 0.0f, 1.0f);
    return throttle;
}

// throttle curve for engine #2
float AP_MotorsHeli_Throttle::calculate_throttlecurve2(float collective_in)
{
    const float inpt = collective_in * 4.0f + 1.0f;
    uint8_t idx = constrain_int16(int8_t(collective_in * 4), 0, 3);
    const float a = inpt - (idx + 1.0f);
    const float b = (idx + 1.0f) - inpt + 1.0f;
    float throttle2 = _throttlecurve2_poly[idx][0] * a + _throttlecurve2_poly[idx][1] * b + _throttlecurve2_poly[idx][2] * (powf(a,3.0f) - a) / 6.0f + _throttlecurve2_poly[idx][3] * (powf(b,3.0f) - b) / 6.0f;

    throttle2 = constrain_float(throttle2, 0.0f, 1.0f);
    return throttle2;
}

// calculate autothrottle for engine #1
void AP_MotorsHeli_Throttle::engine_1_autothrottle_run()
{
    float throttlecurve = calculate_throttlecurve(_collective_in);
    if (!_governor_on) {
        _governor_output = 0.0f;
        _governor_engage = false;
        _throttle_torque_reference = 0.0f;
        _governor_fault = false;    //resets a governor hard fault only if governor switch OFF

        // AutoThrottle OFF if RC8 input less than throttle curve position
        // we don't use rotor_ramp_output on manual throttle
        if (_throttle_input < throttlecurve) {
            _autothrottle_on = false;
            _throttle_output = constrain_float((_idle_output + (_throttle_input - _idle_output)), 0.0f, 1.0f);
        } else {
            // AutoThrottle ON - throttle ramp timer will be used if set to non-zero value
            if (!_autothrottle_on) {
                _autothrottle_on = true;
            }
        _throttle_output = constrain_float(_idle_output + (_throttle_ramp_output * (throttlecurve - _idle_output)), 0.0f, 1.0f);
        }
    } else {
        // GOVERNOR ON - governor can never be active unless system is on AutoThrottle
        // but manual throttle position can override AutoThrottle allowing in-flight shutdown of either engine
        // while governor is still active on the other engine
        if (_throttle_input < throttlecurve) {
            _governor_output = 0.0f;
            _governor_engage = false;
            _throttle_torque_reference = 0.0f;
            _autothrottle_on = false;
            _throttle_output = constrain_float((_idle_output + _throttle_input), 0.0f, 1.0f);
        } else {
            // AutoThrottle is active - calculate governor droop and torque limit
            // governor requires miminum 50% of normal headspeed to initialize or will go to throttle curve
            if (!_autothrottle_on) {
                _autothrottle_on = true;
            }
            if (!_governor_fault && _rotor_rpm > (_governor_reference * 0.5f)) {
                // torque limiter accelerates rotor to the reference speed at constant torque
                if (!_governor_engage && _rotor_rpm < _governor_reference) {
                    float torque_limit = (_governor_torque * _governor_torque);
                    _governor_output = (_rotor_rpm / _governor_reference) * torque_limit;
                    _throttle_output = constrain_float(_idle_output + (_throttle_ramp_output * (throttlecurve + _governor_output - _idle_output)), 0.0f, 1.0f);
                    // initial torque reference is set at the throttle it takes to reach governor reference speed
                    _throttle_torque_reference = _throttle_output;
                } else {
                    // if governor is engaged in rotor over-speed torque reference is set to current throttle curve position
                    if (!_governor_engage && _throttle_torque_reference < 0.2f) {
                        _throttle_torque_reference = throttlecurve;
                    }
                    // governor engaged status with droop compensator
                    if (!_governor_engage) {
                        _governor_engage = true;
                    }
                    // governor fault detection - must maintain Rrpm -3/+1%
                    // this will lock out governor engage from overspeed in excess of 1%
                    // rpm fault detector will allow a fault to persist for 500ms
                    if ((_rotor_rpm < _governor_reference * 0.97f) || (_rotor_rpm > _governor_reference * 1.01f)) {
                        _governor_fault_timer += 1.0f;
                        if (_governor_fault_timer > 200.0f) {
                            _governor_fault = true;
                            gcs().send_text(MAV_SEVERITY_WARNING, "Governor Fault: RPM Range");
                        }
                    } else {
                        _governor_fault_timer = 0.0f;
                    }
                    if (!_governor_fault) {
                        // droop response adjusts governor sensitivity to speed droop
                        float governor_droop = (_governor_reference - _rotor_rpm) * _governor_droop_response;
                        // throttle curve provides feedforward in governor response
                        _governor_output = governor_droop + ((throttlecurve - _throttle_torque_reference) *  _governor_tcgain);
                        // throttle torque reference is captured at governor engage and is governor baseline
                        // if rotor rpm is low or high by more than 2 rpm, increment the torque reference
                        // at 1% throttle/second until rotor speed matches governor reference speed,
                        // at which point governor is in steady-state control
                        if (_rotor_rpm < (_governor_reference - 2.0f)) {
                            _throttle_torque_reference += 0.000025f;
                        } else if (_rotor_rpm > (_governor_reference + 2.0f)) {
                            _throttle_torque_reference -= 0.000025f;
                        } else {
                            _throttle_torque_reference = _throttle_torque_reference;
                        }
                        _throttle_output = constrain_float(_idle_output + (_throttle_ramp_output * (_throttle_torque_reference + _governor_output - _idle_output)), throttlecurve * _governor_tcgain, 1.0f);
                    }
                }
            } else {
                // governor is inactive due to speed sensor failure or rotor speed too low
                _governor_output = 0.0f;
                _governor_engage = false;
                _throttle_torque_reference = 0.0f;
                _throttle_output = _idle_output + (_throttle_ramp_output * (throttlecurve - _idle_output));
            }
        }
    }
}

// calculate autothrottle for engine #2
void AP_MotorsHeli_Throttle::engine_2_autothrottle_run()
{
    float throttlecurve2 = calculate_throttlecurve2(_collective_in);
    if (!_governor_on) {
        _governor2_output = 0.0f;
        _governor2_engage = false;
        _throttle2_torque_reference = 0.0f;
        _governor2_fault = false;    //resets a governor hard fault only if governor switch OFF

        // AutoThrottle OFF if RC7 input less than throttle curve position
        // we don't use rotor_ramp_output on manual throttle
        if (_throttle2_input < throttlecurve2) {
            _autothrottle2_on = false;
            _throttle2_output = constrain_float((_idle_output + (_throttle2_input - _idle_output)), 0.0f, 1.0f);
        } else {
            // AutoThrottle ON - throttle ramp timer will be used if set to non-zero value
            if (!_autothrottle2_on) {
                _autothrottle2_on = true;
            }
            _throttle2_output = constrain_float(_idle_output + (_throttle_ramp_output * (throttlecurve2 - _idle_output)), 0.0f, 1.0f);
        }
    } else {
        // GOVERNOR ON - governor can never be active unless system is on AutoThrottle
        // but manual throttle position can override AutoThrottle allowing in-flight shutdown of either engine
        // while governor is still active on the other engine
        if (_throttle2_input < throttlecurve2) {
            _governor2_output = 0.0f;
            _throttle2_torque_reference = 0.0f;
            _governor2_engage = false;
            _throttle2_torque_reference = 0.0f;
            _autothrottle2_on = false;
            _throttle2_output = constrain_float((_idle_output + _throttle2_input), 0.0f, 1.0f);
        } else {
            // AutoThrottle is active - calculate governor droop and torque limit
            // governor requires miminum 50% of normal headspeed to initialize or will go to throttle curve
            if (!_autothrottle2_on) {
                _autothrottle2_on = true;
            }
            if (!_governor_fault && _rotor_rpm > (_governor_reference * 0.5f)) {
                // torque limiter accelerates rotor to the reference speed at constant torque
                if (!_governor2_engage && _rotor_rpm < _governor_reference) {
                    float torque2_limit = (_governor_torque * _governor_torque);
                    _governor2_output = (_rotor_rpm / _governor_reference) * torque2_limit;
                    _throttle2_output = constrain_float(_idle_output + (_throttle_ramp_output * (throttlecurve2 + _governor2_output - _idle_output)), 0.0f, 1.0f);
                    _throttle2_torque_reference = _throttle2_output;
                } else {
                    // if governor is engaged in rotor over-speed torque reference is set to current throttle curve position
                    if (!_governor2_engage && _throttle2_torque_reference < 0.2f) {
                        _throttle2_torque_reference = throttlecurve2;
                    }
                    // governor engaged status with droop compensator
                    if (!_governor2_engage) {
                        _governor2_engage = true;
                    }
                    // governor fault detection - must maintain Rrpm -3/+1%
                    // the same timer is used for both engines so if a fault does not clear it will disengage
                    // the governor on both engines in 250ms
                    if ((_rotor_rpm < _governor_reference * 0.97f) || (_rotor_rpm > _governor_reference * 1.01f)) {
                        _governor_fault_timer += 1.0f;
                        if (_governor_fault_timer > 200.0f) {
                            _governor2_fault = true;
                            gcs().send_text(MAV_SEVERITY_WARNING, "Governor Engine2 Fault: RPM Range");
                        }
                    } else {
                        _governor_fault_timer = 0.0f;
                    }
                    if (!_governor2_fault) {
                        // droop response adjusts governor sensitivity to speed droop
                        float governor2_droop = (_governor_reference - _rotor_rpm) * _governor2_droop_response;
                        // throttle curve provides feedforward in governor response
                        _governor2_output = governor2_droop + ((throttlecurve2 - _throttle2_torque_reference) *  _governor2_tcgain);
                        // throttle torque reference is captured at governor engage and is governor baseline
                        // if rotor rpm is low or high by more than 2 rpm, increment the torque reference
                        // at 1% throttle/second until rotor speed matches governor reference speed,
                        // at which point governor is in steady-state control
                        if (_rotor_rpm < (_governor_reference - 2.0f)) {
                            _throttle2_torque_reference += 0.000025f;
                        } else if (_rotor_rpm > (_governor_reference + 2.0f)) {
                            _throttle2_torque_reference -= 0.000025f;
                        } else {
                            _throttle2_torque_reference = _throttle2_torque_reference;
                        }
                        _throttle2_output = constrain_float(_idle_output + (_throttle_ramp_output * (_throttle2_torque_reference + _governor_output - _idle_output)), throttlecurve2 * _governor2_tcgain, 1.0f);
                    }
                }
            } else {
                // governor is inactive due to speed sensor failure or rotor speed too low
                _governor2_output = 0.0f;
                _throttle2_torque_reference = 0.0f;
                _governor2_engage = false;
                _throttle2_torque_reference = 0.0f;
                _throttle2_output = _idle_output + (_throttle_ramp_output * (throttlecurve2 - _idle_output));
            }
        }
    }
}
