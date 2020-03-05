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

#include "AP_MotorsHeli_Swash.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Swash::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Swashplate Type
    // @Description: H3_120/H3_140 plates have SERVO1 left side, SERVO2 right side, SERVO3 (elevator) in rear. HR3_120/HR3_140 have SERVO1 right side, SERVO2 left side, SERVO3 (elevator) in front. For HR3 style swashplate use H3_120/H3_140 and reverse servo and collective directions as necessary. For all H3_90 swashplates use H4_90 and don't use servo output for the missing servo. For H4-90 SERVOS1 and 2 are left/right respectively, SERVOS3 and 4 are rear/front respectively. For H4-45 SERVOS1 and 2 are LF/RF, SERVOS3 and 4 are LR/RR. For four-servo swashplates fourth servo output must be enabled on Port5 with SERVO output labeled Cyclic4 
    // @Values: 1:H1 non-CPPM,2:H3_140,3:H3_120,4:H4_90,5:H4_45
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_MotorsHeli_Swash, _swashplate_type, SWASHPLATE_TYPE_H3_120),

    // @Param: COLL_DIR
    // @DisplayName: Collective Control Direction
    // @Description: Direction collective moves to increase blade pitch. 0 for Normal, 1 for Reversed
    // @Values: 0:Normal,1:Reversed
    // @User: Standard
    AP_GROUPINFO("COLL_DIR", 2, AP_MotorsHeli_Swash, _swash_collective_direction, COLLECTIVE_DIRECTION_NORMAL),

    // @Param: LINEAR
    // @DisplayName: Linearize swashplate servo mechanical throw
    // @Description: This setting is primarily for four-servo swashplates to prevent servo binding. It linearizes the swashplate servo's mechanical output to account for nonlinear output due to arm rotation.  This requires a specific mechanical setup procedure to work properly.  The servo arm must be centered on the mechanical throw at the servo trim position and the servo trim position kept as close to 1500 pwm as possible. Leveling the swashplate can only be done through the swash links. After the swashplate is level the pitch links should be adjusted at a value of 1500 pwm to achieve zero blade pitch
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("LINEAR", 3, AP_MotorsHeli_Swash, _linear_swash_servo, 0),
   
    AP_GROUPEND
};

// configure - configure the swashplate settings for any updated parameters
void AP_MotorsHeli_Swash::configure()
{

    _swash_type = static_cast<SwashPlateType>(_swashplate_type.get());
    _collective_direction = static_cast<CollectiveDirection>(_swash_collective_direction.get());
    _make_servo_linear = _linear_swash_servo;
}

// CCPM Mixers - calculate mixing scale factors by swashplate type
void AP_MotorsHeli_Swash::calculate_roll_pitch_collective_factors()
{
    if (_swash_type == SWASHPLATE_TYPE_H1) {
        // CCPM mixing not used
        _collectiveFactor[CH_1] = 0;
        _collectiveFactor[CH_2] = 0;
        _collectiveFactor[CH_3] = 1;
    } else if ((_swash_type == SWASHPLATE_TYPE_H4_90) || (_swash_type == SWASHPLATE_TYPE_H4_45)) {
        // collective mixer for four-servo CCPM
        _collectiveFactor[CH_1] = 1;
        _collectiveFactor[CH_2] = 1;
        _collectiveFactor[CH_3] = 1;
        _collectiveFactor[CH_4] = 1;
    } else {
        // collective mixer for three-servo CCPM
        _collectiveFactor[CH_1] = 1;
        _collectiveFactor[CH_2] = 1;
        _collectiveFactor[CH_3] = 1;
    }
        
        // defined swashplates
    if (_swash_type == SWASHPLATE_TYPE_H3_140) {
        // Three-servo roll/pitch mixer for H3-140
        // HR3-140 uses reversed servo and collective direction in heli setup
        _rollFactor[CH_1] = 1;
        _rollFactor[CH_2] = -1;
        _rollFactor[CH_3] = 0;
        _pitchFactor[CH_1] = 1;
        _pitchFactor[CH_2] = 1;
        _pitchFactor[CH_3] = -1;
    } else if (_swash_type == SWASHPLATE_TYPE_H3_120) {
        // three-servo roll/pitch mixer for H3-120
        // HR3-120 uses reversed servo and collective direction in heli setup
        _rollFactor[CH_1] = 0.866025f;
        _rollFactor[CH_2] = -0.866025f;
        _rollFactor[CH_3] = 0;
        _pitchFactor[CH_1] = 0.5f;
        _pitchFactor[CH_2] = 0.5f;
        _pitchFactor[CH_3] = -1;
    } else if (_swash_type == SWASHPLATE_TYPE_H4_90) {
        // four-servo roll/pitch mixer for H4-90
        // can also be used for all versions of 90 deg three-servo swashplates
        _rollFactor[CH_1] = 1; 
        _rollFactor[CH_2] = -1;
        _rollFactor[CH_3] = 0;
        _rollFactor[CH_4] = 0;
        _pitchFactor[CH_1] = 0;
        _pitchFactor[CH_2] = 0;
        _pitchFactor[CH_3] = -1;
        _pitchFactor[CH_4] = 1;
    } else if (_swash_type == SWASHPLATE_TYPE_H4_45) {
        // four-servo roll/pitch mixer for H4-45
        // for 45 deg plates servos 1&2 are LF&RF, 3&4 are LR&RR.
        _rollFactor[CH_1] = 0.707107f; 
        _rollFactor[CH_2] = -0.707107f;
        _rollFactor[CH_3] = 0.707107f;
        _rollFactor[CH_4] = -0.707107f;
        _pitchFactor[CH_1] = 0.707107f;
        _pitchFactor[CH_2] = 0.707107f;
        _pitchFactor[CH_3] = -0.707f;
        _pitchFactor[CH_4] = -0.707f;
    } else {
        // CCPM mixing not being used, so H1 straight outputs
        _rollFactor[CH_1] = 1;
        _rollFactor[CH_2] = 0;
        _rollFactor[CH_3] = 0;
        _pitchFactor[CH_1] = 0;
        _pitchFactor[CH_2] = 1;
        _pitchFactor[CH_3] = 0;
    }
}

// get_servo_out - calculates servo output
float AP_MotorsHeli_Swash::get_servo_out(int8_t ch_num, float pitch, float roll, float collective) const
{
    // Collective control direction. Swash moves up for negative collective pitch, down for positive collective pitch
    if (_collective_direction == COLLECTIVE_DIRECTION_REVERSED){
        collective = 1 - collective;
    }

    float servo = ((_rollFactor[ch_num] * roll) + (_pitchFactor[ch_num] * pitch))*0.45f + _collectiveFactor[ch_num] * collective;
    if (_swash_type == SWASHPLATE_TYPE_H1 && (ch_num == CH_1 || ch_num == CH_2)) {
        servo += 0.5f;
    }

    // rescale from -1..1, so we can use the pwm calc that includes trim
    servo = 2.0f * servo - 1.0f;

    if (_make_servo_linear == 1) {
        servo = get_linear_servo_output(servo);
    }

    return servo;
}

// set_linear_servo_out - sets swashplate servo output to be linear
float AP_MotorsHeli_Swash::get_linear_servo_output(float input) const
{

    input = constrain_float(input, -1.0f, 1.0f);

    //servo output is calculated by normalizing input to 50 deg arm rotation as full input for a linear throw
    return safe_asin(0.766044f * input) * 1.145916;

}

