/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_brake.pde - init and run calls for brake flight mode
 */

// brake_init - initialise brake controller
bool Copter::brake_init(bool ignore_checks)
{
    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-500, 500);
    // pos_control.set_accel_z(wp_nav.get_accel_z());

    return true;
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void Copter::brake_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;


    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    pos_control.accel_to_throttle(-200);

    //pos_control.update_z_controller();

}

void Copter::brake_timeout_to_loiter_ms(uint32_t timeout_ms)
{
    brake_timeout_start = millis();
    brake_timeout_ms = timeout_ms;
}

