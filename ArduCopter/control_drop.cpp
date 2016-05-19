/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"



// drop_init - initialise land controller
bool Copter::drop_init(bool ignore_checks)
{
    // get position controller's target altitude above terrain
    Location_Class target_loc = pos_control.get_pos_target();
    int32_t target_alt_cm;

    // get altitude target above home by default
    target_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_HOME, target_alt_cm);

    if(target_alt_cm >= 100) {
        // initialize vertical speeds and leash lengths
        // pos_control.set_speed_z(g.k_param_drop_max_vel, g.k_param_drop_max_vel);
        // pos_control.set_accel_z(wp_nav.get_accel_z());
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);

        return true;
    }else{
        return false;
    }

}

// drop_run - runs the land controller
// should be called at 100hz or more
void Copter::drop_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // get position controller's target altitude above terrain
    Location_Class target_loc = pos_control.get_pos_target();
    int32_t target_alt_cm;

    // get altitude target above home by default
    target_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_HOME, target_alt_cm);

    if(target_alt_cm >= 30) {
        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch, target_yaw_rate,
                                                                            get_smoothing_gain());

        pos_control.accel_to_throttle(g.drop_acc);

        //pos_control.update_z_controller();
    }
    else{

        auto_land_run();

    }

}
