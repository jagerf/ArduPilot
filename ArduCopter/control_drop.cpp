//
// Created by David Forés Diaz on 17/5/16.
//

#include "Copter.h"



// drop_init - initialise land controller
bool Copter::drop_init(bool ignore_checks) {

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-1200, 500);

    
    //pos_control.set_accel_z(wp_nav.get_accel_z());


    return true;


}


// land_run - runs the land controller
// should be called at 100hz or more
void Copter::drop_run()
{

    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;


    pos_control.accel_to_throttle(-981);
    pos_control.update_z_controller();


}
