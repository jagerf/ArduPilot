/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"



// drop_init - initialise land controller
bool Copter::drop_init(bool ignore_checks)
{

    drop_time_start = millis();

    chute = true;
    init_drop = true;

    relay.on(0);
    return true;

}

// drop_run - runs the land controller
// should be called at 100hz or more
void Copter::drop_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;
    float cmb_rate = 0;
    int32_t diff = millis()-drop_time_start;


    if(diff < 2000){

        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch,
                                                                            target_yaw_rate, get_smoothing_gain());
        pos_control.accel_to_throttle(0);


    }else{

        RC_Channel_aux::set_radio(RC_Channel_aux::k_cam_trigger, 1300);

        if(init_drop) {

            motors.set_throttle_range(0, 1100, 1450);

            init_drop = false;
        }

        if(diff < (g.drop_hold_time+2000)) {

            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

            // call attitude controller
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch,
                                                                                target_yaw_rate, get_smoothing_gain());
            drop_acc_increment = ((g.drop_acc/(g.drop_hold_time/1000)) * (diff/1000));

            pos_control.accel_to_throttle(drop_acc_increment);



        }else{
            if(diff < (g.drop_time+g.drop_hold_time+2000)) {

                // set motors to full range
                motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

                // call attitude controller
                attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(target_roll, target_pitch,
                                                                                    target_yaw_rate, get_smoothing_gain());

                pos_control.accel_to_throttle(g.drop_acc);


            }else{

                if(chute) {

                    relay.off(0);
                    // send message to gcs and dataflash
                    gcs_send_text(MAV_SEVERITY_INFO, "Parachute: Released");
                    Log_Write_Event(DATA_PARACHUTE_RELEASED);

                    // disarm motors
                    init_disarm_motors();

                    // release parachute
                    parachute.release();
                    chute = false;
                }
            }
        }
    }



}
