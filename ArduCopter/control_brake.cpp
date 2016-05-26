/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_brake.pde - init and run calls for brake flight mode
 */

// brake_init - initialise brake controller
bool Copter::brake_init(bool ignore_checks)
{
    return true;
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void Copter::brake_run()
{
    // disarm motors
    init_disarm_motors();
}

void Copter::brake_timeout_to_loiter_ms(uint32_t timeout_ms)
{
    brake_timeout_start = millis();
    brake_timeout_ms = timeout_ms;
}