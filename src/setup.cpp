//
// Created by bendstein on 12/27/2025.
//
#include "setup.h"
#include "gp2y0e02b.h"

bool try_configure_gp2y0e02b(gp2y0e02b::distance_sensor* sensor)
{
    bool success = true;

    //Perform soft reset of sensor to make sure
    //all settings are at initial default
    if (!sensor->try_soft_reset())
        success = false;

    if (!sensor->try_apply_distance_shift(gp2y0e02b::shift_bit::cm_128))
        success = false;

    //Return whether all operations succeeded
    return success;
}