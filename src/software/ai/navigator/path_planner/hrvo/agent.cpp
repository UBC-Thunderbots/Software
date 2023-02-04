//
// Created by boris on 2023-02-04.
//

#include "agent.h"

Agent::Agent(double min_radius, double max_speed, double max_accel, double max_radius_inflation)
    : min_radius(min_radius), max_speed(max_speed), max_accel(max_accel), max_radius_inflation(max_radius_inflation)
{

}

double Agent::getMaxAccel() const
{
    return max_accel;
}


