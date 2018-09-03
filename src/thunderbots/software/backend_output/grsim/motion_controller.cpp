//
// Created by evan on 18/08/18.
//

#include "motion_controller.h"
#include "ai/world/robot.h"
#include "geom/point.h"
#include "geom/angle.h"
#include <utility>
#include <iostream>
#include <chrono>
#include <ctime>


// need to determine these values from experimentation with real robots
const double ROBOT_MAX_ACCELERATION  = 3; // [m/s^2]
const double ROBOT_MAX_ANG_ACCELERATION = 3; // [rad/s^2]

std::pair<Vector2, Double> grSim_bang_bang(Robot robot, Point dest, double desiredFinalSpeed, Angle desiredFinalOrientation, double timeOfLastRun) {

    // pair to hold the X/Y velocities and angular velocity of the robot to send to GrSim
    std::pair <Vector2, double> robotSpeeds;

    double acceleration;
    double timeToDest;


    // calculate the current time (will be used to calculate the time step since the last time the motion controller was run)
    const double currentTime = std::chrono::system_clock::to_time_t( std::chrono::system_clock::now() );
    const double deltaTime = currentTime - timeOfLastRun;


    double distanceToDest = (robot.position() - dest).len();

    // calculates robot angle based on unit vector that points from the robot location to the destination (used to calculate the X/Y velocity magnitudes
    Angle directionAngle = (dest - robot.position()).norm().orientation();

    double angleToDest = (robot.orientation().toRadians() - desiredFinalOrientation);

    // calculate the expected speed at the destination based on current speed and acceleration
    // Vf = sqrt( Vi^2 - 2*a*d)
    double expectedFinalSpeed = sqrt(  pow(robot.velocity(), 2) - 2*ROBOT_MAX_ACCELERATION*distanceToDest );
    double expectedFinalAngSpeed = sqrt( pow(robot.angularVelocity(), 2) - 2*ROBOT_MAX_ANG_ACCELERATION*angleToDest );


    // variables used to hold the change in velocities based on the maximum acceleration and the change in time since the last motion controller run
    double deltaSpeedX, deltaSpeedY, deltaAngularSpeed;

    // the robot can stop in time if it's desired final speed is higher than the speed if the robot maximum decelerates from the current state
    bCanStopInTime = expectedFinalSpeed < desiredFinalSpeed;
    bCanStopRotateInTime = expectedFinalAngSpeed < 0.0;


    if( bCanStopInTime ) {

        // if the robot can stop in time and is going slower than the desired final speed then accelerate
        if( expectedFinalSpeed < desiredFinalSpeed ) {
            deltaSpeedX = (ROBOT_MAX_ACCELERATION * deltaTime)*directionAngle.cos();
            deltaSpeedY = (ROBOT_MAX_ACCELERATION * deltaTime)*directionAngle.sin();
        }

        // if not then maintain speed
        else {
            // maintain speed
            deltaSpeedX = 0.0;
            deltaSpeedY = 0.0;
        }
    }

    // if the robot can't stop in time then decelerate
    else {
        deltaSpeedX = -(ROBOT_MAX_ACCELERATION * deltaTime)*directionAngle.cos();
        deltaSpeedY = -(ROBOT_MAX_ACCELERATION * deltaTime)*directionAngle.sin();
    }

    // if the robot can stop rotating in time
    if ( bCanStopRotateInTime ) {

        // if the final expected angle is less than the desired angle then angularly accelerate
        if( expectedFinalAngSpeed < 0) {
            // angularly accelerate
            deltaAngularSpeed = ROBOT_MAX_ANG_ACCELERATION*deltaTime;
        }

        // if not then maintain angular velocity
        else {
            deltaAngularSpeed = 0.0;
        }
    }

    // if the robot can't stop rotating in time then angular decelerate
    else {
        deltaAngylarSpeed = -(ROBOT_MAX_ANG_ACCELERATION*deltaTime);
    }

    const Vector2 robotXYVelocities = (robot.velocity().x + deltaSpeedX, robot.velocity().y + deltaSpeedY); // calculate new X/Y/ang velocities based on the current robot speeds and delta speeds
    const double robotAngularVelocity = Angle::of_radians(robot.avelocity()) + deltaAngularSpeed;



    return std::make_pair(robotXYVelocities, robotAngularVelocity);
}
