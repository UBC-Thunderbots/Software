#ifndef PHYSBOT_H
#define PHYSBOT_H

#include "../dr.h"

// Used for computing accelerations
#define TIME_HORIZON 0.05f //s

/**
 * component to build information along major axis, minor axis, or rotation.
 * disp is the displacement along that axis.
 * vel is the velocity along the axis.
 * accel is the acceleration along the axis.
 * time is the time that is calculated by the primitive.
 */
typedef struct {
    float disp;
    float vel;
    float accel;
    float time;  
} Component;


/**
 * data container for components and information of the robot.
 * use this to pass information around between functions so that we don't
 * have to have functions with a ton of arguments. 
 * 
 * dr is the x, y values for displacement on the global axis
 * major_vec is the x, y components of the major axis on the global axis
 * minor_vec is the x, y components of the minor axis on the global axis
 * min is a Component container to hold minor axis information
 * maj is a Component container to hold major axis information
 * rot is a Component container to hold rotational axis information
 */
typedef struct {
    float dr[2];
    float major_vec[2];
    float minor_vec[2];
    Component min;
    Component maj;
    Component rot;
} PhysBot;

/**
 * Call this function from a primitive's tick function to set up a PhysBot
 * data container with all the information about the robot that you could
 * possibly need for the primitive. It contains major and minor axis information
 * as well as data about the bot on the global axis. Details about what
 * can go in the PhysBot are in the physbot.h file.
 * 
 * @param states The current state of the robot
 * @param destination a 3 length array of {x, y, rotation} destination values
 * on the global axis
 * @param major_vec the major vector components on the global axis
 * @param minor_vec the minor vector components on the global axis
 * @return a new PhysBot data container
 */ 
PhysBot setup_bot(dr_data_t states, float destination[3], float major_vec[2], 
    float minor_vec[2]);

/**
 * Creates the BBProfile for a component. It is assumed that the displacement, 
 * velocity, and acceleration lie along the major or minor axis (i.e. the 
 * Component given is a major or minor axis component). 
 * 
 * @param c A major or minor axis component that contains displacement and
 * velocity information
 * @param p A 3 length array of {final velocity, max acceleration, max_velocity}
 * @return void
 */
void plan_move(Component *c, float p[3]);

/**
 * Uses a rotaion matrix to rotate the acceleration vectors of the given 
 * PhysBot back to local xy coordinates and store them in a separate array. The
 * given angle should be the bot's angle relative to the global x-axis.
 * 
 * @param accel a 3 length array of {0, 0, rotational acceleration}. It will
 * contain {x, y, rotational} accelerations in local axes afterwards
 * @param pb A data container that has major and minor axis accelerations stored
 * in it
 * @param angle the angle that the robot is facing
 * @param major_vec the major vector components on the global axis
 * @param minor_vec the minor vector components on the global axis
 * @return void
 */
void to_local_coords(float accel[3], PhysBot pb, float angle, float major_vec[2], 
    float minor_vec[2]);

#endif
