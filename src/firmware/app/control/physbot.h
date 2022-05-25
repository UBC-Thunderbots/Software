#pragma once

/**
 * component to build information along major axis, minor axis, or rotation.
 * disp is the displacement along that axis.
 * vel is the velocity along the axis.
 * accel is the acceleration along the axis.
 * time is the time that is calculated by the primitive.
 */
typedef struct
{
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
 * pos is the x, y values for displacement on the global axis
 * major_vec is the x, y components of the major axis on the global axis
 * minor_vec is the x, y components of the minor axis on the global axis
 * min is a Component container to hold minor axis information
 * maj is a Component container to hold major axis information
 * rot is a Component container to hold rotational axis information
 */
typedef struct
{
    float pos[2];
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
 * @param velocity_x the robot's x component of velocity
 * @param velocity_y the robot's y component of velocity
 * @param position_x the robot's x component of position
 * @param position_y the robot's y component of position
 * @param orientation the robot's orientation
 * @param destination a 3 length array of {x, y, rotation} destination values
 * on the global axis
 * @param major_vec the major vector components on the global axis
 * @param minor_vec the minor vector components on the global axis
 * @return a new PhysBot data container
 */
PhysBot app_physbot_create(float velocity_x, float velocity_y, float position_x,
                           float position_y, float orientation, float *destination,
                           float *major_vec, float *minor_vec);

/**
 * Creates the BBProfile for a component. It is assumed that the displacement,
 * velocity, and acceleration lie along the major or minor axis (i.e. the
 * Component given is a major or minor axis component).
 *
 * @param c A major or minor axis component that contains displacement and
 * velocity information
 * @param p A 3 length array of {final velocity, max acceleration, max_velocity}
 */
void app_physbot_planMove(Component *c, float *p);

/**
 * Uses a rotation matrix to rotate the acceleration vectors of the given
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
 */
void app_physbot_computeAccelInLocalCoordinates(float *accel, PhysBot pb, float angle,
                                                float *major_vec, float *minor_vec);
