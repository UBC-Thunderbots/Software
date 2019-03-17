#ifndef DR_H
#define DR_H

#include <stdbool.h>

// In ticks
#define LOG_TICK_T 0.03

#define BASE_CAMERA_DELAY 3
#define SPEED_SIZE 100

/**
 * \brief The type of data returned by the dead reckoning module.
 *
 * The following units are used:
 * \li Linear positions are in metres.
 * \li Orientations are in radians.
 * \li Linear velocities are in metres per second.
 * \li Angular velocities are in radians per second.
 *
 * The following coordinate system is used:
 * \li Positive X is in the direction the robot was facing at the last call to
 * \ref dr_reset.
 * \li Positive Y is 90° to the left of positive X.
 * \li Positive angles are leftward rotation.
 *
 * The origin is the position and orientation of the robot at the last call to
 * \ref dr_reset.
 */
typedef struct {
	/**
	 * \brief The X component of the robot’s accumulated motion.
	 */
	float x;

	/**
	 * \brief The Y component of the robot’s accumulated motion.
	 */
	float y;

	/**
	 * \brief The angular component of the robot’s accumulated motion.
	 */
	float angle;

	/**
	 * \brief The X component of the robot’s velocity.
	 */
	float vx;

	/**
	 * \brief The Y component of the robot’s velocity.
	 */
	float vy;

	/**
	 * \brief The robot’s angular velocity.
	 */
	float avel;

} dr_data_t;


void dr_get(dr_data_t*);
void sim_apply_wheel_force(const float wheel_force[4]);
void sim_tick(float delta_t);
void sim_log_tick(float time);
void sim_log_start();
void sim_log_end();
void sim_reset();
float get_pos_x();

#endif//
