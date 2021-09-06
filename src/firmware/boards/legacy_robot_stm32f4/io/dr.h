#ifndef DR_H
#define DR_H

#include "util/log.h"

// In ticks
#define BASE_CAMERA_DELAY 3
#define SPEED_SIZE 100


// gyro running at 2000/second and in integers such that 32767 is 2000
// 61.0 millidegrees/second / LSB
#define DEGREES_PER_GYRO (61.0f / 1000.0f)
#define MS_PER_DEGREE (2.0f * (float)P_PI * ROBOT_MAX_RADIUS_METERS / 360.0f)
#define MS_PER_GYRO (MS_PER_DEGREE * DEGREES_PER_GYRO)

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
typedef struct
{
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

typedef struct
{
    /**
     * \brief The X component of the ball’s accumulated motion.
     */
    float x;

    /**
     * \brief The Y component of the ball’s accumulated motion.
     */
    float y;

    /**
     * \brief The X component of the ball’s velocity.
     */
    float vx;

    /**
     * \brief The Y component of the ball’s velocity.
     */
    float vy;

} dr_ball_data_t;


typedef struct
{
    /**
     * \brief The x component of the robot's global position in metres.
     */
    float x;

    /**
     * \brief The y component of the robot's global position in metres.
     */
    float y;

    /**
     * \brief The theta component of the robot's position in the global frame in rad.
     */
    float angle;

    /**
     * \brief The timestamp associated with this camera frame.
     */
    uint64_t timestamp;

    /**
     * \brief Whether the camera data has been updated since last tick
     */
    bool new_data;

} robot_camera_data_t;


typedef struct
{
    /**
     * \brief The x component of the ball's global position in metres.
     */
    float x;

    /**
     * \brief The y component of the ball's global position in metres.
     */
    float y;

    /**
     * \brief The timestamp associated with this camera frame.
     */
    uint64_t timestamp;

} ball_camera_data_t;


void dr_init(void);
bool dr_calibrated(void);
void dr_reset(void);
void dr_tick(log_record_t *log);
void dr_get(dr_data_t *ret);
void dr_get_ball(dr_ball_data_t *ret);
float dr_get_robot_position_x(void);
float dr_get_robot_position_y(void);
float dr_get_robot_orientation(void);
float dr_get_robot_velocity_x(void);
float dr_get_robot_velocity_y(void);
float dr_get_robot_angular_velocity(void);
float dr_get_ball_position_x(void);
float dr_get_ball_position_y(void);
float dr_get_ball_velocity_x(void);
float dr_get_ball_velocity_y(void);
void dr_set_robot_frame(int16_t x, int16_t y, int16_t angle);
void dr_apply_cam(void);
void dr_set_ball_frame(int16_t x, int16_t y);
void dr_set_ball_frame_timestamp(int16_t x, int16_t y, uint64_t timestamp);
void dr_set_robot_timestamp(uint64_t timestamp);
void dr_set_ball_timestamp(uint64_t timestamp);
#endif
