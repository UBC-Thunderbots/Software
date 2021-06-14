#include "io/dr.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "firmware/shared/physics.h"
#include "io/dsp.h"
#include "io/encoder.h"
#include "io/sensors.h"
#include "shared/constants.h"
#include "util/circbuff.h"

#define SPEED_SIZE 100
#define BASE_CAMERA_DELAY 3

static dr_data_t current_state;
static dr_ball_data_t current_ball_state;

/**
 * \brief The most recent ball and camera data frames.
 */
static robot_camera_data_t robot_camera_data;
static ball_camera_data_t ball_camera_data;
static wheel_speeds_t past_wheel_speeds[SPEED_SIZE];

// Variables for hard coded drive pattern (testing)
static uint16_t tick_count = 0;

// Function pre-declarations
void dr_log(log_record_t *log);

/**
 * \brief called a system boot to configure deadreckoning system
 */
void dr_init(void)
{
    current_state.x     = 0.0f;
    current_state.y     = 0.0f;
    current_state.angle = 0.0f;

    current_ball_state.x  = 0.0f;
    current_ball_state.y  = 0.0f;
    current_ball_state.vx = 0.0f;
    current_ball_state.vy = 0.0f;

    ball_camera_data.x         = 0.0;
    ball_camera_data.y         = 0.0;
    ball_camera_data.timestamp = 0;
    circbuff_init(past_wheel_speeds, SPEED_SIZE);
}


/**
 * \brief called on tick after accel and gyros are read to update state
 *
 * \param[out] log the log record to fill, if any
 */
void dr_tick(log_record_t *log)
{
    tick_count++;

    // New camera data- update dead reckoning
    if (robot_camera_data.new_data)
    {
        dr_apply_cam();
        robot_camera_data.new_data = false;
    }
    float encoder_speeds[4];
    float wheel_speeds[3];

    for (unsigned int i = 0; i < 4; i++)
    {
        encoder_speeds[i] = (float)encoder_speed(i) * QUARTERDEGREE_TO_MS;
    }
    // Todo: Check for wheel slippage

    shared_physics_legacySpeed4ToSpeed3(encoder_speeds, wheel_speeds);
    wheel_speeds[2] =
        wheel_speeds[2] / ROBOT_MAX_RADIUS_METERS;  // Convert to angular velocity (rad/s)

    wheel_speeds_t wheel_speed_object;
    wheel_speed_object.speed_x     = wheel_speeds[0];
    wheel_speed_object.speed_y     = wheel_speeds[1];
    wheel_speed_object.speed_angle = wheel_speeds[2];

    // Store speeds for use when new camera data arrives
    // Speeds are stored in local reference frame- they are converted to global in
    // integration process
    add_to_circ_buff(past_wheel_speeds, SPEED_SIZE, wheel_speed_object);

    // Convert to global reference frame for state update
    shared_physics_rotate(wheel_speeds, current_state.angle);

    // Update the current velocity to match wheel velocities
    current_state.vx   = wheel_speeds[0];
    current_state.vy   = wheel_speeds[1];
    current_state.avel = wheel_speeds[2];

    // Update position by integrating velocities
    current_state.x += current_state.vx * TICK_TIME;
    current_state.y += current_state.vy * TICK_TIME;
    current_state.angle += current_state.avel * TICK_TIME;

    if (current_state.angle > P_PI)
        current_state.angle -= 2 * P_PI;
    else if (current_state.angle < -P_PI)
        current_state.angle += 2 * P_PI;

    // Update ball positions
    current_ball_state.x += current_ball_state.vx * TICK_TIME;
    current_ball_state.y += current_ball_state.vy * TICK_TIME;
    if (log)
    {
        dr_log(log);
    }
}

/**
 * \brief provides current state information about the robot to caller
 * \param ret The struct to populate with the robot data
 */
void dr_get(dr_data_t *ret)
{
    *ret = current_state;
}

/**
 * \brief provides current state information about the ball to caller
 * \param ret The struct to populate with the ball data
 */
void dr_get_ball(dr_ball_data_t *ret)
{
    *ret = current_ball_state;
}

/**
 * \brief Get the x-component of the robot's position
 * \return The x-component of the robot's position
 */
float dr_get_robot_position_x(void)
{
    return current_state.x;
}

/**
 * \brief Get the y-component of the robot's position
 * \return The y-component of the robot's position
 */
float dr_get_robot_position_y(void)
{
    return current_state.y;
}

/**
 * \brief Get the robot's orientation
 * \return The orientation of the robot
 */
float dr_get_robot_orientation(void)
{
    return current_state.angle;
}

/**
 * \brief Get the x-component of the robot's velocity
 * \return The x-component of the robot's velocity
 */
float dr_get_robot_velocity_x(void)
{
    return current_state.vx;
}

/**
 * \brief Get the y-component of the robot's velocity
 * \return The y-component of the robot's velocity
 */
float dr_get_robot_velocity_y(void)
{
    return current_state.vy;
}

/**
 * \brief Get the robot's angular velocity
 * \return The robot's angular velocity
 */
float dr_get_robot_angular_velocity(void)
{
    return current_state.avel;
}

/**
 * \brief Get the x-component of the ball's position
 * \return The x-component of the ball's position
 */
float dr_get_ball_position_x(void)
{
    return current_ball_state.x;
}

/**
 * \brief Get the y-component of the ball's position
 * \return The y-component of the ball's position
 */
float dr_get_ball_position_y(void)
{
    return current_ball_state.y;
}

/**
 * \brief Get the x-component of the ball's velocity
 * \return The x-component of the ball's velocity
 */
float dr_get_ball_velocity_x(void)
{
    return current_ball_state.vx;
}

/**
 * \brief Get the y-component of the ball's velocity
 * \return The y-component of the ball's velocity
 */
float dr_get_ball_velocity_y(void)
{
    return current_ball_state.vy;
}

/**
 * \brief Sets the robot's camera frame.
 */
void dr_set_robot_frame(int16_t x, int16_t y, int16_t angle)
{
    // Check that the old data has been processed first
    // before overwriting with new camera data.
    // Provides some thread safety
    if (robot_camera_data.new_data == false)
    {
        robot_camera_data.x        = (float)(x / 1000.0);
        robot_camera_data.y        = (float)(y / 1000.0);
        robot_camera_data.angle    = (float)(angle / 1000.0);
        robot_camera_data.new_data = true;
    }
}


void dr_apply_cam(void)
{
    float x     = robot_camera_data.x;
    float y     = robot_camera_data.y;
    float angle = robot_camera_data.angle;

    wheel_speeds_t wheel_speed;

    float wheel_speeds[3];

    // int additional_delay = (int)(robot_camera_data.timestamp)/((int)(1000*TICK_TIME));
    // //In number of robot ticks Todo: make sure delay is less than size of circ buffer
    int total_delay = BASE_CAMERA_DELAY + 5;  // + additional_delay;
    for (int i = total_delay; i >= 0; i--)
    {
        wheel_speed = get_from_circ_buff(past_wheel_speeds, SPEED_SIZE, i);

        wheel_speeds[0] = wheel_speed.speed_x;
        wheel_speeds[1] = wheel_speed.speed_y;
        wheel_speeds[2] = wheel_speed.speed_angle;

        shared_physics_rotate(wheel_speeds, angle);
        x += wheel_speeds[0] * TICK_TIME;
        y += wheel_speeds[1] * TICK_TIME;
        angle += wheel_speeds[2] * TICK_TIME;
    }

    angle = fmod(angle, 2 * P_PI);
    if (angle > P_PI)
        angle -= 2 * P_PI;

    current_state.x     = x;
    current_state.y     = y;
    current_state.angle = angle;

    // TODO: apply ball data, linearly extrapolate

    float ball_pos[2] = {ball_camera_data.x, ball_camera_data.y};
    float ball_vel[2] = {current_ball_state.vx, current_ball_state.vy};

    for (int i = total_delay; i >= 0; i--)
    {
        ball_pos[0] += ball_vel[0] * TICK_TIME;
        ball_pos[1] += ball_vel[1] * TICK_TIME;
    }
    current_ball_state.x  = ball_pos[0];
    current_ball_state.y  = ball_pos[1];
    current_ball_state.vx = ball_vel[0];
    current_ball_state.vy = ball_vel[1];
}


/**
 * \brief Sets the ball's camera frame.
 */
void dr_set_ball_frame(int16_t x, int16_t y)
{
    ball_camera_data.x = (float)(x / 1000.0);
    ball_camera_data.y = (float)(y / 1000.0);
}


/**
 * \brief Sets the ball's camera frame and timestamp.
 */
void dr_set_ball_frame_timestamp(int16_t x, int16_t y, uint64_t timestamp)
{
    float new_x    = (float)(x / 1000.0);
    float new_y    = (float)(y / 1000.0);
    uint64_t new_t = timestamp;

    float delta_x = new_x - ball_camera_data.x;
    float delta_y = new_y - ball_camera_data.y;
    float delta_t = (float)(new_t - ball_camera_data.timestamp) / 1000.0;

    if (delta_t > 0)
    {
        current_ball_state.vx = delta_x / delta_t;
        current_ball_state.vy = delta_y / delta_t;
    }

    ball_camera_data.x = new_x;
    ball_camera_data.y = new_y;
}

/**
 * \brief Sets the robot's camera frame timestamp.
 */
void dr_set_robot_timestamp(uint64_t timestamp)
{
    robot_camera_data.timestamp = timestamp;
}

/**
 * \brief Sets the ball's camera frame timestamp.
 */
void dr_set_ball_timestamp(uint64_t timestamp)
{
    ball_camera_data.timestamp = timestamp;
}


void dr_log(log_record_t *log)
{
    sensors_gyro_data_t gyrodata;
    sensors_accel_data_t acceldata;
    float wheel_speeds[3];

    gyrodata  = sensors_get_gyro();
    acceldata = sensors_get_accel();

    for (unsigned int i = 0; i < 3; i++)
    {
        wheel_speeds[i] = (float)encoder_speed(i) * QUARTERDEGREE_TO_RPM;
    }

    log->tick.dr_x     = current_state.x;
    log->tick.dr_y     = current_state.y;
    log->tick.dr_angle = current_state.angle;
    log->tick.dr_vx    = current_state.vx;
    log->tick.dr_vy    = current_state.vy;
    log->tick.dr_avel  = current_state.avel;

    log->tick.enc_vx          = wheel_speeds[0];
    log->tick.enc_vy          = wheel_speeds[1];
    log->tick.enc_avel        = wheel_speeds[2];
    log->tick.accelerometer_x = acceldata.data.reading.x;
    log->tick.accelerometer_y = acceldata.data.reading.y;
    log->tick.accelerometer_z = acceldata.data.reading.z;

    log->tick.gyro_avel = MS_PER_GYRO * gyrodata.data.reading.z;

    log->tick.cam_x     = robot_camera_data.x;
    log->tick.cam_y     = robot_camera_data.y;
    log->tick.cam_angle = robot_camera_data.angle;

    log->tick.cam_ball_x = ball_camera_data.x;
    log->tick.cam_ball_y = ball_camera_data.y;

    log->tick.cam_delay = (uint16_t)robot_camera_data.timestamp;
}
