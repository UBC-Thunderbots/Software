#pragma once

/**
 * This struct represents the ball from the perspective of firmware
 */
typedef struct FirmwareBall FirmwareBall_t;

/**
 * Create a ball with methods for accessing it's state
 *
 * NOTE: All positions are in global field coordinates (ie. 0,0 is the center of the
 *       field)
 *
 * @param get_ball_position_x A function that can be called to get the x-position of the
 *                            ball, in meters
 * @param get_ball_position_y A function that can be called to get the y-position of the
 *                            ball, in meters
 * @param get_ball_velocity_x A function that can be called to get the x-position of the
 *                            ball, in meters per second
 * @param get_ball_velocity_y A function that can be called to get the y-position of the
 *                            ball, in meters per second
 *
 * @return A pointer to a ball that will use the given methods to get the ball state,
 *         ownership of the ball is given to the caller
 */
FirmwareBall_t* app_firmware_ball_create(float (*get_ball_position_x)(void),
                                         float (*get_ball_position_y)(void),
                                         float (*get_ball_velocity_x)(void),
                                         float (*get_ball_velocity_y)(void));

/**
 * Destroy the given ball, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed to the
 *       `create` function
 *
 * @param ball The ball to destroy
 */
void app_firmware_ball_destroy(FirmwareBall_t* ball);

/**
 * Get the x-position of the given ball
 *
 * @param ball The ball to get the x-position for
 *
 * @return The x-position of the given ball, in meters, in global field coordinates
 */
float app_firmware_ball_getPositionX(const FirmwareBall_t* ball);

/**
 * Get the y-position of the given ball
 *
 * @param ball The ball to get the y-position for
 *
 * @return The y-position of the given ball, in meters, in global field coordinates
 */
float app_firmware_ball_getPositionY(const FirmwareBall_t* ball);

/**
 * Get the x component of velocity of the given ball
 *
 * @param ball The ball to get the x component of velocity for
 *
 * @return The x component of velocity of the given ball, in meters per second,
 *         in global field coordinates
 */
float app_firmware_ball_getVelocityX(const FirmwareBall_t* ball);

/**
 * Get the y component of velocity of the given ball
 *
 * @param ball The ball to get the y component of velocity for
 *
 * @return The y component of velocity of the given ball, in meters per second,
 *         in global field coordinates
 */
float app_firmware_ball_getVelocityY(const FirmwareBall_t* ball);
