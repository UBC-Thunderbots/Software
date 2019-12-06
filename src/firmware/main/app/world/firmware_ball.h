#pragma once

/**
 * This struct represents the ball from the perspective of firmware
 */
struct FirmwareBall;
typedef struct FirmwareBall FirmwareBall;

/**
 * Create a ball with methods for accessing it's state
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
FirmwareBall* app_firmware_ball_create(float (*get_ball_position_x)(),
                                       float (*get_ball_position_y)(),
                                       float (*get_ball_velocity_x)(),
                                       float (*get_ball_velocity_y)());

/**
 * Destroy the given ball, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed to the
 *       `create` function
 *
 * @param ball The ball to destroy
 */
void app_firmware_ball_destroy(FirmwareBall* ball);

/**
 * Get the x-position of the given ball
 * @param ball The ball to get the x-position for
 * @return The x-position of the given ball, in meters
 */
float app_firmware_ball_getPositionX(FirmwareBall* ball);

/**
 * Get the y-position of the given ball
 * @param ball The ball to get the y-position for
 * @return The y-position of the given ball, in meters
 */
float app_firmware_ball_getPositionY(FirmwareBall* ball);

/**
 * Get the x component of velocity of the given ball
 * @param ball The ball to get the x component of velocity for
 * @return The x component of velocity of the given ball, in meters per second
 */
float app_firmware_ball_getVelocityX(FirmwareBall* ball);

/**
 * Get the y component of velocity of the given ball
 * @param ball The ball to get the y component of velocity for
 * @return The y component of velocity of the given ball, in meters per second
 */
float app_firmware_ball_getVelocityY(FirmwareBall* ball);
