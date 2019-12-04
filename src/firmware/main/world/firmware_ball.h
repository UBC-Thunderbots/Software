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
 *                            ball
 * @param get_ball_position_y A function that can be called to get the y-position of the
 *                            ball
 * @param get_ball_velocity_x A function that can be called to get the x-position of the
 *                            ball
 * @param get_ball_velocity_y A function that can be called to get the y-position of the
 *                            ball
 *
 * @return A pointer to a ball that will use the given methods to get the ball state,
 *         ownership of the ball is given to the caller
 */
FirmwareBall* FirmwareBall_create(float (*get_ball_position_x)(),
                                  float (*get_ball_position_y)(),
                                  float (*get_ball_velocity_x)(),
                                  float (*get_ball_velocity_y)());

/**
 * Destroy the given ball, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed into
 *       `FirmwareBall_create`
 *
 * @param ball The ball to destroy
 */
void FirmwareBall_destroy(FirmwareBall* ball);

/**
 * Get the x-position of the given ball
 * @param ball The ball to get the x-position for
 * @return The x-position of the given
 */
float FirmwareBall_getPositionX(FirmwareBall* ball);

/**
 * Get the y-position of the given ball
 * @param ball The ball to get the y-position for
 * @return The y-position of the given
 */
float FirmwareBall_getPositionY(FirmwareBall* ball);

/**
 * Get the x component of velocity of the given ball
 * @param ball The ball to get the x component of velocity for
 * @return The x component of velocity of the given ball
 */
float FirmwareBall_getVelocityX(FirmwareBall* ball);

/**
 * Get the y component of velocity of the given ball
 * @param ball The ball to get the y component of velocity for
 * @return The y component of velocity of the given ball
 */
float FirmwareBall_getVelocityY(FirmwareBall* ball);
