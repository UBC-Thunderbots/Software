#include "world/firmware_ball.h"

#include <stdlib.h>

struct FirmwareBall
{
    float (*get_ball_position_x)();
    float (*get_ball_position_y)();
    float (*get_ball_velocity_x)();
    float (*get_ball_velocity_y)();
};

FirmwareBall* FirmwareBall_create(float (*get_ball_position_x)(),
                                  float (*get_ball_position_y)(),
                                  float (*get_ball_velocity_x)(),
                                  float (*get_ball_velocity_y)())
{
    FirmwareBall* new_ball = malloc(sizeof(FirmwareBall));

    new_ball->get_ball_position_x = get_ball_position_x;
    new_ball->get_ball_position_y = get_ball_position_y;
    new_ball->get_ball_velocity_x = get_ball_velocity_x;
    new_ball->get_ball_velocity_y = get_ball_velocity_y;

    return new_ball;
}

void FirmwareBall_destroy(FirmwareBall* ball)
{
    free(ball);
}

float FirmwareBall_getPositionX(FirmwareBall* ball)
{
    return ball->get_ball_position_x();
}

float FirmwareBall_getPositionY(FirmwareBall* ball)
{
    return ball->get_ball_position_y();
}

float FirmwareBall_getVelocityX(FirmwareBall* ball)
{
    return ball->get_ball_velocity_x();
}

float FirmwareBall_getVelocityY(FirmwareBall* ball)
{
    return ball->get_ball_position_y();
}
