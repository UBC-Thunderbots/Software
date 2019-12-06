#include "app/world/firmware_ball.h"

#include <stdlib.h>

struct FirmwareBall
{
    float (*get_ball_position_x)();
    float (*get_ball_position_y)();
    float (*get_ball_velocity_x)();
    float (*get_ball_velocity_y)();
};

FirmwareBall_t* app_firmware_ball_create(float (*get_ball_position_x)(),
                                         float (*get_ball_position_y)(),
                                         float (*get_ball_velocity_x)(),
                                         float (*get_ball_velocity_y)())
{
    FirmwareBall_t* new_ball = malloc(sizeof(FirmwareBall_t));

    new_ball->get_ball_position_x = get_ball_position_x;
    new_ball->get_ball_position_y = get_ball_position_y;
    new_ball->get_ball_velocity_x = get_ball_velocity_x;
    new_ball->get_ball_velocity_y = get_ball_velocity_y;

    return new_ball;
}

void app_firmware_ball_destroy(FirmwareBall_t* ball)
{
    free(ball);
}

float app_firmware_ball_getPositionX(FirmwareBall_t* ball)
{
    return ball->get_ball_position_x();
}

float app_firmware_ball_getPositionY(FirmwareBall_t* ball)
{
    return ball->get_ball_position_y();
}

float app_firmware_ball_getVelocityX(FirmwareBall_t* ball)
{
    return ball->get_ball_velocity_x();
}

float app_firmware_ball_getVelocityY(FirmwareBall_t* ball)
{
    return ball->get_ball_velocity_y();
}
