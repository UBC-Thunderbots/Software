#include "firmware/app/world/firmware_ball.h"

#include <stdlib.h>

struct FirmwareBall
{
    float (*get_ball_position_x)(void);
    float (*get_ball_position_y)(void);
    float (*get_ball_velocity_x)(void);
    float (*get_ball_velocity_y)(void);
};

FirmwareBall_t* app_firmware_ball_create(float (*get_ball_position_x)(void),
                                         float (*get_ball_position_y)(void),
                                         float (*get_ball_velocity_x)(void),
                                         float (*get_ball_velocity_y)(void))
{
    FirmwareBall_t* new_ball = (FirmwareBall_t*)malloc(sizeof(FirmwareBall_t));

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

float app_firmware_ball_getPositionX(const FirmwareBall_t* ball)
{
    return ball->get_ball_position_x();
}

float app_firmware_ball_getPositionY(const FirmwareBall_t* ball)
{
    return ball->get_ball_position_y();
}

float app_firmware_ball_getVelocityX(const FirmwareBall_t* ball)
{
    return ball->get_ball_velocity_x();
}

float app_firmware_ball_getVelocityY(const FirmwareBall_t* ball)
{
    return ball->get_ball_velocity_y();
}
