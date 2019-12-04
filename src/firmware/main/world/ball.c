#include "world/ball.h"

#include <stdlib.h>

struct Ball
{
    float (*get_ball_position_x)();
    float (*get_ball_position_y)();
    float (*get_ball_velocity_x)();
    float (*get_ball_velocity_y)();
};

Ball* Ball_create(float (*get_ball_position_x)(), float (*get_ball_position_y)(),
                  float (*get_ball_velocity_x)(), float (*get_ball_velocity_y)())
{
    Ball* new_ball = malloc(sizeof(Ball));

    new_ball->get_ball_position_x = get_ball_position_x;
    new_ball->get_ball_position_y = get_ball_position_y;
    new_ball->get_ball_velocity_x = get_ball_velocity_x;
    new_ball->get_ball_velocity_y = get_ball_velocity_y;

    return new_ball;
}

void Ball_destroy(Ball* ball){
    free ball;
}

float Ball_getPositionX(Ball* ball)
{
    return ball->get_ball_position_x();
}

float Ball_getPositionY(Ball* ball)
{
    return ball->get_ball_position_x();
}

float Ball_getVelocityX(Ball* ball)
{
    return ball->get_ball_position_x();
}

float Ball_getVelocityY(Ball* ball)
{
    return ball->get_ball_position_x();
}
