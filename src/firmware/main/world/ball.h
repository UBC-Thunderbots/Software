#pragma once

struct Ball;
typedef struct Ball Ball;

// TODO: jdocs

Ball* Ball_create(float (*get_ball_position_x)(), float (*get_ball_position_y)(),
                  float (*get_ball_velocity_x)(), float (*get_ball_velocity_y)());
Ball* Ball_destroy(Ball* ball);
float Ball_getPositionX(Ball* ball);
float Ball_getPositionY(Ball* ball);
float Ball_getVelocityX(Ball* ball);
float Ball_getVelocityY(Ball* ball);
