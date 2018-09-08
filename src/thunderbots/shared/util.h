#pragma once
#include <math.h>

typedef struct
{
    float x, y;
} Vector2D;

float dot2D(Vector2D first, Vector2D second);

Vector2D toLocalCoords(Vector2D point, float orientationAngle);