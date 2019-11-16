#ifndef PRIMITIVES_IMU_TEST_H
#define PRIMITIVES_IMU_TEST_H

#include <unused.h>

#include "control/control.h"
#include "io/dr.h"
#include "io/wheels.h"
#include "math.h"
#include "physics/physics.h"
#include "primitive.h"

#define SPACE_FACTOR 0.01f
#define TICK 0.005f
#define MAX_VX_STEP MAX_X_A* TICK
#define MAX_VY_STEP MAX_Y_A* TICK

extern const primitive_t IMU_TEST_PRIMITIVE;

#endif
