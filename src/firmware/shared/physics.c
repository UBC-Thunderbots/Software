#include "firmware/shared/physics.h"

#include <math.h>
#include <stdint.h>

#include "firmware/shared/physics_linear_algebra.h"

// Wheel angles for these matricies are (55, 135, 225, 305) degrees
// these matrices may be derived as per omnidrive paper:
// http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf


// TODO (#2099): Remove these matrices once the conversion functions are based on the
// wheel angles
//
// This matrix is a unitless matrix which takes the force/speed exerted
// on the floor by each of the four wheels and converts it into forces/speed
// within the robot domain, the third component is is newtons and not
// torque as the matrix is unitless (multiply by ROBOT_MAX_RADIUS_METERS to unnormalize)
// the transpose of this matrix is the velocity coupling matrix and can
// convert speeds in the robot coordinates into linear wheel speeds
static const float shared_physics_wheels_to_local_vel_matrix_transpose[3][4] = {
    {-0.8192f, -0.7071f, 0.7071f, 0.8192f},
    {0.5736f, -0.7071f, -0.7071f, 0.5736f},
    {1.0000f, 1.0000f, 1.0000f, 1.0000f}};

// Transformation matrix to convert a 4 velocity/force to a 3 velocity/force (derived as
// pinv(shared_physics_force4ToForce3^t)
static const float shared_physics_local_vel_to_wheels_matrix[3][4] = {
    {-0.3498f, -0.3019f, 0.3019f, 0.3498f},
    {0.3904f, -0.3904f, -0.3904f, 0.3904f},
    {0.2761f, 0.2239f, 0.2239f, 0.2761f}};

void shared_physics_speed4ToSpeed3(const float speed4[4], float speed3[3],
                                   float front_wheel_angle_deg,
                                   float back_wheel_angle_deg)
{
    // TODO (#2099): use wheel angles to implement this properly
    (void)front_wheel_angle_deg;
    (void)back_wheel_angle_deg;
    shared_physics_linear_algebra_matrixMultiply(
        speed3, 3, speed4, 4, shared_physics_local_vel_to_wheels_matrix);
}

void shared_physics_legacySpeed4ToSpeed3(const float speed4[4], float speed3[3])
{
    shared_physics_linear_algebra_matrixMultiply(
        speed3, 3, speed4, 4, shared_physics_local_vel_to_wheels_matrix);
}

void shared_physics_speed3ToSpeed4(const float speed3[3], float speed4[4],
                                   float front_wheel_angle_deg,
                                   float back_wheel_angle_deg)
{
    // TODO (#2099): use wheel angles to implement this properly
    (void)front_wheel_angle_deg;
    (void)back_wheel_angle_deg;
    shared_physics_linear_algebra_matrixMultiplyTranspose(
        speed4, 4, speed3, 3, shared_physics_wheels_to_local_vel_matrix_transpose);
}

void shared_physics_force3ToForce4(float force3[3], float force4[4],
                                   float front_wheel_angle_deg,
                                   float back_wheel_angle_deg)
{
    // TODO (#2099): use wheel angles to implement this properly
    (void)front_wheel_angle_deg;
    (void)back_wheel_angle_deg;
    shared_physics_linear_algebra_matrixMultiplyTranspose(
        force4, 4, force3, 3, shared_physics_local_vel_to_wheels_matrix);
}

// return the minimum angle from angle1 to angle2
// test with angle2 increased or decreased by 2pi
float shared_physics_minAngleDelta(float angle1, float angle2)
{
    angle1 = fmodf(angle1, 2 * P_PI);
    angle2 = fmodf(angle2, 2 * P_PI);
    if (angle2 >= angle1)
    {
        float ang_sub = angle2 - 2 * P_PI;
        if ((ang_sub - angle1) * (ang_sub - angle1) <=
            (angle2 - angle1) * (angle2 - angle1))
        {
            return (ang_sub - angle1);
        }
        else
        {
            return (angle2 - angle1);
        }
    }
    else
    {
        float ang_plus = angle2 + 2 * P_PI;
        if ((ang_plus - angle1) * (ang_plus - angle1) <=
            (angle2 - angle1) * (angle2 - angle1))
        {
            return (ang_plus - angle1);
        }
        else
        {
            return (angle2 - angle1);
        }
    }
}

float shared_physics_norm2(float a1, float a2)
{
    return (sqrtf(a1 * a1 + a2 * a2));
}

/**
 * \ingroup Physics
 *
 * \brief implements 2D rotation matrix
 *
 * \param[in,out] the speed to rotate
 * \param[in] amount in radian to rotate
 */
void shared_physics_rotate(float speed[2], float angle)
{
    float temp = cosf(angle) * speed[0] - sinf(angle) * speed[1];
    speed[1]   = sinf(angle) * speed[0] + cosf(angle) * speed[1];
    speed[0]   = temp;
}

/**
 * Use for dot product on arbitrarily large arrays.
 *
 * @param vec1 the first vector in the dot product
 * @param vec2 the second vector in the dot product
 * @param size the size of the vectors. They should be the same size
 * @return the dot product result of the vectors
 */
float shared_physics_dotProduct(const float vec1[], const float vec2[], const int size)
{
    float result = 0;
    for (int i = 0; i < size; i++)
    {
        result += (vec1[i] * vec2[i]);
    }
    return result;
}

/**
 * Dot product for 2D vectors.
 *
 * @param vec1 the first vector in the dot product
 * @param vec2 the second vector in the dot product
 * @return the dot product result of the vectors
 */
float shared_physics_dot2D(float vec1[2], float vec2[2])
{
    return vec1[0] * vec2[0] + vec1[1] * vec2[1];
}

float shared_physics_getFinalSpeed(const float initial_speed, const float displacement,
                                   const float acceleration)
{
    // Source:
    // https://www.khanacademy.org/science/physics/one-dimensional-motion/kinematic-formulas/a/what-are-the-kinematic-formulas
    // vf^2 = vi^2 + 2ad
    return sqrtf(powf(initial_speed, 2) + 2 * displacement * acceleration);
}
