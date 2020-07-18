#include "firmware/shared/physics.h"

#include <math.h>
#include <stdint.h>

#include "firmware/shared/physics_linear_algebra.h"

// Wheel angles for these matricies are (55, 135, 225, 305) degrees
// these matrices may be derived as per omnidrive_kiart paper


// This matrix is a unitless matrix which takes the force exerted
// on the floor by each of the four wheels and converts it into forces
// within the robot domain, the third component is is newtons and not
// torque as the matrix is unitless (multiply by ROBOT_RADIUS to unnormalize)
// the transpose of this matrix is the velocity coupling matrix and can
// convert speeds in the robot coordinates into linear wheel speeds
static const float force4_to_force3_mat[3][4] = {{-0.8192f, -0.7071f, 0.7071f, 0.8192f},
                                                 {0.5736f, -0.7071f, -0.7071f, 0.5736f},
                                                 {1.0000f, 1.0000f, 1.0000f, 1.0000f}};

// Transformation matricies to convert a 4 velocity to
// a 3 velocity (derived as pinv(force4_to_force3^t)
// this is also the transpose of force3_to_force4 mat
static const float speed4_to_speed3_mat[3][4] = {{-0.3498f, -0.3019f, 0.3019f, 0.3498f},
                                                 {0.3904f, -0.3904f, -0.3904f, 0.3904f},
                                                 {0.2761f, 0.2239f, 0.2239f, 0.2761f}};

// mass vector (consists linear robot mass and interial mass)
const float ROBOT_MASS[3] = {ROBOT_POINT_MASS, ROBOT_POINT_MASS, ROT_MASS};

const float MAX_VEL[3] = {MAX_X_V, MAX_Y_V, MAX_T_V* ROBOT_RADIUS};
const float MAX_ACC[3] = {MAX_X_A, MAX_Y_A, MAX_T_A* ROBOT_RADIUS};

/**
 * \ingroup Physics
 *
 * \brief performs the unitless conversion of wheel speeds into robot speeds
 *
 * \param[in] the 4 wheel speeds
 * \param[out] the 3 robot speeds in the same units
 */
void speed4_to_speed3(const float speed4[4], float speed3[3])
{
    matrix_mult(speed3, 3, speed4, 4, speed4_to_speed3_mat);
}

/**
 * \ingroup Physics
 *
 * \brief performs the unitless conversion of the robots speeds into wheel speeds
 *
 * \param[in] the robot speeds in x,y,theta*R coordinates
 * \param[out] the robot wheel speeds in the same units as input
 */
void speed3_to_speed4(const float speed3[3], float speed4[4])
{
    matrix_mult_t(speed4, 4, speed3, 3, force4_to_force3_mat);
}

/**
 * \ingroup Physics
 *
 * Implements the conversion between forces in the robot coordinate system and the
 * Force per wheel. This is nominally equal to the speed3_to_speed4 conversion if
 * the center of mass of the robot coincides with the wheel center, however this is
 * not the case and so when computing wheel forces this should be transform should
 * be used.
 *
 * \brief Implements the conversion from force in robot coordinates to Wheel force
 *
 * \param[in] force in robot coordinates
 * \param[out] force to exert per wheel
 */
void force3_to_force4(float force3[3], float force4[4])
{
    matrix_mult_t(force4, 4, force3, 3, speed4_to_speed3_mat);
}

// return the minimum angle from angle1 to angle2
// test with angle2 increased or decreased by 2pi
float min_angle_delta(float angle1, float angle2)
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

float norm2(float a1, float a2)
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
void rotate(float speed[2], float angle)
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
float dot_product(const float vec1[], const float vec2[], const int size)
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
float dot2D(float vec1[2], float vec2[2])
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
