#include "firmware/main/app/primitives/spin.h"

#include <math.h>
#include <stdio.h>

#include "firmware/main/app/control/bangbang.h"
#include "firmware/main/app/control/control.h"
#include "firmware/main/shared/physics.h"

#define TIME_HORIZON 0.5f

static float x_final;
static float y_final;
static float avel_final;
static float end_speed;
static bool slow;

static float major_vec[2];
static float minor_vec[2];
static float major_angle;

static void spin_init(void) {}

/**
 * \brief Starts a movement of this type.
 *
 * This function runs each time the host computer requests to start a spin
 * movement.
 *
 * input to 3->4 matrix is quarter-degrees per 5 ms, matrix is dimensionless
 * linear ramp up for velocity and linear fall as robot approaches point
 * constant angular velocity
 *
 * \param[in] params the movement parameters, which are only valid until this
 * function returns and must be copied into this module if needed
 * \param[in] world The world to perform the primitive in
 */
static void spin_start(const primitive_params_t *p, FirmwareWorld_t *world)
{
    // Parameters:  param[0]: g_destination_x   [mm]
    //              param[1]: g_destination_y   [mm]
    //              param[2]: g_angular_v_final [centi-rad/s]
    //              param[3]: g_end_speed       [millimeter/s]

    // Parse the parameters with the standard units
    x_final    = (float)p->params[0] / 1000.0f;
    y_final    = (float)p->params[1] / 1000.0f;
    avel_final = (float)p->params[2] / 100.0f;
    end_speed  = (float)p->params[3] / 1000.0f;
    slow       = p->slow;

    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);

    // Construct major and minor axis for the path
    // get maginutude
    float distance = norm2(x_final - app_firmware_robot_getPositionX(robot),
                           y_final - app_firmware_robot_getPositionY(robot));

    // major vector - unit vector from start to destination
    major_vec[0] = (x_final - app_firmware_robot_getPositionX(robot)) / distance;
    major_vec[1] = (y_final - app_firmware_robot_getPositionY(robot)) / distance;

    // minor vector - orthogonal to major vector
    minor_vec[0] = -major_vec[1];
    minor_vec[1] = major_vec[0];

    // major angle - angle relative to global x
    major_angle = atan2f(major_vec[1], major_vec[0]);
}

static void spin_end(FirmwareWorld_t *world) {}

static void spin_tick(FirmwareWorld_t *world)
{
    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    // Trajectories
    BBProfile major;
    BBProfile minor;

    // current to destination vector
    float x_disp = x_final - app_firmware_robot_getPositionX(robot);
    float y_disp = y_final - app_firmware_robot_getPositionY(robot);

    // project current to destination vector to major/minor axis
    float major_disp = x_disp * major_vec[0] + y_disp * major_vec[1];
    float minor_disp = x_disp * minor_vec[0] + y_disp * minor_vec[1];

    // project velocity vector to major/minor axis
    const float curr_vx = app_firmware_robot_getVelocityX(robot);
    const float curr_vy = app_firmware_robot_getVelocityY(robot);
    float major_vel     = curr_vx * major_vec[0] + curr_vy * major_vec[1];
    float minor_vel     = curr_vx * minor_vec[0] + curr_vy * minor_vec[1];

    // Prepare trajectory
    app_bangbang_prepareTrajectoryMaxV(&major, major_disp, major_vel, end_speed, MAX_X_A,
                                       MAX_X_V);
    app_bangbang_prepareTrajectoryMaxV(&minor, minor_disp, minor_vel, 0, MAX_Y_A,
                                       MAX_Y_V);

    // Plan
    app_bangbang_planTrajectory(&major);
    app_bangbang_planTrajectory(&minor);

    // Compute acceleration
    float major_accel = app_bangbang_computeAccel(&major, TIME_HORIZON);
    float minor_accel = app_bangbang_computeAccel(&minor, TIME_HORIZON);
    float a_accel = (avel_final - app_firmware_robot_getVelocityAngular(robot)) / 0.05f;

    // Clamp acceleration
    if (a_accel > MAX_T_A)
    {
        a_accel = MAX_T_A;
    }
    if (a_accel < -MAX_T_A)
    {
        a_accel = -MAX_T_A;
    }

    // Local cartesian represented as global cartesian
    const float curr_orientation = app_firmware_robot_getOrientation(robot);
    float local_x_vec[2]         = {cosf(curr_orientation), sinf(curr_orientation)};
    float local_y_vec[2]         = {-sinf(curr_orientation), cosf(curr_orientation)};

    // Get local x acceleration
    float major_dot_x = dot_product(local_x_vec, major_vec, 2);
    float minor_dot_x = dot_product(local_x_vec, minor_vec, 2);
    float x_accel     = major_accel * major_dot_x + minor_accel * minor_dot_x;

    // Get local y acceleration
    float major_dot_y = dot_product(local_y_vec, major_vec, 2);
    float minor_dot_y = dot_product(local_y_vec, minor_vec, 2);
    float y_accel     = major_accel * major_dot_y + minor_accel * minor_dot_y;

    // Apply acceleration in robot's coordinates
    float linear_acc[2] = {
        x_accel,
        y_accel,
    };

    app_control_applyAccel(robot, linear_acc[0], linear_acc[1], a_accel);
}

/**
 * \brief The spin movement primitive.
 */
const primitive_t SPIN_PRIMITIVE = {
    .direct = false,
    .init   = &spin_init,
    .start  = &spin_start,
    .end    = &spin_end,
    .tick   = &spin_tick,
};
