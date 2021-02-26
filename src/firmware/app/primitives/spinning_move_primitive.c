#include "firmware/app/primitives/spinning_move_primitive.h"

#include <math.h>
#include <stdio.h>

#include "firmware/app/control/bangbang.h"
#include "firmware/app/control/control.h"
#include "firmware/shared/physics.h"

#define TIME_HORIZON 0.5f

typedef struct SpinningMovePrimitiveState
{
    float x_final;
    float y_final;
    float avel_final;
    float end_speed;

    float major_vec[2];
    float minor_vec[2];
    float major_angle;
} SpinningMovePrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(SpinningMovePrimitiveState_t)

void app_spinning_move_primitive_start(TbotsProto_SpinningMovePrimitive prim_msg,
                                       void *void_state_ptr, FirmwareWorld_t *world)
{
    SpinningMovePrimitiveState_t *state = (SpinningMovePrimitiveState_t *)void_state_ptr;

    // Parameters:  param[0]: g_destination_x   [mm]
    //              param[1]: g_destination_y   [mm]
    //              param[2]: g_angular_v_final [centi-rad/s]
    //              param[3]: g_end_speed       [millimeter/s]

    // Parse the parameters with the standard units
    state->x_final    = prim_msg.destination.x_meters;
    state->y_final    = prim_msg.destination.y_meters;
    state->avel_final = prim_msg.angular_velocity.radians_per_second;
    state->end_speed  = prim_msg.final_speed_meters_per_second;

    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);

    // Construct major and minor axis for the path
    // get maginutude
    float distance = norm2(state->x_final - app_firmware_robot_getPositionX(robot),
                           state->y_final - app_firmware_robot_getPositionY(robot));

    // major vector - unit vector from start to destination
    state->major_vec[0] =
        (state->x_final - app_firmware_robot_getPositionX(robot)) / distance;
    state->major_vec[1] =
        (state->y_final - app_firmware_robot_getPositionY(robot)) / distance;

    // minor vector - orthogonal to major vector
    state->minor_vec[0] = -state->major_vec[1];
    state->minor_vec[1] = state->major_vec[0];

    // major angle - angle relative to global x
    state->major_angle = atan2f(state->major_vec[1], state->major_vec[0]);

    Dribbler_t *dribbler = app_firmware_robot_getDribbler(robot);
    app_dribbler_setSpeed(dribbler, (uint32_t)prim_msg.dribbler_speed_rpm);
}

static void app_spinning_move_primitive_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    const FirmwareRobot_t *robot = app_firmware_world_getRobot(world);
    const SpinningMovePrimitiveState_t *state =
        (SpinningMovePrimitiveState_t *)void_state_ptr;

    // Trajectories
    BBProfile major;
    BBProfile minor;

    // current to destination vector
    float x_disp = state->x_final - app_firmware_robot_getPositionX(robot);
    float y_disp = state->y_final - app_firmware_robot_getPositionY(robot);

    // project current to destination vector to major/minor axis
    float major_disp = x_disp * state->major_vec[0] + y_disp * state->major_vec[1];
    float minor_disp = x_disp * state->minor_vec[0] + y_disp * state->minor_vec[1];

    // project velocity vector to major/minor axis
    const float curr_vx = app_firmware_robot_getVelocityX(robot);
    const float curr_vy = app_firmware_robot_getVelocityY(robot);
    float major_vel     = curr_vx * state->major_vec[0] + curr_vy * state->major_vec[1];
    float minor_vel     = curr_vx * state->minor_vec[0] + curr_vy * state->minor_vec[1];

    // Prepare trajectory
    app_bangbang_prepareTrajectoryMaxV(&major, major_disp, major_vel, state->end_speed,
                                       MAX_X_A, MAX_X_V);
    app_bangbang_prepareTrajectoryMaxV(&minor, minor_disp, minor_vel, 0, MAX_Y_A,
                                       MAX_Y_V);

    // Plan
    app_bangbang_planTrajectory(&major);
    app_bangbang_planTrajectory(&minor);

    // Compute acceleration
    float major_accel = app_bangbang_computeAccel(&major, TIME_HORIZON);
    float minor_accel = app_bangbang_computeAccel(&minor, TIME_HORIZON);
    float a_accel =
        (state->avel_final - app_firmware_robot_getVelocityAngular(robot)) / 0.05f;

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
    float major_dot_x = dot_product(local_x_vec, state->major_vec, 2);
    float minor_dot_x = dot_product(local_x_vec, state->minor_vec, 2);
    float x_accel     = major_accel * major_dot_x + minor_accel * minor_dot_x;

    // Get local y acceleration
    float major_dot_y = dot_product(local_y_vec, state->major_vec, 2);
    float minor_dot_y = dot_product(local_y_vec, state->minor_vec, 2);
    float y_accel     = major_accel * major_dot_y + minor_accel * minor_dot_y;

    // Apply acceleration in robot's coordinates
    float linear_acc[2] = {
        x_accel,
        y_accel,
    };

    app_control_applyAccel(robot, linear_acc[0], linear_acc[1], a_accel);
}

/**
 * \brief The spinning_move movement primitive.
 */
const primitive_t SPINNING_MOVE_PRIMITIVE = {
    .direct        = false,
    .tick          = &app_spinning_move_primitive_tick,
    .create_state  = &createSpinningMovePrimitiveState_t,
    .destroy_state = &destroySpinningMovePrimitiveState_t};
