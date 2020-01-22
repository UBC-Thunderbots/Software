#include "firmware/main/app/control/physbot.h"

#include "firmware/main/app/control/bangbang.h"
#include "firmware/main/shared/physics.h"

// TODO: this is redefined in so many places, should be centralized?
// Used for computing accelerations
#define TIME_HORIZON 0.05f  // s

PhysBot create_physbot(const FirmwareRobot_t *robot, float *destination, float *major_vec,
                  float *minor_vec)
{
    float v[2]            = {app_firmware_robot_getVelocityX(robot),
                  app_firmware_robot_getVelocityY(robot)};
    float dr[2]           = {destination[0] - app_firmware_robot_getPositionX(robot),
                   destination[1] - app_firmware_robot_getPositionY(robot)};
    const Component major = {
        .disp = dot2D(major_vec, dr), .vel = dot2D(major_vec, v), .accel = 0, .time = 0};
    const Component minor = {
        .disp = dot2D(minor_vec, dr), .vel = dot2D(minor_vec, v), .accel = 0, .time = 0};
    const Component rot = {.disp = min_angle_delta(
                               app_firmware_robot_getOrientation(robot), destination[2])};
    PhysBot pb          = {.rot = rot, .maj = major, .min = minor};
    for (unsigned i = 0; i < 2; i++)
    {
        pb.pos[i]       = dr[i];
        pb.major_vec[i] = major_vec[i];
        pb.minor_vec[i] = minor_vec[i];
    }
    return pb;
}

void plan_move(Component *c, float p[3])
{
    BBProfile profile;
    PrepareBBTrajectoryMaxV(&profile, c->disp, c->vel, p[0], p[1], p[2]);
    PlanBBTrajectory(&profile);
    c->accel = BBComputeAvgAccel(&profile, TIME_HORIZON);
    c->time  = GetBBTime(&profile);
}

void to_local_coords(float accel[3], PhysBot pb, float angle, float major_vec[2],
                     float minor_vec[2])
{
    float local_norm_vec[2][2] = {{cosf(angle), sinf(angle)},
                                  {cosf(angle + P_PI / 2), sinf(angle + P_PI / 2)}};
    for (int i = 0; i < 2; i++)
    {
        accel[i] = pb.min.accel * dot2D(local_norm_vec[i], minor_vec);
        accel[i] += pb.maj.accel * dot2D(local_norm_vec[i], major_vec);
    }
}