#include "firmware/app/control/physbot.h"

#include "firmware/app/control/bangbang.h"
#include "firmware/shared/physics.h"

#define TIME_HORIZON 0.05f  // s

PhysBot app_physbot_create(float velocity_x, float velocity_y, float position_x,
                           float position_y, float orientation, float *destination,
                           float *major_vec, float *minor_vec)
{
    float v[2]            = {velocity_x, velocity_y};
    float dr[2]           = {destination[0] - position_x, destination[1] - position_y};
    const Component major = {.disp  = shared_physics_dot2D(major_vec, dr),
                             .vel   = shared_physics_dot2D(major_vec, v),
                             .accel = 0,
                             .time  = 0};
    const Component minor = {.disp  = shared_physics_dot2D(minor_vec, dr),
                             .vel   = shared_physics_dot2D(minor_vec, v),
                             .accel = 0,
                             .time  = 0};

    // current orientation is directly subtracted from destination because we do not
    // assume that the user wants the rotational displacement to be the min angle between
    // the destination and current orientation
    const Component rot = {.disp = destination[2] - orientation};
    PhysBot pb          = {.rot = rot, .maj = major, .min = minor};
    for (unsigned i = 0; i < 2; i++)
    {
        pb.pos[i]       = dr[i];
        pb.major_vec[i] = major_vec[i];
        pb.minor_vec[i] = minor_vec[i];
    }
    return pb;
}

void app_physbot_planMove(Component *c, float *p)
{
    BBProfile profile;
    app_bangbang_prepareTrajectoryMaxV(&profile, c->disp, c->vel, p[0], p[1], p[2]);
    app_bangbang_planTrajectory(&profile);
    c->accel = app_bangbang_computeAvgAccel(&profile, TIME_HORIZON);
    c->time  = app_bangbang_computeProfileDuration(&profile);
}

void app_physbot_computeAccelInLocalCoordinates(float *accel, PhysBot pb, float angle,
                                                float *major_vec, float *minor_vec)
{
    float local_norm_vec[2][2] = {{cosf(angle), sinf(angle)},
                                  {cosf(angle + P_PI / 2), sinf(angle + P_PI / 2)}};
    for (int i = 0; i < 2; i++)
    {
        accel[i] = pb.min.accel * shared_physics_dot2D(local_norm_vec[i], minor_vec);
        accel[i] += pb.maj.accel * shared_physics_dot2D(local_norm_vec[i], major_vec);
    }
}
