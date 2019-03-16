#include "physbot.h"
#include "../dr.h"
#include "../bangbang.h"
#include "../physics.h"

PhysBot setup_bot(dr_data_t states, float destination[3], float major_vec[2], 
    float minor_vec[2]) {
    float v[2] = {states.vx, states.vy};
    float dr[2] = {destination[0] - states.x, destination[1] - states.y};
    PhysBot pb = {
        .rot = {
            .disp = min_angle_delta(states.angle, destination[2])
        },
        .maj = {
            .disp = dot2D(major_vec, dr),
            .vel = dot2D(major_vec, v),
            .accel = 0,
            .time = 0
        },
        .min = {
            .disp = dot2D(minor_vec, dr),
            .vel = dot2D(minor_vec, v),
            .accel = 0,
            .time = 0
        }
    };
    unsigned i;
    for (i = 0; i < 2; i++) {
        pb.dr[i] = dr[i];
        pb.major_vec[i] = major_vec[i];
        pb.minor_vec[i] = minor_vec[i];
    }
    return pb;
}

// need the ifndef here so that we can ignore this code when compiling
// the firmware tests
#ifndef FWTEST
void plan_move(Component *c, float p[3]) {
    BBProfile profile;
    PrepareBBTrajectoryMaxV(&profile, c->disp, c->vel, p[0], p[1], p[2]); 
    PlanBBTrajectory(&profile);
    c->accel = BBComputeAvgAccel(&profile, TIME_HORIZON);
    c->time = GetBBTime(&profile);
}
#endif

void to_local_coords(float accel[3], PhysBot pb, float angle, float major_vec[2], 
    float minor_vec[2]) {
    float local_norm_vec[2][2] = {
        {cosf(angle), sinf(angle)}, 
        {cosf(angle + P_PI / 2), sinf(angle + P_PI / 2)}
    };
    for (int i = 0; i < 2; i++) {
        accel[i] =  pb.min.accel * dot2D(local_norm_vec[i], minor_vec);
        accel[i] += pb.maj.accel * dot2D(local_norm_vec[i], major_vec); 
    }
}