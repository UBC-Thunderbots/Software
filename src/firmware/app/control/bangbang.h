#pragma once
/*
 * This file implements a bang bang trajectory planner with optional control methods on
 * top to use this library first prepare a trajectory via one of the Perpare functions.
 * These functions serve as a simple wrapper to direct the user which fields of the
 * Profile struct to compete. The next stage is to call Plan which will analyze the inputs
 * and complete the trajectory computing things like which way to accelerate, for how long
 * and what speed will be reached. Finally, various pieces of information can be gleaned
 * from the completed trajectory such as the total traversal
 * time(app_bangbang_computeProfileDuration), or the expected stat at a given time in the
 * future(app_bangbang_getState). Finally, users may wish to use the information to
 * compute an acceleration to apply as a form of control law.
 *
 * There are many control laws which one could apply to a planned trajectory, such as
 * simply taking a1 and using that. However, such simple approaches will tend to oscillate
 * severely and their use is discouraged. The default method is app_bangbang_computeAccel
 * which takes a point in the future at which to look. At this future time it computes the
 * expected velocity and position and then asks the question, if I were to apply a
 * trajectory of constant jerk from now until this final horizon time, what should my
 * initial acceleration and jerk be to achieve this final state.
 *
 * It should be noted however that there are many possible control laws which could be
 * mapped on top of a computed BB plan. Take for example the plan of looking some time
 * into the future and applying the average acceleration which can be found there. It is
 * up to the user how the best consume this plan for their purposes.
 *
 * As a final note, this planner could be used once at the beginning of a motion or once
 * every controller iteration. Again it is up to the user to decide what is the best
 * application.
 */

// a full trajectory plan for a vehicle undergoing maximum acceleration control
typedef struct
{
    // inputs
    // The distance to travel
    float Distance;
    // The starting velocity of the vehicle
    float Vinitial;
    // the terminal velocity upon reaching the destination (currently not read)
    float Vfinal;

    // the following two maxes must be positive
    // maximum allowable acceleration
    float MaxA;
    // maximum allowable velocity (can be infinity)
    float MaxV;

    // computed info
    // The time spent accelerating with acceleration a1
    float t1;
    // the time spent coasting with velocity Vmid
    float t2;
    // the time spent accelerating with acceleration a3
    float t3;

    // see above
    float a1;
    float a3;
    float Vmid;
} BBProfile;


// wrappers to fill out the Profile struct with the input information
void app_bangbang_prepareTrajectoryMaxV(BBProfile *b, float d, float vi, float vf,
                                        float MaxA, float MaxV);
void app_bangbang_prepareTrajectory(BBProfile *b, float d, float vi, float vf,
                                    float MaxA);

// meat and potatoes, computes the relevant details of the trajectory
void app_bangbang_planTrajectory(BBProfile *b);

// the default acceleration calculator, a typical horizon would be on
// the time scale of the vehicle in question
float app_bangbang_computeAccel(const BBProfile *b, float horizon);


// takes the velocity that should be obtained at time horizon and
// apply that average velocity to the wheels
float app_bangbang_computeAvgAccel(const BBProfile *b, float horizon);

// retrieve the state at some point in the future if the plan were
// to be followed exactly
void app_bangbang_getState(const BBProfile *b, float time, float *d, float *v);

// This should perhaps be in physics, this takes an initial and fianl velocoty as well as
// a distance and a time and computes the initial acceleration and jerk to experience over
// that time such that the velocity and distance constraints are met
float app_bangbang_computeInitialAccelerationForConstantJerkProfile(
    float Vinit, float Vfinal, float Distance, float time, float *Jerk);

// Computes how long a movement profile will take
float app_bangbang_computeProfileDuration(BBProfile *b);
