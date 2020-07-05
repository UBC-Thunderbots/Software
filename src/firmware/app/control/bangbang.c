#include "bangbang.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

// decreases current by the maxmum of current or limit
// and returns the amount it decreased by
// this is used by app_bangbang_getState for walking the plan
float timeLimit(float *current, float limit)
{
    float temp;
    if (*current < limit)
    {
        temp     = *current;
        *current = 0.0f;
        return temp;
    }
    else
    {
        *current -= limit;
        return limit;
    }
}

// perform a single step on a position and acceleration given an acceleration and deltaT
void stepTime(float *dist, float *vel, float time, float accel)
{
    *dist += accel / 2.0f * time * time + (*vel) * time;
    *vel += accel * time;
}

/**
 * \ingroup Controls
 *
 * \brief implements a 1D bang bang controller
 *
 * \param[out] bang bang trajectory
 * \param[in] distance to travel
 * \param[in] current velocity
 * \param[in] desired final velocity
 * \param[in] maximum acceleration
 *
 */
void app_bangbang_prepareTrajectory(BBProfile *b, float d, float vi, float vf, float MaxA)
{
    if (vf * d < 0)
        vf = 0;  // not allowed- must be in same direction
    b->Distance = d + vf * vf / (2.0f * MaxA);
    b->Vinitial = vi;
    b->MaxA     = MaxA;
    b->MaxV = INFINITY;  // no maxV therefore exceed the speed of light TO INFINITY AND
                         // BEYOND>>>>>>>>>>
}

// Computes how long a movement will take
float app_bangbang_computeProfileDuration(BBProfile *b)
{
    return b->t1 + b->t2 + b->t3;
}

/**
 * \ingroup Controls
 *
 * \brief implements a 1D bang bang controller
 *
 * \param[out] bang bang trajectory
 * \param[in] distance to travel
 * \param[in] current velocity
 * \param[in] desired final velocity
 * \param[in] maximum acceleration
 * \param[in] maximum velocity
 *
 */
void app_bangbang_prepareTrajectoryMaxV(BBProfile *b, float d, float vi, float vf,
                                        float MaxA, float MaxV)
{
    if (vf * d < 0)
        vf = 0;  // not allowed- must be in same direction
    b->Distance = d + vf * vf / (2.0f * MaxA);
    b->Vinitial = vi;
    b->MaxA     = MaxA;
    b->MaxV     = MaxV;
}

// does the actually trajectory planning
// do not call this method directly as it assumes
// that the accelerations are already fixed
// call app_bangbang_planTrajectory instead.
void BBPositivePlan(BBProfile *b)
{
    // first we will assume we are not in the coast condition
    // and hence the time spent in region two is zero
    b->t2 = 0.0f;

    // if one makes the assumpton that a1 and a3 are in different directons
    //(which they should be) then the equation a1*t1 + a3*t3 + vi = vf
    // can be simplified into a1*dT = deltaV, because deltaV and a1 are assumed
    // known we can then compute dT or the difference between t1 and t3 (dT = t1-t3)
    float deltaT = -b->Vinitial / b->a1;

    // next we tackle and attempt to compute t1 by using
    // a1/2*t1^2 + vi*t1 + a3/2*t3^2 + Vm*t3 = d
    // sub in that Vm + a3*t3 = Vf = 0
    // and apply the assumption that a1 = -a3
    // a1/2*(t1^2 + t3^2) + vi*t1 = d
    // then sub in t3 = t1 - dt, some rearrangment and subbng in the definition of dt =
    // -Vi/a1 and we have a1*t1^2 + 2*Vi*t1 + (dT*dT/2*a1 -d) = 0 which is quadratic
    float A = b->a1;
    float B = 2.0f * b->Vinitial;
    float C = deltaT * deltaT / 2.0f * b->a1 - b->Distance;

    // discrimanate of the quadratic equation, if this is negative well idon't know
    float discrm = B * B - 4.0f * A * C;
    if (discrm < 0.0f)
    {
        printf("Failed discrim test\n");  // perhaps we can log this with and error code
                                          // or something
        return;                           // cannot achieve so give it a fail time;
    }

    // two possible solutons to the quadraic equations
    // a and b
    float root = sqrtf(discrm);
    float t1a  = (-B + root) / 2.0f / A;
    float t1b  = (-B - root) / 2.0f / A;
    float t3a  = t1a - deltaT;
    float t3b  = t1b - deltaT;

    // but of course if that solution contains a negative time we should reject it
    bool aGood = (t1a >= 0.0f && t3a >= 0.0f);
    bool bGood = (t1b >= 0.0f && t3b >= 0.0f);

    // eliminate negative solutions
    if (aGood)
    {
        if (bGood)
        {
            // both solutions are fully positive so take the quicker one
            if (t1a + t3a < t1b + t3b)
            {
                b->t1 = t1a;
                b->t3 = t3a;
            }
            else
            {
                b->t1 = t1b;
                b->t3 = t3b;
            }
        }
        else
        {
            b->t1 = t1a;
            b->t3 = t3a;
        }
    }
    else
    {
        if (bGood)
        {
            b->t1 = t1b;
            b->t3 = t3b;
        }
        else
        {
            // both solutions contain negative time,
            // again perhaps some form of error logging
            b->t1 = INFINITY;
            b->t3 = INFINITY;
        }
    }

    // compute the velocity at the t1,t3 boundary
    float vm = b->a1 * b->t1 + b->Vinitial;
    b->Vmid  = vm;
    // if it is below the maximum we have our solution
    // so bail
    if (vm < b->MaxV && vm > -b->MaxV)
    {
        return;
    }

    // if we hit here then we are in the velocity limited case
    // so clamp velocity to the maximium
    b->Vmid = (vm < 0.0f) ? (-b->MaxV) : (b->MaxV);

    // here we recompute the times based on new maximum velocity
    // and if either time is negative the acceleration should be flipped
    // to regularize things. This is important because we could have an initial
    // velocity higher than Vmax and in which case we should decellerate to MaxV and
    // then decellerate again when we reach the destination which violated our previous
    // opposite sign assumption.
    b->t1 = (b->Vmid - b->Vinitial) / b->a1;
    if (b->t1 < 0.0f)
    {
        b->a1 *= -1.0f;
        b->t1 *= -1.0f;
    }
    b->t3 = -b->Vmid / b->a3;
    if (b->t3 < 0.0f)
    {
        b->a3 *= -1.0f;
        b->t3 *= -1.0f;
    }

    // apply the kinomatic formulas here again to figure out
    // how long we need to coast for in region 2
    float drem = b->Distance;
    // a*t^2/2 + vi*t = d   x 2
    drem -= b->a1 * b->t1 * b->t1 / 2.0f;
    drem -= b->a3 * b->t3 * b->t3 / 2.0f;
    drem -= b->Vmid * b->t3;
    drem -= b->Vinitial * b->t1;

    // compute t2 (this shouldn't be negative
    // but again if it is there is a need for error logging
    b->t2 = drem / b->Vmid;
    return;
}


/**
 * \ingroup Controls
 *
 * \brief plans out acceleration duration an directions for 1D bang bang control
 *
 * \param[in,out] bang bang trajectory
 */
void app_bangbang_planTrajectory(BBProfile *b)
{
    // This code goes through the cases where we can't accelerate positively initially
    // and in those cases pre-flips the signs of the inputs so that the planning code
    // does not have to deal with the different cases. The two cases are if the the
    // distance you are trying to travel is negative or if at your current speed you will
    // overshoot your destination even at maximum decelleration. If both cases are
    // true then the flips cancel out.

    // are we going leftwards
    bool Dflip = b->Distance < 0.0f;

    // are the target distance and current velocity in the same direction
    bool SameSign = !((b->Vinitial < 0.0f) ^ (b->Distance < 0.0f));

    // would we overshoot if D and Vi had the same sign
    bool Overshoot = (b->Vinitial * b->Vinitial > 2.0f * b->MaxA * fabsf(b->Distance));

    // will we overshoot
    bool Oflip = SameSign && Overshoot;

    // flip only if one of the two cases are applied
    bool flip = Dflip ^ Oflip;
    if (flip)
    {
        b->Distance *= -1.0f;
        b->Vinitial *= -1.0f;
    }
    // we do all this so that we can assume a1 is posive
    b->a1 = b->MaxA;
    b->a3 = -b->MaxA;
    BBPositivePlan(b);
    // undo the flip if it was applied
    if (flip)
    {
        b->Distance *= -1.0f;
        b->Vinitial *= -1.0f;
        b->a1 *= -1.0f;
        b->a3 *= -1.0f;
        b->Vmid *= -1.0f;
    }
}


/**
 * \ingroup Controls
 *
 *\brief forward simulates a BB trajectory and returns a future distance and velocity
 *
 * \param[in] planned BBtrajectory to simulate
 * \param[in] time in the future to simulate
 * \param[out] distance traveled
 * \param[out] velocity at that point
 **/
void app_bangbang_getState(const BBProfile *b, float time, float *d, float *v)
{
    *v         = b->Vinitial;
    *d         = 0.0f;
    float rem  = time;
    float step = timeLimit(&rem, b->t1);
    stepTime(d, v, step, b->a1);
    step = timeLimit(&rem, b->t2);
    stepTime(d, v, step, 0.0f);
    step = timeLimit(&rem, b->t3);
    stepTime(d, v, step, b->a3);
    stepTime(d, v, rem, 0.0f);
}


/**
 * \brief computes a constant jerk trajectory which connects two states
 *
 * \param[in] Vinit initial velocity
 * \param[in] Vfinal terminal velocity
 * \param[in] Distance, the total distance travelled during maneuver
 * \param[in] time over which the maneuver is performed
 * \param[out] Jerk the quantity of jerk to apply
 * \return the initial acceleration
 */
float app_bangbang_computeInitialAccelerationForConstantJerkProfile(
    float Vinit, float Vfinal, float Distance, float time, float *Jerk)
{
    // D = t^3*J/6 + t^2 * A / 2 + Vinit*t
    // Vf = t^2 * J / 2 + A*t + Vinit

    //(Vf - Vi)/t = t*J / 2 + A
    //(d - Vi*t)/t^2 = J*t/6 + A/2
    //
    // matrix
    // [ t/2  1   ] [ J ] = [ (Vf-Vi)/t      ]
    // [ t/6  1/2 ] [ A ] = [ (d - vi*t)/t^2 ]
    //
    // [ 6/t  -12/t ] [ (Vf - Vi)/t     ] = [ J ]
    // [ -2     6   ] [ (d - Vi*t)/t^2  ] = [ A ]
    //
    float dV = (Vfinal - Vinit) / time;
    float dD = (Distance - Vinit * time) / time / time;
    if (Jerk)
    {
        *Jerk = (6.0f * dV - 12.0f * dD) / time;
    }
    return (6.0f * dD - 2.0f * dV);
}

/**
 * \ingroup Controls
 *
 * \brief Implements a horizon for the acceleration computation
 *
 * Computes the acceleration that would get the robot into the desired
 * state at horizon time, if the robot was experiencing constant jerk.
 * This effectively becomes a jerk limiter instead of applying a raw acceleration.
 *
 * \param[in] bb controller profile
 * \param[in] time horizon in future
 * \return acceleration to apply
 */
float app_bangbang_computeAccel(const BBProfile *b, float horizon)
{
    float dist, vel;
    app_bangbang_getState(b, horizon, &dist, &vel);
    return app_bangbang_computeInitialAccelerationForConstantJerkProfile(
        b->Vinitial, vel, dist, horizon, 0);
}


/**
 * \ingroup Controls
 *
 * \brief Implements a horizon for the acceleration computation based on average accel
 *
 * takes a BB profile and a time into the future, computes the average acceleration to
 * reach the  * required velocity in the future and returns said velocity.
 *
 * \param[in] bb controller profile
 * \param[in] time horizon in future
 * \return acceleration to apply
 */
float app_bangbang_computeAvgAccel(const BBProfile *b, float horizon)
{
    float dist, vel;
    app_bangbang_getState(b, horizon, &dist, &vel);
    return (vel - b->Vinitial) / horizon;
}
