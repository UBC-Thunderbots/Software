#ifdef FWSIM
#include "simulate.h"
#include "physics.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


const float MU = 0.6;
const float SLIP_FORCE = 0.8 * /* MU */ ROBOT_POINT_MASS * 9.8 * 0.25 /*weight distribution (not entirely accurate)*/;
static float pos[3] = {0};
static float vel[3] = {0};
static float accel[3];
static float force3[3];
static float force4[4];
static bool slip[4];

static FILE* logFile;

void dr_get(dr_data_t * ret){
	ret->x = pos[0];
	ret->y = pos[1];
	ret->angle = pos[2];
	
	ret->vx = vel[0];
	ret->vy = vel[1];
	ret->avel = vel[2];
}

void sim_apply_wheel_force(const float new_wheel_force[4]){
    // printf("\nWF0:%f, WF2:%f, WF2:%f,
    // WF3:%f",new_wheel_force[0],new_wheel_force[1],new_wheel_force[2],new_wheel_force[3]);
    unsigned int i;
    for (i = 0; i < 4; i++)
    {
        if (new_wheel_force[i] > SLIP_FORCE)
        {
            // printf("\nSLIPPING, limit = %f, force = %f\n",SLIP_FORCE,
            // new_wheel_force[i]);
            force4[i] = SLIP_FORCE;  //* 0.2; TODO: uncomment this
            slip[i]   = true;
        }
        else if (new_wheel_force[i] < -SLIP_FORCE)
        {
            // printf("\nSLIPPING, limit = %f, force = %f\n",SLIP_FORCE,
            // new_wheel_force[i]);
            force4[i] = -SLIP_FORCE;  //* 0.2; TODO: uncomment this
            slip[i]   = true;
        }
        else
        {
            force4[i] = new_wheel_force[i];
            slip[i]   = false;
        }
    }

    // printf("\nWF0:%f, WF2:%f, WF2:%f,
    // WF3:%f",force4[0],force4[1],force4[2],force4[3]);
    force4_to_force3(force4, force3);
    // printf("\nFX:%f, FY:%f, FA:%f",force3[0],force3[1],force3[2]);

    float locaccel[3];
    locaccel[0] = force3[0] / ROBOT_POINT_MASS;
    locaccel[1] = force3[1] / ROBOT_POINT_MASS;
    locaccel[2] = force3[2] * ROBOT_RADIUS / INERTIA;

    // printf("sim local y accel: %f", locaccel[1]);

    rotate(locaccel, pos[2]);  // put it back in global coords
    accel[0] = locaccel[0];
    accel[1] = locaccel[1];
    accel[2] = locaccel[2];
    // fprintf(logFile,"sim global y accel: %f",accel[1]);
}
void sim_tick(float delta_t){
    pos[0] += vel[0] * delta_t;
    pos[1] += vel[1] * delta_t;
    pos[2] += vel[2] * delta_t;

    vel[0] += accel[0] * delta_t - 1.0 * vel[0] * delta_t;  // complete guess
    vel[1] += accel[1] * delta_t - 1.0 * vel[1] * delta_t;  // complete guess
    vel[2] += accel[2] * delta_t - 1.0 * vel[2] * delta_t;  // complete guess
}

void sim_log_tick(float time){
    fprintf(logFile, "SIM, ");
    fprintf(logFile, "%f, ", time);
    fprintf(logFile, "%f, ", pos[0]);
    fprintf(logFile, "%f, ", pos[1]);
    fprintf(logFile, "%f, ", pos[2]);
    fprintf(logFile, "%f, ", vel[0]);
    fprintf(logFile, "%f, ", vel[1]);
    fprintf(logFile, "%f, ", vel[2]);
    fprintf(logFile, "%f, ", accel[0]);
    fprintf(logFile, "%f, ", accel[1]);
    fprintf(logFile, "%f\n", accel[2]);
}
void sim_log_start(char* fileName)
{
    logFile = fopen(fileName, "w");
    fprintf(logFile, "TYPE, ");
    fprintf(logFile, "TIME, ");
    fprintf(logFile, "X, ");
    fprintf(logFile, "Y, ");
    fprintf(logFile, "THETA, ");
    fprintf(logFile, "VX, ");
    fprintf(logFile, "VY, ");
    fprintf(logFile, "VA, ");
    fprintf(logFile, "AX, ");
    fprintf(logFile, "AY, ");
    fprintf(logFile, "AA\n");
}
void sim_log_end()
{
    fclose(logFile);
}

void sim_reset(){
    pos[0] = 0.0;
    pos[1] = 0.0;
    pos[2] = 0.0;
    for (unsigned i = 0; i < 3; i++)
    {
        vel[i]    = 0.0;
        force3[i] = 0.0;
        force4[i] = 0.0;
        slip[i]   = 0.0;
        accel[i]  = 0.0;
    }
    force4[3] = 0.0;
    slip[3]   = 0.0;
}

float get_pos_x() {
	return pos[0];
}

#endif // FWSIm
