#ifndef QUADRATIC_H
#define QUADRATIC_H

#include "physbot.h"
#include "../dr.h"

/**
 * Builds the M matrix for the optimization. This matrix has one 
 * row for each acceleration (major, minor, rotational) and one 
 * column for each wheel on the bot.
 * 
 * @param pb a PhysBot that should be setup by the setup_bot function
 * in physbot.c
 * @param state The current state of the robot that should have the
 * @param M a 3 x 4 matrix where each column corresponds to a wheel
 * @return void
 * \
 */ 
void build_M_matrix(PhysBot pb, dr_data_t state, float M[3][4]);

/**
 * Takes the Q matrix and converts it to a 16 length matrix for the 
 * CVXGEN solver.
 * 
 * @param Q A 4 x 4 matrix that is the result of multiplying M.T * M
 * @return void
 */
void to_1d_matrix(float Q[4][4]);

/**
 * Creates the c matrix for the optimization.
 * 
 * @param a_req The requested accelerations supplied by the primitive.
 * @param M a 3 x 4 matrix where each column corresponds to a wheel
 * @param c The c matrix that is being created for the optimization 
 * @return void
 *
 * c = 2 * a_req.T * M
 */
void build_c_matrix(float a_req[3], float M[3][4], float c[4]);

/**
 * Transposes the M matrix.
 * 
 * @param M a 3 x 4 matrix where each column corresponds to a wheel
 * @param M_T the transpose of the M matrix
 * @return void
 */ 
void transpose_qp(float M[3][4], float M_T[4][3]);

/**
 * Multiplies the M.T * M to get the Q matrix for the optimization.
 * 
 * @param M a 3 x 4 matrix where each column corresponds to a wheel
 * @param M_T the transpose of the M matrix
 * @param Q A 4 x 4 matrix that is the result of multiplying M.T * M
 * @return void
 */ 
void build_Q_matrix(float M[3][4], float M_T[4][3], float Q[4][4]);

/**
 * A primitive should call this function to optimize acclerations
 * in the major, minor, and rotational directions. This function
 * sets up the matrices required for the CVXGEN solver by using
 * the helper functions defined in this file then calls the solve()
 * method in CVXGEN to run the optimization.
 * 
 * @param pb a PhysBot that was set up by that setup_bot function 
 * in physbot.c
 * @param state a dr_data_t that contains the bot's current angle, 
 * @param a_req a 3 length array with requested accelerations for major, minor,
 * and rotational accelerations
 * @return void
 * 
 */
void quad_optimize(PhysBot pb, dr_data_t state, float a_req[3]);

#endif