/* Produced by CVXGEN, 2018-01-06 14:20:50 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"

void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1);
  lhs[1] = -rhs[1]*(-1);
  lhs[2] = -rhs[2]*(-1);
  lhs[3] = -rhs[3]*(-1);
  lhs[4] = -rhs[0]*(1);
  lhs[5] = -rhs[1]*(1);
  lhs[6] = -rhs[2]*(1);
  lhs[7] = -rhs[3]*(1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1)-rhs[4]*(1);
  lhs[1] = -rhs[1]*(-1)-rhs[5]*(1);
  lhs[2] = -rhs[2]*(-1)-rhs[6]*(1);
  lhs[3] = -rhs[3]*(-1)-rhs[7]*(1);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.Q[0])+rhs[1]*(2*params.Q[4])+rhs[2]*(2*params.Q[8])+rhs[3]*(2*params.Q[12]);
  lhs[1] = rhs[0]*(2*params.Q[1])+rhs[1]*(2*params.Q[5])+rhs[2]*(2*params.Q[9])+rhs[3]*(2*params.Q[13]);
  lhs[2] = rhs[0]*(2*params.Q[2])+rhs[1]*(2*params.Q[6])+rhs[2]*(2*params.Q[10])+rhs[3]*(2*params.Q[14]);
  lhs[3] = rhs[0]*(2*params.Q[3])+rhs[1]*(2*params.Q[7])+rhs[2]*(2*params.Q[11])+rhs[3]*(2*params.Q[15]);
}
void fillq(void) {
  work.q[0] = params.c[0];
  work.q[1] = params.c[1];
  work.q[2] = params.c[2];
  work.q[3] = params.c[3];
}
void fillh(void) {
  work.h[0] = 1;
  work.h[1] = 1;
  work.h[2] = 1;
  work.h[3] = 1;
  work.h[4] = 1;
  work.h[5] = 1;
  work.h[6] = 1;
  work.h[7] = 1;
}
void fillb(void) {
}
void pre_ops(void) {
}
