#include "physics.h"
#include "encoder.h"
#include <math.h>
#include <stdint.h>

#ifndef FWSIM
#include "adc.h"
#else
#include <stdio.h>
#include <stdlib.h>
#endif


//Wheel angles for these matricies are (55, 135, 225, 305) degrees
//these matrices may be derived as per omnidrive_kiart paper


//This matrix is a unitless matrix which takes the force exerted
//on the floor by each of the four wheels and converts it into forces
//within the robot domain, the third component is is newtons and not 
//torque as the matrix is unitless (multiply by ROBOT_RADIUS to unnormalize)
//the transpose of this matrix is the velocity coupling matrix and can convert
//speeds in the robot coordinates into linear wheel speeds
static const float force4_to_force3_mat[3][4]=
{
		{-0.8192, -0.7071, 0.7071, 0.8192},
		{ 0.5736, -0.7071,-0.7071, 0.5736},
		{ 1.0000,  1.0000, 1.0000, 1.0000} 
};

// Transformation matricies to convert a 4 velocity to
// a 3 velocity (derived as pinv(force4_to_force3^t)
// this is also the transpose of force3_to_force4 mat
static const float speed4_to_speed3_mat[3][4]=
{
	{-0.3498,-0.3019, 0.3019, 0.3498},
	{ 0.3904,-0.3904,-0.3904, 0.3904},
	{ 0.2761, 0.2239, 0.2239, 0.2761}
};

//mass vector (consists linear robot mass and interial mass)
const float ROBOT_MASS[3] = {ROBOT_POINT_MASS, ROBOT_POINT_MASS, ROT_MASS};

const float MAX_VEL[3] = {MAX_X_V, MAX_Y_V, MAX_T_V*ROBOT_RADIUS};
const float MAX_ACC[3] = {MAX_X_A, MAX_Y_A, MAX_T_A*ROBOT_RADIUS};

//contention vector, where in forces will simply consume power.
static const float contention_vector[4] = {-0.4621, 0.5353, -0.5353, 0.4621};

float norm2(float a1, float a2){
	return(sqrtf(a1*a1 + a2*a2) );
}



// return the minimum angle from angle1 to angle2
// test with angle2 increased or decreased by 2pi
float min_angle_delta(float angle1, float angle2){
	angle1 = fmod(angle1, 2*P_PI);
	angle2 = fmod(angle2, 2*P_PI);
	if(angle2 >= angle1 ){                                                                                                        
		float ang_sub = angle2 - 2*P_PI;                                                                                        
		if((ang_sub - angle1)*(ang_sub - angle1) <= (angle2 - angle1)*(angle2 - angle1)){                             
			return(ang_sub - angle1);                                                              
		}else{                                                                                                                 
			return(angle2 - angle1);                                                                                 
		}                                                                                                                      
	}else{                                                                                                                       
		float ang_plus = angle2 + 2*P_PI;                                                                                       
		if((ang_plus - angle1)*(ang_plus - angle1) <= (angle2 - angle1)*(angle2 - angle1)){                           
			return(ang_plus - angle1);                                                              
		}else{                                                                                                                 
			return(angle2 - angle1);                                                                                 
		}                                                                                                                      
	}                                                                                                                       

}

/**
 * \brief Multiplies a matrix by a vector.
 *
 * \param[out] lhs the result, \p matrix * \p rhs
 * \param[in] lhs_len the size of the output vector
 * \param[in] rhs the vector to multiply
 * \param[in] rhs_len the size of the input vector
 * \param[in] matrix the matrix to multiply
 */
void matrix_mult(float* lhs, int lhs_len, const float* rhs,int rhs_len, const float matrix[lhs_len][rhs_len]) {
	for(int j=0;j<lhs_len;++j) {
		lhs[j]=0.0f;
		for(int i=0;i<rhs_len;++i) {
			lhs[j] += matrix[j][i]*rhs[i];
		}
	}
}

/**
 * \brief Multiplies a matrix's transpose by a vector.
 *
 * \param[out] lhs the result, \p matrixT * \p rhs
 * \param[in] lhs_len the size of the output vector
 * \param[in] rhs the vector to multiply
 * \param[in] rhs_len the size of the input vector
 * \param[in] matrix the matrix to multiply
 */
void matrix_mult_t(float* lhs, int lhs_len, const float* rhs, int rhs_len, const float matrix[rhs_len][lhs_len]) {
	for(int j=0;j<lhs_len;++j) {
		lhs[j]=0.0f;
		for(int i=0;i<rhs_len;++i) {
			lhs[j] += matrix[i][j]*rhs[i];
		}
	}
}

/**
 * \brief Multiplies two matrices.
 *
 * \param[out] matrix_out the result 
 * \param[in] lm_rows number of rows of left matrix
 * \param[in] lm_cols number of cols of right matrix
 * \param[in] rm_rows number of rows of right matrix
 * \param[in] rm_cols number of cols of right matrix
 * \param[in] lmatrix left matrix to multiply
 * \param[in] rmatrix right matrix to multiply 
 */
void mm_mult(int lm_rows, int rm_rows, int rm_cols, const float lmatrix[lm_rows][rm_rows], const float rmatrix[rm_rows][rm_cols], float matrix_out[lm_rows][rm_cols]) {
  int i;
  int j;
  int k;
  // NOTE rm_rows and lm_cols must be equal!
  float temp;
  for(i = 0; i < rm_cols; i++) {
    for(j = 0; j < lm_rows; j++) {
      temp = 0;
      for(k = 0; k < rm_rows; k++) {
        temp += lmatrix[j][k]*rmatrix[k][i];
      }
      matrix_out[j][i] = temp;     
    }
  }
}

/**
 * \brief Multiplies one matrix by another's transpose.
 *
 * \param[out] matrix_out the result 
 * \param[in] lm_rows number of rows of left matrix
 * \param[in] lm_cols number of cols of left matrix
 * \param[in] rm_rows number of rows of right matrix
 * \param[in] rm_cols number of cols of right matrix
 * \param[in] lmatrix left matrix to multiply
 * \param[in] rmatrix right matrix to transpose multiply 
 */
void mm_mult_t(int lm_rows, int rm_rows, int rm_cols, const float lmatrix[lm_rows][rm_cols], const float rmatrix[rm_rows][rm_cols], float matrix_out[lm_rows][rm_rows]) {
  int i;
  int j;
  int k;
  // NOTE rm_rows and lm_cols must be equal!
  float temp;
  for(i = 0; i < rm_rows; i++) {
    for(j = 0; j < lm_rows; j++) {
      temp = 0;
      for(k = 0; k < rm_cols; k++) {
        temp += lmatrix[j][k]*rmatrix[i][k];
      }
      matrix_out[j][i] = temp;     
    }
  }
}

/**
 * \ingroup Physics
 *
 * \brief copies source B into destination A. 
 *
 * \param[in] nrows number of rows
 * \param[in] ncols number of cols
 * \param[in, out] A destination matrix 
 * \param[in] B source matrix 
 *
 */
void mm_copy(int nrows, int ncols, float A[nrows][ncols], float B[nrows][ncols]) {
  int i;
  int j;
  for(i = 0; i < nrows; i++) {
    for(j = 0; j < ncols; j++){
      A[i][j] = B[i][j];
    } 
  }
}


/**
 * \brief Adds matrix B to A, and stores in A. 
 *
 * \param[in] nrows number of rows
 * \param[in] ncols number of cols
 * \param[in, out] a destination matrix 
 * \param[in] b source matrix 
 */

void mm_add(int nrows, int ncols, float a[nrows][ncols], const float b[nrows][ncols]) {
  int i, j;
  for (i = 0; i < nrows; i++) {
    for (j = 0; j < ncols; j++) {
      a[i][j] = a[i][j] + b[i][j];
    }
  }
}
/**
 * \brief Subtracts matrix B from A, and stores in C. 
 *
 * \param[out] c the result 
 * \param[in] nrows number of rows
 * \param[in] ncols number of cols
 * \param[in] a source matrix 1
 * \param[in] b source matrix 2 
 */

void mm_sub(int nrows, int ncols, const float a[nrows][ncols], const float b[nrows][ncols], float c[nrows][ncols]) {
  int i, j;
  for (i = 0; i < nrows; i++) {
    for (j = 0; j < ncols; j++) {
      c[i][j] = a[i][j] - b[i][j];
    }
  }
}

/**
 * \brief Inverts a 2x2 or a 1x1 matrix. 
 *
 * \param[in, out] a the result 
 * \param[in] n number of rows and columns
 */

void mm_inv(int n, float a[n][n]) {
  float det, temp;
  float temp_m[n][n];
  switch(n) {
    case (1):
      a[0][0] = 1 / a[0][0];
      break;
    case (2):
      det = (a[0][0]*a[1][1] - a[0][1]*a[1][0]);
      temp = a[0][0];
      a[0][0] = a[1][1] / det;
      a[1][1] = temp / det;
      a[0][1] = -a[0][1] / det;
      a[1][0] = -a[1][0] / det;
      break;
    case (3):
		  for(int i=0;i<3;i++)
		      det = det + (a[0][i]*(a[1][(i+1)%3]*a[2][(i+2)%3] - a[1][(i+2)%3]*a[2][(i+1)%3]));
		   for(int i=0;i<3;i++){
		      for(int j=0;j<3;j++)
		           temp_m[i][j] = ((a[(i+1)%3][(j+1)%3] * a[(i+2)%3][(j+2)%3]) - (a[(i+1)%3][(j+2)%3]*a[(i+2)%3][(j+1)%3]))/ det;
		   }
		   for(int i=0;i<3;i++){
		      for(int j=0;j<3;j++)
		           a[i][j] = temp_m[i][j];
		   }
		   break;
    default:
	  break;
	}

  }


/**
 * \ingroup Physics
 *
 * \brief performs the unitless conversion of wheel speeds into robot speeds
 * 
 * \param[in] the 4 wheel speeds
 * \param[out] the 3 robot speeds in the same units
 */
void speed4_to_speed3(const float speed4[4], float speed3[3]) {
	matrix_mult(speed3,3,speed4,4, speed4_to_speed3_mat);
}

#ifdef FWSIM
void force4_to_force3(const float force4[4], float force3[3]) {
	matrix_mult(force3,3,force4,4, force4_to_force3_mat);
}
#endif

/**
 * \ingroup Physics
 *
 * \brief performs the unitless conversion of the robots speeds into wheel speeds
 * 
 * \param[in] the robot speeds in x,y,theta*R coordinates
 * \param[out] the robot wheel speeds in the same units as input
 */
void speed3_to_speed4(const float speed3[3], float speed4[4]) {
	matrix_mult_t(speed4, 4, speed3, 3, force4_to_force3_mat);
}


/**
 * \ingroup Physics
 *
 * \brief implements the vector transfrom of A=A-B
 *
 * \param[in,out] the A vector
 * \param[in] the B vector
 * \param[in] the length of the vectors
 *
 */
void vectorSub(float *a,const float *b, int len) {
	for(int i=0;i<len;i++) {
		a[i] = a[i]-b[i];
	}
}

/**
 * \ingroup Physics
 *
 * \brief implements the vector transform of A=A+B
 *
 * \param[in,out] the A vector
 * \param[in] the B vector
 * \param[in] the length of the vectors
 *
 */
void vectorAdd(float *a,const float *b, int len) {
	for(int i = 0; i < len; i++) {
		a[i] = a[i] + b[i];
	}
}

/**
 * \ingroup Physics
 *
 * \brief copies source b into destination a. 
 *
 * \param[in,out] the destination vector
 * \param[in] the source vector
 * \param[in] the length of the vectors
 *
 */
void vectorCopy(float *a, const float *b, int len) {
  int i;
  for(i = 0; i < len; i++) {
    a[i] = b[i]; 
  }
}

/**
 * \ingroup Physics
 *
 * \brief perfoms 2D cartesian to polar coords
 *
 * \param[in,out] input vector in cartesian coordinates to replace with polar
 *
 */
void Cart2Pol(float vec[2]) {
	float temp = sqrtf(vec[0]*vec[0] + vec[1]*vec[1]);
	vec[1] = atan2f(vec[1], vec[0]);
	vec[0] = temp;
}

/**
 * \ingroup Physics
 *
 * \brief perfoms 2D polar to cartesian
 *
 * \param[in,out] input vector in polar to replace with cartesian
 *
 */
void Pol2Cart(float vec[2]) {
	float temp = vec[0]*cosf(vec[1]);
	vec[1] = vec[0]*sinf(vec[1]);
	vec[0] = temp;
}

/**
 * \ingroup Physics
 *
 * \brief converts a 2D cartesian velocity into a polar velocity given a polar location
 *
 * \param[in] location of velocity in polar coordinates
 * \param[in,out] velocity in cartesian coordinates to replace with polar
 */
void CartVel2Pol(float const loc[2], float vel[2]) {
	float temp = cosf(loc[1])*vel[0] + sinf(loc[1])*vel[1];
	vel[1] = -sinf(loc[1])/loc[0]*vel[0] + cosf(loc[1])/loc[0]*vel[1];
	vel[0] = temp;
}

/**
 * \ingroup Physics
 *
 * \brief converts a 2D polar velocity into a cartesian velocity given a polar location
 *
 * \param[in] location of velocity in polar coordinates
 * \param[in,out] velocity in polar coordinates to replace with cartesian
 */
void PolVel2Cart(float const loc[2], float vel[2]) {
	float temp = cosf(loc[1])*vel[0] - loc[0]*sinf(loc[1])*vel[1];
	vel[1] = sinf(loc[1])*vel[0] + loc[0]*cosf(loc[1])*vel[1];
	vel[0] = temp;
}

/**
 * \ingroup Physics
 *
 * \brief converts a 2D polar acceleration into linear acceleration
 *
 * \param[in] Polar Position
 * \param[in] Polar Velocity
 * \param[in] Polar Acceleration
 * \param[out] acceleration in cartesian coords
 */
void PolAcc2Cart(float const loc[2], float const vel[2], float const Pacc[2], float Cacc[2]) {
	float cosT = cosf(loc[1]);
	float sinT = sinf(loc[1]);
	Cacc[0] = cosT*Pacc[0] - loc[0]*sinT*Pacc[1] + vel[1] * (-2.0f*sinT*vel[0] - loc[0]*cosT*vel[1]);
	Cacc[1] = sinT*Pacc[0] + loc[0]*cosT*Pacc[1] + vel[1] * ( 2.0f*cosT*vel[0] - loc[1]*sinT*vel[1]);
}



/**
 * \ingroup Physics
 *
 * \brief implements 2D rotation matrix
 *
 * \param[in,out] the speed to rotate
 * \param[in] amount in radian to rotate
 */
void rotate(float speed[2], float angle) {
	float temp = cosf(angle)*speed[0] - sinf(angle)*speed[1];
	speed[1] = sinf(angle)*speed[0] + cosf(angle)*speed[1];
	speed[0]=temp;
}

/**
 * \ingroup Physics
 *
 * Implements the conversion between forces in the robot coordinate system and the 
 * Force per wheel. This is nominally equal to the speed3_to_speed4 conversion if
 * the center of mass of the robot coincides with the wheel center, however this is 
 * not the case and so when computing wheel forces this should be transform should
 * be used.
 *
 * \brief Implements the conversion from force in robot coordinates to Wheel force
 * 
 * \param[in] force in robot coordinates
 * \param[out] force to exert per wheel 
 */
void force3_to_force4(float force3[3], float force4[4]) {
	matrix_mult_t(force4, 4, force3, 3, speed4_to_speed3_mat);
}

// need the ifndef here so that we can ignore this code when compiling
// the firmware tests
#ifndef FWTEST
/**
 * \ingroup Physics
 *
 * \brief compute the scaling constant to bring motor torques to maximum 
 *
 * \param[in] attempted torque to exert per wheel
 *
 * \return the amount by which to scale the torque vector to max it out
 */
#ifndef FWSIM
float get_maximal_torque_scaling(const float torque[4]) {
	float acc_max = -INFINITY;
	float vapp_max = -INFINITY;
	for(int i=0;i<4;i++) {
		float volt = torque[i]*CURRENT_PER_TORQUE*PHASE_RESISTANCE;
		float back_emf = (float)encoder_speed(i)*QUARTERDEGREE_TO_VOLT;
		float appl_volt = fabsf(volt+back_emf);
		float max_app = fabsf(volt);
		if(max_app > acc_max) {
			acc_max = max_app;
		}
		if(appl_volt > vapp_max) {
			vapp_max = appl_volt;
		}
	}
	
	float slip_ratio = DELTA_VOLTAGE_LIMIT / acc_max;
	float emf_ratio = adc_battery() / vapp_max;

	return (emf_ratio > slip_ratio)?slip_ratio:emf_ratio; 
}
#endif


/**
 * \ingroup Physics
 *
 * \brief compute the scaling constant to bring robot acceleration to maximum
 *
 * \param[in] attempted linear acceleration in robot coordinates
 * \param[in] attempted angular acceleration
 *
 * \return amount by which to scale acceleration
 */
#ifndef FWSIM
float get_maximal_accel_scaling(const float linear_accel[2], float angular_accel) {
	//first convert accelerations into consistent units
	//choose units of Force (N)
	float normed_force[3];
	normed_force[0]=linear_accel[0]*ROBOT_MASS[0];
	normed_force[1]=linear_accel[1]*ROBOT_MASS[1];
	normed_force[2]=angular_accel*ROBOT_MASS[2]*ROBOT_RADIUS;

	float wheel_force[4];
	force3_to_force4(normed_force, wheel_force);
	for(int i=0;i<4;++i) {
		wheel_force[i] *= WHEEL_RADIUS*GEAR_RATIO; //convert to motor torque
	}
	return get_maximal_torque_scaling(wheel_force);
}
#endif

#endif

/**
 * \ingroup Physics
 *
 * \brief decompose desired final velocity into x and y components. Calculated
 *        using the robot's initial coordinate system when the primitive began.
 *
 * \param[in]  final speed magnitude
 * \param[out] array of x and y components of final velocity
 * \param[in]  array of x and y coordinates of initial position
 * \param[in]  array of x and y coordinates of final position
 *
 */

void decompose_radial(const float speed, float* vf, const float* init_pos,
	const float* final_pos)
{
	float angle = atan2f(final_pos[1] - init_pos[1], final_pos[0] - init_pos[0]);

	vf[0] = cosf(angle) * speed;
	vf[1] = sinf(angle) * speed;
}

/**
 * Use for dot product on arbitrarily large arrays.
 *
 * @param vec1 the first vector in the dot product
 * @param vec2 the second vector in the dot product
 * @param size the size of the vectors. They should be the same size
 * @return the dot product result of the vectors
 */
float dot_product(float vec1[], float vec2[], int size) {
    float result = 0;
    for (int i = 0; i < size; i++) {
        result += (vec1[i] * vec2[i]);
    }
    return result;
}

/**
 * Dot product for 2D vectors.
 * 
 * @param vec1 the first vector in the dot product
 * @param vec2 the second vector in the dot product
 * @return the dot product result of the vectors
 */
float dot2D(float vec1[2], float vec2[2]) {
    return vec1[0] * vec2[0] + vec1[1] * vec2[1];
}

/**
 * Dot product for 3D vectors.
 *
 * @param vec1 the first vector in the dot product
 * @param vec2 the second vector in the dot product
 * @return the dot product result of the vectors
 */
float dot3D(float vec1[3], float vec2[3]) {
    return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
}
