#include <Eigen/Dense>
MAKE_ENUM(StateIndex, X_POSITION, Y_POSITION, ORIENTATION, X_VELOCITY, Y_VELOCITY,
          ANGULAR_VELOCITY);

MAKE_ENUM(MeasurementIndex, VISION_X_POSITION, VISION_Y_POSITION, VISION_ORIENTATION,
          MOTOR_X_VELOCITY, MOTOR_Y_VELOCITY, MOTOR_ANGULAR_VELOCITY,
          IMU_ANGULAR_VELOCITY);

MAKE_ENUM(ControlIndex, X_VELOCITY_TARGET, Y_VELOCITY_TARGET);

MAKE_ENUM(FilterStepType, PREDICT, MOTOR_DATA, IMU_DATA, VISION_DATA);


static constexpr size_t STATE_SIZE       = reflective_enum::size<StateIndex>();
static constexpr size_t MEASUREMENT_SIZE = reflective_enum::size<MeasurementIndex>();
static constexpr size_t CONTROL_SIZE     = reflective_enum::size<ControlIndex>();

// Initial Covariances
static constexpr double VISION_X_INITIAL_VARIANCE_M = 0.00001;
static constexpr double VISION_Y_INITIAL_VARIANCE_M = 0.00001;
static constexpr double VISION_THETA_INITIAL_VARIANCE_RAD = 0.00001;

static constexpr double MOTOR_THETA_INITIAL_VARIANCE_M_S = 0.5;
static constexpr double MOTOR_THETA_INITIAL_VARIANCE_M_S = 0.5;
static constexpr double MOTOR_THETA_INITIAL_VARIANCE_M_RAD = 0.5;

static constexpr double PROCESS_MODEL_INITIAL_VARIANCE = 0.5;


// Measurement models
static constexpr Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> VISION_MEASUREMENT_MODEL << 
1,0,0,0,0,0,0,
0,1,0,0,0,0,0,
0,0,1,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,0,0,0,0;

static constexpr Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> MOTOR_MEASUREMENT_MODEL << 
0,0,0,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,1,0,0,0,
0,0,0,0,1,0,0,
0,0,0,0,0,1,0;

static constexpr Eigen::Matrix<double, MEASUREMENT_SIZE, MEASUREMENT_SIZE> IMU_MEASUREMENT_MODEL<< 
0,0,0,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,0,0,0,1;
