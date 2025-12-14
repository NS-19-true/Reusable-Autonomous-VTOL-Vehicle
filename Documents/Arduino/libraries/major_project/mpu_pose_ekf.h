/*
  IMU Pose + Position EKF glue
  Orchestrates sensor setup and data fusion:
  - Reads IMU (MPU9250), compensates biases, and runs AHRS_EKF to estimate quaternion `q`.
  - Reads GPS (u-blox NAV-PVT) and BMP280, converts LLA to NED, and runs POS_EKF to estimate
    acceleration, velocity, and position `x` in NED.
  Exposed functions:
  - `imu_setup()` / `pose_setup()` for initialization, `imu_read()` for IMU sampling,
    `pose_update()` to run one fusion step, with outputs `q` and `x`.
*/
#include "ahrs_ekf.h"
#include "pos_ekf.h"
#include "MPU9250.h"
#include "gps.h"
#include "bmp.h"


#define GRAVITY 9.802 // The gravity acceleration in New York City

// Calibration outcomes
/*
#define GYRO_X_OFFSET 0.0000702
#define GYRO_Y_OFFSET -0.0001424
#define GYRO_Z_OFFSET -0.0003332

#define ACCEL_X_OFFSET -1.4086211
#define ACCEL_Y_OFFSET -8.8509712
#define ACCEL_Z_OFFSET -9.3008728
*/

const float magn_ellipsoid_center[3] = {33.0393, 27.8563, -6.24899};
const float magn_ellipsoid_transform[3][3] = {{0.843576, -0.0375454, -0.0320949}, {-0.0375454, 0.838998, 0.0437144}, {-0.0320949, 0.0437144, 0.97602}};


const double g[3] = {0, 0, -1};
// magnetic inclination angle symbol = delta, magnetic declination symbol = gamma
const double theta = 56.5919*PI/180.0;
const double beta = -4.1934*PI/180.0;
const double r[3] = {cos(theta) * cos(beta), cos(theta) * sin(beta), sin(theta)};


const float Ts = 0.004;
const double P_ahrs[4] = {0.0,0.0,0.0,0.0}; //uncertainty of quaternion
//const double P[4] = {0.1,0.1,0.1,0.1}; //uncertainty of quaternion
//const float q[4] = {1,0,0,0}; //initial quaternion

// Covariance matrices
const double Q_ahrs[3] = {0.0000006459498903751044, 0.0000006459498903751044, 0.0000006459498903751044};  // covariance for gyro
//const double Q[3] = {0.00000006459498903751044, 0.00000006459498903751044, 0.00000006459498903751044};  // covariance for gyro

//const double R[6] = {0.0000011620665443494157, 0.0000011620665443494157, 0.0000011620665443494157, 0.00017709437975418487, 0.00017709437975418487, 0.00017709437975418487};  // covariance for accel and mag
const double R_ahrs[6] = {0.000011620665443494157, 0.000011620665443494157, 0.000011620665443494157, 0.00517709437975418487, 0.00517709437975418487, 0.00517709437975418487};  // covariance for accel and mag

//extern double accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
//extern double magnetom[3];
//extern double gyro[3];

const double P_pos[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; //uncertainty of position
const double Q_pos[7] = {0.0, 0.0, 0.0, 0.0, R_ahrs[0] * GRAVITY , R_ahrs[1] * GRAVITY , R_ahrs[2] * GRAVITY };  // initial covariance for quat and covariance for accel input
//const double Q_pos[7] = {0.0, 0.0, 0.0, 0.0, R_ahrs[0] * GRAVITY *10, R_ahrs[1] * GRAVITY *10, R_ahrs[2] * GRAVITY *10};
const double quat_P_sf = GRAVITY;  // quaternion covariance from ahrs to pos scale factor

//const double R_pos[6] = {0.0003455960, 0.000571210, 4.9284, 1.384622890, 1.253504160, 0.043056250};   // from uNavIns data
const double R_pos[6] = {0.0223, 0.0365, 3.3640, 1.3846, 1.2535, 0.0346};       // from love garden data

extern double* q;
extern double* x;

void imu_read();
void compensate_imu_errors();
void Matrix_Vector_Multiply(const float a[3][3], const double b[3], double out[3]);
void imu_setup();
void calibrate_imu_bias();
void pose_setup();
void pose_update();