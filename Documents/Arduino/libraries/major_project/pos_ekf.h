/*
  POS_EKF: Position/Velocity/Acceleration EKF in NED frame
  9-state model x = [aN, aE, aD, vN, vE, vD, pN, pE, pD].
  Inputs u = [q0, q1, q2, q3, aX, aY, aZ] (quaternion and body accel),
  Measurements z = [vN, vE, vD, lat(rad), lon(rad), alt(m)] converted to NED.
  - `init(...)` sets input/measurement covariances (Q, R), gravity, reference LLA, timestep, P, x.
  - `update(u, Q_q, iTOW, z)` predicts every cycle; corrects only when GPS iTOW advances.
*/
#include "eigen.h"
#include <Eigen/Dense>

extern Eigen::Vector<double, 3> z_ned;

Eigen::Vector<double, 3> lla2ecef(Eigen::Vector<double, 3> &lla);
Eigen::Vector<double, 3> lla2ned(Eigen::Vector<double, 3> &ref_lla, Eigen::Vector<double, 3> &lla);

class POS_EKF
{
public:
    POS_EKF();
    ~POS_EKF();

    void init(const double (&Q_in)[7], const double (&R_in)[6], const double (&g_in)[3], const double (&pos_ref_lla_in)[3], const float &Ts, const double (&P_in)[9], const double (&x_in)[9]);
    
    double* update(double (&u_in)[7], Eigen::Matrix<double, 4, 4> &Q_q, unsigned long &iTOW, double (&z_in)[6]);      // Q_q is the covariance matrix of quaternion input
    

private:
    Eigen::Vector<double, 3> pos_ref_lla; // reference position in lla
    unsigned long iTOW_prev = 0;

    Eigen::Matrix<double, 9, 9> P; // state covariance matrix at time t-1
    Eigen::Matrix<double, 7, 7> Q;  // input noise covariance <quat, accel(m/s2)>
    Eigen::Matrix<double, 6, 6> R;  // gps, baro noise covariance <vel(m/s), pos(m)>

    Eigen::Vector<double, 3> g; // gravity vector in ned frame <0, 0, g(m/s2)>

    float _t; // time interval in seconds

    Eigen::Vector<double, 9> x; // state vector at time t-1


    void f_func(Eigen::Vector<double, 7> &u);
    Eigen::Matrix<double, 9, 9> F;
    Eigen::Matrix<double, 6, 9> H;
    Eigen::Matrix<double, 9, 7> W_func(Eigen::Vector<double, 7> &u);
};