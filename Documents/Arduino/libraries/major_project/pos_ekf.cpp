/*
  POS_EKF implementation
  - Provides LLA→ECEF and LLA→NED helpers.
  - `init(...)` builds Q/R/P/F/H and seeds `x` and reference LLA.
  - `update(u, Q_q, iTOW, z)` replaces quaternion covariance in Q with input `Q_q`,
    predicts continuous-time dynamics, and when GPS iTOW increments:
      * converts [lat, lon, alt] to NED relative to `pos_ref_lla`
      * runs correction to update x and P
*/
#include "pos_ekf.h"
#include "Arduino.h"

Eigen::Vector<double, 3> z_ned;

Eigen::Vector<double, 3> lla2ecef(Eigen::Vector<double, 3> &lla)
{
    const double a = 6378137.0;     // equatorial radius of earth
    const double e2 = 0.00669437999014; // square of first eccentricity
    
    Eigen::Vector<double, 3> ecef;

    double N = a / sqrt(1 - e2 * sin(lla[0]) * sin(lla[0]));

    ecef[0] = (N + lla[2]) * cos(lla[0]) * cos(lla[1]);
    ecef[1] = (N + lla[2]) * cos(lla[0]) * sin(lla[1]);
    ecef[2] = (N * (1 - e2) + lla[2]) * sin(lla[0]);

    return ecef;
}

Eigen::Vector<double, 3> lla2ned(Eigen::Vector<double, 3> &ref_lla, Eigen::Vector<double, 3> &lla)
{
    Eigen::Vector<double, 3> ref_ecef = lla2ecef(ref_lla);
    Eigen::Vector<double, 3> ecef = lla2ecef(lla);

    double dX = ecef[0] - ref_ecef[0];
    double dY = ecef[1] - ref_ecef[1];
    double dZ = ecef[2] - ref_ecef[2];

    Eigen::Vector<double, 3> ned;

    ned[0] = -sin(ref_lla[0]) * cos(ref_lla[1]) * dX - sin(ref_lla[0]) * sin(ref_lla[1]) * dY + cos(ref_lla[0]) * dZ;
    ned[1] = -sin(ref_lla[1]) * dX + cos(ref_lla[1]) * dY;
    ned[2] = -cos(ref_lla[0]) * cos(ref_lla[1]) * dX - cos(ref_lla[0]) * sin(ref_lla[1]) * dY - sin(ref_lla[0]) * dZ;

    return ned;
}

POS_EKF::POS_EKF()
{
}

POS_EKF::~POS_EKF()
{
}

void POS_EKF::init(const double (&Q_in)[7], const double (&R_in)[6], const double (&g_in)[3], const double (&pos_ref_lla_in)[3], const float &Ts, const double (&P_in)[9], const double (&x_in)[9])
{
    Q << Q_in[0], 0, 0, 0, 0, 0, 0,
        0, Q_in[1], 0, 0, 0, 0, 0,
        0, 0, Q_in[2], 0, 0, 0, 0,
        0, 0, 0, Q_in[3], 0, 0, 0,
        0, 0, 0, 0, Q_in[4], 0, 0,
        0, 0, 0, 0, 0, Q_in[5], 0,
        0, 0, 0, 0, 0, 0, Q_in[6];

    R << R_in[0], 0, 0, 0, 0, 0,
        0, R_in[1], 0, 0, 0, 0,
        0, 0, R_in[2], 0, 0, 0,
        0, 0, 0, R_in[3], 0, 0,
        0, 0, 0, 0, R_in[4], 0,
        0, 0, 0, 0, 0, R_in[5];

    g << g_in[0], g_in[1], g_in[2];

    pos_ref_lla << pos_ref_lla_in[0], pos_ref_lla_in[1], pos_ref_lla_in[2];

    _t = Ts;

    P << P_in[0], 0, 0, 0, 0, 0, 0, 0, 0,
        0, P_in[1], 0, 0, 0, 0, 0, 0, 0,
        0, 0, P_in[2], 0, 0, 0, 0, 0, 0,
        0, 0, 0, P_in[3], 0, 0, 0, 0, 0,
        0, 0, 0, 0, P_in[4], 0, 0, 0, 0,
        0, 0, 0, 0, 0, P_in[5], 0, 0, 0,
        0, 0, 0, 0, 0, 0, P_in[6], 0, 0,
        0, 0, 0, 0, 0, 0, 0, P_in[7], 0,
        0, 0, 0, 0, 0, 0, 0, 0, P_in[8];

    F << 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0,
        _t, 0, 0, 1, 0, 0, 0, 0, 0,
        0, _t, 0, 0, 1, 0, 0, 0, 0,
        0, 0, _t, 0, 0, 1, 0, 0, 0,
        0.5*_t*_t, 0, 0, _t, 0, 0, 1, 0, 0,
        0, 0.5*_t*_t, 0, 0, _t, 0, 0, 1, 0,
        0, 0, 0.5*_t*_t, 0, 0, _t, 0, 0, 1;

    H << 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;

    x << x_in[0], x_in[1], x_in[2], x_in[3], x_in[4], x_in[5], x_in[6], x_in[7], x_in[8];
}

void POS_EKF::f_func(Eigen::Vector<double, 7> &u)
{
    x[0] = u[4]*(1-2*u[2]*u[2]-2*u[3]*u[3]) + u[5]*(2*u[1]*u[2]-2*u[0]*u[3]) + u[6]*(2*u[1]*u[3]+2*u[0]*u[2])-g[0];
    x[1] = u[4]*(2*u[1]*u[2]+2*u[0]*u[3]) + u[5]*(1-2*u[1]*u[1]-2*u[3]*u[3]) + u[6]*(2*u[2]*u[3]-2*u[0]*u[1])-g[1];
    x[2] = u[4]*(2*u[1]*u[3]-2*u[0]*u[2]) + u[5]*(2*u[2]*u[3]+2*u[0]*u[1]) + u[6]*(1-2*u[1]*u[1]-2*u[2]*u[2])-g[2];



// serial print x

    x[3] = x[3] + x[0]*_t;
    x[4] = x[4] + x[1]*_t;
    x[5] = x[5] + x[2]*_t;
    /*

    // filter small bias from accel NED
    for (int i = 0; i < 3; i++)
    {
        if (abs(x[i]) < 0.1)
        {
            x[i] = 0;
            x[i+3] = 0;
        }
    }
    */
    x[6] = x[6] + x[3]*_t + 0.5*x[0]*_t*_t;
    x[7] = x[7] + x[4]*_t + 0.5*x[1]*_t*_t;
    x[8] = x[8] + x[5]*_t + 0.5*x[2]*_t*_t;
}

Eigen::Matrix<double, 9, 7> POS_EKF::W_func(Eigen::Vector<double, 7> &u)
{
    Eigen::Matrix<double, 9, 7> W;
    W << -u[5]*u[3]+u[6]*u[2], u[5]*u[2]+u[6]*u[3], -2*u[4]*u[2]+u[5]*u[1]+u[6]*u[0], -2*u[4]*u[3]-u[5]*u[0]+u[6]*u[1], 0.5-(u[3]*u[3]+u[2]*u[2]), u[2]*u[1]-u[0]*u[3], u[2]*u[0]+u[1]*u[3],
        u[4]*u[3]-u[6]*u[1], u[4]*u[2]-2*u[5]*u[1]-u[6]*u[0], u[4]*u[1]+u[6]*u[3], u[4]*u[0]-2*u[5]*u[3]+u[6]*u[2], u[2]*u[1]+u[0]*u[3], 0.5-(u[1]*u[1]+u[3]*u[3]), u[2]*u[3]-u[0]*u[1],
        -u[4]*u[2]+u[5]*u[1], u[4]*u[3]+u[5]*u[0]-2*u[6]*u[1], -u[4]*u[0]+u[5]*u[3]-2*u[6]*u[2], u[4]*u[1]+u[5]*u[2], u[1]*u[3]-u[0]*u[2], u[0]*u[1]+u[2]*u[3], 0.5-(u[1]*u[1]+u[2]*u[2]),
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0;
    
    // multiply w by 2
    W *= 2;
    
    return W;
}

double* POS_EKF::update(double (&u_in)[7], Eigen::Matrix<double, 4, 4> &Q_q, unsigned long &iTOW, double (&z_in)[6])      // Q_q is the covariance matrix of quaternion input
{
    Eigen::Vector<double, 7> u = Eigen::Vector<double, 7>(u_in);
    Q.block<4, 4>(0, 0) = Q_q;    // replace the 4 by 4 initial part of Q matrix by Q_q


    // predict
    f_func(u);
    Eigen::Matrix<double, 9, 7> W = W_func(u);
    P = F * P * F.transpose() + W * Q * W.transpose();


    // correct and update
    // run correction step if iTOW is different from the previous one
    
    if (iTOW - iTOW_prev > 0)
    {   
        Eigen::Vector<double, 6> y = Eigen::Vector<double, 6>(x.block<6, 1>(3, 0));    //h is the last 6 elements of x
        Eigen::Vector<double, 6> z = Eigen::Vector<double, 6>(z_in);
        //convert z to ned
        Eigen::Vector<double, 3> z_lla = z.block<3, 1>(3, 0);
        //Eigen::Vector<double, 3> z_ned = lla2ned(pos_ref_lla, z_lla);
        z_ned  = lla2ned(pos_ref_lla, z_lla);
        z.block<3, 1>(3, 0) = z_ned;

        Eigen::Matrix<double, 6, 6> S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 9, 6> K = P * H.transpose() * S.inverse();

        x = x + K * (z - y);
        P = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * P;

        iTOW_prev = iTOW;
    }
    
    return x.data();
}