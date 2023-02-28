/* Discrete Kalman filter implementation for object tracking
 * Reference: https://machinelearningspace.com/object-tracking-python/
 */


#include <cmath>
#include <iostream>
#include <math.h>

#include <Eigen/Dense>

// // Standard deviations of acceleration and measurement:
// #define SIGMA_A 0.25  // m/s^2
// #define SIGMA_Z 1.2   // m

using namespace std;
using namespace Eigen;

class KalmanFilter {
private:
    double dt = 0;
    double u = 0;
    double std_acc, std_meas;
    Matrix2d A {
        {1, dt},
        {0, 1}
    };
    Matrix<double, 2, 1> B {(dt * dt) / 2, dt};
    RowVector2d H {1, 0};
    Matrix2d Q {
        {pow(std_acc, 2) * (pow(dt, 4) / 4), pow(std_acc, 2) * (pow(dt, 3) / 2)},
        {pow(std_acc, 2) * (pow(dt, 3) / 2), pow(std_acc, 2) * (dt * dt)},
    };
    double R = std_meas * std_meas;
    Matrix2d P = Matrix2d::Identity();
    Vector2d x {0, 0};

public:
    KalmanFilter(double dt_, double u_, double sigma_a, double sigma_z):dt(dt_), u(u_), std_acc(sigma_a), std_meas(sigma_z) {};

    // Time update equations:
    Matrix<double, 2, 1> predict() {
        // Update time state:
        x = A * x + B * u;

        // Calculate error covariance:
        // P = (A * P * A') + Q
        P = A * P * A.transpose() + Q;
        return x;
    }

    // Measurement update equations:
    void update(double z0) {
        // Calculating Kalman gain (K):
        auto S = H * P * H.transpose() + R;
        Vector2d K = (P * H.transpose() * (1 / S));

        x += K * (z0 - H * x);
        x(0) = round(x(0));
        x(1) = round(x(1));

        Matrix2d I = Matrix2d::Identity();

        P = (I - (K * H)) * P;
    }

    RowVector2d get_H() {
        return H;
    }

};