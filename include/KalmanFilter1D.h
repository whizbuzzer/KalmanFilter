/* Discrete Kalman filter implementation for object tracking
 * Reference: https://machinelearningspace.com/object-tracking-python/
 */


#include <cmath>
// #include <iostream>
#include <math.h>

#include <Eigen/Dense>

// // Standard deviations of acceleration and measurement:
// #define SIGMA_A 0.25  // m/s^2
// #define SIGMA_Z 1.2   // m


class KalmanFilter1D {
private:
    double dt = 0;             // Time for one cycle
    Eigen::Vector2d x {0, 0};  // Initial State (position and velocity)
    double u = 0;              // Control input

    // Standard deviations of acceleration(sigma_a) and measurement(sigma_z)
    double std_acc = 0, std_meas = 0;

    // State estimation matrix
    Eigen::Matrix2d A {
        {1, dt},
        {0,  1}
    };

    // Control input matrix
    Eigen::Vector2d B {(dt * dt) / 2, dt};
    
    // State-to-measurement domain transformation matrix
    Eigen::RowVector2d H {1, 0};
    
    // Process Noise Covariance
    double sig_a_2 = pow(std_acc, 2);
    Eigen::Matrix2d Q {
        {sig_a_2 * (pow(dt, 4) / 4), sig_a_2 * (pow(dt, 3) / 2)},
        {sig_a_2 * (pow(dt, 3) / 2),        sig_a_2 * (dt * dt)}
    };

    double R = std_meas * std_meas;              // Measurement Noise Covariance
    Eigen::Matrix2d P = Eigen::Matrix2d::Identity();  // Error Covariance

public:
    KalmanFilter1D(double dt_, double u_, double sigma_a, double sigma_z):dt(dt_), u(u_), std_acc(sigma_a), std_meas(sigma_z) {};

    // Time update equations:
    Eigen::Vector2d predict() {
        // Update time state:
        x = A * x + B * u;

        // Calculate error covariance:
        // P = (A * P * A') + Q
        P = A * P * A.transpose() + Q;
        return x;
    }

    // Measurement update equations:
    void update(double& z0) {
        // Calculating Kalman gain (K):
        auto S = H * P * H.transpose() + R;
        Eigen::Vector2d K = (P * H.transpose()) * (1 / S);

        x += K * (z0 - H * x);
        x(0) = round(x(0));
        x(1) = round(x(1));

        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();

        P = (I - (K * H)) * P;
    }

    // Getters:
    Eigen::Matrix2d get_A() {
        return A;
    }

    Eigen::Vector2d get_B() {
        return B;
    }

    Eigen::RowVector2d get_H() {
        return H;
    }

    Eigen::Matrix2d get_Q() {
        return Q;
    }

    double get_R() {
        return R;
    }

    Eigen::Matrix2d get_P() {
        return P;
    }
};