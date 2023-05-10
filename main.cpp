/* Main program to test out the designed Discrete Kalman filter */

#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include "KalmanFilter1D.h"
#include <matplot/matplot.h>


int main() {
    // Using 'double' rather than 'float' for a higher precision
    double dt = 0.1;
    std::vector<double> t = {0};
    for(double i = 0.1; i < 100; i += dt) {
        t.push_back(i);
    }

    // An arbitrary non-linear model track:
    /* f(t) = 0.1 * (t^2 - t)*/
    std::vector<double> model_track(t);
    for(int i = 0; i < model_track.size(); i++) {
        model_track[i] = 0.1 * ((model_track[i] * model_track[i]) - model_track[i]);
    }

    double u = 2;
    double std_acc = 0.25;
    double std_meas = 1.2;    

    KalmanFilter1D kf = KalmanFilter1D(dt, u, std_acc, std_meas);

    std::vector<double> predictions;
    std::vector<double> measurements;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0, 50.0);

    for(float x:model_track) {
        double randval = distribution(generator);
        Eigen::RowVector2d z = (kf.get_H() * x);
        z(0) = z(0) + randval;
        z(1) = z(1) + randval;

        measurements.push_back(z(0));
        predictions.push_back(kf.predict()(0));
        kf.update(z(0));
    }

    std::vector<std::string> newcolors = {"#D41243", "#5F2AF0", "#1B8712"};
    matplot::colororder(newcolors);

    matplot::plot(t, measurements)->line_width(2);
    matplot::hold(matplot::on);
    matplot::plot(t, model_track)->line_width(2);
    matplot::plot(t, predictions)->line_width(2);
    matplot::hold(matplot::off);

    // auto axes = matplot::axes();
    // axes->color("#94F008");
    auto l = matplot::legend({"Measurements", "Model track", "Predictions"});
    l->location(matplot::legend::general_alignment::topleft);
    l->num_rows(2);
    matplot::title("Discrete Kalman Filter tracking a moving object in 1-D");
    matplot::xlabel("Time (s)");
    matplot::ylabel("Position (m)");
    matplot::grid(matplot::on);
    matplot::show();
    return 0;
}