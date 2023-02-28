/* Main program to test out the designed Discrete Kalman filter */

#include <iostream>
#include <random>
#include <vector>

#include "KalmanFilter.h"
#include <matplot/matplot.h>


using namespace std;
using namespace matplot;

int main() {
    double dt = 0.1;
    vector<double> t = {0};
    for(double i = 0.1; i < 100; i += dt) {
        t.push_back(i);
    }

    // An arbitrary non-linear model track:
    vector<double> model_track(t);
    for(int i = 0; i < model_track.size(); i++) {
        model_track[i] = 0.1 * ((model_track[i] * model_track[i]) - model_track[i]);
    }

    double u = 2;
    double std_acc = 0.25;
    double std_meas = 1.2;    

    KalmanFilter kf = KalmanFilter(dt, u, std_acc, std_meas);

    vector<double> predictions;
    vector<double> measurements;

    default_random_engine generator;
    normal_distribution<double> distribution(0, 50.0);

    for(float x:model_track) {
        double randval = distribution(generator);
        RowVector2d z = (kf.get_H() * x);
        z(0) = z(0) + randval;
        z(1) = z(1) + randval;

        measurements.push_back(z(0));
        predictions.push_back(kf.predict()(0));
        kf.update(z(0));
    }

    std::vector<std::string> newcolors = {"#D41243", "#5F2AF0", "#1B8712"};
    colororder(newcolors);

    plot(t, measurements)->line_width(2);
    hold(on);
    grid(on);
    plot(t, model_track)->line_width(2);
    plot(t, predictions)->line_width(2);
    hold(off);

    auto l = legend("Measurements", "Model track", "Predictions");
    l->location(legend::general_alignment::topleft);
    l->num_rows(2);
    title("Discrete Kalman Filter tracking a moving object in 1-D");
    show();
    return 0;
}