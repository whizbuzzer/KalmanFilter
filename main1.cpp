/* Main program to test out the designed Discrete Kalman filter */

#include <iostream>
#include <random>
#include <vector>

#include "Detector.h"
#include <Eigen/Dense>
#include "KalmanFilter1D.h"
#include "KalmanFilter2D.h"
#include <matplot/matplot.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>


void KF1DImplementation() {
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

    char c = (char)cv::waitKey(25);
    if (c == 27) {
        return;
    }
}

void KF2DImplementation(int debug_mode) {
    cv::VideoCapture videocap("data/randomball0.avi");

    // // Check if camera opened successfully
    // if(!videocap.isOpened()){
    //     std::cout << "Error opening video stream or file" << std::endl;
    //     return -1;
    // }

    double dt = 0.1;
    double u_x = 1, u_y = 1;
    double std_acc = 0.25;
    double std_meas_x = 0.1, std_meas_y = 0.1; 

    KalmanFilter2D kf = KalmanFilter2D(dt, u_x, u_y, std_acc, std_meas_x, std_meas_y);

    // int debug_mode = debug_mode;

    while (1) {
        cv::Mat frame;
        videocap >> frame;

        if (frame.empty()) {
            break;
        }

        auto centers = detect(frame, debug_mode);

        // If centers are detected, then we track them
        if (!centers.empty()) {	
            cv::circle(frame, centers[0], 10, cv::Scalar(0, 191, 255), 2);

            // Prediction
            auto prediction = kf.predict();
            auto x_p = prediction[0], y_p = prediction[1];
            cv::Point pt1(int(x_p - 15), int(y_p - 15)), pt2(int(x_p + 15), int(y_p + 15));
            cv::rectangle(frame, pt1, pt2, cv::Scalar(255, 0, 0), 2);

            // Correction
            auto correction = kf.update(centers[0]);
            auto x_c = correction[0], y_c = correction[1];
            cv::Point pt3(int(x_c - 15), int(y_c - 15)), pt4(int(x_c + 15), int(y_c + 15));
            cv::rectangle(frame, pt1, pt2, cv::Scalar(0, 0, 255), 2);

            cv::Point pt5(int(x_p + 15), int(y_p + 10)), pt6(int(x_c + 15), int(y_c + 10));
            cv::Point pt7(int(centers[0].x + 15), int(centers[0].y + 10));
            cv::putText(frame, "Estimated position", pt5, 0, 0.5, cv::Scalar(0, 0, 255), 2);
            cv::putText(frame, "Predicted position", pt6, 0, 0.5, cv::Scalar(255, 0, 0), 2);
            cv::putText(frame, "Estimated position", pt7, 0, 0.5, cv::Scalar(0, 191, 255), 2);
        }

        cv::imshow("image", frame);

        // Press ESC on keyboard to exit
        char c = (char)cv::waitKey(25);
        if (c == 27) {
            break;
        }
    }
    videocap.release();
    cv::destroyAllWindows();
}

int main() {
    int dimension;
    std::cout << "Enter \"1\" for 1D implementation or \"2\" for 2D:" << std::endl;
    std::cin >> dimension;
    if (dimension == 1) {
        KF1DImplementation();
    } else if (dimension == 2) {KF2DImplementation(1);}
    else {
        std::cout << "Incorrect input" << std::endl;
        return -1;
    }
}

