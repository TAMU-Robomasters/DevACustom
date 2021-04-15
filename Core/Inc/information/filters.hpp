#pragma once

namespace filter {

class Kalman {
private:
    double q; //process noise covariance
    double r; //measurement noise covariance
    double p; //estimation error covariance
    double x; //value
    double k; //kalman gain

public:
    Kalman(double process_noise, double sensor_noise, double estimated_error, double intial_value) : q(process_noise), r(sensor_noise), p(estimated_error), x(intial_value) {}

    double step(double measurement) {
        p = p + q;

        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1 - k) * p;

        return x;
    }
};

} // namespace filter
