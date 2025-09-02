#include "gtsam/simulation/IMUErrorModel.h"

namespace gtsam {
namespace simulation {

IMUErrorModel::IMUErrorModel(
    const gtsam::Matrix33& C_gyro,
    const gtsam::Matrix33& C_accel,
    const gtsam::Vector3&  bias_gyro,
    const gtsam::Vector3&  bias_accel,
    const gtsam::Vector3&  bias_rw_gyro,
    const gtsam::Vector3&  bias_rw_accel,
    const gtsam::Vector3&  sigma_gyro,
    const gtsam::Vector3&  sigma_accel,
    unsigned int seed,
    double epsilon
)
: C_gyro_(C_gyro),
  C_accel_(C_accel),
  bias_gyro_(bias_gyro),
  bias_accel_(bias_accel),
  bias_rw_gyro_(bias_rw_gyro),
  bias_rw_accel_(bias_rw_accel),
  sigma_gyro_(sigma_gyro),
  sigma_accel_(sigma_accel),
  rng_(seed),
  epsilon_(epsilon)
{}

void IMUErrorModel::corruptGyro(TimedSensorData& data) {
    double previous_timestamp = -1.0;

    for (auto& [t, sensors] : data) {
        auto it = sensors.find("gyroscope");
        if (it == sensors.end()) continue; // skip if missing

        gtsam::Vector3 omega = std::any_cast<gtsam::Vector3>(it->second);

        if (previous_timestamp < 0) {
            // First measurement: apply calibration + constant bias only
            it->second = (C_gyro_ * omega + bias_gyro_).eval();
        } else {
            // Subsequent measurements: random walk + white noise
            double dt = t - previous_timestamp;
            updateGyroBias(dt);

            std::normal_distribution<double> noise_dist(0.0, 1.0);
            gtsam::Vector3 noise;
            for (int i = 0; i < 3; ++i)
                noise(i) = sigma_gyro_(i) * noise_dist(rng_);

            it->second = (C_gyro_ * omega + bias_gyro_ + noise).eval();
        }

        previous_timestamp = t;
    }
}

void IMUErrorModel::corruptAccel(TimedSensorData& data) {
    double previous_timestamp = -1.0;

    for (auto& [t, sensors] : data) {
        auto it = sensors.find("accelerometer");
        if (it == sensors.end()) continue; // skip if missing

        gtsam::Vector3 accel = std::any_cast<gtsam::Vector3>(it->second);

        if (previous_timestamp < 0) {
            // First measurement: apply calibration + constant bias only
            it->second = (C_accel_ * accel + bias_accel_).eval();
        } else {
            // Subsequent measurements: random walk + white noise
            double dt = t - previous_timestamp;
            updateAccelBias(dt);

            std::normal_distribution<double> noise_dist(0.0, 1.0);
            gtsam::Vector3 noise;
            for (int i = 0; i < 3; ++i)
                noise(i) = sigma_accel_(i) * noise_dist(rng_);

            it->second = (C_accel_ * accel + bias_accel_ + noise).eval();
        }

        previous_timestamp = t;
    }
}

void IMUErrorModel::updateGyroBias(double dt) {
    std::normal_distribution<double> N(0.0, 1.0);
    for (int i = 0; i < 3; ++i)
        bias_gyro_(i) += bias_rw_gyro_(i) * sqrt(dt) * N(rng_);
}

void IMUErrorModel::updateAccelBias(double dt) {
    std::normal_distribution<double> N(0.0, 1.0);
    for (int i = 0; i < 3; ++i)
        bias_accel_(i) += bias_rw_accel_(i) * sqrt(dt) * N(rng_);
}

} // namespace simulation
} // namespace gtsam
