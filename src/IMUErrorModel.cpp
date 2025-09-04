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
  C_gyro_inv_(C_gyro.inverse()),
  C_accel_inv_(C_accel.inverse()),
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
    std::normal_distribution<double> N01(0.0, 1.0);

    for (auto& [t, sensors] : data) {
        auto it = sensors.find("gyroscope");
        if (it == sensors.end()) continue;

        gtsam::Vector3 omega = std::any_cast<gtsam::Vector3>(it->second);

        if (previous_timestamp >= 0.0) {
            double dt = std::max(t - previous_timestamp, epsilon_);
            updateGyroBias(dt);
        }

        gtsam::Vector3 noise;
        for (int i = 0; i < 3; ++i) {
            noise(i) = sigma_gyro_(i) * N01(rng_);
        }

        it->second = (C_gyro_inv_ * (omega + bias_gyro_ + noise)).eval();

        previous_timestamp = t;
    }
}

void IMUErrorModel::corruptAccel(TimedSensorData& data) {
    double previous_timestamp = -1.0;
    std::normal_distribution<double> N01(0.0, 1.0);

    for (auto& [t, sensors] : data) {
        auto it = sensors.find("accelerometer");
        if (it == sensors.end()) continue;

        gtsam::Vector3 a = std::any_cast<gtsam::Vector3>(it->second);

        if (previous_timestamp >= 0.0) {
            double dt = std::max(t - previous_timestamp, epsilon_);
            updateAccelBias(dt);
        }

        gtsam::Vector3 noise;
        for (int i = 0; i < 3; ++i) {
            noise(i) = sigma_accel_(i) * N01(rng_);
        }

        it->second = (C_accel_inv_ * (a + bias_accel_ + noise)).eval();

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
