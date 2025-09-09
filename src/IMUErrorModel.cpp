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

void IMUErrorModel::corruptGyro(TimedSensorData& data)
{
    corruptSensor(
        data,
        "gyroscope",
        C_gyro_inv_,
        bias_gyro_,
        bias_rw_gyro_,
        sigma_gyro_,
        [this](double dt){ updateBias(bias_gyro_, bias_rw_gyro_, dt); });
}

void IMUErrorModel::corruptAccel(TimedSensorData& data)
{
    corruptSensor(
        data,
        "accelerometer",
        C_accel_inv_,
        bias_accel_,
        bias_rw_accel_,
        sigma_accel_,
        [this](double dt){ updateBias(bias_accel_, bias_rw_accel_, dt); });
}

void IMUErrorModel::updateGyroBias(double dt)
{
    updateBias(bias_gyro_, bias_rw_gyro_, dt);
}

void IMUErrorModel::updateAccelBias(double dt)
{
    updateBias(bias_accel_, bias_rw_accel_, dt);
}

void IMUErrorModel::updateBias(
    gtsam::Vector3& bias,
    const gtsam::Vector3& bias_rw,
    double dt)
{
    std::normal_distribution<double> N(0.0, 1.0);
    for (int i = 0; i < 3; ++i)
    {
        bias(i) += bias_rw(i) * sqrt(dt) * N(rng_);
    }
}

void IMUErrorModel::corruptSensor(
    TimedSensorData& data,
    const std::string& sensor,
    const gtsam::Matrix33& C_inv,
    gtsam::Vector3& bias,
    const gtsam::Vector3& bias_rw,
    const gtsam::Vector3& sigma,
    std::function<void(double)> updateBias)
{
    double previous_timestamp = -1.0;
    std::normal_distribution<double> N01(0.0, 1.0);

    for (auto& [t, sensors] : data)
    {
        auto it = sensors.find(sensor);
        if (it == sensors.end()) continue;

        gtsam::Vector3 measurement = std::any_cast<gtsam::Vector3>(it->second);

        
        if (previous_timestamp >= 0.0)
        {
            double dt = std::max(t - previous_timestamp, epsilon_);
            updateBias(dt);
        }

        gtsam::Vector3 noise;
        for (int i = 0; i < 3; ++i)
        {
            noise(i) = sigma(i) * N01(rng_);
        }

        it->second = (C_inv * (measurement + bias + noise)).eval();

        previous_timestamp = t;
    }
}

} // namespace simulation
} // namespace gtsam
