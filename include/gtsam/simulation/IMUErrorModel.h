#pragma once

// Standard library
#include <any>
#include <map>
#include <random>

// Third-party
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

namespace gtsam {
namespace simulation {

/**
 * @class IMUErrorModel
 * @brief Represents an IMU error model applying calibration, biases, random walks, and measurement noise.
 *
 * The class initializes all error parameters with default values or user-provided inputs:
 *
 * **1. Calibration matrices**:  
 * - Gyroscope: @f$C_\text{gyro}@f$, 3x3 matrix representing scale factors and axis misalignment  
 * - Accelerometer: @f$C_\text{accel}@f$, 3x3 matrix representing scale factors and axis misalignment  
 * Applied as:  
 * @f[
 *    \omega_\text{cal} = C_\text{gyro} \, \omega, \quad
 *    a_\text{cal} = C_\text{accel} \, a
 * @f]
 *
 * **2. Constant biases**:  
 * - Gyroscope: @f$b_\text{gyro}@f$, vector added to every measurement  
 * - Accelerometer: @f$b_\text{accel}@f$, vector added to every measurement  
 * Applied as:  
 * @f[
 *    \omega_\text{biased} = \omega_\text{cal} + b_\text{gyro}, \quad
 *    a_\text{biased} = a_\text{cal} + b_\text{accel}
 * @f]
 *
 * **3. Bias random walk**:  
 * - Gyroscope: @f$w_\text{gyro}@f$, std deviation per √s  
 * - Accelerometer: @f$w_\text{accel}@f$, std deviation per √s  
 * Bias evolves at each timestep @f$\Delta t@f$ as:  
 * @f[
 *    b_\text{gyro,new} = b_\text{gyro,old} + w_\text{gyro} \sqrt{\Delta t} \, \mathcal{N}(0,1)
 * @f]  
 * @f[
 *    b_\text{accel,new} = b_\text{accel,old} + w_\text{accel} \sqrt{\Delta t} \, \mathcal{N}(0,1)
 * @f]
 *
 * **4. Measurement noise**:  
 * - Gyroscope: @f$\sigma_\text{gyro}@f$, per-sample std  
 * - Accelerometer: @f$\sigma_\text{accel}@f$, per-sample std  
 * Applied at each timestep as:  
 * @f[
 *    \omega_\text{meas} = \omega_\text{biased} + \sigma_\text{gyro} \, \mathcal{N}(0,1)
 * @f]  
 * @f[
 *    a_\text{meas} = a_\text{biased} + \sigma_\text{accel} \, \mathcal{N}(0,1)
 * @f]
 *
 * **5. Random number generator**:  
 * - Seeded for reproducibility. Default seed = 42.
 */
class IMUErrorModel {
public:

    /// SensorData: sensor type → value
    using SensorData = std::map<std::string, std::any>;

    /// TimedSensorData: time → SensorData
    using TimedSensorData = std::map<double, SensorData>;

    /**
     * @brief Construct an IMU error model with optional calibration and noise parameters.
     *
     * @param C_gyro Gyroscope calibration matrix (scale, misalignment, non-orthogonality)
     * @param C_accel Accelerometer calibration matrix (scale, misalignment, non-orthogonality)
     * @param bias_gyro Constant gyroscope bias
     * @param bias_accel Constant accelerometer bias
     * @param bias_rw_gyro Random-walk standard deviation for gyroscope bias (per √s)
     * @param bias_rw_accel Random-walk standard deviation for accelerometer bias (per √s)
     * @param sigma_gyro White noise standard deviation for gyroscope (per sample)
     * @param sigma_accel White noise standard deviation for accelerometer (per sample)
     * @param seed Random number generator seed
     */
    IMUErrorModel(
        const gtsam::Matrix33& C_gyro   = gtsam::Matrix33::Identity(),
        const gtsam::Matrix33& C_accel  = gtsam::Matrix33::Identity(),
        const gtsam::Vector3&  bias_gyro = gtsam::Vector3::Zero(),
        const gtsam::Vector3&  bias_accel = gtsam::Vector3::Zero(),
        const gtsam::Vector3&  bias_rw_gyro = gtsam::Vector3::Zero(),
        const gtsam::Vector3&  bias_rw_accel = gtsam::Vector3::Zero(),
        const gtsam::Vector3&  sigma_gyro = gtsam::Vector3::Zero(),
        const gtsam::Vector3&  sigma_accel = gtsam::Vector3::Zero(),
        unsigned int seed = 42,
        double epsilon = 1e-8
    );

    /**
     * @brief Corrupt gyroscope measurements in the provided timed sensor data.
     *
     * Applies calibration, bias, bias random walk, and white noise to all `"gyroscope"` entries.
     * The first measurement is corrupted only with calibration and bias (no white noise).
     *
     * @param data Map of timestamps to sensor measurements to be updated
     */
    void corruptGyro(TimedSensorData& data);

    /**
     * @brief Corrupt accelerometer measurements in the provided timed sensor data.
     *
     * Applies calibration, bias, bias random walk, and white noise to all `"accelerometer"` entries.
     * The first measurement is corrupted only with calibration and bias (no white noise).
     *
     * @param data Map of timestamps to sensor measurements to be updated
     */
    void corruptAccel(TimedSensorData& data);

private:
     /// @name Calibration models
    /// @{
    /// Gyroscope calibration matrix.
    gtsam::Matrix33 C_gyro_;

    /// Accelerometer calibration matrix.
    gtsam::Matrix33 C_accel_;

    /// Inverse gyroscope calibration matrix (precomputed for efficiency).
    gtsam::Matrix33 C_gyro_inv_;

    /// Inverse accelerometer calibration matrix (precomputed for efficiency).
    gtsam::Matrix33 C_accel_inv_;
    /// @}

    /// @name Bias models
    /// @{
    /// Current gyroscope bias vector (rad/s).
    gtsam::Vector3 bias_gyro_;

    /// Current accelerometer bias vector (m/s²).
    gtsam::Vector3 bias_accel_;

    /// Gyroscope bias random walk standard deviation (rad/sqrt(s)).
    gtsam::Vector3 bias_rw_gyro_;

    /// Accelerometer bias random walk standard deviation (m/s²/sqrt(s)).
    gtsam::Vector3 bias_rw_accel_;
    /// @}

    /// @name Noise models
    /// @{
    /// Gyroscope measurement noise standard deviation (rad/s).
    gtsam::Vector3 sigma_gyro_;

    /// Accelerometer measurement noise standard deviation (m/s²).
    gtsam::Vector3 sigma_accel_;

    /// Random number generator used for noise and bias random walks.
    std::mt19937 rng_;
    /// @}

    /// @name Common parameters
    /// @{
    /// Minimum timestep allowed to avoid degenerate bias updates.
    double epsilon_;
    /// @}

    /**
     * @brief Update gyroscope bias using a random walk model.
     *
     * @param dt Time step in seconds.
     */
    void updateGyroBias(double dt);

    /**
     * @brief Update accelerometer bias using a random walk model.
     *
     * @param dt Time step in seconds.
     */
    void updateAccelBias(double dt);

    /**
     * @brief Update a generic sensor bias vector using a random walk model.
     *
     * @param bias Reference to the current bias vector.
     * @param bias_rw Random walk standard deviation for each axis.
     * @param dt Time step in seconds.
     */
    void updateBias(
        gtsam::Vector3& bias,
        const gtsam::Vector3& bias_rw,
        double dt);

    /**
     * @brief Corrupt sensor measurements with bias and Gaussian noise.
     *
     * @param data Timed sensor data to modify in-place.
     * @param sensor Sensor name to corrupt (e.g., "gyroscope", "accelerometer").
     * @param C_inv Inverse calibration matrix for the sensor.
     * @param bias Reference to the current sensor bias vector.
     * @param bias_rw Random walk standard deviation for sensor bias.
     * @param sigma Measurement noise standard deviation vector.
     * @param updateBias Function to update bias given timestep @p dt.
     */
    void corruptSensor(
        TimedSensorData& data,
        const std::string& sensor,
        const gtsam::Matrix33& C_inv,
        gtsam::Vector3& bias,
        const gtsam::Vector3& bias_rw,
        const gtsam::Vector3& sigma,
        std::function<void(double)> updateBias);

};

} // namespace simulation
} // namespace gtsam
