#pragma once

// Standard library
#include <any>
#include <functional>
#include <map>
#include <string>
#include <vector>

// Third-party
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

// Local project headers
#include "ScenarioSimulator.h"

namespace gtsam {
namespace simulation {

/**
 * @class IMUScenarioSimulator
 * @brief Simulates IMU measurements (accelerometer & gyroscope) from a user-defined trajectory.
 *
 * The class accepts a trajectory function that outputs a Pose3 at time @f$t@f$, and computes:
 *
 * - **Linear velocity** (world frame):  
 *   For times @f$t_{i-1}, t_i, t_{i+1}@f$ with step sizes
 *   @f$h_1 = t_i - t_{i-1},\; h_2 = t_{i+1} - t_i@f$:
 *   @f[
 *   v(t_i) \approx
 *     \frac{-h_2}{h_1(h_1+h_2)}\, p(t_{i-1})
 *   + \frac{h_2-h_1}{h_1 h_2}\, p(t_i)
 *   + \frac{h_1}{h_2(h_1+h_2)}\, p(t_{i+1})
 *   @f]
 *
 * - **Linear acceleration** (world frame):  
 *   Using velocities at @f$t_{i-1}, t_i, t_{i+1}@f$:
 *   @f[
 *   a(t_i) \approx
 *     \frac{-h_2}{h_1(h_1+h_2)}\, v(t_{i-1})
 *   + \frac{h_2-h_1}{h_1 h_2}\, v(t_i)
 *   + \frac{h_1}{h_2(h_1+h_2)}\, v(t_{i+1})
 *   @f]
 *
 * - **Angular velocity** (world frame, from rotation logmap):  
 *   @f[
 *   \omega(t_i) \approx
 *     \text{weighted combination of }
 *     \frac{\log(R_{i-1}^\top R_i)}{h_1},\;
 *     \frac{\log(R_i^\top R_{i+1})}{h_2}
 *   @f]
 *
 * - **Angular acceleration** (world frame):  
 *   @f[
 *   \alpha(t_i) \approx
 *     \frac{-h_2}{h_1(h_1+h_2)}\, \omega_{i-1}
 *   + \frac{h_2-h_1}{h_1 h_2}\, \omega_i
 *   + \frac{h_1}{h_2(h_1+h_2)}\, \omega_{i+1}
 *   @f]
 *
 * - **Lever-arm rotational accelerations** (body frame, lever arm @f$l@f$):  
 *   @f[
 *   a_{\mathrm{centripetal}} = \omega \times (\omega \times l), \quad
 *   a_{\mathrm{tangential}} = \alpha \times l, \quad
 *   a_{\mathrm{rot}} = a_{\mathrm{centripetal}} + a_{\mathrm{tangential}}
 *   @f]
 *
 * - **Accelerometer measurement** (body frame, includes gravity & lever-arm):  
 *   @f[
 *   f_{\mathrm{meas}}(t_i) = R(t_i)^\top \big(a(t_i) - g\big) + a_{\mathrm{rot}},
 *   \qquad g = (0,0,-9.81)^T
 *   @f]
 *
 * The simulator returns a map of measurement vectors keyed by timestamp. Each map contains:
 * - "accelerometer" : 3D vector (body-frame acceleration incl. gravity & lever-arm)
 * - "gyroscope"     : 3D vector (body-frame angular velocity)
 * - "true_acceleration" : 3D vector (world-frame linear acceleration without gravity)
 * - "true_velocity"     : 3D vector (world-frame linear velocity)
 * - "true_angular_velocity" : 3D vector (world-frame angular velocity)
 * - "true_angular_acceleration" : 3D vector (world-frame angular acceleration)
 * - "pose" : Pose3 (ground-truth pose at timestamp)
 */
class IMUScenarioSimulator : public ScenarioSimulator
{
public:
    /// SensorData: sensor type → value
    using SensorData = std::map<std::string, std::any>;

    /// TimedSensorData: time → SensorData
    using TimedSensorData = std::map<double, SensorData>;

    /// Trajectory: time → Pose3 (kept sorted automatically by std::map)
    using Trajectory = std::map<double, gtsam::Pose3>;

    /// DifferentiationMethod: Method for numerical derivatives
    enum class DifferentiationMethod { Central, Forward, Backward };

    /// Vector3Seq: index → Vec3
    using Vector3Seq = std::vector<gtsam::Vector3>;

    /**
     * @brief Construct an IMU scenario simulator from a trajectory.
     *
     * @param trajectory Sorted map of timestamps to Pose3, giving ground-truth IMU poses
     * @param method Differentiation method for numerical derivatives (Central, Forward, Backward)
     * @param lever_arm_history Optional lever-arm vectors (IMU center to sensor) for rotational acceleration
     * @param epsilon Small value for numerical stability in derivative calculations
     */
    IMUScenarioSimulator(
        const Trajectory& trajectory,
        DifferentiationMethod method = DifferentiationMethod::Central,
        const Vector3Seq& lever_arm_history = Vector3Seq(),
        double epsilon = 1e-8);

    /**
     * @brief Simulate the scenario.
     *
     * @return A map of measurements keyed by timestamp.
     *         Each entry maps a simulation time (double) to a collection of sensor readings.
     *         Sensor readings are stored as a map from sensor name (string) to sensor data (std::any).
     *         Subclasses decide the exact sensor types and contents.
     *
     * Example:
     * @code
     * measurements[0.0]["accelerometer"] = gtsam::Vector3(0.0, 0.0, -9.81);
     * measurements[0.0]["gyroscope"]     = gtsam::Vector3(0.01, 0.0, 0.0);
     * @endcode
     */
    TimedSensorData simulateScenario() override;

private:
    Trajectory trajectory_;             ///< Ground-truth poses keyed by time
    DifferentiationMethod diff_method_; ///< Differencing method for numerical derivatives
    Vector3Seq lever_arm_history_;      ///< Optional lever-arm vectors per timestep
    double epsilon_;                    ///< Small tolerance for safe divisions

    /**
     * @brief Safe scalar division with tolerance.
     * @param numerator Numerator of the fraction
     * @param denominator Denominator of the fraction
     * @return numerator/denominator if denominator >= epsilon_, otherwise 0.0
     */
    double safeDiv(
        double numerator,
        double denominator);

    /**
     * @brief Safe vector-scalar division with tolerance.
     * @param v Vector numerator
     * @param denominator Scalar denominator
     * @return v/denominator if denominator >= epsilon_, otherwise zero vector
     */
    gtsam::Vector3 safeDiv(
        const gtsam::Vector3& v,
        double denominator);

    /**
     * @brief Compute central difference derivative for a vector quantity.
     *
     * Uses non-uniform central difference for arbitrary timestep spacing.
     *
     * @param v_prev Value at t_prev
     * @param v_curr Value at t_curr
     * @param v_next Value at t_next
     * @param t_prev Previous timestamp
     * @param t_curr Current timestamp
     * @param t_next Next timestamp
     * @return Central-difference estimate of derivative at t_curr
     */
    gtsam::Vector3 computeCentralDiffVector(
        const gtsam::Vector3& v_prev,
        const gtsam::Vector3& v_curr,
        const gtsam::Vector3& v_next,
        double t_prev,
        double t_curr,
        double t_next);

    /**
     * @brief Compute forward difference derivative for a vector quantity.
     *
     * @param v_curr Value at t_curr
     * @param v_next Value at t_next
     * @param t_curr Current timestamp
     * @param t_next Next timestamp
     * @return Forward-difference estimate of derivative at t_curr
     */
    gtsam::Vector3 computeForwardDiffVector(
        const gtsam::Vector3& v_curr,
        const gtsam::Vector3& v_next,
        double t_curr,
        double t_next);

    /**
     * @brief Compute backward difference derivative for a vector quantity.
     *
     * @param v_prev Value at t_prev
     * @param v_curr Value at t_curr
     * @param t_prev Previous timestamp
     * @param t_curr Current timestamp
     * @return Backward-difference estimate of derivative at t_curr
     */
    gtsam::Vector3 computeBackwardDiffVector(
        const gtsam::Vector3& v_prev,
        const gtsam::Vector3& v_curr,
        double t_prev,
        double t_curr);

    /**
     * @brief Compute central difference angular velocity from SO(3).
     *
     * Uses logmap of relative rotations with non-uniform central differencing.
     *
     * @param R_prev Rotation at t_prev
     * @param R_curr Rotation at t_curr
     * @param R_next Rotation at t_next
     * @param t_prev Previous timestamp
     * @param t_curr Current timestamp
     * @param t_next Next timestamp
     * @return Angular velocity vector at t_curr (world frame)
     */
    gtsam::Vector3 computeCentralDiffSO3(
        const gtsam::Rot3& R_prev,
        const gtsam::Rot3& R_curr,
        const gtsam::Rot3& R_next,
        double t_prev,
        double t_curr,
        double t_next);

    /**
     * @brief Compute forward difference angular velocity from SO(3).
     *
     * @param R_curr Rotation at t_curr
     * @param R_next Rotation at t_next
     * @param t_curr Current timestamp
     * @param t_next Next timestamp
     * @return Angular velocity vector at t_curr (world frame)
     */
    gtsam::Vector3 computeForwardDiffSO3(
        const gtsam::Rot3& R_curr,
        const gtsam::Rot3& R_next,
        double t_curr,
        double t_next);

    /**
     * @brief Compute backward difference angular velocity from SO(3).
     *
     * @param R_prev Rotation at t_prev
     * @param R_curr Rotation at t_curr
     * @param t_prev Previous timestamp
     * @param t_curr Current timestamp
     * @return Angular velocity vector at t_curr (world frame)
     */
    gtsam::Vector3 computeBackwardDiffSO3(
        const gtsam::Rot3& R_prev,
        const gtsam::Rot3& R_curr,
        double t_prev,
        double t_curr);
};

} // namespace simulation
} // namespace gtsam
