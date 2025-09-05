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
 * The simulator supports two modes: discrete trajectory (poses at sampled times) and continuous trajectory
 * (user-provided analytic functions for pose and optionally derivatives). It computes IMU measurements
 * including linear velocity, acceleration, angular velocity, and angular acceleration, plus lever-arm effects.
 *
 * ### Discrete trajectory formulas
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
 * ### Continuous trajectory formulas
 *
 * If analytic derivatives are provided via a continuous trajectory model:
 * - Pose: @f$p(t) = \text{pose}(t)@f$
 * - Linear velocity: @f$v(t) = \text{velocity}(t)@f$, else fallback finite difference
 * - Linear acceleration: @f$a(t) = \text{acceleration}(t)@f$, else fallback finite difference
 * - Angular velocity: @f$\omega(t) = \text{angularVelocity}(t)@f$, else finite difference via logmap
 * - Angular acceleration: @f$\alpha(t) = \text{angularAcceleration}(t)@f$, else fallback finite difference
 *
 * Lever-arm accelerations are computed using:
 * @f[
 * a_{\mathrm{centripetal}} = \omega \times (\omega \times l(t)),\quad
 * a_{\mathrm{tangential}} = \alpha \times l(t),\quad
 * a_{\mathrm{rot}} = a_{\mathrm{centripetal}} + a_{\mathrm{tangential}}
 * @f]
 *
 * Accelerometer measurement (body frame, includes gravity & lever-arm):
 * @f[
 * f_{\mathrm{meas}}(t) = R(t)^\top \big(a(t) - g\big) + a_{\mathrm{rot}},\quad
 * g = (0,0,-9.81)^T
 * @f]
 *
 * ### Output
 *
 * Returns a map of measurement vectors keyed by timestamp. Each map contains:
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
    /// Sensor Type → Value
    using SensorData        = std::map<std::string, std::any>;

    /// Time → SensorData
    using TimedSensorData   = std::map<double, SensorData>;

    /// Time → Pose3
    using Trajectory        = std::map<double, gtsam::Pose3>;

    /// Method for numerical derivatives
    enum class DifferentiationMethod { Central, Forward, Backward };

    /// Index → Vector3
    using Vector3Seq        = std::vector<gtsam::Vector3>;

    /// Pose3(t), v(t), a(t), omega(t), alpha(t)
    struct TrajectoryModel {
        std::function<gtsam::Pose3(double)> pose;
        std::function<gtsam::Vector3(double)> velocity            = nullptr;
        std::function<gtsam::Vector3(double)> acceleration        = nullptr;
        std::function<gtsam::Vector3(double)> angularVelocity     = nullptr;
        std::function<gtsam::Vector3(double)> angularAcceleration = nullptr;
    };

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
     * @brief Construct an IMU scenario simulator from a continuous trajectory model.
     *
     * Uses analytic derivatives if provided; otherwise falls back to finite differencing.
     *
     * @param model Continuous trajectory model with pose (required) and optional derivatives
     * @param dt Sampling interval for evaluating the model
     * @param duration Total duration to simulate
     * @param method Differentiation method for fallback numerical derivatives (Central, Forward, Backward)
     * @param lever_arm_func Optional time-varying lever-arm function (IMU center to sensor)
     * @param epsilon Small value for numerical stability in derivative calculations
     */
    IMUScenarioSimulator(
        const TrajectoryModel& model,
        const std::vector<double>& timestamps,
        DifferentiationMethod method = DifferentiationMethod::Central,
        std::function<gtsam::Vector3(double)> lever_arm_func = nullptr,
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
    /// @name Discrete trajectory data
    /// @{
    /// Ground-truth IMU poses keyed by timestamp
    Trajectory trajectory_;
    /// Numerical differentiation method for discrete trajectory (Central, Forward, Backward)
    DifferentiationMethod diff_method_;
    /// Optional lever-arm vectors (IMU center to sensor) per discrete timestep
    Vector3Seq lever_arm_history_;
    /// @}

    /// @name Continuous trajectory data
    /// @{
    /// User-provided continuous trajectory model (pose + optional derivatives)
    TrajectoryModel trajectory_model_;
    /// Vector of timestamps at which to evaluate the continuous trajectory
    std::vector<double> timestamps_;
    /// Optional time-varying lever-arm function for continuous trajectory
    std::function<gtsam::Vector3(double)> lever_arm_func_;
    /// @}

    /// @name Common parameters
    /// @{
    /// Small tolerance used in safe divisions and numerical calculations
    double epsilon_;
    /// @}

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

    /**
     * @brief Differentiate translation trajectory at t_curr.
     *
     * Computes linear velocity by applying finite differences to poses.
     * Uses central difference if both neighbors are available, otherwise
     * falls back to forward/backward difference.
     *
     * @param pose_prev Pose at t_prev (ignored if has_prev = false)
     * @param pose_curr Pose at t_curr
     * @param pose_next Pose at t_next (ignored if has_next = false)
     * @param t_prev Previous timestamp
     * @param t_curr Current timestamp
     * @param t_next Next timestamp
     * @param has_prev Whether a valid previous pose exists
     * @param has_next Whether a valid next pose exists
     * @return Linear velocity vector at t_curr (world frame)
     */
    gtsam::Vector3 differentiateTranslation(
        const gtsam::Pose3& pose_prev,
        const gtsam::Pose3& pose_curr,
        const gtsam::Pose3& pose_next,
        double t_prev,
        double t_curr,
        double t_next,
        bool has_prev,
        bool has_next);

    /**
     * @brief Differentiate rotation trajectory at t_curr.
     *
     * Computes angular velocity by applying finite differences to poses.
     * Uses central difference if both neighbors are available, otherwise
     * falls back to forward/backward difference.
     *
     * @param pose_prev Pose at t_prev (ignored if has_prev = false)
     * @param pose_curr Pose at t_curr
     * @param pose_next Pose at t_next (ignored if has_next = false)
     * @param t_prev Previous timestamp
     * @param t_curr Current timestamp
     * @param t_next Next timestamp
     * @param has_prev Whether a valid previous pose exists
     * @param has_next Whether a valid next pose exists
     * @return Angular velocity vector at t_curr (world frame)
     */
    gtsam::Vector3 differentiateRotation(
        const gtsam::Pose3& pose_prev,
        const gtsam::Pose3& pose_curr,
        const gtsam::Pose3& pose_next,
        double t_prev,
        double t_curr,
        double t_next,
        bool has_prev,
        bool has_next);

    /**
     * @brief Differentiate a vector trajectory at t_curr.
     *
     * Computes the time derivative of a vector sequence (e.g. velocity or
     * angular velocity) using finite differences. Uses central difference
     * if both neighbors are available, otherwise falls back to forward/
     * backward difference.
     *
     * @param v_prev Vector at t_prev (ignored if has_prev = false)
     * @param v_curr Vector at t_curr
     * @param v_next Vector at t_next (ignored if has_next = false)
     * @param t_prev Previous timestamp
     * @param t_curr Current timestamp
     * @param t_next Next timestamp
     * @param has_prev Whether a valid previous vector exists
     * @param has_next Whether a valid next vector exists
     * @return Estimated derivative vector at t_curr
     */
    gtsam::Vector3 differentiateVector(
        const gtsam::Vector3& v_prev,
        const gtsam::Vector3& v_curr,
        const gtsam::Vector3& v_next,
        double t_prev,
        double t_curr,
        double t_next,
        bool has_prev,
        bool has_next);

    /**
     * @brief Simulate IMU measurements using a discrete trajectory.
     *
     * Computes linear velocity, angular velocity, linear acceleration, and angular acceleration
     * at each discrete timestep provided in the trajectory_ member. Also accounts for lever-arm
     * effects if lever_arm_history_ is provided.
     *
     * @return TimedSensorData A map from timestamp to sensor measurements, including:
     *         - "gyroscope" : angular velocity (Vector3)
     *         - "accelerometer" : linear acceleration in body frame (Vector3)
     *         - "true_velocity" : linear velocity (Vector3)
     *         - "true_angular_velocity" : angular velocity (Vector3)
     *         - "true_acceleration" : linear acceleration (Vector3)
     *         - "true_angular_acceleration" : angular acceleration (Vector3)
     *         - "pose" : Pose3 at the current timestep
     */
    TimedSensorData simulateDiscrete();

    /**
     * @brief Simulate IMU measurements using a continuous trajectory model.
     *
     * Computes linear velocity, angular velocity, linear acceleration, and angular acceleration
     * at each timestamp provided in timestamps_. If the trajectory model provides analytical
     * velocity or acceleration, it is used; otherwise, numerical differentiation is applied.
     * Also accounts for lever-arm effects using lever_arm_func_ or lever_arm_history_.
     *
     * @return TimedSensorData A map from timestamp to sensor measurements, including:
     *         - "gyroscope" : angular velocity (Vector3)
     *         - "accelerometer" : linear acceleration in body frame (Vector3)
     *         - "true_velocity" : linear velocity (Vector3)
     *         - "true_angular_velocity" : angular velocity (Vector3)
     *         - "true_acceleration" : linear acceleration (Vector3)
     *         - "true_angular_acceleration" : angular acceleration (Vector3)
     *         - "pose" : Pose3 at the current timestamp
     */
    TimedSensorData simulateContinuous();
};

} // namespace simulation
} // namespace gtsam
