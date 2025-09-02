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
 * **1. Linear velocity** (world-frame, non-uniform weighted finite differences):  
 * For times @f$t_{i-1}, t_i, t_{i+1}@f$ with step sizes  
 * @f$h_1 = t_i - t_{i-1}@f$, @f$h_2 = t_{i+1} - t_i@f$:  
 * @f[
 *    v(t_i) \approx 
 *        \frac{-h_2}{h_1 (h_1 + h_2)} p(t_{i-1})
 *      + \frac{h_2 - h_1}{h_1 h_2} p(t_i)
 *      + \frac{h_1}{h_2 (h_1 + h_2)} p(t_{i+1})
 * @f]
 * One-sided formulas are used at trajectory boundaries.
 *
 * **2. Linear acceleration** (world-frame, non-uniform weighted finite differences):  
 * Using velocities at @f$t_{i-1}, t_i, t_{i+1}@f$:  
 * @f[
 *    a(t_i) \approx 
 *        \frac{-h_2}{h_1 (h_1 + h_2)} v(t_{i-1})
 *      + \frac{h_2 - h_1}{h_1 h_2} v(t_i)
 *      + \frac{h_1}{h_2 (h_1 + h_2)} v(t_{i+1})
 * @f]
 * One-sided formulas are used at trajectory boundaries.
 *
 * **3. Angular velocity** (world-frame, gyroscope) via rotation logmap:  
 * @f[
 *    \omega(t_i) \approx 
 *        \text{weighted sum of } \log(R_{i-1}^\top R_i),\; \log(R_i^\top R_{i+1})
 * @f]
 * Central, forward, or backward differences are applied depending on neighboring data availability.
 *
 * **4. Angular acceleration** (world-frame, second derivative of angular velocity):  
 * Computed using the same non-uniform finite differences as linear acceleration:  
 * @f[
 *    \alpha(t_i) \approx 
 *        \frac{-h_2}{h_1 (h_1 + h_2)} \omega_{i-1}
 *      + \frac{h_2 - h_1}{h_1 h_2} \omega_i
 *      + \frac{h_1}{h_2 (h_1 + h_2)} \omega_{i+1}
 * @f]
 * One-sided formulas are used at trajectory boundaries.
 *
 * **5. Rotational contributions due to lever arm** (body-frame):  
 * For a lever arm @f$l@f$ (vector from IMU center to sensor point in body frame):  
 * - Centripetal acceleration: @f$ a_\text{centripetal} = \omega \times (\omega \times l) @f$  
 * - Tangential acceleration:  @f$ a_\text{tangential} = \alpha \times l @f$  
 * - Total lever-arm contribution: @f$ a_\text{rot} = a_\text{centripetal} + a_\text{tangential} @f$
 *
 * **6. Accelerometer measurement** (body-frame, includes gravity and lever-arm effects):  
 * @f[
 *    f_\text{meas}(t_i) = R(t_i)^\top (a(t_i) - g) + a_\text{rot}
 * @f]
 * where @f$g = (0,0,-9.81)^T@f$ and @f$R(t_i)@f$ is the rotation of the IMU from world to body frame.
 *
 * The simulator returns a map of measurement maps keyed by timestamp. Each map contains:
 * - `"accelerometer"`            : 3D vector (body-frame linear acceleration including gravity and lever-arm effects)
 * - `"gyroscope"`                : 3D vector (body-frame angular velocity)
 * - `"true_acceleration"`        : 3D vector (world-frame linear acceleration without gravity)
 * - `"true_velocity"`            : 3D vector (world-frame linear velocity)
 * - `"true_angular_velocity"`    : 3D vector (world-frame angular velocity)
 * - `"true_angular_acceleration"`: 3D vector (world-frame angular acceleration)
 * - `"pose"`                      : Pose3 (ground-truth pose at the measurement timestamp)
 */
class IMUScenarioSimulator : public ScenarioSimulator {
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
        double epsilon = 1e-8
    );

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
    Trajectory trajectory_;
    DifferentiationMethod diff_method_;
    Vector3Seq lever_arm_history_;

    double epsilon_;

    double safeDiv(double numerator, double denominator, double epsilon);
    gtsam::Vector3 safeDiv(const gtsam::Vector3& v, double denominator, double epsilon);
};

} // namespace simulation
} // namespace gtsam
