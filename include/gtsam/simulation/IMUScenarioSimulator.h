#pragma once

// Standard library
#include <any>
#include <functional>
#include <map>
#include <string>
#include <vector>

// Third-party
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>

// Local project headers
#include "ScenarioSimulator.h"

namespace gtsam {
namespace simulation {

/**
 * @class IMUScenarioSimulator
 * @brief Simulates IMU measurements (accelerometer & gyroscope) from a user-defined trajectory.
 *
 * The simulator accepts a trajectory function that outputs a Pose3 at time @f$t@f$, and computes:
 *
 * - Linear velocity (non-uniform weighted finite differences):  
 *   For times @f$t_{i-1}, t_i, t_{i+1}@f$ with step sizes  
 *   @f$h_1 = t_i - t_{i-1}@f$, @f$h_2 = t_{i+1} - t_i@f$:  
 *   @f[
 *      v(t_i) \approx 
 *          \frac{-h_2}{h_1 (h_1 + h_2)} p(t_{i-1})
 *        + \frac{h_2 - h_1}{h_1 h_2} p(t_i)
 *        + \frac{h_1}{h_2 (h_1 + h_2)} p(t_{i+1})
 *   @f]
 *   - If only @f$p(t_{i-1}), p(t_i)@f$ are available (start):  
 *     @f[
 *        v(t_i) \approx \frac{p(t_i) - p(t_{i-1})}{h_1}
 *     @f]
 *   - If only @f$p(t_i), p(t_{i+1})@f$ are available (end):  
 *     @f[
 *        v(t_i) \approx \frac{p(t_{i+1}) - p(t_i)}{h_2}
 *     @f]
 *
 * - Linear acceleration (non-uniform weighted finite differences):  
 *   Using velocities at @f$t_{i-1}, t_i, t_{i+1}@f$:  
 *   @f[
 *      a(t_i) \approx 
 *          \frac{-h_2}{h_1 (h_1 + h_2)} v(t_{i-1})
 *        + \frac{h_2 - h_1}{h_1 h_2} v(t_i)
 *        + \frac{h_1}{h_2 (h_1 + h_2)} v(t_{i+1})
 *   @f]
 *   with analogous one-sided formulas at the trajectory boundaries.
 *
 * - Angular velocity (gyroscope):  
 *   Using the rotation logmap of relative rotations with non-uniform weights:  
 *   @f[
 *      \omega(t_i) \approx 
 *          \frac{-h_2}{h_1 (h_1 + h_2)} \, \log\!\left(R_{i-1}^\top R_i\right)
 *        + \frac{h_2 - h_1}{h_1 h_2} \, \log\!\left(R_i^\top R_i\right) \; (=0)
 *        + \frac{h_1}{h_2 (h_1 + h_2)} \, \log\!\left(R_i^\top R_{i+1}\right)
 *   @f]
 *   (which simplifies in practice to backward, forward, or central depending on data available).
 *
 * - Accelerometer output includes gravity:  
 *   @f[
 *       f_\text{meas}(t) = R(t)^\top \left(a(t) - g\right)
 *   @f]
 *   where @f$g = (0, 0, -9.81)^T@f$.
 *
 * The simulator returns a vector of measurement maps. Each map contains:
 * - `"accelerometer"` : 3D vector (body-frame acceleration including gravity)
 * - `"gyroscope"`     : 3D vector (body-frame angular velocity)
 * - `"true_accel"`    : 3D vector (world-frame acceleration without gravity)
 * - `"true_velocity"` : 3D vector (world-frame linear velocity)
 * - `"true_omega"`    : 3D vector (world-frame angular velocity)
 * - `"pose"`          : Pose3 (ground-truth pose at the measurement timestamp)
 */
class IMUScenarioSimulator : public ScenarioSimulator {
public:

    /// Differentiation methods for numerical derivatives
    enum class DifferentiationMethod { Central, Forward, Backward };

    /// User-provided trajectory: time → Pose3 (kept sorted automatically by std::map)
    using Trajectory = std::map<double, gtsam::Pose3>;

    /**
     * @brief Construct an IMU scenario simulator from a trajectory.
     * @param trajectory User-provided trajectory as a sorted map of timestamps to poses
     * @param method Differentiation method (Central by default)
     */
    IMUScenarioSimulator(Trajectory trajectory,
                         DifferentiationMethod method = DifferentiationMethod::Central,
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
    std::map<double, std::map<std::string, std::any>> simulateScenario() override;

private:
    Trajectory trajectory_;
    DifferentiationMethod diff_method_;
    double epsilon_;
};

} // namespace simulation
} // namespace gtsam
