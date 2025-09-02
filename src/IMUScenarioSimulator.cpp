// Standard library
#include <any>
#include <functional>
#include <map>
#include <vector>

// Third-party
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>

// Local project headers
#include "gtsam/simulation/ScenarioSimulator.h"
#include "gtsam/simulation/IMUScenarioSimulator.h"

namespace gtsam {
namespace simulation {

IMUScenarioSimulator::IMUScenarioSimulator(
    const Trajectory& trajectory,
    DifferentiationMethod method,
    const Vector3Seq& lever_arm_history,
    double epsilon
)
: trajectory_(trajectory),
  diff_method_(method),
  lever_arm_history_(lever_arm_history),
  epsilon_(epsilon)
{}

IMUScenarioSimulator::TimedSensorData
IMUScenarioSimulator::simulateScenario() {

    std::map<double, std::map<std::string, std::any>> measurements;
    gtsam::Vector3 gravity(0, 0, -9.81);

    // First derivatives (velocity and angular velocity)
    for (auto it = trajectory_.begin(); it != trajectory_.end(); ++it) {
        gtsam::Vector3 velocity(0, 0, 0);
        gtsam::Vector3 omega(0, 0, 0);

        const auto it_prev = (it == trajectory_.begin()) ? it : std::prev(it);
        const auto it_next = (it == std::prev(trajectory_.end())) ? it : std::next(it);

        const bool has_prev = (it_prev != it);
        const bool has_next = (it_next != it);

        const double t_prev = it_prev->first;
        const double t_curr = it->first;
        const double t_next = it_next->first;

        const double h_1 = t_curr - t_prev;
        const double h_2 = t_next - t_curr;
        const double h_total = h_1 + h_2;

        const gtsam::Pose3& pose_prev = it_prev->second;
        const gtsam::Pose3& pose_curr = it->second;
        const gtsam::Pose3& pose_next = it_next->second;

        const gtsam::Vector3 x_prev = pose_prev.translation();
        const gtsam::Vector3 x_curr = pose_curr.translation();
        const gtsam::Vector3 x_next = pose_next.translation();

        const gtsam::Rot3 R_prev = pose_prev.rotation();
        const gtsam::Rot3 R_next = pose_next.rotation();

        const gtsam::Vector3 omega_back = safeDiv(gtsam::Rot3::Logmap(R_prev.inverse() * pose_curr.rotation()), h_1, epsilon_);
        const gtsam::Vector3 omega_fwd  = safeDiv(gtsam::Rot3::Logmap(pose_curr.rotation().inverse() * R_next), h_2, epsilon_);

        if (diff_method_ == DifferentiationMethod::Central && (has_prev && has_next)) {
            // Central difference velocity (non-uniform step sizes)
            velocity = safeDiv(-h_2, h_1 * h_total, epsilon_)  * x_prev
                     + safeDiv(h_2 - h_1, h_1 * h_2, epsilon_) * x_curr
                     + safeDiv(h_1, h_2 * h_total, epsilon_)   * x_next;

            // Central angular velocity (non-uniform step sizes)
            omega = safeDiv(h_2, h_total, epsilon_) * omega_back + safeDiv(h_1, h_total, epsilon_) * omega_fwd;
        } else if (diff_method_ == DifferentiationMethod::Forward  || (!has_prev && has_next)) {
            // Forward difference velocity
            velocity = safeDiv(x_next - x_curr, h_2, epsilon_);

            // Forward angular velocity
            omega = omega_fwd;
        } else if (diff_method_ == DifferentiationMethod::Backward || (has_prev && !has_next)) {
            // Backward difference velocity
            velocity = safeDiv(x_curr - x_prev, h_1, epsilon_);

            // Backward angular velocity
            omega = omega_back;
        }

        std::map<std::string, std::any> step;
        step["gyroscope"] = omega;
        step["true_velocity"] = velocity;
        step["true_angular_velocity"] = omega;
        step["pose"] = pose_curr;

        measurements[t_curr] = std::move(step);
    }

    // Second derivatives (acceleration and angular acceleration)
    size_t idx = 0; // index for lever_arm_history_ sequence
    for (auto it = measurements.begin(); it != measurements.end(); ++it, ++idx) {
        gtsam::Vector3 accel(0, 0, 0);
        gtsam::Vector3 alpha(0,0,0);

        const auto it_prev = (it == measurements.begin()) ? it : std::prev(it);
        const auto it_next = (std::next(it) == measurements.end()) ? it : std::next(it);

        const bool has_prev = (it_prev != it);
        const bool has_next = (it_next != it);

        const double t_prev = it_prev->first;
        const double t_curr = it->first;
        const double t_next = it_next->first;

        const double h_1 = t_curr - t_prev;
        const double h_2 = t_next - t_curr;
        const double h_total = h_1 + h_2;

        const gtsam::Vector3 v_prev = std::any_cast<gtsam::Vector3>(it_prev->second.at("true_velocity"));
        const gtsam::Vector3 v_curr = std::any_cast<gtsam::Vector3>(it->second.at("true_velocity"));
        const gtsam::Vector3 v_next = std::any_cast<gtsam::Vector3>(it_next->second.at("true_velocity"));

        const gtsam::Vector3 omega_prev = std::any_cast<gtsam::Vector3>(it_prev->second.at("true_angular_velocity"));
        const gtsam::Vector3 omega_curr = std::any_cast<gtsam::Vector3>(it->second.at("true_angular_velocity"));
        const gtsam::Vector3 omega_next = std::any_cast<gtsam::Vector3>(it_next->second.at("true_angular_velocity"));

        if (diff_method_ == DifferentiationMethod::Central && (has_prev && has_next)) {
            // Central difference acceleration (non-uniform step sizes)
            accel = safeDiv(-h_2, h_1 * h_total, epsilon_)  * v_prev
                  + safeDiv(h_2 - h_1, h_1 * h_2, epsilon_) * v_curr
                  + safeDiv(h_1, h_2 * h_total, epsilon_)   * v_next;

            // Central difference angular acceleration (non-uniform step sizes)
            alpha = safeDiv(-h_2, h_1 * h_total, epsilon_)  * omega_prev
                  + safeDiv(h_2 - h_1, h_1 * h_2, epsilon_) * omega_curr
                  + safeDiv(h_1, h_2 * h_total, epsilon_)   * omega_next;
        } else if (diff_method_ == DifferentiationMethod::Forward || (!has_prev && has_next)) {
            // Forward difference acceleration
            accel = safeDiv(v_next - v_curr, h_2, epsilon_);

            // Forward difference angular acceleration
            alpha = safeDiv(omega_next - omega_curr, h_2, epsilon_);
        } else if (diff_method_ == DifferentiationMethod::Backward || (has_prev && !has_next)) {
            // Backward difference acceleration
            accel = safeDiv(v_curr - v_prev, h_1, epsilon_);

            // Backward difference angular acceleration
            alpha = safeDiv(omega_curr - omega_prev, h_1, epsilon_);
        }

        // Lever arm for this timestep
        gtsam::Vector3 lever_arm = lever_arm_history_.empty() ? gtsam::Vector3::Zero() : (lever_arm_history_.size() > idx ? lever_arm_history_[idx] : lever_arm_history_.front());

        // Rotational accelerations
        gtsam::Vector3 accel_centripetal = omega_curr.cross(omega_curr.cross(lever_arm));
        gtsam::Vector3 accel_tangential = alpha.cross(lever_arm);
        gtsam::Vector3 accel_rot = accel_centripetal + accel_tangential;

        // Body frame accelerations
        const gtsam::Pose3& pose_curr = std::any_cast<gtsam::Pose3>(it->second.at("pose"));
        gtsam::Vector3 accel_body = pose_curr.rotation().transpose() * (accel - gravity) + accel_rot;

        it->second["accelerometer"] = accel_body;
        it->second["true_acceleration"] = accel;
        it->second["true_angular_acceleration"] = alpha;
    }

    return measurements;
}

double IMUScenarioSimulator::safeDiv(double numerator, double denominator, double epsilon) {
    if (std::abs(denominator) < epsilon) {
        return 0.0;
    }
    return numerator / denominator;
}

gtsam::Vector3 IMUScenarioSimulator::safeDiv(const gtsam::Vector3& v, double denominator, double epsilon) {
    if (std::abs(denominator) < epsilon) {
        return gtsam::Vector3::Zero();
    }
    return v / denominator;
}

} // namespace simulation
} // namespace gtsam
