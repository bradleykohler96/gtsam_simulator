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
    double epsilon)
: trajectory_(trajectory),
  diff_method_(method),
  lever_arm_history_(lever_arm_history),
  epsilon_(epsilon)
{}

IMUScenarioSimulator::TimedSensorData IMUScenarioSimulator::simulateScenario()
{
    std::map<double, std::map<std::string, std::any>> measurements;
    gtsam::Vector3 gravity(0, 0, -9.81);

    // First derivatives (velocity and angular velocity)
    for (auto it = trajectory_.begin(); it != trajectory_.end(); ++it)
    {
        const auto it_prev = (it == trajectory_.begin()) ? it : std::prev(it);
        const auto it_next = (it == std::prev(trajectory_.end())) ? it : std::next(it);

        const bool has_prev = (it_prev != it);
        const bool has_next = (it_next != it);

        const double t_prev = it_prev->first;
        const double t_curr = it->first;
        const double t_next = it_next->first;

        const gtsam::Pose3& pose_prev = it_prev->second;
        const gtsam::Pose3& pose_curr = it->second;
        const gtsam::Pose3& pose_next = it_next->second;

        gtsam::Vector3 velocity(0,0,0);
        gtsam::Vector3 omega(0,0,0);

        if (diff_method_ == DifferentiationMethod::Central && has_prev && has_next)
        {
            velocity = computeCentralDiffVector(
                pose_prev.translation(),
                pose_curr.translation(),
                pose_next.translation(),
                t_prev,
                t_curr,
                t_next
            );
            omega = computeCentralDiffSO3(
                pose_prev.rotation(),
                pose_curr.rotation(),
                pose_next.rotation(),
                t_prev,
                t_curr,
                t_next
            );
        }
        else if (diff_method_ == DifferentiationMethod::Forward || (!has_prev && has_next))
        {
            velocity = computeForwardDiffVector(
                pose_curr.translation(),
                pose_next.translation(),
                t_curr,
                t_next
            );
            omega = computeForwardDiffSO3(
                pose_curr.rotation(),
                pose_next.rotation(),
                t_curr,
                t_next
            );
        }
        else if (diff_method_ == DifferentiationMethod::Backward || (has_prev && !has_next))
        {
            velocity = computeBackwardDiffVector(
                pose_prev.translation(),
                pose_curr.translation(),
                t_prev,
                t_curr
            );
            omega = computeBackwardDiffSO3(
                pose_prev.rotation(),
                pose_curr.rotation(),
                t_prev,
                t_curr
            );
        }

        std::map<std::string, std::any> step;
        step["gyroscope"] = omega;
        step["true_velocity"] = velocity;
        step["true_angular_velocity"] = omega;
        step["pose"] = pose_curr;

        measurements[t_curr] = std::move(step);
    }

    // Second derivatives (acceleration and angular acceleration)
    size_t idx = 0;
    for (auto it = measurements.begin(); it != measurements.end(); ++it, ++idx)
    {
        const auto it_prev = (it == measurements.begin()) ? it : std::prev(it);
        const auto it_next = (std::next(it) == measurements.end()) ? it : std::next(it);

        const bool has_prev = (it_prev != it);
        const bool has_next = (it_next != it);

        const double t_prev = it_prev->first;
        const double t_curr = it->first;
        const double t_next = it_next->first;

        const gtsam::Vector3 v_prev = std::any_cast<gtsam::Vector3>(it_prev->second.at("true_velocity"));
        const gtsam::Vector3 v_curr = std::any_cast<gtsam::Vector3>(it->second.at("true_velocity"));
        const gtsam::Vector3 v_next = std::any_cast<gtsam::Vector3>(it_next->second.at("true_velocity"));

        const gtsam::Vector3 omega_prev = std::any_cast<gtsam::Vector3>(it_prev->second.at("true_angular_velocity"));
        const gtsam::Vector3 omega_curr = std::any_cast<gtsam::Vector3>(it->second.at("true_angular_velocity"));
        const gtsam::Vector3 omega_next = std::any_cast<gtsam::Vector3>(it_next->second.at("true_angular_velocity"));

        gtsam::Vector3 accel(0,0,0);
        gtsam::Vector3 alpha(0,0,0);

        if (diff_method_ == DifferentiationMethod::Central && has_prev && has_next)
        {
            accel = computeCentralDiffVector(v_prev, v_curr, v_next, t_prev, t_curr, t_next);
            alpha = computeCentralDiffVector(omega_prev, omega_curr, omega_next, t_prev, t_curr, t_next);
        }
        else if (diff_method_ == DifferentiationMethod::Forward || (!has_prev && has_next))
        {
            accel = computeForwardDiffVector(v_curr, v_next, t_curr, t_next);
            alpha = computeForwardDiffVector(omega_curr, omega_next, t_curr, t_next);
        }
        else if (diff_method_ == DifferentiationMethod::Backward || (has_prev && !has_next))
        {
            accel = computeBackwardDiffVector(v_prev, v_curr, t_prev, t_curr);
            alpha = computeBackwardDiffVector(omega_prev, omega_curr, t_prev, t_curr);
        }

        // Lever arm
        gtsam::Vector3 lever_arm = lever_arm_history_.empty() ? gtsam::Vector3::Zero()
            : (lever_arm_history_.size() > idx ? lever_arm_history_[idx] : lever_arm_history_.front());

        gtsam::Vector3 accel_centripetal = omega_curr.cross(omega_curr.cross(lever_arm));
        gtsam::Vector3 accel_tangential = alpha.cross(lever_arm);
        gtsam::Vector3 accel_rot = accel_centripetal + accel_tangential;

        const gtsam::Pose3& pose_curr = std::any_cast<gtsam::Pose3>(it->second.at("pose"));
        gtsam::Vector3 accel_body = pose_curr.rotation().transpose() * (accel - gravity) + accel_rot;

        it->second["accelerometer"] = accel_body;
        it->second["true_acceleration"] = accel;
        it->second["true_angular_acceleration"] = alpha;
    }

    return measurements;
}

double IMUScenarioSimulator::safeDiv(
    double numerator,
    double denominator)
{
    if (std::abs(denominator) < epsilon_)
    {
        return 0.0;
    }
    return numerator / denominator;
}

gtsam::Vector3 IMUScenarioSimulator::safeDiv(
    const gtsam::Vector3& v,
    double denominator)
{
    if (std::abs(denominator) < epsilon_)
    {
        return gtsam::Vector3::Zero();
    }
    return v / denominator;
}

gtsam::Vector3 IMUScenarioSimulator::computeCentralDiffVector(
    const gtsam::Vector3& v_prev,
    const gtsam::Vector3& v_curr,
    const gtsam::Vector3& v_next,
    double t_prev,
    double t_curr,
    double t_next)
{
    double h1 = std::max(t_curr - t_prev, epsilon_);
    double h2 = std::max(t_next - t_curr, epsilon_);
    double h_total = h1 + h2;

    return (-h2 / (h1 * h_total)) * v_prev
           + ((h2 - h1) / (h1 * h2)) * v_curr
           + (h1 / (h2 * h_total)) * v_next;
}

gtsam::Vector3 IMUScenarioSimulator::computeForwardDiffVector(
    const gtsam::Vector3& v_curr,
    const gtsam::Vector3& v_next,
    double t_curr,
    double t_next)
{
    double dt = std::max(t_next - t_curr, epsilon_);
    return (v_next - v_curr) / dt;
}

gtsam::Vector3 IMUScenarioSimulator::computeBackwardDiffVector(
    const gtsam::Vector3& v_prev,
    const gtsam::Vector3& v_curr,
    double t_prev,
    double t_curr)
{
    double dt = std::max(t_curr - t_prev, epsilon_);
    return (v_curr - v_prev) / dt;
}

gtsam::Vector3 IMUScenarioSimulator::computeCentralDiffSO3(
    const gtsam::Rot3& R_prev,
    const gtsam::Rot3& R_curr,
    const gtsam::Rot3& R_next,
    double t_prev,
    double t_curr,
    double t_next)
{
    double h1 = std::max(t_curr - t_prev, epsilon_);
    double h2 = std::max(t_next - t_curr, epsilon_);
    double h_total = h1 + h2;

    gtsam::Vector3 omega_back = gtsam::Rot3::Logmap(R_prev.inverse() * R_curr) / h1;
    gtsam::Vector3 omega_fwd  = gtsam::Rot3::Logmap(R_curr.inverse() * R_next) / h2;

    return (h2 / h_total) * omega_back + (h1 / h_total) * omega_fwd;
}

gtsam::Vector3 IMUScenarioSimulator::computeForwardDiffSO3(
    const gtsam::Rot3& R_curr,
    const gtsam::Rot3& R_next,
    double t_curr,
    double t_next)
{
    double dt = std::max(t_next - t_curr, epsilon_);
    return gtsam::Rot3::Logmap(R_curr.inverse() * R_next) / dt;
}

gtsam::Vector3 IMUScenarioSimulator::computeBackwardDiffSO3(
    const gtsam::Rot3& R_prev,
    const gtsam::Rot3& R_curr,
    double t_prev,
    double t_curr)
{
    double dt = std::max(t_curr - t_prev, epsilon_);
    return gtsam::Rot3::Logmap(R_prev.inverse() * R_curr) / dt;
}

} // namespace simulation
} // namespace gtsam
