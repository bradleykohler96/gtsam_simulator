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

IMUScenarioSimulator::IMUScenarioSimulator(
    const TrajectoryModel& model,
    const std::vector<double>& timestamps,
    DifferentiationMethod method,
    std::function<gtsam::Vector3(double)> lever_arm_func,
    double epsilon)
: trajectory_model_(model),
  timestamps_(timestamps),
  diff_method_(method),
  lever_arm_func_(lever_arm_func),
  epsilon_(epsilon)
{}

IMUScenarioSimulator::TimedSensorData IMUScenarioSimulator::simulateScenario()
{
    if (!trajectory_.empty())
    {
        return simulateDiscrete();
    }
    else if (!timestamps_.empty() && trajectory_model_.pose)
    {
        return simulateContinuous();
    }
    else
    {
        return TimedSensorData{};
    }
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

gtsam::Vector3 IMUScenarioSimulator::differentiateTranslation(
    const gtsam::Pose3& pose_prev,
    const gtsam::Pose3& pose_curr,
    const gtsam::Pose3& pose_next,
    double t_prev,
    double t_curr,
    double t_next,
    bool has_prev,
    bool has_next)
{
    if (diff_method_ == DifferentiationMethod::Central && has_prev && has_next)
    {
        return computeCentralDiffVector(
            pose_prev.translation(),
            pose_curr.translation(),
            pose_next.translation(),
            t_prev, t_curr, t_next);
    }
    else if (diff_method_ == DifferentiationMethod::Forward || (!has_prev && has_next))
    {
        return computeForwardDiffVector(
            pose_curr.translation(),
            pose_next.translation(),
            t_curr, t_next);
    }
    else if (diff_method_ == DifferentiationMethod::Backward || (has_prev && !has_next))
    {
        return computeBackwardDiffVector(
            pose_prev.translation(),
            pose_curr.translation(),
            t_prev, t_curr);
    }
    return gtsam::Vector3::Zero();
}

gtsam::Vector3 IMUScenarioSimulator::differentiateRotation(
    const gtsam::Pose3& pose_prev,
    const gtsam::Pose3& pose_curr,
    const gtsam::Pose3& pose_next,
    double t_prev,
    double t_curr,
    double t_next,
    bool has_prev,
    bool has_next)
{
    if (diff_method_ == DifferentiationMethod::Central && has_prev && has_next)
    {
        return computeCentralDiffSO3(
            pose_prev.rotation(),
            pose_curr.rotation(),
            pose_next.rotation(),
            t_prev, t_curr, t_next);
    }
    else if (diff_method_ == DifferentiationMethod::Forward || (!has_prev && has_next))
    {
        return computeForwardDiffSO3(
            pose_curr.rotation(),
            pose_next.rotation(),
            t_curr, t_next);
    }
    else if (diff_method_ == DifferentiationMethod::Backward || (has_prev && !has_next))
    {
        return computeBackwardDiffSO3(
            pose_prev.rotation(),
            pose_curr.rotation(),
            t_prev, t_curr);
    }
    return gtsam::Vector3::Zero();
}

gtsam::Vector3 IMUScenarioSimulator::differentiateVector(
    const gtsam::Vector3& v_prev,
    const gtsam::Vector3& v_curr,
    const gtsam::Vector3& v_next,
    double t_prev,
    double t_curr,
    double t_next,
    bool has_prev,
    bool has_next)
{
    if (diff_method_ == DifferentiationMethod::Central && has_prev && has_next)
    {
        return computeCentralDiffVector(v_prev, v_curr, v_next, t_prev, t_curr, t_next);
    }
    else if (diff_method_ == DifferentiationMethod::Forward || (!has_prev && has_next))
    {
        return computeForwardDiffVector(v_curr, v_next, t_curr, t_next);
    }
    else if (diff_method_ == DifferentiationMethod::Backward || (has_prev && !has_next))
    {
        return computeBackwardDiffVector(v_prev, v_curr, t_prev, t_curr);
    }
    return gtsam::Vector3::Zero();
}


IMUScenarioSimulator::TimedSensorData IMUScenarioSimulator::simulateDiscrete()
{
    TimedSensorData measurements;
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

        gtsam::Vector3 velocity    = differentiateTranslation(pose_prev, pose_curr, pose_next, t_prev, t_curr, t_next, has_prev, has_next);
        gtsam::Vector3 omega_body  = differentiateRotation(pose_prev, pose_curr, pose_next, t_prev, t_curr, t_next, has_prev, has_next);
        gtsam::Vector3 omega       = pose_curr.rotation() * omega_body;

        std::map<std::string, std::any> step;
        step["gyroscope"]             = omega_body;
        step["true_velocity"]         = velocity;
        step["true_angular_velocity"] = omega;
        step["pose"]                  = pose_curr;

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

        const gtsam::Pose3& pose_curr = *std::any_cast<gtsam::Pose3>(&it->second["pose"]);

        const gtsam::Vector3& v_prev = *std::any_cast<gtsam::Vector3>(&it_prev->second["true_velocity"]);
        const gtsam::Vector3& v_curr = *std::any_cast<gtsam::Vector3>(&it->second["true_velocity"]);
        const gtsam::Vector3& v_next = *std::any_cast<gtsam::Vector3>(&it_next->second["true_velocity"]);

        const gtsam::Vector3& omega_prev = *std::any_cast<gtsam::Vector3>(&it_prev->second["true_angular_velocity"]);
        const gtsam::Vector3& omega_curr = *std::any_cast<gtsam::Vector3>(&it->second["true_angular_velocity"]);
        const gtsam::Vector3& omega_next = *std::any_cast<gtsam::Vector3>(&it_next->second["true_angular_velocity"]);

        gtsam::Vector3 accel = differentiateVector(v_prev, v_curr, v_next, t_prev, t_curr, t_next, has_prev, has_next);
        gtsam::Vector3 alpha = differentiateVector(omega_prev, omega_curr, omega_next, t_prev, t_curr, t_next, has_prev, has_next);

        // Lever arm
        gtsam::Vector3 lever_arm;
        if (!lever_arm_history_.empty())
        {
            lever_arm = (lever_arm_history_.size() > idx ? lever_arm_history_[idx] : lever_arm_history_.front());
        }
        else
        {
            lever_arm = gtsam::Vector3::Zero();
        }

        gtsam::Vector3 accel_centripetal = omega_curr.cross(omega_curr.cross(lever_arm));
        gtsam::Vector3 accel_tangential  = alpha.cross(lever_arm);
        gtsam::Vector3 accel_rot         = accel_centripetal + accel_tangential;
        gtsam::Vector3 accel_body        = pose_curr.rotation().transpose() * (accel - gravity) + accel_rot;

        it->second["accelerometer"]             = accel_body;
        it->second["true_acceleration"]         = accel;
        it->second["true_angular_acceleration"] = alpha;
    }

    return measurements;
}

IMUScenarioSimulator::TimedSensorData IMUScenarioSimulator::simulateContinuous()
{
    TimedSensorData measurements;
    gtsam::Vector3 gravity(0, 0, -9.81);
    const size_t N = timestamps_.size();

    // First derivatives (velocity and angular velocity)
    for (size_t i = 0; i < N; ++i)
    {
        const bool has_prev = (i > 0);
        const bool has_next = (i + 1 < N);

        const double t_curr = timestamps_[i];
        const double t_prev = has_prev ? timestamps_[i - 1] : t_curr;
        const double t_next = has_next ? timestamps_[i + 1] : t_curr;

        const gtsam::Pose3 pose_curr = trajectory_model_.pose(t_curr);
        const gtsam::Pose3 pose_prev = has_prev ? trajectory_model_.pose(t_prev) : pose_curr;
        const gtsam::Pose3 pose_next = has_next ? trajectory_model_.pose(t_next) : pose_curr;

        gtsam::Vector3 velocity;
        if (trajectory_model_.velocity)
        {
            velocity = trajectory_model_.velocity(t_curr);
        }
        else
        {
            velocity = differentiateTranslation(pose_prev, pose_curr, pose_next, t_prev, t_curr, t_next, has_prev, has_next);
        }

        gtsam::Vector3 omega;
        gtsam::Vector3 omega_body;
        if (trajectory_model_.angularVelocity)
        {
            omega      = trajectory_model_.angularVelocity(t_curr);
            omega_body = pose_curr.rotation().transpose() * omega;
        }
        else
        {
            omega_body = differentiateRotation(pose_prev, pose_curr, pose_next, t_prev, t_curr, t_next, has_prev, has_next);
            omega      = pose_curr.rotation() * omega_body;
        }

        std::map<std::string, std::any> step;
        step["gyroscope"]             = omega_body;
        step["true_velocity"]         = velocity;
        step["true_angular_velocity"] = omega;
        step["pose"]                  = pose_curr;

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

        const gtsam::Pose3& pose_curr = *std::any_cast<gtsam::Pose3>(&it->second["pose"]);

        const gtsam::Vector3& v_prev = *std::any_cast<gtsam::Vector3>(&it_prev->second["true_velocity"]);
        const gtsam::Vector3& v_curr = *std::any_cast<gtsam::Vector3>(&it->second["true_velocity"]);
        const gtsam::Vector3& v_next = *std::any_cast<gtsam::Vector3>(&it_next->second["true_velocity"]);

        const gtsam::Vector3& omega_prev = *std::any_cast<gtsam::Vector3>(&it_prev->second["true_angular_velocity"]);
        const gtsam::Vector3& omega_curr = *std::any_cast<gtsam::Vector3>(&it->second["true_angular_velocity"]);
        const gtsam::Vector3& omega_next = *std::any_cast<gtsam::Vector3>(&it_next->second["true_angular_velocity"]);

        gtsam::Vector3 accel;
        if (trajectory_model_.acceleration)
        {
            accel = trajectory_model_.acceleration(t_curr);
        }
        else
        {
            accel = differentiateVector(v_prev, v_curr, v_next, t_prev, t_curr, t_next, has_prev, has_next);
        }

        gtsam::Vector3 alpha;
        gtsam::Vector3 alpha_body;
        if (trajectory_model_.angularAcceleration)
        {
            alpha      = trajectory_model_.angularAcceleration(t_curr);
            alpha_body = pose_curr.rotation().transpose() * alpha;
        }
        else
        {
            alpha_body = differentiateVector(omega_prev, omega_curr, omega_next, t_prev, t_curr, t_next, has_prev, has_next);
            alpha      = pose_curr.rotation() * alpha_body;
        }

        // Lever arm
        gtsam::Vector3 lever_arm = gtsam::Vector3::Zero();
        if (lever_arm_func_)
        {
            lever_arm = lever_arm_func_(t_curr);
        }
        else if (!lever_arm_history_.empty())
        {
            lever_arm = (lever_arm_history_.size() > idx ? lever_arm_history_[idx] : lever_arm_history_.front());
        }
        else
        {
            lever_arm = gtsam::Vector3::Zero();
        }

        gtsam::Vector3 accel_centripetal = omega_curr.cross(omega_curr.cross(lever_arm));
        gtsam::Vector3 accel_tangential  = alpha_body.cross(lever_arm);
        gtsam::Vector3 accel_rot         = accel_centripetal + accel_tangential;
        gtsam::Vector3 accel_body        = pose_curr.rotation().transpose() * (accel - gravity) + accel_rot;

        it->second["accelerometer"]             = accel_body;
        it->second["true_acceleration"]         = accel;
        it->second["true_angular_acceleration"] = alpha;
    }

    return measurements;
}

} // namespace simulation
} // namespace gtsam
