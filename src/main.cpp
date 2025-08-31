// Standard library
#include <any>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

// Third-party
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>

// Local project headers
#include "gtsam/simulation/ScenarioSimulator.h"
#include "gtsam/simulation/IMUScenarioSimulator.h"

using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector3;
using gtsam::simulation::IMUScenarioSimulator;

int main() {
    double duration = 5.0;  // [s]
    double dt       = 0.1;  // [s]

    // Motion parameters
    double vx = 0.5;  // slow forward velocity [m/s]
    double wx = 0.5;  // angular velocity about x [rad/s]

    IMUScenarioSimulator::Trajectory trajectory;
    for (double t = 0.0; t <= duration + 1e-8; t += dt) {
        // Translate forward along X
        Vector3 trans(vx * t, 0.0, 0.0);

        // Rotate about X axis, so gravity changes direction in the body frame
        Rot3 rot = Rot3::Rx(wx * t);

        trajectory[t] = Pose3(rot, trans);
    }

    // Construct simulator with discrete trajectory
    IMUScenarioSimulator simulator(
        trajectory,
        IMUScenarioSimulator::DifferentiationMethod::Central
    );

    // Run simulation
    auto measurements = simulator.simulateScenario();

    std::cout << std::fixed << std::setprecision(3);

    for (const auto& [t, sensors] : measurements) {
        auto accel = std::any_cast<Vector3>(sensors.at("accelerometer"));
        auto gyro  = std::any_cast<Vector3>(sensors.at("gyroscope"));

        std::cout << "t = " << t << " s\n";
        std::cout << "  accel_body = [" << accel.transpose() << "]\n";
        std::cout << "  omega_body = [" << gyro.transpose() << "]\n";

        if (sensors.count("true_accel")) {
            auto true_accel = std::any_cast<Vector3>(sensors.at("true_accel"));
            auto true_vel   = std::any_cast<Vector3>(sensors.at("true_velocity"));
            auto true_omega = std::any_cast<Vector3>(sensors.at("true_omega"));

            std::cout << "  true_accel = [" << true_accel.transpose() << "]\n";
            std::cout << "  true_vel   = [" << true_vel.transpose() << "]\n";
            std::cout << "  true_omega = [" << true_omega.transpose() << "]\n";
        }

        std::cout << "-----------------------------------\n";
    }

    return 0;
}
