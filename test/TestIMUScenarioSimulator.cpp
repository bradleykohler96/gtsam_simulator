#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>
#include "gtsam/simulation/IMUScenarioSimulator.h"

using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Vector3;
using gtsam::simulation::IMUScenarioSimulator;

TEST(IMUScenarioSimulatorTest, BasicTrajectoryOutput) {
    double duration = 5.0;  // [s]
    double dt       = 0.1;  // [s]

    // Motion parameters
    double vx = 0.5;  // slow forward velocity [m/s]
    double wx = 0.5;  // angular velocity about x [rad/s]

    IMUScenarioSimulator::Trajectory trajectory;
    for (double t = 0.0; t <= duration + 1e-8; t += dt) {
        Vector3 trans(vx * t, 0.0, 0.0);
        Rot3 rot = Rot3::Rx(wx * t);
        trajectory[t] = Pose3(rot, trans);
    }

    IMUScenarioSimulator simulator(
        trajectory,
        IMUScenarioSimulator::DifferentiationMethod::Central
    );

    auto measurements = simulator.simulateScenario();

    // Test: same number of steps as trajectory
    EXPECT_EQ(measurements.size(), trajectory.size());

    for (const auto& [t, sensors] : measurements) {
        // Test: sensor keys exist
        EXPECT_TRUE(sensors.count("accelerometer"));
        EXPECT_TRUE(sensors.count("gyroscope"));
        EXPECT_TRUE(sensors.count("pose"));
        EXPECT_TRUE(sensors.count("true_velocity"));
        EXPECT_TRUE(sensors.count("true_angular_velocity"));

        // Test: linear velocity magnitude roughly matches expected vx
        Vector3 vel = std::any_cast<Vector3>(sensors.at("true_velocity"));
        double expected_vel_mag = vx;
        EXPECT_NEAR(vel.norm(), expected_vel_mag, 1e-2);

        // Test: angular velocity x-component matches expected wx
        Vector3 omega = std::any_cast<Vector3>(sensors.at("true_angular_velocity"));
        EXPECT_NEAR(omega.x(), wx, 1e-2);
        // Optional: y and z should be ~0
        EXPECT_NEAR(omega.y(), 0.0, 1e-2);
        EXPECT_NEAR(omega.z(), 0.0, 1e-2);

        // Test: accelerometer includes gravity magnitude
        Vector3 accel = std::any_cast<Vector3>(sensors.at("accelerometer"));
        double g_mag = accel.norm();
        EXPECT_GT(g_mag, 9.0);  // should be roughly near gravity (~9.81)
    }
}
