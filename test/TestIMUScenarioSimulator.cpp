// Standard library
#include <any>
#include <cmath>
#include <vector>

// Third-party
#include <gtest/gtest.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

// Local project headers
#include "gtsam/simulation/IMUScenarioSimulator.h"

using namespace gtsam;
using namespace gtsam::simulation;

static const double kTolPoseTrans     = 5e-4;  // for translations
static const double kTolPoseRot       = 1e-6;  // for rotations
static const double kTolVelocity      = 5e-2;  // for velocity
static const double kTolAcceleration  = 6e-1;  // for acceleration
static const double kTolAngularVel    = 1e-6;  // for angular velocity
static const double kTolAngularAccel  = 1e-6;  // for angular acceleration

TEST(IMUScenarioSimulator, ContinuousVsDiscreteHelix) {
    // 1) Build continuous trajectory model (analytic)
    IMUScenarioSimulator::TrajectoryModel model;

    model.pose = [](double t) -> Pose3 {
        // world position
        Point3 p(t, std::cos(t), std::sin(t));

        // body axes as columns expressed in world frame
        Vector3 ex(0.0, -std::cos(t), -std::sin(t)); // radial inward (unit)
        Vector3 ez(1.0, -std::sin(t), std::cos(t));  // tangent (not unit)
        ez = ez / std::sqrt(2.0);                    // normalize -> unit tangent
        Vector3 ey = ez.cross(ex);
        ey = ey / ey.norm();

        // assemble rotation matrix columns = [ex ey ez]
        Matrix3 mat;
        mat.col(0) = ex;
        mat.col(1) = ey;
        mat.col(2) = ez;

        Rot3 R(mat);
        return Pose3(R, p);
    };

    model.velocity = [](double t) -> Vector3 {
        return Vector3(1.0, -std::sin(t), std::cos(t));
    };

    model.acceleration = [](double t) -> Vector3 {
        return Vector3(0.0, -std::cos(t), -std::sin(t));
    };

    model.angularVelocity = [](double /*t*/) -> Vector3 {
        return Vector3(1.0, 0.0, 0.0); // world-x constant
    };

    model.angularAcceleration = [](double /*t*/) -> Vector3 {
        return Vector3(0.0, 0.0, 0.0);
    };

    // 2) timestamps
    std::vector<double> timestamps;
    const double dt = 0.1;
    for (double t = 0.0; t <= 10.0 + 1e-12; t += dt) timestamps.push_back(t);

    // 3) run continuous simulator
    IMUScenarioSimulator sim_cont(model, timestamps);
    auto cont_results = sim_cont.simulateScenario();

    // 4) build discrete trajectory from analytic poses and run discrete simulator
    IMUScenarioSimulator::Trajectory discrete_traj;
    for (double t : timestamps) discrete_traj[t] = model.pose(t);

    IMUScenarioSimulator sim_disc(discrete_traj);
    auto disc_results = sim_disc.simulateScenario();

    // 5) compare entries
    for (double t : timestamps) {
        ASSERT_TRUE(cont_results.count(t)) << "continuous result missing at t=" << t;
        ASSERT_TRUE(disc_results.count(t)) << "discrete result missing at t=" << t;

        const auto& cont = cont_results.at(t);
        const auto& disc = disc_results.at(t);

        // velocity
        ASSERT_TRUE(cont.count("true_velocity"));
        ASSERT_TRUE(disc.count("true_velocity"));
        Vector3 v_cont = std::any_cast<Vector3>(cont.at("true_velocity"));
        Vector3 v_disc = std::any_cast<Vector3>(disc.at("true_velocity"));
        EXPECT_NEAR((v_cont - v_disc).norm(), 0.0, kTolVelocity) << "velocity mismatch at t=" << t;

        // acceleration
        ASSERT_TRUE(cont.count("true_acceleration"));
        ASSERT_TRUE(disc.count("true_acceleration"));
        Vector3 a_cont = std::any_cast<Vector3>(cont.at("true_acceleration"));
        Vector3 a_disc = std::any_cast<Vector3>(disc.at("true_acceleration"));
        EXPECT_NEAR((a_cont - a_disc).norm(), 0.0, kTolAcceleration) << "acceleration mismatch at t=" << t;

        // angular velocity
        ASSERT_TRUE(cont.count("true_angular_velocity"));
        ASSERT_TRUE(disc.count("true_angular_velocity"));
        Vector3 w_cont = std::any_cast<Vector3>(cont.at("true_angular_velocity"));
        Vector3 w_disc = std::any_cast<Vector3>(disc.at("true_angular_velocity"));
        EXPECT_NEAR((w_cont - w_disc).norm(), 0.0, kTolAngularVel) << "angular velocity mismatch at t=" << t;

        // angular acceleration
        ASSERT_TRUE(cont.count("true_angular_acceleration"));
        ASSERT_TRUE(disc.count("true_angular_acceleration"));
        Vector3 alpha_cont = std::any_cast<Vector3>(cont.at("true_angular_acceleration"));
        Vector3 alpha_disc = std::any_cast<Vector3>(disc.at("true_angular_acceleration"));
        EXPECT_NEAR((alpha_cont - alpha_disc).norm(), 0.0, kTolAngularAccel) << "angular acceleration mismatch at t=" << t;

        // pose translation
        ASSERT_TRUE(cont.count("pose"));
        ASSERT_TRUE(disc.count("pose"));
        Pose3 pose_cont = std::any_cast<Pose3>(cont.at("pose"));
        Pose3 pose_disc = std::any_cast<Pose3>(disc.at("pose"));
        EXPECT_NEAR((pose_cont.translation() - pose_disc.translation()).norm(), 0.0, kTolPoseTrans)
            << "position mismatch at t=" << t;

        // pose rotation: compare Rot3 between the two rotations
        Rot3 Rcont = pose_cont.rotation();
        Rot3 Rdisc = pose_disc.rotation();
        Rot3 Rdiff = Rcont.between(Rdisc); // Rot3 between Rot3
        Vector3 rpy = Rdiff.rpy();         // small rpy expected
        EXPECT_NEAR(rpy.norm(), 0.0, kTolPoseRot) << "rotation mismatch at t=" << t;

        // optional: compare body-frame sensor outputs if present
        if (cont.count("gyroscope") && disc.count("gyroscope")) {
            Vector3 gy_cont = std::any_cast<Vector3>(cont.at("gyroscope"));
            Vector3 gy_disc = std::any_cast<Vector3>(disc.at("gyroscope"));
            EXPECT_NEAR((gy_cont - gy_disc).norm(), 0.0, kTolAngularVel) << "gyro mismatch at t=" << t;
        }
        if (cont.count("accelerometer") && disc.count("accelerometer")) {
            Vector3 acc_cont = std::any_cast<Vector3>(cont.at("accelerometer"));
            Vector3 acc_disc = std::any_cast<Vector3>(disc.at("accelerometer"));
            EXPECT_NEAR((acc_cont - acc_disc).norm(), 0.0, kTolAcceleration) << "accelerometer(meas) mismatch at t=" << t;
            // note: accelerometer measurement comparison may need a looser tol depending on gravity/lever-arm handling
        }
    }
}
