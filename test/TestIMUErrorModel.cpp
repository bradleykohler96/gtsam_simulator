#include <gtest/gtest.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include "gtsam/simulation/IMUErrorModel.h"

using gtsam::Vector3;
using gtsam::Matrix33;
using gtsam::simulation::IMUErrorModel;

TEST(IMUErrorModelTest, DefaultConstruction) {
    IMUErrorModel model;

    // Basic check: first corruption should return exactly bias (no noise)
    IMUErrorModel::TimedSensorData data;
    data[0.0]["gyroscope"] = Vector3(1.0, 2.0, 3.0);
    data[0.0]["accelerometer"] = Vector3(4.0, 5.0, 6.0);

    model.corruptGyro(data);
    model.corruptAccel(data);

    Vector3 gyro_meas = std::any_cast<Vector3>(data[0.0]["gyroscope"]);
    Vector3 accel_meas = std::any_cast<Vector3>(data[0.0]["accelerometer"]);

    // With default model, bias is zero, calibration identity -> measurements unchanged
    EXPECT_NEAR(gyro_meas.x(), 1.0, 1e-8);
    EXPECT_NEAR(gyro_meas.y(), 2.0, 1e-8);
    EXPECT_NEAR(gyro_meas.z(), 3.0, 1e-8);

    EXPECT_NEAR(accel_meas.x(), 4.0, 1e-8);
    EXPECT_NEAR(accel_meas.y(), 5.0, 1e-8);
    EXPECT_NEAR(accel_meas.z(), 6.0, 1e-8);
}

TEST(IMUErrorModelTest, BiasAndNoise) {
    Vector3 bias_gyro(0.1, 0.2, 0.3);
    Vector3 bias_accel(-0.1, 0.0, 0.05);
    Vector3 sigma_gyro(0.0, 0.0, 0.0);   // no measurement noise
    Vector3 sigma_accel(0.0, 0.0, 0.0);

    IMUErrorModel model(
        Matrix33::Identity(),
        Matrix33::Identity(),
        bias_gyro,
        bias_accel,
        Vector3::Zero(),
        Vector3::Zero(),
        sigma_gyro,
        sigma_accel,
        42
    );

    IMUErrorModel::TimedSensorData data;
    data[0.0]["gyroscope"] = Vector3(1.0, 2.0, 3.0);
    data[0.0]["accelerometer"] = Vector3(4.0, 5.0, 6.0);

    model.corruptGyro(data);
    model.corruptAccel(data);

    Vector3 gyro_meas = std::any_cast<Vector3>(data[0.0]["gyroscope"]);
    Vector3 accel_meas = std::any_cast<Vector3>(data[0.0]["accelerometer"]);

    // Bias applied, no noise
    EXPECT_NEAR(gyro_meas.x(), 1.0 + 0.1, 1e-8);
    EXPECT_NEAR(gyro_meas.y(), 2.0 + 0.2, 1e-8);
    EXPECT_NEAR(gyro_meas.z(), 3.0 + 0.3, 1e-8);

    EXPECT_NEAR(accel_meas.x(), 4.0 - 0.1, 1e-8);
    EXPECT_NEAR(accel_meas.y(), 5.0 + 0.0, 1e-8);
    EXPECT_NEAR(accel_meas.z(), 6.0 + 0.05, 1e-8);
}

TEST(IMUErrorModelTest, MultipleStepsRandomWalk) {
    Vector3 bias_rw_gyro(0.01, 0.01, 0.01);
    Vector3 bias_rw_accel(0.02, 0.02, 0.02);

    IMUErrorModel model(
        Matrix33::Identity(),
        Matrix33::Identity(),
        Vector3::Zero(),
        Vector3::Zero(),
        bias_rw_gyro,
        bias_rw_accel,
        Vector3::Zero(),
        Vector3::Zero(),
        42
    );

    IMUErrorModel::TimedSensorData data;
    double dt = 0.1;
    for (int i = 0; i < 10; ++i) {
        double t = i * dt;
        data[t]["gyroscope"] = Vector3(0.0, 0.0, 0.0);
        data[t]["accelerometer"] = Vector3(0.0, 0.0, 0.0);
    }

    model.corruptGyro(data);
    model.corruptAccel(data);

    // Each measurement should now differ due to random walk (simple check: not all zero)
    bool gyro_changed = false;
    bool accel_changed = false;
    for (const auto& [t, sensors] : data) {
        Vector3 g = std::any_cast<Vector3>(sensors.at("gyroscope"));
        Vector3 a = std::any_cast<Vector3>(sensors.at("accelerometer"));
        if (g.norm() > 1e-8) gyro_changed = true;
        if (a.norm() > 1e-8) accel_changed = true;
    }
    EXPECT_TRUE(gyro_changed);
    EXPECT_TRUE(accel_changed);
}
