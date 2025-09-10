// Third-party
#include <gtest/gtest.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

// Local project headers
#include "gtsam/simulation/IMUErrorModel.h"

using namespace gtsam;
using namespace gtsam::simulation;

TEST(IMUErrorModelTest, DefaultConstruction)
{
    IMUErrorModel model;

    IMUErrorModel::TimedSensorData data;
    data[0.0]["gyroscope"] = Vector3(1.0, 2.0, 3.0);
    data[0.0]["accelerometer"] = Vector3(4.0, 5.0, 6.0);

    model.corruptGyro(data);
    model.corruptAccel(data);

    Vector3 gyro_meas = std::any_cast<Vector3>(data[0.0]["gyroscope"]);
    Vector3 accel_meas = std::any_cast<Vector3>(data[0.0]["accelerometer"]);

    EXPECT_NEAR(gyro_meas.x(), 1.0, 1e-8);
    EXPECT_NEAR(gyro_meas.y(), 2.0, 1e-8);
    EXPECT_NEAR(gyro_meas.z(), 3.0, 1e-8);

    EXPECT_NEAR(accel_meas.x(), 4.0, 1e-8);
    EXPECT_NEAR(accel_meas.y(), 5.0, 1e-8);
    EXPECT_NEAR(accel_meas.z(), 6.0, 1e-8);
}

TEST(IMUErrorModelTest, BiasOnly)
{
    Vector3 bias_gyro(0.1, 0.2, 0.3);
    Vector3 bias_accel(-0.1, 0.0, 0.05);

    IMUErrorModel model(
        Matrix33::Identity(),
        Matrix33::Identity(),
        bias_gyro,
        bias_accel,
        Vector3(Vector3::Zero()),
        Vector3(Vector3::Zero()),
        Vector3(Vector3::Zero()),
        Vector3(Vector3::Zero()),
        42 // fixed seed
    );

    IMUErrorModel::TimedSensorData data;
    data[0.0]["gyroscope"] = Vector3(1.0, 2.0, 3.0);
    data[0.0]["accelerometer"] = Vector3(4.0, 5.0, 6.0);

    model.corruptGyro(data);
    model.corruptAccel(data);

    Vector3 gyro_meas = std::any_cast<Vector3>(data[0.0]["gyroscope"]);
    Vector3 accel_meas = std::any_cast<Vector3>(data[0.0]["accelerometer"]);

    EXPECT_NEAR(gyro_meas.x(), 1.1, 1e-8);
    EXPECT_NEAR(gyro_meas.y(), 2.2, 1e-8);
    EXPECT_NEAR(gyro_meas.z(), 3.3, 1e-8);

    EXPECT_NEAR(accel_meas.x(), 3.9, 1e-8);
    EXPECT_NEAR(accel_meas.y(), 5.0, 1e-8);
    EXPECT_NEAR(accel_meas.z(), 6.05, 1e-8);
}

TEST(IMUErrorModelTest, CalibrationMatrix)
{
    Matrix33 C;
    C << 2.0, 0.5, 0.0,
         0.0, 1.5, 0.1,
         0.0, 0.0, 3.0;

    IMUErrorModel model(
        C, C,
        Vector3(Vector3::Zero()),
        Vector3(Vector3::Zero()),
        Vector3(Vector3::Zero()),
        Vector3(Vector3::Zero()),
        Vector3(Vector3::Zero()),
        Vector3(Vector3::Zero()),
        123 // fixed seed
    );

    Vector3 omega(1.0, 2.0, 3.0);
    Vector3 accel(4.0, 5.0, 6.0);

    IMUErrorModel::TimedSensorData data;
    data[0.0]["gyroscope"] = omega;
    data[0.0]["accelerometer"] = accel;

    model.corruptGyro(data);
    model.corruptAccel(data);

    Vector3 gyro_meas = std::any_cast<Vector3>(data[0.0]["gyroscope"]);
    Vector3 accel_meas = std::any_cast<Vector3>(data[0.0]["accelerometer"]);

    Vector3 gyro_recovered = C * gyro_meas;
    Vector3 accel_recovered = C * accel_meas;

    EXPECT_NEAR(gyro_recovered.x(), omega.x(), 1e-8);
    EXPECT_NEAR(gyro_recovered.y(), omega.y(), 1e-8);
    EXPECT_NEAR(gyro_recovered.z(), omega.z(), 1e-8);

    EXPECT_NEAR(accel_recovered.x(), accel.x(), 1e-8);
    EXPECT_NEAR(accel_recovered.y(), accel.y(), 1e-8);
    EXPECT_NEAR(accel_recovered.z(), accel.z(), 1e-8);
}

TEST(IMUErrorModelTest, RandomWalkDeterministic)
{
    Vector3 bias_rw_gyro(0.01, 0.01, 0.01);
    Vector3 bias_rw_accel(0.02, 0.02, 0.02);

    IMUErrorModel model(
        Matrix33::Identity(),
        Matrix33::Identity(),
        Vector3(Vector3::Zero()),
        Vector3(Vector3::Zero()),
        bias_rw_gyro,
        bias_rw_accel,
        Vector3(Vector3::Zero()),
        Vector3(Vector3::Zero()),
        42 // fixed seed
    );

    IMUErrorModel::TimedSensorData data;
    double dt = 0.1;
    for (int i = 0; i < 3; ++i)
    {
        double t = i * dt;
        data[t]["gyroscope"] = Vector3(Vector3::Zero());
        data[t]["accelerometer"] = Vector3(Vector3::Zero());
    }

    model.corruptGyro(data);
    model.corruptAccel(data);

    Vector3 g0 = std::any_cast<Vector3>(data[0.0]["gyroscope"]);
    Vector3 g1 = std::any_cast<Vector3>(data[0.1]["gyroscope"]);
    Vector3 g2 = std::any_cast<Vector3>(data[0.2]["gyroscope"]);

    EXPECT_NEAR(g0.norm(), 0.0, 1e-8);
    EXPECT_NEAR(g1.norm(), 0.00699997, 1e-6);
    EXPECT_NEAR(g2.norm(), 0.00688195, 1e-6);
}
