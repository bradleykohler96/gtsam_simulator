// Standard library
#include <iomanip>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

// Third-party
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/PreintegratedRotation.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <matplot/matplot.h>

// Local project headers
#include "gtsam/simulation/IMUScenarioSimulator.h"
#include "gtsam/simulation/IMUErrorModel.h"

using namespace std;
using namespace gtsam;
using namespace gtsam::simulation;
using namespace matplot;

int main()
{
    IMUScenarioSimulator::TrajectoryModel model;

    model.pose = [](double t) -> Pose3
    {
        Point3 p(t, cos(t), sin(t));

        Vector3 ex(0.0, -cos(t), -sin(t));
        Vector3 ez(1.0, -sin(t), cos(t));
        ez = ez / sqrt(2.0);
        Vector3 ey = ez.cross(ex);
        ey = ey / ey.norm();

        Matrix3 mat;
        mat.col(0) = ex;
        mat.col(1) = ey;
        mat.col(2) = ez;

        Rot3 R(mat);
        return Pose3(R, p);
    };

    model.velocity = [](double t) -> Vector3
    {
        return Vector3(1.0, -sin(t), cos(t));
    };

    model.angularVelocity = [](double /*t*/) -> Vector3
    {
        return Vector3(1.0, 0.0, 0.0);
    };

    vector<double> timestamps;
    double dt = 0.1;
    for (double t = 0.0; t <= 10.0; t += dt) timestamps.push_back(t);

    IMUScenarioSimulator sim(model, timestamps);
    auto imu_data = sim.simulateScenario();

    IMUErrorModel error_model(Matrix33::Identity(), Matrix33::Identity(),
                              Vector3(0.01, 0.02, 0.03), Vector3(0.0, 0.0, 0.0));
    error_model.corruptGyro(imu_data);
    error_model.corruptAccel(imu_data);

    auto params = PreintegrationParams::MakeSharedU(9.81);
    PreintegratedRotation preint(params);
    Vector3 gyro_bias(0.01, 0.02, 0.03);

    Pose3 estimated_pose;
    Point3 estimated_position(0.0, 1.0, 0.0);

    vector<double> true_x, true_y, true_z, est_x, est_y, est_z;

    for (double t : timestamps)
    {
        Vector3 w = any_cast<Vector3>(imu_data[t]["gyroscope"]);
        preint.integrateMeasurement(w, gyro_bias, dt);
        Rot3 delta_R = preint.deltaRij();
        estimated_pose = Pose3(delta_R, estimated_position);

        Vector3 v = model.velocity(t);
        estimated_position += v * dt;

        Pose3 true_pose = model.pose(t);

        true_x.push_back(true_pose.translation().x());
        true_y.push_back(true_pose.translation().y());
        true_z.push_back(true_pose.translation().z());
        est_x.push_back(estimated_pose.translation().x());
        est_y.push_back(estimated_pose.translation().y());
        est_z.push_back(estimated_pose.translation().z());
    }

    figure();
    hold(on);

    auto p1 = plot3(true_x, true_y, true_z);
    p1->display_name("True Trajectory");

    auto p2 = plot3(est_x, est_y, est_z);
    p2->display_name("Estimated Trajectory");

    double xmin = std::min(*std::min_element(true_x.begin(), true_x.end()),
                           *std::min_element(est_x.begin(), est_x.end()));
    double xmax = std::max(*std::max_element(true_x.begin(), true_x.end()),
                           *std::max_element(est_x.begin(), est_x.end()));
    double ymin = std::min(*std::min_element(true_y.begin(), true_y.end()),
                           *std::min_element(est_y.begin(), est_y.end()));
    double ymax = std::max(*std::max_element(true_y.begin(), true_y.end()),
                           *std::max_element(est_y.begin(), est_y.end()));
    double zmin = std::min(*std::min_element(true_z.begin(), true_z.end()),
                           *std::min_element(est_z.begin(), est_z.end()));
    double zmax = std::max(*std::max_element(true_z.begin(), true_z.end()),
                           *std::max_element(est_z.begin(), est_z.end()));

    gca()->xlim({xmin, xmax});
    gca()->ylim({ymin, ymax});
    gca()->zlim({zmin, zmax});

    legend();
    xlabel("X");
    ylabel("Y");
    zlabel("Z");
    title("IMU Helix Simulation");

    show();
}
