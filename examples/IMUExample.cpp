// Standard library
#include <iomanip>
#include <iostream>
#include <vector>

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

int main() {

    // ---------------------------
    // 1. Define the trajectory model
    // ---------------------------
    IMUScenarioSimulator::TrajectoryModel model;

    model.pose = [](double t) -> Pose3 {
        Point3 p(t, cos(t), sin(t));
        Rot3 R = Rot3::RzRyRx(0.1*t, 0.05*t, 0.0);
        return Pose3(R, p);
    };

    model.velocity = [](double t) -> Vector3 {
        return Vector3(1.0, -sin(t), cos(t));
    };

    model.angularVelocity = [](double /*t*/) -> Vector3 {
        return Vector3(0.1, 0.0, 0.0);
    };

    // ---------------------------
    // 2. Generate timestamps
    // ---------------------------
    vector<double> timestamps;
    double dt = 0.1;
    for (double t=0.0; t<=2.0; t+=dt) timestamps.push_back(t);

    // ---------------------------
    // 3. Simulate IMU measurements
    // ---------------------------
    IMUScenarioSimulator sim(model, timestamps);
    auto imu_data = sim.simulateScenario();

    // ---------------------------
    // 4. Corrupt IMU measurements
    // ---------------------------
    IMUErrorModel error_model(
        Matrix33::Identity(), Matrix33::Identity(),
        Vector3(0.01,0.02,0.03), Vector3(0.0,0.0,0.0)
    );
    error_model.corruptGyro(imu_data);
    error_model.corruptAccel(imu_data);

    // ---------------------------
    // 5. Setup rotation preintegration
    // ---------------------------
    auto params = PreintegrationParams::MakeSharedU(9.81);
    PreintegratedRotation preint(params);

    Vector3 gyro_bias(0.01, 0.02, 0.03);

    // ---------------------------
    // 6. Pose estimation loop
    // ---------------------------
    Pose3 estimated_pose; 
    Point3 estimated_position(0,0,0);

    cout << fixed << setprecision(3);
    cout << setw(5) << "t" 
         << setw(12) << "true_x" << setw(12) << "true_y" << setw(12) << "true_z"
         << setw(12) << "est_x"  << setw(12) << "est_y"  << setw(12) << "est_z"
         << setw(12) << "rot_err_deg" << endl;

    vector<double> times, true_x, true_y, true_z, est_x, est_y, est_z;

    for (double t : timestamps) {

        // Integrate angular velocity
        Vector3 w = any_cast<Vector3>(imu_data[t]["gyroscope"]);
        preint.integrateMeasurement(w, gyro_bias, dt);
        Rot3 delta_R = preint.deltaRij();

        estimated_pose = Pose3(delta_R, estimated_position);

        // Integrate translation
        Vector3 v = model.velocity(t);
        estimated_position += v * dt;

        Pose3 true_pose = model.pose(t);

        Rot3 R_error = estimated_pose.rotation().between(true_pose.rotation());
        Vector3 rpy_error = R_error.rpy() * 180.0 / M_PI;

        cout << setw(5) << t
             << setw(12) << true_pose.translation().x()
             << setw(12) << true_pose.translation().y()
             << setw(12) << true_pose.translation().z()
             << setw(12) << estimated_pose.translation().x()
             << setw(12) << estimated_pose.translation().y()
             << setw(12) << estimated_pose.translation().z()
             << setw(12) << rpy_error.norm() 
             << endl;

        times.push_back(t);
        true_x.push_back(true_pose.translation().x());
        true_y.push_back(true_pose.translation().y());
        true_z.push_back(true_pose.translation().z());
        est_x.push_back(estimated_pose.translation().x());
        est_y.push_back(estimated_pose.translation().y());
        est_z.push_back(estimated_pose.translation().z());
    }

    // ---------------------------
    // 7. Plot results with Matplot++
    // ---------------------------
    figure();
    hold(on);

    auto p1 = plot3(true_x, true_y, true_z);
    p1->display_name("True Trajectory");

    auto p2 = plot3(est_x, est_y, est_z);
    p2->display_name("Estimated Trajectory");

    // Manually compute bounds
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

    // Set axes manually
    gca()->xlim({xmin, xmax});
    gca()->ylim({ymin, ymax});
    gca()->zlim({zmin, zmax});

    legend();
    xlabel("X (m)");
    ylabel("Y (m)");
    zlabel("Z (m)");
    title("IMU Simulation: True vs Estimated 3D Trajectories");
    grid(true);

    show();

    return 0;
}
