// Standard library
#include <iomanip>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>

// Third-party
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuFactor.h> // Contains PreintegratedImuMeasurements
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/ImuBias.h> // For imuBias::ConstantBias
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
    // --- 1. Define Trajectory: Helix (Perfect Analytic Derivatives) ---
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

    model.acceleration = [](double t) -> Vector3
    {
        return Vector3(0.0, -cos(t), -sin(t));
    };

    model.angularVelocity = [](double t) -> Vector3
    {
        return Vector3(1.0, 0.0, 0.0);
    };

    model.angularAcceleration = [](double t) -> Vector3
    {
        return Vector3(0.0, 0.0, 0.0);
    };

    // --- 2. Simulation Setup (Clean Data) ---
    vector<double> timestamps;
    double dt = 0.1;
    for (double t = 0.0; t <= 10.0 + 1e-6; t += dt) timestamps.push_back(t);

    IMUScenarioSimulator sim(model, timestamps);
    auto imu_data = sim.simulateScenario(); // Generates raw IMU readings

    // Set IMUErrorModel to ZERO (No Noise, No Bias)
    IMUErrorModel error_model(Matrix33::Identity(), Matrix33::Identity(),
                             Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0));
    error_model.corruptGyro(imu_data);
    error_model.corruptAccel(imu_data);

    // --- 3. IMU Pre-integration Setup (Zero Assumed Bias) ---
    auto params = PreintegrationParams::MakeSharedU(9.81);
    params->setAccelerometerCovariance(Matrix33::Identity());
    params->setGyroscopeCovariance(Matrix33::Identity());
    
    // Estimator assumes a ZERO bias, matching the clean simulated data
    Vector3 zero_vector = Vector3::Zero();
    imuBias::ConstantBias estimated_bias(zero_vector, zero_vector);

    // Initial State (t=0.0)
    Pose3 estimated_pose = model.pose(0.0);
    Vector3 estimated_velocity = model.velocity(0.0);

    // Initialize the pre-integration object
    PreintegratedImuMeasurements preint(params, estimated_bias);

    // --- 4. Plotting Vector Initialization ---
    vector<double> true_x, true_y, true_z, est_x, est_y, est_z;
    
    // Add the starting point (t=0.0)
    true_x.push_back(estimated_pose.translation().x());
    true_y.push_back(estimated_pose.translation().y());
    true_z.push_back(estimated_pose.translation().z());
    est_x.push_back(estimated_pose.translation().x());
    est_y.push_back(estimated_pose.translation().y());
    est_z.push_back(estimated_pose.translation().z());

    // --- 5. Main Integration Loop ---
    for (size_t i = 1; i < timestamps.size(); ++i)
    {
        double t = timestamps[i];
        double current_dt = dt; 

        try {
            // Get clean measurements for time t
            Vector3 w = any_cast<Vector3>(imu_data.at(t)["gyroscope"]); 
            Vector3 a = any_cast<Vector3>(imu_data.at(t)["accelerometer"]);
            
            // 1. Integrate the measurement
            preint.integrateMeasurement(a, w, current_dt);

            // 2. Predict the state at time t
            NavState predicted_state = preint.predict(NavState(estimated_pose, estimated_velocity), estimated_bias);

            // 3. Update the estimated state for the next step
            estimated_pose = predicted_state.pose();
            estimated_velocity = predicted_state.velocity();

            // 4. Reset the pre-integration object
            preint.resetIntegration(); 
            
            // 5. Store data for plotting
            Pose3 true_pose = model.pose(t);

            true_x.push_back(true_pose.translation().x());
            true_y.push_back(true_pose.translation().y());
            true_z.push_back(true_pose.translation().z());
            est_x.push_back(estimated_pose.translation().x());
            est_y.push_back(estimated_pose.translation().y());
            est_z.push_back(estimated_pose.translation().z());
        
        } catch (const std::out_of_range& e) {
            // This catches floating point key errors if they arise
            cerr << "Error: Could not find IMU data for time t=" << t << ". Check simulation keys." << endl;
            break;
        } catch (const std::exception& e) {
            cerr << "An unexpected error occurred at time t=" << t << ": " << e.what() << endl;
            break;
        }
    }

    // --- 6. Plot Results ---
    figure();
    hold(on);

    auto p1 = plot3(true_x, true_y, true_z);
    p1->display_name("True Trajectory");

    auto p2 = plot3(est_x, est_y, est_z);
    p2->display_name("Estimated Trajectory (Perfect Dead Reckoning)");

    // Calculate dynamic axis limits
    double xmin = std::min(*std::min_element(true_x.begin(), true_x.end()), *std::min_element(est_x.begin(), est_x.end()));
    double xmax = std::max(*std::max_element(true_x.begin(), true_x.end()), *std::max_element(est_x.begin(), est_x.end()));
    double ymin = std::min(*std::min_element(true_y.begin(), true_y.end()), *std::min_element(est_y.begin(), est_y.end()));
    double ymax = std::max(*std::max_element(true_y.begin(), true_y.end()), *std::max_element(est_y.begin(), est_y.end()));
    double zmin = std::min(*std::min_element(true_z.begin(), true_z.end()), *std::min_element(est_z.begin(), est_z.end()));
    double zmax = std::max(*std::max_element(true_z.begin(), true_z.end()), *std::max_element(est_z.begin(), est_z.end()));

    gca()->xlim({xmin, xmax});
    gca()->ylim({ymin, ymax});
    gca()->zlim({zmin, zmax});

    legend();
    xlabel("X");
    ylabel("Y");
    zlabel("Z");
    title("IMU Dead Reckoning with Perfect Input");

    show();
}