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
    // --- 1. Define Trajectory: Helix ---
    IMUScenarioSimulator::TrajectoryModel model;

    model.pose = [](double t) -> Pose3
    {
        // Trajectory parameters
        const double A = 5.0; // Size of the figure-eight
        const double omega = 0.5; // Frequency of motion

        // 1. Position (Figure-Eight / Lemniscate in XY-plane)
        double x = A * sin(omega * t);
        double y = A * sin(omega * t) * cos(omega * t);
        double z = 0.5 * t; // Gentle climb along Z
        Point3 p(x, y, z);

        // 2. Rotation (Rotation needs to follow the tangent direction for correct IMU data)
        // Simplified approach: Yaw follows the trajectory's bearing
        double dx_dt = A * omega * cos(omega * t);
        double dy_dt = A * omega * (cos(omega * t) * cos(omega * t) - sin(omega * t) * sin(omega * t));
        
        // Calculate the yaw angle (atan2(dy, dx))
        double yaw = atan2(dy_dt, dx_dt);
        
        // Pitch and Roll are constant (level)
        double pitch = 0.0;
        double roll = 0.0;

        Rot3 R = Rot3::Ypr(yaw, pitch, roll);
        
        return Pose3(R, p);
    };

    // --- 2. Simulation Setup (Clean Data) ---
    vector<double> timestamps;
    double dt = 0.1;
    for (double t = 0.0; t <= 10.0 + 1e-6; t += dt) timestamps.push_back(t);

    IMUScenarioSimulator sim(
            model, 
            timestamps,
            IMUScenarioSimulator::DifferentiationMethod::Central,
            (std::function<gtsam::Vector3(double)>)nullptr,
            1e-8,
            true
        );
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
    Pose3 estimated_pose = any_cast<Pose3>(imu_data.at(0.0)["pose"]); 
    Vector3 estimated_velocity = any_cast<Vector3>(imu_data.at(0.0)["true_velocity"]);

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
    title("IMU Dead Reckoning with Perfect Input (Figure-Eight)");

    show();
}
