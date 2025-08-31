#pragma once

// Standard library
#include <any>
#include <map>
#include <string>
#include <vector>

namespace gtsam {
namespace simulation {

/**
 * @brief Abstract base class for scenario simulators.
 *
 * The ScenarioSimulator defines the interface for generating synthetic
 * sensor data. Subclasses implement specific simulation models.
 *
 * The output is a sequence of time-ordered measurements, where each
 * timestep's data is stored as a map from sensor name (string) to
 * sensor measurement (std::any for type flexibility).
 */
class ScenarioSimulator {
public:
    /// Virtual destructor for safe polymorphic deletion
    virtual ~ScenarioSimulator() = default;

    /**
     * @brief Simulate the scenario.
     *
     * @return A map of measurements keyed by timestamp.
     *         Each entry maps a simulation time (double) to a collection of sensor readings.
     *         Sensor readings are stored as a map from sensor name (string) to sensor data (std::any).
     *         Subclasses decide the exact sensor types and contents.
     */
    virtual std::map<double, std::map<std::string, std::any>> simulateScenario() = 0;
};

} // namespace simulation
} // namespace gtsam
