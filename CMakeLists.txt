cmake_minimum_required(VERSION 3.10)
project(gtsam_simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------
# Find dependencies
# ----------------------------
# Eigen3
find_package(Eigen3 3.3 REQUIRED)

# GTSAM
find_package(GTSAM REQUIRED)
if(TARGET gtsam)
    set(GTSAM_TARGET gtsam)
elseif(TARGET GTSAM::gtsam)
    set(GTSAM_TARGET GTSAM::gtsam)
else()
    message(FATAL_ERROR "GTSAM target not found")
endif()

# GTest
find_package(GTest REQUIRED)

# Matplot++
find_package(Matplot++ REQUIRED)

# ----------------------------
# Include directories
# ----------------------------
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}                # Eigen
    ${GTEST_INCLUDE_DIRS}                 # GTest
)

# ----------------------------
# Simulation library
# ----------------------------
add_library(simulation
    src/IMUErrorModel.cpp
    src/IMUScenarioSimulator.cpp
)

target_include_directories(simulation PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(simulation PUBLIC ${GTSAM_TARGET})

# ----------------------------
# Main executable
# ----------------------------
add_executable(sim_main src/main.cpp)

target_include_directories(sim_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(sim_main PRIVATE simulation ${GTSAM_TARGET})

# ----------------------------
# Test GTSAM executable
# ----------------------------
add_executable(test_gtsam src/test_gtsam.cpp)

target_link_libraries(test_gtsam PRIVATE simulation ${GTSAM_TARGET})

# ----------------------------
# GoogleTest executable
# ----------------------------
enable_testing()

add_executable(test_simulator test/TestIMUErrorModel.cpp test/TestIMUScenarioSimulator.cpp)
target_link_libraries(test_simulator PRIVATE simulation gtest gtest_main ${GTSAM_TARGET})

add_test(NAME IMUSimulatorTest COMMAND test_simulator)

# ----------------------------
# Example executable
# ----------------------------
add_executable(imu_example examples/IMUExample.cpp)

target_include_directories(imu_example PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${Matplot_INCLUDE_DIRS}
)

target_link_libraries(imu_example PRIVATE simulation ${GTSAM_TARGET} Matplot++::matplot)
