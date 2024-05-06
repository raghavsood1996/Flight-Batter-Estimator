cmake_minimum_required(VERSION 3.10)

# Set the project name
project(flight_battery_estimator)

# Find the Eigen3 and GTest libraries
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

# Add the library
add_library(${PROJECT_NAME}
        src/flight_battery_estimator.cpp
        src/waypoint_wind_association_functions.cpp)

# Add the executable
add_executable(test_flight_battery_estimator
        test/test_flight_battery_estimator.cpp
        test/test_waypoint_wind_association_functions.cpp
        test/test_main.cpp)

# Specify include directories
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_include_directories(test_flight_battery_estimator PRIVATE include)

# Link Eigen3 library, GTest libraries and the project library
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(test_flight_battery_estimator ${PROJECT_NAME} Eigen3::Eigen GTest::GTest GTest::Main)