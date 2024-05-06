# Flight Battery Estimator

This project is a C++ application that estimates the remaining battery charge for a flight mission given a set of waypoints and wind data. The application uses a `FlightBatteryEstimator` class to calculate the remaining battery charge based on the mission's airspeed, initial battery charge, and constant speed power consumption.

## Features

- **Waypoint and Wind Data Input**: The application accepts a set of waypoints and wind data as input. Each waypoint is defined by its coordinates, and each wind data point is defined by its location and wind speed.

- **Battery Charge Estimation**: The application estimates the remaining battery charge after the flight mission. The estimation is based on the mission's airspeed, initial battery charge, and constant speed power consumption.

- **Test Cases**: The project includes a set of test cases to verify the functionality of the `FlightBatteryEstimator` class. The test cases cover different scenarios, including varying wind speeds and directions.

## Usage

To use the `FlightBatteryEstimator`, create an instance of the class and call the `estimatedBatteryRemaining` function with the appropriate parameters. The function returns the estimated remaining battery charge.

```cpp
FlightBatteryEstimator flight_battery_estimator{flight_battery_estimator::associateWindVectorsWithWaypointsUsingClosestWindVector};
std::vector<FlightWaypoint> waypoints;
std::vector<std::shared_ptr<WindData>> wind_speeds;

// Add waypoints and wind data...

double mission_airspeed = 30.0;
double battery_initial_charge = 100.0;
double constant_speed_power_consumption = 1.0;

double battery_remaining = flight_battery_estimator.estimatedBatteryRemaining(waypoints,
                                                                               wind_speeds,
                                                                               mission_airspeed,
                                                                               battery_initial_charge,
                                                                               constant_speed_power_consumption);
```

## Testing

The project includes a set of test cases in the `test_flight_battery_estimator.cpp` file. To run the tests, compile and run the test file.

```cpp
// Example of a test case
TEST(FlightBatteryEstimatorTest, TestEstimatedBatteryRemainingCase1) {
    // Setup and call the estimatedBatteryRemaining function...
    EXPECT_NEAR(battery_remaining, 99.666, 1e-3);
}
```
## Building the Project

This project uses CMake as its build system. Follow these steps to build the project:

1. Install CMake on your system if it's not already installed. You can download it from the [official CMake website](https://cmake.org/download/).

2. Open a terminal and navigate to the root directory of the project.

3. Create a new directory for the build files and navigate into it:

    ```bash
    mkdir build
    cd build
    ```

4. Run CMake to generate the build files. Replace `..` with the path to the directory containing your `CMakeLists.txt` file:

    ```bash
    cmake ..
    ```

5. Run the build command:

    ```bash
    make
    ```

6. After the build process is complete, you can run the tests:

    ```bash
    ./test_flight_battery_estimator
    ```
## Dependencies

This project is written in C++ and requires a C++ compiler to build and run. The project also uses the following libraries:

- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page): A high-level C++ library of template headers for linear algebra, matrix and vector operations, numerical solvers and related algorithms.

- [Google Test](https://github.com/google/googletest): A unit testing library for the C++ programming language, based on the xUnit architecture.

Please ensure that these libraries are installed on your system before building and running the project.