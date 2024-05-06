/**
 *@file test_waypoint_wind_association_functions.cpp
 *@author Raghav Sood (raghav2sood@gmail.com)
 *@brief Unit tests for the waypoint wind association functions
**/

#include <flight_battery_estimator/waypoint_wind_association_functions.h>

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <gtest/gtest.h>

#include <vector>
#include <cstdlib>

namespace
{
    void addWaypoint(std::vector<FlightWaypoint>& waypoints, double x, double y)
    {
        FlightWaypoint waypoint;
        waypoint.position = Eigen::Vector2d(x, y);
        waypoints.push_back(waypoint);
    }

    void addWindData(std::vector<std::shared_ptr<WindData>>& wind_speeds, double x, double y, double wind_x,
                     double wind_y)
    {
        WindData wind_speed;
        wind_speed.position = Eigen::Vector2d(x, y);
        wind_speed.wind_speed = Eigen::Vector2d(wind_x, wind_y);
        wind_speeds.push_back(std::make_shared<WindData>(wind_speed));
    }
}


TEST(WaypointWindAssociationFunctionsTest, TestAssociateWindVectorsWithWaypointsUsingClosestWindVector)
{
    // Create some waypoints and wind data
    std::vector<FlightWaypoint> waypoints;
    std::vector<std::shared_ptr<WindData>> wind_speeds;

    addWaypoint(waypoints, 0, 5);
    addWaypoint(waypoints, 0, 20);

    addWindData(wind_speeds, 0, 0, 5, 0);
    addWindData(wind_speeds, 0, 20, -5, 0);

    // Call the function to test
    flight_battery_estimator::associateWindVectorsWithWaypointsUsingClosestWindVector(waypoints, wind_speeds, 10.0);

    // Check that the wind data was associated correctly
    ASSERT_EQ(waypoints[0].associated_wind, wind_speeds[0]);
    ASSERT_EQ(waypoints[1].associated_wind, wind_speeds[1]);
}

TEST(WaypointWindAssociationFunctionsTest, TestAssociateWindVectorsWithWaypointsUsingClosestWindVectorNoAssociation)
{
    // Create some waypoints and wind data
    std::vector<FlightWaypoint> waypoints;
    std::vector<std::shared_ptr<WindData>> wind_speeds;

    addWaypoint(waypoints, 0, 10);
    addWaypoint(waypoints, 0, 20);

    addWindData(wind_speeds, 100, 100, 5, 0);
    addWindData(wind_speeds, 100, 200, -5, 0);

    // Call the function to test
    flight_battery_estimator::associateWindVectorsWithWaypointsUsingClosestWindVector(waypoints, wind_speeds, 10.0);

    // Check that the wind data was not associated because the wind data is too far away
    ASSERT_EQ(waypoints[0].associated_wind->wind_speed, Eigen::Vector2d(0, 0));
    ASSERT_EQ(waypoints[1].associated_wind->wind_speed, Eigen::Vector2d(0, 0));
}

TEST(WaypointWindAssociationFunctionsTest, TestNoWaypointsOrWindData)
{
    std::vector<FlightWaypoint> waypoints;
    std::vector<std::shared_ptr<WindData>> wind_speeds;

    flight_battery_estimator::associateWindVectorsWithWaypointsUsingClosestWindVector(waypoints, wind_speeds, 10.0);

    ASSERT_TRUE(waypoints.empty());
    ASSERT_TRUE(wind_speeds.empty());
}

TEST(WaypointWindAssociationFunctionsTest, TestWaypointsButNoWindData)
{
    std::vector<FlightWaypoint> waypoints;
    std::vector<std::shared_ptr<WindData>> wind_speeds;

    addWaypoint(waypoints, 0, 10);

    flight_battery_estimator::associateWindVectorsWithWaypointsUsingClosestWindVector(waypoints, wind_speeds, 10.0);

    ASSERT_EQ(waypoints[0].associated_wind->wind_speed, Eigen::Vector2d(0, 0));
}

TEST(WaypointWindAssociationFunctionsTest, TestWindDataButNoWaypoints)
{
    std::vector<FlightWaypoint> waypoints;
    std::vector<std::shared_ptr<WindData>> wind_speeds;

    addWindData(wind_speeds, 0, 10, 5, 0);

    flight_battery_estimator::associateWindVectorsWithWaypointsUsingClosestWindVector(waypoints, wind_speeds, 10.0);

    ASSERT_TRUE(waypoints.empty());
}

TEST(WaypointWindAssociationFunctionsTest, TestSamePosition)
{
    std::vector<FlightWaypoint> waypoints;
    std::vector<std::shared_ptr<WindData>> wind_speeds;

    addWaypoint(waypoints, 0, 10);
    addWindData(wind_speeds, 0, 10, 5, 0);

    flight_battery_estimator::associateWindVectorsWithWaypointsUsingClosestWindVector(waypoints, wind_speeds, 10.0);

    ASSERT_EQ(waypoints[0].associated_wind, wind_speeds[0]);
}

TEST(WaypointWindAssociationFunctionsTest, TestZeroWindSpeed)
{
    std::vector<FlightWaypoint> waypoints;
    std::vector<std::shared_ptr<WindData>> wind_speeds;

    addWaypoint(waypoints, 0, 10);
    addWaypoint(waypoints, 0, 20);

    addWindData(wind_speeds, 0, 10, 0, 0);
    addWindData(wind_speeds, 0, 20, 5, 0);

    flight_battery_estimator::associateWindVectorsWithWaypointsUsingClosestWindVector(waypoints, wind_speeds, 10.0);

    ASSERT_EQ(waypoints[0].associated_wind, wind_speeds[0]);
    ASSERT_EQ(waypoints[1].associated_wind, wind_speeds[1]);
}
