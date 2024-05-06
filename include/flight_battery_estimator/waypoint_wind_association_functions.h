/**
 * @file waypoint_wind_association_functions.h
 * @author Raghav Sood (raghav2sood@gmail.com)
 * @brief Header file for the waypoint wind association functions
**/

#ifndef WAYPOINT_WIND_ASSOCIATION_FUNCTIONS_H
#define WAYPOINT_WIND_ASSOCIATION_FUNCTIONS_H

#include <vector>
#include <memory>

#include "flight_battery_estimator/flight_battery_estimator.h"

namespace flight_battery_estimator
{
/**
 * @brief Function to associate wind vectors with waypoints using the closest wind vector
 * @details for each waypoint, the function finds the closest wind vector and associates it with the waypoint if the
 * distance is less than max_wind_association_distance
 * @param waypoints waypoints of the flight mission as vector of FlightWaypoint
 * @param wind_speeds wind speed data as vector of shared pointers to WindData
 * @param max_wind_association_distance maximum distance to associate a wind vector with a waypoint
 */
void associateWindVectorsWithWaypointsUsingClosestWindVector(std::vector<FlightWaypoint>& waypoints,
                                                             const std::vector<std::shared_ptr<WindData>>& wind_speeds,
                                                             double max_wind_association_distance);
}


#endif //WAYPOINT_WIND_ASSOCIATION_FUNCTIONS_H
