/**
 * @file waypoint_wind_association_functions.cpp
 * @author Raghav Sood (raghav2sood@gmail.com)
 * @brief Source file for the waypoint wind association functions
**/

#include <flight_battery_estimator/waypoint_wind_association_functions.h>

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <iostream>


namespace flight_battery_estimator
{
void associateWindVectorsWithWaypointsUsingClosestWindVector(std::vector<FlightWaypoint>& waypoints,
                                                            const std::vector<std::shared_ptr<WindData>>& wind_speeds,
                                                            double max_wind_association_distance){

    //loop through waypoints and associate the closest wind vector
    for (auto & waypoint : waypoints){
        Eigen::Vector2d waypoint_position = waypoint.position;
        double min_distance = std::numeric_limits<double>::max();
        int min_index = -1;
        for (int j = 0; j < wind_speeds.size(); j++){
            Eigen::Vector2d wind_position = wind_speeds[j]->position;
            const double distance = (waypoint_position - wind_position).norm();
            if (distance < min_distance){
                min_distance = distance;
                min_index = j;
            }
        }
        if (min_distance < max_wind_association_distance && min_index != -1){
            waypoint.associated_wind = wind_speeds[min_index];
        }
        else
        {
            const auto wind_data = std::make_shared<WindData>();
            wind_data->position = waypoint_position;
            wind_data->wind_speed = Eigen::Vector2d(0, 0);
            waypoint.associated_wind = wind_data;
        }
    }
}
}
