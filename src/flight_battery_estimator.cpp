/**
* @file flight_battery_estimator.cpp
* @author Raghav Sood (raghav2sood@gmail.com)
* @brief Source file for the flight battery estimator
**/

#include <flight_battery_estimator/flight_battery_estimator.h>

#include <Eigen/Dense>

#include <iostream>
#include <vector>



FlightBatteryEstimator::FlightBatteryEstimator(WindDataAssociationFunction wind_data_association_function) :
    wind_data_association_function_(std::move(wind_data_association_function))
{
}


double FlightBatteryEstimator::estimatedBatteryRemaining(std::vector<FlightWaypoint>& waypoints,
                                                         const std::vector<std::shared_ptr<WindData>>& wind_speeds,
                                                         double mission_airspeed,
                                                         double battery_initial_charge,
                                                         double constant_speed_power_consumption) const
{
    double battery_remaining = battery_initial_charge;
    //if waypoints is empty return the initial battery charge
    if (waypoints.empty()){
        return battery_remaining;
    }

    wind_data_association_function_(waypoints, wind_speeds, max_wind_association_distance_);
    //print out the waypoints and their associated wind vectors
    for (int i = 0; i < waypoints.size(); i++){
        std::cout<<"Waypoint "<<i<<" position: "<<waypoints[i].position.transpose()<<" associated wind: "<<
            waypoints[i].associated_wind->wind_speed.transpose()<<"\n";
    }

    //loop through waypoints and calculate the distance between them and the power consumed while average the wind speed between the waypoints
    for (int i = 0; i < waypoints.size() - 1; i++){
        const Eigen::Vector2d& waypoint_position = waypoints[i].position;
        const Eigen::Vector2d& next_waypoint_position = waypoints[i+1].position;
        const double distance = (next_waypoint_position - waypoint_position).norm();
        auto average_wind = (waypoints[i].associated_wind->wind_speed + waypoints[i+1].associated_wind->wind_speed) / 2;
        const double airspeed = mission_airspeed + average_wind.norm();
        const double time = distance / airspeed;
        const double power_consumed = time * constant_speed_power_consumption;
        battery_remaining -= power_consumed;
        std::cout<<"Battery remaining: "<<battery_remaining<<"\n";
    }

    return battery_remaining;
}
