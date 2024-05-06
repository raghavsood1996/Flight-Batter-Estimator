/**
* @file flight_battery_estimator.h
* @author Raghav Sood (raghav2sood@gmail.com)
* @brief Header file for the FlightBatteryEstimator class
**/

#ifndef FLIGHT_BATTERY_ESTIMATOR_H
#define FLIGHT_BATTERY_ESTIMATOR_H

#include <Eigen/Dense>
#include <vector>
#include <memory>

/**
 * @brief Struct to represent wind data
 */
struct WindData
{
    Eigen::Vector2d position; //meters represents the position where the wind is measured
    Eigen::Vector2d wind_speed{0.0, 0.0}; //m/s represents direction and magnitude of wind
};

/**
 * @brief Struct to represent a waypoint in a flight mission
 */
struct FlightWaypoint
{
    Eigen::Vector2d position; //meters represents the position of the waypoint
    std::shared_ptr<WindData> associated_wind; //m/s represents the wind vector associated with the waypoint
};

/**
 * @brief Class to estimate the remaining battery charge after a flight mission
 */
class FlightBatteryEstimator
{
public:
    using WindDataAssociationFunction = std::function<void(std::vector<FlightWaypoint>&,
                                                           const std::vector<std::shared_ptr<WindData>>&,
                                                           double)>;

    /**
     * @brief Constructor for the FlightBatteryEstimator
     * @param wind_data_association_function function that associates wind vectors with mission waypoints
     */
    explicit FlightBatteryEstimator(WindDataAssociationFunction wind_data_association_function);
    ~FlightBatteryEstimator() = default;

    /**
     * @brief estimates the remaining battery charge after a flight mission
     * @param waypoints waypoints of the flight mission as vector of FlightWaypoint
     * @param wind_speeds wind speed data as vector of shared pointers to WindData
     * @param mission_airspeed constant airspeed followed between waypoints
     * @param battery_initial_charge initial battery charge before the mission
     * @param constant_speed_power_consumption power consumed while flying at constant speed
     * @return remaining battery charge after the mission, return negative value if more power is consumed than initial charge
     */
    double estimatedBatteryRemaining(std::vector<FlightWaypoint>& waypoints,
                                     const std::vector<std::shared_ptr<WindData>>& wind_speeds,
                                     double mission_airspeed,
                                     double battery_initial_charge,
                                     double constant_speed_power_consumption) const;

private:
    double max_wind_association_distance_ = 100.0; //meters
    WindDataAssociationFunction wind_data_association_function_;
};


#endif //FLIGHT_BATTERY_ESTIMATOR_H
