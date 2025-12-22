#pragma once

#include <xy_type.h>

class PositionController
{
public:
    static constexpr float M_PI_F = 3.14159265358979323846F;
    float cross(const xy_t& A, const xy_t& B) { return A.x*B.y - A.y*B.x; }
    /**
    * The current target bearing
    *
    * @return bearing angle (-pi..pi, in NED frame)
    */
    //float nav_bearing() { return matrix::wrap_pi(_nav_bearing); }

    /**
    * Get lateral acceleration demand.
    *
    * @return Lateral acceleration in m/s^2
    */
    float nav_lateral_acceleration_demand() { return _lateral_accel; }

    /**
    * Bearing from aircraft to current target.
    *
    * @return bearing angle (-pi..pi, in NED frame)
    */
    float target_bearing() { return _target_bearing; }

    /**
    * Get the current crosstrack error.
    *
    * @return Crosstrack error in meters.
    */
    float crosstrack_error() { return _crosstrack_error; }

    /**
    * Navigate between two waypoints
    *
    * Calling this function with two waypoints results in the
    * control outputs to fly to the line segment defined by
    * the points and once captured following the line segment.
    * This follows the logic in [1].
    *
    * @return sets _lateral_accel setpoint
    */
    void navigate_waypoints(const xy_t& A, const xy_t& B, const xy_t& currentPosition, const xy_t& groundSpeed);

    /**
    * Set the L1 period.
    */
    void set_l1_period(float period);

    /**
    * Set the L1 damping factor.
    *
    * The original publication recommends a default of sqrt(2) / 2 = 0.707
    */
    void set_l1_damping(float damping);

private:
// 40,010,040 meters

//m_per_deg_lat = 111132.954 - 559.822 * cos( 2 * latMid ) + 1.175 * cos( 4 * latMid);
//m_per_deg_lon = 111132.954 * cos ( latMid );
// https://en.wikipedia.org/wiki/Geographical_distance


    float _lateral_accel{0.0f};        ///< Lateral acceleration setpoint in m/s^2
    float _L1_distance{20.0f};        ///< L1 lead distance, defined by period and damping
    float _nav_bearing{0.0f};        ///< bearing to L1 reference point
    float _crosstrack_error{0.0f};    ///< crosstrack error in meters
    float _target_bearing{0.0f};        ///< the heading setpoint

    float _L1_period{25.0f};        ///< L1 tracking period in seconds
    float _L1_damping{0.75f};        ///< L1 damping ratio
    float _L1_ratio{5.0f};        ///< L1 ratio for navigation
    float _K_L1{2.0f};            ///< L1 control gain for _L1_damping
    float _heading_omega{1.0f};        ///< Normalized frequency
};
