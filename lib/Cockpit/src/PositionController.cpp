#include "PositionController.h"
#include <cmath>


void PositionController::navigate_waypoints(const xy_t& A, const xy_t& B, const xy_t& currentPosition, const xy_t& groundSpeed)
{
    // this follows the logic presented in [1]
    float eta = 0.0F;

    // get the direction between the last (visited) and next waypoint
    const xy_t P_to_B = B - currentPosition;
    const xy_t P_to_B_normalized = P_to_B.normalize();
    _target_bearing = atan2f(P_to_B_normalized.y, P_to_B_normalized.x);

    // enforce a minimum ground speed of 0.1 m/s to avoid singularities
    float ground_speed = std::max(groundSpeed.magnitude(), 0.1F);

    // calculate the L1 length required for the desired period
    _L1_distance = _L1_ratio * ground_speed;

    // calculate vector from A to B
    xy_t AB = B - A;

    // check if waypoints are on top of each other. 
    // If yes, skip A and directly continue to B
    
    if (AB.magnitude() < 1.0e-6F) {
        AB = B - currentPosition;
    }

    AB.normalize();

    // calculate the vector from waypoint A to the aircraft
    const xy_t A_to_aircraft = currentPosition - A;

    // calculate crosstrack error (output only)
    _crosstrack_error = cross(AB, A_to_aircraft);

    // If the current position is in a +-135 degree angle behind waypoint A
    // and further away from A than the L1 distance, then A becomes the L1 point.
    // If the aircraft is already between A and B normal L1 logic is applied.
    

    // estimate aircraft position WRT to B
    const xy_t B_to_P = currentPosition - B;
    const xy_t B_to_P_normalized = B_to_P.normalize();

    // calculate angle of aircraft position vector relative to line
    const float AB_to_BP_bearing = atan2f(cross(B_to_P_normalized, AB), B_to_P_normalized.dot(AB));

    // extension from [2], fly directly to A
    const float distance_A_to_aircraft = A_to_aircraft.magnitude();
    const float alongTrackDist = A_to_aircraft.dot(B);
    if (distance_A_to_aircraft > _L1_distance && alongTrackDist / std::max(distance_A_to_aircraft, 1.0F) < -0.7071F) {
        // calculate eta to fly to waypoint A

        // unit vector from waypoint A to current position
        const xy_t A_to_aircraft_normalized = A_to_aircraft.normalize();

        // velocity across / orthogonal to line
        const float crossTrackVelocity = cross(groundSpeed, -A_to_aircraft_normalized);

        // velocity along line
        const float alongTrackVelocity = groundSpeed.dot(-A_to_aircraft_normalized);
        eta = atan2f(crossTrackVelocity, alongTrackVelocity);

        // bearing from current position to L1 point
        _nav_bearing = atan2f(-A_to_aircraft_normalized.y, -A_to_aircraft_normalized.x);

        // If the AB vector and the vector from B to aircraft point in the same
        // direction, we have missed the waypoint. At +- 90 degrees we are just passing it.
    } else if (fabsf(AB_to_BP_bearing) < 1.5F) {//math::radians(100.0F)) {
        // Extension, fly back to waypoint.
        // This corner case is possible if the system was following the AB line from waypoint A to waypoint B, 
        // and then is switched to manual mode (or otherwise misses the waypoint)
        // and from behind the waypoint continues to follow the AB line.

        // calculate eta to fly to waypoint B

        // velocity across / orthogonal to line
        const float crossTrackVelocity = cross(groundSpeed, (-B_to_P_normalized));

        // velocity along line
        const float alongTrackVelocity = groundSpeed.dot(-B_to_P_normalized);
        eta = atan2f(crossTrackVelocity, alongTrackVelocity);

        // bearing from current position to L1 point
        _nav_bearing = atan2f(-B_to_P_normalized.y, -B_to_P_normalized.x);

    } else {
        // calculate eta to fly along the line between A and B

        // velocity across / orthogonal to line
        const float crossTrackVelocity = cross(groundSpeed, AB);

        // velocity along line
        const float alongTrackVelocity = groundSpeed.dot(AB);

        // calculate eta2 (angle of velocity vector relative to line)
        const float eta2 = atan2f(crossTrackVelocity, alongTrackVelocity);

        // calculate eta1 (angle to L1 point)
        const float crossTrackError = cross(A_to_aircraft, AB);
        float sine_eta1 = crossTrackError / std::max(_L1_distance, 0.1f);

        // limit output to feasible values
        //sine_eta1 = std::clamp(sine_eta1, -1.0F, 1.0F);
        const float eta1 = asinf(sine_eta1);
        eta = eta1 + eta2;

        // bearing from current position to L1 point
        _nav_bearing = atan2f(AB.y, AB.x) + eta1;
    }

    // limit angle to +-90 degrees
    //eta = std::constrain(eta, (-M_PI_F) / 2.0F, +M_PI_F / 2.0F);
    _lateral_accel = _K_L1 * ground_speed * ground_speed / _L1_distance * sinf(eta);
}

void PositionController::set_l1_period(float period)
{
    _L1_period = period;

    // calculate the ratio introduced in [2]
    _L1_ratio = 1.0F / M_PI_F * _L1_damping * _L1_period;

    // calculate normalized frequency for heading tracking
    _heading_omega = sqrtf(2.0F) * M_PI_F / _L1_period;
}

void PositionController::set_l1_damping(float damping)
{
    _L1_damping = damping;

    // calculate the ratio introduced in [2]
    _L1_ratio = 1.0F / M_PI_F * _L1_damping * _L1_period;

    // calculate the L1 gain (following [2])
    _K_L1 = 4.0F * _L1_damping * _L1_damping;
}
