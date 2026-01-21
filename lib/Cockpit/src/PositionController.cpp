#include "PositionController.h"
#include <algorithm>
#include <cmath>


void PositionController::navigate_waypoints(const xy_t& A, const xy_t& B, const xy_t& P, const xy_t& groundSpeed)
{
    // this follows the logic presented in [1]
    float eta = 0.0F;

    // get the direction between the last (visited) and next waypoint
    const xy_t PB = B - P;
    const xy_t PB_normalized = PB.normalized();
    _target_bearing = atan2f(PB_normalized.y, PB_normalized.x);

    // enforce a minimum ground speed of 0.1 m/s to avoid singularities
    const float ground_speed = std::max(groundSpeed.magnitude(), 0.1F);

    // calculate the L1 length required for the desired period
    _L1_distance = _L1_ratio * ground_speed;

    // calculate vector from A to B
    xy_t AB = B - A;
    // check if waypoints are on top of each other.
    // If yes, skip A and directly continue to B
    if (AB.magnitude() < 1.0e-6F) {
        AB = B - P;
    }
    AB.normalize();

    // calculate the vector from waypoint A to the aircraft (P)
    const xy_t AP = P - A;

    // calculate crosstrack error (output only)
    _crosstrack_error = AB.cross(AP);

    // If the current position is in a +-135 degree angle behind waypoint A
    // and further away from A than the L1 distance, then A becomes the L1 point.
    // If the aircraft is already between A and B normal L1 logic is applied.

    // estimate aircraft position WRT to B
    const xy_t BP_normalized = (P - B).normalized();

    // calculate angle of aircraft position vector relative to line
    const float AB_BP_bearing = atan2f(BP_normalized.cross(AB), BP_normalized.dot(AB));

    // extension from [2], fly directly to A
    const float distance_AP = AP.magnitude();
    const float alongTrackDistance = AP.dot(B);
    if ((distance_AP > _L1_distance) && (alongTrackDistance / std::max(distance_AP, 1.0F) < -0.7071F)) {
        // calculate eta to fly to waypoint A

        // unit vector from waypoint A to current position
        const xy_t AP_normalized = AP.normalized();

        // velocity across / orthogonal to line
        const float crossTrackVelocity = groundSpeed.cross(-AP_normalized);

        // velocity along line
        const float alongTrackVelocity = groundSpeed.dot(-AP_normalized);
        eta = atan2f(crossTrackVelocity, alongTrackVelocity);

        // bearing from current position to L1 point
        _nav_bearing = atan2f(-AP_normalized.y, -AP_normalized.x);

        // If the AB vector and the vector from B to aircraft point in the same
        // direction, we have missed the waypoint. At +- 90 degrees we are just passing it.
    } else if (fabsf(AB_BP_bearing) < 1.5F) {//math::radians(100.0F)) {
        // Extension, fly back to waypoint.
        // This corner case is possible if the system was following the AB line from waypoint A to waypoint B,
        // and then is switched to manual mode (or otherwise misses the waypoint)
        // and from behind the waypoint continues to follow the AB line.

        // calculate eta to fly to waypoint B

        // velocity across / orthogonal to line
        const float crossTrackVelocity = groundSpeed.cross(-BP_normalized);

        // velocity along line
        const float alongTrackVelocity = groundSpeed.dot(-BP_normalized);
        eta = atan2f(crossTrackVelocity, alongTrackVelocity);

        // bearing from current position to L1 point
        _nav_bearing = atan2f(-BP_normalized.y, -BP_normalized.x);

    } else {
        // calculate eta to fly along the line between A and B

        // velocity across / orthogonal to line
        const float crossTrackVelocity = groundSpeed.cross(AB);

        // velocity along line
        const float alongTrackVelocity = groundSpeed.dot(AB);

        // calculate eta2 (angle of velocity vector relative to line)
        const float eta2 = atan2f(crossTrackVelocity, alongTrackVelocity);

        // calculate eta1 (angle to L1 point)
        const float crossTrackError = AP.cross(AB);
        const float sine_eta1 = std::clamp(crossTrackError / std::max(_L1_distance, 0.1F), -1.0F, 1.0F);
        const float eta1 = asinf(sine_eta1);
        eta = eta1 + eta2;

        // bearing from current position to L1 point
        _nav_bearing = atan2f(AB.y, AB.x) + eta1;
    }

    // limit angle to +-90 degrees
    eta = std::clamp(eta, -M_PI_F/2.0F, M_PI_F/2.0F);
    _lateral_accel = _K_L1 * ground_speed * ground_speed / _L1_distance * sinf(eta);
}

void PositionController::set_L1_period(float period)
{
    _L1_period = period;

    // calculate the ratio introduced in [2]
    _L1_ratio = 1.0F / M_PI_F * _L1_damping * _L1_period;

    // calculate normalized frequency for heading tracking
    _heading_omega = sqrtf(2.0F) * M_PI_F / _L1_period;
}

void PositionController::set_L1_damping(float damping)
{
    _L1_damping = damping;

    // calculate the ratio introduced in [2]
    _L1_ratio = 1.0F / M_PI_F * _L1_damping * _L1_period;

    // calculate the L1 gain (following [2])
    _K_L1 = 4.0F * _L1_damping * _L1_damping;
}
