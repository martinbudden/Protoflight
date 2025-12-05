#pragma once

#include <xyz_type.h>

class Geodetic {
public:
    static constexpr float WGS84_EQUATORIAL_RADIUS_METERS = 6378137.0F;
public:
    static void setMedian(float latitudeDegrees, float longitudeDegrees) {
        MedianLatitudeDegrees = latitudeDegrees;
        MedianLongitudeDegrees = longitudeDegrees;
        static constexpr float DEGREES_TO_RADIANS = 3.14159265358979323846F / 180.0F;
        const float mLat = MedianLatitudeDegrees*DEGREES_TO_RADIANS;
        // https://en.wikipedia.org/wiki/Geographical_distance
        KLatitude = 111132.09F - 566.05F*std::cos(2.0F*mLat) + 1.20F*std::cos(4.0F*mLat);
        KLongitude = 111415.13F*std::cos(mLat) - 94.55F*std::cos(3.0F*mLat) + 0.12F*std::cos(5.0F*mLat);
    }
    float latitudeDistanceMeters(float deltaLatitude) { return KLatitude*deltaLatitude; }
    float longitudeDistanceMeters(float deltaLongitude) { return KLongitude*deltaLongitude; }
    xyz_t distanceMeters(const Geodetic& other) {
        return {
            KLatitude*(latitude - other.latitude),
            KLongitude*(longitude - other.longitude),
            altitude - other.altitude
        };
    }
    float latitude;
    float longitude;
    float altitude;
private:
    static float MedianLatitudeDegrees;
    static float MedianLongitudeDegrees;
    static float KLatitude;
    static float KLongitude;
};
