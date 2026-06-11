// Geo.h - Geometry / path math for the simulation.
#pragma once

#include <cmath>

namespace nmea {

#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

// A local planar offset from the simulation centre, measured in nautical miles.
// x is east (+) / west (-), y is north (+) / south (-).
struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

// Path shapes the ownship / targets can travel.
enum class Shape {
    Circle = 0,
    Square = 1,
    FigureEight = 2
};

// Returns a point (in NM, east/north) on the requested closed path for
// parameter t in radians [0, 2*pi). The path is centred on (0,0) and fits a
// box of half-width a = width/2 and half-height b = height/2.
inline Vec2 PathPoint(Shape shape, double t, double width, double height) {
    const double a = width * 0.5;
    const double b = height * 0.5;

    // Normalise t to [0, 2*pi).
    t = std::fmod(t, 2.0 * M_PI);
    if (t < 0.0) t += 2.0 * M_PI;

    switch (shape) {
        case Shape::Circle:
            return { a * std::cos(t), b * std::sin(t) };

        case Shape::FigureEight: {
            // Lemniscate of Gerono: a clean figure-eight.
            return { a * std::sin(t), b * std::sin(t) * std::cos(t) };
        }

        case Shape::Square:
        default: {
            // Walk the rectangle perimeter clockwise from the top-right corner.
            const double f = t / (2.0 * M_PI); // [0,1)
            if (f < 0.25) {
                const double k = f / 0.25;            // (a,b) -> (a,-b)
                return { a, b - k * 2.0 * b };
            } else if (f < 0.5) {
                const double k = (f - 0.25) / 0.25;   // (a,-b) -> (-a,-b)
                return { a - k * 2.0 * a, -b };
            } else if (f < 0.75) {
                const double k = (f - 0.5) / 0.25;    // (-a,-b) -> (-a,b)
                return { -a, -b + k * 2.0 * b };
            } else {
                const double k = (f - 0.75) / 0.25;   // (-a,b) -> (a,b)
                return { -a + k * 2.0 * a, b };
            }
        }
    }
}

// Magnitude of the path tangent dP/dt (NM per radian) via a central finite
// difference. Used to advance the path parameter at (approximately) constant
// ground speed regardless of shape.
inline double PathSpeed(Shape shape, double t, double width, double height) {
    const double h = 1e-4;
    const Vec2 p1 = PathPoint(shape, t - h, width, height);
    const Vec2 p2 = PathPoint(shape, t + h, width, height);
    const double dx = (p2.x - p1.x) / (2.0 * h);
    const double dy = (p2.y - p1.y) / (2.0 * h);
    return std::sqrt(dx * dx + dy * dy);
}

// Convert a local NM offset to an absolute latitude/longitude given the
// simulation centre. 1 NM == 1 arc-minute of latitude.
inline void OffsetToLatLon(double centreLat, double centreLon,
                           double xNm, double yNm,
                           double& outLat, double& outLon) {
    const double dLat = yNm / 60.0;
    const double cosLat = std::cos(centreLat * M_PI / 180.0);
    const double dLon = (cosLat > 1e-6) ? (xNm / (60.0 * cosLat)) : 0.0;
    outLat = centreLat + dLat;
    outLon = centreLon + dLon;
}

// Compass bearing (degrees, 0..360, 0 = north) of the vector (east, north).
inline double BearingDeg(double east, double north) {
    double deg = std::atan2(east, north) * 180.0 / M_PI;
    if (deg < 0.0) deg += 360.0;
    return deg;
}

} // namespace nmea
