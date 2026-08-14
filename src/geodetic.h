// Convert DIS geocentric (ECEF, WGS84) coordinates to geodetic lat/lon/alt.
// DIS Entity State PDUs carry entity location as ECEF X,Y,Z in metres.
#pragma once
#include <cmath>

namespace geo {

struct LatLonAlt {
    double lat;  // degrees, +N
    double lon;  // degrees, +E
    double alt;  // metres above ellipsoid
};

// WGS84 ellipsoid constants
constexpr double kA  = 6378137.0;                 // semi-major axis (m)
constexpr double kF  = 1.0 / 298.257223563;       // flattening
constexpr double kE2 = kF * (2.0 - kF);           // first eccentricity squared

// Closed-form-ish iterative ECEF -> geodetic (Bowring style). Converges in a
// handful of iterations for anything within a sane distance of Earth.
inline LatLonAlt ecefToLla(double x, double y, double z) {
    constexpr double kRad2Deg = 57.29577951308232;

    const double lon = std::atan2(y, x);
    const double p   = std::sqrt(x * x + y * y);

    // Degenerate case: exactly on the polar axis.
    if (p < 1e-9) {
        const double lat = (z >= 0.0) ? 90.0 : -90.0;
        const double alt = std::fabs(z) - kA * std::sqrt(1.0 - kE2);
        return { lat, lon * kRad2Deg, alt };
    }

    double lat = std::atan2(z, p * (1.0 - kE2));   // initial guess
    double N = kA;
    double alt = 0.0;
    for (int i = 0; i < 8; ++i) {
        const double s = std::sin(lat);
        N = kA / std::sqrt(1.0 - kE2 * s * s);
        alt = p / std::cos(lat) - N;
        lat = std::atan2(z, p * (1.0 - kE2 * N / (N + alt)));
    }

    return { lat * kRad2Deg, lon * kRad2Deg, alt };
}

// Rotate an ECEF velocity vector into the local East/North/Up frame at the
// given geodetic position. Used to derive true ground-track heading & speed.
inline void ecefVelToEnu(double latDeg, double lonDeg,
                         double vx, double vy, double vz,
                         double& e, double& n, double& u) {
    constexpr double kDeg2Rad = 0.017453292519943295;
    const double lat = latDeg * kDeg2Rad;
    const double lon = lonDeg * kDeg2Rad;
    const double sl = std::sin(lat), cl = std::cos(lat);
    const double so = std::sin(lon), co = std::cos(lon);
    e = -so * vx + co * vy;
    n = -sl * co * vx - sl * so * vy + cl * vz;
    u =  cl * co * vx + cl * so * vy + sl * vz;
}

// Geodetic -> ECEF, used by the test sender to place entities by lat/lon.
inline void llaToEcef(double latDeg, double lonDeg, double alt,
                      double& x, double& y, double& z) {
    constexpr double kDeg2Rad = 0.017453292519943295;
    const double lat = latDeg * kDeg2Rad;
    const double lon = lonDeg * kDeg2Rad;
    const double s = std::sin(lat);
    const double N = kA / std::sqrt(1.0 - kE2 * s * s);
    x = (N + alt) * std::cos(lat) * std::cos(lon);
    y = (N + alt) * std::cos(lat) * std::sin(lon);
    z = (N * (1.0 - kE2) + alt) * s;
}

} // namespace geo
