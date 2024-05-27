/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_COORD_SYSTEM_H
#define SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_COORD_SYSTEM_H

#include <array>

namespace swift {
namespace internal {

/**
 * @brief Geodetic coordinates latitude, longitude and height.
 */
struct LatLonHgt {
  double lat_rad = 0.0;
  double lon_rad = 0.0;
  double hgt_m = 0.0;
};

/**
 * @brief Geographic coordinates latitude and longitude in radians.
 */
struct LatLon {
  double lat_rad = 0.0;
  double lon_rad = 0.0;
};

/**
 * @brief Subtract two longitudes (first - second).
 *
 * @details The result will be in the shortest cardinal direction. If first is
 * closest to second in the east direction (first east of second), then the
 * result will be positive. If first is closest to second in the west direction
 * (first west of second) then the result will be negative.
 *
 * @param first_rad Longitude in radians to subtract from.
 * @param second_rad Longitude in radians to subtract first operator with.
 *
 * @return Difference (first - second) in radians. Positive result is east,
 * negative result is west.
 */
double diff_lon(const double first_rad, const double second_rad);

/**
 * @brief Convert from WGS84 ECEF to WGS84 geodetic coordinates
 *
 * @details Converts from WGS84 Earth Centered, Earth Fixed (ECEF) Cartesian
 * coordinates (X, Y and Z) into WGS84 geodetic coordinates (latitude,
 * longitude and height).
 *
 * Conversion from Cartesian to geodetic coordinates is a much harder problem
 * than conversion from geodetic to Cartesian. There is no satisfactory closed
 * form solution but many different iterative approaches exist.
 *
 * Here we implement a relatively new algorithm due to Fukushima (2006) that is
 * very computationally efficient, not requiring any transcendental function
 * calls during iteration and very few divisions. It also exhibits cubic
 * convergence rates compared to the quadratic rate of convergence seen with
 * the more common algortihms based on the Newton-Raphson method.
 *
 * References:
 *   -# "A comparison of methods used in rectangular to Geodetic Coordinates
 *      Transformations", Burtch R. R. (2006), American Congress for Surveying
 *      and Mapping Annual Conference. Orlando, Florida.
 *   -# "Transformation from Cartesian to Geodetic Coordinates Accelerated by
 *      Halley’s Method", T. Fukushima (2006), Journal of Geodesy.
 *
 * @param ecef Cartesian coordinates to be converted in meters.
 *
 * @return Converted geodetic coordinates in [radians, radians, meters].
 */
LatLonHgt wgs_ecef_to_llh(const std::array<double, 3> &ecef);

/**
 * @brief Get Elevation and Azimuth from WGS84 coordinates, in radians.
 *
 * @details Determine the azimuth and elevation of a point in WGS84 Earth
 * Centered, Earth Fixed (ECEF) Cartesian coordinates from a reference point
 * given in WGS84 ECEF coordinates.
 *
 * First the vector between the points is converted into the local North, East,
 * Down frame of the reference point. Then we can directly calculate the
 * azimuth and elevation.
 *
 * @param ecef      Cartesian coordinates of the point, passed as [X, Y, Z],
 *                  all in meters.
 * @param ref_ecef  Cartesian coordinates of the reference point from which the
 *                  azimuth and elevation is to be determined, passed as
 *                  [X, Y, Z], all in meters.
 * @param azimuth   Pointer to where to store the calculated azimuth output (in
 *                  radians). Must not be NULL.
 * @param elevation Pointer to where to store the calculated elevation output
 *                  (in radians). Must not be NULL.
 */
void wgs_ecef_to_azel(const std::array<double, 3> &ecef,
                      const std::array<double, 3> &ref_ecef, double *azimuth,
                      double *elevation);

/**
 * @brief Get NED coordinates
 *
 * @details Returns a vector to a point given in WGS84 Earth Centered, Earth
 * Fixed (ECEF) Cartesian coordinates from a reference point, also given in
 * WGS84 ECEF coordinates, in the local North, East, Down (NED) frame of the
 * reference point.
 *
 * @param ecef      Cartesian coordinates of the point, passed as [X, Y, Z],
 *                  all in meters.
 * @param ref_ecef  Cartesian coordinates of the reference point, passed as
 *                  [X, Y, Z], all in meters.
 *
 * @return ned      The North, East, Down vector is written into this array as
 *                  [N, E, D], all in meters.
 */
std::array<double, 3> wgs_ecef_to_ned(const std::array<double, 3> &ecef,
                                      const std::array<double, 3> &ref_ecef);

/**
 * @brief Returns rotation matrix from ECEF to NED at at an ECEF position.
 * @param llh ECEF position provided as WGS84 latitude, longitude, altitude.
 * @return The Direction Cosine Matrix ECEF -> NED
 */
std::array<std::array<double, 3>, 3> wgs_ecef_to_ned_matrix(
    const LatLonHgt &llh);

/**
 * @brief Resolves a vector given in the ECEF frame in the satellite's
 * body frame.
 *
 * @details The satellite's frame in relation to ECEF has the satellite z-axis
 * pointing towards the center of the earth (ECEF origin).
 * The satellite yaw angle defines rotation around the satellite z-axis.
 * For zero yaw, the satellite x-axis points in the direction
 * of the satellite velocity vector.
 * The satellite frame in this case is as referenced in RTCM DF480.
 *
 * @param vec_ecef 3D vector in the ECEF frame [X, Y, Z]
 * @param p_sat_ecef 3D vector of satellite position in ECEF [X, Y, Z]
 * @param v_sat_ecef 3D vector of satellite velocity in ECEF [X, Y, Z]
 * @param sat_yaw_rad Yaw angle of satellite (rotation around it's z-axis), in
 * radians.
 *
 * @return 3D vector of vec_ecef transformed into the satellite coordinate
 * frame.
 */
std::array<double, 3> ecef_to_satframe(const std::array<double, 3> &vec_ecef,
                                       const std::array<double, 3> &p_sat_ecef,
                                       const std::array<double, 3> &v_sat_ecef,
                                       const double sat_yaw_rad);

/**
 * @brief Returns the Direction Cosine Matrix R that resolves a vector given in
 * the ECEF frame in the satellite's body frame.
 *
 * @example v_sat = R * v_ecef
 *
 * @details The satellite's frame in relation to ECEF has the satellite z-axis
 * pointing towards the center of the earth (ECEF origin).
 * For zero yaw, the satellite x-axis points in the general direction
 * of the satellite velocity vector.
 * The satellite yaw angle defines an additional rotation around the satellite
 * z-axis.
 * This satellite frame is the phase-windup frame described in
 * RTCM-3 SSR stage 2 v07 draft of DF480.
 *
 * @param vec_ecef 3D vector in the ECEF frame [X, Y, Z]
 * @param p_sat_ecef 3D vector of satellite position in ECEF [X, Y, Z]
 * @param v_sat_ecef 3D vector of satellite velocity in ECEF [X, Y, Z]
 * @param sat_yaw_rad Yaw angle of satellite (rotation around it's z-axis), in
 * radians.
 *
 * @return DCM ecef_to_satframe as an array of row vectors.
 *
 */
std::array<std::array<double, 3>, 3> dcm_ecef_to_satframe(
    const std::array<double, 3> &p_sat_ecef,
    const std::array<double, 3> &v_sat_ecef, const double sat_yaw_rad);

}  // namespace internal
}  // namespace swift

#endif  // SWIFT_SSR2LOS_SRC_INTERNAL_TYPES_COORD_SYSTEM_H
