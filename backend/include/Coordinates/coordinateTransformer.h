#pragma once

#include "vector3.h"
#include "quaternion.h"

/**
 * @class CoordinateTransformer
 * @brief Coordinate frame transformation utility for guidance,
 *        navigation, and control systems.
 *
 * This class provides deterministic transformations between the
 * coordinate frames used throughout the simulation framework.
 *
 * The transformer itself is stateless and does not perform any
 * numerical integration or state propagation. Its sole purpose is
 * to convert vectors and coordinate representations between
 * physically meaningful reference frames.
 *
 * Implemented frame hierarchy:
 *
 * MCI
 *  └── Moon-Centered Inertial frame
 *      Used for orbital propagation and inertial dynamics
 *
 * MCMF
 *  └── Moon-Centered Moon-Fixed rotating frame
 *      Rotates with the lunar surface
 *
 * MSC
 *  └── Moon surface coordinates
 *      Latitude / longitude / altitude representation
 *
 * ENU
 *  └── East-North-Up local tangent frame
 *      Used for landing and surface-relative navigation
 *
 * LVLH
 *  └── Local Vertical Local Horizontal frame
 *      Used for orbital-relative guidance
 *
 * SBF
 *  └── Spacecraft Body Frame
 *      Used for spacecraft attitude and body dynamics
 *
 * Coordinate conventions:
 *
 * - All frames are right-handed
 * - Angular quantities are expressed in radians
 * - Altitudes are expressed in meters
 * - The Moon is currently modeled as a perfect sphere
 * - Quaternion convention:
 *
 *      q_IB : inertial -> body
 *
 *      v_body = q_IB * v_inertial * q_IB^-1
 *
 * This class intentionally separates inertial and rotating frames
 * to support physically consistent planetary navigation and future
 * orbital expansion.
 */

class CoordinateTransformer {
public:

    /**
     * @brief Cartesian position and velocity state representation.
     *
     * Both vectors are expressed in the same reference frame.
     *
     * @param position Position vector [m]
     * qparam velocity Velocity vector [m/s]
     */
    struct State {
        Vector3 position;
        Vector3 velocity;
    };

    /**
     * @brief Moon surface coordinate representation.
     *
     * The coordinates describe a point relative to the lunar surface.
     *
     * @param latitude and longitude are geocentric and expressed in radians.
     * @param altitude is measured relative to the reference lunar radius in meters.
     */
    struct MoonSurfaceCoordinates {
        double latitude;
        double longitude;
        double altitude;
    };

    /**
     * @brief East-North-Up local tangent frame basis vectors.
     *
     * The frame is locally tangent to the lunar surface and is used
     * primarily for landing guidance and surface-relative navigation.
     *
     * Coordinate convention:
     *
     * x -> east
     * y -> north
     * z -> up
     *
     * @param east local east direction
     * @param north local north direction
     * @param up local surface normal
     */
    struct ENUFrame {
        Vector3 east;
        Vector3 north;
        Vector3 up;
    };

    /**
     * @brief Local Vertical Local Horizontal frame basis vectors.
     *
     * The LVLH frame is defined relative to the spacecraft orbit.
     *
     * Coordinate convention:
     *
     * x -> forward (velocity direction)
     * y -> right
     * z -> down (toward central body)
     *
     * @param forward velocity-aligned direction
     * @param right orbit-normal aligned direction
     * @param down nadir direction
     */
    struct LVLHFrame {
        Vector3 forward;
        Vector3 right;
        Vector3 down;
    };

    CoordinateTransformer();

    ~CoordinateTransformer();

    /**
     * @brief Transforms a vector from MCI into MCMF coordinates.
     *
     * The transformation accounts for lunar rotation about the
     * inertial z-axis.
     *
     * @param positionMCI Vector expressed in MCI coordinates
     * @param moonRotationAngle Lunar rotation angle [rad]
     * @return Vector expressed in MCMF coordinates
     */
    Vector3 mciToMcmf(
        const Vector3& positionMCI,
        double moonRotationAngle) const;

    /**
     * @brief Transforms a vector from MCMF into MCI coordinates.
     *
     * @param positionMCMF Vector expressed in MCMF coordinates
     * @param moonRotationAngle Lunar rotation angle [rad]
     * @return Vector expressed in MCI coordinates
     */
    Vector3 mcmfToMci(
        const Vector3& positionMCMF,
        double moonRotationAngle) const;

    /**
     * @brief Converts MCMF Cartesian coordinates into
     *        moon surface coordinates.
     *
     * @param positionMCMF Position expressed in MCMF coordinates
     * @return Surface coordinate representation
     */
    MoonSurfaceCoordinates mcmfToMSC(
        const Vector3& positionMCMF) const;

    /**
     * @brief Converts moon surface coordinates into
     *        MCMF Cartesian coordinates.
     *
     * @param msc Surface coordinates
     * @return Cartesian position in MCMF coordinates
     */
    Vector3 mscToMcmf(
        const MoonSurfaceCoordinates& msc) const;

    /**
     * @brief Computes the local ENU frame at a surface location.
     *
     * The resulting basis vectors form an orthonormal local tangent
     * frame aligned with the lunar surface.
     *
     * @param positionMCMF Surface position in MCMF coordinates
     * @return ENU frame basis vectors
     */
    ENUFrame computeENUFrame(
        const Vector3& positionMCMF) const;

    /**
     * @brief Transforms a vector from ENU coordinates into MCMF.
     *
     * @param vectorENU Vector expressed in ENU coordinates
     * @param enu ENU frame basis
     * @return Vector expressed in MCMF coordinates
     */
    Vector3 enuToMcmf(
        const Vector3& vectorENU,
        const ENUFrame& enu) const;

    /**
     * @brief Transforms a vector from MCMF coordinates into ENU.
     *
     * @param vectorMCMF Vector expressed in MCMF coordinates
     * @param enu ENU frame basis
     * @return Vector expressed in ENU coordinates
     */
    Vector3 mcmfToEnu(
        const Vector3& vectorMCMF,
        const ENUFrame& enu) const;

    /**
     * @brief Computes the LVLH frame from an inertial state.
     *
     * The frame orientation is derived from the spacecraft position
     * and velocity vectors.
     *
     * @param stateMCI Spacecraft inertial state
     * @return LVLH frame basis vectors
     */
    LVLHFrame computeLVLHFrame(
        const State& stateMCI) const;

    /**
     * @brief Transforms a vector from LVLH into MCI coordinates.
     *
     * @param vectorLVLH Vector expressed in LVLH coordinates
     * @param lvlh LVLH frame basis
     * @return Vector expressed in MCI coordinates
     */
    Vector3 lvlhToMci(
        const Vector3& vectorLVLH,
        const LVLHFrame& lvlh) const;

    /**
     * @brief Transforms a vector from MCI into LVLH coordinates.
     *
     * @param vectorMCI Vector expressed in MCI coordinates
     * @param lvlh LVLH frame basis
     * @return Vector expressed in LVLH coordinates
     */
    Vector3 mciToLvlh(
        const Vector3& vectorMCI,
        const LVLHFrame& lvlh) const;

    /**
     * @brief Transforms a vector from MCI into body-frame coordinates.
     *
     * Quaternion convention:
     *
     * q_IB : inertial -> body
     *
     * @param vectorMCI Vector expressed in MCI coordinates
     * @param q_IB Inertial-to-body quaternion
     * @return Vector expressed in body coordinates
     */
    Vector3 mciToSbf(
        const Vector3& vectorMCI,
        const Quaternion& q_IB) const;


    /**
     * @brief Transforms a vector from body-frame coordinates into MCI.
     *
     * @param vectorBody Vector expressed in body coordinates
     * @param q_IB Inertial-to-body quaternion
     * @return Vector expressed in MCI coordinates
     */
    Vector3 sbfToMci(
        const Vector3& vectorBody,
        const Quaternion& q_IB) const;

private:

    /// Mean lunar reference radius [m]
    static constexpr double R_MOON = 1737400.0;

    //// Lunar sidereal rotation rate [rad/s]
    static constexpr double MOON_ROTATION_RATE =
        2.6617e-6;
};