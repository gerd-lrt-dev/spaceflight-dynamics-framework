#pragma once

#include "vector3.h"
#include "quaternion.h"

/**
 * @class CoordinateTransformer
 * @brief Provides coordinate transformations between multiple lunar and
 * spacecraft reference frames.
 *
 * This class implements transformations between inertial, Moon-fixed,
 * surface-relative, orbital-relative, and spacecraft body-fixed coordinate
 * systems used in lunar spacecraft simulation.
 *
 * Supported reference frames:
 *
 * - MCI  : Moon-Centered Inertial frame
 * - MCMF : Moon-Centered Moon-Fixed rotating frame
 * - MSC  : Moon surface geodetic coordinates
 * - ENU  : East-North-Up local tangent frame
 * - LVLH : Local Vertical Local Horizontal orbital frame
 * - SBF  : Spacecraft Body Frame
 *
 * The MCI frame is treated as the fundamental inertial physics frame of the
 * simulation. Other frames are derived from MCI states for purposes such as
 * surface navigation, local guidance, rendering, docking, and spacecraft
 * attitude operations.
 */
class CoordinateTransformer {
public:

    /**
     * @struct State
     * @brief Represents a translational state in a reference frame.
     *
     * The state contains Cartesian position and velocity vectors expressed
     * in the same coordinate frame.
     */
    struct State {
        Vector3 position;
        Vector3 velocity;
    };

    /**
     * @struct MoonSurfaceCoordinates
     * @brief Represents lunar surface geodetic coordinates.
     *
     * Latitude and longitude are expressed in radians.
     * Altitude is measured above the mean lunar reference radius.
     */
    struct MoonSurfaceCoordinates {
        double latitude;
        double longitude;
        double altitude;
    };

    /**
     * @struct ENUFrame
     * @brief Represents a local East-North-Up tangent frame.
     *
     * The frame is attached to a fixed location on the lunar surface and is
     * commonly used for local navigation, landing guidance, terrain-relative
     * operations, and surface rendering.
     */
    struct ENUFrame {
        Vector3 east;
        Vector3 north;
        Vector3 up;

        State origin;
    };

    /**
     * @struct LVLHFrame
     * @brief Represents a Local Vertical Local Horizontal orbital frame.
     *
     * The frame is centered on an orbiting spacecraft and rotates with the
     * orbital trajectory. It is commonly used for rendezvous, docking,
     * relative navigation, and local orbital guidance.
     */
    struct LVLHFrame {
        Vector3 forward;
        Vector3 right;
        Vector3 down;

        State origin;
    };

    /**
     * @struct SpacecraftBodyFrame
     * @brief Represents the spacecraft body-fixed frame.
     *
     * The frame is rigidly attached to the spacecraft structure and rotates
     * with the vehicle attitude. It is primarily used for thrust vectors,
     * control systems, inertia tensors, torque calculations, and onboard
     * sensors.
     */
    struct SpacecraftBodyFrame{
        Quaternion orientation;

        State origin;
    };


    /**
     * @brief Constructs a CoordinateTransformer instance.
     */
    CoordinateTransformer();

    /**
     * @brief Destroys the CoordinateTransformer instance.
     */
    ~CoordinateTransformer();

    /**
     * @brief Transforms a state from MCI to MCMF coordinates.
     *
     * Converts an inertial Moon-centered state into the rotating
     * Moon-fixed frame by applying lunar rotation.
     *
     * @param inertial State expressed in the MCI frame.
     * @param t Simulation time in seconds.
     * @return State expressed in the MCMF frame.
     */
    State MCItoMCMF(
        const State& inertial,
        double t) const;

    /**
     * @brief Transforms a state from MCMF to MCI coordinates.
     *
     * Converts a Moon-fixed rotating-frame state into the inertial
     * Moon-centered frame.
     *
     * @param fixed State expressed in the MCMF frame.
     * @param t Simulation time in seconds.
     * @return State expressed in the MCI frame.
     */
    State MCMFtoMCI(
        const State& fixed,
        double t) const;

    /**
     * @brief Converts a Moon-fixed Cartesian state into surface coordinates.
     *
     * Computes latitude, longitude, and altitude from a position in the
     * MCMF frame.
     *
     * @param fixed State expressed in the MCMF frame.
     * @return Lunar surface coordinates corresponding to the position.
     */
    MoonSurfaceCoordinates MCMFtoMSC(
        const State& fixed) const;

    /**
     * @brief Converts lunar surface coordinates into an MCMF state.
     *
     * Computes the corresponding Moon-fixed Cartesian position from
     * latitude, longitude, and altitude.
     *
     * The returned velocity is zero in the Moon-fixed frame.
     *
     * @param msc Lunar surface coordinates.
     * @return State expressed in the MCMF frame.
     */
    State MSCtoMCMF(
        const MoonSurfaceCoordinates& msc) const;

    /**
     * @brief Computes a local ENU frame from an MCMF state.
     *
     * Constructs a local tangent frame whose origin is attached to the
     * supplied Moon-fixed state.
     *
     * @param fixed Origin state expressed in the MCMF frame.
     * @return Constructed ENU frame.
     */
    ENUFrame computeENUFrame(
        const State& fixed) const;

    /**
     * @brief Transforms a state from ENU coordinates into MCMF coordinates.
     *
     * Converts a local tangent-frame state into the global Moon-fixed frame.
     *
     * @param targetENU State expressed in the ENU frame.
     * @param frame ENU frame definition.
     * @return State expressed in the MCMF frame.
     */
    State ENUtoMCMF(
        const State& targetENU,
        const ENUFrame& frame) const;

    /**
     * @brief Transforms a state from MCMF coordinates into ENU coordinates.
     *
     * Converts a Moon-fixed state into a local tangent-frame representation.
     *
     * @param target State expressed in the MCMF frame.
     * @param frame ENU frame definition.
     * @return State expressed in the ENU frame.
     */
    State MCMFtoENU(
        const State& target,
            const ENUFrame& frame) const;


    /**
     * @brief Computes an LVLH frame from an MCI state.
     *
     * Constructs a spacecraft-centered orbital reference frame using the
     * inertial position and velocity vectors.
     *
     * @param stateMCI Spacecraft state expressed in the MCI frame.
     * @return Constructed LVLH frame.
     */
    LVLHFrame computeLVLHFrame(
        const State& stateMCI) const;

    /**
     * @brief Transforms a state from LVLH coordinates into MCI coordinates.
     *
     * Converts a local orbital-frame state into inertial coordinates.
     *
     * @param targetLVLH State expressed in the LVLH frame.
     * @param frame LVLH frame definition.
     * @return State expressed in the MCI frame.
     */
    State LVLHtoMCI(
        const State& targetLVLH,
        const LVLHFrame& frame) const;

    /**
     * @brief Transforms a state from MCI coordinates into LVLH coordinates.
     *
     * Converts an inertial state into a spacecraft-centered orbital frame.
     *
     * @param target State expressed in the MCI frame.
     * @param frame LVLH frame definition.
     * @return State expressed in the LVLH frame.
     */
    State MCItoLVLH(
        const State& target,
        const LVLHFrame& frame) const;


    /**
     * @brief Transforms a state from MCI coordinates into the spacecraft
     * body frame.
     *
     * Converts an inertial state into spacecraft-local coordinates using
     * the spacecraft attitude quaternion.
     *
     * @param inertial State expressed in the MCI frame.
     * @param body Spacecraft body frame definition.
     * @return State expressed in the spacecraft body frame.
     */
    State MCItoSBF(
        const State& inertial,
        const SpacecraftBodyFrame& body) const;

    /**
     * @brief Transforms a state from the spacecraft body frame into MCI
     * coordinates.
     *
     * Converts a spacecraft-local state into inertial coordinates using
     * the spacecraft attitude quaternion.
     *
     * @param bodyState State expressed in the spacecraft body frame.
     * @param body Spacecraft body frame definition.
     * @return State expressed in the MCI frame.
     */
    State SBFtoMCI(
        const State& bodyState,
        const SpacecraftBodyFrame& body) const;

    /**
     * @brief Transforms a Vector3 from MCI coordinates into the spacecraft
     * body frame.
     *
     * Converts an acceleration vector into spacecraft-local coordinates using
     * the spacecraft attitude quaternion.
     *
     * @param MCI_acc Vector3 expressed in the MCI frame.
     * @param body Spacecraft body frame definition.
     * @return Vector3 expressed in the spacecraft body frame.
     */
    Vector3 MCIAcctoSBFAcc(
        const Vector3& MCI_acc,
        const SpacecraftBodyFrame& body) const;

    /**
     * @brief Transforms a Vector3 from SBF coordinates into the moon centered inertial frame.
     *
     * Converts an acceleration vector into MCI coordinates using
     * the spacecraft attitude quaternion.
     *
     * @param SBF_acc Vector3 expressed in the SBF frame.
     * @param body Spacecraft body frame definition.
     * @return Vector3 expressed in the moon centered inertial frame.
     */
    Vector3 SBFAcctoMCIAcc(
        const Vector3& SBF_acc,
        const SpacecraftBodyFrame& body) const;

private:

    /// Mean lunar reference radius [m]
    static constexpr double R_MOON = 1737400.0;

    /// Lunar sidereal rotation rate [rad/s]
    static constexpr double MOON_ROTATION_RATE =
        2.6617e-6;

    /// Threshold for detecting near-parallel vectors
    static constexpr double ALIGNMENT_THRESHOLD =
        0.9999;

    /// Threshold for detecting near-zero orbital angular momentum
    static constexpr double
        ANGULAR_MOMENTUM_EPSILON = 1e-8;
};