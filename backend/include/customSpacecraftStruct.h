#ifndef CUSTOMSPACECRAFTSTRUCT_H
#define CUSTOMSPACECRAFTSTRUCT_H

#include "vector3.h"
#include "Thrust/EngineConfig.h"
#include "Thrust/FueltankStruct.h"

#include <vector>

/**
 * @struct customSpacecraft
 * @brief Defines all physical and configuration parameters of a spacecraft.
 *
 * All parameters are expressed in the spacecraft's body reference frame (B-frame),
 * unless explicitly stated otherwise. The struct contains mass properties,
 * propulsion characteristics, attitude-related inertial properties, and
 * frame-initialization parameters.
 */
struct customSpacecraft
{
    // -------------------------------------------------------------------------
    // Mass Properties
    // -------------------------------------------------------------------------

    double emptyMass;
    ///< [kg] Dry mass of the spacecraft (structure + engines, no fuel).

    double fuelM;
    ///< [kg] Initial fuel mass at simulation start.

    double maxFuelM;
    ///< [kg] Maximum fuel tank capacity (optional; used for validation or HUD).

    // -------------------------------------------------------------------------
    // Propulsion System
    // -------------------------------------------------------------------------

    /**
     * @brief List of engine configurations for the spacecraft.
     *
     * Each element in this vector represents a single engine of the spacecraft,
     * including its thrust capabilities, specific impulse, orientation, and
     * dynamic response properties.
     *
     * Examples of engine types:
     * - Main descent engine
     * - Attitude control thrusters (RCS)
     *
     * Access example:
     * @code
     * for(const auto& eng : spacecraft.engines_) {
     *     double thrust = eng.maxThrust;
     *     Vector3 dir = eng.direction;
     * }
     * @endcode
     */
    std::vector<EngineConfig> engines_;

    // -------------------------------------------------------------------------
    // Tank System
    // -------------------------------------------------------------------------

    std::vector<FuelTank> tanks_;

    // -------------------------------------------------------------------------
    // Attitude Dynamics (Rigid Body Inertia)
    // -------------------------------------------------------------------------

    double Ixx;
    ///< [kg·m²] Moment of inertia about the spacecraft’s body X-axis.

    double Iyy;
    ///< [kg·m²] Moment of inertia about the body Y-axis.

    double Izz;
    ///< [kg·m²] Moment of inertia about the body Z-axis.
    ///< Assumed diagonal inertia tensor (symmetric spacecraft).


    // -------------------------------------------------------------------------
    // State (Body Frame Coordinates)
    // -------------------------------------------------------------------------

    Vector3 I_initialPos;
    ///< [m] Initial spacecraft position expressed in body coordinates.
    ///< Typically initial (0, 0, 0) because physics handles world/Moon coordinates.

    Vector3 I_initialVelocity;
    ///< [m/s] Velocity in three spatial directions

    Vector3 B_initialRot;
    ///< [rad] Initial orientation (pitch, yaw, roll) in body coordinates.

    Vector3 B_initialCenterOfMass;
    ///< [m] Center of mass location in body frame at t=0.
    ///< Should be updated dynamically if fuel mass distribution changes.

    // -------------------------------------------------------------------------
    // Integrity & Operational Status
    // Attributes controlling spacecraft health, integrity, and operational status.
    // -------------------------------------------------------------------------

    double       structuralIntegrity;       ///< [%] Threshold at which the spacecraft becomes non-operational
    double       safeVelocity;              ///< [m/s] Velocity in which is the spacecraft able to land safely
};

#endif // CUSTOMSPACECRAFTSTRUCT_H
