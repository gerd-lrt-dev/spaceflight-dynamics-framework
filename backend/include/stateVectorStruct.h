#ifndef STATEVECTORSTRUCT_H
#define STATEVECTORSTRUCT_H

#include "vector3.h"
#include "quaternion.h"
#include "Coordinates/coordinateTransformer.h"

/**
 * @struct StateVector
 * @brief Encapsulates the complete translational and rotational state of the spacecraft.
 *
 * This struct represents the full dynamic state of the spacecraft at any given time.
 * It includes inertial position and velocity, orientation with respect to the inertial frame,
 * angular velocity in the body frame, and the total mass.
 *
 * It is intended to be the single source of truth for the spacecraft's state in the simulation.
 */
struct StateVector
{
    /**
     * @brief Inertial position vector of the spacecraft [m].
     *
     * Expressed in the chosen inertial frame (e.g., Moon-centered inertial frame).
     */
    Vector3 MCI_Position = {0.0, 0.0, 0.0};

    /**
     * @brief Inertial velocity vector of the spacecraft [m/s].
     *
     * Expressed in the chosen inertial frame.
     */
    Vector3 MCI_Velocity = {0.0, 0.0, 0.0};

    /**
     * @brief Orientation quaternion of the spacecraft.
     *
     * Represents the rotation from the body-fixed frame (SBF) to the inertial frame (MCI).
     * Default constructor creates the identity quaternion (no rotation).
     */
    Quaternion IB_Orientation;

    /**
     * @brief Angular velocity vector of the spacecraft [rad/s].
     *
     * Expressed in the body-fixed frame.
     */
    Vector3 SBF_AngularVelocity = {0.0, 0.0, 0.0};

    /**
     * @brief Total mass of the spacecraft [kg].
     *
     * Can change during simulation if propellant consumption is modeled.
     */
    double totalMass = 0.0;
};

#endif // STATEVECTORSTRUCT_H
