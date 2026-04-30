#pragma once

#include <vector3.h>

/**
 * @file FlightCommand.h
 * @brief Defines the FlightCommand structure used to control the vehicle.
 */

/**
 * @struct FlightCommand
 * @brief Aggregates all manual and assisted control inputs for the vehicle.
 *
 * This structure represents a high-level command interface between the input layer
 * (e.g., UI, keyboard, controller) and the simulation core. It is intentionally kept
 * independent of any framework (e.g., Qt) to allow usage in a pure C++ backend.
 *
 * All values are interpreted as normalized or physical command inputs depending
 * on the subsystem consuming them.
 *
 * Typical usage:
 * - Populated by input systems (UI, controller, autopilot)
 * - Consumed by simulation core or control systems
 *
 * @note Units and ranges must be consistently interpreted by the simulation core.
 *       This struct does not enforce constraints.
 */
struct FlightCommand
{
    /**
     * @brief Main engine thrust command.
     *
     * Represents the commanded thrust level for the main engine.
     *
     * @note Expected range: [0.0, 100.0] (percentage).
     */
    float mainEngine = 0.0f;

    /**
     * @brief Rotational command (attitude control).
     *
     * Represents desired rotational inputs around the principal axes.
     *
     * Components:
     * - x: pitch (rotation around lateral axis)
     * - y: yaw   (rotation around vertical axis)
     * - z: roll  (rotation around longitudinal axis)
     *
     * @note Units: radians or normalized input depending on implementation.
     *       Must be interpreted consistently by the simulation core.
     */
    Vector3 rotation = {0.0, 0.0, 0.0};

    /**
     * @brief Translational command (RCS linear control).
     *
     * Represents desired translational movement along the principal axes.
     *
     * Components:
     * - x: lateral movement (left/right)
     * - y: vertical movement (up/down)
     * - z: longitudinal movement (forward/backward)
     *
     * @note Typically mapped to RCS thruster groups.
     *       Units are usually normalized input [-1.0, 1.0].
     */
    Vector3 translation = {0.0, 0.0, 0.0};

    /**
     * @brief Stabilization request flag.
     *
     * When set, requests the control system to stabilize the vehicle,
     * typically by damping angular velocities and holding attitude.
     */
    bool stabilize = false;

    /**
     * @brief Kill rotation command flag.
     *
     * When set, requests immediate damping of all rotational motion.
     * Often implemented as an aggressive attitude hold or zero angular velocity controller.
     */
    bool killRotation = false;
};
