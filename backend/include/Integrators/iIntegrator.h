#pragma once

#include <eigen3/Eigen/Dense>

/**
 * @class IIntegrator
 * @brief Interface for numerical time integration of motion equations.
 *
 * IIntegrator defines the contract for all numerical integrators used
 * to advance the state of an object in time. Implementations are purely
 * mathematical and must not contain any physics-specific logic such as
 * gravity, thrust, or mass handling.
 *
 * Typical implementations are Euler, Semi-Implicit Euler, Verlet or RK4.
 * The integrator operates on position, velocity and acceleration vectors
 * and a discrete timestep.
 */
class IIntegrator {
public:

    /**
     * @brief Virtual destructor to allow proper cleanup of derived classes.
     */
    virtual ~IIntegrator() = default;

    /**
     * @brief Integrates velocity over a timestep.
     *
     * Computes the new velocity based on the current velocity,
     * acceleration and timestep.
     *
     * @param vel Current velocity vector.
     * @param acc Current acceleration vector.
     * @param dt  Time step in seconds.
     * @return Updated velocity vector after dt.
     */
    virtual Eigen::Vector3d integrateVel(const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double dt) const = 0;

    /**
     * @brief Integrates position over a timestep.
     *
     * Computes the new position based on the current position,
     * velocity, acceleration and timestep.
     *
     * @param pos Current position vector.
     * @param vel Current velocity vector.
     * @param acc Current acceleration vector.
     * @param dt  Time step in seconds.
     * @return Updated position vector after dt.
     */
    virtual Eigen::Vector3d integratePos(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double dt) const = 0;
};
