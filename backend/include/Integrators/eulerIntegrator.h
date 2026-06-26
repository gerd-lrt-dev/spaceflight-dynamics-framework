#pragma once

#include "Integrators/iIntegrator.h"

#include <eigen3/Eigen/Dense>

/**
 * @class EulerIntegrator
 * @brief Concrete integrator implementing the explicit Euler method.
 *
 * The EulerIntegrator advances position and velocity using a first–order
 * explicit Euler scheme assuming constant acceleration within a timestep.
 *
 * This class is purely numerical and independent of any physical model.
 * It operates only on vectors and time deltas.
 */
class EulerIntegrator : public IIntegrator {
public:

    /**
     * @brief Integrates velocity over a timestep using Euler integration.
     *
     * Formula:
     * v(t + dt) = v(t) + a(t) * dt
     *
     * @param vel Current velocity vector.
     * @param acc Current acceleration vector.
     * @param dt  Time step in seconds.
     * @return Updated velocity vector after dt.
     */
    virtual Eigen::Vector3d integrateVel(const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double dt) const override;

    /**
     * @brief Integrates position over a timestep using Euler integration.
     *
     * Formula:
     * r(t + dt) = r(t) + v(t) * dt + 0.5 * a(t) * dt^2
     *
     * Assumes constant acceleration during the timestep.
     *
     * @param pos Current position vector.
     * @param vel Current velocity vector.
     * @param acc Current acceleration vector.
     * @param dt  Time step in seconds.
     * @return Updated position vector after dt.
     */
    virtual Eigen::Vector3d integratePos(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double dt) const override;
};
