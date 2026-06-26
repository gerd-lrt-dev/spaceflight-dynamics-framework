#ifndef PHYSICS_H
#define PHYSICS_H

#include <memory>
#include <eigen3/Eigen/Dense>

#include "environmentConfig.h"
#include "spacemath.h"
#include "Physics/iPhysicsModel.h"
#include "Integrators/iIntegrator.h"
#include "Sensory_Perception/iSensor.h"

/**
 * @class physics
 * @brief Orchestrator class coordinating physics model and numerical integration.
 *
 * The physics class acts as a façade and coordination layer between
 * the physical model (IPhysicsModel) and the numerical integrator
 * (IIntegrator). It does not implement physical laws or numerical
 * algorithms itself.
 *
 * Responsibilities:
 * - Query the active physics model for current acceleration.
 * - Delegate position and velocity updates to the configured integrator.
 * - Provide wrapper functions for external systems (e.g. spacecraft)
 *   to keep their interface stable and independent of internal changes.
 *
 * This design enables interchangeable physics models (e.g. Moon, Mars,
 * drag models) and integrators (Euler, RK4, Verlet) without modifying
 * higher-level systems.
 *
 * The class is intentionally lightweight and stateless except for
 * references to the selected model and integrator.
 */
class physics
{
private:
    // Constants & parameters
    EnvironmentConfig configData;
    spacemath math;
    std::shared_ptr<IPhysicsModel> model_;
    std::shared_ptr<IIntegrator> integrator_;
    std::shared_ptr<ISensor> sensor_;
public:
    /**
     * @brief Constructor
     */
    physics(std::shared_ptr<IPhysicsModel> model, std::shared_ptr<IIntegrator> integrator, std::shared_ptr<ISensor> sensor) : model_(model), integrator_(integrator), sensor_(sensor) {};

    /**
     * @brief Destructor
     */
    ~physics();

    /**
     * @brief Computes the current acceleration via the active physics model.
     *
     * Wrapper function delegating the calculation to the configured
     * IPhysicsModel implementation.
     *
     * @param thrust    Current thrust force magnitude.
     * @param mass      Current spacecraft mass.
     * @param thrustDir Normalized thrust direction vector.
     * @param pos       Current position vector.
     * @return Resulting acceleration vector.
     */
    Eigen::Vector3d computeAcc(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double mass, const Eigen::Vector3d& thrust) const;

    /**
     * @brief Integrates velocity using the configured integrator.
     *
     * Wrapper function delegating numerical integration to IIntegrator.
     *
     * @param vel Current velocity vector.
     * @param acc Current acceleration vector.
     * @param dt  Time step in seconds.
     * @return Updated velocity vector.
     */
    Eigen::Vector3d computeVel(const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double dt) const;

    /**
     * @brief Integrates position using the configured integrator.
     *
     * Wrapper function delegating numerical integration to IIntegrator.
     *
     * @param pos Current position vector.
     * @param vel Current velocity vector.
     * @param acc Current acceleration vector.
     * @param dt  Time step in seconds.
     * @return Updated position vector.
     */
    Eigen::Vector3d computePos(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc, double dt) const;

    /**
     * @brief Computes the proper G-load experienced by the spacecraft.
     *
     * This function calculates the proper acceleration (felt acceleration)
     * excluding the gravitational component. The result is expressed in
     * multiples of Earth's standard gravity (g₀ = 9.80665 m/s²).
     *
     * The G-load corresponds to what an ideal accelerometer would measure.
     * It reflects all non-gravitational forces acting on the spacecraft,
     * such as thrust or ground reaction forces.
     *
     * If the spacecraft is landed, the ground reaction force compensates
     * gravitational acceleration, resulting in a non-zero proper G-load
     * even though the total acceleration may be zero (static equilibrium).
     *
     * @param totalAcceleration   Total acceleration acting on the spacecraft [m/s²],
     *                            typically including thrust, gravity, and
     *                            contact forces.
     * @param gravityAcceleration Gravitational acceleration vector [m/s²]
     *                            at the spacecraft's current position.
     * @param isLanded            Indicates whether the spacecraft is in
     *                            ground contact. This may be used to handle
     *                            numerical stabilization or special landing logic.
     *
     * @return Proper G-load in multiples of g₀.
     *
     * @note Proper acceleration is computed as:
     *       \f$ G = \frac{||\vec{a}_\text{total} - \vec{a}_\text{gravity}||}{g_0} \f$
     *
     * @note In free fall: totalAcceleration ≈ gravityAcceleration → G ≈ 0.
     * @note In static equilibrium on a surface: totalAcceleration = 0,
     *       but proper acceleration equals the ground reaction force,
     *       resulting in a non-zero G-load (e.g., ~0.166 g on the Moon).
     *
     * @see Eigen::Vector3d
     */
    double computeGLoad(const Eigen::Vector3d& totalAcceleration, const Eigen::Vector3d& gravityAcceleration, bool isLanded);
};

#endif
