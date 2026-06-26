#pragma once

#include "Physics/iPhysicsModel.h"
#include "environmentConfig.h"

#include <eigen3/Eigen/Dense>

/**
 * @class BasicMoonGravityModel
 * @brief Simple physical model providing lunar gravity and thrust acceleration.
 *
 * This model computes the total acceleration of a spacecraft assuming:
 * - A central gravitational field originating from the moon center.
 * - No atmosphere, drag or perturbations.
 * - A thrust force acting in a given direction.
 *
 * The gravitational field is modeled as an inverse-square radial field.
 * The class is stateless except for the referenced EnvironmentConfig.
 */
class BasicMoonGravityModel : public IPhysicsModel {
public:

    /**
     * @brief Constructs the model with environment configuration data.
     *
     * @param cfg Reference to global environment configuration containing
     *            physical constants such as gravitational parameters.
     */
    BasicMoonGravityModel(const EnvironmentConfig& cfg) : configData(cfg) {}

    /**
     * @brief Computes total acceleration acting on the spacecraft.
     *
     * The resulting acceleration is the sum of gravitational acceleration
     * and thrust acceleration.
     *
     * @param pos        Current position vector relative to moon center.
     * @param vel        Current velocity vector (not used in this basic model
     *                   but kept for interface compatibility).
     * @param mass       Current spacecraft mass.
     * @param thrust     Current thrust force magnitude.
     * @param thrustDir  Normalized thrust direction vector.
     * @return Total acceleration vector in world space.
     */
    Eigen::Vector3d computeAcceleration(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, double mass, const Eigen::Vector3d& thrust) const override;

private:

    //********************************************
    //*********  MEMBERS  ************************
    //********************************************

    /**
     * @brief Reference to environment configuration containing constants.
     */
    const EnvironmentConfig& configData;

    //********************************************
    //*********  METHODS  ************************
    //********************************************

    /**
     * @brief Computes gravitational acceleration toward the moon center.
     *
     * Uses an inverse-square radial gravity model:
     * a = -μ / r² * r̂
     *
     * @param pos Position vector relative to moon center.
     * @return Gravitational acceleration vector.
     */
    Eigen::Vector3d calcAccelerationAlignedToCenterOfMoon(const Eigen::Vector3d& pos) const;
};
