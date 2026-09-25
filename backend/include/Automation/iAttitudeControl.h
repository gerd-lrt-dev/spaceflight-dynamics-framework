#ifndef IATTITUDECONTROL_H
#define IATTITUDECONTROL_H

#include "Controller/iController.h"
#include "Eigen/Dense"

/**
 * @class IAttitudeControl
 * @brief Abstract interface for automatic spacecraft attitude-control modes.
 *
 * Defines the contract for attitude-control functions that generate normalized
 * rotational control commands for the spacecraft RCS system.
 *
 * Implementations may provide different automatic rotational modes, while the
 * caller remains responsible for selecting which mode currently owns rotational
 * control authority.
 */
class IAttitudeControl
{
public:
    /**
     * @brief Virtual destructor for safe polymorphic deletion.
     */
    virtual ~IAttitudeControl() = default;

    /**
     * @brief Damp the spacecraft angular velocity toward zero.
     *
     * Generates a three-axis rotational command based exclusively on the current
     * body-frame angular velocity. The method does not attempt to recover or hold
     * a specific spacecraft attitude.
     *
     * @param omega Current angular velocity expressed in SBF [rad/s].
     * @param dt Discrete simulation timestep [s].
     * @return Three-axis rotational control command for the RCS allocator.
     */
    virtual Eigen::Vector3d killRotation(
        const Eigen::Vector3d& omega,
        const double& dt) const = 0;

    /**
     * @brief Hold the spacecraft attitude captured at Stabilize activation.
     *
     * On first activation, an implementation may capture the current attitude as
     * the internal target attitude. Subsequent calls generate a three-axis command
     * that reduces quaternion attitude error while damping angular velocity.
     *
     * @param currentOrientation Current spacecraft attitude quaternion.
     * @param angularVelocity Current angular velocity expressed in SBF [rad/s].
     * @return Three-axis rotational control command for the RCS allocator.
     */
    virtual Eigen::Vector3d stabilize(
        const Eigen::Quaterniond& currentOrientation,
        const Eigen::Vector3d& angularVelocity) = 0;

    /**
     * @brief Reset the internal Stabilize mode state.
     *
     * Clears the stored activation state so that the next Stabilize activation
     * captures a new target attitude.
     */
    virtual void deactivateStabilize() = 0;
};

#endif // IATTITUDECONTROL_H
