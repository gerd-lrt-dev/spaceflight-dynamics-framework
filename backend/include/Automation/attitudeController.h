#ifndef ATTITUDECONTROLLER_H
#define ATTITUDECONTROLLER_H

#include <memory>
#include "Automation/iAttitudeControl.h"

/**
 * @class AttitudeController
 * @brief Automatic rotational controller for Kill Rotation and Stabilize modes.
 *
 * The controller provides two distinct attitude-control functions:
 * - Kill Rotation damps the body angular velocity toward zero.
 * - Stabilize captures the spacecraft attitude at activation and subsequently
 *   commands the RCS to return to and hold that reference attitude.
 *
 * The class owns the required controller instances and the internal reference
 * attitude used by Stabilize mode.
 */
class AttitudeController : public IAttitudeControl
{
public:
    /**
     * @brief Construct the attitude controller and its internal PD controllers.
     */
    AttitudeController();

    /**
     * @brief Damp all body-axis angular rates toward zero.
     *
     * Each principal axis is controlled independently. A small angular-rate
     * deadband prevents continuous RCS actuation close to zero rotation.
     *
     * @param omega Current angular velocity expressed in SBF [rad/s].
     * @param dt Discrete simulation timestep [s].
     * @return Three-axis rotational control command.
     */
    Eigen::Vector3d killRotation(
        const Eigen::Vector3d& omega,
        const double& dt) const override;

    /**
     * @brief Hold the attitude captured when Stabilize mode is first activated.
     *
     * The first call after activation stores @p currentOrientation as the target
     * attitude. Later calls use quaternion attitude error and angular-rate damping
     * to generate the rotational command.
     *
     * A hysteresis band is applied around the settled state. Correction is
     * disabled once attitude error and angular velocity fall below the inner
     * thresholds and is only re-enabled after either quantity exceeds the
     * corresponding outer threshold. This reduces RCS chattering with the
     * current binary thruster allocation.
     *
     * @param currentOrientation Current spacecraft attitude quaternion.
     * @param angularVelocity Current angular velocity expressed in SBF [rad/s].
     * @return Three-axis rotational control command.
     */
    Eigen::Vector3d stabilize(
        const Eigen::Quaterniond& currentOrientation,
        const Eigen::Vector3d& angularVelocity) override;

    /**
     * @brief Clear the stored Stabilize activation state.
     *
     * The stored target quaternion is replaced automatically on the next call to
     * stabilize().
     */
    void deactivateStabilize() override;

private:
    std::unique_ptr<IController> controllerX_;   ///< Kill Rotation controller for x-axis.
    std::unique_ptr<IController> controllerY_;   ///< Kill Rotation controller for y-axis.
    std::unique_ptr<IController> controllerZ_;   ///< Kill Rotation controller for z-axis.

    std::unique_ptr<IController> stabController_; ///< Quaternion attitude PD controller.

    Eigen::Quaterniond targetOrientation_{1.0, 0.0, 0.0, 0.0}; ///< Captured Stabilize target attitude.
    bool stabilizeInitialized_ = false; ///< True after the current Stabilize target has been captured.
    bool stabilizeCorrectionActive_ = true; ///< True while Stabilize is actively commanding attitude correction.
};

#endif // ATTITUDECONTROLLER_H
