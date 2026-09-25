#include "Automation/attitudeController.h"
#include "Controller/pd_controller.h"

#include <algorithm>
#include <cmath>

AttitudeController::AttitudeController() : controllerX_(std::make_unique<PD_Controller>()), controllerY_(std::make_unique<PD_Controller>()), controllerZ_(std::make_unique<PD_Controller>()), stabController_(std::make_unique<PD_Controller>())
{

}

Eigen::Vector3d AttitudeController::killRotation(const Eigen::Vector3d& omega, const double& dt) const
{
    constexpr double omegaTolerance = 0.005; // rad/s

    Eigen::Vector3d rotationCmd{0.0, 0.0, 0.0};
    if (std::abs(omega.x()) > omegaTolerance) rotationCmd.x() = controllerX_->control(0.0, omega.x(), 1.0, 0.0, dt);
    if (std::abs(omega.y()) > omegaTolerance) rotationCmd.y() = controllerY_->control(0.0, omega.y(), 1.0, 0.0, dt);
    if (std::abs(omega.z()) > omegaTolerance) rotationCmd.z() = controllerZ_->control(0.0, omega.z(), 1.0, 0.0, dt);

    return rotationCmd;
}

Eigen::Vector3d AttitudeController::stabilize(const Eigen::Quaterniond& currentOrientation, const Eigen::Vector3d& angularVelocity)
{
    constexpr double attitudeToleranceOff = 0.5 * 3.14159265358979323846 / 180.0; // rad
    constexpr double attitudeToleranceOn  = 1.0 * 3.14159265358979323846 / 180.0; // rad
    constexpr double omegaToleranceOff    = 0.005;                                // rad/s
    constexpr double omegaToleranceOn     = 0.010;                                // rad/s

    const Eigen::Quaterniond currentNormalized = currentOrientation.normalized();

    if (!stabilizeInitialized_)
    {
        targetOrientation_ = currentNormalized;
        stabilizeInitialized_ = true;
        stabilizeCorrectionActive_ = true;
    }

    Eigen::Quaterniond qError =
        targetOrientation_.conjugate() * currentNormalized;

    if (qError.w() < 0.0)
        qError.coeffs() *= -1.0;

    const double qErrorScalar =
        std::clamp(qError.w(), -1.0, 1.0);

    const double attitudeErrorAngle =
        2.0 * std::acos(qErrorScalar);

    const double maxAngularVelocity =
        angularVelocity.cwiseAbs().maxCoeff();

    if (stabilizeCorrectionActive_)
    {
        const bool attitudeSettled =
            attitudeErrorAngle < attitudeToleranceOff;

        const bool angularVelocitySettled =
            maxAngularVelocity < omegaToleranceOff;

        if (attitudeSettled && angularVelocitySettled)
            stabilizeCorrectionActive_ = false;
    }
    else
    {
        const bool attitudeOutsideHoldBand =
            attitudeErrorAngle > attitudeToleranceOn;

        const bool angularVelocityOutsideHoldBand =
            maxAngularVelocity > omegaToleranceOn;

        if (attitudeOutsideHoldBand || angularVelocityOutsideHoldBand)
            stabilizeCorrectionActive_ = true;
    }

    if (!stabilizeCorrectionActive_)
        return Eigen::Vector3d::Zero();

    return stabController_->controlQuaternion(
        targetOrientation_,
        currentNormalized,
        angularVelocity,
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0});
}

void AttitudeController::deactivateStabilize()
{
    stabilizeInitialized_ = false;
    stabilizeCorrectionActive_ = true;
}