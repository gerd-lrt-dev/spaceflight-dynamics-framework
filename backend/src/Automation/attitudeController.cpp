#include "Automation/attitudeController.h"
#include "Controller/pd_controller.h"

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
    if (!stabilizeInitialized_)
    {
        targetOrientation_ = currentOrientation.normalized();
        stabilizeInitialized_ = true;
    }

    return stabController_->controlQuaternion(
        targetOrientation_,
        currentOrientation,
        angularVelocity,
        {1.0, 1.0, 1.0},
        {1.0, 1.0, 1.0});
}

void AttitudeController::deactivateStabilize()
{
    stabilizeInitialized_ = false;
}