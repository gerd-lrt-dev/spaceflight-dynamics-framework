#include "Automation/attitudeController.h"

Eigen::Vector3d AttitudeController::killRotation(const Eigen::Vector3d& omega, const double& dt) const
{
    Eigen::Vector3d rotationCmd{0.0, 0.0, 0.0};
    rotationCmd.x() = controllerX_->control(0.0, omega.x(), 1.0, 0.0, dt);
    rotationCmd.y() = controllerY_->control(0.0, omega.y(), 1.0, 0.0, dt);
    rotationCmd.z() = controllerZ_->control(0.0, omega.z(), 1.0, 0.0, dt);

    return rotationCmd;
}