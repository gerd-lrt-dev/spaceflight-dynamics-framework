#include "Automation/attitudeController.h"

Eigen::Vector3d AttitudeController::killRotation(const Eigen::Vector3d& omega, const double& dt) const
{
    Eigen::Vector3d F_cmd{0.0, 0.0, 0.0};
    F_cmd.x() = controller_->control(0.0, omega.x(), 1.0, 0.0, dt);
    F_cmd.y() = controller_->control(0.0, omega.y(), 1.0, 0.0, dt);
    F_cmd.z() = controller_->control(0.0, omega.z(), 1.0, 0.0, dt);

    return F_cmd;
}