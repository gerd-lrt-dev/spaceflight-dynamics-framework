#include "Automation/attitudeController.h"
#include "Controller/pd_controller.h"

AttitudeController::AttitudeController() : controllerX_(std::make_unique<PD_Controller>()), controllerY_(std::make_unique<PD_Controller>()), controllerZ_(std::make_unique<PD_Controller>())
{

}

Eigen::Vector3d AttitudeController::killRotation(const Eigen::Vector3d& omega, const double& dt) const
{
    Eigen::Vector3d rotationCmd{0.0, 0.0, 0.0};
    rotationCmd.x() = controllerX_->control(0.0, omega.x(), 1.0, 0.0, dt);
    rotationCmd.y() = controllerY_->control(0.0, omega.y(), 1.0, 0.0, dt);
    rotationCmd.z() = controllerZ_->control(0.0, omega.z(), 1.0, 0.0, dt);

    return rotationCmd;
}