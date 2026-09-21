#ifndef ATTITUDECONTROLLER_H
#define ATTITUDECONTROLLER_H

#include <memory>
#include "Automation/iAttitudeControl.h"

class AttitudeController : public IAttitudeControl
{
public:
    explicit AttitudeController(std::unique_ptr<IController> controller) : controllerX_(std::move(controller)), controllerY_(std::move(controller)), controllerZ_(std::move(controller))
    {
    }

    Eigen::Vector3d killRotation(const Eigen::Vector3d& omega, const double& dt) const override;

private:
    std::unique_ptr<IController> controllerX_;
    std::unique_ptr<IController> controllerY_;
    std::unique_ptr<IController> controllerZ_;
};

#endif // ATTITUDECONTROLLER_H
