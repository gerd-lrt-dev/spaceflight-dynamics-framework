#ifndef ATTITUDECONTROLLER_H
#define ATTITUDECONTROLLER_H

#include <memory>
#include "Automation/iAttitudeControl.h"

class AttitudeController : public IAttitudeControl
{
public:
    AttitudeController();

    Eigen::Vector3d killRotation(const Eigen::Vector3d& omega, const double& dt) const override;

private:
    std::unique_ptr<IController> controllerX_;
    std::unique_ptr<IController> controllerY_;
    std::unique_ptr<IController> controllerZ_;
};

#endif // ATTITUDECONTROLLER_H
