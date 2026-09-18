#ifndef ATTITUDECONTROLLER_H
#define ATTITUDECONTROLLER_H

#include <memory>
#include "Automation/iAttitudeControl.h"

class AttitudeController : public IAttitudeControl
{
public:
    explicit AttitudeController(std::unique_ptr<IController> controller) : controller_(std::move(controller))
    {
    }

    double killRotationOfOneAxis(const double& omega, const double& dt) const override;

private:
    std::unique_ptr<IController> controller_;
};

#endif // ATTITUDECONTROLLER_H
