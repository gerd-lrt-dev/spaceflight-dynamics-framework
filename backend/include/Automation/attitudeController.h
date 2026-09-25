#ifndef ATTITUDECONTROLLER_H
#define ATTITUDECONTROLLER_H

#include <memory>
#include "Automation/iAttitudeControl.h"

class AttitudeController : public IAttitudeControl
{
public:
    AttitudeController();

    Eigen::Vector3d killRotation(const Eigen::Vector3d& omega, const double& dt) const override;

    Eigen::Vector3d stabilize(const Eigen::Quaterniond& currentOrientation, const Eigen::Vector3d& angularVelocity) override;

    void deactivateStabilize() override;


private:
    std::unique_ptr<IController> controllerX_;
    std::unique_ptr<IController> controllerY_;
    std::unique_ptr<IController> controllerZ_;

    std::unique_ptr<IController> stabController_;

    Eigen::Quaterniond targetOrientation_{1.0, 0.0, 0.0, 0.0};
    bool stabilizeInitialized_ = false;
};

#endif // ATTITUDECONTROLLER_H
