#ifndef IATTITUDECONTROL_H
#define IATTITUDECONTROL_H
#include "Controller/iController.h"
#include "Eigen/Dense"


class IAttitudeControl{
public:
    virtual ~IAttitudeControl() = default;

    virtual Eigen::Vector3d killRotation(const Eigen::Vector3d& omega, const double& dt) const = 0;

    virtual Eigen::Vector3d stabilize(const Eigen::Quaterniond& currentOrientation, const Eigen::Quaterniond& targetOrientation, const Eigen::Vector3d& angularVelocity) const = 0;
};

#endif // IATTITUDECONTROL_H
