#ifndef IATTITUDECONTROL_H
#define IATTITUDECONTROL_H
#include "Controller/iController.h"


class IAttitudeControl{
public:
    virtual ~IAttitudeControl() = default;

    virtual double killRotationOfOneAxis(const double& omega, const double& dt) const = 0;
};

#endif // IATTITUDECONTROL_H
