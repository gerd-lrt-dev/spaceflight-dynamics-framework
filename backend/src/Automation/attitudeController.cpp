#include "Automation/attitudeController.h"

double AttitudeController::killRotationOfOneAxis(const double& omega, const double& dt) const
{
    return controller_->control(0.0, omega, 1.0, 0.0, dt);
}