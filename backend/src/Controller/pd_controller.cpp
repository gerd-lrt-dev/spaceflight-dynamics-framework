#include "Controller/pd_controller.h"
#include <iostream>

// ------------------------------------------------
// Public:
// ------------------------------------------------
double PD_Controller::control(const double &targetValue, const double &measuredValue, const double &K_p, const double &K_d, const double &dt) const
{
    double error        = calcError(targetValue, measuredValue);

    double differential = calcDifferential(error, error_old_, dt);

    double P_term = error * K_p;

    double D_term = differential * K_d;

    double controlValue = P_term + D_term;

    return controlValue;
}

double PD_Controller::controlWithOwnDTerm(const double &targetValue, const double &measuredValue, const double differential, const double &K_p, const double &K_d, const double &dt) const
{
    double error        = calcError(targetValue, measuredValue);

    double P_term = error * K_p;

    double D_term = differential * K_d;

    double controlValue = P_term + D_term;

    return controlValue;
}

Eigen::Quaterniond PD_Controller::controlQuaternion(const double& targetValue, const double &currentValue, const double& differential, const Eigen::quaterniond& K_p, const Eigen::quaterniond& K_d) const
{
    Eigen::Quaterniond qError = calcQError(targetValue, currentValue);

    if (qError.w() < 0.0) qError.coeffs() *= -1.0;

    Eigen::Vector3d attitudeError = qError.vec();

    Eigen::Vector3d P_term = - K_p * attitudeError;

    Eigen::Vector3d D_term = - K_d * differential;

    Eigen::Vector3d controlValue = P_term + D_term;

    return controlValue;
}

// ------------------------------------------------
// Private:
// ------------------------------------------------
double PD_Controller::calcError(const double &targetValue, const double &measureValue) const
{
    return targetValue - measureValue;
}

double PD_Controller::calcDifferential(const double &error, const double &error_old, const double &dt) const
{
    double differential = (error - error_old) / dt;

    error_old_ = error;

    return differential;
}

Eigen::Quaterniond PD_Controller::calcQError(const Eigen::Quaterniond& target, const Eigen::Quaterniond& current) const
{
    return target.conjugate() * current;
}