#pragma once

#include "Controller/iController.h"

/**
 * @brief PD (Proportional-Derivative) controller implementation.
 *
 * Calculates a control output using a proportional and derivative term based
 * on the target and measured value. Maintains previous error internally to
 * compute the derivative term.
 */
class PD_Controller : public IController
{
public:
    /**
     * @brief Compute the PD control output for a single timestep.
     *
     * @param targetValue Desired setpoint.
     * @param measuredValue Current measured value.
     * @param K_p Proportional gain.
     * @param K_d Derivative gain.
     * @param dt Time step since last update.
     * @return Control output (e.g., throttle or force command).
     *
     * @note Overrides IController::control.
     */
    double control(const double &targetValue, const double &measuredValue, const double &K_p, const double &K_d, const double &dt) const override;

    /**
     * @brief Compute the PD control output for a single timestep. The differential Term is given by user.
     *
     * @param targetValue Desired setpoint.
     * @param measuredValue Current measured value.
     * @param differential
     * @param K_p Proportional gain.
     * @param K_d Derivative gain.
     * @param dt Time step since last update.
     * @return Control output (e.g., throttle or force command).
     *
     * @note Overrides IController::control.
     */
    double controlWithOwnDTerm(const double &targetValue, const double &measuredValue, const double differential, const double &K_p, const double &K_d, const double &dt) const override;

    /**
     * @brief Computes a quaternion-based PD control output.
     *
     * Calculates a quaternion control response from the deviation between the
     * desired target value and the current value. The derivative contribution is
     * supplied explicitly through the differential term and is weighted together
     * with the proportional contribution by the corresponding controller gains.
     *
     * @param targetValue Desired target value.
     * @param currentValue Current measured value.
     * @param differential Differential term used for damping.
     * @param K_P Proportional gain.
     * @param K_D Derivative gain.
     *
     * @return Quaternion representing the resulting control output.
     */
    Eigen::Quaternion controlQuaternion(const double& targetValue, const double &currentValue, const double& differential, const double& K_P, const double& K_D) const override;

private:
    //***********************************************************
    //*************        Members                   ************
    //***********************************************************

    /**
     * @brief Previous error used to calculate derivative term.
     *
     * Mutable to allow modification in const compute function.
     */
    mutable double error_old_ = 0.0;

    /**
    * @brief Previous error used to calculate derivative term.
    *
    * Mutable to allow modification in const compute function.
    */
    mutable Eigen::Quaternion qError_old_{0.0, 0.0, 0.0, 0.0};

    //***********************************************************
    //*************        Methods                   ************
    //***********************************************************

    /**
     * @brief Calculate the error between target and measured value.
     *
     * @param targetValue Desired setpoint.
     * @param measureValue Current measured value.
     * @return Error = target - measured.
     */
    double calcError(const double &targetValue, const double &measureValue) const;

    /**
     * @brief Calculate the derivative term from current and previous error.
     *
     * @param error Current error.
     * @param error_old Previous error.
     * @param dt Time step.
     * @return Derivative term (change rate of error).
     */
    double calcDifferential(const double &error, const double &error_old, const double &dt) const;

    /**
     * @brief Calculate the error between target and measured quaternion value.
     *
     * @param targetValue Desired setpoint.
     * @param measureValue Current measured value.
     * @return Error = target^(-1) * measured.
     */
    Eigen::Quaternion calcQError(const Eigen::Quaternion& target, const Eigen::Quaternion& current) const;
};
