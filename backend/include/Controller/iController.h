#pragma once
#include <Eigen/Dense>

/**
 * @brief Abstract base class for all controllers.
 *
 * Defines the interface for a generic controller that calculates a control signal
 * based on a target and measured value. Designed to be overridden by specific
 * controller types (e.g., PD, PID, Bang-Bang).
 */
class IController
{
public:
    /**
     * @brief Virtual destructor for safe polymorphic deletion.
     */
    virtual ~IController() = default;

    /**
     * @brief Compute the control output for a single timestep.
     *
     * @param targetValue Desired setpoint.
     * @param measuredValue Current measured value.
     * @param K_p Proportional gain.
     * @param K_d Derivative gain.
     * @param dt Time step since last update.
     * @return Control output (e.g., throttle or force command).
     *
     * @note This function is intended to be overridden in derived classes.
     */
    virtual double control(const double &targetValue, const double &measuredValue, const double &K_p, const double &K_d, const double &dt) const = 0;

    /**
     * @brief Compute the control output for a single timestep.
     *
     * @param targetValue Desired setpoint.
     * @param measuredValue Current measured value.
     * @param differential term.
     * @param K_p Proportional gain.
     * @param K_d Derivative gain.
     * @param dt Time step since last update.
     * @return Control output (e.g., throttle or force command).
     *
     * @note This function is intended to be overridden in derived classes.
     */
    virtual double controlWithOwnDTerm(const double &targetValue, const double &measuredValue, const double differential, const double &K_p, const double &K_d, const double &dt) const = 0;

    /**
     * @brief Computes a PD control output for a quaternion-related scalar component.
     *
     * Calculates the control output from the deviation between the target and current
     * value and an externally provided differential term.
     *
     * Unlike the standard scalar PD control method, the derivative term is not
     * calculated internally from consecutive error values. Instead, the differential
     * value is provided explicitly by the caller.
     *
     * @param targetValue Desired target value.
     * @param currentValue Current measured value.
     * @param differential Differential term used for damping.
     * @param K_P Proportional gain.
     * @param K_D Derivative gain.
     *
     * @return Calculated control output.
     */
    virtual Eigen::Quaternion controlQuaternion(const double& targetValue, const double &currentValue, const double& differential, const double& K_P, const double& K_D) const = 0;
};
