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
     * @brief Compute a three-axis quaternion-based PD control output.
     *
     * Calculates a rotational control vector from the quaternion attitude error
     * between target and current orientation. The derivative contribution is
     * supplied explicitly as angular velocity rather than derived numerically
     * from consecutive quaternion errors.
     *
     * @param target Desired target attitude quaternion.
     * @param current Current attitude quaternion.
     * @param angularVelocity Current angular velocity used as damping term [rad/s].
     * @param K_P Three-axis proportional gain vector.
     * @param K_D Three-axis derivative gain vector.
     * @return Three-axis rotational control command.
     */
    virtual Eigen::Vector3d controlQuaternion(const Eigen::Quaterniond& target, const Eigen::Quaterniond& current, const Eigen::Vector3d& angularVelocity, const Eigen::Vector3d& K_P, const Eigen::Vector3d& K_D) const = 0;
};
