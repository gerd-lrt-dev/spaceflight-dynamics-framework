#pragma once

#include <eigen3/Eigen/Dense>

struct ControlCommand{
    double thrustInPercentage = 0.0;
    bool autopilotActive = false;

    /**
     * @brief Main engine thrust command.
     *
     * Represents the commanded thrust level for the main engine.
     *
     * @note Expected range: [0.0, 100.0] (percentage).
     */
    float mainEngine = 0.0f;

    /**
     * @brief Rotational command (attitude control).
     *
     * Represents desired rotational inputs around the principal axes.
     *
     * Components:
     * - x: pitch (rotation around lateral axis)
     * - y: yaw   (rotation around vertical axis)
     * - z: roll  (rotation around longitudinal axis)
     *
     * @note Units: radians or normalized input depending on implementation.
     *       Must be interpreted consistently by the simulation core.
     */
    Eigen::Vector3d rotation;

    /**
     * @brief Translational command (RCS linear control).
     *
     * Represents desired translational movement along the principal axes.
     *
     * Components:
     * - x: lateral movement (left/right)
     * - y: vertical movement (up/down)
     * - z: longitudinal movement (forward/backward)
     *
     * @note Typically mapped to RCS thruster groups.
     *       Units are usually normalized input [-1.0, 1.0].
     */
    Eigen::Vector3d translation;

    /**
     * @brief Stabilization request flag.
     *
     * When set, requests the control system to stabilize the vehicle,
     * typically by damping angular velocities and holding attitude.
     */
    bool stabilize = false;

    /**
     * @brief Kill rotation command flag.
     *
     * When set, requests immediate damping of all rotational motion.
     * Often implemented as an aggressive attitude hold or zero angular velocity controller.
     */
    bool killRotation = false;
};

class InputArbiter{
public:
    ControlCommand chooseCommand();
    void receiveUserControlCommand(const ControlCommand &userCmd);
    void receiveAutoControlCommand(const ControlCommand &autoCmd);

private:
    void setAutomationActiveFlag(bool on);
    bool automationActive = false;
    ControlCommand usrCmd_;
    ControlCommand autoCmd_;
};
