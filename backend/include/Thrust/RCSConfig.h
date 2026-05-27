#pragma once

#include "vector3.h"

#include <iostream>
#include <string>

/**
 * @brief Configuration parameters for a single RCS thruster.
 *
 * This structure contains the static configuration data required by the
 * BasicRCSModel. It describes one individual reaction control thruster,
 * including metadata, tank assignment, thrust performance, actuator delay,
 * first-order response dynamics, thrust direction, and mounting position.
 *
 * The RCS thruster is modeled as a binary on/off actuator. The commanded input
 * is delayed by commandDelay and then converted into thrust through separate
 * first-order rise and decay dynamics.
 */
struct RCSEngineConfig
{
    // -------------------------------------------------------------------------
    // Metadata
    // -------------------------------------------------------------------------

    /**
     * @brief Switch to activate or deactivate this RCS thruster.
     *
     * If false, the thruster does not generate thrust even if it receives a
     * firing command.
     */
    bool engineActivated{false};

    /**
     * @brief Unique engine identifier.
     */
    int id{-1};

    /**
     * @brief Human-readable engine name.
     *
     * Example: "RCS_PosX", "RCS_NegY".
     */
    std::string name{};

    /**
     * @brief Engine type category.
     *
     * Recommended value for this model: "translation".
     */
    std::string type{"translation"};

    /**
     * @brief Controlled RCS axis.
     *
     * Examples: "+x", "-x", "+y", "-y", "+z", "-z".
     */
    std::string axis{};

    /**
     * @brief Identifier of the fuel tank assigned to this thruster.
     */
    int tankID{-1};

    // -------------------------------------------------------------------------
    // Physical Parameters
    // -------------------------------------------------------------------------

    /**
     * @brief Specific impulse of the RCS thruster.
     *
     * Used to compute propellant mass flow from generated thrust.
     *
     * Unit: s
     */
    double Isp{285.0};

    /**
     * @brief Nominal maximum thrust of the RCS thruster.
     *
     * Since the BasicRCSModel is binary, this value represents the steady-state
     * thrust reached when the thruster is fully active.
     *
     * Unit: N
     */
    double maxThrust{40.0};

    /**
     * @brief Command delay between requested firing command and valve response.
     *
     * Represents electronic, command-processing, and valve actuation delay.
     *
     * Unit: s
     */
    double commandDelay{0.02};

    /**
     * @brief First-order rise time constant during thruster activation.
     *
     * Determines how quickly the normalized thrust state approaches one after
     * the delayed command becomes active.
     *
     * Unit: s
     */
    double tauOn{0.08};

    /**
     * @brief First-order decay time constant during thruster deactivation.
     *
     * Determines how quickly the normalized thrust state approaches zero after
     * the delayed command becomes inactive.
     *
     * Unit: s
     */
    double tauOff{0.08};

    /**
     * @brief Minimum accepted pulse width for the RCS thruster.
     *
     * Commands shorter than this duration may be rejected or extended depending
     * on the BasicRCSModel implementation.
     *
     * Unit: s
     */
    double minimumPulseWidth{0.0};

    /**
     * @brief Normalized thrust direction in spacecraft body frame.
     *
     * Must not be zero. The vector is normalized during creation.
     *
     * Unit: -
     */
    Vector3 direction{};

    /**
     * @brief Thruster mounting position relative to the spacecraft body frame.
     *
     * Used for future torque computation through the moment arm relative to the
     * center of gravity.
     *
     * Unit: m
     */
    Vector3 position{};

    /**
     * @brief Creates a validated RCSEngineConfig instance.
     *
     * Checks physical parameters for plausibility and applies conservative
     * defaults if values are outside valid ranges.
     *
     * @return Validated RCSEngineConfig instance
     */
    static RCSEngineConfig Create(bool engineActivated,
                                  int id,
                                  const std::string& name,
                                  const std::string& type,
                                  const std::string& axis,
                                  int tankID,
                                  double Isp,
                                  double maxThrust,
                                  double commandDelay,
                                  double tauOn,
                                  double tauOff,
                                  double minimumPulseWidth,
                                  Vector3 direction,
                                  Vector3 position)
    {
        if (Isp < 40.0 || Isp > 450.0)
        {
            std::cerr << "[RCSEngineConfig] Warning: Isp out of range ("
                      << Isp << "), resetting to default 285 [s].\n";
            Isp = 285.0;
        }

        if (maxThrust <= 0.0 || maxThrust > 1000.0)
        {
            std::cerr << "[RCSEngineConfig] Warning: maxThrust out of range ("
                      << maxThrust << "), resetting to default 40 [N].\n";
            maxThrust = 40.0;
        }

        if (commandDelay < 0.0 || commandDelay > 0.2)
        {
            std::cerr << "[RCSEngineConfig] Warning: commandDelay out of range ("
                      << commandDelay << "), resetting to default 0.02 [s].\n";
            commandDelay = 0.02;
        }

        if (tauOn <= 0.0 || tauOn > 1.0)
        {
            std::cerr << "[RCSEngineConfig] Warning: tauOn out of range ("
                      << tauOn << "), resetting to default 0.08 [s].\n";
            tauOn = 0.08;
        }

        if (tauOff <= 0.0 || tauOff > 1.0)
        {
            std::cerr << "[RCSEngineConfig] Warning: tauOff out of range ("
                      << tauOff << "), resetting to default 0.08 [s].\n";
            tauOff = 0.08;
        }

        if (minimumPulseWidth < 0.0 || minimumPulseWidth > 1.0)
        {
            std::cerr << "[RCSEngineConfig] Warning: minimumPulseWidth out of range ("
                      << minimumPulseWidth << "), resetting to default 0 [s].\n";
            minimumPulseWidth = 0.0;
        }

        if (direction.norm() == 0.0)
        {
            std::cerr << "[RCSEngineConfig] Warning: thrust direction is zero, "
                      << "resetting to (1,0,0).\n";
            direction = {1.0, 0.0, 0.0};
        }
        else
        {
            direction = direction.normalized();
        }

        return {
            engineActivated,
            id,
            name,
            type,
            axis,
            tankID,
            Isp,
            maxThrust,
            commandDelay,
            tauOn,
            tauOff,
            minimumPulseWidth,
            direction,
            position
        };
    }
};
