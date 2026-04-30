#pragma once

#include "Thrust/iThrust.h"
#include "Thrust/FuelStateStruct.h"
#include "Thrust/EngineConfig.h"
#include "Thrust/ME_thrustState.h"

#include <iostream>

class basicMainEngineModel : public IThrustModel{

public:
    /**
     * @brief Constructor
     * @param eConfig Configuration parameters for a spacecraft engine
     * @param fState Fuel-related state variables for the engine
     */
    basicMainEngineModel(const EngineConfig& eConfig, FuelState fState);

    /**
     * @brief Destructor
     *
     * Currently trivial, no dynamic resources.
     */
    ~basicMainEngineModel() override = default;

    /**
     * @brief Updates the current thrust value using an exponential smoothing approach.
     *
     * This function adjusts the current thrust toward the target thrust based on
     * a continuous-time first-order response model. The differential equation
     *
     *     d(current) / dt = (target - current) / τ
     *
     * describes the convergence of the current thrust toward the target with
     * time constant τ. Solving this yields the discrete update rule:
     *
     *     current += (1 - exp(-dt / τ)) * (target - current)
     *
     * This provides framerate-independent, smooth convergence of the thrust,
     * where a smaller τ results in faster response.
     *
     * @param dt  Time step in seconds.
     */
    void updateThrust(const double &dt) override;

    // -------------------------------------------------------------------------
    // Public setter override functions
    // -------------------------------------------------------------------------

    /**
     * @brief Activates specific engine
     * @param engineNr
     *
     * Only thrust orchestrator is authorized to activate engine (to use this function)
     */
    void setEnginePowerSwitch(bool activateEngine) override;

    /**
     * @brief Set a new target thrust
     * @param tThrust ///<  [N] Target thrust
     */
    void setTarget(const double &tThrust) override;

    /**
     * @brief Set a new target thrust in percentage
     * @param tThrust ///<  [%] Target thrust 0.0 ... 1.0
     *
     * thrust in percentage is going to tranformed in function
     */
    void setTargetInPercentage(const double &tThrustInPercentage) override;

    // -------------------------------------------------------------------------
    // Public getter override functions
    // -------------------------------------------------------------------------

    /**
     * @brief Get the Engine ID
     * @return Engine ID as an Integer
     */
    int getEngineID() const override;

    /**
     * @brief Get the Engine Type such as main, translation (RCS), rotation (RCS)
     * @return Engine Type as steady string
     */
    std::string getEngineType() const override;

    /**
     * @brief Get the target thrust
     * @return ///< [N] target thrust
     */
    double getTargetThrust() const override;

    /**
     * @brief Get the current thrust
     * @return ///< [N] Current thrust
     */
    double getCurrentThrust() const override;

    /**
     * @brief Getter function for fuel live fuel consumption
     * @return ///< [kg / dt] Real-time fuel consumption
     */
    double getFuelConsumption() const override;

    /**
     * @brief Getter function for current fuel mass
     * @return ///< [kg] fuel mass
     */
    double getCurrentFuelMass() const override;

    /**
     * @brief Getter function for tank ID
     * @return tank id as integer
     *
     * The tank ID specifies which tank is connected to the engine system
     */
    double getTankID() const override;

    /**
     * @brief Getter function for thrust direction
     * @return ///< [-] Vector with direction of thrust
     *
     * The vector is aligned with the static coordinate system of the spacecraft
     */
    Vector3 getDirectionOfThrust() const override;

    /**
     * @brief Calculate fuel cunsomption
     * @param fuelMass      ///< [kg] Mass of fuel
     * @param massflowFuel  ///< [kg/s] Mass flow of fuel
     * @param dt            ///< [s] discrete time step and update parameter
     *
     * Reduces fuel supply depending on fuel consumption through the thrust provision process
     */
    double calcFuelReduction(const double &fuelMass,const double &massFlowFuel,const double &dt) override;

    /**
     * @brief Getter function for maximum thrust given by engine config
     * @return ///< [N] Maximum Thrust
     */
    double getMaxThrust() const override;

private:
    EngineConfig engineConfig_;      ///< [-] Configuration parameters for a spacecraft engine.
    ME_ThrustState ME_thrustState_;  ///< [-] Dynamic state of the engine thrust.
    FuelState fuelstate_;            ///< [-] Fuel-related state variables for the engine.

    // -------------------------------------------------------------------------
    // Private setter functions
    // -------------------------------------------------------------------------
    /**
     * @brief Set default values & start conditions
     */
    void setDefaultValues();

    // -------------------------------------------------------------------------
    // Private calculation methods
    // -------------------------------------------------------------------------
    double calcMassFlow(const double &currenThrust, const double &Isp, const double &earthGravity);
};
