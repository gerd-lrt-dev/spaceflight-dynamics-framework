#ifndef BASICRCSMODEL_H
#define BASICRCSMODEL_H

#include "Thrust/iThrust.h"
#include "Thrust/RCSConfig.h"
#include "Thrust/FuelStateStruct.h"
#include "Thrust/RCS_ThrustState.h"
#include "vector3.h"
#include "environmentConfig.h"

#include <algorithm>
#include <deque>
#include <iostream>
#include <string>

/**
 * @brief Numerical integration method used for the internal RCS actuator state.
 */
enum class IntegrationMethod
{
    /**
     * @brief Explicit Euler integration.
     */
    Euler,

    /**
     * @brief Exact discrete first-order solution for constant input over one time step.
     */
    ExactFirstOrder
};

/**
 * @brief Time-stamped RCS command sample used for command-delay simulation.
 *
 * Each sample stores a binary command together with the simulation time at which
 * the command was issued. The command buffer is processed as a FIFO queue to
 * approximate the delayed valve input.
 */
struct RCSCommandSample
{
    /**
     * @brief Simulation time at which the command was issued.
     *
     * Unit: s
     */
    double time{0.0};

    /**
     * @brief Binary RCS firing command.
     *
     * Valid values:
     * - 0 = thruster off
     * - 1 = thruster on
     *
     * Unit: -
     */
    int cmd{0};
};

/**
 * @brief Low-order model of a single Reaction Control System thruster.
 *
 * The basicRCSModel represents one individual RCS engine as a binary,
 * valve-controlled actuator with command delay, first-order thrust buildup,
 * first-order thrust decay, and propellant consumption.
 *
 * The model stores thrust internally as a scalar magnitude. The corresponding
 * thrust vector is generated externally by the Thrust Orchestrator using the
 * configured thrust direction:
 *
 * \f[
 * \mathbf{F}_{thr} = T \hat{\mathbf{d}}
 * \f]
 *
 * This design separates actuator dynamics from force-vector aggregation and
 * allows one instance of this class to be created for each configured RCS
 * thruster.
 */
class basicRCSModel : public IThrustModel
{
public:
    /**
     * @brief Constructs a single RCS thruster model.
     *
     * Initializes the model using static RCS engine configuration data and the
     * fuel state associated with the assigned tank.
     *
     * @param rcsConfig Configuration parameters of the RCS thruster.
     * @param fState Fuel-related runtime state associated with the thruster.
     */
    basicRCSModel(const RCSEngineConfig& rcsConfig, FuelState fState);

    /**
     * @brief Destroys the RCS model.
     *
     * The class does not own dynamic resources.
     */
    ~basicRCSModel() override = default;

    /**
     * @brief Updates the RCS thruster state over one simulation time step.
     *
     * Processes delayed commands, integrates the normalized actuator state,
     * computes the current scalar thrust magnitude, and updates fuel
     * consumption.
     *
     * @param dt Discrete simulation time step [s].
     */
    void updateThrust(const double& dt) override;

    // -------------------------------------------------------------------------
    // Public command interface
    // -------------------------------------------------------------------------

    /**
     * @brief Activates or deactivates the RCS thruster.
     *
     * If the engine is deactivated, the thruster should not generate thrust even
     * if a firing command is received.
     *
     * @param activateEngine Engine activation flag.
     */
    void setEnginePowerSwitch(bool activateEngine) override;

    /**
     * @brief Sets the commanded target thrust magnitude.
     *
     * For the BasicRCSModel this function is not the preferred command interface,
     * because the model is designed as a binary on/off actuator. The implementation
     * may map the scalar thrust command to a normalized binary command or reject
     * unsupported continuous values.
     *
     * @param tThrust Commanded target thrust [N].
     */
    void setTarget(const double& tThrust) override;

    /**
     * @brief Sets the commanded RCS firing input.
     *
     * The BasicRCSModel uses a binary command input:
     *
     * \f[
     * u(t) \in \{0,1\}
     * \f]
     *
     * where 0 represents an inactive thruster and 1 represents a commanded
     * firing state. Non-binary values within the valid command interval may be
     * mapped to the nearest binary state by the implementation. Invalid values
     * outside the accepted range are rejected or ignored.
     *
     * The accepted command is stored in the command buffer and becomes effective
     * only after the configured command delay.
     *
     * @param tThrustInPercentage Commanded normalized thrust input [-].
     */
    void setTargetInPercentage(const double& tThrustInPercentage) override;

    // -------------------------------------------------------------------------
    // Public query interface
    // -------------------------------------------------------------------------

    /**
     * @brief Returns the unique engine identifier.
     *
     * @return Engine identifier [-].
     */
    int getEngineID() const override;

    /**
     * @brief Returns the engine name
     *
     * The engine Name specifies the function of engine
     * Example: "RCS_PosZ"
     *
     * @return Engine name string
     */
    std::string getEngineName() const override;

    /**
     * @brief Returns the engine type.
     *
     * The engine type describes the propulsion model category, for example
     * "translation" or "rcs".
     *
     * @return Engine type string.
     */
    std::string getEngineType() const override;

    /**
     * @brief Returns the commanded scalar target thrust.
     *
     * This value represents the requested thrust magnitude derived from the
     * binary RCS command before actuator delay and dynamic response effects are
     * applied.
     *
     * @return Target thrust magnitude [N].
     */
    double getTargetThrust() const override;

    /**
     * @brief Returns the current scalar thrust magnitude.
     *
     * This value represents the actual generated thrust after command delay,
     * actuator response dynamics, and physical limitations have been applied.
     *
     * @return Current thrust magnitude [N].
     */
    double getCurrentThrust() const override;

    /**
     * @brief Returns the normalized thrust direction vector.
     *
     * The direction vector defines the fixed thrust axis of this RCS thruster
     * in the spacecraft body frame.
     *
     * @return Normalized thrust direction [-].
     */
    Vector3 getDirectionOfThrust() const override;

    /**
     * @brief Returns the current propellant mass flow rate.
     *
     * The value represents the instantaneous fuel consumption caused by the
     * current thrust output.
     *
     * @return Fuel consumption rate [kg/s].
     */
    double getFuelConsumption() const override;

    /**
     * @brief Returns the current remaining fuel mass of the assigned tank.
     *
     * @return Current fuel mass [kg].
     */
    double getCurrentFuelMass() const override;

    /**
     * @brief Returns the identifier of the assigned fuel tank.
     *
     * @return Tank identifier [-].
     */
    double getTankID() const override;

    /**
     * @brief Returns the maximum available thrust of this RCS thruster.
     *
     * @return Maximum thrust [N].
     */
    double getMaxThrust() const override;

private:
    // -------------------------------------------------------------------------
    // Private members
    // -------------------------------------------------------------------------

    /**
     * @brief Static configuration parameters of the RCS thruster.
     */
    RCSEngineConfig rcsConfig_{};

    /**
     * @brief Runtime fuel state associated with the assigned tank.
     */
    FuelState fuelstate_{};

    /**
     * @brief Runtime scalar thrust state of the RCS thruster.
     */
    RCS_ThrustState thruststate_{};

    /**
     * @brief Environmental constants used by the propulsion model.
     */
    EnvironmentConfig envConfig_{};

    /**
     * @brief Current commanded binary input.
     *
     * Represents the most recent accepted user or controller command.
     *
     * Valid values:
     * - 0 = thruster off
     * - 1 = thruster on
     *
     * Unit: -
     */
    int cmdInput{0};

    /**
     * @brief Delayed binary command input.
     *
     * Represents the command after applying the configured command delay:
     *
     * \f[
     * u_d(t) = u(t - t_d)
     * \f]
     *
     * Unit: -
     */
    int cmdInputDelayed{0};

    /**
     * @brief FIFO command buffer used to simulate command and valve delay.
     *
     * Each accepted command is stored with its issue time and becomes active
     * once its timestamp is older than the configured command delay.
     */
    std::deque<RCSCommandSample> commandBuffer{};

    /**
     * @brief Internal model time accumulated by this engine instance.
     *
     * Unit: s
     */
    double totalEngineTime{0.0};

    /**
     * @brief Normalized internal actuator state.
     *
     * This state represents the dynamic thrust buildup and decay of the RCS
     * thruster.
     *
     * Typical range:
     * - 0.0 = no generated thrust
     * - 1.0 = nominal steady-state thrust
     *
     * Unit: -
     */
    double thrustState{0.0};

    // -------------------------------------------------------------------------
    // Private calculation methods
    // -------------------------------------------------------------------------

    /**
     * @brief Computes the current scalar thrust magnitude.
     *
     * Integrates the normalized actuator state according to the delayed binary
     * command and converts the resulting state into thrust magnitude.
     *
     * @param cmdInputDelayed Delayed binary command input [-].
     * @param tauOn First-order rise time constant [s].
     * @param tauOff First-order decay time constant [s].
     * @param dt Discrete simulation time step [s].
     * @param nominalThrust Nominal maximum thrust [N].
     *
     * @return Current thrust magnitude [N].
     */
    double calcThrust(const int cmdInputDelayed,
                      const double& tauOn,
                      const double& tauOff,
                      const double& dt,
                      const double& nominalThrust);

    /**
     * @brief Integrates a first-order actuator state over one time step.
     *
     * The state follows the first-order model:
     *
     * \f[
     * \dot{s} = \frac{u - s}{\tau}
     * \f]
     *
     * where s is the current state, u is the target state, and tau is the
     * characteristic time constant.
     *
     * @param state Current normalized state [-].
     * @param target Target normalized state [-].
     * @param tau First-order time constant [s].
     * @param dt Discrete simulation time step [s].
     * @param method Selected numerical integration method.
     *
     * @return Updated normalized state [-].
     */
    double integrateFirstOrderState(double state,
                                    double target,
                                    double tau,
                                    double dt,
                                    IntegrationMethod method);

    /**
     * @brief Converts normalized actuator state into thrust magnitude.
     *
     * @param nominalThrust Nominal maximum thrust [N].
     * @param state Normalized actuator state [-].
     *
     * @return Scalar thrust magnitude [N].
     */
    double calcNominalThrust(const double nominalThrust,
                             const double state);

    /**
     * @brief Converts a normalized command input into a binary RCS command.
     *
     * The BasicRCSModel accepts binary commands only. This function maps valid
     * input values to either 0 or 1.
     *
     * @param input Normalized command input [-].
     *
     * @return Binary command value [-].
     */
    int convertToBinaryCommand(double input);

    /**
     * @brief Computes the remaining fuel mass after one simulation time step.
     *
     * Reduces the available fuel mass according to the current mass flow rate
     * and simulation time step.
     *
     * @param fuelMass Current fuel mass [kg].
     * @param massFlowFuel Fuel mass flow rate [kg/s].
     * @param dt Discrete simulation time step [s].
     *
     * @return Updated fuel mass [kg].
     */
    double calcFuelReduction(const double& fuelMass,
                             const double& massFlowFuel,
                             const double& dt) override;

    /**
     * @brief Computes the propellant mass flow rate.
     *
     * The mass flow rate is computed from thrust and specific impulse:
     *
     * \f[
     * \dot{m} = \frac{T}{I_{sp} g_0}
     * \f]
     *
     * @param currentThrust Current thrust magnitude [N].
     * @param Isp Specific impulse [s].
     * @param earthGravity Standard gravitational acceleration [m/s²].
     *
     * @return Propellant mass flow rate [kg/s].
     */
    double calcMassFlow(const double& currentThrust,
                        const double& Isp,
                        const double& earthGravity);
};

#endif // BASICRCSMODEL_H
