#ifndef SPACECRAFT_H
#define SPACECRAFT_H

#include "Thrust.h"
#include "vector3.h"
#include "environmentConfig.h"
#include "quaternion.h"
#include "physics.h"
#include "simDataStruct.h"
#include "spacecraftStateStruct.h"
#include "stateVectorStruct.h"
#include "customSpacecraftStruct.h"
#include "Optimization/thrustOptimizer.h"
#include "Automation/iautopilot.h"
#include "Controller/iController.h"
#include "Thrust/EngineConfig.h"

#include <memory>

/**
 * @class spacecraft
 * @brief Represents a spacecraft with a main engine and fuel.
 *
 * The class encapsulates the physical properties of a spacecraft,
 * such as mass (empty and fuel), maximum thrust, and its main engine.
 * This allows for physics-based simulations including thrust updates
 * and mass-dependent calculations.
 * This class calculates the damage a spacecraft takes by specific manouvers.
 * 
 * TODO: Spacecraft should get an point for center of gravity, which moves, when
 * fuel is gettin lower
 */
class spacecraft
{
private:
    /**
     * @name Physical properties
     * Physical and mechanical properties of the spacecraft.
     * Dynamic parameters are strictly seperated from constant parameters
     * Dynamic -> private members, static -> customSpacecraft struct
     * operation vs. configuration
     */
    ///@{
    std::unique_ptr<physics> physics_;          ///< Physics engine handling lander motion
    Thrust thrustOrchestration;                 ///< Orchestrator class for engine simulation

    StateVector state_;                     ///< Encapsulates the complete translational and rotational state of the spacecraft and is single source of thruth
    EnvironmentConfig environmentConfig_;   ///< [-] Environment config struct with constant parameters.
    SpacecraftState spacecraftState_;       ///< State of spacecraft
    customSpacecraft landerMoon;            ///< [] Parameters which defines spacecraft. This are filled by json config data.

    double totalMass;               ///< [kg] Total mass of spacecraft.
    double dt = 0;                  ///< [s] Time steps. Provided by updateTime.
    double time = 0;                ///< [s] Absolute time. Will be added by dt from udpateTime.
    double GLoad = 0;               ///< [m/s²] GLoad of spacecraft

    double spacecraftIntegrity;     ///< [%] Current integrity of the spacecraft.

    std::string consoleTxt;

    ///@}

    /**
     * @brief Sets default values required to run the simulation
     * 
     * This function initializes all spacecraft parameters that are not explicitly
     * set via the constructor or via customSpacecraft struct with constant parameters.
     * It ensures that the spacecraft has a valid initial state for simulation.
     *
     * Specifically, it sets default values for:
     * - @ref spacecraftIntegrity
     * - @ref spacecraftState_
     * - @ref totalMass
     * - @ref state_.I_Position
     * - @ref state_.I_Velocity
     *
     * Called automatically in the constructor.
     */
    void setDefaultValues();

    // -------------------------------------------------------------------------
    // Private update functions
    // -------------------------------------------------------------------------

    /**
     * @brief Updates total mass
     * Updates the total mass of the spacecraft in connection with fuel consumption due to the combustion process.
     * Updates the total mass of the class directly without return.
     */
    void updateTotalMassOnFuelReduction(double emptyMass, double fuelMass);

    /**
     * @brief Updates all dynamic movement-related data of the spacecraft.
     *
     * This function evaluates all relevant physical models affecting the spacecraft
     * motion, including environmental forces, actuation effects, and control inputs.
     * Based on these evaluations, the time derivatives of the spacecraft state are
     * computed and applied to update the internal state vector.
     *
     * The function does not manage simulation time or time stepping directly.
     * It is intended to be called as part of a higher-level update routine
     * (e.g. updateStep), ensuring a consistent and coordinated state update.
     *
     * All updates performed by this function are internal to the spacecraft object.
     * No partial or intermediate state is exposed to external components.
     */
    void updateMovementData(double dt);

    /**
     * @brief Updates all dynamic movement-related data of the spacecraft to zero
     * @param dt
     *
     * Function is used for landed or crashed states
     */
    void updateMovementDataToZero(double dt);

    /**
     * @brief Update G-Load in preperation for sending to UI
     * @param totalAcceleration
     * @param gravityAcceleration
     * @return GLoad
     */
    void updateGLoad(const Vector3& totalAcceleration, const Vector3& gravityAcceleration);

    // -------------------------------------------------------------------------
    // Private intializer functions
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    // Private setter functions
    // -------------------------------------------------------------------------
    /**
     * @brief Sets spacecraft state
     * @param newState
     *
     * Currently not used. Maybe obsolete
     */
    void setSpacecraftState(SpacecraftState newState);

    /**
     * @brief Update position to new values
     * @param 3D Vector with cartesian-coordinates [m]
     */
    void setPosition(const Vector3& pos);

    /**
     * @brief Sets velocity of spacecraft
     * @param 3D Vector with velocities in three dimensions [m/s]
     */
    void setVelocity(const Vector3& vel);

    /**
     * @brief Sets orientation of spacecraft
     * @param Quaternion with q0 Scalar component, q1 First vector component, q2 Second vector component, q3 Third vector component
     */
    void setOrientation(const Quaternion& q);

    /**
     * @brief Sets angular velocity of spacecraft
     * @param Angular velocity vector of the spacecraft [rad/s].
     */
    void setAngularVelocity(const Vector3& angVel);

    /**
     * @brief Sets GLoad
     * @param GLoad [m/²].
     */
    void setGLoad(const double& GLoad);

    // -------------------------------------------------------------------------
    // Private getter functions
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    // Apply functions
    // -------------------------------------------------------------------------

    /**
     * @brief Applies landing damage based on impact velocity and spacecraft mass.
     *
     * This function calculates the percentage damage the spacecraft receives
     * during landing. The damage is proportional to the kinetic energy (KE) at impact
     * relative to a reference safe landing energy.
     *
     * The calculation steps are as follows:
     * 1. Compute the total kinetic energy of the spacecraft:
     *      KE = 0.5 * total_mass * v^2
     *      where:
     *          - total_mass = emptyMass + fuelMass
     *          - v = vertical velocity at impact [m/s]
     *
     * 2. Define a reference kinetic energy (KE_ref) corresponding to a safe landing:
     *      KE_ref = 0.5 * total_mass * v_safe^2
     *      where:
     *          - v_safe = maximum safe landing velocity
     *
     * 3. Compute proportional damage:
     *      damage_percent = KE / KE_ref
     *
     * 4. Update spacecraft integrity:
     *      new_integrity = old_integrity - damage_percent
     *      Integrity is clamped to [0, 1].
     *
     * 5. Update operational status:
     *      spacecraftIsOperational = (new_integrity >= structuralIntegrity)
     *
     * @param impactVelocity Vertical velocity at landing [m/s]
     * @param safeVelocity Maximum safe landing velocity [m/s] (default is 2 m/s)
     */
    void applyLandingDamage(double impactVelocity);

public:
    /**
     * @brief Constructs a spacecraft using parameters loaded from a configuration object.
     *
     * This constructor initializes all physical, mechanical, and inertial properties of the
     * spacecraft using the values provided in a @ref customSpacecraft configuration structure.
     * All mass properties, propulsion system parameters, inertial characteristics, initial
     * state vectors, and integrity limits are copied directly into the spacecraft instance.
     *
     * The spacecraft’s main engine, thrust model, inertia model, and initial state (position,
     * rotation, velocity, and center of mass) are fully initialized based on this configuration.
     *
     * @param landerMoon
     *     A configuration structure containing complete physical and operational parameters
     *     of the spacecraft. Fields include:
     *       - @c emptyMass                 Dry mass of the spacecraft (kg)
     *       - @c fuelM                     Initial fuel mass (kg)
     *       - @c maxFuelM                  Maximum fuel capacity (kg)
     *
     *       - @c maxT                      Maximum main engine thrust (N)
     *       - @c Isp                       Specific impulse of the main engine (s)
     *       - @c timeConstant              Engine throttle response time constant (s)
     *       - @c responseRate              Maximum engine thrust rate-of-change (1/s)
     *       - @c B_mainThrustDirection     Direction of thrust vector in body frame (unit vector)
     *       - @c B_mainThrustPosition      Thrust application point relative to body origin (m)
     *
     *       - @c Ixx                       Moment of inertia around body X-axis (kg·m²)
     *       - @c Iyy                       Moment of inertia around body Y-axis (kg·m²)
     *       - @c Izz                       Moment of inertia around body Z-axis (kg·m²)
     *
     *       - @c B_initialPos              Initial spacecraft position in body coordinates (m)
     *       - @c B_initialRot              Initial spacecraft rotation in body coordinates (rad)
     *       - @c B_initialCenterOfMass     Initial center of mass location (m)
     *       - @c initialVelocity           Initial velocity vector in body frame (m/s)
     *
     *       - @c structuralIntegrity       Percentage threshold below which spacecraft fails (%)
     *       - @c safeVelocity              Maximum safe landing velocity (m/s)
     *
     * The constructor assumes that the provided configuration structure has been validated
     * prior to construction (e.g., via JSON loader or configuration class).
     */
    spacecraft(customSpacecraft lMoon);

    /**
     * @brief Destructor
     *
     * Cleans up resources if needed. Currently trivial.
     */
    ~spacecraft();

    // -------------------------------------------------------------------------
    // Updater functions
    // -------------------------------------------------------------------------

    /**
     * @brief Advances the simulation by one discrete time step.
     *
     * This function performs a complete and physically consistent state update
     * of the spacecraft over the given time increment. It triggers the evaluation
     * of all relevant dynamic models (e.g. environment, actuation, and control),
     * computes the time derivatives of the full state vector, and integrates the
     * result over the specified time step.
     *
     * The update is executed as a single, atomic operation to ensure that the
     * translational and rotational states remain synchronized. No partial state
     * updates are exposed during the execution of this function.
     *
     * This method is intended to be called exactly once per simulation time step
     * by the simulation controller.
     *
     * @param dt Simulation time step [s].
     */
    void updateStep(double dt);

    /**
     * @brief Updates spacecraft integrity in case of damage
     * 
     * If the spacecraft sustains damage, this function adds the damage to 
     * the spacecraft's integrity. Funktion updates spacecraftIntegrity automatically
     * 
     * @note Damage (delta) has to be a negative value! 
     * 
     * 
     * Damage will deduct the integrity of spacecraft
     * - @ref spacecraftIntegrity
     */
    void updateSpacecraftIntegrity();

    /**
     * @brief Set thrust up to specific level for main engine
     * @param targetThrustInPercentage clamped to [0, 1] where 1 is 100% of max thrust
     * 
     * This function initiates the main engine and provide thrust for spacecraft. It 
     * provides thrust until the setted target thrust is reached.
     */
    void setMainEngineThrust(const double &targetThrustInPercentage);

    /**
     * @brief Set thrust for RCS engines up to specific level
     * @param targetThrust Thrust clamped to [0, 1] where 1 is 100% of max thrust
     *
     * This function initiates the RCS (Reaction Control System) engines and provide thrust for spacecraft. It
     * provides thrust until the setted target thrust is reached.
     */
    void setTargetRCSThrust(const Vector3 &targetThrustInPercentage);

    /**
     * @brief Set Console Text for output in cockpit page
     * @param txt as std string
     */
    void setConsoleText(const std::string &txt);

    /**
     * @brief compute optimization
     * @return vector with optimized thrust controls
     */
    std::vector<double> compute_optimization(double h0, double v0, double m0, double dt);

    // -------------------------------------------------------------------------
    // Requester functions
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // Request functions
    // -------------------------------------------------------------------------
    /**
     * @brief Returns the current totalthrust.
     *
     * Queries the thrust orchestration for sum of all thrust forces.
     * This function gets called for physics calculation
     *
     * @return Current thrust vector [N]
     */
    Vector3 requestTotalThrust() const;

    /**
     * @brief Returns the commanded main engine thrust.
     *
     * Queries the propulsion model for the desired main engine thrust as provided
     * by the guidance or control system prior to actuator dynamics and limitations.
     *
     * @return Commanded main engine thrust vector [N]
     */
    Vector3 requestMainEngineTargetThrust() const;

    /**
     * @brief Returns the actual main engine thrust.
     *
     * Queries the propulsion model for the current thrust generated by the main
     * engine after applying actuator dynamics and physical constraints.
     *
     * @return Actual main engine thrust vector [N]
     */
    Vector3 requestMainEngineThrust() const;

    /**
     * @brief Returns the commanded main engine thrust as percentage.
     *
     * Provides the commanded thrust level relative to the maximum available
     * main engine thrust.
     *
     * @return Commanded thrust as percentage of maximum thrust [%]
     */
    Vector3 requestMainEngineThrustInPercentage() const;

    /**
     * @brief Return thrust direction of engine due to config paramter
     * @return Thrust direction as Vector3
     */
    Vector3 requestMainEngineDirection() const;

    /**
     * @brief Returns the current main engine fuel consumption rate.
     *
     * Queries the propulsion model for the instantaneous propellant mass flow
     * associated with the main engine.
     *
     * @return Fuel mass flow rate [kg/s]
     */
    double requestMainEngineLiveFuelConsumption() const;

    /**
     * @brief Returns the commanded RCS thrust vector.
     *
     * Queries the propulsion model for the desired RCS thrust vector prior to
     * actuator dynamics, allocation, and saturation effects.
     *
     * @return Commanded RCS thrust vector [N]
     */
    Vector3 requestRCSTargetThrust() const;

    /**
     * @brief Returns the actual RCS thrust vector.
     *
     * Queries the propulsion model for the resulting RCS thrust vector after
     * actuator dynamics, allocation, and physical constraints have been applied.
     *
     * @return Actual RCS thrust vector [N]
     */
        Vector3 requestRCSThrust() const;

    /**
     * @brief Returns the commanded RCS thrust as percentage.
     *
     * Provides the commanded RCS thrust vector relative to the available thrust
     * authority along each axis.
     *
     * @return Commanded RCS thrust as percentage [%]
     */
    Vector3 requestRCSThrustInPercentage() const;

    /**
     * @brief Returns the current RCS fuel consumption rate.
     *
     * Queries the propulsion model for the instantaneous propellant mass flow
     * associated with the RCS.
     *
     * @return Fuel mass flow rate [kg/s]
     */
    double requestRCSLiveFuelConsumption() const;

    // -------------------------------------------------------------------------
    // Public setter functions
    // -------------------------------------------------------------------------

    /**
     * @brief Set inital position
     * Sets position only in spacecraftdata struct.
     * This function has no rights to wirte into statevector!
     */
    void setInitalPosition(const Vector3& position);

    /**
     * @brief Set inital velocity
     * Sets velocity only in spacecraftdata struct.
     * This function has no rights to wirte into statevector!
     */
    void setInitalVelocity(const Vector3& velocity);


    // -------------------------------------------------------------------------
    // Public getter functions
    // -------------------------------------------------------------------------

    simData getFullSimulationData() const;

    /**
     * @brief getter for Integrity
     * @return [%] integrity
     */
    double getIntegrity();

     /**
     * @brief Return current state of spacecraft
     * @return Current state of space craft organized in a full state struct
     */
    const StateVector& getState() const;

    /**
     * @brief Return current position of spacecraft
     * @return Current position in cartesian-coordinates [m]
     */
    Vector3 getPosition() const;

    /**
     * @brief Return current velocity of spacecraft
    //  * @return Current velocity [m/s]
     */
    Vector3 getVelocity() const;

    /**
     * @brief Return orientation of spacecraft
     * @return Quaternion with q0 Scalar component, q1 First vector component, q2 Second vector component, q3 Third vector component
     */
    Quaternion getOrientation() const;

    /**
     * @brief Return angular velocity of spacecraft
     * @return Angular velocity vector of the spacecraft [rad/s].
     */
    Vector3 getAngularVelocity() const;

    /**
     * @brief Return current total mass
     * @return total mass
     * 
     * Note that total mass is constantly changing due to fuelmass 
     * which decrease in case of providing thrust
     */
    double getTotalMass();

    /**
     * @brief Return current total mass
     * Considers mass of all tanks for physical calculation
     * @return total mass of fuel [kg]
     */
    double getTotalFuelMass() const;

    /**
     * @brief Return Fuel tank information for all installed tanks
     *
     * @return Vector with Fuel Tank Struct
     */
    std::vector<FuelTank> getFuelTanks() const;

    /**
     * @brief Return current fuel mass
     * @return fuel mass
     * 
     * Fuel mass is updated by update thrust
     */
    const std::vector<FuelTank> getfuelMass() const;

    /**
     * @brief Return GLoad
     * @return GLoad [m/s²]
     */
    double getGload() const;

    std::string getConsoleTxt() const;

    /**
     * @brief Getter for spacecraft state
     * @return Current spacecraft state
     */
    SpacecraftState getSpacecraftState() const;
};

#endif
