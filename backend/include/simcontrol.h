#ifndef SIMCONTROL_H
#define SIMCONTROL_H

#include "spacecraft.h"
#include "simDataStruct.h"
#include "jsonConfigReader.h"
#include "Control/inputArbiter.h"

#include <optional>
#include <memory>
#include <nlopt.h>

/**
 * @class simcontrol
 * @brief Central simulation orchestrator.
 *
 * SimControl is responsible for coordinating all high-level control flow of the simulation.
 * It does not perform physics calculations itself, but manages the interaction between
 * the major subsystems.
 *
 * Responsibilities:
 * - Collect and forward user input commands
 * - Query and apply automation / autopilot commands
 * - Resolve command priority (user vs. automation)
 * - Trigger and sequence simulation time steps
 * - Notify and update the UI / frontend state
 *
 * In short, SimControl decides *who controls the spacecraft* and *when the simulation advances*,
 * while the actual physics and state changes are handled by the spacecraft and its subsystems.
 *
 */
class simcontrol
{
private:
    //***********************************************************
    //*************        Members                   ************
    //***********************************************************
    std::unique_ptr<spacecraft>     landerSpacecraft;   ///< Spacecraft with specs and integrity
    std::unique_ptr<InputArbiter>   inputArbiter_;      ///< Arbiter for input commands
    std::unique_ptr<IAutopilot>     autopilot_;         ///< Virtual autopilot instance
    std::unique_ptr<IController>    controller_;        ///< Virtual controller instance

    std::string jsonConfigString;                   ///< String with raw space config data provided by frontend
    customSpacecraft landerMoon1;                   ///< Config for used spacecraft provided by json config
    EnvironmentConfig config_;                      ///< Config struct for moon environment
    ControlCommand cmd_;                            ///< Command structure for autopilot
    bool resetRequested;                            ///< Represents user desire to reset simulation

    // Inital data
    double initialTime;                             ///< [s] Initial simulation time

    //***********************************************************
    //*************    Memberfuctions                ************
    //***********************************************************
    /**
     * @brief Build instances necessary for the simulation
     */
    void buildSimulationEnvironment(double t);

    /**
     * @brief Load json config out of string provided from frontend which defines spacecraft parameters
     * @param jsonString                            ///< String with config data
     * @param spacecraftName                        ///< Selected spacecraft from config data
     *
     * Working jsonconfig together with frontend!
     */
    customSpacecraft loadSpacecraftFromJsonString(const std::string& jsonString);

    /**
     * @brief Sets target thrust
     * @param thrustPercent [%]
     */
    void setTargetMainEngineThrust(const double& thrustPercent = 0, const double& thrustInNewton = 0);

    /**
     * @brief Set RCS Thrust
     * @param Vector3 translation with thrust in cartasian coordinates within the principle ENU (East North Up)
     */
    void setTargetRCSThrust(const Vector3 &ENU_translation);

    /**
     * @brief Process commands
     *
     * This class accepts all commands from the front end. The commands are then sent
     * to a control instance, which decides which commands are currently valid. This
     * means that several commands from different sources can be sent to this method
     * at the same time.
     *
     */
    void processCommands();

    /**
     * @brief orchestrate autopilot
     * @param Current Spacecraft state. Autopilot is only active, when spacecraft state is operational
     * @param engineNr [-] Number of engine that will be controlled automatically
     * @param dt [s] Discrete timestep
     */
    void runAutopilot(const SpacecraftState& currentSpacecraftstate, const int &engineNr, const double& dt);

public:
    //***********************************************************
    //*************    Memberfuctions                ************
    //***********************************************************
    /**
     * @brief Constructor
     * @param t     ///< [s] Initial simulation time
     */
    simcontrol(double t0);

    /**
     * @brief Destructor
     *
     * Cleans up resources automatically via smart pointers.
     */
    ~simcontrol();

    /**
     * @brief Initializes simulation environment and spacecraft config
     * @param jsonConfigStr                         ///< String with spacecraft config data
     */
    void initialize(const std::string& jsonConfigStr);

    /**
     * @brief Instances the logging action and provides filepath for logging file
     */
    void instanceLoggingAction();

    /**
     * @brief Computes simulation due to timesteps given from simulation worker
     *
     * This function owns the states of simulation. It knows all physical, environmental and spacecraft conditions.
     * All states are calculated by given timesteps from worker.
     * @param dt                                    ///< [s] discrete timestep
     */
    simData runSimulation(const double dt);

    /**
     * @brief Receives a control command from the frontend.
     *
     * This method is called by the frontend to submit a user-generated
     * control command. The command is forwarded to an internal, private
     * method that collects and arbitrates all incoming commands.
     *
     * The private arbitration method can handle both user commands and
     * automated commands, but the frontend is restricted from sending
     * automation commands directly. This ensures that user input and
     * autopilot input remain separated and properly managed.
     *
     * @param userCmd The control command provided by the user interface.
     */
    void receiveCommandFromFrontEnd(const ControlCommand& userCmd);

    /**
     * @brief Receives a control command from the autopilot system.
     *
     * This method is called by the autopilot to submit an automated
     * control command. The command is forwarded to an internal, private
     * method that collects and arbitrates all incoming commands.
     *
     * The private arbitration method can handle both user commands and
     * automated commands, ensuring that the autopilot can override
     * user input when enabled, while maintaining a clear separation
     * of command sources.
     *
     * @param autoCmd The control command generated by the autopilot system.
     */
    void receiveCommandFromAutopilot(const ControlCommand& autoCmd);

    /**
     * @brief set loaded json config str
     * @param jsonConfigStr with spacecraft configs
     *
     * transferd from qstring to std::string in frontend
     */
    void setJsonConfigStr(const std::string &jsonConfigStr);

    /**
     * @brief Sets reset Boolean to true
     */
    void setResetBoolean();

    /**
     * @brief Takes over command from autopilot & convert them into command struct
     */
    void setAutoPilotCommand(const double &autoThrust);
};

#endif
