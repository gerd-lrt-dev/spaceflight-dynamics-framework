#include "simcontrol.h"
#include "logger.h"
#include "Automation/adaptiveDescentController.h"
#include "Controller/pd_controller.h"

#include <iostream>
#include <stdexcept>

//***********************************************************
//*************        Private                   ************
//***********************************************************
void simcontrol::buildSimulationEnvironment(double t)
{
    // Instance classes
    landerSpacecraft    = std::make_unique<spacecraft>(landerMoon1);
    inputArbiter_       = std::make_unique<InputArbiter>();
}

customSpacecraft simcontrol::loadSpacecraftFromJsonString(const std::string& jsonString)
{
    nlohmann::json config;

    try
    {
        config = nlohmann::json::parse(jsonString);
    }
    catch (const nlohmann::json::parse_error& e)
    {
        throw std::runtime_error(
            std::string("JSON parse error: ") + e.what());
    }

    // Validate root structure
    if (!config.is_object())
    {
        throw std::runtime_error("Invalid JSON: expected spacecraft object");
    }

    std::cout << "simcontrol[loadSpacecraftFromJsonString]: Successfully parsed config string"
              << std::endl;

    return jsonConfigReader::parseLander(config);
}

void simcontrol::processCommands()
{
    ControlCommand activeCommand = inputArbiter_->chooseCommand();

    setTargetMainEngineThrust(activeCommand.mainEngine);
    setTargetRCSThrust(activeCommand.translation);
}

void simcontrol::runAutopilot(const SpacecraftState& currentSpacecraftstate, const int &engineNr, const double& dt)
{
    // --- Autopilot Control ---
    if(currentSpacecraftstate == SpacecraftState::Operational)
    {
        // Autopilot is used for main engine of spacecraft with index number 0!
        double autoThrust = autopilot_->setAutoThrustInNewton(controller_.get(), landerMoon1.engines_[0].maxThrust, landerSpacecraft->getVelocity().z, landerSpacecraft->getPosition().z - config_.radiusMoon, dt, landerMoon1.emptyMass + landerMoon1.fuelM, config_.moonGravity);
        double autoThrustNormalized = autopilot_->normalizAutoThrust(autoThrust, landerMoon1.engines_[0].maxThrust);
        ControlCommand autoCmd;
        autoCmd.mainEngine = autoThrustNormalized;
        receiveCommandFromAutopilot(autoCmd);
        processCommands();
    }
    else if (currentSpacecraftstate == SpacecraftState::Landed)
    {
        ControlCommand cmd;
        cmd.mainEngine          = 0.0;
        cmd.autopilotActive     = false;
        receiveCommandFromAutopilot(cmd);
        processCommands();
    }
}

//***********************************************************
//*************        Public                   ************
//***********************************************************

simcontrol::simcontrol(double t0) : initialTime(t0)
{
    autopilot_  = std::make_unique<AdaptiveDescentController>(landerMoon1.safeVelocity);
    controller_ = std::make_unique<PD_Controller>();
}

simcontrol::~simcontrol()
{

}

void simcontrol::initialize(const std::string& jsonConfigStr)
{
    landerMoon1 = loadSpacecraftFromJsonString(jsonConfigStr);

    buildSimulationEnvironment(initialTime);
}

void simcontrol::instanceLoggingAction()
{
    // Initialize logger once
    Logger::instance().init("/tmp/simulation.log");
}

simData simcontrol::runSimulation(const double dt)
{
    Logger& logger = Logger::instance();
    simData simdata_;

    try
    {
        logger.log("Simulation step started. dt = " + std::to_string(dt));

        // --- Autopilot Control ---
        runAutopilot(landerSpacecraft->getSpacecraftState(), 0, dt);
        landerSpacecraft->setConsoleText(autopilot_->getDescentMode());

        // --- Update spacecraft state (translation, velocity, etc.) ---
        landerSpacecraft->updateStep(dt);   ///< Updates simulation steps

        // --- Retrieve full simulation data ---
        simdata_ = landerSpacecraft->getFullSimulationData();   ///< SimData struct can be requested from frontend

        // --- Log results (adapt later to new state vector) ---
        /*
        logger.log("Step results send to UI - Pos: (" +
                   std::to_string(spacecraftData.pos.x) + ", " +
                   std::to_string(spacecraftData.pos.y) + ", " +
                   std::to_string(spacecraftData.pos.z) + "), Vel: (" +
                   std::to_string(spacecraftData.vel.x) + ", " +
                   std::to_string(spacecraftData.vel.y) + ", " +
                   std::to_string(spacecraftData.vel.z) + "), Acc: (" +
                   std::to_string(spacecraftData.acc.x) + ", " +
                   std::to_string(spacecraftData.acc.y) + ", " +
                   std::to_string(spacecraftData.acc.z) + "), Fuel: " +
                   std::to_string(spacecraftData.fuelMass)
                   );
        */
    }
    catch (const std::exception& e)
    {
        std::cerr << "[EXCEPTION] in runSimulation: " << e.what() << std::endl;
        logger.log(std::string("Exception in runSimulation: ") + e.what());
        throw; // rethrow
    }
    catch (...)
    {
        std::cerr << "[EXCEPTION] Unknown exception in runSimulation." << std::endl;
        logger.log("Unknown exception in runSimulation.");
        throw;
    }

    return simdata_;
}

void simcontrol::receiveCommandFromFrontEnd(const ControlCommand& userCmd)
{
    inputArbiter_->receiveUserControlCommand(userCmd);
}

void simcontrol::receiveCommandFromAutopilot(const ControlCommand& autoCmd)
{
    inputArbiter_->receiveAutoControlCommand(autoCmd);
}

void simcontrol::setJsonConfigStr(const std::string &jsonConfigStr)
{
    jsonConfigString = jsonConfigStr;
}

void simcontrol::setTargetMainEngineThrust(const double& thrustPercent, const double& thrustInNewton)
{
    landerSpacecraft->setMainEngineThrust(thrustPercent);
}

// Function is going to get obsolet when RCS model is introduced

void simcontrol::setTargetRCSThrust(const Vector3 &ENU_translation)
{
    landerSpacecraft->setTargetRCSThrust(ENU_translation);
}

void simcontrol::setResetBoolean()
{
    resetRequested = true;
}

void simcontrol::setAutoPilotCommand(const double &autoThrust)
{
    receiveCommandFromAutopilot(cmd_);
}
