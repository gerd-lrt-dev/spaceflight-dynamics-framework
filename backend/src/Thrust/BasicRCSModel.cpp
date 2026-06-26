#include "Thrust/BasicRCSModel.h"

// -------------------------------------------------------------------------
// Public class methods
// -------------------------------------------------------------------------
basicRCSModel::basicRCSModel(const RCSEngineConfig& rcsConfig, FuelState fState)
    : rcsConfig_(RCSEngineConfig::Create(rcsConfig.engineActivated,
                                         rcsConfig.id,
                                         rcsConfig.name,
                                         rcsConfig.type,
                                         rcsConfig.axis,
                                         rcsConfig.tankID,
                                         rcsConfig.Isp,
                                         rcsConfig.maxThrust,
                                         rcsConfig.commandDelay,
                                         rcsConfig.tauOn,
                                         rcsConfig.tauOff,
                                         rcsConfig.minimumPulseWidth,
                                         rcsConfig.direction,
                                         rcsConfig.position)
                ),
    fuelstate_(fState)
{
    // Set meta data for thrust state struct
    thruststate_.engineID   = rcsConfig_.id;
    thruststate_.engineName = rcsConfig_.name;
    thruststate_.axis       = rcsConfig_.axis;
    thruststate_.SBF_direction  = rcsConfig_.direction;

    // Activate engine
    rcsConfig_.engineActivated = true;
}

void basicRCSModel::updateThrust(const double &dt)
{
    totalEngineTime += dt;

    // Simulate the time required to adjust the valve
    const double delayedTime = totalEngineTime - rcsConfig_.commandDelay;

    while(!commandBuffer.empty() && commandBuffer.front().time <= delayedTime)
    {
        cmdInputDelayed = commandBuffer.front().cmd;
        commandBuffer.pop_front();
    }

    thruststate_.currentThrust  = calcThrust(cmdInputDelayed, rcsConfig_.tauOn, rcsConfig_.tauOff, dt, rcsConfig_.maxThrust);

    fuelstate_.consumptionRate  = calcMassFlow(thruststate_.currentThrust, rcsConfig_.Isp, envConfig_.earthGravity);

    fuelstate_.massCurrent      = calcFuelReduction(fuelstate_.massCurrent, fuelstate_.consumptionRate, dt);

    /*
    std::cout
        << "[basicRCSModel]-updateThrust | "
        << "Engine: " << rcsConfig_.name
        << " | Cmd: " << cmdInput
        << " | CmdDelayed: " << cmdInputDelayed
        << " | ThrustState: " << thrustState
        << " | CurrentThrust: " << thruststate_.currentThrust << " N"
        << " | FuelFlow: " << fuelstate_.consumptionRate << " kg/s"
        << " | FuelRemaining: " << fuelstate_.massCurrent << " kg"
        << std::endl;
    */
}

// -------------------------------------------------------------------------
// Public setter
// -------------------------------------------------------------------------
void basicRCSModel::setEnginePowerSwitch(bool activateEngine)
{
    rcsConfig_.engineActivated = activateEngine;
}

void basicRCSModel::setTarget(const double& tThrust)
{
    if (rcsConfig_.maxThrust <= 0.0)
    {
        std::cerr << "[basicRCSModel]-setTarget- Invalid maxThrust. Command rejected."
                  << std::endl;
        return;
    }

    double normalizedCommand = tThrust / rcsConfig_.maxThrust;

    // Process dynamic target state to thrustState Struct
    thruststate_.targetThrust           = tThrust;
    thruststate_.targetThrustPercentage = normalizedCommand;

    setTargetInPercentage(normalizedCommand);
}

void basicRCSModel::setTargetInPercentage(const double& tThrustInPercentage)
{
    int target = convertToBinaryCommand(tThrustInPercentage);

    thruststate_.targetThrustPercentage = static_cast<double>(target);
    thruststate_.targetThrust = static_cast<double>(target) * rcsConfig_.maxThrust;

    cmdInput = target;

    commandBuffer.push_back(RCSCommandSample{
            .time = totalEngineTime,
            .cmd = cmdInput
        }
        );
}

// -------------------------------------------------------------------------
// Public getter
// -------------------------------------------------------------------------

int basicRCSModel::getEngineID() const
{
    return rcsConfig_.id;
}

std::string basicRCSModel::getEngineName() const
{
    return rcsConfig_.name;
}

std::string basicRCSModel::getEngineType() const
{
    return rcsConfig_.type;
}

double basicRCSModel::getTargetThrust() const
{
    return thruststate_.targetThrust;
    //return cmdInput * rcsConfig_.maxThrust;
}

double basicRCSModel::getCurrentThrust() const
{
    return thruststate_.currentThrust;
}

Eigen::Vector3d basicRCSModel::getSBF_DirectionOfThrust() const
{
    return rcsConfig_.direction;
}

double basicRCSModel::getFuelConsumption() const
{
    return fuelstate_.consumptionRate;
}

double basicRCSModel::getCurrentFuelMass() const
{
    return fuelstate_.massCurrent;
}

double basicRCSModel::getTankID() const
{
    return rcsConfig_.tankID;
}

double basicRCSModel::getMaxThrust() const
{
    return rcsConfig_.maxThrust;
}
// -------------------------------------------------------------------------
// Private calculation methods
// -------------------------------------------------------------------------
double basicRCSModel::calcThrust(const int cmdInputDelayed, const double &tauOn, const double &tauOff, const double &dt, const double &nominalThrust)
{
    const double targetState = static_cast<double>(cmdInputDelayed);

    const double tau = (cmdInputDelayed == 1) ? tauOn : tauOff;

    thrustState = integrateFirstOrderState(thrustState, targetState, tau, dt, IntegrationMethod::ExactFirstOrder);

    return calcNominalThrust(nominalThrust, thrustState);
}

double basicRCSModel::integrateFirstOrderState(double state, double target, double tau, double dt, IntegrationMethod method)
{
    if (tau <= 0.0) return target;

    switch (method) {
        case IntegrationMethod::Euler:
        {
            const double alpha = std::clamp(dt / tau, 0.0, 1.0);
            return state + (target - state) * alpha;
        }
        case IntegrationMethod::ExactFirstOrder:
        {
            const double alpha = 1.0 - std::exp(-dt / tau);
            return state + (target - state) * alpha;
        }
        default:
        {
            return state;
        }
    }
}

double basicRCSModel::calcNominalThrust(const double nominalThrust, const double state)
{
    return nominalThrust * state;
}

int basicRCSModel::convertToBinaryCommand(double input)
{
    int target = 0;

    // Check whether the user input is binary
    if (input != 0.0 && input != 1.0)
    {
        std::cerr << "[basicRCSModel]-setTargetInPercentage- BasicRCSModel only accept binary input. Input is going to round up" << std::endl;

        if (input > 0.0 && input <= 0.49)
        {
            std::cerr << "[basicRCSModel]-setTargetInPercentage- Command Input is rounded down to zero " << std::endl;
            target = 0;
        }
        else if (input >0.49 && input < 1.0)
        {
            std::cerr << "[basicRCSModel]-setTargetInPercentage- Command Input is rounded up to one" << std::endl;
            target = 1;
        }
        else
        {
            std::cerr << "[basicRCSModel]-setTargetInPercentage- Command Input is invalid, because the value is ether smaller than zero or higher than one." << std::endl;
            std::cerr << "[basicRCSModel]-setTargetInPercentage- Command Input can not be processed!" << std::endl;
        }
    }
    else
    {
        target = static_cast<int>(input);
    }

    return target;
}

double basicRCSModel::calcFuelReduction(const double &fuelMass, const double &massFlowFuel, const double &dt)
{
    double newFuelMass = fuelMass - (massFlowFuel * dt);

    return newFuelMass;
}

double basicRCSModel::calcMassFlow(const double &currenThrust, const double &Isp, const double &earthGravity)
{
    return currenThrust / (Isp * earthGravity);
}
