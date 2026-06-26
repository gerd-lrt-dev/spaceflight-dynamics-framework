#include "Thrust.h"
#include <iostream>
// ---Private-------------------------------------



// ---Public--------------------------------------
Thrust::Thrust()
{
}

Thrust::~Thrust()
{

}

// -------------------------------------------------------------------------
// Public setter functions
// -------------------------------------------------------------------------
void Thrust::setTargetThrustInNewton(EngineType engine, const double &tMainEngineThrust, const Eigen::Vector3d &tRCSThrust)
{
    if (engine == EngineType::All)
    {
        std::cerr << "[Thrust]-setTargetThrustInPercentage- Engine Type is ALL but the type must be specified!" << std::endl;
        return;
    }

        if (engine == EngineType::MainEngine)
        {
            for (const auto& model : models_)
            {
                if (model->getEngineType() == "main")
                {
                    model->setTarget(tMainEngineThrust);
                }
            }
        }

        if (engine == EngineType::RCS)
        {
            for (const auto& model : models_)
            {
                if (model->getEngineType() == "translation")
                {
                    const double command = RCSControlAllocator::mapAxisCommandToThrusterNewton(tRCSThrust, model->getSBF_DirectionOfThrust());

                    model->setTarget(command);
                }
            }
        }
}

void Thrust::setTargetThrustInPercentage(EngineType engine, const double &tMainEngineThrust, const Eigen::Vector3d &tRCSThrust)
{
    if (engine == EngineType::All)
    {
        std::cerr << "[Thrust]-setTargetThrustInPercentage- Engine Type is ALL but the type must be specified!" << std::endl;
        return;
    }

    int counter(0);
    if (engine == EngineType::MainEngine)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == "main")
            {
                model->setTargetInPercentage(tMainEngineThrust);
            }
        }
    }

    if (engine == EngineType::RCS)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == "translation")
            {
                const double command = RCSControlAllocator::mapAxisCommandToThrusterPercentage(tRCSThrust, model->getSBF_DirectionOfThrust());

                model->setTargetInPercentage(command);
            }
        }
    }
}

void Thrust::shutDownAllEngines() const
{
    for (const auto& model : models_)
    {
        model->setTarget(0.0);
    }
}

void Thrust::initializeEngines(std::vector<EngineConfig>& engineConfigs, std::vector<RCSEngineConfig>& RCSEngines, const std::vector<FuelTank>& tanks)
{
    // -----------------------------------------
    // Initialize tanks
    // -----------------------------------------
    addFuelTank(tanks);

    // -----------------------------------------
    // Initialize main engines
    // -----------------------------------------

    for (const auto& cfg_ : engineConfigs)
    {
        FuelState state;
        state.consumptionRate = 0.0;

        if (cfg_.type == "main")
        {
            std::cout
                << "[Thrust]-initializeEngines- Configured Main Engine | "
                << cfg_.name
                << " | Direction: ("
                << cfg_.direction.x() << ", "
                << cfg_.direction.y() << ", "
                << cfg_.direction.z() << ")"
                << std::endl;

            addModel(std::make_unique<basicMainEngineModel>(cfg_, state));
        }
        else
        {
            std::cerr << "[Thrust]-initializeEngines- Engine Type unknown!!"
                      << std::endl;
            return;
        }
    }

    // -----------------------------------------
    // Initialize RCS engines
    // -----------------------------------------

    for (const auto& rcscfg_ : RCSEngines)
    {
        FuelState RCSFuelState;
        RCSFuelState.consumptionRate = 0.0;

        if (rcscfg_.type == "translation")
        {
            std::cout
                << "[Thrust]-initializeEngines- Configured Translational RCS Engine | "
                << rcscfg_.name
                << " | Axis: " << rcscfg_.axis
                << " | Direction: ("
                << rcscfg_.direction.x() << ", "
                << rcscfg_.direction.y() << ", "
                << rcscfg_.direction.z() << ")"
                << std::endl;

            addModel(std::make_unique<basicRCSModel>(rcscfg_, RCSFuelState));
        }
        else if (rcscfg_.type == "rotation")
        {
            std::cout
                << "[Thrust]-initializeEngines- Configured Rotational RCS Engine | "
                << rcscfg_.name
                << std::endl;
        }
        else
        {
            std::cerr << "[Thrust]-initializeEngines- Engine Type unknown!!"
                      << std::endl;
            return;
        }
    }
}

void Thrust::activateEngine(const size_t &engineNr)
{
    models_[engineNr]->setEnginePowerSwitch(true);
}

void Thrust::deactivateEngine(const size_t &engineNr)
{
    models_[engineNr]->setEnginePowerSwitch(false);
}

void Thrust::turnOffAllEngines()
{
    for (auto& model : models_)
    {
        model->setEnginePowerSwitch(false);
    }
}
void Thrust::updateThrust(double dt)
{
    if (getFuelMassOfAllTanks() > 0.0)
    {
        // Update thrust for all engines
        for (int i = 0; i < models_.size(); ++i)
        {
            models_[i]->updateThrust(dt);
            tanks_[models_[i]->getTankID()].mass = models_[i]->calcFuelReduction(tanks_[models_[i]->getTankID()].mass, models_[i]->getFuelConsumption(), dt);
        }
    }
    else
    {
        return;
    }
}

// --- Getter functions ---------------------------------------------

Eigen::Vector3d Thrust::getTargetThrust(EngineType engine) const
{
    Eigen::Vector3d     total{0.0, 0.0, 0.0};
    Eigen::Vector3d     dir{0.0, 0.0, 0.0};
    double      thrust(0.0);

    if (engine == EngineType::All)
    {
        for (const auto& model : models_)
        {
            dir = model->getSBF_DirectionOfThrust();
            thrust = model->getTargetThrust();

            total += dir * thrust;
        }
    }
    else if (engine == EngineType::MainEngine)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == "main")
            {
                dir = model->getSBF_DirectionOfThrust();
                thrust = model->getTargetThrust();
                total += dir * thrust;
            }
        }
    }
    else if (engine == EngineType::RCS)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == "translation" || model->getEngineType() == "rotation")
            {
                dir = model->getSBF_DirectionOfThrust();
                thrust = model->getTargetThrust();
                total += dir * thrust;
            }
        }
    }

    return total;
}

Eigen::Vector3d Thrust::getCurrentThrust(EngineType engine) const
{
    Eigen::Vector3d     total{0.0, 0.0, 0.0};
    Eigen::Vector3d     dir{0.0, 0.0, 0.0};
    double      thrust(0.0);

    if (engine == EngineType::All)
    {
        for (const auto& model : models_)
        {
            dir = model->getSBF_DirectionOfThrust();
            thrust = model->getCurrentThrust();

            total += dir * thrust;
        }
    }
    else if (engine == EngineType::MainEngine)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == "main")
            {
                dir = model->getSBF_DirectionOfThrust();
                thrust = model->getCurrentThrust();

                total += dir * thrust;
            }
        }
    }
    else if (engine == EngineType::RCS)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == "translation" || model->getEngineType() == "rotation")
            {
                dir = model->getSBF_DirectionOfThrust();
                thrust = model->getCurrentThrust();

                total += dir * thrust;
            }
        }
    }
    return total;
}

Eigen::Vector3d Thrust::getCurrentThrustInPercentage(EngineType engine) const
{
    Eigen::Vector3d     total{0.0, 0.0, 0.0};
    Eigen::Vector3d     dir{0.0, 0.0, 0.0};
    double      thrustInPercentage(0.0);

    if (engine == EngineType::All)
    {
        for (const auto& model : models_)
        {
            dir = model->getSBF_DirectionOfThrust();
            thrustInPercentage = model->getCurrentThrust() / model->getMaxThrust();

            total += dir * thrustInPercentage;
        }
    }
    else if (engine == EngineType::MainEngine)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == "main")
            {
                dir = model->getSBF_DirectionOfThrust();
                thrustInPercentage = model->getCurrentThrust() / model->getMaxThrust();
            }

            total += dir * thrustInPercentage;
        }
    }
    else if (engine == EngineType::RCS)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == "translation" || model->getEngineType() == "rotation")
            {
                dir = model->getSBF_DirectionOfThrust();
                thrustInPercentage = model->getCurrentThrust() / model->getMaxThrust();
            }

            total += dir * thrustInPercentage;
        }
    }

    return total;
}

Eigen::Vector3d Thrust::getDirectionOfThrust(EngineType engine, int engineID) const
{
    Eigen::Vector3d dir{0.0, 0.0, 0.0};

    if (engine == EngineType::MainEngine && engineID == 0)
    {
        for (const auto& model : models_)
        {
            dir = model->getSBF_DirectionOfThrust();
        }
    }
    else if(engineID > 0)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineID() == engineID)
            {
                dir = model->getSBF_DirectionOfThrust();
            }
        }
    }
    else
    {
        std::cerr << "[Thrust]-getDirectionOfThrust- Failed requesting direction of thrust" << std::endl;
        return {0.0, 0.0, 0.0};
    }
    return dir;
}

std::vector<RCS_ThrustState> Thrust::getFullRCSEngineData() const
{
    std::vector<RCS_ThrustState> rcsThrustStates;

    for (const auto& model : models_)
    {
        if (model->getEngineType() == "translation" || model->getEngineType() == "rotation")
        {
            RCS_ThrustState state{};

            state.engineID                  = model->getEngineID();
            state.engineName                = model->getEngineName();
            state.currentThrust             = model->getCurrentThrust();
            state.targetThrust              = model->getTargetThrust();
            state.targetThrustPercentage    = model->getTargetThrust() / model->getMaxThrust();
            state.SBF_direction             = model->getSBF_DirectionOfThrust();

            rcsThrustStates.push_back(state);
        }
    }

    return rcsThrustStates;
}

double Thrust::getFuelConsumption(EngineType engine) const
{
    double sum = 0.0;
    if (engine == EngineType::All)
    {
        for (const auto& model : models_)
        {
            sum += model->getFuelConsumption();
        }
    }
    else if (engine == EngineType::MainEngine)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == "main")
            {
                sum += model->getFuelConsumption();
            }
        }
    }
    else if (engine == EngineType::RCS)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == "translation" || model->getEngineType() == "rotation")
            {
                sum += model->getFuelConsumption();
            }
        }
    }
    for (const auto& model : models_)
    {
        sum += model->getFuelConsumption();
    }

    return sum;
}

const std::vector<FuelTank>& Thrust::getFuelTanks() const
{
    return tanks_;
}

void Thrust::addModel(std::unique_ptr<IThrustModel> model)
{
    models_.push_back(std::move(model));
}

void Thrust::addFuelTank(const std::vector<FuelTank> &tanks)
{
    tanks_ = tanks;

    for (size_t i = 0; i < tanks.size(); ++i)
    {

        //std::cout << "[Thrust]-addFuelTank- Add Tank " << tanks[i].
    }

    std::cout << "[Thrust] Added " << tanks.size() << " tanks" << std::endl;
}

double Thrust::getFuelMassOfAllTanks() const
{
    FuelTank tmpTank;
    double fuelMassOfAllTanks = 0.0;

    for (auto &tmpTank : tanks_)
    {
        fuelMassOfAllTanks += tmpTank.mass;
    }

    return fuelMassOfAllTanks;
}


