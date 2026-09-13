#include "Thrust.h"
#include <iostream>
#include <iomanip>
#include "Thrust/EngineType.h"
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
    if (engine == EngineType::MainEngine)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == EngineType::MainEngine)
            {
                model->setTarget(tMainEngineThrust);
            }
        }
    }

    if (engine == EngineType::RCS_translation)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == EngineType::RCS_translation)
            {
                const double command = RCSControlAllocator::mapTranslationCommandToThrusterNewton(tRCSThrust, model->getSBF_DirectionOfThrust());

                model->setTarget(command);
            }
        }
    }

    if (engine == EngineType::RCS_rotation)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == EngineType::RCS_rotation)
            {
                const double command = RCSControlAllocator::mapTranslationCommandToThrusterNewton(tRCSThrust, model->getSBF_DirectionOfThrust());

                model->setTarget(command);
            }
        }
    }
}

void Thrust::setTargetThrustInPercentage(EngineType engine, const double &tMainEngineThrust, const Eigen::Vector3d &tRCSThrust)
{
    if (engine == EngineType::MainEngine)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == EngineType::MainEngine)
            {
                model->setTargetInPercentage(tMainEngineThrust);
            }
        }
    }

    if (engine == EngineType::RCS_translation)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineType() == EngineType::RCS_translation)
            {
                const double command = RCSControlAllocator::mapTranslationCommandToThrusterPercentage(tRCSThrust, model->getSBF_DirectionOfThrust());
                if (command != 0)
                std::cout << "[Thrust]-setTargetThrustInPercentage-: Thrust allocated for direction: \n" << model->getSBF_DirectionOfThrust() << std::endl;

                model->setTargetInPercentage(command);
            }
        }
    }
}

void Thrust::setRCSRotationTargetThrustInPercentage(const Eigen::Vector3d &RCSThrust, const Eigen::Vector3d &centerOfMass)
{
    for (const auto &model : models_)
    {
        if (model->getEngineType() == EngineType::RCS_rotation)
        {
            const double command =
                RCSControlAllocator::mapAttitudeCommandToThrusterPercentage(RCSThrust, model->getEnginePosition(), centerOfMass, model->getSBF_DirectionOfThrust());

            if (command != 0.0)
            {
                std::cout
                    << "[Thrust]-setRCSRotationTargetThrustInPercentage- "
                    << "Thrust allocated to <"
                    << model->getEngineName()
                    << ">."
                    << std::endl;
            }

            model->setTargetInPercentage(command);
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
        if (cfg_.type == EngineType::MainEngine)
        {
            std::cout
                << "[Thrust]-initializeEngines- Configured Main Engine | "
                << cfg_.name
                << " | Direction: ("
                << cfg_.direction.x() << ", "
                << cfg_.direction.y() << ", "
                << cfg_.direction.z() << ")"
                << std::endl;

            addModel(std::make_unique<basicMainEngineModel>(cfg_));
        }
        else
        {
            std::cerr << "[Thrust]-initializeEngines- Engine Type unknown!!" << std::endl;
            return;
        }
    }

    // -----------------------------------------
    // Initialize RCS engines
    // -----------------------------------------

    std::cout << "[Thrust]-initializeEngines- Found " << RCSEngines.size() << " engines to configure." << std::endl;

    for (const auto& rcscfg_ : RCSEngines)
    {
        if (rcscfg_.type == EngineType::RCS_translation || rcscfg_.type == EngineType::RCS_rotation)
        {
            std::cout
                << "[Thrust]-initializeEngines- Configured RCS Engine | "
                << std::left
                << "Type: " << std::setw(12) << engineTypeToString(rcscfg_.type)
                << " | Name: " << std::setw(24) << rcscfg_.name
                << " | Axis: " << std::setw(3) << rcscfg_.axis
                << " | Direction: ("
                << std::fixed << std::setprecision(1)
                << std::setw(4) << rcscfg_.direction.x() << ", "
                << std::setw(4) << rcscfg_.direction.y() << ", "
                << std::setw(4) << rcscfg_.direction.z() << ")"
                << '\n';

            addModel(std::make_unique<basicRCSModel>(rcscfg_));
        }
        else
        {
            std::cerr << "[Thrust]-initializeEngines- Engine Type unknown!!" << std::endl;
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

void Thrust::updatePropulsion(double dt)
{
    if (getFuelMassOfAllTanks() > 0.0)
    {
        // Update thrust for all engines
        for (int i = 0; i < models_.size(); ++i)
        {
            // Update thrust
            models_[i]->updateThrust(dt);

            // Update torque
            models_[i]->updateTorque();

            // Update fuel reduction
            for (auto& tank : tanks_)
            {
                if (tank.id == models_[i]->getTankID())
                {
                    tank.consume(models_[i]->getFuelConsumption() * dt);
                    break;
                }
            }
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

    for (const auto& model : models_)
    {
        if (model->getEngineType() == engine)
        {
            dir = model->getSBF_DirectionOfThrust();
            thrust = model->getTargetThrust();
            total += dir * thrust;
        }
    }

    return total;
}

Eigen::Vector3d Thrust::getCurrentThrustOfOneEngine(EngineType engine) const
{
    Eigen::Vector3d     total{0.0, 0.0, 0.0};
    Eigen::Vector3d     dir{0.0, 0.0, 0.0};
    double      thrust(0.0);

    for (const auto& model : models_)
    {
        if (model->getEngineType() == engine)
        {
            dir = model->getSBF_DirectionOfThrust();
            thrust = model->getCurrentThrust();

            total += dir * thrust;
        }
    }

    return total;
}

Eigen::Vector3d Thrust::getCurrentThrustOfAllEngines() const
{
    Eigen::Vector3d     total{0.0, 0.0, 0.0};
    Eigen::Vector3d     dir{0.0, 0.0, 0.0};
    double      thrust(0.0);

    for (const auto& model : models_)
    {
        dir = model->getSBF_DirectionOfThrust();
        thrust = model->getCurrentThrust();

        total += dir * thrust;
    }

    return total;
}

Eigen::Vector3d Thrust::getCurrentThrustInPercentage(EngineType engine) const
{
    Eigen::Vector3d     total{0.0, 0.0, 0.0};
    Eigen::Vector3d     dir{0.0, 0.0, 0.0};
    double      thrustInPercentage(0.0);


        for (const auto& model : models_)
        {
            if (model->getEngineType() == engine)
            {
                dir = model->getSBF_DirectionOfThrust();
                thrustInPercentage = model->getCurrentThrust() / model->getMaxThrust();
            }

            total += dir * thrustInPercentage;
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
            if (model->getEngineType() == EngineType::MainEngine)
            {
                dir = model->getSBF_DirectionOfThrust();
            }
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

    return dir;
}

Eigen::Vector3d Thrust::getTotalTorque() const
{
    Eigen::Vector3d torque = {0.0, 0.0, 0.0};

    for (const auto& model : models_)
    {
        torque += model->getCurrentTorque();
    }

    return torque;
}

std::vector<RCS_ThrustState> Thrust::getFullRCSEngineData() const
{
    std::vector<RCS_ThrustState> rcsThrustStates;

    for (const auto& model : models_)
    {
        if (model->getEngineType() == EngineType::RCS_translation || model->getEngineType() == EngineType::RCS_rotation)
        {
            RCS_ThrustState state{};

            state.engineID                  = model->getEngineID();
            state.engineName                = model->getEngineName();
            state.currentThrust             = model->getCurrentThrust();
            state.targetThrust              = model->getTargetThrust();
            state.targetThrustPercentage    = model->getTargetThrust() / model->getMaxThrust();
            state.maxThrust                 = model->getMaxThrust();
            state.SBF_direction             = model->getSBF_DirectionOfThrust();

            rcsThrustStates.push_back(state);
        }
    }

    return rcsThrustStates;
}

double Thrust::getTotalFuelConsumption() const
{
    double sum = 0.0;

    // For single engine if needed
    /*
    for (const auto& model : models_)
    {
        if (model->getEngineType() == engine)
        {
            sum += model->getFuelConsumption();
        }
    }
    */

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


