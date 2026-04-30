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
void Thrust::setTargetThrustInNewton(EngineType engine, const double &tMainEngineThrust, const Vector3 &tRCSThrust)
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
        else if (engine == EngineType::RCS)
        {
            // To be done...
        }
}

void Thrust::setTargetThrustInPercentage(EngineType engine, const double &tMainEngineThrust, const Vector3 &tRCSThrust)
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
                model->setTargetInPercentage(tMainEngineThrust);
            }
        }
    }
    else if (engine == EngineType::RCS)
    {
        // To be done...
        std::cout << "RCS Thrust: \n"
                  << "x: " << tRCSThrust.x << "\n"
                  << "y: " << tRCSThrust.y << "\n"
                  << "z: " << tRCSThrust.z << "\n"
                  << std::endl;
    }
}

void Thrust::shutDownAllEngines() const
{
    for (const auto& model : models_)
    {
        model->setTarget(0.0);
    }
}

void Thrust::initializeEngines(std::vector<EngineConfig> &engineConfigs, const std::vector<FuelTank> &tanks)
{
    // -----------------------------------------
    // Initialize tanks
    // -----------------------------------------
    addFuelTank(tanks);

    // -----------------------------------------
    // Initialize engine configs
    // -----------------------------------------

    for (const auto &cfg_ : engineConfigs)
    {
        FuelState state;
        state.consumptionRate = 0.0;
        if (cfg_.type == "main")
        {
            std::cout << "[Thrust]-initializeEngines- Configured Main Engine" << std::endl;
            addModel(std::make_unique<basicMainEngineModel>(cfg_, state));
        }
        else if (cfg_.type == "translation")
        {
            std::cout << "[Thrust]-initializeEngines- Configured RCS translational engine" << std::endl;
            std::cout << "[Thrust]-initializeEngines- RCS Model not included yet" << std::endl;
            addModel(std::make_unique<basicMainEngineModel>(cfg_, state));
        }
        else if (cfg_.type == "rotation")
        {
            std::cout << "[Thrust]-initializeEngines- Configured RCS rotational engine" << std::endl;
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

Vector3 Thrust::getTargetThrust(EngineType engine) const
{
    Vector3     total{0.0, 0.0, 0.0};
    Vector3     dir{0.0, 0.0, 0.0};
    double      thrust(0.0);

    if (engine == EngineType::All)
    {
        for (const auto& model : models_)
        {
            dir = model->getDirectionOfThrust();
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
                dir = model->getDirectionOfThrust();
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
                dir = model->getDirectionOfThrust();
                thrust = model->getTargetThrust();
                total += dir * thrust;
            }
        }
    }

    return total;
}

Vector3 Thrust::getCurrentThrust(EngineType engine) const
{
    Vector3     total{0.0, 0.0, 0.0};
    Vector3     dir{0.0, 0.0, 0.0};
    double      thrust(0.0);

    if (engine == EngineType::All)
    {
        for (const auto& model : models_)
        {
            dir = model->getDirectionOfThrust();
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
                dir = model->getDirectionOfThrust();
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
                dir = model->getDirectionOfThrust();
                thrust = model->getCurrentThrust();

                total += dir * thrust;
            }
        }
    }
    return total;
}

Vector3 Thrust::getCurrentThrustInPercentage(EngineType engine) const
{
    Vector3     total{0.0, 0.0, 0.0};
    Vector3     dir{0.0, 0.0, 0.0};
    double      thrustInPercentage(0.0);

    if (engine == EngineType::All)
    {
        for (const auto& model : models_)
        {
            dir = model->getDirectionOfThrust();
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
                dir = model->getDirectionOfThrust();
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
                dir = model->getDirectionOfThrust();
                thrustInPercentage = model->getCurrentThrust() / model->getMaxThrust();
            }

            total += dir * thrustInPercentage;
        }
    }

    return total;
}

Vector3 Thrust::getDirectionOfThrust(EngineType engine, int engineID) const
{
    Vector3 dir{0.0, 0.0, 0.0};

    if (engine == EngineType::MainEngine && engineID == 0)
    {
        for (const auto& model : models_)
        {
            dir = model->getDirectionOfThrust();
        }
    }
    else if(engineID > 0)
    {
        for (const auto& model : models_)
        {
            if (model->getEngineID() == engineID)
            {
                dir = model->getDirectionOfThrust();
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


