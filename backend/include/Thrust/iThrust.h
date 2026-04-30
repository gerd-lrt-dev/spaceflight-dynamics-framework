#pragma once

#include "vector3.h"
#include <iostream>

class IThrustModel{
public:

    virtual             ~IThrustModel() = default;

    virtual void        updateThrust(const double &dt) = 0;

    virtual double      calcFuelReduction(const double &fuelMass,const double &massFlowFuel,const double &dt) = 0;

    virtual void        setEnginePowerSwitch(bool activateEngine) = 0;

    virtual void        setTarget(const double &tThrust) = 0;

    virtual void        setTargetInPercentage(const double &tThrustInPercentage) = 0;

    virtual int         getEngineID() const = 0;

    virtual std::string getEngineType() const = 0;

    virtual double      getTargetThrust() const = 0;

    virtual double      getCurrentThrust() const = 0;

    virtual Vector3     getDirectionOfThrust() const = 0;

    virtual double      getFuelConsumption() const = 0;

    virtual double      getCurrentFuelMass() const = 0;

    virtual double      getTankID() const = 0;

    virtual double      getMaxThrust() const = 0;
};
