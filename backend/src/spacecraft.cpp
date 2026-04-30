#include "spacecraft.h"
#include "spacemath.h"
#include "Physics/basicMoonGravityModel.h"
#include "Integrators/eulerIntegrator.h"
#include "Sensory_Perception/sensorModel.h"

#include <iostream>
// -------------------------------------------------------------------------
// Private
// -------------------------------------------------------------------------

void spacecraft::setDefaultValues()
{
    spacecraftIntegrity = 1.0;
    spacecraftState_ = SpacecraftState::Operational;
    totalMass = landerMoon.emptyMass + landerMoon.fuelM;
    state_.I_Position = landerMoon.I_initialPos;
    state_.I_Velocity = landerMoon.I_initialVelocity;

    thrustOrchestration.initializeEngines(landerMoon.engines_, landerMoon.tanks_);


    // TODO just testing here optimization

    //double h0 = landerMoon.I_initialPos.z;      // Höhe über Oberfläche
    //double v0 = landerMoon.I_initialVelocity.z; // vertikale Geschwindigkeit
    //double m0 = totalMass;

    //std::vector<double> thrust = compute_optimization(h0, v0, m0, 0.5);

}

void spacecraft::updateTotalMassOnFuelReduction(double emptyMass, double fuelMass)
{
    state_.totalMass = emptyMass + fuelMass;
}

void spacecraft::updateMovementData(double dt)
{
    // --- Pre-Checks ---
    if (!physics_)
    {
        std::cerr << "[CRASH] physics_ pointer is null!" << std::endl;
        return;
    }

    // --- Compute acceleration ---
    //TODO: eliminate minus with request thrust when coordinate transformation class is written
    Vector3 acceleration = physics_->computeAcc(getPosition(), getVelocity(), getTotalMass(), -requestTotalThrust());

    // --- Compute velocity ---
    Vector3 velocity = physics_->computeVel(getVelocity(), acceleration, dt);

    // --- Compute position ---
    Vector3 position = physics_->computePos(getPosition(), velocity, acceleration, dt);

    // --- TODO: Compute orientation and angular velocity ---
    // ...

    // --- TODO: Update total mass ---
    // ...

    updateGLoad(acceleration, environmentConfig_.moonGravityVec);

    // --- Commit to state vector ---
    setVelocity(velocity);
    setPosition(position);
    //setGload(GLoad);
}

void spacecraft::updateMovementDataToZero(double dt)
{
    Vector3 zeroVector = {0.0, 0.0, 0.0};
    // --- Commit to state vector ---
    setVelocity(zeroVector);
    updateGLoad(zeroVector, environmentConfig_.moonGravityVec);
}

void spacecraft::updateGLoad(const Vector3& totalAcceleration, const Vector3& gravityAcceleration)
{
    bool isLanded = false;

    if (spacecraftState_ == SpacecraftState::Landed) isLanded = true;

    GLoad = physics_->computeGLoad(totalAcceleration, environmentConfig_.moonGravityVec, isLanded);
}

void spacecraft::setSpacecraftState(SpacecraftState newState)
{
    spacecraftState_ = newState;
}

SpacecraftState spacecraft::getSpacecraftState() const
{
    return spacecraftState_;
}

void spacecraft::applyLandingDamage(double impactVelocity)
{
    double KE(0), KEref(0), damageInPercent(0);

    KEref   = spacemath::kineticEnergy(totalMass, landerMoon.safeVelocity);
    KE      = spacemath::kineticEnergy(totalMass, impactVelocity);

    damageInPercent = KE / KEref;

    spacecraftIntegrity += -damageInPercent;
}

void spacecraft::setPosition(const Vector3& pos)
{
    state_.I_Position = pos;
}

void spacecraft::setVelocity(const Vector3& vel)
{
    state_.I_Velocity = vel;
}

void spacecraft::setOrientation(const Quaternion& q)
{
    state_.IB_Orientation = q;
}

void spacecraft::setAngularVelocity(const Vector3& angVel)
{
    state_.B_AngularVelocity = angVel;
}

// -------------------------------------------------------------------------
// Public
// -------------------------------------------------------------------------
spacecraft::spacecraft(customSpacecraft lMoon) : landerMoon(lMoon)
    {
        // initialize
        std::shared_ptr<IPhysicsModel> model_       = std::make_shared<BasicMoonGravityModel>(environmentConfig_);
        std::shared_ptr<IIntegrator> integrator_    = std::make_shared<EulerIntegrator>();
        std::shared_ptr<ISensor> sensor_            = std::make_shared<SensorModel>(environmentConfig_);

        physics_ = std::make_unique<physics>(model_, integrator_, sensor_);

        setDefaultValues();
    };

spacecraft::~spacecraft()
{
}

void spacecraft::updateStep(double dt)
{
    // Update mass data
    updateTotalMassOnFuelReduction(landerMoon.emptyMass, getTotalFuelMass());

    thrustOrchestration.updateThrust(dt);

    // Update time systems are running
    time += dt;

    // Apply landing damage
    if (state_.I_Position.z <= environmentConfig_.radiusMoon)
    {
        applyLandingDamage(state_.I_Velocity.z);
    }

    updateSpacecraftIntegrity();

    // Update Movement data due to spacecraft state
    switch (spacecraftState_)
    {
    case SpacecraftState::Operational:
        updateMovementData(dt);
        break;

    case SpacecraftState::Landed:
        // Translation disabled, rotation optional
        updateMovementDataToZero(dt);
        thrustOrchestration.shutDownAllEngines();
        break;

    case SpacecraftState::Crashed:
        // Freeze kinematics, allow logging
        break;

    case SpacecraftState::Destroyed:
        // Do nothing
        break;
    }
}

void spacecraft::updateSpacecraftIntegrity()
{
    if(spacecraftIntegrity > 1.0) spacecraftIntegrity = 1.0;
    if(spacecraftIntegrity < 0.0) spacecraftIntegrity = 0.0;

    // --- Update spacecraft state ---

    // 1. Completely destroyed (terminal)
    if (spacecraftIntegrity <= 0.0)
    {
        spacecraftState_ = SpacecraftState::Destroyed;
        return;
    }

    // 2. Structural failure (terminal but stable)
    if (spacecraftIntegrity < landerMoon.structuralIntegrity)
    {
        spacecraftState_ = SpacecraftState::Crashed;
        return;
    }

    // 3. Successful touchdown
    if (getPosition().z <= environmentConfig_.radiusMoon)
    {
        spacecraftState_ = SpacecraftState::Landed;
        return;
    }

    // 4. Still operational (possibly damaged)
    spacecraftState_ = SpacecraftState::Operational;
}

void spacecraft::setMainEngineThrust(const double &targetThrustInPercentage)
{
    (getSpacecraftState() == SpacecraftState::Operational) ? thrustOrchestration.setTargetThrustInPercentage(EngineType::MainEngine, targetThrustInPercentage) : thrustOrchestration.setTargetThrustInPercentage(EngineType::MainEngine, 0.0);
}

void spacecraft::setTargetRCSThrust(const Vector3 &targetThrustInPercentage)
{
    (getSpacecraftState() == SpacecraftState::Operational) ? thrustOrchestration.setTargetThrustInPercentage(EngineType::RCS, 0.0, targetThrustInPercentage) : thrustOrchestration.setTargetThrustInPercentage(EngineType::RCS, 0.0, targetThrustInPercentage);
}

void spacecraft::setConsoleText(const std::string &txt)
{
    consoleTxt = txt;
}

std::vector<double> spacecraft::compute_optimization(double h0, double v0, double m0, double dt)
{
    ThrustOptimizationProblem problem;

    // -----------------------------
    // Initial state
    // -----------------------------
    problem.x0.h = h0;
    problem.x0.v = v0;
    problem.x0.m = m0;

    // -----------------------------
    // Physical model
    // -----------------------------
    problem.params.mu_moon = 4.9048695e12;   // [m^3/s^2]
    problem.params.R_moon  = 1.7374e6;       // [m]
    problem.params.Isp     = 300.0;          // [s]
    problem.params.g0      = 9.80665;        // [m/s^2]

    // -----------------------------
    // Horizon
    // -----------------------------
    problem.dt = 0.1;
    problem.N  = 200;

    // -----------------------------
    // Cost weights
    // -----------------------------
    problem.w_fuel        = 5.0;
    problem.w_terminal    = 5000;
    problem.w_hf          = 0.05;
    problem.w_vf          = 50;
    problem.w_v_constraint= 30.0;
    problem.w_smooth      = 0.05;
    problem.w_descent     = 0.0;

    // -----------------------------
    // References
    // -----------------------------
    problem.h_ref = std::max(1.0, std::abs(h0 - environmentConfig_.radiusMoon));
    problem.v_safe = 2.5;                    // [m/s] touchdown safe speed
    problem.m_ref = m0;
    problem.T_ref = 7000.0;

    // -----------------------------
    // Constraints
    // -----------------------------
    problem.m_dry = landerMoon.emptyMass;

    problem.v_min = -50.0;   // max fall speed
    problem.v_max =  50.0;   // max upward drift

    // -----------------------------
    // Target properties
    // -----------------------------
    problem.r_target = environmentConfig_.radiusMoon;

    // -----------------------------
    // Optimize
    // -----------------------------
    ThrustOptimizer optimizer;
    return optimizer.optimize(problem, 7000.0);
}

Vector3 spacecraft::requestTotalThrust() const
{
    return thrustOrchestration.getCurrentThrust();
}

Vector3 spacecraft::requestMainEngineTargetThrust() const
{
    
    return thrustOrchestration.getTargetThrust(EngineType::MainEngine);
}

Vector3 spacecraft::requestMainEngineThrust() const
{
    
    return thrustOrchestration.getCurrentThrust(EngineType::MainEngine);
}

Vector3 spacecraft::requestMainEngineThrustInPercentage() const
{
    return thrustOrchestration.getCurrentThrustInPercentage(EngineType::MainEngine);
}

Vector3 spacecraft::requestMainEngineDirection() const
{
    return thrustOrchestration.getDirectionOfThrust(EngineType::MainEngine);
}

double spacecraft::requestMainEngineLiveFuelConsumption() const
{
    return thrustOrchestration.getFuelConsumption(EngineType::MainEngine);
}

void spacecraft::setInitalPosition(const Vector3& position)
{
    landerMoon.I_initialPos = position;
}

void spacecraft::setInitalVelocity(const Vector3& velocity)
{
    landerMoon.I_initialVelocity = velocity;
}

simData spacecraft::getFullSimulationData() const
{
    simData simData_;

    simData_.statevector_ = getState();

    // Reduce height by radius of moon
    simData_.statevector_.I_Position.z -= environmentConfig_.radiusMoon;

    // Fill struct with data for emitting signal to UI
    simData_.spacecraftState_ = spacecraftState_;


    simData_.ME_ThrustState_.current            = requestMainEngineThrust().dot(requestMainEngineDirection());
    simData_.ME_ThrustState_.target             = requestMainEngineTargetThrust().dot(requestMainEngineDirection());
    simData_.ME_ThrustState_.targetPercentage   = requestMainEngineThrustInPercentage().dot(requestMainEngineDirection());
    simData_.ME_ThrustState_.direction          = requestMainEngineDirection();

    simData_.tanks    = getFuelTanks();
    simData_.fuelMass = getTotalFuelMass();
    simData_.fuelFlow = requestMainEngineLiveFuelConsumption();

    simData_.GLoad = getGload();

    simData_.output = getConsoleTxt();

    return simData_;
}

double spacecraft::getIntegrity()
{
    return spacecraftIntegrity;
}

const StateVector& spacecraft::getState() const
{
    return state_;
}

Vector3 spacecraft::getPosition() const
{
    return state_.I_Position;
}

Vector3 spacecraft::getVelocity() const
{
    return state_.I_Velocity;
}

Quaternion spacecraft::getOrientation() const
{
    return state_.IB_Orientation;
}

Vector3 spacecraft::getAngularVelocity() const
{
    return state_.B_AngularVelocity;
}

double spacecraft::getTotalMass()
{
    return state_.totalMass;
}

double spacecraft::getTotalFuelMass() const
{
    return thrustOrchestration.getFuelMassOfAllTanks();
}

std::vector<FuelTank> spacecraft::getFuelTanks() const
{
    return thrustOrchestration.getFuelTanks();
}

double spacecraft::getGload() const
{
    return GLoad;
}

std::string spacecraft::getConsoleTxt() const
{
    return consoleTxt;
}
