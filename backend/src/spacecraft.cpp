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
    totalMass = spacecraftConfig_.emptyMass + spacecraftConfig_.fuelM;
    state_.MCI_Position = spacecraftConfig_.MCI_initialPos;
    state_.MCI_Velocity = spacecraftConfig_.MCI_initialVelocity;

    originState_.origin.position    = spacecraftConfig_.MCI_initialPos;
    originState_.origin.velocity    = spacecraftConfig_.MCI_initialVelocity;
    originState_.orientation        = spacecraftConfig_.IB_initialRot;

    initializeMissionFrames(0.0);

    thrustOrchestration.initializeEngines(spacecraftConfig_.engines_, spacecraftConfig_.RCSengines_, spacecraftConfig_.tanks_);


    // TODO just testing here optimization

    //double h0 = landerMoon.I_initialPos.z;      // Höhe über Oberfläche
    //double v0 = landerMoon.I_initialVelocity.z; // vertikale Geschwindigkeit
    //double m0 = totalMass;

    //std::vector<double> thrust = compute_optimization(h0, v0, m0, 0.5);
}

void spacecraft::initializeMissionFrames(double t0)
{
    missionContext_.MCMF_landingSite    = coordTransf_.MSCtoMCMF(missionContext_.MSC_LandingSite);

    missionContext_.MCI_landingSite     = coordTransf_.MCMFtoMCI(missionContext_.MCMF_landingSite, t0);

    missionContext_.ENU_landingSite     = coordTransf_.computeENUFrame(missionContext_.MCMF_landingSite);
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
    Eigen::Vector3d MCI_total_Thrust = coordTransf_.GenSBFtoMCI(requestTotalThrust(), originState_);

    Eigen::Vector3d MCI_acceleration = physics_->computeAcc(getPosition(), getVelocity(), getTotalMass(), MCI_total_Thrust);

    // --- Compute velocity ---
    Eigen::Vector3d MCI_velocity = physics_->computeVel(getVelocity(), MCI_acceleration, dt);

    // --- Compute position ---
    Eigen::Vector3d MCI_position = physics_->computePos(getPosition(), MCI_velocity, MCI_acceleration, dt);

    // --- Update Frames ---
    updateFrames(time);

    // --- TODO: Compute orientation and angular velocity ---
    // ...

    // --- TODO: Update total mass ---
    // ...

    updateGLoad(MCI_acceleration, environmentConfig_.moonGravityVec);

    // --- Commit to state vector ---
    setVelocity(MCI_velocity);
    setPosition(MCI_position);
    //setGload(GLoad);
}

void spacecraft::updateMovementDataToZero(double dt)
{
    Eigen::Vector3d zeroVector = {0.0, 0.0, 0.0};
    // --- Commit to state vector ---
    setVelocity(zeroVector);
    updateGLoad(zeroVector, environmentConfig_.moonGravityVec);
}

void spacecraft::updateFrames(double t)
{
    // --- MCI  ---
    simFrameContext_.MCI_State.position = state_.MCI_Position;
    simFrameContext_.MCI_State.velocity = state_.MCI_Velocity;

    // --- MCI to MCMF ---
    simFrameContext_.MCMF_State = coordTransf_.MCItoMCMF(simFrameContext_.MCI_State, dt);

    // --- Compute ENU ---
    simFrameContext_.ENU_Frame  = missionContext_.ENU_landingSite;

    // --- Compute LVLH ---
    simFrameContext_.LVLH_Frame = coordTransf_.computeLVLHFrame(simFrameContext_.MCI_State);

    // --- MCMF to MSC
    simFrameContext_.MSC_State  = coordTransf_.MCMFtoMSC(simFrameContext_.MCMF_State);

    // --- MCMF to ENU
    simFrameContext_.ENU_State  = coordTransf_.MCMFtoENU(simFrameContext_.MCMF_State, simFrameContext_.ENU_Frame);

    // --- MCI to LVLH
    simFrameContext_.LVLH_State = coordTransf_.MCItoLVLH(simFrameContext_.MCI_State, simFrameContext_.LVLH_Frame);

    // --- SBF ---
    simFrameContext_.SBF_Frame.orientation      = state_.IB_Orientation;

    simFrameContext_.SBF_Frame.origin.position  = state_.MCI_Position;

    simFrameContext_.SBF_Frame.origin.velocity  = state_.MCI_Velocity;
}

void spacecraft::updateGLoad(const Eigen::Vector3d& totalAcceleration, const Eigen::Vector3d& gravityAcceleration)
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

    KEref   = spacemath::kineticEnergy(totalMass, spacecraftConfig_.safeVelocity);
    KE      = spacemath::kineticEnergy(totalMass, impactVelocity);

    damageInPercent = KE / KEref;

    spacecraftIntegrity += -damageInPercent;
}

void spacecraft::setPosition(const Eigen::Vector3d& pos)
{
    state_.MCI_Position = pos;
}

void spacecraft::setVelocity(const Eigen::Vector3d& vel)
{
    state_.MCI_Velocity = vel;
}

void spacecraft::setOrientation(const Eigen::Quaterniond& q)
{
    state_.IB_Orientation = q;
}

void spacecraft::setAngularVelocity(const Eigen::Vector3d& angVel)
{
    state_.SBF_AngularVelocity = angVel;
}

// -------------------------------------------------------------------------
// Public
// -------------------------------------------------------------------------
spacecraft::spacecraft(customSpacecraft lMoon, MissionContext mContext) : spacecraftConfig_(lMoon), missionContext_(mContext)
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
    updateTotalMassOnFuelReduction(spacecraftConfig_.emptyMass, getTotalFuelMass());

    thrustOrchestration.updateThrust(dt);

    // Update time systems are running
    time += dt;

    // Apply landing damage
    if (state_.MCI_Position.z <= environmentConfig_.radiusMoon)
    {
        applyLandingDamage(state_.MCI_Velocity.z);
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
    if (spacecraftIntegrity < spacecraftConfig_.structuralIntegrity)
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

void spacecraft::setTargetRCSThrust(const Eigen::Vector3d &targetThrustInPercentage)
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
    problem.m_dry = spacecraftConfig_.emptyMass;

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

Eigen::Vector3d spacecraft::requestTotalThrust() const
{
    return thrustOrchestration.getCurrentThrust();
}

Eigen::Vector3d spacecraft::requestMainEngineTargetThrust() const
{
    
    return thrustOrchestration.getTargetThrust(EngineType::MainEngine);
}

Eigen::Vector3d spacecraft::requestMainEngineThrust() const
{
    
    return thrustOrchestration.getCurrentThrust(EngineType::MainEngine);
}

Eigen::Vector3d spacecraft::requestMainEngineThrustInPercentage() const
{
    return thrustOrchestration.getCurrentThrustInPercentage(EngineType::MainEngine);
}

Eigen::Vector3d spacecraft::requestMainEngineDirection() const
{
    return thrustOrchestration.getDirectionOfThrust(EngineType::MainEngine);
}

double spacecraft::requestMainEngineLiveFuelConsumption() const
{
    return thrustOrchestration.getFuelConsumption(EngineType::MainEngine);
}

std::vector<RCS_ThrustState> spacecraft::requestFullRCSEngineData() const
{
    return thrustOrchestration.getFullRCSEngineData();
}

void spacecraft::setInitalPosition(const Eigen::Vector3d& position)
{
    spacecraftConfig_.MCI_initialPos = position;
}

void spacecraft::setInitalVelocity(const Eigen::Vector3d& velocity)
{
    spacecraftConfig_.MCI_initialVelocity = velocity;
}

simData spacecraft::getFullSimulationData() const
{
    simData simData_;

    simData_.statevector_ = getState();

    // Reduce height by radius of moon
    simData_.statevector_.MCI_Position.z -= environmentConfig_.radiusMoon;

    // Fill struct with data for emitting signal to UI
    simData_.spacecraftState_ = spacecraftState_;

    simData_.ME_ThrustState_.current            = requestMainEngineThrust().dot(requestMainEngineDirection());
    simData_.ME_ThrustState_.target             = requestMainEngineTargetThrust().dot(requestMainEngineDirection());
    simData_.ME_ThrustState_.targetPercentage   = requestMainEngineThrustInPercentage().dot(requestMainEngineDirection());
    simData_.ME_ThrustState_.SBF_direction          = requestMainEngineDirection();

    simData_.RCS_ThrustState_ = requestFullRCSEngineData();

    simData_.tanks    = getFuelTanks(); //TODO: Should be a request
    simData_.fuelMass = getTotalFuelMass(); //TODO: Should be a request
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

Eigen::Vector3d spacecraft::getPosition() const
{
    return state_.MCI_Position;
}

Eigen::Vector3d spacecraft::getVelocity() const
{
    return state_.MCI_Velocity;
}

Eigen::Quaterniond spacecraft::getOrientation() const
{
    return state_.IB_Orientation;
}

Eigen::Vector3d spacecraft::getAngularVelocity() const
{
    return state_.SBF_AngularVelocity;
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
