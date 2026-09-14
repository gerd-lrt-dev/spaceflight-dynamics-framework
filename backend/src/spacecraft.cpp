#include "spacecraft.h"
#include "spacemath.h"
#include "Physics/basicMoonGravityModel.h"
#include "Physics/rigidBodyRotationalModel.h"
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

    // Mission reference frames are required for derived frame state
    initializeMissionFrames(0.0);

    // ---------------------------------------------------------
    // Authoritative runtime state
    // ---------------------------------------------------------

    state_.MCI_Position = spacecraftConfig_.MCI_initialPos;
    state_.MCI_Velocity = spacecraftConfig_.MCI_initialVelocity;

    if (spacecraftConfig_.initialStateFrame_ == InitialStateFrame::ENU)
    {
        CoordinateTransformer::State initialMCMF = coordTransf_.ENUtoMCMF(spacecraftConfig_.ENU_initialState, missionContext_.ENU_landingSite);

        CoordinateTransformer::State initialMCI = coordTransf_.MCMFtoMCI(initialMCMF, 0.0);

        state_.MCI_Position = initialMCI.position;
        state_.MCI_Velocity = initialMCI.velocity;
    }
    // ---------------------------------------------------------
    // Spacecraft body-frame origin
    // ---------------------------------------------------------

    originState_.origin.position    = state_.MCI_Position;
    originState_.origin.velocity    = state_.MCI_Velocity;
    originState_.orientation        = spacecraftConfig_.IB_initialRot;

    // Initialize derived frame representations from the resolved MCI state
    updateFrames(0.0);

    // ---------------------------------------------------------
    // Propulsion
    // ---------------------------------------------------------
    thrustOrchestration.initializeEngines(spacecraftConfig_.engines_, spacecraftConfig_.RCSengines_, spacecraftConfig_.tanks_);


    // ---------------------------------------------------------
    // Optimization
    // ---------------------------------------------------------

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
    Eigen::Vector3d MCI_total_Thrust = coordTransf_.GenSBFtoMCI(requestTotalThrust(), simFrameContext_.SBF_Frame);

    Eigen::Vector3d MCI_acceleration = physics_->computeAcc(MCI_getPosition(), MCI_getVelocity(), getTotalMass(), MCI_total_Thrust);

    // --- Compute velocity ---
    Eigen::Vector3d MCI_velocity = physics_->computeVel(MCI_getVelocity(), MCI_acceleration, dt);

    // --- Compute position ---
    Eigen::Vector3d MCI_position = physics_->computePos(MCI_getPosition(), MCI_velocity, MCI_acceleration, dt);

    // --- Compute orientation and angular velocity ---
    Eigen::Vector3d SBF_torque      = thrustOrchestration.getTotalTorque();
    Eigen::Vector3d SBF_angularAcc  = physics_->computeAngAcc(SBF_getAngularVelocity(), spacecraftConfig_.SBF_inertia, SBF_torque);
    Eigen::Vector3d SBF_angularVel  = physics_->computeAngVel(SBF_getAngularVelocity(), SBF_angularAcc, dt);
    Eigen::Quaterniond SBF_orientation = physics_->computeAttitude(IB_getOrientation(), SBF_angularVel, dt);

    /*
    std::cout << "\n========== ROTATIONAL DYNAMICS ==========\n"

              << "Angular velocity (old) [rad/s] : "
              << getAngularVelocity().transpose() << '\n'

              << "Torque                [N*m]    : "
              << SBF_torque.transpose() << '\n'

              << "Torque magnitude      [N*m]    : "
              << SBF_torque.norm() << '\n'

              << "Inertia tensor        [kg*m²]  :\n"
              << spacecraftConfig_.SBF_inertia << '\n'

              << "Angular acceleration  [rad/s²] : "
              << SBF_angularAcc.transpose() << '\n'

              << "Angular velocity (new)[rad/s]  : "
              << SBF_angularVel.transpose() << '\n'

              << "Attitude (old) [w x y z]       : "
              << getOrientation().w() << " "
              << getOrientation().x() << " "
              << getOrientation().y() << " "
              << getOrientation().z() << '\n'

              << "Attitude (new) [w x y z]       : "
              << SBF_orientation.w() << " "
              << SBF_orientation.x() << " "
              << SBF_orientation.y() << " "
              << SBF_orientation.z() << '\n'

              << "Quaternion norm                : "
              << SBF_orientation.norm() << '\n'

              << "=========================================\n";
    */
    // --- TODO: Update total mass ---
    // ...

    updateGLoad(MCI_acceleration, environmentConfig_.moonGravityVec);

    // --- Commit to state vector ---
    setVelocity(MCI_velocity);
    setPosition(MCI_position);
    setAngularVelocity(SBF_angularVel);
    setOrientation(SBF_orientation);
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
    simFrameContext_.MCMF_State = coordTransf_.MCItoMCMF(simFrameContext_.MCI_State, t);

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
        std::shared_ptr<IPhysicsModel> model_               = std::make_shared<BasicMoonGravityModel>(environmentConfig_);
        std::shared_ptr<IRotationalPhysicsModel> rotModel_  = std::make_shared<RigidBodyRotationalModel>(environmentConfig_);
        std::shared_ptr<IIntegrator> integrator_            = std::make_shared<EulerIntegrator>();
        std::shared_ptr<ISensor> sensor_                    = std::make_shared<SensorModel>(environmentConfig_);

        physics_ = std::make_unique<physics>(model_, rotModel_, integrator_, sensor_);

        setDefaultValues();
    };

spacecraft::~spacecraft()
{
}

void spacecraft::updateStep(double dt)
{
    // Update time systems are running
    time += dt;

    // Update mass data
    updateTotalMassOnFuelReduction(spacecraftConfig_.emptyMass, requestTotalFuelMass());

    // Update propulsion systems
    thrustOrchestration.updatePropulsion(dt);

    // Update Movement data (dynamics state) due to spacecraft state
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
        thrustOrchestration.shutDownAllEngines();

        updateMovementDataToZero(dt);
        break;
    }

    // --- Derive all frame representations from the thruth state---
    updateFrames(time);

    // Apply landing damage
    if (state_.MCI_Position.norm() <= environmentConfig_.radiusMoon)
    {
        applyLandingDamage(std::abs(simFrameContext_.ENU_State.velocity.z()));
    }

    // Update integrity
    updateSpacecraftIntegrity();
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
    if (MCI_getPosition().norm() <= environmentConfig_.radiusMoon)
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

void spacecraft::setTargetRCSThrust(const Eigen::Vector3d &targetThrustInPercentage, const EngineType &engineType)
{
    if (engineType == EngineType::RCS_translation)
    {
        (getSpacecraftState() == SpacecraftState::Operational) ? thrustOrchestration.setTargetThrustInPercentage(EngineType::RCS_translation, 0.0, targetThrustInPercentage) : thrustOrchestration.setTargetThrustInPercentage(EngineType::RCS_translation, 0.0, {0.0, 0.0, 0.0});
    }
    else if (engineType == EngineType::RCS_rotation)
    {
        (getSpacecraftState() == SpacecraftState::Operational) ? thrustOrchestration.setRCSRotationTargetThrustInPercentage(targetThrustInPercentage, spacecraftConfig_.centerOfMass) : thrustOrchestration.setRCSRotationTargetThrustInPercentage({0.0, 0.0, 0.0}, spacecraftConfig_.centerOfMass);
    }
    else
    {
        std::cerr << "Engine type <" << engineTypeToString(engineType) << "> does not match any configured engine Type!" << std::endl;
    }
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
    return thrustOrchestration.getCurrentThrustOfAllEngines();
}

Eigen::Vector3d spacecraft::requestMainEngineTargetThrust() const
{
    return thrustOrchestration.getTargetThrust(EngineType::MainEngine);
}

Eigen::Vector3d spacecraft::requestMainEngineThrust() const
{
    
    return thrustOrchestration.getCurrentThrustOfOneEngine(EngineType::MainEngine);
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
    return thrustOrchestration.getTotalFuelConsumption();
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

    // State

    simData_.time = time;

    simData_.statevector_ = getState();

    // Frames & Mission
    simData_.simFrameContext_ = simFrameContext_;
    simData_.missionContext_  = missionContext_;

    // Propulsion
    simData_.ME_ThrustState_.current            = requestMainEngineThrust().dot(requestMainEngineDirection());
    simData_.ME_ThrustState_.target             = requestMainEngineTargetThrust().dot(requestMainEngineDirection());
    simData_.ME_ThrustState_.targetPercentage   = requestMainEngineThrustInPercentage().dot(requestMainEngineDirection());
    simData_.ME_ThrustState_.SBF_direction      = requestMainEngineDirection();
    simData_.ME_ThrustState_.consumptionRate    = requestMainEngineLiveFuelConsumption();

    simData_.RCS_ThrustState_ = requestFullRCSEngineData();

    // Tanks
    simData_.tanks    = requestFuelTanks();
    simData_.fuelMass = requestTotalFuelMass();
    simData_.fuelFlow = requestMainEngineLiveFuelConsumption();

    // Sensors
    simData_.GLoad = getGload();

    // Integrity
    simData_.spacecraftIntegrity    = spacecraftIntegrity;
    simData_.spacecraftState_       = spacecraftState_;

    // Output
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

Eigen::Vector3d spacecraft::MCI_getPosition() const
{
    return state_.MCI_Position;
}

Eigen::Vector3d spacecraft::MCI_getVelocity() const
{
    return state_.MCI_Velocity;
}

Eigen::Quaterniond spacecraft::IB_getOrientation() const
{
    return state_.IB_Orientation;
}

Eigen::Vector3d spacecraft::SBF_getAngularVelocity() const
{
    return state_.SBF_AngularVelocity;
}

Eigen::Vector3d spacecraft::ENU_getPosition() const
{
    return simFrameContext_.ENU_State.position;
}

Eigen::Vector3d spacecraft::ENU_getVelocity() const
{
    return simFrameContext_.ENU_State.velocity;
}

const CoordinateTransformer::State& spacecraft::ENU_getState() const
{
    return simFrameContext_.ENU_State;
}

double spacecraft::getTotalMass()
{
    return state_.totalMass;
}

double spacecraft::requestTotalFuelMass() const
{
    return thrustOrchestration.getFuelMassOfAllTanks();
}

std::vector<FuelTank> spacecraft::requestFuelTanks() const
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
