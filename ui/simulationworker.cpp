#include "simulationworker.h"

#include <QString>

SimulationWorker::SimulationWorker(QObject *parent)
    : QObject(parent)
{
    // Build timer event
    simulationTimer = new QTimer(this);
    simulationTimer->setInterval(50); // 20 Hz

    // connect timer with worker function
    connect(simulationTimer, &QTimer::timeout, this, &SimulationWorker::stepSimulation);
}

void SimulationWorker::start()
{
    try {
        controller = std::make_unique<simcontrol>(0);
        controller->initialize(jsonConfig);
    }
    catch (const std::exception& e)
    {
        qCritical() << "Simulation start failed: " << e.what();
        emit simulationError(QString::fromStdString(e.what()));
    }

    running = true;
    simulationTimer->start();

    qDebug("[simulationworker]-start-: Simulation started successfully");
}

void SimulationWorker::pause()
{
    running = false;
    simulationTimer->stop();
}

void SimulationWorker::stop()
{
    running = false;
    simulationTimer->stop();
    currentTime = 0.0;

    emit stateUpdated(currentTime,
                      {0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0},
                      0.0,
                      SpacecraftState::Operational,
                      {0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0},
                      {0.0, 0.0, 0.0},
                      QVector<RCSCockpitTelemetry>{},
                      QVector<FuelTank>{},
                      0.0,
                      0.0,
                      "");

    controller->setResetBoolean();
}

void SimulationWorker::receiveJsonConfig(const QString &json)
{
    this->jsonConfig = json.toStdString();

    qDebug() << "SimulationWorker received JSON config, size:"
             << jsonConfig.size();
}

void SimulationWorker::setFlightCommand(FlightCommand cmd)
{
    QMutexLocker locker(&mutex);

    collectControlCommands(cmd);
}

void SimulationWorker::setAutopilotFlag(bool active)
{
    QMutexLocker locker(&mutex); // protect access
    collectAutopilotCommand(active);
}

void SimulationWorker::stepSimulation()
{
    double dt = 0.00;   ///< Initialized discrete timestep

    // Return if not running
    if(!running)
        return;

    // times
    // TODO: Should be change by timer event
    dt = 0.05; // TODO: should specified in json as well
    currentTime += dt;

    // Calling backend simulator --> Will be refactored with Issue 19 Frontend should no know data structs from backend!
    spacecraftData = controller->runSimulation(dt);

    // Withdraw user input due to thrust
    sendControlCommands();

    // Change data type for console output
    QString consoleOutput = QString::fromStdString(spacecraftData.output);

    // Change vector type for fuel tanks information
    QVector<FuelTank> fuelTanksQVec(spacecraftData.tanks.begin(), spacecraftData.tanks.end());

    QVector<RCSCockpitTelemetry> RCSTelemetryVec;

    // TODO: This wrapper is going to be transfered with Issue 19 into own Translater class
    for (const auto  &RCS_State_: spacecraftData.RCS_ThrustState_)
    {
        RCSCockpitTelemetry RCSTelemetry_;
        RCSTelemetry_.engineID = RCS_State_.engineID;
        RCSTelemetry_.name = QString::fromStdString(RCS_State_.engineName);
        RCSTelemetry_.axis = QString::fromStdString(RCS_State_.axis);
        RCSTelemetry_.currentThrust = RCS_State_.currentThrust;
        RCSTelemetry_.targetThrust = RCS_State_.targetThrust;
        RCSTelemetry_.actuatorState = RCS_State_.targetThrustPercentage;

        RCSTelemetryVec.push_back(RCSTelemetry_);
    }

    // signals
    emit stateUpdated(currentTime,
                      spacecraftData.statevector_.MCI_Position,
                      spacecraftData.statevector_.MCI_Velocity,
                      spacecraftData.GLoad,
                      spacecraftData.spacecraftState_,
                      spacecraftData.ME_ThrustState_.direction * spacecraftData.ME_ThrustState_.current,
                      spacecraftData.ME_ThrustState_.direction * spacecraftData.ME_ThrustState_.target,
                      spacecraftData.ME_ThrustState_.direction * spacecraftData.ME_ThrustState_.targetPercentage,
                      RCSTelemetryVec,
                      fuelTanksQVec,
                      spacecraftData.fuelMass,
                      spacecraftData.fuelFlow,
                      consoleOutput
                      );
}

void SimulationWorker::collectControlCommands(const FlightCommand &cmd, const double &thrustInPercentage, const double &thrustInNewton)
{
    //TODO: Do not call back end structures. Change that with Issue 19
    FEControlCommands_.mainEngine       = cmd.mainEngine;
    FEControlCommands_.translation.x    = cmd.translation.x;
    FEControlCommands_.translation.y    = cmd.translation.y;
    FEControlCommands_.translation.z    = cmd.translation.z;

    FEControlCommands_.rotation.x       = cmd.rotation.x;
    FEControlCommands_.rotation.y       = cmd.rotation.y;
    FEControlCommands_.rotation.z       = cmd.rotation.z;
}

void SimulationWorker::collectAutopilotCommand(bool autopilotActive)
{
    FEControlCommands_.autopilotActive      = autopilotActive;
}

void SimulationWorker::sendControlCommands()
{
    // Call Back End function
    // TODO: Change that with Issue 19
    controller->receiveCommandFromFrontEnd(FEControlCommands_);
}


