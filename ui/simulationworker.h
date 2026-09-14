/**
 * @file simulationworker.h
 * @brief Worker object executing the lunar lander simulation in a separate thread.
 *
 * The SimulationWorker owns the simulation backend and performs
 * time-stepped simulation updates independently of the UI thread.
 */

#ifndef SIMULATIONWORKER_H
#define SIMULATIONWORKER_H

#include <QObject>
#include <QTimer>
#include <QDebug>
#include <QMutexLocker>
#include <QVector>
#include <Eigen/Dense>
#include <QVector>

#include "FlightCommandDTO.h"
#include "TelemetryMapper.h"

/**
 * @class SimulationWorker
 * @brief Executes the simulation loop in a worker thread.
 *
 * This class encapsulates the simulation backend and periodically
 * advances the simulation state. Updated state data is emitted via
 * Qt signals and consumed by the UI.
 */
class SimulationWorker : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief SimulationWorker construct the simulation worker.
     * @param parent Optional QObject parent
     */
    explicit SimulationWorker(QObject *parent = nullptr);

public slots:
    /**
     * @brief Starts the simulation loop.
     */
    void start();

    /**
     * @brief Pause the simulation loop.
     */
    void pause();

    /**
     * @brief Stops the simulation loop.
     */
    void stop();

    /**
     * @brief Sets the desired thrust lvl
     * @param FlightCommand Struct with translational and rotational commands
     */
    void setFlightCommand(FlightCommandDTO cmd);

    /**
     * @brief Receives QString with json config data
     */
    void receiveJsonConfig(const QString &json);

    /**
     * @brief Sets autopilot to activated
     * @param Boolean active
     */
    void setAutopilotFlag(bool active);

signals:
    /**
     * @brief Emitted after each completed simulation step.
     *
     * Provides the complete frontend telemetry snapshot generated from the
     * current backend simulation state.
     *
     * The telemetry object contains all data required by the user interface,
     * including simulation time, navigation state, reference-frame data,
     * mission context, spacecraft integrity, propulsion telemetry, sensor data
     * and backend console output.
     *
     * The signal is emitted once the backend state has been advanced by one
     * discrete simulation timestep and the corresponding telemetry snapshot
     * has been generated.
     *
     * @param telemetry_ Complete telemetry snapshot representing the current
     *                   simulation state.
     */
    void stateUpdated(Telemetry telemetry_);

    /**
     * @brief simulationError
     * @param errorMsg
     */
    void simulationError(QString errorMsg);

public slots:
    /**
     * @brief Adavnces the simulation by one timestep.
     */
    void stepSimulation();

private:
     // ==========================
     // Internal State
     // ==========================
    TelemetryMapper telemetryMapper_;
    Telemetry telemetry_;
    FlightCommandDTO collectedCmdData;

    QVector<Telemetry> telemetryHistory_;   ///< History of full telemetry data

    std::string jsonConfig;     ///< String with spacecraft config data
    QTimer *simulationTimer;    ///< Drives simulation ticks
    bool running = false;       ///< Simulation running flag
    bool initialized = false;   ///< Config initializer flag

    QMutex mutex;               ///< Thread safety
    double requestedThrustPercent = 0.0; ///< Desired thrust in percentage

    // ==========================
    // Private Methods
    // ==========================

    /**
     * @brief Collects a user-issued control command from the frontend.
     *
     * This function stores the thrust command values received from the user.
     * Both parameters are optional and default to zero.
     *
     * @param FlightCommand Struct with translational and rotational input
     * @param thrustInPercentage Commanded thrust as a percentage of maximum thrust.
     * @param thrustInNewton     Commanded thrust in Newtons.
     */
    void collectControlCommands(const FlightCommandDTO &cmd, const double &thrustInPercentage = 0.0, const double &thrustInNewton = 0.0);

    /**
     * @brief Collects an autopilot command from the automation system.
     *
     * This function updates the internal flag indicating whether the autopilot
     * should be active. Only the automation system should call this.
     *
     * @param autopilotActive True if the autopilot is engaged, false otherwise.
     */
    void collectAutopilotCommand(bool autopilotActive);

    /**
     * @brief Sends the currently collected control commands to the backend.
     *
     * This function forwards the stored user and/or autopilot commands to the
     * arbiter, which decides which commands are active. Ensures frontend cannot
     * directly manipulate automation commands.
     */
    void sendControlCommands();



};

#endif // SIMULATIONWORKER_H
