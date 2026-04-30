/**
 * @file cockpitpage.h
 * @brief Cockpit / simulation view page of the Lunar Lander simulator.
 *
 * This file defines the cockpitPage class which represents the main
 * simulation cockpit UI. It contains LCD-style instruments for navigation,
 * engine data, fuel state, and spacecraft status.
 */

#ifndef COCKPITPAGE_H
#define COCKPITPAGE_H

#include <QGroupBox>
#include <QWidget>
#include <QLCDNumber>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSlider>
#include <QProgressBar>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QMap>
#include <QVector>

#include <vector3.h>
#include <spacecraftStateStruct.h>


#include "landingview.h"
#include "uibuilder.h"
#include "inputmapper.h"
#include "Thrust/FueltankStruct.h"

/**
 * @class cockpitPage
 * @brief Main simulation cockpit interface.
 *
 * The cockpitPage class represents the primary in-simulation user interface.
 * It visualizes real-time spacecraft telemetry such as altitude, velocity,
 * thrust, acceleration, fuel state, and hull integrity.
 *
 * The class itself does not perform any simulation logic. Instead, it exposes
 * a set of update functions which are intended to be called by a simulation
 * controller or worker thread via Qt signals and slots.
 *
 * Typical usage:
 * - Instantiate cockpitPage
 * - Add it to a QStackedWidget or main layout
 * - Connect simulation outputs to the update*() slots
 *
 * Info:
 * - This file was initially created by ChatGPT and then edited by gerd-lrt-dev.
 */
class cockpitPage : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief Constructs the cockpit page.
     *
     * Initializes all UI elements and layouts required for the cockpit view.
     *
     * @param parent Optional parent widget.
     */
    explicit cockpitPage(QWidget *parent = nullptr);

    // =====================================================
    // Public Update Interface (Simulation → UI)
    // =====================================================

    /**
     * @brief Updates the simulation time display.
     * @param t Simulation time in seconds.
     */
    void updateTime(double t);

    /**
     * @brief Updates the current position display.
     * @param position in all dimensions in meters.
     */
    void updatePosition(Vector3 position);

    /**
     * @brief Updates the rotational displays.
     * @param Vector with Latitude, Longitude and Roation in degree.
     */
    void updateRotation(Vector3 rotation);

    /**
     * @brief Updates the velocity displays.
     * @param Vector with velocity in meters per second.
     */
    void updateVelocity(Vector3 velocity);

    /**
     * @brief Updates the angular velocity displays.
     * @param Vector with Pitch, Roll and Yaw in degrees per second.
     */
    void updateAngularVelocity(Vector3 angularVelocity);

    /**
     * @brief Updates the current spacecraft acceleration.
     * @param accel Acceleration in meters per second squared.
     */
    void updateAcceleration(double accel);

    /**
     * @brief Updates the current engine thrust.
     * @param thrust Current thrust value (percentage or normalized).
     */
    void updateThrust(Vector3 thrust);

    /**
     * @brief Updates the target thrust setpoint.
     * @param target Target thrust value.
     */
    void updateTargetThrust(Vector3 target);

    /**
     * @brief Updates the remaining fuel mass.
     * @param fuel Remaining fuel mass in kilograms.
     */
    void updateFuelMass(double fuel);

    /**
     * @brief Updates the current fuel consumption rate.
     * @param flow Fuel consumption in kilograms per second.
     */
    void updateFuelFlow(double flow);

    /**
     * @brief Updates the hull integrity indicator.
     *
     * Displays visual feedback whether the spacecraft hull
     * is still intact or has been damaged.
     *
     * @param spacecraftState_ Current spacecraft state enum.
     */
    void updateHullStatus(SpacecraftState spacecraftState_);

    /**
     * @brief Updates the autopilot status indicator.
     *
     * Changes the visual label and/or indicator that shows
     * whether the autopilot is currently active.
     *
     * @param active True if autopilot is enabled, false otherwise.
     */
    void updateAutopilotStatus(bool active);

signals:
    /**
     * @brief Emitted when the user requests simulation start.
     */
    void startRequested();

    /**
     * @brief Emitted when the user requests simulation pause.
     */
    void pauseRequested();

    /**
     * @brief Emitted after the user confirmed a simulation stop/reset.
     */
    void stopConfirmed();

    /**
     * @brief Emitted when the user requests a full simulation reset.
     */
    void resetSimulationRequested();

    /**
     * @brief Emitted when the user changes the thrust slider.
     * @param percent Target thrust in percent.
     */
    void thrustTargetRequested(double percent);

    /**
     * @brief Emitted when the user provide flight command input
     * @param FlightCommand struct
     */
    void flightCmdRequested(FlightCommand cmd);

    /**
     * @brief Emitted when the autopilot toggle button is pressed.
     * @param acitve True if autopilot should be enabled.
     */
    void autopilotToggled(bool acitve);

public slots:
    /**
     * @brief Slot receiving updated simulation state.
     *
     * Called by the simulation backend or controller to update
     * all cockpit instruments and visualization elements.
     *
     * @param time Current simulation time in seconds.
     * @param pos Current spacecraft position vector.
     * @param vel Current spacecraft velocity vector.
     * @param GLoad Current acceleration magnitude.
     * @param spacecraftState_ Current spacecraft state enum.
     * @param thrust Current thrust in Newton.
     * @param targetThrust Target thrust in Newton.
     * @param fuelMass Remaining fuel mass in kilograms.
     * @param fuelFlow Current fuel flow in kilograms per second.
     */
    void onStateUpdated(double time,
                        const Vector3& pos,
                        const Vector3& vel,
                        const double& GLoad,
                        SpacecraftState spacecraftState_,
                        Vector3 thrust,
                        Vector3 targetThrust,
                        Vector3 thrustInPercentage,
                        QVector<FuelTank> tanks,
                        double fuelMass,
                        double fuelFlow,
                        QString consoleOutput);

private slots:
    /**
     * @brief Handles stop button click including confirmation dialog.
     */
    void onStopClicked(); ///< Combining stop button with prompt

    /**
     * @brief Handles autopilot toggle button click.
     *
     * Toggles internal autopilot state and emits corresponding signal.
     */
    void onAutopilotClicked();

    /**
     * @brief Slot for autopilot blink event
     */
    void onAutopilotBlinkTimeout();

    /**
     * @brief Display console Output
     * @param output
     */
    void consoleOutput(const QString &output);

private:
    // Members
    double lastTimeDisplay;     ///< Intermediate storage of time to calm the display down
    UIBuilder uibuilder;        ///< UI Building helper class
    inputmapper *m_inputMapper; ///< Keyboard and controller input class
    FlightCommand collectedCmd; ///< Collected flight command. This command will be send to worker thread

    // =====================================================
    // Internal Setup Functions
    // =====================================================

    /**
     * @brief Builds all cockpit UI elements and layouts.
     *
     * Creates LCD instruments, group boxes, and assigns
     * them to the cockpit layout.
     */
    void setupUI();

    /**
     * @brief Function for dynamic storage and pointer allocation
     * Declaration of objects can be found as private members of this class
     */
    void initializeQTObjects();

    /**
     * @brief Initialize all relevant object for keyboard and controller input
     */
    void initializeControlInput();

    /**
     * @brief Helper class for LCD Fields
     * @return QWidget with LCD field
     */
    QWidget *createLcdField(const QString& title, QLCDNumber* lcd, int digits);

    /**
     * @brief Builds navigation elements.
     * @return Navigation Box as QGroupBox.
     */
    QGroupBox *setupNavBox();

    /**
     * @brief Builds engine box elements.
     * @return Engine Box as QGroupBox.
     */
    QGroupBox *setupEngineBox();

    /**
     * @brief Builds fuel elements.
     * @return Fuel Box as QGroupBox.
     */
    QGroupBox *setupFuelBox();

    /**
     * @brief setupFuelDetailBox
     * @return
     */
    QWidget *setupFuelDetailBox();

    /**
     * @brief Rebuilds the dynamic fuel tank UI from a tank list.
     *
     * Creates one row per tank including:
     * - tank name
     * - tank role
     * - remaining mass LCD
     * - fill level progress bar
     *
     * Existing widgets are removed and recreated.
     *
     * @param tanks Current list of tanks.
     */
    void rebuildFuelTankPanel(const QVector<FuelTank>& tanks);

    /**
     * @brief Updates dynamic fuel tank values.
     *
     * Assumes that the UI was already built for the same tank count/order.
     *
     * @param tanks Current list of tanks.
     */
    void updateFuelTanks(const QVector<FuelTank>& tanks);

    /**
     * @brief Builds status elements.
     * @return Status Box as QGroupBox.
     */
    QGroupBox *setupStatusBox();

    /**
     * @brief Builds landing elements.
     * @return Landing Box as QGroupBox.
     */
    QGroupBox *setupLandingBox();

    /**
     * @brief Connects internal signals and slots.
     *
     * Reserved for future extensions such as local UI controls
     * or debug interactions.
     */
    void setupConnections();

    /**
     * @brief Handles key press events.
     *
     * This function is called by Qt when a key is pressed while this widget has focus.
     * The event is forwarded to the input mapper for processing (e.g., thrust control).
     *
     * @param event Pointer to the key press event.
     */
    void keyPressEvent(QKeyEvent* event);

        /**
     * @brief Handles key release events.
     *
     * This function is called by Qt when a key is released while this widget has focus.
     * The event is forwarded to the input mapper for processing.
     *
     * @param event Pointer to the key release event.
     */
    void keyReleaseEvent(QKeyEvent* event);

    /**
     * @brief Collect and send flight command to worker thread
     *
     */
    void sendFlightCmd();

    // =====================================================
    // Navigation Instruments
    // =====================================================

    QLCDNumber *lcdTime;            ///< Simulation time display [s]

    // Moon Centered Inertial (physical truth)
    QLCDNumber *MCI_lcdPosX;        ///< Position in x [m]
    QLCDNumber *MCI_lcdPosY;        ///< Position in y [m]
    QLCDNumber *MCI_lcdPosZ;        ///< Position in z [m]

    // Local Navigation Frame
    QLCDNumber *LNF_lcdLat;         ///< Latitude [°]
    QLCDNumber *LNF_lcdLon;         ///< Longitude [°]
    QLCDNumber *LNF_lcdRot;         ///< Rotation [°]

    QLCDNumber *LNF_lcdVelX;        ///< Velocity in x -> North [m/s]
    QLCDNumber *LNF_lcdVelY;        ///< Velocity in y -> East [m/s]
    QLCDNumber *LNF_lcdVelZ;        ///< Velocity in z -> Up [m/s]

    QLCDNumber *LNF_lcdRoll;        ///< Roll [°/s]
    QLCDNumber *LNF_lcdPitch;       ///< Pitch [°/s]
    QLCDNumber *LNF_lcdYaw;         ///< Yaw [°/s]

    QLCDNumber *LNF_totalVel;       ///< Total velocity [m/s]

    // =====================================================
    // Engine Instruments
    // =====================================================

    QLCDNumber *LNF_lcdThrust_BX;       ///< Current engine thrust in body frame of spacecraft x direction
    QLCDNumber *LNF_lcdThrust_BY;       ///< Current engine thrust in body frame of spacecraft y direction
    QLCDNumber *LNF_lcdThrust_BZ;       ///< Current engine thrust in body frame of spacecraft z direction
    QLCDNumber *LNF_lcdTargetThrust_BX; ///< Target thrust setpoint in body frame of spacecraft x direction
    QLCDNumber *LNF_lcdTargetThrust_BY; ///< Target thrust setpoint in body frame of spacecraft y direction
    QLCDNumber *LNF_lcdTargetThrust_BZ; ///< Target thrust setpoint in body frame of spacecraft z direction
    QLCDNumber *lcdGLoad;               ///< Current acceleration [m/s²]

    // =====================================================
    // Fuel Instruments
    // =====================================================

    QLCDNumber *lcdFuelMass; ///< Remaining fuel mass [kg]
    QLCDNumber *lcdFuelFlow; ///< Fuel consumption rate [kg/s]

    // =====================================================
    // Fuel Tank UI
    // =====================================================

    QWidget *fuelTankContainer = nullptr;          ///< Container widget for dynamic tank list
    QVBoxLayout *fuelTankLayout = nullptr;         ///< Layout for dynamic tank rows

    QVector<QLCDNumber*> lcdTankMasses;            ///< One LCD per tank
    QVector<QProgressBar*> barTankFillLevels;      ///< One fill bar per tank
    QVector<QLabel*> lblTankNames;                 ///< One label per tank name
    QVector<QLabel*> lblTankRoles;                 ///< One label per tank role

    // =====================================================
    // Status Indicators
    // =====================================================

    QLabel* lblHullStatus;        ///< Displays the hull integrity status
    QLabel* lblControllerOutput;  ///< Displays the controller or console output in the cockpit UI


    // =====================================================
    // Visualization
    // =====================================================

    LandingView *landingView; ///< 2D landing visualization widget

    // =====================================================
    // Buttons
    // =====================================================

    QPushButton *btnSimStart; ///< Simulation start button
    QPushButton *btnSimPause; ///< Simulation pause button
    QPushButton *btnSimStop;  ///< Simulation stop button

    // =====================================================
    // Thrust Controle Console
    // =====================================================

    QSlider* thrustSlider = nullptr; ///< Thrust control slider
    QLabel* lblThrustCmd  = nullptr; ///< Displays commanded thrust percentage

    // =====================================================
    // Autopilot
    // =====================================================

    QPushButton* btnAutopilot; ///< Autopilot toggle button
    QTimer* autopilotBlinkTimer;  ///< Timer used to blink the autopilot status indicator
    QLabel* lblAutopilotStatus;   ///< Displays the current autopilot state
    bool autopilotActive = false; ///< Internal autopilot state flag
    bool autopilotBlinkOn;        ///< Internal flag to track the blink state of the autopilot indicator
};

#endif // COCKPITPAGE_H
