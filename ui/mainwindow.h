/**
 * @file mainwindow.h
 * @brief Main GUI window for the Lunar Lander simulator application.
 *
 * MainWindow is the central frontend shell of the application.
 * It owns the global navigation structure, the page stack, the shared
 * configuration manager and the simulation worker thread.
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QRect>
#include <QScreen>
#include <QShowEvent>
#include <QStackedWidget>
#include <QThread>
#include <QVBoxLayout>
#include <QWidget>
#include <QWindow>

#include "configmanager.h"
#include "cockpitpage.h"
#include "homepage.h"
#include "simulationworker.h"
#include "spacecraftselectionpage.h"
#include "topbarwidget.h"
#include "controlshelppage.h"
#include "settingspage.h"

/**
 * @class MainWindow
 * @brief Main application shell and frontend coordinator.
 *
 * Responsibilities:
 * - Owns the persistent TopBarWidget
 * - Owns the central QStackedWidget
 * - Owns all main frontend pages
 * - Owns the shared ConfigManager
 * - Owns and manages the simulation worker thread
 * - Connects UI navigation signals to page transitions
 * - Connects cockpit input signals to the simulation worker
 * - Connects simulation telemetry updates back to the cockpit
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief Constructs the main window.
     * @param parent Optional parent widget.
     */
    explicit MainWindow(QWidget *parent = nullptr);

    /**
     * @brief Destructor.
     *
     * Stops and joins the simulation worker thread.
     */
    ~MainWindow();

private:
    // ==========================================
    // Resources
    // ==========================================

    /**
     * @brief Shared spacecraft configuration manager.
     */
    ConfigManager configManager_;

    /**
     * @brief Currently selected spacecraft JSON configuration.
     *
     * Empty means that the default spacecraft configuration should be used.
     */
    QString selectedJsonConfigStr;

    // ==========================================
    // Widgets & Layouts
    // ==========================================

    /**
     * @brief Main vertical layout containing top bar and page stack.
     */
    QVBoxLayout *vLayout = nullptr;

    /**
     * @brief Persistent top navigation bar.
     */
    TopBarWidget *topBar = nullptr;

    /**
     * @brief Central page stack.
     */
    QStackedWidget *stackedWidget = nullptr;

    // ==========================================
    // Pages
    // ==========================================

    /**
     * @brief Application landing page.
     */
    Homepage *homepage = nullptr;

    /**
     * @brief Spacecraft selection page.
     */
    SpacecraftSelectionPage *selectionPage = nullptr;

    /**
     * @brief Main simulation cockpit page.
     */
    cockpitPage *cockpit = nullptr;

    /**
     * @brief Controls Help Page
     */
    ControlsHelpPage *controlsHelpPage = nullptr;

    /**
     * @brief Settings Page
     */
    SettingsPage *settingsPage = nullptr;

    // ==========================================
    // Simulation Thread / Worker
    // ==========================================

    /**
     * @brief Thread used to run the simulation worker.
     */
    QThread *simulationThread = nullptr;

    /**
     * @brief Worker object executing the simulation loop.
     */
    SimulationWorker *simulationWorker = nullptr;

    // ==========================================
    // Setup Functions
    // ==========================================

    /**
     * @brief Builds the main UI shell and creates all pages.
     */
    void setupUI();

    /**
     * @brief Creates and starts the simulation worker thread.
     */
    void setupThread();

    /**
     * @brief Connects navigation, UI and simulation signals.
     */
    void setupConnections();

    /**
     * @brief Sends the selected or default spacecraft JSON to the worker.
     */
    void sendCurrentConfigToWorker();

protected:
    /**
     * @brief Handles initial window sizing after the window is shown.
     * @param event Qt show event.
     */
    void showEvent(QShowEvent *event) override;
};

#endif // MAINWINDOW_H
