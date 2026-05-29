/**
 * @file homepage.h
 * @brief Landing page of the Lunar Lander simulator frontend.
 *
 * The Homepage class represents the initial user-facing start page
 * of the simulation software. It provides navigation entry points
 * to the spacecraft selection page and the simulation cockpit.
 *
 * This class is intentionally lightweight and UI-focused.
 * It does not own application-wide resources such as:
 * - simulation workers
 * - configuration managers
 * - page stacks
 * - simulation threads
 *
 * Those responsibilities are handled by MainWindow.
 */

#ifndef HOMEPAGE_H
#define HOMEPAGE_H

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

/**
 * @class Homepage
 * @brief Initial landing page of the application.
 *
 * The Homepage widget displays the simulator title,
 * project branding and the primary navigation buttons.
 *
 * Responsibilities:
 * - Display landing UI
 * - Emit navigation requests
 *
 * Non-responsibilities:
 * - Simulation control
 * - Thread management
 * - Page ownership
 * - Spacecraft configuration handling
 */
class Homepage : public QWidget
{
    Q_OBJECT

public:

    /**
     * @brief Constructs the homepage widget.
     *
     * Initializes all visual UI elements and connects
     * button interactions to outgoing navigation signals.
     *
     * @param parent Optional parent widget.
     */
    explicit Homepage(QWidget *parent = nullptr);

    /**
     * @brief Default destructor.
     */
    ~Homepage() = default;

signals:

    /**
     * @brief Emitted when the user requests spacecraft selection.
     *
     * Typically connected to page navigation logic in MainWindow.
     */
    void selectSpacecraftRequested();

    /**
     * @brief Emitted when the user requests simulation start.
     *
     * Typically connected to cockpit navigation and simulation
     * startup logic in MainWindow.
     */
    void startSimulationRequested();

private:

    // ==========================================
    // Member Variables
    // ==========================================

    /**
     * @brief Main vertical layout of the homepage.
     */
    QVBoxLayout *vHomepageLayout = nullptr;

    /**
     * @brief Button opening the spacecraft selection page.
     */
    QPushButton *selectSpacecraftButton = nullptr;

    /**
     * @brief Button requesting simulation startup.
     */
    QPushButton *startSimulationButton = nullptr;

    // ==========================================
    // Internal Setup Functions
    // ==========================================

    /**
     * @brief Builds the homepage UI layout.
     *
     * Creates and arranges:
     * - title
     * - logo
     * - copyright
     * - navigation buttons
     */
    void setupUI();

    /**
     * @brief Connects button interactions to outgoing signals.
     */
    void setupConnections();
};

#endif // HOMEPAGE_H
