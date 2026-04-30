/**
 * @file mainWindow.h
 * @brief Main GUI window for the Lunar Lander simulator application.
 *
 * The MainWindow class manages the central GUI of the application.
 * It provides navigation between different pages of the simulator using
 * a QStackedWidget. The main pages include:
 * - Homepage
 * - Cockpit / Simulation view
 * - Settings and configuration (if applicable)
 *
 * The class also handles screen detection and layout adjustments to
 * ensure proper scaling on different monitors.
 *
 * @author Schendzielorz Gerd
 * @date 2025-11-27
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QLabel>
#include <QMainWindow>
#include <QPixmap>
#include <QPushButton>
#include <QRect>
#include <QScreen>
#include <QShowEvent>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QWindow>

#include "homepage.h"

/**
 * @class MainWindow
 * @brief Main application window for the Lunar Lander simulator.
 *
 * Inherits from QMainWindow. This class is responsible for:
 * - Creating and managing the central GUI layout
 * - Handling navigation between different pages via QStackedWidget
 * - Detecting screen size and adjusting layouts accordingly
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief Constructor for MainWindow.
     * @param parent Optional parent widget. Defaults to nullptr.
     *
     * Initializes the main window and prepares the central GUI layout.
     */
    MainWindow(QWidget *parent = nullptr);

    /**
     * @brief Destructor for MainWindow.
     *
     * Cleans up allocated resources.
     */
    ~MainWindow();

private:
    // ==========================================
    // Member Variables
    // ==========================================
    QVBoxLayout *vLayout;   ///< Main vertical layout for the central widget
    Homepage    *homepage;  ///< First page of program. Used to select initial steps.

    // ==========================================
    // Member Functions
    // ==========================================
    void setupUI();          ///< Sets up the main window UI components

protected:
    /**
     * @brief Handles the show event to detect the current screen and adjust window size.
     *
     * Reimplementation of the QWidget/QMainWindow showEvent, automatically
     * called by Qt when the window becomes visible (after show() is invoked).
     * It retrieves the screen on which the window is currently displayed and
     * its available geometry to scale UI elements appropriately.
     *
     * Note: windowHandle() must be valid when accessing the screen. This method
     * is preferred over accessing the primary screen directly, especially
     * in multi-monitor setups.
     *
     * @param event Pointer to the QShowEvent triggered when the widget is shown.
     */
    void showEvent(QShowEvent *event) override;
};

#endif // MAINWINDOW_H
