#pragma once

#include <QLabel>
#include <QObject>
#include <QSlider>

#include "flightcommandstruct.h"

class QKeyEvent;

/**
 * @file inputmapper.h
 * @brief Declares the inputmapper class used to map keyboard input to spacecraft control commands.
 */

/**
 * @class inputmapper
 * @brief Maps keyboard input to main engine slider interaction and RCS translation commands.
 *
 * This class is a Qt-based input adapter used by the cockpit UI.
 *
 * It supports two different input paths:
 * - Main engine control via keyboard by modifying the associated QSlider
 * - RCS translation control via discrete key press and release states
 *
 * For main engine control, the keyboard does not emit thrust commands directly.
 * Instead, it changes the value of the externally managed thrust slider.
 * This keeps the slider as the single source of truth for the main engine command.
 *
 * For RCS control, the class tracks pressed keys and emits a FlightCommand
 * containing the currently requested translational RCS input.
 *
 * Current keyboard mapping:
 * - Arrow Up / Arrow Down: increase / decrease main engine slider value
 * - D / A: +X / -X translation
 * - W / S: +Y / -Y translation
 * - E / Q: +Z / -Z translation
 *
 * This class is intended to be owned by the UI layer, e.g. cockpitPage.
 *
 * @note The receiving widget must have keyboard focus in order for key events
 *       to reach this mapper.
 */
class inputmapper : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief Constructs the input mapper.
     *
     * Creates a keyboard input helper for spacecraft control.
     *
     * The provided slider is used as the externally managed UI control for
     * main engine thrust. The label pointer may be used by the owning UI
     * layer for thrust display, but is not required for RCS command generation.
     *
     * @param mainEngineSlider Pointer to the main engine thrust slider.
     * @param label Pointer to the label associated with thrust display.
     * @param parent Optional Qt parent object.
     */
    explicit inputmapper(QSlider* mainEngineSlider, QLabel* label, QObject* parent = nullptr);

    /**
     * @brief Handles keyboard press events.
     *
     * Processes relevant user input and updates either:
     * - the main engine slider value, or
     * - the internal RCS translation state
     *
     * Main engine keys:
     * - Qt::Key_Up
     * - Qt::Key_Down
     *
     * RCS translation keys:
     * - Qt::Key_D / Qt::Key_A
     * - Qt::Key_W / Qt::Key_S
     * - Qt::Key_E / Qt::Key_Q
     *
     * @param event Pointer to the key press event.
     *
     * @note Auto-repeat key press events are ignored.
     */
    void handleKeyPress(QKeyEvent* event);

    /**
     * @brief Handles keyboard release events.
     *
     * Updates the internal pressed/released state of RCS control keys and emits
     * an updated translational command if required.
     *
     * Key releases for main engine slider input do not directly trigger thrust
     * changes, because the slider value itself is the authoritative state.
     *
     * @param event Pointer to the key release event.
     *
     * @note Auto-repeat key release events are ignored.
     */
    void handleKeyRelease(QKeyEvent* event);

signals:
    /**
     * @brief Emitted when a new main engine thrust target is requested.
     *
     * The emitted value corresponds to the currently commanded main engine thrust
     * in percent.
     *
     * @param value Main engine thrust command in percent.
     *
     * @note This signal is currently optional and may be unused if the owning UI
     *       layer handles slider changes directly.
     */
    void ME_thrustTargetRequested(double value);

    /**
     * @brief Emitted when a new translational RCS command is requested.
     *
     * The emitted FlightCommand contains the currently requested translational
     * control state derived from keyboard input.
     *
     * Only the RCS-relevant fields are expected to be meaningful when emitted
     * by this class.
     *
     * @param cmdSignal Flight command containing translational RCS input.
     */
    void RCS_cmdRequested(const FlightCommand& cmdSignal);

private:
    /**
     * @brief Updates the thrust command label text.
     *
     * Formats and writes the current commanded main engine thrust value to the
     * associated label.
     *
     * @param value Current thrust command in percent.
     *
     * @note This helper is only relevant if thrust label updates are handled
     *       inside this class.
     */
    void updateLabel(int value);

    /**
     * @brief Builds and emits the current translational RCS command.
     *
     * Evaluates the internal key state flags and constructs a FlightCommand
     * representing the currently requested translational RCS input.
     */
    void updateFlightCommand();

    /**
     * @brief Pointer to the main engine thrust slider.
     *
     * The slider is externally managed by the UI and serves as the main engine
     * command source.
     */
    QSlider* mE_Slider = nullptr;

    /**
     * @brief Pointer to the thrust command label.
     */
    QLabel* m_label = nullptr;

    /**
     * @brief Internal RCS translation state for positive X.
     */
    bool ENU_RCS_PosX = false;

    /**
     * @brief Internal RCS translation state for negative X.
     */
    bool ENU_RCS_NegX = false;

    /**
     * @brief Internal RCS translation state for positive Y.
     */
    bool ENU_RCS_PosY = false;

    /**
     * @brief Internal RCS translation state for negative Y.
     */
    bool ENU_RCS_NegY = false;

    /**
     * @brief Internal RCS translation state for positive Z.
     */
    bool ENU_RCS_PosZ = false;

    /**
     * @brief Internal RCS translation state for negative Z.
     */
    bool ENU_RCS_NegZ = false;
};
