/**
 * @file landingview.h
 * @brief 2.5D landing visualization widget for lunar landing simulation.
 *
 * This widget provides a compact situational awareness view of the spacecraft
 * during descent and landing in a local ENU (East-North-Up) frame.
 *
 * The visualization is intentionally designed as a lightweight alternative
 * to full 3D rendering, focusing on interpretability and controller analysis.
 *
 * The widget is split into three main regions:
 *
 * - Side View (E-UP): Shows vertical motion and lateral offset
 * - Top View (E-N):   Shows horizontal drift and target relation
 * - State Panel:      Displays key numerical telemetry values
 *
 * The widget is passive and only visualizes values provided
 * through update functions.
 *
 * Coordinate Convention:
 * - X → East
 * - Y → North
 * - Z → Up
 *
 * Info:
 * - Quaternion-based attitude is expected to be converted externally
 *   (e.g. to yaw angle) before passing to this widget.
 * - This widget does NOT perform simulation or control logic.
 *
 * @author
 * Generated with ChatGPT, extended by gerd-lrt-dev
 */

#ifndef LANDINGVIEW_H
#define LANDINGVIEW_H

#include <QWidget>
#include <QTimer>
#include <QVector>
#include <QPointF>

#include <spacecraftStateStruct.h>
#include <vector3.h>

/**
 * @class LandingView
 * @brief 2.5D landing visualization widget.
 *
 * LandingView renders a simplified spatial representation of the spacecraft
 * in a local ENU coordinate system. It combines multiple views to provide
 * an intuitive understanding of vehicle motion without requiring full 3D rendering.
 *
 * Displayed information includes:
 * - Position relative to landing target
 * - Velocity vectors (vertical and lateral)
 * - Trajectory history (top view)
 * - Thrust indication
 * - Yaw orientation (top view)
 * - Hull integrity status
 *
 * The widget is optimized for:
 * - Controller tuning
 * - Stability analysis
 * - Landing behavior interpretation
 */
class LandingView : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief Constructs a LandingView widget.
     *
     * Initializes rendering settings and animation timer.
     *
     * @param parent Optional parent widget.
     */
    explicit LandingView(QWidget *parent = nullptr);

    // =====================================================
    // Public Update Interface (Simulation → UI)
    // =====================================================

    /**
     * @brief Updates spacecraft position in ENU frame.
     *
     * @param pos Position vector:
     * - x → East [m]
     * - y → North [m]
     * - z → Up (altitude) [m]
     */
    void setPositionENU(const Vector3& pos);

    /**
     * @brief Updates spacecraft velocity in ENU frame.
     *
     * @param vel Velocity vector:
     * - x → East velocity [m/s]
     * - y → North velocity [m/s]
     * - z → Vertical velocity [m/s]
     */
    void setVelocityENU(const Vector3& vel);

    /**
     * @brief Updates yaw angle for top view visualization.
     *
     * @param yawDeg Yaw angle in degrees.
     *
     * @note Expected to be derived externally from quaternion or rotation state.
     */
    void setYawDeg(double yawDeg);

    /**
     * @brief Updates target landing position in ENU frame.
     *
     * @param target Target position vector:
     * - x → East [m]
     * - y → North [m]
     * - z → typically 0 (ground)
     */
    void setTargetENU(const Vector3& target);

    /**
     * @brief Updates engine thrust level.
     *
     * @param percent Thrust percentage [0–100].
     */
    void setThrust(double percent);

    /**
     * @brief Updates RCS active switch
     * @param current thrust vector
     * - x → East thrust [N]
     * - y → North thrust [N]
     * - z → Vertical thrust [N]
     * @param thrust
     */
    void setRCSActive(Vector3 thrust);

    /**
     * @brief Updates hull integrity state.
     *
     * @param spacecraftState_ Current spacecraft state enum.
     *
     * @note Determines visual color of the lander.
     */
    void setHullIntact(SpacecraftState spacecraftState_);

protected:
    /**
     * @brief Paints the complete landing visualization.
     *
     * @param event Paint event.
     */
    void paintEvent(QPaintEvent *event) override;

private:
    // =====================================================
    // Internal State
    // =====================================================

    Vector3 positionENU {0.0, 0.0, 0.0}; ///< Current position (ENU)
    Vector3 velocityENU {0.0, 0.0, 0.0}; ///< Current velocity (ENU)
    Vector3 targetENU   {0.0, 0.0, 0.0}; ///< Target position (ENU)

    double thrustPercent = 0.0; ///< Main Engine thrust [%]
    bool RCSActive = false;     ///< Switch that represents if RCS engines are active [0,1]
    int activeThruster = 0.0;   ///< Amount of engine which are active
    double yawDeg = 0.0;        ///< Yaw angle [deg]

    bool hullIntact = true;     ///< Hull integrity state

    QVector<QPointF> trajectoryEN; ///< Trajectory history (E-N plane)

    // =====================================================
    // Animation
    // =====================================================

    QTimer animationTimer; ///< Timer for motion stripe animation
    int motionOffset = 0;  ///< Animation phase offset

    // =====================================================
    // Rendering Helpers
    // =====================================================

    /**
     * @brief Draws a framed panel with title.
     */
    void drawPanelFrame(QPainter& p, const QRect& r, const QString& title);

    /**
     * @brief Draws side view (East-Up plane).
     */
    void drawSideView(QPainter& p, const QRect& r);

    /**
     * @brief Draws top view (East-North plane).
     */
    void drawTopView(QPainter& p, const QRect& r);

    /**
     * @brief Draws compact state/telemetry panel.
     */
    void drawStatusBox(QPainter& p, const QRect& r);

    /**
     * @brief Maps ENU coordinates to side view screen coordinates.
     */
    QPointF mapSideView(const QRect& r,
                        double east,
                        double up,
                        double maxEastAbs,
                        double maxUp) const;

    /**
     * @brief Maps ENU coordinates to top view screen coordinates.
     */
    QPointF mapTopView(const QRect& r,
                       double east,
                       double north,
                       double maxEastAbs,
                       double maxNorthAbs) const;

    /**
     * @brief Draws lander in side view.
     */
    void drawLanderSide(QPainter& p, const QPointF& center);

    /**
     * @brief Draws lander in top view including yaw orientation.
     */
    void drawLanderTop(QPainter& p, const QPointF& center, double yawDeg);

    /**
     * @brief Draws vector (e.g. velocity or thrust).
     */
    void drawVector(QPainter& p,
                    const QPointF& start,
                    const QPointF& vec,
                    const QColor& color);
};

#endif // LANDINGVIEW_H
