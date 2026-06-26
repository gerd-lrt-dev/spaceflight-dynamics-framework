#ifndef ENGINECONFIG_H
#define ENGINECONFIG_H

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

/**
 * @brief Configuration parameters for a single spacecraft engine.
 *
 * This structure contains both identifying metadata and static physical
 * parameters describing an engine's performance, orientation, and position.
 *
 * Typical use:
 * - Main descent engines
 * - Attitude control thrusters (RCS)
 *
 * Engines can be stored in a vector inside a spacecraft struct
 * to support multi-engine configurations.
 */
struct EngineConfig
{
    // -------------------
    // Metadata
    // -------------------
    bool engineActivated = false;
    /**<
     * @brief Switch to activate engine
     *
     * if engine is deactivated no thrust will be provided from this engine
    */

    int id;
    /**<
     * @brief Unique engine identifier
     * @unit none
     * Used to distinguish engines programmatically.
     */

    std::string name;
    /**<
     * @brief Human-readable engine name
     * Example: "Main Descent Engine", "RCS Port Thruster"
     */

    std::string type;
    /**<
     * @brief Engine type category
     * Example: "Main", "RCS", "Vernier", "AttitudeControl"
     */

    int tankID;
    /**<
     * @brief Specifies which tank the engine uses
     */

    // -------------------
    // Physical Parameters
    // -------------------
    double Isp;
    /**<
     * @brief Specific impulse of the engine [s]
     * Determines propellant efficiency.
     * Typical ranges:
     * - Cold Gas: 40–80
     * - Hydrazine: 200–230
     * - Bipropellant: 280–320
     * - Lunar Lander main engine: ~311
     */

    double timeConstant;
    /**<
     * @brief Engine dynamic response time (τ) [s]
     * Recommended: 0.3–0.8 s for lunar landing engines.
     */

    double responseRate;
    /**<
     * @brief Maximum rate of thrust change [Hz]
     * Typical: 2–10 Hz for main engines, higher for RCS thrusters.
     */

    double maxThrust;
    /**<
     * @brief Maximum thrust the engine can provide [N]
     * Main engines: 4,000–45,000 N
     * RCS thrusters: 10–500 N
     */

    Eigen::Vector3d direction;
    /**<
     * @brief Normalized thrust direction in spacecraft body frame [-]
     * Should never be zero; normalized automatically.
     */

    Eigen::Vector3d position;
    /**<
     * @brief Engine position relative to spacecraft body frame [m]
     * For point-mass lander this can be nominal; used in multi-engine
     * torque calculations if moment arms are considered.
     */

    /**
     * @brief Creates a validated EngineConfig instance with metadata.
     *
     * Checks physical parameters for plausibility and sets defaults if out of range.
     *
     * @param id Unique engine identifier
     * @param name Human-readable engine name
     * @param type Engine type/category
     * @param Isp Specific impulse [s]
     * @param timeConstant Engine response time [s]
     * @param responseRate Maximum thrust change rate [Hz]
     * @param maxThrust Maximum thrust [N]
     * @param direction Thrust vector (body frame)
     * @param position Engine position in body frame
     *
     * @return EngineConfig struct with validated fields
     */
    static EngineConfig Create(bool engineActivated,
                               int id,
                               const std::string& name,
                               const std::string& type,
                               int tankID,
                               double Isp,
                               double timeConstant,
                               double responseRate,
                               double maxThrust,
                               Eigen::Vector3d direction,
                               Eigen::Vector3d position)
    {
        if(Isp < 40.0 || Isp > 900.0) {
            std::cerr << "[EngineConfig] Warning: Isp out of range (" << Isp << "), resetting to default 300 [s].\n";
            Isp = 300.0;
        }

        if(timeConstant < 0.05 || timeConstant > 3.0) {
            std::cerr << "[EngineConfig] Warning: timeConstant out of range (" << timeConstant << "), resetting to default 0.5 [s].\n";
            timeConstant = 0.5;
        }

        if(responseRate < 2.0 || responseRate > 50.0) {
            std::cerr << "[EngineConfig] Warning: responseRate out of range (" << responseRate << "), resetting to default 8.0 [Hz].\n";
            responseRate = 8.0;
        }

        if(maxThrust <= 0.0) {
            std::cerr << "[EngineConfig] Warning: maxThrust <= 0, resetting to 100 [N].\n";
            maxThrust = 100.0;
        }

        if(direction.norm() == 0.0) {
            std::cerr << "[EngineConfig] Warning: thrust direction zero, resetting to (0,0,1).\n";
            direction = {0.0,0.0,1.0};
        } else {
            direction = direction.normalized();
        }

        return {engineActivated, id, name, type, tankID, Isp, timeConstant, responseRate, maxThrust, direction, position};
    }
};

#endif // ENGINECONFIG_H
