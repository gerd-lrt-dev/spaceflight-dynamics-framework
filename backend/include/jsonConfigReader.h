#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "spacecraft.h"
#include "Thrust/RCSConfig.h"
#include "Coordinates/missionContext.h"

/**
 * @class jsonConfigReader
 * @brief Static utility class for loading and parsing SDF JSON configuration files.
 *
 * The jsonConfigReader provides helper functions for reading JSON configuration
 * files from disk and converting them into strongly typed simulation configuration
 * structures.
 *
 * The reader separates spacecraft-specific configuration from mission-specific
 * configuration:
 *
 * - @ref customSpacecraft contains vehicle properties such as mass, inertia,
 *   propulsion, tanks, initial dynamic state and spacecraft-fixed reference data.
 * - @ref MissionContext contains mission-level reference data such as landing
 *   sites and navigation targets.
 *
 * The class does not own simulation state and does not perform physics
 * initialization. Runtime frame data such as MCI, MCMF, ENU, LVLH or SBF states
 * should be derived later by the simulation using CoordinateTransformer and the
 * current StateVector.
 *
 * This class is stateless and intended to be used only through its static member
 * functions.
 */
class jsonConfigReader
{
public:

    /**
     * @brief Default constructor.
     *
     * The class is designed as a stateless static utility helper. The constructor
     * performs no initialization and should normally not be used directly.
     */
    jsonConfigReader() = default;

    /**
     * @brief Default destructor.
     *
     * The reader owns no resources and therefore performs no cleanup.
     */
    ~jsonConfigReader() = default;

    /**
     * @brief Loads a JSON configuration file from disk.
     *
     * Opens the file at the given path and reads its complete JSON content.
     * The raw parsed document is returned as an @ref nlohmann::json object.
     *
     * @param filename Path to the JSON configuration file.
     *
     * @return Parsed JSON document.
     *
     * @throws std::runtime_error If the file cannot be opened or read.
     * @throws nlohmann::json::parse_error If the file content is not valid JSON.
     *
     * @note This function only loads the JSON document. It does not validate
     *       spacecraft, mission or frame-specific semantics.
     */
    static nlohmann::json loadConfig(const std::string& filename);

    /**
     * @brief Parses spacecraft configuration from a JSON object.
     *
     * This function converts a single spacecraft JSON object into a
     * @ref customSpacecraft configuration structure.
     *
     * The parsed data includes:
     *
     * - dry mass and initial fuel mass
     * - rigid-body inertia values
     * - initial MCI position and velocity
     * - initial body-to-inertial orientation
     * - initial SBF angular velocity and center of mass
     * - main engine definitions
     * - RCS engine definitions
     * - fuel tank definitions
     * - structural and operational limits
     *
     * Frame convention:
     *
     * - Values prefixed with `MCI_` are expressed in the Moon-Centered Inertial frame.
     * - Values prefixed with `SBF_` are expressed in the Spacecraft Body Frame.
     * - Orientation values using `IB_` describe the rotation from body frame to inertial frame.
     *
     * Mission-level data such as landing sites or navigation targets should not
     * be parsed into @ref customSpacecraft. Use @ref parseMissionContext instead.
     *
     * @param j JSON object describing one spacecraft entry.
     *
     * @return Fully initialized @ref customSpacecraft instance.
     *
     * @throws nlohmann::json::exception If required fields are missing or have
     *         incompatible types.
     * @throws std::runtime_error If unsupported configuration values are detected.
     */
    static customSpacecraft parseLander(const nlohmann::json& j);

    /**
     * @brief Parses mission-level navigation context from a JSON object.
     *
     * This function extracts mission-specific reference data from a spacecraft
     * or scenario JSON object and stores it in a @ref MissionContext.
     *
     * The MissionContext is intended to hold persistent mission references such
     * as landing sites, target points or future navigation reference locations.
     * These values are not part of the physical spacecraft definition.
     *
     * For the current implementation, the landing site is expected to be defined
     * in lunar surface coordinates:
     *
     * @code
     * "mission": {
     *   "landingSite": {
     *     "frame": "MSC",
     *     "latitude": 0.0,
     *     "longitude": 0.0,
     *     "altitude": 0.0
     *   }
     * }
     * @endcode
     *
     * Frame convention:
     *
     * - `MSC` represents Moon Surface Coordinates.
     * - Latitude and longitude are expected in radians.
     * - Altitude is expected in meters above the mean lunar reference radius.
     *
     * Runtime frame representations such as MCMF landing site, MCI landing site
     * or landing-site ENU frame should be derived later using
     * CoordinateTransformer.
     *
     * @param j JSON object containing the mission section.
     *
     * @return Parsed @ref MissionContext.
     *
     * @throws nlohmann::json::exception If required fields are missing or have
     *         incompatible types.
     * @throws std::runtime_error If an unsupported mission frame is specified.
     */
    static MissionContext parseMissionContext(const nlohmann::json& j);
};
