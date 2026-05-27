#pragma once
#include <string>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

#include "spacecraft.h"
#include "vector3.h"
#include "Thrust/RCSConfig.h"

/**
 * @class JsonConfigReader
 * @brief Utility class for loading and parsing JSON configuration files for Moonlander.
 *
 * This class provides static helper functions to load JSON files from disk and
 * convert them into fully initialized @ref customSpacecraft structures.  
 * It acts purely as a utility class and therefore holds no internal state.
 *
 * The constructor and destructor are intentionally empty and not meant to be used.
 */
class jsonConfigReader {
public:

    /**
     * @brief Default constructor.
     *
     * This class is designed as a purely static utility helper.  
     * The constructor performs no action and should generally not be used.
     */
    jsonConfigReader() = default;

    /**
     * @brief Default destructor.
     *
     * JsonConfigReader manages no resources;  
     * therefore, the destructor performs no cleanup.
     */
    ~jsonConfigReader() = default;

    /**
     * @brief Loads a JSON configuration file from disk.
     *
     * Opens the file at the provided path and reads its full JSON content.  
     * The result is returned as an @ref nlohmann::json object.
     *
     * @param filename Path to the JSON file (e.g. `"configs/lander.json"`).
     *
     * @return Parsed JSON object containing all data from the file.
     *
     * @throws std::runtime_error If the file cannot be opened or read.
     *
     * @note This function only loads the raw JSON document.  
     *       It does not interpret or validate its contents.
     */
    static nlohmann::json loadConfig(const std::string& filename);

    /**
     * @brief Parses a JSON object containing spacecraft configuration with multiple engines.
     *
     * This function reads all basic lander parameters and constructs
     * a vector of engines from the JSON array "engines".
     *
     * Each engine in JSON should provide:
     * - id        : unique integer identifier
     * - name      : descriptive engine name
     * - type      : type string, e.g. "main", "attitude"
     * - maxThrust : maximum thrust in Newtons
     * - Isp       : specific impulse in seconds
     * - timeConstant : engine response time in seconds
     * - responseRate : maximum thrust change rate in Hz
     * - direction : unit vector of thrust direction
     * - position  : engine position in body frame (Vector3)
     *
     * Example JSON snippet:
     * @code
     * "engines": [
     *   {
     *     "id": 0,
     *     "name": "MainEngine",
     *     "type": "main",
     *     "maxThrust": 7000.0,
     *     "Isp": 300.0,
     *     "timeConstant": 0.15,
     *     "responseRate": 8.0,
     *     "direction": { "x": 0, "y": 0, "z": 1 },
     *     "position":  { "x": 0, "y": 0, "z": 0.5 }
     *   },
     *   {
     *     "id": 1,
     *     "name": "AttitudeThruster",
     *     "type": "attitude",
     *     "maxThrust": 50.0,
     *     "Isp": 80.0,
     *     "timeConstant": 0.05,
     *     "responseRate": 20.0,
     *     "direction": { "x": 1, "y": 0, "z": 0 },
     *     "position": { "x": 0.2, "y": 0, "z": 0.3 }
     *   }
     * ]
     * @endcode
     *
     * @param j JSON object containing spacecraft and engine configuration.
     * @return Fully initialized customSpacecraft instance with engine vector filled.
     * @throws nlohmann::json::exception if required fields are missing or have wrong type.
     */
    static customSpacecraft parseLander(const nlohmann::json& j);
};
