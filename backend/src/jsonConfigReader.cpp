#include "jsonConfigReader.h"

void from_json(const nlohmann::json& j, Vector3& v) 
{
    if (j.is_array() && j.size() == 3) {
        v.x = j[0].get<double>();
        v.y = j[1].get<double>();
        v.z = j[2].get<double>();
    } 
    else {
        v.x = j.at("x").get<double>();
        v.y = j.at("y").get<double>();
        v.z = j.at("z").get<double>();
    }
}


void to_json(nlohmann::json& j, const Vector3& v) {
    j = nlohmann::json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
}

// Loads json Config file and return nlohmann object
nlohmann::json jsonConfigReader::loadConfig(const std::string& filename) 
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open config file: " + filename);
    }

    nlohmann::json j;
    file >> j;
    return j;
}

customSpacecraft jsonConfigReader::parseLander(const nlohmann::json& j)
{
    customSpacecraft lander;

    // -------------------------
    // Base lander properties
    // -------------------------
    lander.emptyMass    = j.at("emptyMass").get<double>();
    lander.fuelM        = j.at("fuelM").get<double>();

    lander.Ixx          = j.at("Ixx").get<double>();
    lander.Iyy          = j.at("Iyy").get<double>();
    lander.Izz          = j.at("Izz").get<double>();

    lander.I_initialPos = j.at("I_initialPos").get<Vector3>();
    lander.B_initialRot = j.at("B_initialRot").get<Vector3>();
    lander.B_initialCenterOfMass = j.at("B_initialCenterOfMass").get<Vector3>();

    lander.I_initialVelocity = j.at("initialVelocity").get<Vector3>();

    lander.structuralIntegrity = j.at("structuralIntegrity").get<double>();
    lander.safeVelocity        = j.at("safeVelocity").get<double>();

    // -------------------------
    // Engines
    // -------------------------
    const auto& engines = j.at("engines");
    for (const auto& e : engines)
    {
        EngineConfig engine;

        engine.Isp          = e.at("Isp").get<double>();
        engine.timeConstant = e.at("timeConstant").get<double>();
        engine.responseRate = e.at("responseRate").get<double>();
        engine.maxThrust    = e.at("maxThrust").get<double>();
        engine.direction    = e.at("direction").get<Vector3>();
        engine.position     = e.at("position").get<Vector3>();

        engine.id           = e.at("id").get<int>();
        engine.name         = e.at("name").get<std::string>();
        engine.type         = e.at("type").get<std::string>();
        engine.tankID       = e.at("tankID").get<int>();

        lander.engines_.push_back(engine);
    }

    // -------------------------
    // Tanks
    // -------------------------
    const auto& tanks = j.at("tanks");
    for (const auto& t : tanks)
    {
        FuelTank tank;

        tank.id             = t.at("id").get<int>();
        tank.name           = t.at("name").get<std::string>();
        tank.role           = t.at("role").get<std::string>();
        tank.mass           = t.at("fuelMass").get<double>();
        tank.capacity       = t.at("fuelMass").get<double>();

        lander.tanks_.push_back(tank);
    }

    return lander;
}


