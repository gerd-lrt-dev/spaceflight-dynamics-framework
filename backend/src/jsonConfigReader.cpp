#include "jsonConfigReader.h"
#include "stateVectorStruct.h"

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

void from_json(const nlohmann::json& j, Quaternion& q)
{
    if (j.is_array())
    {
        if (j.size() != 4)
        {
            throw std::runtime_error("Quaternion array must contain exactly 4 elements.");
        }

        q = Quaternion(
            j[0].get<double>(),
            j[1].get<double>(),
            j[2].get<double>(),
            j[3].get<double>()
            );
    }
    else
    {
        q = Quaternion(
            j.at("w").get<double>(),
            j.at("x").get<double>(),
            j.at("y").get<double>(),
            j.at("z").get<double>()
            );
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

    const auto& initialstate = j.at("initialState");

    lander.MCI_initialPos           = initialstate.at("MCI_InitialPosition").get<Vector3>();
    lander.IB_initialRot            = initialstate.at("IB_InitialOrientation").get<Quaternion>();
    lander.MCI_initialVelocity      = initialstate.at("MCI_InitialVelocity").get<Vector3>();
    lander.SBF_initialCenterOfMass  = initialstate.at("SBF_InitialCenterOfMass").get<Vector3>();



    lander.structuralIntegrity = j.at("structuralIntegrity").get<double>();
    lander.safeVelocity        = j.at("safeVelocity").get<double>();

    // -------------------------
    // Main Engines
    // -------------------------
    const auto& engines = j.at("engines");
    for (const auto& e : engines)
    {
        if (e.at("type").get<std::string>() == "main")
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
    }

    // -------------------------
    // RCS Engines
    // -------------------------
    const auto& rcsengines = j.at("engines");
    for (const auto& e : rcsengines)
    {
        if (e.at("type").get<std::string>() == "translation")
        {
            RCSEngineConfig RCS;

            RCS.id                  = e.at("id").get<int>();
            RCS.name                = e.at("name").get<std::string>();
            RCS.type                = e.at("type").get<std::string>();
            RCS.axis                = e.at("axis").get<std::string>();
            RCS.tankID              = e.at("tankID").get<int>();

            RCS.Isp                 = e.at("Isp").get<double>();
            RCS.maxThrust           = e.at("maxThrust").get<double>();
            RCS.commandDelay        = e.at("commandDelay").get<double>();
            RCS.tauOn               = e.at("tauOn").get<double>();
            RCS.tauOff              = e.at("tauOff").get<double>();
            RCS.minimumPulseWidth   = e.at("minimumPulseWidth").get<double>();

            RCS.direction           = e.at("direction").get<Vector3>();
            RCS.position            = e.at("position").get<Vector3>();

            lander.RCSengines_.push_back(RCS);
        }
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

MissionContext jsonConfigReader::parseMissionContext(const nlohmann::json& j)
{
    MissionContext mission;

    const auto& missionJson = j.at("mission");
    const auto& landingSite = missionJson.at("landingSite");

    const std::string frame = landingSite.at("frame").get<std::string>();

    if (frame != "MSC")
    {
        throw std::runtime_error(
            "Unsupported landingSite frame: " + frame +
            ". Currently only MSC is supported.");
    }

    mission.MSC_LandingSite.latitude = landingSite.at("latitude").get<double>();

    mission.MSC_LandingSite.longitude = landingSite.at("longitude").get<double>();

    mission.MSC_LandingSite.altitude = landingSite.at("altitude").get<double>();

    return mission;
}


