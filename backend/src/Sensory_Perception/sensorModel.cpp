#include "Sensory_Perception/sensorModel.h"

double SensorModel::computeGLoad(const Eigen::Vector3d& totalAcceleration, const Eigen::Vector3d& gravityAcceleration, bool isLanded) const
{
    // Counterforce upon ground contact
    Eigen::Vector3d groundReaction = - gravityAcceleration;

    Eigen::Vector3d properAcceleration = {0.0, 0.0, 0.0};

    // Berechne Proper Acceleration
    if (isLanded)
    {
        properAcceleration = totalAcceleration - gravityAcceleration - groundReaction;
    }
    else
    {
        properAcceleration = totalAcceleration - gravityAcceleration;
    }


    double properNorm = properAcceleration.norm();
    double gLoad = properNorm / configData.earthGravity;

    return gLoad;
}
