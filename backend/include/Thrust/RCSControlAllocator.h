#pragma once

#include "vector3.h"

#include <cmath>

class RCSControlAllocator
{
public:
    static double mapAxisCommandToThrusterPercentage(const Vector3& rcsCommand, const Vector3& thrusterDirection)
    {
        const bool fire =
            (rcsCommand.x > 0.0 && thrusterDirection.x > 0.0) ||
            (rcsCommand.x < 0.0 && thrusterDirection.x < 0.0) ||
            (rcsCommand.y > 0.0 && thrusterDirection.y > 0.0) ||
            (rcsCommand.y < 0.0 && thrusterDirection.y < 0.0) ||
            (rcsCommand.z > 0.0 && thrusterDirection.z > 0.0) ||
            (rcsCommand.z < 0.0 && thrusterDirection.z < 0.0);

        return fire ? 1.0 : 0.0;
    }

    static double mapAxisCommandToThrusterNewton(const Vector3& rcsCommand, const Vector3& thrusterDirection)
    {
        if (rcsCommand.x > 0.0 && thrusterDirection.x > 0.0)
            return std::abs(rcsCommand.x);

        if (rcsCommand.x < 0.0 && thrusterDirection.x < 0.0)
            return std::abs(rcsCommand.x);

        if (rcsCommand.y > 0.0 && thrusterDirection.y > 0.0)
            return std::abs(rcsCommand.y);

        if (rcsCommand.y < 0.0 && thrusterDirection.y < 0.0)
            return std::abs(rcsCommand.y);

        if (rcsCommand.z > 0.0 && thrusterDirection.z > 0.0)
            return std::abs(rcsCommand.z);

        if (rcsCommand.z < 0.0 && thrusterDirection.z < 0.0)
            return std::abs(rcsCommand.z);

        return 0.0;
    }
};
