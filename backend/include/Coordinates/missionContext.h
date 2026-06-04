#ifndef NAVIGATIONCONTEXT_H
#define NAVIGATIONCONTEXT_H

#include "coordinateTransformer.h"

/**
 * @struct MissionContext
 * @brief Stores mission-specific navigation reference data.
 *
 * This structure contains persistent navigation references that remain
 * independent of the current spacecraft state.
 *
 * Typical examples include:
 *
 * - landing sites
 * - target locations
 * - navigation beacons
 * - future docking targets
 *
 * The MissionContext is intended to provide stable reference frames
 * used by guidance and navigation algorithms.
 */
struct MissionContext
{
    /**
     * @brief Landing site expressed in Moon Surface Coordinates (MSC).
     *
     * This represents the primary mission target using geodetic lunar
     * coordinates consisting of:
     *
     * - latitude  [rad]
     * - longitude [rad]
     * - altitude  [m]
     *
     * The MSC representation serves as the canonical mission definition.
     * All other landing-site representations (MCMF, MCI and ENU) are
     * derived from this state during simulation initialization.
     */
    CoordinateTransformer::MoonSurfaceCoordinates MSC_LandingSite;

    /**
     * @brief Landing site state in the Moon-Centered Moon-Fixed frame.
     *
     * Used for local surface-relative navigation and ENU frame generation.
     */
    CoordinateTransformer::State MCMF_landingSite;

    /**
     * @brief Landing site state in the Moon-Centered Inertial frame.
     *
     * Useful when guidance algorithms operate directly in the inertial frame.
     */
    CoordinateTransformer::State MCI_landingSite;

    /**
     * @brief ENU frame attached to the landing site.
     *
     * This frame is expected to become the primary frame for terminal landing
     * guidance and precision landing operations.
     */
    CoordinateTransformer::ENUFrame ENU_landingSite;
};

#endif // NAVIGATIONCONTEXT_H
