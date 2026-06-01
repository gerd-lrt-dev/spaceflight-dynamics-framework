#ifndef SIMULATIONFRAMECONTEXT_H
#define SIMULATIONFRAMECONTEXT_H

#include "coordinateTransformer.h"

/**
 * @struct SimulationFrameContext
 * @brief Stores the current spacecraft state expressed in multiple reference frames.
 *
 * This structure contains frame representations derived from the primary
 * simulation state vector.
 *
 * The StateVector remains the single source of truth and is propagated in
 * the Moon-Centered Inertial (MCI) frame. The SimulationFrameContext is
 * reconstructed from the current StateVector whenever frame-dependent
 * information is required.
 *
 * Typical consumers include:
 *
 * - Guidance, Navigation and Control (GNC)
 * - Telemetry generation
 * - XML export
 * - Visualization
 * - Flight instrumentation
 * - Future docking and rendezvous systems
 *
 * No values stored in this structure are integrated directly by the physics
 * engine.
 */
struct SimulationFrameContext
{
    /**
     * @brief Spacecraft state expressed in the Moon-Centered Inertial frame.
     *
     * This frame serves as the primary physics frame of SDF.
     */
    CoordinateTransformer::State MCI_State;

    /**
     * @brief Spacecraft state expressed in the Moon-Centered Moon-Fixed frame.
     *
     * Useful for surface-relative calculations and geodetic conversions.
     */
    CoordinateTransformer::State MCMF_State;

    /**
     * @brief Spacecraft geodetic position on or above the lunar surface.
     *
     * Contains latitude, longitude and altitude.
     */
    CoordinateTransformer::MoonSurfaceCoordinates MSC_State;

    /**
     * @brief Spacecraft state expressed in the active local ENU frame.
     *
     * Typically used for landing guidance, navigation and telemetry.
     */
    CoordinateTransformer::State ENU_State;

    /**
     * @brief Active local East-North-Up frame.
     *
     * The origin is usually attached to a landing site or other mission
     * reference location.
     */
    CoordinateTransformer::ENUFrame ENU_Frame;

    /**
     * @brief Spacecraft state expressed in the LVLH frame.
     *
     * Primarily intended for orbital guidance, docking and rendezvous
     * operations.
     */
    CoordinateTransformer::State LVLH_State;

    /**
     * @brief Current Local Vertical Local Horizontal frame.
     */
    CoordinateTransformer::LVLHFrame LVLH_Frame;

    /**
     * @brief Current spacecraft body frame definition.
     *
     * Derived from the spacecraft attitude quaternion and state.
     */
    CoordinateTransformer::SpacecraftBodyFrame SBF_Frame;
};

#endif // SIMULATIONFRAMECONTEXT_H
