project(MoonLanderBackend)

find_package(NLopt REQUIRED)

file(GLOB
    BACKEND_SOURCES src/*.cpp
    BACKEND_HEADERS include/*.h
)

add_library(moonlander_backend

    # =========================================================================
    # Core
    # =========================================================================

    src/Thrust.cpp
    src/jsonConfigReader.cpp
    src/logger.cpp
    src/physics.cpp
    #src/quaternion.cpp
    src/simcontrol.cpp
    src/spacecraft.cpp
    src/spacemath.cpp

    include/Thrust.h
    include/customSpacecraftStruct.h
    include/environmentConfig.h
    include/jsonConfigReader.h
    include/physics.h
    include/simDataStruct.h
    include/simcontrol.h
    include/spacecraft.h
    include/spacecraftStateStruct.h
    include/spacemath.h
    include/stateVectorStruct.h

    # =========================================================================
    # Automation
    # =========================================================================

    src/Automation/adaptiveDescentController.cpp

    include/Automation/adaptiveDescentController.h
    include/Automation/iautopilot.h

    # =========================================================================
    # Control
    # =========================================================================

    src/Control/inputArbiter.cpp

    include/Control/inputArbiter.h

    # =========================================================================
    # Controller
    # =========================================================================

    src/Controller/pd_controller.cpp

    include/Controller/iController.h
    include/Controller/pd_controller.h

    # =========================================================================
    # Coordinate Systems
    # =========================================================================

    src/Coordinates/coordinateTransformer.cpp

    include/Coordinates/coordinateTransformer.h
    include/Coordinates/missionContext.h
    include/Coordinates/simulationFrameContext.h

    # =========================================================================
    # Numerical Integration
    # =========================================================================

    src/Integrators/Dynamics.cpp
    src/Integrators/eulerIntegrator.cpp

    include/Integrators/Dynamics.h
    include/Integrators/eulerIntegrator.h
    include/Integrators/iIntegrator.h

    # =========================================================================
    # Optimization
    # =========================================================================

    src/Optimization/thrustCostFunction.cpp
    src/Optimization/thrustOptimizer.cpp

    include/Optimization/modelParams.h
    include/Optimization/optimizationStruct.h
    include/Optimization/thrustOptimizationProblem.h
    include/Optimization/thrustOptimizer.h

    # =========================================================================
    # Physics Models
    # =========================================================================

    src/Physics/basicMoonGravityModel.cpp

    include/Physics/basicMoonGravityModel.h
    include/Physics/iPhysicsModel.h

    # =========================================================================
    # Sensor Models
    # =========================================================================

    src/Sensory_Perception/sensorModel.cpp

    include/Sensory_Perception/iSensor.h
    include/Sensory_Perception/sensorModel.h

    # =========================================================================
    # Propulsion Models
    # =========================================================================

    src/Thrust/BasicMainEngineModel.cpp
    src/Thrust/BasicRCSModel.cpp

    include/Thrust/BasicMainEngineModel.h
    include/Thrust/BasicRCSModel.h
    include/Thrust/EngineConfig.h
    include/Thrust/FuelStateStruct.h
    include/Thrust/FueltankStruct.h
    include/Thrust/ME_thrustState.h
    include/Thrust/RCSConfig.h
    include/Thrust/RCSControlAllocator.h
    include/Thrust/RCS_ThrustState.h
    include/Thrust/iThrust.h
)

target_include_directories(moonlander_backend
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(moonlander_backend
    PUBLIC
        NLopt::nlopt
)
