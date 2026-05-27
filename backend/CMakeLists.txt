project(MoonLanderBackend)

find_package(NLopt REQUIRED)

file(GLOB
    BACKEND_SOURCES src/*.cpp
    BACKEND_HEADERS include/*.h
)

add_library(moonlander_backend
    ${BACKEND_SOURCES}
    ${BACKEND_HEADERS}
    src/Integrators/Dynamics.cpp
    src/Integrators/eulerIntegrator.cpp
    src/Optimization/thrustCostFunction.cpp
    src/Optimization/thrustOptimizer.cpp
    src/Physics/basicMoonGravityModel.cpp
    src/Sensory_Perception/sensorModel.cpp
    src/Automation/adaptiveDescentController.cpp
    src/Control/inputArbiter.cpp
    src/Controller/pd_controller.cpp
    src/Thrust/BasicMainEngineModel.cpp
    src/Thrust/BasicRCSModel.cpp
    include/Integrators/Dynamics.h
    include/Integrators/iIntegrator.h
    include/Integrators/eulerIntegrator.h
    include/Optimization/optimizationStruct.h
    include/Optimization/modelParams.h
    include/Optimization/thrustOptimizationProblem.h
    include/Optimization/thrustOptimizer.h
    include/Physics/iPhysicsModel.h
    include/Physics/basicMoonGravityModel.h
    include/Sensory_Perception/iSensor.h
    include/Sensory_Perception/sensorModel.h
    include/Automation/iautopilot.h
    include/Automation/adaptiveDescentController.h
    include/Control/inputArbiter.h
    include/Controller/iController.h
    include/Controller/pd_controller.h
    include/Thrust/iThrust.h
    include/Thrust/BasicMainEngineModel.h
    include/Thrust/FuelStateStruct.h
    include/Thrust/EngineConfig.h
    include/Thrust/RCSConfig.h
    include/Thrust/ME_thrustState.h
    include/Thrust/RCS_ThrustState.h
    include/Thrust/FueltankStruct.h
    include/Thrust/BasicRCSModel.h
    include/Thrust/RCSControlAllocator.h
)

target_include_directories(moonlander_backend
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(moonlander_backend
    PUBLIC
        NLopt::nlopt
)
