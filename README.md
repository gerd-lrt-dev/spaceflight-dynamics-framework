# Moonlander 🚀
C++ Lunar Landing Simulation

Moonlander is a **modular C++ lunar landing simulation** designed as a research-oriented testbed for spacecraft guidance, control systems, and physics-based simulation.

The project separates a **simulation backend** from a **Qt-based cockpit UI**, enabling real-time telemetry, system observability, and controller evaluation during descent.

Moonlander is intended for developers interested in:

- aerospace simulations
- spacecraft guidance and control algorithms
- physics-based simulations
- modular C++ architecture
- autopilot and landing strategies
- simulation-based controller evaluation

🌐 **Project Website**  
[Project website](https://aerospace-simulation.dev)

---

# 🚀 Demo

For a detailed demonstration, visit the [project website demo](https://www.aerospace-simulation.dev/simulation/demo/).

Moonlander provides a **research-oriented cockpit interface** displaying spacecraft telemetry during descent.

Telemetry includes:

- full kinematic state (position and velocity in local ENU frame)
- vertical and lateral motion components
- propulsion state (throttle and multi-engine setup)
- dynamic fuel tank system (per-tank mass and fill level)
- hull integrity and spacecraft state machine
- autopilot state and controller output

The landing view provides a lightweight **2.5D situational visualization** of the descent.

It combines:
- a side view (East–Up) for vertical motion and descent dynamics
- a top view (East–North) for lateral drift and target-relative motion

The visualization includes trajectory history, velocity vectors and target reference,
allowing intuitive interpretation of controller behavior without requiring full 3D rendering.

![Moonlander Cockpit](docs/cockpit.jpg)

---

# ✨ Features

- Modular simulation architecture
- Interchangeable physics models
- Integrator abstraction layer
- Adaptive descent autopilot controller
- Real-time telemetry cockpit interface
- Qt-based graphical frontend
- Threaded simulation backend
- JSON-based spacecraft configuration
- Runtime spacecraft selection
- Experimental thrust optimization using NLopt

### Extended Simulation Features

- 2.5D situational landing visualization (ENU frame based)
- dynamic multi-tank fuel system with per-tank monitoring
- trajectory tracking and vector-based motion visualization
- controller observability (telemetry + system state exposure)
- separation of control space and physics space (commands vs forces)

---

# ⚙️ Quick Start

You can find detailed installation guidelines for Windows and Ubuntu in the `/docs` folder.

## Requirements

- C++20
- Qt6
- NLopt library
- GCC or Clang

Install NLopt on Linux:

sudo apt install libnlopt-dev

## Build

git clone https://github.com/gerd-lrt-dev/moonlander.git  
cd moonlander  
mkdir build  
cd build  
cmake ..  
make  

## Run

./moonlander

---

# 🧠 Architecture Overview

Detailed architecture with flow diagrams on the project website:
[project website architecture](https://www.aerospace-simulation.dev/simulation/architecture/).

Moonlander follows a **modular simulation architecture** where physics, control systems, sensors, and visualization are strictly separated.

The main system components include:

### Physics Models
Compute forces and environmental effects such as lunar gravity.

### Integrators
Advance spacecraft state using numerical integration methods.

### Controllers
Generate control commands (e.g. thrust levels) based on spacecraft state and guidance algorithms.

### Sensors
Provide telemetry derived from spacecraft state such as proper G-load.

### Simulation Core
Coordinates the simulation loop and manages spacecraft state updates.

### Frontend UI
Provides a research-oriented cockpit interface for real-time telemetry visualization.

The UI is designed for:
- controller debugging
- trajectory analysis
- propulsion system observation

It includes:
- ENU-based navigation display
- dynamic fuel system visualization
- 2.5D landing situational view

📚 Detailed architecture documentation:

docs/architecture.md

---

# 🔬 Scientific Use Case

Moonlander is designed as a testbed for evaluating lunar landing control strategies.

The system emphasizes:

- reproducible simulation conditions
- clear separation of physics, control and visualization
- real-time observability of system state
- modular integration of controllers and propulsion systems

The simulator can be used to:

- analyze descent trajectories
- evaluate controller stability and robustness
- compare fuel efficiency across control strategies
- study multi-engine and multi-tank propulsion architectures

Planned extensions include:

- structured simulation data export (CSV/XML)
- plotting and post-processing tools
- controller benchmarking framework

---

# 🤖 Adaptive Descent Controller

Moonlander includes an **energy-based autopilot controller** capable of performing automated lunar landings.

The controller evaluates spacecraft altitude, velocity, and thrust limits to compute safe descent trajectories.

Controller features include:

- brake ratio based descent planning
- adaptive gain scheduling
- gravity compensation
- thrust saturation handling
- phase-based descent control logic

📚 Technical details:

docs/adaptive_descent_controller.md
or mathmatical:
[project website adaptive descent controller](https://www.aerospace-simulation.dev/mathematics/adaptiveDescentController/)).

---

# 🛰 Spacecraft Configuration

Spacecraft are defined using **JSON configuration files**.

This allows new spacecraft variants to be added **without recompiling the simulation**.

Typical configuration parameters include:

- spacecraft mass
- fuel capacity
- engine configuration (multi-engine support)
- tank definitions (multi-tank architecture)
- thrust limits and response characteristics
- geometry parameters
- initial state conditions

Configurations are loaded at runtime using the `ConfigManager`.

---

# 🗺 Roadmap

## Short-Term

- implement reaction control system engine model (RCS)
- full 3D spacecraft visualization with rotational dynamics

## Mid-Term

- ROS2 integration for system decoupling
- Controller for automatic position control
- controller testing environment

## Long-Term

- simulation replay system
- terrain modelling
- environmental effects
- multi-spacecraft simulations

---

# 🤝 Contributing

Have a look at:  
[project website contributing](https://www.aerospace-simulation.dev/recruiting)

Contributions are welcome.

Areas where help is appreciated include:

- new autopilot algorithms
- additional sensor models
- improved physics models
- UI improvements
- optimization experiments

Typical contribution workflow:

1. Fork the repository
2. Create a feature branch
3. Implement your changes
4. Open a pull request

If you're looking for a starting point, check issues labeled:

- good first issue
- help wanted

---

# 📚 Documentation

Additional technical documentation can be found in the `docs` directory.

Detailed documentation, including mathematical derivations, is available here:  
[Project website mathmatical documentation](https://www.aerospace-simulation.dev/docs)

Example documentation files:

docs/architecture.md  
docs/adaptive_descent_controller.md

---

# 📜 License

This project is open source.  
License information will be provided in the repository.
