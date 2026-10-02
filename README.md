# Spaceflight Dynamics Framework (SDF)

<p align="center">
  <img src="docs/images/Logo_with_background.png" width="220">
</p>

<p align="center">
<b>An open-source C++ framework for spacecraft simulation, guidance, navigation and control.</b>
</p>

<p align="center">
Modern • Modular • Extensible • Research-Oriented
</p>

---------------------------------------------------------------------------------------

## 🚀 What is SDF?

The **Spaceflight Dynamics Framework (SDF)** is an open-source C++
framework for developing modular aerospace simulation software.

Rather than focusing on a single simulation, SDF provides reusable 
building blocks for 6-DOF spacecraft dynamics, propulsion, guidance, 
navigation, control, telemetry and visualization.

Its mission is to provide a platform where aerospace enthusiasts,
students, researchers and developers can learn, experiment and develop
modern aerospace software while following clean software engineering
principles.

SDF is **not just a simulation**. It is intended to become an extensible
platform for education, experimentation and scientific research in
spacecraft dynamics and aerospace software engineering.

------------------------------------------------------------------------

## 🌙 Demonstration Application

The repository currently contains a demonstration application showcasing 
the capabilities of the framework including full six-degree-of-freedom 
spacecraft dynamics, autonomous lunar landing and a modular cockpit architecture.

Future applications can reuse the same framework architecture while
targeting completely different spacecraft missions and simulation
scenarios.

<p align="center">
  <img src="docs/images/cockpit.png" width="900">
</p>

<p align="center">
  <i>Current SDF demonstration application</i>
</p>

------------------------------------------------------------------------

## ✨ Framework Highlights

- Full 6-DOF rigid-body dynamics
- Quaternion-based attitude propagation
- Modular translational & rotational physics
- Modular simulation backend
- Dedicated interface layer
- Frontend / backend decoupling
- Telemetry Mapper
- Telemetry DTOs
- Eigen-based mathematics
- JSON spacecraft configuration
- Adaptive descent controller
- Multi-engine propulsion system
- Individual RCS thruster simulation
- Qt cockpit application

------------------------------------------------------------------------

## 🏗 Architecture

<p align="center">
  <img src="docs/images/architecture.png" width="900">
</p>

<p align="center">
  <i>SDF layered architecture overview</i>
</p>

Further documentation:

- [Core Data Flow Documentation](docs/data-flow-diagrams.md) — architectural data flow diagrams and subsystem ownership

https://www.aerospace-simulation.dev/simulation/architecture/

------------------------------------------------------------------------

## 🔬 Scientific Vision

SDF is intended to become an experimental environment for aerospace
research.

Research topics include:

-   optimal control
-   GNC
-   adaptive controllers
-   numerical integration
-   simulation stability
-   spacecraft dynamics
-   optimization
-   telemetry analysis
-   controller benchmarking
-   rigid-body dynamics
-   attitude dynamics
-   spacecraft simulation validation

------------------------------------------------------------------------

## ✅ Current Project Status

SDF has reached **Code Freeze for the first SDF Light release**.

The implementation scope of the initial **6DoF Core Simulation** is considered feature-complete. Development is now transitioning from feature implementation to **release qualification, verification and acceptance testing**.

### Code Freeze

For SDF Light, Code Freeze means:

- no new `D-issues` (development/features) are planned for the release
- the production code baseline is kept stable
- `T-issues` are now the primary focus for testing, verification and release acceptance
- `B-issues` may still be created and fixed if acceptance testing reveals reproducible defects
- non-critical enhancements and architectural extensions are deferred to later releases

### Implemented release baseline

The current SDF Light baseline includes:

- Full translational rigid-body dynamics
- Full rotational rigid-body dynamics
- Quaternion-based attitude propagation
- Quaternion-based attitude stabilization
- Modular main-engine and RCS propulsion
- Force & torque aggregation
- Deterministic fixed-step numerical propagation
- Multi-frame coordinate architecture
- Landing-site-relative initialization
- Structured telemetry export
- Qt cockpit demonstration application

### Test and verification status

The automated test infrastructure introduced with **T03** is now established and provides:

- GoogleTest-based C++ tests
- CTest integration
- separated unit, integration and verification test structure
- a reproducible local PASS / FAIL workflow for release qualification

The project is currently in the **acceptance-test phase** for SDF Light. The remaining work is focused on analytical reference cases, end-to-end verification, regression testing and the final Milestone 1 acceptance.

### Release status

**SDF Light is approaching its first public release.**

No additional functionality is planned for the release baseline. Remaining production-code changes should only result from confirmed defects found during verification or final release-candidate testing.

------------------------------------------------------------------------

## 🚧 Roadmap

The Spaceflight Dynamics Framework follows an incremental development strategy. Each release expands the framework while maintaining a stable architectural foundation.

<p align="center">
  <img src="docs/images/Release_Strategy.png" width="850">
</p>

<p align="center">
  <i>SDF development roadmap and release strategy</i>
</p>

The long-term vision is to extend the existing 6-DOF simulation core with:

- orbital mechanics
- advanced GNC algorithms
- ROS2 integration
- replay & telemetry analysis
- controller benchmarking
- planetary mission scenarios
- advanced sensor models
- environmental models

------------------------------------------------------------------------

## ⚙️ Quick Start

See the installation guides in the docs folder.

``` bash
git clone https://github.com/gerd-lrt-dev/spaceflight-dynamics-framework.git
cd spaceflight-dynamics-framework
mkdir build
cd build
cmake ..
cmake --build .
```

------------------------------------------------------------------------

## 📚 Documentation

Comprehensive documentation is available on the project website.

### 🌐 Project Website

https://www.aerospace-simulation.dev

---

### 🏗 Architecture

Framework architecture and software design.

https://www.aerospace-simulation.dev/simulation/architecture/

---

### 🎮 Demonstration

Overview of the current demonstration application.

https://www.aerospace-simulation.dev/simulation/demo/

---

### 🧮 Mathematics

Physics & Motion

https://www.aerospace-simulation.dev/mathematics/physics/

Coordinate Frames & Transformations

https://www.aerospace-simulation.dev/mathematics/coordinateTransformation/

Main Engine Model

https://www.aerospace-simulation.dev/mathematics/thrust/

Reaction Control System

https://www.aerospace-simulation.dev/mathematics/RCSBasicModel/

Adaptive Descent Controller

https://www.aerospace-simulation.dev/mathematics/adaptiveDescentController/

Impact & Structural Integrity

https://www.aerospace-simulation.dev/mathematics/impact/

---

### 👥 Team

Meet the people behind the project.

https://www.aerospace-simulation.dev/team/

------------------------------------------------------------------------

## 🤝 Contributing

Whether you're an experienced aerospace engineer, a software developer, a researcher, a student or simply curious about spacecraft simulation — contributions are always welcome.

SDF is an interdisciplinary project, and contributing means much more than writing code. There are many ways to get involved:

- develop new simulation models
- validate and benchmark physical models
- improve numerical methods
- design and evaluate control algorithms
- test and benchmark new features
- improve existing software components
- enhance the user interface
- extend the documentation
- discuss ideas, report bugs and suggest improvements

If you would like to contribute, please have a look at:

- CONTRIBUTING.md
- Installation Guidelines

Every contribution—whether it is code, validation, documentation, testing or engineering expertise—helps make SDF a better platform for learning, experimentation and aerospace research.

We'd be happy to have you onboard.

------------------------------------------------------------------------

## 📜 License

This project is released as open-source software.

See the `LICENSE` file for licensing information.

---

If you are interested in spacecraft simulation, aerospace software engineering or simply want to learn something new, you're invited to join the journey.
