# Spaceflight Dynamics Framework (SDF) 🚀

Open-Source Aerospace Simulation Framework

The **Spaceflight Dynamics Framework (SDF)** is a modular C++ aerospace simulation framework designed for spacecraft dynamics, propulsion modeling, guidance and control systems, telemetry workflows, and autonomous flight research.

The framework is developed with a strong focus on:

- aerospace engineering applications
- spacecraft dynamics simulation
- guidance, navigation and control (GNC)
- propulsion system modeling
- telemetry and data analysis
- modular software architecture
- open-source collaboration

The project aims to provide a flexible research and development platform that can evolve from educational simulations toward advanced aerospace engineering and scientific applications.

---

# 🌙 Moonlander Demonstration Application

**Moonlander** is the first demonstration application built on top of the Spaceflight Dynamics Framework.

It serves as:

- a technology demonstrator
- a controller development environment
- a propulsion modeling testbed
- a framework validation platform

The Moonlander scenario currently focuses on autonomous lunar landing and spacecraft descent control.

🌐 **Project Website**

https://www.aerospace-simulation.dev

---

# 🚀 Demo

For a detailed demonstration, visit:

https://www.aerospace-simulation.dev/simulation/demo/

The current cockpit interface provides real-time visualization of:

- spacecraft state
- navigation data
- propulsion system state
- fuel system state
- controller output
- spacecraft integrity
- active RCS thruster telemetry

The landing view provides a lightweight 2.5D situational representation of the descent.

Features include:

- East-North-Up (ENU) reference frame visualization
- trajectory tracking
- velocity vectors
- target-relative motion
- active RCS indication
- propulsion telemetry integration

![Moonlander Cockpit](docs/cockpit.jpg)

---

# ✨ Features

## Core Framework Features

- Modular simulation architecture
- Physics model abstraction
- Integrator abstraction layer
- Sensor abstraction layer
- Controller abstraction layer
- Multi-engine propulsion architecture
- JSON-based spacecraft configuration
- Runtime spacecraft selection
- Threaded simulation backend
- Real-time telemetry interface
- Experimental optimization support via NLopt

---

## Propulsion Features

- Main engine modeling
- Individual RCS thruster modeling
- Multi-engine thrust aggregation
- Fuel tank assignment
- Multi-tank architecture
- Command-delay simulation
- First-order actuator dynamics
- RCS thrust allocation
- Engine-level telemetry

---

## Visualization Features

- Qt-based cockpit interface
- Dynamic telemetry display
- Active RCS thruster visualization
- Dynamic fuel tank visualization
- Navigation monitoring
- Controller observability
- Spacecraft state monitoring
- 2.5D landing visualization

---

## Research-Oriented Features

- Controller evaluation environment
- Telemetry-driven analysis
- Modular subsystem replacement
- Future ROS2 integration path
- Scientific model documentation
- Reproducible simulation workflows

---

# ⚙️ Quick Start

Detailed installation guides can be found in the repository documentation.

## Requirements

- C++20
- Qt6
- NLopt
- GCC / Clang / MSVC

---

## Install NLopt (Ubuntu)

```bash
sudo apt install libnlopt-dev
```

---

## Clone Repository

```bash
git clone https://github.com/gerd-lrt-dev/moonlander.git
cd moonlander
```

---

## Build

```bash
mkdir build
cd build

cmake ..
make
```

---

## Run

```bash
./moonlander
```

---

# 🛰 Reaction Control System (RCS)

The framework includes a modular Reaction Control System implementation.

Current capabilities:

- translational RCS control
- individual thruster simulation
- command-delay modeling
- first-order actuator response
- engine-level telemetry
- dynamic cockpit visualization

The propulsion architecture separates:

- command generation
- thrust allocation
- actuator dynamics
- thrust generation
- force aggregation

This enables future extension toward:

- rotational RCS control
- attitude control systems
- full 6-DOF spacecraft dynamics

Mathematical documentation:

- https://www.aerospace-simulation.dev/mathematics/RCSBasicModel/
- https://www.aerospace-simulation.dev/mathematics/thrust/
- https://www.aerospace-simulation.dev/mathematics/adaptiveDescentController/

---

# 🧠 Architecture Overview

Architecture documentation:

https://www.aerospace-simulation.dev/simulation/architecture/

The framework is organized into several independent domains:

## Physics

Responsible for:

- gravity models
- environmental effects
- force generation

---

## Integrators

Responsible for:

- state propagation
- numerical integration

---

## Propulsion

Responsible for:

- engine simulation
- fuel systems
- thrust generation
- RCS allocation

---

## Controllers

Responsible for:

- autopilots
- control laws
- guidance algorithms

---

## Sensors

Responsible for:

- telemetry generation
- derived measurements

---

## Frontend

Responsible for:

- visualization
- telemetry display
- operator interaction

The frontend is currently Qt-based and is being prepared for future ROS-based decoupling.

---

# 🔬 Scientific Use Case

The framework is intended to support:

- spacecraft dynamics research
- autonomous landing research
- controller development
- propulsion system studies
- telemetry analysis
- software architecture experimentation

The framework emphasizes:

- reproducibility
- modularity
- observability
- scientific transparency

Future data export capabilities will support:

- XML telemetry export
- controller comparison
- offline analysis workflows

---

# 🤖 Adaptive Descent Controller

The current Moonlander demonstration includes an energy-based Adaptive Descent Controller.

Controller capabilities:

- brake-ratio based descent planning
- gravity compensation
- adaptive gain scheduling
- thrust saturation handling
- phase-based landing logic

Documentation:

- https://www.aerospace-simulation.dev/mathematics/adaptiveDescentController/

---

# 🛰 Spacecraft Configuration

Spacecraft are defined using JSON configuration files.

Configurable elements include:

- mass properties
- inertia parameters
- propulsion systems
- fuel tanks
- structural limits
- initial conditions
- controller limits

This allows new spacecraft configurations to be introduced without modifying framework source code.

---

# 🚀 Release Strategy

The project follows a staged release strategy that separates accessibility-oriented framework releases from research-oriented simulation capabilities.

The goal is to support both:

- contributor onboarding
- framework stability
- long-term aerospace research development

---

## v0.1 — SDF Light

### Objective

Provide the first lightweight public release of the Spaceflight Dynamics Framework.

### Focus Areas

- simplified installation
- reduced dependency complexity
- contributor onboarding
- architecture visibility
- baseline simulation functionality

### Purpose

The Light Release serves as:

- an entry point for contributors
- a platform demonstration
- a stable framework foundation

---

## v0.2 — SDF Research

### Objective

Extend the framework toward research-oriented spacecraft simulation.

### Focus Areas

- advanced propulsion modeling
- telemetry export
- controller evaluation workflows
- optimization support
- scientific reproducibility

### Purpose

The Research Release serves as:

- an aerospace research platform
- a controller development environment
- a basis for future 6-DOF spacecraft dynamics

---

# 🗺 Roadmap

## Near-Term

- rotational RCS thruster model
- telemetry export (XML)
- frontend/backend abstraction layer
- improved spacecraft configuration workflows

---

## Mid-Term

- ROS2 integration
- automatic position controller
- controller testing environment
- advanced telemetry tooling

---

## Long-Term

- full 6-DOF spacecraft dynamics
- simulation replay system
- terrain modeling
- environmental effects
- orbital mechanics support
- multi-spacecraft simulation

---

# 🤝 Contributing

Interested in contributing?

Visit:

https://www.aerospace-simulation.dev/recruiting

Areas where support is welcome:

- aerospace simulation
- C++ development
- controller development
- ROS integration
- web development
- Qt frontend development
- mathematical modeling
- software architecture

Typical workflow:

1. Fork repository
2. Create feature branch
3. Implement changes
4. Submit pull request

Please read:

- CONTRIBUTING.md
- Installation Guidelines

before submitting contributions.

---

# 📚 Documentation

Complete project documentation is available on the project website.

### Architecture

https://www.aerospace-simulation.dev/simulation/architecture/

### Mathematics

Physics & Motion:
https://www.aerospace-simulation.dev/mathematics/physics/

Main Engine Model:
https://www.aerospace-simulation.dev/mathematics/thrust/

RCS Basic Model:
https://www.aerospace-simulation.dev/mathematics/RCSBasicModel/

Adaptive Descent Controller:
https://www.aerospace-simulation.dev/mathematics/adaptiveDescentController/

Impact & Structural Integrity:
https://www.aerospace-simulation.dev/mathematics/impact/

### Team

https://www.aerospace-simulation.dev/team/

---

# 📜 License

This project is released as open-source software.

License information is provided within the repository.
