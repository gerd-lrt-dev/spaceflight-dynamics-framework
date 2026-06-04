# Contributing to Spaceflight Dynamics Framework (SDF)

Thank you for your interest in contributing to the Spaceflight Dynamics Framework (SDF).

The Spaceflight Dynamics Framework is an open-source aerospace simulation framework focused on spacecraft dynamics, propulsion systems, guidance, navigation and control (GNC), telemetry workflows, and autonomous flight research.

Moonlander currently serves as the primary demonstration application built on top of the framework.

Contributions are welcome from developers, students, researchers, and aerospace enthusiasts.

---

# Project Philosophy

SDF is designed as a modular aerospace simulation framework.

The long-term goal is to provide a scalable engineering and research platform where physics models, propulsion systems, controllers, telemetry interfaces and visualization systems can evolve independently.

Every contributor directly influences how maintainable, extensible and scientifically useful the framework becomes.

Please keep this in mind when proposing changes.

---

# Core Architecture Principles

## Modularity

All systems should follow a modular architecture.

Simulation components should be separated into clearly defined domains:

- physics models
- propulsion systems
- controllers
- sensors
- spacecraft models
- integrators
- telemetry systems
- visualization systems
- optimization modules

Avoid tightly coupled implementations.

Subsystems should be replaceable without requiring major changes to the simulation core.

---

## Single Responsibility Principle

Functions and classes should ideally follow a single responsibility principle.

A component should:

- have one clearly defined purpose
- avoid duplicated logic
- avoid hidden side effects
- expose clear interfaces

If functionality is used in multiple places, it should be implemented once in a shared module.

---

## Interface-Driven Design

Whenever possible, systems should be implemented behind interfaces.

Examples already used in the framework include:

- IPhysicsModel
- IIntegrator
- ISensor
- IAutopilot
- IThrustModel

This allows subsystem implementations to be exchanged without modifying the simulation core.

---

## Frontend / Backend Decoupling

The framework follows a strict separation between:

- simulation backend
- telemetry interfaces
- visualization frontend

Frontend systems should never depend directly on backend implementation details.

The long-term architecture targets middleware-based communication using ROS2 and telemetry interfaces.

Contributors should avoid introducing direct dependencies between simulation logic and frontend visualization.

---

## Telemetry-First Design

Simulation data exposed to external systems should be transferred using dedicated telemetry structures.

Backend domain models should remain independent from frontend-specific data representations.

This architecture supports:

- frontend replacement
- telemetry export
- ROS integration
- distributed simulation environments
- future multi-application support

Whenever possible, introduce dedicated telemetry structures rather than exposing internal simulation models directly.

---

# Units and Physical Consistency

All physical quantities must be expressed in SI units.

Examples:

Length → meters (m)

Velocity → meters per second (m/s)

Acceleration → meters per second squared (m/s²)

Mass → kilograms (kg)

Force → Newton (N)

Time → seconds (s)

Angular velocity → radians per second (rad/s)

Units should always be clear from:

- variable names
- documentation
- Doxygen comments

Scientific consistency should always take priority over convenience.

---

# Documentation

Simulation software requires clear documentation.

---

## Doxygen

All public classes and important functions should contain Doxygen comments.

Documentation should describe:

- purpose
- assumptions
- units
- inputs
- outputs

AI-assisted documentation generation is welcome as long as it remains technically accurate.

---

## Architecture Documentation

When introducing new architectural components, contributors should:

1. Describe the design
2. Explain the purpose
3. Explain how the component integrates into the framework
4. Open an issue if architecture documentation updates are required

Architecture documentation is maintained on the project website.

---

## Mathematical Models

When introducing new mathematical models:

- document the equations
- define variables
- explain assumptions
- explain limitations
- explain intended use cases

If necessary, open an issue to ensure the equations are documented on the project website.

---

## Documentation Strategy

The project website serves as the primary source of technical documentation.

Contributors should update website documentation whenever introducing:

- mathematical models
- architecture changes
- propulsion systems
- controllers
- telemetry interfaces
- major simulation features

Repository markdown files should remain lightweight and focus primarily on:

- contribution workflow
- installation
- project organization
- developer onboarding

Project Website:

https://www.aerospace-simulation.dev

---

# Issue-Based Development

SDF follows a strict issue-based development process.

---

## Core Rule

NO CODE WITHOUT AN ISSUE.

Every development activity must be associated with a GitHub issue.

Reasons:

- development decisions remain traceable
- architectural changes remain documented
- collaboration becomes easier
- roadmap progress becomes visible

---

# Feature Development Workflow

If a contributor wants to implement a new feature or subsystem, the following process must be used:

1. Create a GitHub Issue
2. Describe the idea or required change
3. Create a feature branch
4. Implement the solution based on the issue
5. Submit a Pull Request referencing the issue

Example:

Issue:

```text
D23 - Basic RCS Model
```

Branch:

```text
feature/D23-Basic-RCS-Model
```

Development should always happen on the issue branch.

---

# Commit Message Convention

To keep the development history structured, each commit must begin with an issue identifier.

Format:

```text
<KEY><NUMBER> - Description
```

Example:

```text
D14 - Implement thrust saturation handling
```

---

# Issue Key Definitions

### D — Developer Task

Implementation, code changes or smaller features.

Example:

```text
D23 - Basic RCS Model
```

---

### M — Mathematical Model

Used when introducing or modifying:

- equations
- guidance laws
- optimization models
- controller formulations
- physics models

Example:

```text
M05 - Introduce energy-based descent metric
```

---

### A — Architecture

Structural changes such as:

- interfaces
- subsystem boundaries
- major refactoring
- framework restructuring

Example:

```text
A12 - Introduce telemetry abstraction layer
```

---

### W — Website / Documentation

Tasks affecting:

- website
- technical documentation
- figures
- mathematical documentation

Example:

```text
W03 - Update propulsion documentation
```

---

### P — Project Management

Tasks affecting:

- roadmap
- milestones
- repository organization
- planning activities

Example:

```text
P02 - Define release roadmap
```

---

### F — Feature Proposal

Large functional extensions.

Example:

```text
F04 - Multi-spacecraft simulation support
```

---

### B — Bugfix

Incorrect behavior or runtime errors.

Example:

```text
B07 - Fix thrust saturation bug
```

---

### R — Refactoring

Code restructuring without changing functionality.

Example:

```text
R08 - Refactor thrust orchestration module
```

---

### T — Tests / Validation

Verification and validation activities.

Example:

```text
T04 - Validate RCS actuator response
```

---

# Release Philosophy

The project follows a staged release strategy.

Current planned releases:

- v0.1 — SDF Light
- v0.2 — SDF Research

Contributors should consider whether a change primarily supports:

- contributor onboarding
- framework accessibility
- research capabilities
- long-term framework evolution

when defining issues and priorities.

---

# Contribution Workflow

Typical workflow:

1. Fork the repository
2. Create a feature branch

```bash
git checkout -b feature/my-feature
```

3. Implement your changes
4. Commit using the issue-key format
5. Submit a Pull Request referencing the issue

---

# Coding Guidelines

Contributors should generally prefer:

- readable code over clever code
- modular code over monolithic code
- documented code over undocumented code
- interfaces over hard dependencies
- explicit assumptions over hidden assumptions

When in doubt:

Choose the solution that improves maintainability.

---

# Final Note

The Spaceflight Dynamics Framework is both:

- an engineering project
- a research platform

The objective is not merely to create a simulation, but to develop a scalable and extensible framework for spacecraft dynamics, propulsion systems, telemetry workflows and autonomous flight research.

Every contributor influences the architecture and future direction of the framework.

Please keep:

- clarity
- modularity
- scientific consistency
- maintainability

in mind when contributing.

Thank you for helping build the Spaceflight Dynamics Framework.
