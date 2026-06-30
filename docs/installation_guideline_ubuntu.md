# Spaceflight Dynamics Framework (SDF) – Ubuntu Installation Guide

This guide describes how to set up the **Spaceflight Dynamics Framework (SDF)** on a fresh Ubuntu installation.

The current demonstration application included in the framework is **Moonlander**, which is built and executed through the same installation process.

---

# 1. Update System

Open a terminal and run:

```bash
sudo apt update
sudo apt upgrade -y
```

---

# 2. Install Basic Development Tools

Install essential build tools and utilities:

```bash
sudo apt install -y \
build-essential \
git \
cmake \
ninja-build
```

---

# 3. Install Qt6

The recommended installation method is the official Qt Online Installer.

## Download Qt Installer

1. Visit

    https://www.qt.io/download

2. Download the Qt Online Installer for Linux x64.

3. Create a Qt account if required.

---

## Run Installer

Navigate to your Downloads folder:

```bash
cd ~/Downloads
```

Make the installer executable:

```bash
chmod +x qt-unified-linux-x64-*.run
```

Run the installer:

```bash
./qt-unified-linux-x64-*.run
```

---

## Common Installer Error (XCB)

If the Qt Installer reports:

```
The required xcb cursor platform library was not found
```

Install the missing dependencies:

```bash
sudo apt update

sudo apt install -y \
libxcb-cursor0 \
libxcb-xinerama0 \
libxkbcommon-x11-0 \
libxcb-render0 \
libxcb-shape0 \
libxcb-xfixes0 \
libxcb-randr0 \
libxcb-image0 \
libxcb-keysyms1 \
libxcb-icccm4 \
libxcb-sync1 \
libxcb-xkb1 \
libx11-xcb1 \
libgl1-mesa-dev
```

Then rerun the installer.

---

## Recommended Qt Components

Install:

### Qt

- Latest Qt 6 Version
- Desktop GCC 64-bit

### Development Tools

- Qt Creator
- CMake
- Ninja
- Qt Installer Framework

---

# 4. Install Framework Dependencies

## Eigen3

The framework uses the **Eigen** library for all mathematical operations,
including vectors, matrices and quaternions.

Install:

```bash
sudo apt install -y libeigen3-dev
```

---

## NLopt

The optimization subsystem uses **NLopt**.

Install:

```bash
sudo apt install -y libnlopt-dev
```

---

## nlohmann JSON

Install:

```bash
sudo apt install -y nlohmann-json3-dev
```

---

# 5. Clone Repository

## Fork Repository

Before contributing:

1. Visit

    https://github.com/gerd-lrt-dev/spaceflight-dynamics-framework

2. Click **Fork**

3. Create your own fork.

---

## Clone Your Fork

```bash
mkdir -p ~/Code

cd ~/Code

git clone https://github.com/<your-username>/spaceflight-dynamics-framework.git

cd spaceflight-dynamics-framework
```

---

# 6. Open Project in Qt Creator

Launch Qt Creator:

```bash
~/Qt/Tools/QtCreator/bin/qtcreator
```

---

## Open Project

Open:

```
spaceflight-dynamics-framework/CMakeLists.txt
```

---

## Select Build Kit

Choose:

```
Desktop Qt 6 (GCC 64-bit)
```

Verify:

- GCC compiler detected
- Qt installation detected
- CMake detected

---

## Configure Project

Click

```
Configure Project
```

Qt Creator will automatically generate the build directory.

---

## Build

```
Build → Build Project
```

or

```
Ctrl + B
```

---

## Run

```
Run → Run
```

or

```
Ctrl + R
```

---

# 7. Verify Installation

A successful installation should allow you to

- load spacecraft configurations
- start simulation runs
- open the cockpit interface
- observe telemetry updates
- execute Moonlander demonstration scenarios

---

# Development Notes

The framework currently consists of

- simulation backend
- interface layer
- telemetry mapper
- telemetry data transfer objects (DTO)
- propulsion subsystem
- controller subsystem
- sensor subsystem
- Qt-based cockpit frontend

The interface layer provides a strict separation between frontend and backend. All communication between both layers is performed through the `TelemetryMapper` using frontend-oriented data transfer objects.

Future releases will introduce

- ROS2 integration
- telemetry export workflows
- advanced spacecraft dynamics

---

# Troubleshooting

## Compiler Missing

```bash
sudo apt install -y build-essential
```

---

## Git Missing

```bash
sudo apt install -y git
```

---

## Eigen Not Found

```bash
sudo apt install -y libeigen3-dev
```

---

## NLopt Not Found

```bash
sudo apt install -y libnlopt-dev
```

---

## JSON Library Not Found

```bash
sudo apt install -y nlohmann-json3-dev
```

---

## CMake Configuration Fails

Delete the build directory:

```bash
rm -rf build
```

Then reopen the project in Qt Creator and configure it again.

---

## Qt Version Not Detected

Verify the installation:

```bash
~/Qt/Tools/QtCreator/bin/qtcreator
```

Ensure that a **Qt 6 Desktop Kit** is installed.

---

# Recommended Platform

The framework is primarily developed and tested on

- Ubuntu 24.04 LTS (recommended)
- Ubuntu 22.04 LTS (supported)

---

# Additional Documentation

Project Website

https://www.aerospace-simulation.dev

Architecture Documentation

https://www.aerospace-simulation.dev/simulation/architecture/

Mathematical Models

Physics & Motion

https://www.aerospace-simulation.dev/mathematics/physics/

Main Engine Model

https://www.aerospace-simulation.dev/mathematics/thrust/

RCS Basic Model

https://www.aerospace-simulation.dev/mathematics/RCSBasicModel/

Adaptive Descent Controller

https://www.aerospace-simulation.dev/mathematics/adaptiveDescentController/

Impact & Structural Integrity

https://www.aerospace-simulation.dev/mathematics/impact/

---

✅ After completing these steps, the **Spaceflight Dynamics Framework** should build and run successfully on Ubuntu using the current CMake-based build system.
