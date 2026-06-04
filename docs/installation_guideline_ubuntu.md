# Spaceflight Dynamics Framework (SDF) – Ubuntu Installation Guide

This guide describes how to set up the **Spaceflight Dynamics Framework (SDF)** on a fresh Ubuntu installation.

The current demonstration application included in the framework is **Moonlander**, which is built and executed through the same installation process.

---

# 1. Update System

Open a terminal and run:

    sudo apt update
    sudo apt upgrade -y

---

# 2. Install Basic Development Tools

Install essential build tools and utilities:

    sudo apt install -y \
    build-essential \
    git \
    cmake \
    ninja-build

---

# 3. Install Qt6

The recommended installation method is the official Qt Online Installer.

---

## Download Qt Installer

1. Visit:

       https://www.qt.io/download

2. Download the Qt Online Installer for Linux x64.

3. Create a Qt account if required.

---

## Run Installer

Navigate to your Downloads folder:

    cd ~/Downloads

Make the installer executable:

    chmod +x qt-unified-linux-x64-*.run

Run the installer:

    ./qt-unified-linux-x64-*.run

---

## Common Installer Error (XCB)

If Qt Installer fails with:

    The required xcb cursor platform library was not found

Install the missing dependencies:

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

Then rerun the installer.

---

## Recommended Qt Components

Within the installer select:

### Qt

- Latest Qt 6 Version (recommended)
- Desktop GCC 64-bit

### Development Tools

- Qt Creator
- CMake
- Ninja
- Qt Installer Framework

---

# 4. Install Framework Dependencies

---

## NLopt

The optimization subsystem currently uses NLopt.

Install:

    sudo apt install -y libnlopt-dev

---

## nlohmann JSON

Install:

    sudo apt install -y nlohmann-json3-dev

---

# 5. Clone Repository

---

## Fork Repository

Before contributing:

1. Visit the project repository:

       https://github.com/gerd-lrt-dev/spaceflight-dynamics-framework

2. Click **Fork**

3. Create your own fork

---

## Clone Your Fork

    mkdir -p ~/Code

    cd ~/Code

    git clone https://github.com/<your-username>/spaceflight-dynamics-framework.git

    cd spaceflight-dynamics-framework

---

# 6. Open Project in Qt Creator

Launch Qt Creator:

    ~/Qt/Tools/QtCreator/bin/qtcreator

---

## Open Project

Select:

    File → Open File or Project

Open:

    spaceflight-dynamics-framework/CMakeLists.txt

---

## Select Build Kit

Choose:

    Desktop Qt 6 (GCC 64-bit)

Verify:

- GCC compiler detected
- Qt version detected
- CMake detected

---

## Configure Project

Press:

    Configure Project

Qt Creator should automatically generate the build directory.

---

## Build

    Build → Build Project

or:

    Ctrl + B

---

## Run

    Run → Run

or:

    Ctrl + R

---

# 7. Verify Installation

A successful installation should allow you to:

- load spacecraft configurations
- start simulation runs
- open the cockpit interface
- observe telemetry updates
- execute Moonlander demonstration scenarios

---

# Development Notes

The framework currently consists of:

- simulation backend
- propulsion subsystem
- controller subsystem
- sensor subsystem
- telemetry layer
- Qt-based cockpit frontend

Future releases will introduce:

- ROS2 integration
- frontend/backend decoupling
- telemetry export workflows
- advanced spacecraft dynamics

---

# Troubleshooting

---

## Compiler Missing

    sudo apt install -y build-essential

---

## Git Missing

    sudo apt install -y git

---

## NLopt Not Found

    sudo apt install -y libnlopt-dev

---

## JSON Library Not Found

    sudo apt install -y nlohmann-json3-dev

---

## CMake Configuration Fails

Delete the build directory and reconfigure:

    rm -rf build

Then reopen the project in Qt Creator and configure again.

---

## Qt Version Not Detected

Verify installation:

    ~/Qt/Tools/QtCreator/bin/qtcreator

and ensure a Qt 6 Desktop Kit is installed.

---

# Recommended Platform

The framework is primarily developed and tested on:

- Ubuntu 24.04 LTS (recommended)
- Ubuntu 22.04 LTS (supported)

---

# Additional Documentation

Project Website:

https://www.aerospace-simulation.dev

Architecture Documentation:

https://www.aerospace-simulation.dev/simulation/architecture/

Mathematical Models:

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

✅ After completing these steps, the Spaceflight Dynamics Framework should build and run successfully on Ubuntu.
