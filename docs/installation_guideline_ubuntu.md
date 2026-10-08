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
ninja-build \
gdb \
pkg-config
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

## OpenGL development dependencies (required for a reliable desktop Qt build)

On a fresh Ubuntu 24.04 installation, install the OpenGL development headers **before configuring Qt Creator**:

```bash
sudo apt install -y libgl1-mesa-dev libopengl-dev libglx-dev
```

`libgl1-mesa-dev` is a transitional package on Ubuntu 24.04; the explicit GLVND development packages are included for clarity. These packages address missing OpenGL development dependencies; they do **not** replace a missing Qt Widgets installation.

---

## Recommended Qt Components

Install:

### Qt

- A Qt 6 version supported by the project (Qt 6.12.0 was used successfully in the October 2026 fresh-install test)
- **Desktop GCC 64-bit**, including the **Qt Widgets** module

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

## TinyXML2

Install the XML parsing development library:

```bash
sudo apt install -y libtinyxml2-dev
```

---

## Verify dependencies

```bash
dpkg -l libeigen3-dev libnlopt-dev nlohmann-json3-dev libtinyxml2-dev \
  libgl1-mesa-dev libopengl-dev libglx-dev
```

Installed packages are marked `ii`. The NLopt package is spelled **`libnlopt-dev`** (not `libnlop-dev`). A terminal's current working directory, including a mounted USB drive, does not determine where `apt` installs system packages.

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

Verify in **Edit → Preferences → Kits** (menu naming may differ by Qt Creator version):

- **Qt Versions:** the Qt installation points to the intended Desktop GCC 64-bit version (for example `~/Qt/6.12.0/gcc_64`).
- **Compilers:** a working GCC C++ compiler is selected (for example `/usr/bin/g++`).
- **Debugger:** GDB is available (for example `/usr/bin/gdb`).
- **CMake tool:** a working CMake installation is selected.
- **CMake generator:** Ninja is selected.
- **Kit:** the Desktop Qt kit associates all of the above with the intended Qt version; resolve any warning icons before continuing.

Qt Creator alone is not the Qt SDK: the matching **Desktop GCC 64-bit Qt libraries** must also be installed. Check the Widgets configuration file, adjusting the version if needed:

```bash
ls "$HOME/Qt/6.12.0/gcc_64/lib/cmake/Qt6Widgets/Qt6WidgetsConfig.cmake"
```

If the file is absent, open the Qt Maintenance Tool and install the **Desktop GCC 64-bit** component for that Qt version.

---

## Configure Project

Click

```
Configure Project
```

Qt Creator will automatically generate the build directory. In **Projects → Build Settings → CMake**, check the CMake configuration for the GUI build:

```text
BUILD_FRONTEND=ON
BUILD_TESTING=OFF
```

Use the actual CMake variable names with underscores; `BUILD-TESTING` or `EBUILD_FRONTEND` will not configure the intended options. Qt Creator normally supplies the Qt installation prefix through the selected kit. If Qt is not detected, verify the kit before adding custom paths.

If manually configuring from a terminal, use `-D` for **every** CMake cache variable:

```bash
cmake -S . -B build-qt-debug -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DBUILD_FRONTEND=ON \
  -DBUILD_TESTING=OFF \
  -DCMAKE_PREFIX_PATH="$HOME/Qt/6.12.0/gcc_64"
```

The terminal command is optional; the recommended GUI workflow uses Qt Creator.

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

## Automated backend tests (recommended)

From the repository root, configure a separate build without the Qt frontend:

```bash
cmake -S . -B build-tests -G Ninja \
  -DBUILD_TESTING=ON \
  -DBUILD_FRONTEND=OFF
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

This isolates the physics/mathematics tests from Qt configuration problems. In the October 2026 Ubuntu 24.04 fresh-install verification, all four then-existing tests passed (two Euler integrator tests, free translation, and axis torque rotation). The number of tests may change as development continues.

## GUI smoke test

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

First read the **first relevant CMake error**, rather than deleting build directories indiscriminately. Confirm the correct Qt kit, installed Widgets component, and OpenGL development packages. In Qt Creator, use **Build → Run CMake** or clear/reconfigure the CMake cache for the selected build configuration.

Only remove a specific disposable build directory if a clean reconfiguration is needed; do not delete source directories or unrelated build trees.

---

## Qt6_FOUND is FALSE / required Qt component Widgets not found

A common fresh-install error is:

```text
Qt6Config.cmake found, but Qt6_FOUND is FALSE
Failed to find required Qt component "Widgets"
```

This means CMake found Qt itself, but the **Widgets component or one of its dependencies** could not be resolved. The mere presence of `Qt6WidgetsConfig.cmake` does not prove that its dependencies are satisfied.

1. Confirm the Qt Creator kit uses the intended Desktop GCC 64-bit Qt installation.
2. Confirm Qt Widgets is installed through the Qt Maintenance Tool.
3. Install OpenGL development dependencies:

   ```bash
   sudo apt install -y libgl1-mesa-dev libopengl-dev libglx-dev
   ```

4. Rerun CMake from Qt Creator and inspect the **earliest underlying dependency error** if the problem persists. Do not assume OpenGL is the only possible cause.

---

## Qt Version Not Detected

Verify the installation:

```bash
~/Qt/Tools/QtCreator/bin/qtcreator
```

Ensure that a **Qt 6 Desktop Kit** is installed and configured with the Qt libraries, compiler, debugger, CMake, and Ninja. A kit can appear in the list yet remain invalid (warning icon) until its dependencies are resolved.

---

# Recommended Platform

The framework is primarily developed and tested on

- Ubuntu 24.04 LTS (recommended; fresh-install build and tests verified in October 2026)
- Ubuntu 22.04 LTS (supported according to project documentation; not part of this fresh-install verification)

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
