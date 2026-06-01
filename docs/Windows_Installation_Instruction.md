# Setting Up the Development Environment for Spaceflight Dynamics Framework

This is an instruction how to install the project under Windows 11. Caution the installation is a little complex. The installation on debian based systems such as ubuntu or mac is way more easier.

## 1. Download and Install Qt
1. Go to [Qt Downloads](https://doc.qt.io/qt-6/windows.html)
2. Create an account and log in
3. Select **Download Qt for Open Source**
4. Platform: **Windows x64**
5. Qt Version: **Latest Version** for Desktop Development
6. Installation: **User Defined Installation**  
   Make sure the following options are selected:
   - **Qt Latest Version**
     - Ensure MSVC support is enabled
   - **Qt Build Tools**
     - Latest CMake version
     - Latest Ninja version
   - **Qt Creator Latest Version**

## 2. Install Git
- Download: [Git for Windows](https://git-scm.com/install/windows)
- Setup: **Git for Windows/x64**
- Select the following options:
  - **Windows Explorer Integration**
    - Open Git Bash here
    - Open Git GUI here
  - **Git LFS (Large File Support)**
  - Associate `.git*` configuration files with default text editor
  - Associate `.sh` files to run with Bash
  - Scalar (Git add-on for large-scale repositories)
  - Git from command line & 3rd-party software
  - Use bundled OpenSSH
  - Use OpenSSL library
  - Checkout Windows-style, commit Unix-style line endings
  - Use Windows Default console window
  - Fast-Forward or Merge
  - Git Credential Manager

## 3. Install Visual Studio
- Edition: **Community**
- Workloads:
  - Desktop development with C++
  - CMake Tools for Windows
  - MSVC Build Tools (x64/x86, latest version)
  - Windows 11 SDK (should be selected by default)

## 4. Install CMake
- Download: [CMake](https://cmake.org/download/)
- Installer: **Windows x64 Installer**
- Verify installation: In powershell type "cmake --version"

## 5. Install PowerShell Core
- Download: [PowerShell 7.5.4](https://github.com/PowerShell/PowerShell/releases/tag/v7.5.4)
- Installer: `PowerShell-7.5.4-win-x64.msi`

## 6. Install Ninja
- Download: [Ninja v1.13.2](https://github.com/ninja-build/ninja/releases/tag/v1.13.2)
- Use `ninja-win.zip`

## 7. Install 7-Zip
- Download: [7-Zip](https://www.7-zip.org/)
- Choose **x64 version for Windows** and install

## 8. Download NLOPT
- Download `stevengj-nlopt-v2.10.1.tar.gz` from [nlopt GitHub](https://github.com/stevengj/nlopt/archive/v2.10.1.tar.gz)
- Rename file to `stevengj-nlopt-v2.10.1.tar.gz`
- Move it to `C:\vcpkg\downloads\`

## 9. Install NLOPT with vcpkg
- Go to powershell
- Type "git clone https://github.com/microsoft/vcpkg C:\vcpkg"
- type cd C:\vcpkg
- execute .\bootstrap-vcpkg.bat
- execute .\vcpkg.exe install nlopt:x64-windows

## 10. Clone the Spaceflight Dynamics Framework Repository
- Fork the repository into your git profil
- Go to powershell
- type cd <your_project_folder>
- type git clone https://github.com/<your-username>/spaceflight-dynamics-framework.git

## 11. Integrate vcpkg with the System
- Go to powershell
- type cd C:\vcpkg
- execute .\vcpkg integrate install

## 12. Install nlohmann JSON for Config Parsing
- Download `nlohmann-json-v3.12.0.tar.gz` from [nlohmann/json](https://github.com/nlohmann/json/archive/v3.12.0.tar.gz)
- Move it to `C:\vcpkg\downloads\`
- Go to powershell
- execute .\vcpkg.exe install nlohmann-json:x64-windows

## 13. Adapt Qt Kit Settings for NLOPT
1. Open **Qt Creator → Tools → Options → Kits**
2. Select `Desktop Qt MSVC2022 64-bit`
3. Add the following to CMake configuration:  
-DCMAKE_TOOLCHAIN_FILE:FILEPATH=C:/vcpkg/scripts/buildsystems/vcpkg.cmake

## 14. Configure the Project
1. Open the top-level `CMakeLists.txt`
2. Select the kit from step 13
3. Build configurations: **Debug** and **Release**
