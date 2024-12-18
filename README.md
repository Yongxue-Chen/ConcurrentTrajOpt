# Co-Optimization of Tool Orientations, Kinematic Redundancy, and Waypoint Timing for Robot-Assisted Manufacturing

## Installation

Please compile the code with Qmake file "ShapeLab.pro".

**Platform**: Windows + Visual Studio + QT-plugin (tested version: VS2022 + QT5.14.2 + msvc2017_64)

**Install Steps**:
- **Install Visual Studio Extension plug-in (QT VS Tool)** to open the *.pro file and generate the project
- **Set 'ShapeLab' as the Startup project**
- **Change Platform Toolset** for three projects (GLKLib, QMeshLib, and ShapeLab) if not matched: Property Pages -> Configuration Properties -> General->General Properties -> Platform Toolset -> choose the correct platform toolset.
- **Install required packages** (1)fcl, (2)abseil, (3)armadillo, (4)gsl, and (5)gtest in the ShapeLab project. An easy way to install these packages is to use the “[vcpkg](https://github.com/microsoft/vcpkg)”. **Note**: Installing fcl with vcpkg may result in a compilation error when the solution configuration is "Debug". Therefore, it is best to use the "Release" configuration.
- **Add additional library dictionaries** at: ShapeLab Property Pages -> Configuration Properties -> Linker -> General -> Additional Library Dictionaries -> add two dictionaries "..\ThirdPartyDependence\libsvm" and "..\ThirdPartyDependence\osqp"
- **Enable OpenMP to get best performace** at: ShapeLab Property Pages -> Configuration Properties -> C/C++ -> Language -> Open MP Support -> Select '**Yes (/openmp)**'
- **Open Console** at: ShapeLab Property Pages -> Configuration Properties -> Linker -> System -> SubSystem -> Select '**Console (/SUBSYSTEM:CONSOLE)**'

## Usage

**Step 0: Load robot model:**
Click button **Load Robots** on the right side of the UI.

**Step 1: Load toolpath and layer:**
Click button **inputPathAndLayer**. Some examples for testing are given in **\DataSet\models**

**Step 2: Initialize optimization:**
Click button **Initialize Optimization**.

**Step 3: Optimization:**
Click button **Optimization**. The robot joint angles, as well as the position and orientation of the tool in the base coordinate system, will be saved into **\DataSet\output**

## Contact Information
- Yongxue Chen (yongxue.chen@postgrad.manchester.ac.uk)
- Charlie C.L. Wang (changling.wang@manchester.ac.uk)
