# Concurrent and Scalable Trajectory Optimization for Manufacturing with Redundant Robots

## Installation

Please compile the code with Qmake file "ShapeLab.pro".

**Platform**: Windows + Visual Studio + QT-plugin (tested version: VS2022 + QT5.14.2 + msvc2017_64)

**Install Steps**:
- **Install Visual Studio Extension plug-in (QT VS Tool)** to open the *.pro file and generate the project
- **Set 'ShapeLab' as the start up project**
- **Change Platform Toolset** for three projects (GLKLib, QMeshLib, and ShapeLab) if not matched: Property Pages -> Configuration Properties -> General->General Properties -> Platform Toolset -> choose the correct platform toolset.
- **Install required packages** (1)fcl, (2)abseil, (3)armadillo, (4)gsl, and (5)gtest in the ShapeLab project. An easy way to install these packages is to use the “[vcpkg](https://github.com/microsoft/vcpkg)”.
- **Add additional library dictionaries** at: ShapeLab Property Pages -> Configuration Properties -> Linker -> General -> Additional Library Dictionaries -> add two dictionaries "..\ThirdPartyDependence\libsvm" and "..\ThirdPartyDependence\osqp"
- **Enable OpenMP to get best performace** at: ShapeLab Property Pages -> Configuration Properties -> C/C++ -> Language -> Open MP Support -> Select '**Yes (/openmp)**'
- **Open Console** at: ShapeLab Property Pages -> Configuration Properties -> Linker -> System -> SubSystem -> Select '**Console (/SUBSYSTEM:CONSOLE)**'
