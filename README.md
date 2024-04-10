# Concurrent and Scalable Trajectory Optimization for Manufacturing with Redundant Robots

## Installation

Please compile the code with Qmake file "ShapeLab.pro".

**Platform**: Windows + Visual Studio + QT-plugin (tested version: VS2022 + QT5.14.2 + msvc2017_64)

**Install Steps**:
- **Install Visual Studio Extension plug-in (QT VS Tool)** to open the *.pro file and generate the project
- **Set 'ShapeLab' as the start up project**
- **Change Platform Toolset** for three projects (GLKLib, QMeshLib, and ShapeLab) if not matched. General->General Properties->Platform Toolset
- **Install required packages** (1)fcl, (2)abseil, (3)armadillo, (4)gsl, and (5)gtest in the ShapeLab project. An easy way to install these packages is to use the “[vcpkg](https://github.com/microsoft/vcpkg)”.

1. vcpkg: fcl, abseil, armadillo, gsl, gtest
2. 设置起始项目，添加h、cpp
3. c/c++: 附加包含目录中添加：ThirdPartyDependency/osqp 以及 libsvm
          预处理器定义添加： _USE_MATH_DEFINES
          语言：打开openmp
4. 链接器： 附加库目录添加：ThirdPartyDependency/osqp 以及 libsvm
           附加依赖项添加：osqp.lib osqp-cpp.lib libsvm.lib
           系统：修改为控制台
