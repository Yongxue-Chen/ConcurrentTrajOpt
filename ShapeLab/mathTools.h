#pragma once

#include <cmath>
#include <limits>
#include <array>
#include <fstream>
#include <iostream>

#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>


namespace mathTools {
	Eigen::Matrix3d logm(Eigen::Matrix3d A);
	Eigen::Matrix4d logm(Eigen::Matrix4d A);
	Eigen::Matrix3d expm(Eigen::Matrix3d A);
	Eigen::Matrix4d expm(Eigen::Matrix4d A);
	Eigen::Matrix3d dexpm(Eigen::Matrix3d A);
	Eigen::Matrix4d dexpm(Eigen::Matrix4d A);

	Eigen::Matrix4d xi2xihat(Eigen::Matrix<double, 6, 1> xi);

	Eigen::Matrix3d skew(Eigen::Vector3d v);
	Eigen::Vector3d vex(Eigen::Matrix3d A);

	Eigen::Matrix<double, 6, 1> vex(Eigen::Matrix4d A);

	Eigen::Matrix3d diffRotm(Eigen::Vector3d w, Eigen::Vector3d dw);
}