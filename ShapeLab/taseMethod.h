#pragma once
#include "../QMeshLib/PolygenMesh.h"

#include "pathStruct.h"
#include "robSystem.h"
#include "colliTrain.h"
#include "trajOpt.h"

#include "nlopt.h"
#include "nlopt.hpp"


struct infoForNlopt {
	std::vector<Eigen::VectorXd> qOther;
	std::vector<Eigen::VectorXd> tmpKJerk;
	Eigen::MatrixXd M;
	robSystem* robs;
	std::vector<Eigen::Vector3d> pList;
	std::vector<Eigen::Vector3d> nList;
};


class taseMethod
{
public:
	taseMethod() {};
	~taseMethod() {};

	void initialize(trajOpt* Opt);

	void solve(toolpath* toolpath, robSystem* robs);

	double test;

private:
	int status = 0;
	double k1, k2, k3 = 1;
	Eigen::MatrixXd M;
	Eigen::VectorXd JointMax, JointMin;

	std::vector<Eigen::Matrix<double, 5, 1>> kJerk;

	std::vector<Eigen::VectorXd> Joints;
	std::vector<Eigen::VectorXd> Jerk;


	int maxIter = 120;
	double jointMax = 100;
	int dmax = 40;

	void localMinJerk(bool& success, int a, int b, robSystem* robs, toolpath* toolpath);

};

