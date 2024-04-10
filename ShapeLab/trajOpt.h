#pragma once
#include "../QMeshLib/PolygenMesh.h"

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

#include <pathStruct.h>
#include <robSystem.h>

#include <omp.h>

#include "fileIO.h"
#include "osqp++.h"

#include "colliTrain.h"

#include <algorithm>


using Transform3d = Eigen::Transform<double, 3, Eigen::Isometry>;


struct updateInfo
{
	std::vector<Eigen::Matrix<double, 8, 1>> JointsUpdate;//储存关节角度
	std::vector<Transform3d> TtcpsUpdate, TposUpdate;//储存末端位姿（喷头、变位机）
	std::vector<Eigen::Matrix<double, 8, 1>> VelUpdate, AccUpdate, JerkUpdate;//储存速度，加速度，跃度
	Eigen::VectorXd phiiUpdate;//储存phi_i
	Eigen::VectorXd ThetaUpdate;// 解
	int a, b;

	std::vector<Eigen::Matrix<double, 5, 1>> kVelUpdate, kAccUpdate, kJerkUpdate;//储存系数
	std::vector<Eigen::Matrix<double, 5, 4>> dkVelUpdate, dkAccUpdate, dkJerkUpdate;//储存系数对时间的导数
	Eigen::VectorXd localTimeUpdate;//储存时间
	Eigen::VectorXd velTTPUpdate, acctTTPUpdate, accnTTPUpdate;//喷头速度、加速度
};




class trajOpt
{
public:
	trajOpt();
	~trajOpt();
	
	void setMaxDist(double x);

	std::vector<toolpath*> splitPath(toolpath* layerPath);

	void initializePath(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool checkColi, int layerIndex);
	void initializeOpt(toolpath* toolpath, robSystem* robs, bool optTime = false, Eigen::Vector4d kSet = Eigen::Vector4d(0, 0, 0, -1));
	void bcd(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool checkColi, int sizeWindow, bool show);
	void bcd_time(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool checkColi, int sizeWindow, bool show);

	void test(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool checkColi, bool show, int a, int b, bool boolTime);

	void getQVAJT(std::vector<Eigen::Matrix<double, 8, 1>>& q, std::vector<Eigen::Matrix<double, 8, 1>>& v, std::vector<Eigen::Matrix<double, 8, 1>>& a,
		std::vector<Eigen::Matrix<double, 8, 1>>& j, Eigen::VectorXd& t, Eigen::VectorXd& vT, Eigen::VectorXd& atT, Eigen::VectorXd& anT);

	void getToTase(int& status, double& k1, double& k2, double& k3, Eigen::MatrixXd& M, Eigen::VectorXd& JointMax, Eigen::VectorXd& JointMin,
		std::vector<Eigen::Matrix<double, 5, 1>>& kJerk, std::vector<Eigen::VectorXd>& Joints, std::vector<Eigen::VectorXd>& Jerk);

	void getSolution(std::vector<Eigen::Matrix<double, 8, 1>>& q, std::vector<Transform3d>& T, Eigen::VectorXd &time);

	void setTipCons(double v, double at, double an);
	void setvTip(double v);
	void setM(Eigen::Matrix<double,8,8> Minput);
	void setAngles(double t1, double t2, double t3);

	void setVmax(Eigen::Matrix<double, 8, 1> Vmax);
	void setJmax(Eigen::Matrix<double, 8, 1> Jmax);

private:
	double maxDist = 5; 

	double errorTol = 0;
	double errorTolBase = -0.1;

	double vTool = 10;
	Eigen::Matrix<double, 8, 1> vJointMax = 0.01 * Eigen::Matrix<double, 8, 1>::Ones();
	Eigen::Matrix<double, 8, 1> aJointMax = 2 * Eigen::Matrix<double, 8, 1>::Ones();
	Eigen::Matrix<double, 8, 1> jJointMax = 20 * Eigen::Matrix<double, 8, 1>::Ones();
	Eigen::Matrix<double, 8, 1> JointMax, JointMin;
	
	double vTTPMax = 20;
	double aTTPMax_T = 12, aTTPMax_N = 160;

	double vTip = 10;


	int nSamYita = 80;
	double k1=1, k2=1, k3=1;
	double kTime = 1;
	int dmax = 50;
	double tau1 = 0.00001, tau2 = 0.001,tau3=0.00001;
	double alpha = 0.94, beta = 0.985, gamma = 0.96;
	//double muMax = pow(36, 2);

	Eigen::Matrix<double, 8, 8> M = Eigen::Matrix<double, 8, 8>::Identity(); 

	std::vector<int> graphJoint(std::vector<std::vector<Eigen::Matrix<double, 8, 1>>> q);
	void local_optimal(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool& success, int a, int b, updateInfo& Info, bool checkColi, bool show = false);
	void local_optimal_time(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool& success, int a, int b, updateInfo& Info, bool checkColi, bool show = false);


	std::vector<Eigen::Matrix<double, 8, 1>> Joints;
	std::vector<Transform3d> Ttcps, Tpos;
	std::vector<Eigen::Matrix<double, 8, 1>> Vel, Acc, Jerk;
	Eigen::VectorXd phi_i;
	Eigen::VectorXd Theta;

	Eigen::VectorXd velTTP, acctTTP, accnTTP;

	std::vector<Eigen::Matrix<double, 5, 1>> kVel, kAcc, kJerk;
	std::vector<Eigen::Matrix<double, 5, 4>> dkVel, dkAcc, dkJerk;
	Eigen::VectorXd localTime;

	double dVel, dAcc, dJerk;//最大最小差值
	double dTime;//最大最小时间差值
	Eigen::VectorXd Curvature;//曲率

	bool isInLimits(Eigen::VectorXd X, Eigen::VectorXd Xmin, Eigen::VectorXd Xmax);
	bool isInLimits(double X, double Xmin, double Xmax);

	//求速度加速度跃度
	Eigen::VectorXd velNum(std::vector<Eigen::VectorXd> joint, std::vector<double> dist, Eigen::Matrix<double, 5, 1>& kVel);
	Eigen::VectorXd accNum(std::vector<Eigen::VectorXd> joint, std::vector<double> dist, Eigen::Matrix<double, 5, 1>& kAcc);
	Eigen::VectorXd jerkNum(std::vector<Eigen::VectorXd> joint, std::vector<double> dist, Eigen::Matrix<double, 5, 1>& kJerk);

	Eigen::Matrix<double, 5, 4> getdkVel(std::vector<double> dist);
	Eigen::Matrix<double, 5, 4> getdkAcc(std::vector<double> dist);
	Eigen::Matrix<double, 5, 4> getdkJerk(std::vector<double> dist);

	int status = 0;


	double getFastronScore(Eigen::VectorXd q, colliDataGotten supportData, Eigen::VectorXd Alpha, robSystem* robs);
	double getFastronScore(Eigen::VectorXd q, colliDataGotten supportData, Eigen::VectorXd Alpha, Eigen::VectorXd& diff, robSystem* robs);
	double kernelRQ(Eigen::VectorXd a, Eigen::VectorXd b, double gamma, double p);

	bool compare(Eigen::VectorXd a, Eigen::VectorXd b, Eigen::VectorXd tmp);

	void findFeasible(int i, Eigen::VectorXd theta0, Eigen::VectorXd joint0, robSystem* robs, toolpath* toolpath);
};