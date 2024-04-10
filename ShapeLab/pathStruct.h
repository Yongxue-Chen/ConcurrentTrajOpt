#pragma once
#include "../QMeshLib/PolygenMesh.h"

using Transform3d = Eigen::Transform<double, 3, Eigen::Isometry>;

class toolpath
{
public:
	toolpath() {};
	toolpath(std::vector<Eigen::Vector3d> Point, std::vector<Eigen::Vector3d> Normal);
	~toolpath() {};

	void setPath(std::vector<Eigen::Vector3d> Point, std::vector<Eigen::Vector3d> Normal);

	void getPart(toolpath* newpath, int a, int b);

	void getDPath();
	void getDTime(double v);
	void getDTime(Eigen::VectorXd time);
	void getCosAngle();


	int length = -1;
	std::vector<Eigen::Vector3d> pathPoint; //路径位置（工件坐标）
	std::vector<Eigen::Vector3d> pathNormal; //曲面法向（工件坐标）
	std::vector<double> dPathPoint;//间距
	std::vector<double> dTimePoint;//时间间距
	std::vector<double> cosAngle;//离散点夹角

	std::vector<Transform3d> posWorkpiece; //工具位姿（工件坐标）

	bool hasJointPath = false; //是否求得关节角
	std::vector<Eigen::Vector2d> jointPos; //变位机关节角
	std::vector<Eigen::Matrix<double, 6, 1>> jointRob; //机械臂关节角

	std::vector<Eigen::Vector3d> toolPosition; //工具末端位置（世界坐标）
	std::vector<Eigen::Vector3d> toolPosture; //so(3)表示的工具末端姿态（世界坐标）
	std::vector<Transform3d> posWorld; //工具位姿（世界坐标）

	



	

};