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
	std::vector<Eigen::Vector3d> pathPoint; //·��λ�ã��������꣩
	std::vector<Eigen::Vector3d> pathNormal; //���淨�򣨹������꣩
	std::vector<double> dPathPoint;//���
	std::vector<double> dTimePoint;//ʱ����
	std::vector<double> cosAngle;//��ɢ��н�

	std::vector<Transform3d> posWorkpiece; //����λ�ˣ��������꣩

	bool hasJointPath = false; //�Ƿ���ùؽڽ�
	std::vector<Eigen::Vector2d> jointPos; //��λ���ؽڽ�
	std::vector<Eigen::Matrix<double, 6, 1>> jointRob; //��е�۹ؽڽ�

	std::vector<Eigen::Vector3d> toolPosition; //����ĩ��λ�ã��������꣩
	std::vector<Eigen::Vector3d> toolPosture; //so(3)��ʾ�Ĺ���ĩ����̬���������꣩
	std::vector<Transform3d> posWorld; //����λ�ˣ��������꣩

	



	

};