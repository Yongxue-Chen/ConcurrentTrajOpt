#pragma once
#include "../QMeshLib/PolygenMesh.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "PQPLib/PQP.h"

#include "pathStruct.h"
#include "robSystem.h"
#include "colliTrain.h"

class fileIO
{
public:
	fileIO() {};
	~fileIO() {};

	void inputLayerPath(toolpath* path, std::string Dir, std::string layerIndex, PolygenMesh* Paths, double zAdjust = 0);

	void readRobotData(PolygenMesh* Rob_model, robSystem* robs, std::string FileDir);
	void readRobotData(robSystem* robs, std::string FileDir);
	void readPosData(PolygenMesh* cnc_Prt, robSystem* robs, std::string FileDir);
	void readPosData(robSystem* robs, std::string FileDir);
	void readLayerData(PolygenMesh* Layers, robSystem* robs, PQP_Model* layer_PQP, std::string FileDir, std::string layerIndex);
	void readLayerData(robSystem* robs, std::string FileDir, std::string layerIndex);

	void readJointsFromSVMData(std::vector<Eigen::VectorXd>& joints, std::vector<double>& labels, std::string Dir);

	void writeVar8(std::vector<Eigen::Matrix<double, 8, 1>> var, std::string Dir);
	void writeVar8(std::vector<Eigen::VectorXd> var, std::string Dir);
	void writeBaseColi(std::vector<Eigen::VectorXd> joints,std::vector<double> Coli,std::string Dir);

	void writeTwoVectors(std::vector<double> v1, std::vector<double> v2, std::string Dir);

	void writeColliData(colliDataGotten colli, std::string Dir);
	void readColliData(colliDataGotten& colli, std::string Dir);

	void writeVectorXd(Eigen::VectorXd v, std::string Dir);
	void readVectorXd(Eigen::VectorXd& v, std::string Dir);

	void writeTransform3d(std::vector<Transform3d> T, std::string Dir);
private:

};