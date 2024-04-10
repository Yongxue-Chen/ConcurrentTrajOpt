#include "fileIO.h"

void fileIO::inputLayerPath(toolpath* path, std::string Dir, std::string layerIndex, PolygenMesh* Paths, double zAdjust) {
	//输入路径
	char filename[1024];

	sprintf(filename, "%s%s%s", Dir.c_str(), layerIndex.c_str(), ".txt");
	

	std::ifstream fin(filename);
	if (!fin) {
		std::cerr << "Fail to open path file" << std::endl;
		return;
	}
	
	std::vector<std::vector<double>> m_matrix;
	std::string line;
	

	while (getline(fin,line)) {
		std::vector<double> row;
		std::stringstream ss(line);
		double value;
		while (ss >> value) {
			row.push_back(value);
		}

		m_matrix.push_back(row);
	}

	int nRow = m_matrix.size();
	int nColumn = m_matrix[0].size();

	std::vector<Eigen::Vector3d> points, normals;

	for (int i = 0; i < nRow; i++) {
		Eigen::Vector3d vec;
		vec << m_matrix[i][0], -m_matrix[i][2], m_matrix[i][1] + zAdjust;

		if (i > 0) {
			if ((vec - points.back()).norm() < 0.0001) {
				continue;
			}
		}

		//std::cout<<vec<<std::endl;
		points.push_back(vec);

		Eigen::Vector3d vec2;
		vec2 << -m_matrix[i][3], m_matrix[i][5], -m_matrix[i][4];
		normals.push_back(vec2);
	}

	path->setPath(points, normals);



	

	//QMeshPatch* waypoint = new QMeshPatch;
	//waypoint->SetIndexNo(Paths->GetMeshList().GetCount()); //index begin from 0
	//Paths->GetMeshList().AddTail(waypoint);
	//waypoint->patchName = "Path_" + layerIndex;

	//waypoint->inputPosNorFile(filename);
	//这里需要修改
}



void fileIO::readRobotData(PolygenMesh* Rob_model, robSystem* robs, std::string FileDir) {

	std::vector<std::string> RobfileSet;
	RobfileSet = { "BASE", "LINK1", "LINK2", "LINK3", "LINK4", "LINK5", "LINK6" };

	//read ROB files and build mesh_patches
	char filename[1024];

	for (int i = 0; i < RobfileSet.size(); i++) {
		sprintf(filename, "%s%s%s%s", FileDir.c_str(), "IRB2600_20kg-165_", RobfileSet[i].c_str(), "_CAD.obj");
		//std::cout << "input " << RobfileSet[i].data() << " from: " << filename << std::endl;

		QMeshPatch* robPatch = new QMeshPatch;
		robPatch->SetIndexNo(Rob_model->GetMeshList().GetCount()); //index begin from 0
		Rob_model->GetMeshList().AddTail(robPatch);
		robPatch->inputOBJFile(filename);
		robPatch->patchName = RobfileSet[i].data();

		robs->fclReadRobot(filename, i);
	}
}

void fileIO::readRobotData(robSystem* robs, std::string FileDir) {

	std::vector<std::string> RobfileSet;
	RobfileSet = { "BASE", "LINK1", "LINK2", "LINK3", "LINK4", "LINK5", "LINK6" };

	//read ROB files and build mesh_patches
	char filename[1024];

	for (int i = 0; i < RobfileSet.size(); i++) {
		sprintf(filename, "%s%s%s%s", FileDir.c_str(), "IRB2600_20kg-165_", RobfileSet[i].c_str(), "_CAD.obj");
		//std::cout << "input " << RobfileSet[i].data() << " from: " << filename << std::endl;

		robs->fclReadRobot(filename, i);
	}
}

void fileIO::readPosData(PolygenMesh* cnc_Prt, robSystem* robs, std::string FileDir) {

	std::vector<std::string> CNCfileSet;
	CNCfileSet = { "B1", "B2", "base1", "base2", "C"};

	//read CNC files and build mesh_patches
	char filename[1024];

	for (int i = 0; i < CNCfileSet.size(); i++) {
		sprintf(filename, "%s%s%s%s", FileDir.c_str(), "a250_", CNCfileSet[i].c_str(), ".obj");
		//std::cout << "input " << CNCfileSet[i].data() << " from: " << filename << std::endl;

		QMeshPatch* cncPatch = new QMeshPatch;
		cncPatch->SetIndexNo(cnc_Prt->GetMeshList().GetCount()); //index begin from 0
		cnc_Prt->GetMeshList().AddTail(cncPatch);
		cncPatch->inputOBJFile(filename);
		cncPatch->patchName = CNCfileSet[i].data();

		robs->fclReadPOS(filename, i);
	}

}

void fileIO::readPosData(robSystem* robs, std::string FileDir) {

	std::vector<std::string> CNCfileSet;
	CNCfileSet = { "B1", "B2", "base1", "base2", "C" };

	//read CNC files and build mesh_patches
	char filename[1024];

	for (int i = 0; i < CNCfileSet.size(); i++) {
		sprintf(filename, "%s%s%s%s", FileDir.c_str(), "a250_", CNCfileSet[i].c_str(), ".obj");
		//std::cout << "input " << CNCfileSet[i].data() << " from: " << filename << std::endl;

		robs->fclReadPOS(filename, i);
	}

}

void fileIO::readLayerData(PolygenMesh* Layers, robSystem* robs, PQP_Model* layer_PQP, std::string FileDir, std::string layerIndex) {
	char filename[1024];

	sprintf(filename, "%s%s%s", FileDir.c_str(), layerIndex.c_str(), ".obj");

	QMeshPatch* layer = new QMeshPatch;
	layer->SetIndexNo(Layers->GetMeshList().GetCount());
	Layers->GetMeshList().AddTail(layer);
	layer->inputOBJFile(filename);

	std::string name = "layer_" + layerIndex;
	layer->patchName = name;

	robs->changeLayerFrame(layer);

	robs->fclReadLayer(filename);

	layer_PQP->BeginModel(); int index = 0;
	PQP_REAL p1[3], p2[3], p3[3];
	for (GLKPOSITION Pos = layer->GetFaceList().GetHeadPosition(); Pos;) {
		QMeshFace* Face = (QMeshFace*)layer->GetFaceList().GetNext(Pos);
		
		Face->CalPlaneEquation();


		Face->GetNodeRecordPtr(0)->GetCoord3D(p1[0], p1[1], p1[2]);
		Face->GetNodeRecordPtr(1)->GetCoord3D(p2[0], p2[1], p2[2]);
		Face->GetNodeRecordPtr(2)->GetCoord3D(p3[0], p3[1], p3[2]);

		layer_PQP->AddTri(p1, p2, p3, index);
		index++;
	}
	layer_PQP->EndModel();

}

void fileIO::readLayerData(robSystem* robs, std::string FileDir, std::string layerIndex) {
	char filename[1024];

	sprintf(filename, "%s%s%s", FileDir.c_str(), layerIndex.c_str(), ".obj");
	robs->fclReadLayer(filename);
}


void fileIO::writeVar8(std::vector<Eigen::Matrix<double, 8, 1>> var, std::string Dir) {
	std::ofstream out(Dir);

	if (out.is_open()) {
		for (int i = 0; i < var.size(); i++) {
			for (int j = 0; j < var[i].size(); j++) {
				out << var[i](j) << " ";
			}
			out << std::endl;
		}
	}
	else {
		std::cout << "Fail to open file" << std::endl;
	}
}

void fileIO::writeVar8(std::vector<Eigen::VectorXd> var, std::string Dir) {
	std::ofstream out(Dir);

	if (out.is_open()) {
		for (int i = 0; i < var.size(); i++) {
			for (int j = 0; j < var[i].size(); j++) {
				out << var[i](j) << " ";
			}
			out << std::endl;
		}
	}
	else {
		std::cout << "Fail to open file" << std::endl;
	}
}

void fileIO::writeBaseColi(std::vector<Eigen::VectorXd> joints, std::vector<double> Coli, std::string Dir) {
	std::ofstream file(Dir);

	if (joints.size() != Coli.size()) {
		std::cout << "joints and coli do not match!" << std::endl;
		return;
	}

	if (file.is_open()) {
		for (int i = 0; i < joints.size(); i++) {
			file << Coli[i];
			for (int j = 1; j < joints[i].size() + 1; j++) {
				file << " " << j << ":" << joints[i](j - 1);
			}
			file << std::endl;
		}
		file.close();
		std::cout << "BaseColi writen!" << std::endl;
	}
	else {
		std::cout << "cannot open BaseColi" << std::endl;
	}
}


void fileIO::readJointsFromSVMData(std::vector<Eigen::VectorXd>& joints, std::vector<double>& labels, std::string Dir) {

	std::ifstream file(Dir);

	if (file.is_open()) {
		std::string line;
		while (getline(file, line)) {
			std::stringstream ss(line);
			std::string value;
			std::vector<double> jointVec;
			int i = 0;
			while (ss >> value) {
				if (i == 0) {
					labels.push_back(std::stod(value));
					i++;
					continue;
				}
				int pos = value.find(":");
				jointVec.push_back(std::stod(value.substr(pos + 1)));
				//joint(i - 1) = std::stod(value.substr(pos + 1));
				i++;
			}

			Eigen::VectorXd joint(jointVec.size());
			for (int i = 0; i < jointVec.size(); i++) {
				joint(i) = jointVec[i];
			}
			joints.push_back(joint);
		}
		file.close();
		std::cout << "joints read!" << std::endl;
	}
	else {
		std::cout << "cannot open file" << std::endl;
	}

}

void fileIO::writeTwoVectors(std::vector<double> v1, std::vector<double> v2, std::string Dir) {
	std::ofstream file(Dir);

	if (v1.size() != v2.size()) {
		std::cout << "vectors do not match!" << std::endl;
		return;
	}

	if (file.is_open()) {
		for (int i = 0; i < v1.size(); i++) {
			file << v1[i] << " " << v2[i] << std::endl;
		}
		file.close();
		std::cout << "vectors writen!" << std::endl;
	}
	else {
		std::cout << "cannot open file" << std::endl;
	}
}

void fileIO::writeColliData(colliDataGotten colli, std::string Dir) {
	std::ofstream file(Dir);

	//write colliDataGotten in a single file
	if (file.is_open()) {
		for (int i = 0; i < colli.coli.size(); i++) {
			file << colli.coli[i] << " ";

			for (int j = 0; j < colli.Joints[i].size(); j++) {
				file << colli.Joints[i](j) << " ";
			}


			for (int j = 0; j < colli.posJoints[i].size(); j++) {
				file << colli.posJoints[i](j) << " ";
			}
			
			file << std::endl;
		}
		file.close();
		std::cout << "colliDataGotten writen!" << std::endl;
	}
	else {
		std::cout << "cannot open file" << std::endl;
	}

}

void fileIO::readColliData(colliDataGotten& colli, std::string Dir) {
	//read colliDataGotten from a single file obtained by writeColliData

	std::ifstream file(Dir);

	std::vector<int> coli;
	std::vector<Eigen::VectorXd> Joints;
	std::vector<Eigen::VectorXd> posJoints;

	if (file.is_open()) {
		std::string line;
		while (getline(file, line)) {
			std::stringstream ss(line);
			std::string value;
			int i = 0;
			Eigen::VectorXd joint(8), posJoint;
			std::vector<double> posJointVec;

			while (ss >> value) {
				//std::cout << i << std::endl;
				if (i == 0) {
					//std::cout<<value<<std::endl;
					coli.push_back(std::stoi(value));
					i++;
					continue;
				}
				if (i < 9) {//the size of joint is 8
					joint(i - 1) = std::stod(value);
					i++;
					continue;
				}

				posJointVec.push_back(std::stod(value));
				i++;
			}

			posJoint.resize(posJointVec.size());
			for (int i = 0; i < posJointVec.size(); i++) {
				posJoint(i) = posJointVec[i];
			}

			Joints.push_back(joint);
			posJoints.push_back(posJoint);
		}
		file.close();
		std::cout << "colliDataGotten read!" << std::endl;
	}
	else {
		std::cout << "cannot open file" << std::endl;
	}

	colli.coli = coli;
	colli.Joints = Joints;
	colli.posJoints = posJoints;


}

void fileIO::writeVectorXd(Eigen::VectorXd v, std::string Dir) {
	std::ofstream file(Dir);

	if (file.is_open()) {
		for (int i = 0; i < v.size(); i++) {
			file << v(i) << " ";
		}
		file.close();
		std::cout << "vectorXd writen!" << std::endl;
	}
	else {
		std::cout << "cannot open file" << std::endl;
	}
}

void fileIO::readVectorXd(Eigen::VectorXd& v, std::string Dir) {
	std::ifstream file(Dir);

	std::vector<double> vec;

	if (file.is_open()) {
		std::string line;
		while (getline(file, line)) {
			std::stringstream ss(line);
			std::string value;
			int i = 0;
			while (ss >> value) {
				//v(i) = std::stod(value);
                vec.push_back(std::stod(value));
				i++;
			}
		}
		file.close();
		std::cout << "vectorXd read!" << std::endl;
	}
	else {
		std::cout << "cannot open file" << std::endl;
	}

	v.resize(vec.size());
    for (int i = 0; i < vec.size(); i++) {
		v(i) = vec[i];
	}
}

void fileIO::writeTransform3d(std::vector<Transform3d> T, std::string Dir) {
    std::ofstream file(Dir);

	if (file.is_open()) {
		for (int i = 0; i < T.size(); i++) {
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 4; k++) {
					file << T[i].matrix()(j,k) << " ";
				}
				file << std::endl;
			}
		}
		file.close();
		std::cout << "Transform3d writen!" << std::endl;
	}
		else {
		std::cout << "cannot open file" << std::endl;
	}
}