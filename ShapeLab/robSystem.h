#pragma once
#include "../QMeshLib/PolygenMesh.h"
#include "pathStruct.h"
#include "mathTools.h"

//#include <gsl/gsl_vector.h>
//#include <gsl/gsl_multiroots.h>

#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"
#include "fcl/math/bv/utility.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/shape/sphere.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision_result.h"
#include "fcl/narrowphase/continuous_collision.h"
#include "fcl/narrowphase/continuous_collision_object.h"
#include "fcl/narrowphase/continuous_collision_request.h"
#include "fcl/narrowphase/continuous_collision_result.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "test_fcl_utility.h"

using namespace mathTools;

class robSystem
{
public:
	robSystem(double setHPos, Eigen::Vector3d setPTool);
	~robSystem();

	//Pos
	Transform3d fkPos(Eigen::Vector2d BC);
	std::vector<Eigen::Vector2d> ikPosA(Eigen::Vector3d NWork, Eigen::Vector3d NWorld);
	void getBCXYZ(Eigen::Vector3d NWork, Eigen::Vector3d NWorld, 
		Eigen::Vector3d PWork, Eigen::Vector3d& PWorld, Eigen::Vector2d& BC, Eigen::Vector2d iniBC = Eigen::Vector2d::Zero());
	void setInitialPos(PolygenMesh* POS);
	std::vector<Eigen::Matrix4d> dfkPos(Eigen::Vector2d BC);

	Transform3d _FK_Pos(Eigen::Vector2d BC, bool flag);

	//Rob
	Transform3d fkABB2600(Eigen::Matrix<double, 6, 1> q, bool flag = false, int length = 6);
	void fkABB2600(Eigen::Matrix<double, 6, 1> q, std::vector<Transform3d>& T, int length = 6);
	Eigen::Matrix<double, 6, 1> ikABB2600(bool& flag, Transform3d T,
		Eigen::Matrix<double, 6, 1> tini = Eigen::Matrix<double, 6, 1>::Zero());
	Eigen::Matrix<double, 6, 6> JaB_ABB(Eigen::Matrix<double, 6, 1> t, Transform3d Ttcp = Transform3d::Identity());
	Eigen::Matrix<double, 6, 6> JaS_ABB(Eigen::Matrix<double, 6, 1> t, Transform3d Ttcp = Transform3d::Identity());
	Eigen::Matrix<double, 6, 6> JaS_ABB(Eigen::Matrix<double, 6, 1> t, std::vector<Eigen::Matrix<double, 6, 6>>& dJ, Transform3d Ttcp = Transform3d::Identity());
	void updateRobPos(PolygenMesh* Rob, Eigen::Matrix<double, 6, 1> q, bool isUpdate_lastCoord3D);

	void rotYFrame(Transform3d& TtoolWorld);
	Transform3d Ttool;

	//fcl
	void fclReadRobot(char* filename, int index);
	void fclReadPOS(char* filename, int index);
	void fclReadLayer(char* filename);

	void Trans_mesh(QMeshPatch* m_tetModel, Eigen::Transform<double, 3, Eigen::Isometry> T, bool isUpdate_lastCoord3D);

	bool checkBaseColi(Eigen::Matrix<double, 8, 1> q);
	bool checkLayerColi(Eigen::Matrix<double, 8, 1> q, int index);
	bool checkAllColi(Eigen::Matrix<double, 8, 1> q, int index = -1);

	void changeLayerFrame(QMeshPatch* layer);

	bool charLength = 467.73;//

	Eigen::VectorXd Joints2Postions(Eigen::VectorXd joints, bool isBase, Eigen::MatrixXd& posDjoint);//将关节角转换为坐标系位置
	Eigen::VectorXd Joints2Postions(Eigen::VectorXd joints, bool isBase);//将关节角转换为坐标系位置

	Eigen::VectorXd Joints2PostionsBC(Eigen::VectorXd joints, int type, Eigen::MatrixXd& posDjoint);//将关节角转换为坐标系位置(变位机坐标系下)
	Eigen::VectorXd Joints2PostionsBC(Eigen::VectorXd joints, int type);//将关节角转换为坐标系位置(变位机坐标系下)


	//fcl
	std::shared_ptr<fcl::BVHModel<fcl::AABBd>> fclABB[7], fclPOS[3];
	std::vector<std::shared_ptr<fcl::BVHModel<fcl::AABBd>>> fclLayer;

	Transform3d TransPos[4], TransPos0[4];
	Transform3d TransABB[7], TransABB0[7];

	void updateLayerPos(PolygenMesh* Layers, Eigen::Vector2d q, bool isUpdate_lastCoord3D);

private:
	//变位机标定结果
	Eigen::Matrix<double, 6, 1> x0Pos;
	Eigen::Matrix<double, 6, 2> xiPos;
	Eigen::Matrix4d gst0Pos;
	std::vector<Eigen::Matrix4d> xiHatPos;

	double hPos = 6.6 + 5 - 0.2 - 0.3;//adjust the z=0 plane

	//工具函数
	Transform3d mult(std::vector<Eigen::Matrix4d> xiAll, Eigen::VectorXd thetaAll);
	Transform3d _FK_eachlink(double t, int index);//fkABB所需函数
	Eigen::Matrix4d _dFK_eachlink(double t, int index);
	void _dFK_Pos(Eigen::Vector2d BC, std::vector<Eigen::Matrix4d>& T);

	

	

	void _FK_Pos(Eigen::Vector2d BC, std::vector<Transform3d>& T);
};