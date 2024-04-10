#include "robSystem.h"

robSystem::robSystem(double setHPos, Eigen::Vector3d setPTool) {
	hPos = setHPos;
	std::cout << "hPos=" << hPos << std::endl;

	//设置变位机标定结果
	x0Pos << 0.00193698086835952, 0.00180303855744732, 1.56691389478263, 860.337244091729, -870.673616644963, 607.360578045015;
	xiPos.col(0) << -0.000223818690577644, 0.999999973588456, -5.22329532248678e-5, -725.882305742486, -0.104829951831055, 1103.44283759528;
	xiPos.col(1) << 0.00248696781672084, -0.000120789176944063, 0.999996900195722, -8.99231368342199, -1101.77184358179, -0.110718862644663;
	//std::cout << "xiPos:\n" << xiPos << std::endl;

	Eigen::Matrix4d x0Hat = xi2xihat(x0Pos);
	gst0Pos = expm(x0Hat);
	gst0Pos.block<3, 1>(0, 3) = gst0Pos.block<3, 1>(0, 3) + gst0Pos.block<3, 1>(0, 2) * hPos;

	xiHatPos.resize(2);
	for (int i = 0; i < 2; i++) {
		xiHatPos[i]= xi2xihat(xiPos.col(i));
	}


	//设置刀具位姿
	Eigen::Quaterniond quatTool;
	quatTool.w() = 0.454812;
	quatTool.x() = -0.184782;
	quatTool.y() = 0.337459;
	quatTool.z() = -0.803195;
	quatTool.normalize();
	Eigen::Matrix3d RotTool;
	RotTool = quatTool.toRotationMatrix();
	//Eigen::Vector3d pTool(155.164, -91.1004, 184.935);
	//Eigen::Vector3d pTool(151.762, -88.265, 188.612);
	Eigen::Vector3d pTool = setPTool;
	std::cout << "pTool=" << pTool.transpose() << std::endl;

	Ttool = Transform3d::Identity();
	Ttool.linear() = RotTool;
	Ttool.translation() = pTool;

	
	//fcl初始化,Trans初始化
	//Rob
	TransABB[0] = Transform3d::Identity();
	Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
	this->fkABB2600(q, true);
	for (int i = 0; i < 7; i++) {
		fclABB[i] = std::make_shared<fcl::BVHModel<fcl::AABBd>>();
		TransABB0[i] = TransABB[i];
	}

	//Pos
	Transform3d Pose_base(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1)));
	Pose_base.translation() = Eigen::Vector3d(1101.77, -8.99, -294.12);
	TransPos[0] = Pose_base;
	Eigen::Vector2d BC = Eigen::Vector2d::Zero();

	this->_FK_Pos(BC, true);
	for (int i = 0; i < 3; i++) {
		fclPOS[i] = std::make_shared<fcl::BVHModel<fcl::AABBd>>();
		TransPos0[i] = TransPos[i];
	}
	TransPos0[3] = TransPos[3];

	//Layer(old version, not used)
	//fclLayer = std::make_shared<fcl::BVHModel<fcl::AABBd>>();

}

robSystem::~robSystem() {
	std::cout << "delete robSystem" << std::endl;
}


Transform3d robSystem::_FK_Pos(Eigen::Vector2d BC, bool flag) {

	Transform3d T1(Transform3d::Identity()), T2(Transform3d::Identity()), Ttmp;

	Eigen::Matrix3d R;
	Eigen::Vector3d v;

	R << -1, 0, 0, 0, 0, -1, 0, -1, 0;
	v << 0, 0, 1020;

	T1.linear() = R;
	T1.translation() = v;

	Ttmp = Eigen::AngleAxisd(BC(0), Eigen::Vector3d(0, 0, 1));

	T1.matrix() = T1.matrix() * Ttmp.matrix();

	R << 0, -1, 0, 0, 0, -1, 1, 0, 0;
	v << 0, 0, 0;

	T2.linear() = R;
	T2.translation() = v;

	Ttmp = Eigen::AngleAxisd(BC(1), Eigen::Vector3d(0, 0, 1));
	T2.matrix() = T2.matrix() * Ttmp.matrix();

	Transform3d Tmp1, Tmp2, Tmp3;

	Tmp1.matrix() = TransPos[0].matrix() * T1.matrix();
	Tmp2.matrix() = Tmp1.matrix() * T2.matrix();

	Ttmp = Transform3d::Identity();
	Ttmp.matrix()(0, 3) = 0.0419;
	Ttmp.matrix()(1, 3) = -1.51;
	Ttmp.matrix()(2, 3) = -119.811 + hPos;
	Tmp3.matrix() = Tmp2.matrix() * Ttmp.matrix();

	if (flag) {
		TransPos[1] = Tmp1;
		TransPos[2] = Tmp2;
		TransPos[3] = Tmp3;
	}

	return Tmp3;
}

void robSystem::_FK_Pos(Eigen::Vector2d BC, std::vector<Transform3d>& T) {

	T.resize(4);
	T[0]=TransPos0[0];

	Transform3d T1(Transform3d::Identity()), T2(Transform3d::Identity()), Ttmp;

	Eigen::Matrix3d R;
	Eigen::Vector3d v;

	R << -1, 0, 0, 0, 0, -1, 0, -1, 0;
	v << 0, 0, 1020;

	T1.linear() = R;
	T1.translation() = v;

	Ttmp = Eigen::AngleAxisd(BC(0), Eigen::Vector3d(0, 0, 1));

	T1.matrix() = T1.matrix() * Ttmp.matrix();

	R << 0, -1, 0, 0, 0, -1, 1, 0, 0;
	v << 0, 0, 0;

	T2.linear() = R;
	T2.translation() = v;

	Ttmp = Eigen::AngleAxisd(BC(1), Eigen::Vector3d(0, 0, 1));
	T2.matrix() = T2.matrix() * Ttmp.matrix();

	Transform3d Tmp1, Tmp2, Tmp3;

	Tmp1.matrix() = TransPos[0].matrix() * T1.matrix();
	Tmp2.matrix() = Tmp1.matrix() * T2.matrix();

	Ttmp = Transform3d::Identity();
	Ttmp.matrix()(0, 3) = 0.0419;
	Ttmp.matrix()(1, 3) = -1.51;
	Ttmp.matrix()(2, 3) = -119.811 + hPos;
	Tmp3.matrix() = Tmp2.matrix() * Ttmp.matrix();


	T[1] = Tmp1;
	T[2] = Tmp2;
	T[3] = Tmp3;

}

void robSystem::_dFK_Pos(Eigen::Vector2d BC, std::vector<Eigen::Matrix4d>& dT) {

	dT.resize(2);

	Transform3d T10(Transform3d::Identity()), T20(Transform3d::Identity()), Ttmp1, Ttmp2, T1, T2;

	Eigen::Matrix3d R;
	Eigen::Vector3d v;

	R << -1, 0, 0, 0, 0, -1, 0, -1, 0;
	v << 0, 0, 1020;

	T10.linear() = R;
	T10.translation() = v;

	Ttmp1 = Eigen::AngleAxisd(BC(0), Eigen::Vector3d(0, 0, 1));


	R << 0, -1, 0, 0, 0, -1, 1, 0, 0;
	v << 0, 0, 0;

	T20.linear() = R;
	T20.translation() = v;

	Ttmp2 = Eigen::AngleAxisd(BC(1), Eigen::Vector3d(0, 0, 1));

	Transform3d Ttmp = Transform3d::Identity();
	Ttmp.matrix()(0, 3) = 0.0419;
	Ttmp.matrix()(1, 3) = -1.51;
	Ttmp.matrix()(2, 3) = -119.811 + hPos;
	
	Eigen::Matrix4d dTmp1 = Eigen::Matrix4d::Zero(), dTmp2 = Eigen::Matrix4d::Zero();
	dTmp1.block<2, 2>(0, 0) << -sin(BC(0)), -cos(BC(0)), cos(BC(0)), -sin(BC(0));
	dTmp2.block<2, 2>(0, 0) << -sin(BC(1)), -cos(BC(1)), cos(BC(1)), -sin(BC(1));

	dT[0] = TransPos[0].matrix() * T10.matrix() * dTmp1 * T20.matrix() * Ttmp2.matrix() * Ttmp.matrix();
	dT[1] = TransPos[0].matrix() * T10.matrix() * Ttmp1.matrix() * T20.matrix() * dTmp2 * Ttmp.matrix();
}


Transform3d robSystem::fkPos(Eigen::Vector2d BC) {

	Transform3d T;
	T = mult(xiHatPos, BC);
	T.matrix() = T.matrix() * gst0Pos;

	return T;

}

Transform3d robSystem::mult(std::vector<Eigen::Matrix4d> xiAll, Eigen::VectorXd thetaAll) {
	Transform3d T = Transform3d::Identity();
	if (xiAll.size() != thetaAll.size()) {
		std::cerr << "Input of mult error" << std::endl;
		return T;
	}
	for (int i = 0; i < xiAll.size(); i++) {
		Eigen::Matrix4d Tmp = xiAll[i] * thetaAll(i);
		T.matrix() = T.matrix() * expm(Tmp);
	}
	return T;
}


std::vector<Eigen::Matrix4d> robSystem::dfkPos(Eigen::Vector2d BC) {
	std::vector<Eigen::Matrix4d> dT(2);

	Eigen::Matrix4d E1, E2, dE1, dE2;

	Eigen::Matrix4d Tmp;

	Tmp = xiHatPos[0] * BC(0);
	E1 = expm(Tmp);
	dE1 = dexpm(Tmp);

	Tmp = xiHatPos[1] * BC(1);
	E2 = expm(Tmp);
	dE2 = dexpm(Tmp);

	if (BC(0) < 0) {
		dE1= -dE1;
	}
	if (BC(1) < 0) {
		dE2 = -dE2;
	}

	dT[0] = dE1 * E2 * gst0Pos;
	dT[1] = E1 * dE2 * gst0Pos;

	return dT;

}


std::vector<Eigen::Vector2d> robSystem::ikPosA(Eigen::Vector3d NWork, Eigen::Vector3d NWorld) {
	NWork.normalize();
	NWorld.normalize();
	//std::cout << "gst0Pos:\n" << this->gst0Pos << std::endl;
	Eigen::Vector3d u = gst0Pos.block<3, 3>(0, 0) * NWork;
	Eigen::Vector3d v = NWorld;

	Eigen::Vector3d w1 = xiPos.col(0).block<3, 1>(0, 0);
	Eigen::Vector3d w2 = xiPos.col(1).block<3, 1>(0, 0);


	double k1, k2;
	k1 = (w1.dot(w2) * w2.dot(u) - w1.dot(v)) / (w1.dot(w2) * w1.dot(w2) - 1);
	k2 = (w1.dot(w2) * w1.dot(v) - w2.dot(u)) / (w1.dot(w2) * w1.dot(w2) - 1);
	std::vector<double> k3(2);

	double ktmp = (1 - k1 * k1 - k2 * k2 - 2 * k1 * k2 * w1.dot(w2));
	if (ktmp < 0) {
		if (ktmp > -1e-5) {
			ktmp = 0;
		}
		else {
			std::cout<<"ktmp="<<ktmp<<std::endl;
			std::cerr << "error in ikPosA" << std::endl;
		}
	}

	k3[0] = sqrt(ktmp / pow(w1.cross(w2).norm(), 2));
	k3[1] = -k3[0];


	std::vector<Eigen::Vector2d> BC(2);
	
	for (int i = 0; i < 2; i++) {
		Eigen::Vector3d z, z1p, z2p, vp, up;
		z = k1 * w1 + k2 * w2 + k3[i] * w1.cross(w2);
		z1p = z - w1 * w1.dot(z);
		vp = v - w1 * w1.dot(v);
		z2p = z - w2 * w2.dot(z);
		up = u - w2 * w2.dot(u);

		Eigen::Vector2d tmp;
		tmp(0) = atan2(w1.dot(z1p.cross(vp)), z1p.dot(vp));
		tmp(1) = atan2(w2.dot(up.cross(z2p)), up.dot(z2p));

		BC[i] = tmp;

		Eigen::Vector3d test;
		test = fkPos(tmp).linear() * NWork - NWorld;

		//std::cout << "BC:\n" << tmp << std::endl;
		//std::cout << "solve ik pos:\n" << test << std::endl;

		if (test.norm() > 1e-5) {
			std::cout << test.norm();
			std::cerr << " error in ikPosA" << std::endl;
		}
	}

	return BC;
}

void robSystem::getBCXYZ(Eigen::Vector3d NWork, Eigen::Vector3d NWorld, Eigen::Vector3d PWork, Eigen::Vector3d& PWorld, Eigen::Vector2d& BC, Eigen::Vector2d iniBC) {
	
	if (iniBC(0) > M_PI || iniBC(0) < -M_PI) {
		std::cerr << "iniBC wrong!" << std::endl;
	}

	if (NWorld(2) > 1 - 1e-5 && NWork(2)>1-1e-5) {
		//std::cout << "singularity" << std::endl;
		BC(0) = 0;
		BC(1)=iniBC(1);
	}
	else {
		std::vector<Eigen::Vector2d> vecBC = ikPosA(NWork, NWorld);

		while (abs(vecBC[0](1) - iniBC(1)) > M_PI) {
			if (vecBC[0](1) - iniBC(1) > 0) {
				vecBC[0](1) -= 2 * M_PI;
			}
			else {
				vecBC[0](1) += 2 * M_PI;
			}
		}

		while (abs(vecBC[1](1) - iniBC(1)) > M_PI) {
			if (vecBC[1](1) - iniBC(1) > 0) {
				vecBC[1](1) -= 2 * M_PI;
			}
			else {
				vecBC[1](1) += 2 * M_PI;
			}
		}


		//if (abs(vecBC[0](0) - iniBC(0)) < abs(vecBC[1](0) - iniBC(0))) {
		//	BC = vecBC[0];
		//}
		//else {
		//	BC = vecBC[1];
		//}

		if ((vecBC[0] - iniBC).norm() < (vecBC[1] - iniBC).norm()) {
			BC = vecBC[0];
		}
		else {
			BC = vecBC[1];
		}

	}
	

	while (abs(BC(1) - iniBC(1)) > M_PI) {
		if (BC(1) - iniBC(1) > 0) {
			BC(1) -= 2 * M_PI;
		}
		else {
			BC(1) += 2 * M_PI;
		}	
	}

	Transform3d T = fkPos(BC);
	Eigen::Vector4d xyz = Eigen::Vector4d::Ones();
	xyz.block<3, 1>(0, 0) = PWork;
	xyz = T.matrix() * xyz;

	PWorld = xyz.block<3, 1>(0, 0);
}


Transform3d robSystem::fkABB2600(Eigen::Matrix<double, 6, 1> q, bool flag, int length) {
	Transform3d T1(Transform3d::Identity());

	Transform3d Ttmp;

	for (int i = 1; i < length + 1; i++) {
		Ttmp = _FK_eachlink(q(i - 1), i);

		T1.matrix() = T1.matrix() * Ttmp.matrix();

		if (flag) {
			TransABB[i] = T1;
		}
	}
	return T1;
}

void robSystem::fkABB2600(Eigen::Matrix<double, 6, 1> q, std::vector<Transform3d>& T, int length) {
	Transform3d T1(Transform3d::Identity());

	Transform3d Ttmp;

	T.resize(length + 1);
	T[0]=TransABB0[0];

	for (int i = 1; i < length + 1; i++) {
		Ttmp = _FK_eachlink(q(i - 1), i);

		T1.matrix() = T1.matrix() * Ttmp.matrix();

		T[i] = T1;
	}
}

Transform3d robSystem::_FK_eachlink(double t, int index) {
	Transform3d T(Transform3d::Identity());
	Eigen::Matrix3d R;
	Eigen::Vector3d tt;
	//std::cout << "index=" << index << std::endl;
	switch (index) {
	case 1:
		R << cos(t), 0, -sin(t),
			sin(t), 0, cos(t),
			0, -1, 0;
		tt << 150 * cos(t), 150 * sin(t), 445;
		break;
	case 2:
		t = t - M_PI / 2;
		R << cos(t), -sin(t), 0,
			sin(t), cos(t), 0,
			0, 0, 1;
		tt << 700 * cos(t), 700 * sin(t), 0;
		break;
	case 3:
		R << cos(t), 0, -sin(t),
			sin(t), 0, cos(t),
			0, -1, 0;
		tt << 115 * cos(t), 115 * sin(t), 0;
		break;
	case 4:
		R << cos(t), 0, sin(t),
			sin(t), 0, -cos(t),
			0, 1, 0;
		tt << 0, 0, 795;
		break;
	case 5:
		t = t + M_PI;
		R << cos(t), 0, sin(t),
			sin(t), 0, -cos(t),
			0, 1, 0;
		tt << 0, 0, 0;
		break;
	case 6:
		R << cos(t), -sin(t), 0,
			sin(t), cos(t), 0,
			0, 0, 1;
		tt << 0, 0, 85;
		break;
	default:
		T = Transform3d::Identity();
		std::cerr << "incorrect index for robot link" << std::endl;
		return T;
	}

	T.linear() = R;
	T.translation() = tt;
	return T;
}

Eigen::Matrix4d robSystem::_dFK_eachlink(double t, int index) {
	Eigen::Matrix4d dT = Eigen::Matrix4d::Zero();
	Eigen::Matrix3d R;
	Eigen::Vector3d tt;
	//std::cout << "index=" << index << std::endl;
	switch (index) {
	case 1:
		R << -sin(t), 0, -cos(t),
			cos(t), 0, -sin(t),
			0, 0, 0;
		tt << -150 * sin(t), 150 * cos(t), 0;
		break;
	case 2:
		t = t - M_PI / 2;
		R << -sin(t), -cos(t), 0,
			cos(t), -sin(t), 0,
			0, 0, 0;
		tt << -700 * sin(t), 700 * cos(t), 0;
		break;
	case 3:
		R << -sin(t), 0, -cos(t),
			cos(t), 0, -sin(t),
			0, 0, 0;
		tt << -115 * sin(t), 115 * cos(t), 0;
		break;
	case 4:
		R << -sin(t), 0, cos(t),
			cos(t), 0, sin(t),
			0, 0, 0;
		tt << 0, 0, 0;
		break;
	case 5:
		t = t + M_PI;
		R << -sin(t), 0, cos(t),
			cos(t), 0, sin(t),
			0, 0, 0;
		tt << 0, 0, 0;
		break;
	case 6:
		R << -sin(t), -cos(t), 0,
			cos(t), -sin(t), 0,
			0, 0, 0;
		tt << 0, 0, 0;
		break;
	default:
		std::cerr << "incorrect index for robot link" << std::endl;
		return dT;
	}

	dT.block<3, 3>(0, 0) = R;
	dT.block<3, 1>(0, 3) = tt;
	return dT;
}



Eigen::Matrix<double, 6, 1> robSystem::ikABB2600(bool& flag, Transform3d T, Eigen::Matrix<double, 6, 1> tini) {

	Eigen::Matrix<double, 6, 6> J;
	double error = 0.00001;
	Eigen::Matrix<double, 6, 1> t, e = Eigen::Matrix<double, 6, 1>::Zero();
	t = tini;
	Transform3d T1;
	T1 = fkABB2600(t, false);
	T1 = T1.inverse();
	Eigen::Matrix4d tmp, eh;
	tmp = T1.matrix() * T.matrix();

	if ((tmp.block<3,3>(0,0) - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 0.00001) {
		tmp.block<3, 3>(0, 0) =Eigen::Matrix3d::Identity();
		//std::cout<<"tmp=I"<<std::endl;	
	}
	
	eh = logm(tmp);
	e.block<3, 1>(0, 0) = eh.block<3, 1>(0, 3);
	e(3) = -eh(1, 2);
	e(4) = eh(0, 2);
	e(5) = -eh(0, 1);

	//std::cout << tmp << std::endl;
	//std::cout<<eh<<std::endl;

	flag = true;
	int i = 0;

	while (e.block<3, 1>(0, 0).norm() > error || e.block<3, 1>(3, 0).norm() > error) {
		if (i > 500) {
			flag = false;
			break;
		}

		i++;

		J = JaB_ABB(t);
		J = J.completeOrthogonalDecomposition().pseudoInverse();
		t = t + J * e;

		T1 = fkABB2600(t, false);
		T1 = T1.inverse();

		tmp = T1.matrix() * T.matrix();

		if ((tmp.block<3, 3>(0, 0) - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 0.00001) {
			tmp.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
			//std::cout << "tmp=I" << std::endl;
		}

		eh = logm(tmp);
		e.block<3, 1>(0, 0) = eh.block<3, 1>(0, 3);
		e(3) = -eh(1, 2);
		e(4) = eh(0, 2);
		e(5) = -eh(0, 1);
	}


	if (tini.sum() < 0.001) {
		for (i = 0; i < 6; i++) {
			t(i) = fmod(t(i) + M_PI, 2 * M_PI) - M_PI;
		}
	}

	if (t(1) < -M_PI / 2 || t(1) > 2.618) {
		flag = false;
	}
	else if (t(2) > 1.309 || t(2) < -M_PI) {
		flag = false;
	}
	else if (t(4) < -M_PI / 2 || t(4) > M_PI / 2) {
		flag = false;
	}


	for (i = 3; i < 6; i += 2) {

		while (abs(t(i) - tini(i)) > M_PI) {
			if (t(i) - tini(i) > 0) {
				t(i) -= 2 * M_PI;
			}
			else {
				t(i) += 2 * M_PI;
			}
		}

		while (t(i) > 6.9813) {
			t(i) -= 2 * M_PI;
		}

		while (t(i) < -6.9813) {
			t(i) += 2 * M_PI;
		}
	}
	
	if (!flag) {
		t = 100 * Eigen::Matrix<double, 6, 1>::Ones();
	}
	else {
		Eigen::Matrix4d tmp = fkABB2600(t).matrix() - T.matrix();
		if (tmp.maxCoeff() > 0.01) {
			std::cerr << "something error in ik" << std::endl;
			flag = false;
			t = 100 * Eigen::Matrix<double, 6, 1>::Ones();
		}
	}
	


	return t;
}

Eigen::Matrix<double, 6, 6> robSystem::JaB_ABB(Eigen::Matrix<double, 6, 1> t, Transform3d Ttcp) {
	Eigen::Matrix<double, 6, 6> J, Tad = Eigen::Matrix<double, 6, 6>::Zero();
	Transform3d T;
	J = JaS_ABB(t, Ttcp);
	T = fkABB2600(t, false);
	T.matrix() = T.matrix() * Ttcp.matrix();
	T = T.inverse();

	Tad.block<3, 3>(0, 0) = T.matrix().block<3, 3>(0, 0);
	Tad.block<3, 3>(3, 3) = T.matrix().block<3, 3>(0, 0);

	J = Tad * J;

	return J;

}

Eigen::Matrix<double, 6, 6> robSystem::JaS_ABB(Eigen::Matrix<double, 6, 1> t, Transform3d Ttcp) {
	Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
	Transform3d T;
	Eigen::Vector3d pe, z, z1, p;
	z << 0, 0, 1;
	z1 = z;
	T = fkABB2600(t, false);
	T.matrix() = T.matrix() * Ttcp.matrix();
	pe = T.matrix().block<3, 1>(0, 3);

	J.block<3, 1>(0, 0) = z.cross(pe);
	J.block<3, 1>(3, 0) = z;

	for (int i = 2; i < 7; i++) {
		T = fkABB2600(t, false, i - 1);
		z = T.matrix().block<3, 3>(0, 0) * z1;
		p = T.matrix().block<3, 1>(0, 3);
		J.block<3, 1>(0, i - 1) = z.cross(pe - p);
		J.block<3, 1>(3, i - 1) = z;
	}
	return J;
}

Eigen::Matrix<double, 6, 6> robSystem::JaS_ABB(Eigen::Matrix<double, 6, 1> t, std::vector<Eigen::Matrix<double, 6, 6>>& dJ, Transform3d Ttcp) {
	Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
	Transform3d T;
	Eigen::Vector3d pe, z, z1, p;
	z << 0, 0, 1;
	z1 = z;
	T = fkABB2600(t, false);
	T.matrix() = T.matrix() * Ttcp.matrix();
	pe = T.matrix().block<3, 1>(0, 3);

	J.block<3, 1>(0, 0) = z.cross(pe);
	J.block<3, 1>(3, 0) = z;

	dJ.resize(6);

	Eigen::Matrix<double, 3, 6> e;
	e.setZero();
	e(2, 0) = 1;
	Eigen::Matrix<double, 3, 6> P;
	P.setZero();


	for (int i = 2; i < 7; i++) {
		T = fkABB2600(t, false, i - 1);
		z = T.matrix().block<3, 3>(0, 0) * z1;
		p = T.matrix().block<3, 1>(0, 3);
		J.block<3, 1>(0, i - 1) = z.cross(pe - p);
		J.block<3, 1>(3, i - 1) = z;

		e.block<3, 1>(0, i - 1) = z;
		P.block<3, 1>(0, i - 1) = p;
	}

	std::vector<Eigen::Matrix<double, 3, 6>> de(6);
	for (int i = 0; i < 6; i++) {
		de[i].setZero();
		if (i < 5) {
			for (int j = i + 1; j < 6; j++) {
				Eigen::Vector3d e1 = e.block<3, 1>(0, i), e2 = e.block<3, 1>(0, j);
				Eigen::Vector3d tmp = e1.cross(e2);
				de[i].block<3, 1>(0, j) = tmp;
			}
		}
	}

	for (int i = 0; i < 6; i++) {
		dJ[i].setZero();
		dJ[i].block<3, 6>(3, 0) = de[i];
	}
	
	std::vector<Eigen::Matrix<double, 3, 6>> dv(6);

	for (int i = 0; i < 6; i++) {
		dv[i].setZero();
		for (int j = 0; j < 6; j++) {
			Eigen::Vector3d dr;
			if (j <= i) {
				Eigen::Vector3d e1 = e.block<3, 1>(0, i);
				Eigen::Vector3d e2 = pe - P.block<3, 1>(0, i);
				dr = e1.cross(e2);
			}
			else {
				Eigen::Vector3d e1 = e.block<3, 1>(0, i);
				Eigen::Vector3d e2 = pe - P.block<3, 1>(0, j);
				dr = e1.cross(e2);
			}

			Eigen::Vector3d e1 = de[i].block<3, 1>(0, j);
			Eigen::Vector3d e2 = pe - P.block<3, 1>(0, j);
			Eigen::Vector3d e3 = e.block<3, 1>(0, j);
			dv[i].block<3, 1>(0, j) = e1.cross(e2) + e3.cross(dr);
		}
	}

	for (int i = 0; i < 6; i++) {
		dJ[i].block<3, 6>(0, 0) = dv[i];
	}

	return J;
}




void robSystem::rotYFrame(Transform3d& TtoolWorld) {
	//将TtoolWorld坐标系绕y轴旋转180度，以便与Ttool相匹配
	//TtoolWorld中z轴为曲面外法向，而Ttool中z轴指向刀具以外

	Transform3d RotY(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
	TtoolWorld.matrix() = TtoolWorld.matrix() * RotY.matrix();

}


void robSystem::fclReadRobot(char* filename, int index) {
	std::vector<fcl::Vector3<double>> p1;
	std::vector<fcl::Triangle> t1;
	fcl::test_FCL::loadOBJFile(filename, p1, t1);

	fclABB[index]->beginModel();
	fclABB[index]->addSubModel(p1, t1);
	fclABB[index]->endModel();

	//std::cout << "contruct fclABB_" << index << std::endl;
}

void robSystem::fclReadPOS(char* filename, int index) {

	if (index > 4)
		return;

	int tmp = 10;

	std::vector<fcl::Vector3<double>> p1;
	std::vector<fcl::Triangle> t1;
	fcl::test_FCL::loadOBJFile(filename, p1, t1);

	//std::cout << "read file" << std::endl;

	switch (index) {
	case 0:
		tmp = 1;
		fclPOS[tmp]->beginModel();
		fclPOS[tmp]->addSubModel(p1, t1);
		break;
	case 1:
		tmp = 1;
		fclPOS[tmp]->addSubModel(p1, t1);
		fclPOS[tmp]->endModel();
		break;
	case 2:
		tmp = 0;
		fclPOS[tmp]->beginModel();
		fclPOS[tmp]->addSubModel(p1, t1);
		break;
	case 3:
		tmp = 0;
		fclPOS[tmp]->addSubModel(p1, t1);
		fclPOS[tmp]->endModel();
		break;
	case 4:
		tmp = 2;
		fclPOS[tmp]->beginModel();
		fclPOS[tmp]->addSubModel(p1, t1);
		fclPOS[tmp]->endModel();
		break;
	default:
		std::cerr << "something wrong in fclReadPOS" << std::endl;
	}

	//std::cout << "update int:" << tmp << std::endl;

	//std::cout << "contruct fclPOS_" << tmp << std::endl;
}

void robSystem::fclReadLayer(char* filename) {
	std::vector<fcl::Vector3<double>> p1;
	std::vector<fcl::Triangle> t1;
	fcl::test_FCL::loadOBJFile(filename, p1, t1);

	std::shared_ptr<fcl::BVHModel<fcl::AABBd>> tmp;
	tmp = std::make_shared<fcl::BVHModel<fcl::AABBd>>();
	tmp->beginModel();
	tmp->addSubModel(p1, t1);
	tmp->endModel();

	fclLayer.push_back(tmp);



	std::cout << "contruct fclLayer: " << fclLayer.size() - 1 << std::endl;

}

void robSystem::setInitialPos(PolygenMesh* POS) {

	Transform3d T = this->TransPos0[0];

	for (GLKPOSITION pos = POS->GetMeshList().GetHeadPosition(); pos;) {
		QMeshPatch* link = (QMeshPatch*)POS->GetMeshList().GetNext(pos);
		this->Trans_mesh(link, T, true);
	}
}

void robSystem::Trans_mesh(QMeshPatch* m_tetModel, Eigen::Transform<double, 3, Eigen::Isometry> T, bool isUpdate_lastCoord3D) {
	//m_tetModel->model_rotMat = T.linear();

	Eigen::Vector3d t;
	t = T.translation();

	for (GLKPOSITION posMesh = m_tetModel->GetNodeList().GetHeadPosition(); posMesh != nullptr;) {
		QMeshNode* node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(posMesh);

		Eigen::Vector3d pp; node->GetCoord3D_last(pp(0), pp(1), pp(2));
		Eigen::Vector3d rotatedpp = T.linear() * pp;

		node->SetCoord3D(rotatedpp[0], rotatedpp[1], rotatedpp[2]);
	}

	//std::cout << "\nFinish rotate model." << std::endl;

	//move model
	double pp[3];
	for (GLKPOSITION Pos = m_tetModel->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode* Node = (QMeshNode*)m_tetModel->GetNodeList().GetNext(Pos);
		Node->GetCoord3D(pp[0], pp[1], pp[2]);

		pp[0] += t(0); pp[1] += t(1); pp[2] += t(2);
		Node->SetCoord3D(pp[0], pp[1], pp[2]);
		if (isUpdate_lastCoord3D)   Node->SetCoord3D_last(pp[0], pp[1], pp[2]);
	}
	//std::cout << "\nFinish move model.\n" << std::endl;
}

bool robSystem::checkBaseColi(Eigen::Matrix<double, 8, 1> q) {
	std::vector<Transform3d> TransABBTmp(7), TransPosTmp(4);
	this->fkABB2600(q.block<6, 1>(0, 0), TransABBTmp);
	this->_FK_Pos(q.block<2, 1>(6, 0), TransPosTmp);

	bool flag = false;

	/*time_t start, end;
	double time;

	start = clock();*/


	fcl::BroadPhaseCollisionManagerd* manager1=new fcl::DynamicAABBTreeCollisionManagerd();
	fcl::BroadPhaseCollisionManagerd* manager2=new fcl::DynamicAABBTreeCollisionManagerd();
	//std::vector<fcl::CollisionObjectd*> robot1, robot2;

	for (int i = 6; i > 1; i--) {
		Transform3d T1(Transform3d::Identity());
		T1.matrix() = TransABBTmp[i].matrix() * TransABB0[i].matrix().inverse();
		manager1->registerObject(new fcl::CollisionObjectd(this->fclABB[i], T1));
	}

	for (int j = 2; j > 0; j--) {
		Transform3d T2(Transform3d::Identity());
		T2.matrix() = TransPosTmp[j].matrix() * TransPos0[j].matrix().inverse() * TransPosTmp[0].matrix();
		manager2->registerObject(new fcl::CollisionObjectd(this->fclPOS[j], T2));
	}

	manager1->setup();
	manager2->setup();

	fcl::DefaultCollisionData<double> collision_data;

	manager1->collide(manager2, &collision_data, fcl::DefaultCollisionFunction);

	if (collision_data.result.isCollision()) {
		flag = true;
	}

	/*end = clock();
	time=(double)(end - start) / CLOCKS_PER_SEC;
	
	std::cout<<"\n"<<collision_data.result.isCollision()<<"  Time: "<<time << std::endl;*/


	//以下代码效率太低
	// 
	//start = clock();
	// 
	//for (int i = 6; i > 1; i--) {//不考虑第一、二个关节对应的碰撞（几乎不可能）
	//	Transform3d T1(Transform3d::Identity());
	//	T1.matrix() = TransABBTmp[i].matrix() * TransABB0[i].matrix().inverse();
	//	fcl::CollisionObjectd* obj1 = new fcl::CollisionObjectd(this->fclABB[i], T1);
	//
	//	//以下检测机器人与layer的碰撞
	//	//fcl::CollisionObjectd* obj2 = new fcl::CollisionObjectd(this->fclLayer, TransPosTmp[3]);
	//	//fcl::CollisionRequestd request;
	//	//fcl::CollisionResultd result;
	//	//fcl::collide(obj1, obj2, request, result);
	//	//if (result.isCollision()) {
	//	//	flag = true;
	//	//	return flag;
	//	//}
	//
	//	for (int j = 2; j > 0; j--) {//不考虑第一关节对应的碰撞（几乎不可能）
	//		Transform3d T2(Transform3d::Identity());
	//		T2.matrix() = TransPosTmp[j].matrix() * TransPos0[j].matrix().inverse() * TransPosTmp[0].matrix();
	//		fcl::CollisionObjectd* obj2 = new fcl::CollisionObjectd(this->fclPOS[j], T2);
	//
	//		fcl::CollisionRequestd request;
	//		fcl::CollisionResultd result;
	//		fcl::collide(obj1, obj2, request, result);
	//
	//		if (result.isCollision()) {
	//			end = clock();
	//			time = (double)(end - start) / CLOCKS_PER_SEC;
	//			std::cout << "\n" << result.isCollision() << "  Time: " << time << std::endl;
//
	//			
	//			flag = true;
	//			return flag;
	//		}
	//	}
	//
	//}
	//
	//end = clock();
	//time = (double)(end - start) / CLOCKS_PER_SEC;
	//std::cout << "\n" << flag << "  Time: " << time << std::endl;

	return flag;
}

bool robSystem::checkLayerColi(Eigen::Matrix<double, 8, 1> q, int index) {
	std::vector<Transform3d> TransABBTmp(7);
	Transform3d TransPosTmp;
	this->fkABB2600(q.block<6, 1>(0, 0), TransABBTmp);
	TransPosTmp = this->fkPos(q.block<2, 1>(6, 0));

	bool flag = false;

	fcl::BroadPhaseCollisionManagerd* manager1 = new fcl::DynamicAABBTreeCollisionManagerd();
	fcl::BroadPhaseCollisionManagerd* manager2 = new fcl::DynamicAABBTreeCollisionManagerd();

	/*for (int i = 6; i > 1; i--) {
		Transform3d T1(Transform3d::Identity());
		T1.matrix() = TransABBTmp[i].matrix() * TransABB0[i].matrix().inverse();
		manager1->registerObject(new fcl::CollisionObjectd(this->fclABB[i], T1));
	}*/

	Transform3d T1(Transform3d::Identity());
	T1.matrix() = TransABBTmp[6].matrix() * TransABB0[6].matrix().inverse();
	manager1->registerObject(new fcl::CollisionObjectd(this->fclABB[6], T1));

	Transform3d Tneed(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
	Transform3d T2;
	T2.matrix() = TransPosTmp.matrix() * Tneed.matrix();

	if (index > (this->fclLayer.size() - 1)) {
		index = this->fclLayer.size() - 1;
	}

	for (int i = 0; i <= index; i++) {
		manager2->registerObject(new fcl::CollisionObjectd(this->fclLayer[i], T2));
	}


	manager1->setup();
	manager2->setup();

	fcl::DefaultCollisionData<double> collision_data;

	manager1->collide(manager2, &collision_data, fcl::DefaultCollisionFunction);

	if (collision_data.result.isCollision()) {
		flag = true;
	}

	delete manager1;
	delete manager2;
	return flag;
}

bool robSystem::checkAllColi(Eigen::Matrix<double, 8, 1> q, int index) {
	std::vector<Transform3d> TransABBTmp(7);
	Transform3d TransPosTmp;
	this->fkABB2600(q.block<6, 1>(0, 0), TransABBTmp);
	TransPosTmp = this->fkPos(q.block<2, 1>(6, 0));

	bool flag = false;

	fcl::BroadPhaseCollisionManagerd* manager1 = new fcl::DynamicAABBTreeCollisionManagerd();
	fcl::BroadPhaseCollisionManagerd* manager2 = new fcl::DynamicAABBTreeCollisionManagerd();

	/*for (int i = 6; i > 1; i--) {
		Transform3d T1(Transform3d::Identity());
		T1.matrix() = TransABBTmp[i].matrix() * TransABB0[i].matrix().inverse();
		manager1->registerObject(new fcl::CollisionObjectd(this->fclABB[i], T1));
	}*/

	Transform3d T1(Transform3d::Identity());
	T1.matrix() = TransABBTmp[6].matrix() * TransABB0[6].matrix().inverse();
	manager1->registerObject(new fcl::CollisionObjectd(this->fclABB[6], T1));

	Transform3d Tneed(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
	Transform3d T2;
	T2.matrix() = TransPosTmp.matrix() * Tneed.matrix();
	if (index == -1 || index > (this->fclLayer.size() - 1)) {
		index = this->fclLayer.size() - 1;
	}
	for (int i = 0; i <= index; i++) {
		manager2->registerObject(new fcl::CollisionObjectd(this->fclLayer[i], T2));
	}
	T2 = Transform3d::Identity();
	T2.matrix() = TransPosTmp.matrix() * TransPos0[3].matrix().inverse() * TransPos0[0].matrix();
	manager2->registerObject(new fcl::CollisionObjectd(this->fclPOS[2], T2));

	manager1->setup();
	manager2->setup();

	fcl::DefaultCollisionData<double> collision_data;

	manager1->collide(manager2, &collision_data, fcl::DefaultCollisionFunction);

	if (collision_data.result.isCollision()) {
		flag = true;
	}

	delete manager1;
	delete manager2;
	return flag;
}




void robSystem::changeLayerFrame(QMeshPatch* layer) {
	Transform3d T(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));

	Trans_mesh(layer, T, true);
}

Eigen::VectorXd robSystem::Joints2Postions(Eigen::VectorXd joints, bool isBase, Eigen::MatrixXd& posDjoint) {
	std::vector<Transform3d> TransABBTmp(7), TransPosTmp(4);
	this->fkABB2600(joints.block<6, 1>(0, 0), TransABBTmp);
	this->_FK_Pos(joints.block<2, 1>(6, 0), TransPosTmp);
	Transform3d TtoolTmp, TposTmp;
	TtoolTmp.matrix() = TransABBTmp[6].matrix() * Ttool.matrix();
	Eigen::Matrix4d tmp(Eigen::Matrix4d::Identity());
	tmp(0, 3) = 150;
	TposTmp.matrix() = TransPosTmp[3].matrix() * tmp.matrix();

	std::vector<Eigen::Vector3d> pos(8);
	for (int i = 1; i < 5; i++) {
		pos[i - 1] = TransABBTmp[i].translation();
	}
	pos[4] = TransABBTmp[6].translation();
	pos[5] = TtoolTmp.translation();
	pos[6] = TransPosTmp[3].translation();
	pos[7] = TposTmp.translation();


	std::vector<Eigen::MatrixXd> dpos(8);
	std::vector<Eigen::Matrix4d> vecTi(6);
	std::vector<Eigen::Matrix4d> vecdTi(6);
	for (int i = 1; i < 7; i++) {
		Eigen::Matrix4d dTi = this->_dFK_eachlink(joints(i - 1), i);
		Eigen::Matrix4d Ti = this->_FK_eachlink(joints(i - 1), i).matrix();
		vecTi[i - 1] = Ti;
		vecdTi[i - 1] = dTi;
		
		Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(3, 8);

		if (i == 6) {
			dpos[5] = Eigen::MatrixXd::Zero(3, 8);
		}

		for (int j = 0; j < i; j++) {
			Eigen::Matrix4d tmp1=Eigen::Matrix4d::Identity();

			for (int k = 0; k < j; k++) {
				tmp1 = tmp1 * vecTi[k];
			}

			tmp1 = tmp1 * vecdTi[j];

			for (int k = j + 1; k < i; k++) {
				tmp1 = tmp1 * vecTi[k];
			}

			tmp.block<3, 1>(0, j) = tmp1.block<3, 1>(0, 3);

			if (i == 6) {
				tmp1 = tmp1 * Ttool.matrix();
				dpos[5].block<3, 1>(0, j) = tmp1.block<3, 1>(0, 3);
			}

		}

		if (i < 5) {
			dpos[i - 1] = tmp;
		}
		else if (i == 6) {
			dpos[4] = tmp;
		}
	}

	std::vector<Eigen::Matrix4d> dTpos(2);
	this->_dFK_Pos(joints.block<2, 1>(6, 0), dTpos);

	dpos[6]=Eigen::MatrixXd::Zero(3, 8);
	dpos[6].block<3, 1>(0, 6) = dTpos[0].block<3, 1>(0, 3);
	dpos[6].block<3, 1>(0, 7) = dTpos[1].block<3, 1>(0, 3);

	dpos[7] = Eigen::MatrixXd::Zero(3, 8);
	dpos[7].block<3, 1>(0, 6) = (dTpos[0] * tmp).block<3, 1>(0, 3);
	dpos[7].block<3, 1>(0, 7) = (dTpos[1] * tmp).block<3, 1>(0, 3);


	Eigen::VectorXd posLnew;
	Eigen::MatrixXd dposLnew;
	if (isBase) {
		Eigen::VectorXd posL(21);
		Eigen::MatrixXd dposL(21, 8);

		for (int i = 0; i < 7; i++) {
			posL.block<3, 1>(3 * i, 0) = pos[i];
			dposL.block<3, 8>(3 * i, 0) = dpos[i];
		}

		posLnew.resize(19);
		dposLnew.resize(19, 8);

		posLnew.block<2, 1>(0, 0) = posL.block<2, 1>(0, 0);
		posLnew.block<16, 1>(2, 0) = posL.block<16, 1>(3, 0);
		posLnew.block<1, 1>(18, 0) = posL.block<1, 1>(20, 0);

		dposLnew.block<2, 8>(0, 0) = dposL.block<2, 8>(0, 0);
		dposLnew.block<16, 8>(2, 0) = dposL.block<16, 8>(3, 0);
		dposLnew.block<1, 8>(18, 0) = dposL.block<1, 8>(20, 0);
	}
	else {
		Eigen::VectorXd posL(24);
		Eigen::MatrixXd dposL(24, 8);

		for (int i = 0; i < 8; i++) {
			posL.block<3, 1>(3 * i, 0) = pos[i];
			dposL.block<3, 8>(3 * i, 0) = dpos[i];
		}

		posLnew.resize(22);
		dposLnew.resize(22, 8);

		posLnew.block<2, 1>(0, 0) = posL.block<2, 1>(0, 0);
		posLnew.block<16, 1>(2, 0) = posL.block<16, 1>(3, 0);
		posLnew.block<4, 1>(18, 0) = posL.block<4, 1>(20, 0);

		dposLnew.block<2, 8>(0, 0) = dposL.block<2, 8>(0, 0);
		dposLnew.block<16, 8>(2, 0) = dposL.block<16, 8>(3, 0);
		dposLnew.block<4, 8>(18, 0) = dposL.block<4, 8>(20, 0);
	}
	
	//std::cout<<"\n"<<posL.transpose()<<std::endl;
	//std::cout << posLnew.transpose() << std::endl;

	posDjoint = dposLnew;
	return posLnew;
}

Eigen::VectorXd robSystem::Joints2Postions(Eigen::VectorXd joints, bool isBase) {
	std::vector<Transform3d> TransABBTmp(7), TransPosTmp(4);
	this->fkABB2600(joints.block<6, 1>(0, 0), TransABBTmp);
	this->_FK_Pos(joints.block<2, 1>(6, 0), TransPosTmp);
	Transform3d TtoolTmp, TposTmp;
	TtoolTmp.matrix() = TransABBTmp[6].matrix() * Ttool.matrix();
	Eigen::Matrix4d tmp(Eigen::Matrix4d::Identity());
	tmp(0, 3) = 150;
	TposTmp.matrix() = TransPosTmp[3].matrix() * tmp.matrix();

	std::vector<Eigen::Vector3d> pos(8);
	for (int i = 1; i < 5; i++) {
		pos[i - 1] = TransABBTmp[i].translation();
	}
	pos[4] = TransABBTmp[6].translation();
	pos[5] = TtoolTmp.translation();
	pos[6] = TransPosTmp[3].translation();
	pos[7] = TposTmp.translation();

	Eigen::VectorXd posLnew;
	if (isBase) {
		Eigen::VectorXd posL(21);

		for (int i = 0; i < 7; i++) {
			posL.block<3, 1>(3 * i, 0) = pos[i];
		}

		posLnew.resize(19);
		posLnew.block<2, 1>(0, 0) = posL.block<2, 1>(0, 0);
		posLnew.block<16, 1>(2, 0) = posL.block<16, 1>(3, 0);
		posLnew.block<1, 1>(18, 0) = posL.block<1, 1>(20, 0);

	}
	else {
		Eigen::VectorXd posL(24);

		for (int i = 0; i < 8; i++) {
			posL.block<3, 1>(3 * i, 0) = pos[i];
		}

		posLnew.resize(22);

		posLnew.block<2, 1>(0, 0) = posL.block<2, 1>(0, 0);
		posLnew.block<16, 1>(2, 0) = posL.block<16, 1>(3, 0);
		posLnew.block<4, 1>(18, 0) = posL.block<4, 1>(20, 0);
	}

	//std::cout<<"\n"<<posL.transpose()<<std::endl;
	//std::cout << posLnew.transpose() << std::endl;
	return posLnew;
}

Eigen::VectorXd robSystem::Joints2PostionsBC(Eigen::VectorXd joints, int type, Eigen::MatrixXd& posDjoint) {
	std::vector<Transform3d> TransABBTmp(7);
	this->fkABB2600(joints.block<6, 1>(0, 0), TransABBTmp);
	Transform3d TtoolTmp;
	TtoolTmp.matrix() = TransABBTmp[6].matrix() * Ttool.matrix();

	Transform3d TransPosTmp = this->fkPos(joints.block<2, 1>(6, 0));

	std::vector<Transform3d> TransABBinBC(7);
	for (int i = 0; i < 7; i++) {
		TransABBinBC[i].matrix() = TransPosTmp.matrix().inverse() * TransABBTmp[i].matrix();
	}
	Transform3d TtoolinBC;
	TtoolinBC.matrix() = TransPosTmp.matrix().inverse() * TtoolTmp.matrix();

	std::vector<Eigen::Vector3d> pos(6);
	for (int i = 1; i < 5; i++) {
		pos[i - 1] = TransABBinBC[i].translation();
	}
	pos[4] = TransABBinBC[6].translation();
	pos[5] = TtoolinBC.translation();


	std::vector<Eigen::MatrixXd> dpos(6);

	std::vector<Eigen::Matrix4d> vecTi(6);
	std::vector<Eigen::Matrix4d> vecdTi(6);
	for (int i = 1; i < 7; i++) {
		Eigen::Matrix4d dTi = this->_dFK_eachlink(joints(i - 1), i);
		Eigen::Matrix4d Ti = this->_FK_eachlink(joints(i - 1), i).matrix();
		vecTi[i - 1] = Ti;
		vecdTi[i - 1] = dTi;

		Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(3, 8);

		if (i == 6) {
			dpos[5] = Eigen::MatrixXd::Zero(3, 8);
		}

		for (int j = 0; j < i; j++) {
			Eigen::Matrix4d tmp1 = TransPosTmp.matrix().inverse();

			for (int k = 0; k < j; k++) {
				tmp1 = tmp1 * vecTi[k];
			}

			tmp1 = tmp1 * vecdTi[j];

			for (int k = j + 1; k < i; k++) {
				tmp1 = tmp1 * vecTi[k];
			}

			tmp.block<3, 1>(0, j) = tmp1.block<3, 1>(0, 3);

			if (i == 6) {
				tmp1 = tmp1 * Ttool.matrix();
				dpos[5].block<3, 1>(0, j) = tmp1.block<3, 1>(0, 3);
			}

		}

		if (i < 5) {
			dpos[i - 1] = tmp;
		}
		else if (i == 6) {
			dpos[4] = tmp;
		}
	}


	std::vector<Eigen::Matrix4d> dTpos(2);
	dTpos = this->dfkPos(joints.block<2, 1>(6, 0));

	std::vector<Eigen::Matrix4d> dinvTpos(2);
	for (int i = 0; i < 2; i++) {
		dinvTpos[i] = -TransPosTmp.matrix().inverse() * dTpos[i] * TransPosTmp.matrix().inverse();
	}


	for (int i = 0; i < 6; i++) {
		if (i < 4) {
			Eigen::Matrix4d tmp = dinvTpos[0] * TransABBTmp[i + 1].matrix();
			dpos[i].block<3, 1>(0, 6) = tmp.block<3, 1>(0, 3);

			tmp = dinvTpos[1] * TransABBTmp[i + 1].matrix();
			dpos[i].block<3, 1>(0, 7) = tmp.block<3, 1>(0, 3);
		}
		if (i == 5) {
			Eigen::Matrix4d tmp = dinvTpos[0] * TransABBTmp[i + 1].matrix();
			dpos[4].block<3, 1>(0, 6) = tmp.block<3, 1>(0, 3);
			tmp = tmp * Ttool.matrix();
			dpos[5].block<3, 1>(0, 6) = tmp.block<3, 1>(0, 3);

			tmp = dinvTpos[1] * TransABBTmp[i + 1].matrix();
			dpos[4].block<3, 1>(0, 7) = tmp.block<3, 1>(0, 3);
			tmp = tmp * Ttool.matrix();
			dpos[5].block<3, 1>(0, 7) = tmp.block<3, 1>(0, 3);
		}	
	}


	Eigen::VectorXd posL(18);
	Eigen::MatrixXd dposL(18, 8);

	for (int i = 0; i < 6; i++) {
		posL.block<3, 1>(3 * i, 0) = pos[i];
		dposL.block<3, 8>(3 * i, 0) = dpos[i];
	}


	Eigen::VectorXd posLnew;
	Eigen::MatrixXd dposLnew;
	//1,2,3,4,5,6;
	if (type >=1&& type<=6) {
		int n1 = 3 * (7 - type);
		posLnew = posL.segment(3 * (type - 1), n1);
		dposLnew = dposL.block(3 * (type - 1), 0, n1, 8);
	}
	else {
		posLnew = posL;
		dposLnew = dposL;
	}


	//std::cout<<"\n"<<posL.transpose()<<std::endl;
	//std::cout << posLnew.transpose() << std::endl;

	posDjoint = dposLnew;
	return posLnew;
}


Eigen::VectorXd robSystem::Joints2PostionsBC(Eigen::VectorXd joints, int type) {
	std::vector<Transform3d> TransABBTmp(7);
	this->fkABB2600(joints.block<6, 1>(0, 0), TransABBTmp);
	Transform3d TtoolTmp;
	TtoolTmp.matrix() = TransABBTmp[6].matrix() * Ttool.matrix();

	Transform3d TransPosTmp = this->fkPos(joints.block<2, 1>(6, 0));

	std::vector<Transform3d> TransABBinBC(7);
	for (int i = 0; i < 7; i++) {
		TransABBinBC[i].matrix() = TransPosTmp.matrix().inverse() * TransABBTmp[i].matrix();
	}
	Transform3d TtoolinBC;
	TtoolinBC.matrix() = TransPosTmp.matrix().inverse() * TtoolTmp.matrix();

	std::vector<Eigen::Vector3d> pos(6);
	for (int i = 1; i < 5; i++) {
		pos[i - 1] = TransABBinBC[i].translation();
	}
	pos[4] = TransABBinBC[6].translation();
	pos[5] = TtoolinBC.translation();

	Eigen::VectorXd posL(18);

	for (int i = 0; i < 6; i++) {
		posL.block<3, 1>(3 * i, 0) = pos[i];
	}


	Eigen::VectorXd posLnew;
	Eigen::MatrixXd dposLnew;
	//1,2,3,4,5,6;
	if (type >= 1 && type <= 6) {
		int n1 = 3 * (7 - type);
		posLnew = posL.segment(3 * (type - 1), n1);
	}
	else {
		posLnew = posL;
	}

	return posLnew;
}

void robSystem::updateRobPos(PolygenMesh* Rob, Eigen::Matrix<double, 6, 1> q, bool isUpdate_lastCoord3D) {
	fkABB2600(q, true);
	int i = 0;
	Transform3d T;
	for (GLKPOSITION rob = Rob->GetMeshList().GetHeadPosition(); rob;) {
		QMeshPatch* link = (QMeshPatch*)Rob->GetMeshList().GetNext(rob);
		T.matrix() = TransABB[i].matrix() * TransABB0[i].matrix().inverse();
		this->Trans_mesh(link, T, isUpdate_lastCoord3D);
		i++;
	}
}

void robSystem::updateLayerPos(PolygenMesh* Layers, Eigen::Vector2d q, bool isUpdate_lastCoord3D) {
	Transform3d T = fkPos(q);

	int i = 0;
	for (GLKPOSITION rob = Layers->GetMeshList().GetHeadPosition(); rob;) {
		QMeshPatch* link = (QMeshPatch*)Layers->GetMeshList().GetNext(rob);
		this->Trans_mesh(link, T, isUpdate_lastCoord3D);
		i++;
	}
}