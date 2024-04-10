#include "mathTools.h"


Eigen::Matrix3d mathTools::logm(Eigen::Matrix3d A) {
	double theta;
	Eigen::Matrix3d omega;

	double trace = A.trace();

	if (fabs(trace + 1) < 1e-12) {
		trace = -1;
	}

	if (trace < -1) {
		//std::cout<<"trace: "<<trace<<std::endl;
		trace = -1;
	}

	if (A.isIdentity()) {
		theta = 0;
		omega = Eigen::Matrix3d::Zero();
	}
	else if (trace == -1) {
		theta = M_PI;
		Eigen::Vector3d omega0;
		if (1 + A(2, 2) > 1e-3) {
			omega0 << A(0, 2), A(1, 2), A(2, 2) + 1;
			omega0 = omega0 / sqrt(2 * (1 + A(2, 2)));
		}
		else if (1 + A(1, 1) > 1e-3) {
			omega0 << A(0, 1), A(1, 1) + 1, A(2, 1);
			omega0 = omega0 / sqrt(2 * (1 + A(1, 1)));
		}
		else {
			omega0 << 1 + A(0, 0), A(1, 0) + 1, A(2, 0);
			omega0 = omega0 / sqrt(2 * (1 + A(0, 0)));
		}
		omega << 0, -omega0(2), omega0(1),
			omega0(2), 0, -omega0(0),
			-omega0(1), omega0(0), 0;
	}
	else {
		if ((trace - 1) / 2 > 1 || (trace - 1) / 2 < -1) {
			std::cout<<"error in logm"<<std::endl;
		}
		theta = acos((A.trace() - 1) / 2);
		omega = (A - A.transpose()) / 2 / sin(theta);
	}
	omega = omega * theta;
	return omega;
}

Eigen::Matrix4d mathTools::logm(Eigen::Matrix4d A) {
	if (A(3, 3) != 1) {
		std::cout << "not good A" << std::endl;
		std::cout << A << std::endl;
	}
	Eigen::Matrix3d R = A.block<3, 3>(0, 0), omega, Ginv;
	Eigen::Vector3d p = A.block<3, 1>(0, 3), v;
	Eigen::Matrix4d S = Eigen::Matrix4d::Zero();
	double theta;
	if (R.isIdentity()) {
		v = p / p.norm();
		theta = p.norm();
		omega = Eigen::Matrix3d::Zero();
	}
	else if (R.trace() == -1) {
		theta = M_PI;
		omega = logm(R);
		Ginv = Eigen::Matrix3d::Identity() / theta - omega / 2 + (1 / theta - 0.5 * tan(M_PI / 2 - theta / 2)) * omega * omega;
		v = Ginv * p;
	}
	else {
		double tmp = (R.trace() - 1) / 2;
		//std::cout<<tmp<<std::endl;
		if (tmp > 1) {
			std::cout << "not good R" << std::endl;
			//std::cout << R << std::endl;

		}
		theta = acos(tmp);
		omega = (R - R.transpose()) / 2 / sin(theta);
		Ginv = Eigen::Matrix3d::Identity() / theta - omega / 2 + (1 / theta - 0.5 * tan(M_PI / 2 - theta / 2)) * omega * omega;
		v = Ginv * p;
	}
	S.block<3, 3>(0, 0) = omega;
	S.block<3, 1>(0, 3) = v;
	S = S * theta;
	return S;
}

Eigen::Matrix3d mathTools::expm(Eigen::Matrix3d A) {
	Eigen::Vector3d r;
	r << A(2, 1), A(0, 2), A(1, 0);
	double theta = r.norm();
	Eigen::Matrix3d A2;
	A2 = A / theta;
	Eigen::Matrix3d B(Eigen::Matrix3d::Identity());
	B = B + sin(theta) * A2 + (1 - cos(theta)) * A2 * A2;
	return B;
}

Eigen::Matrix3d mathTools::dexpm(Eigen::Matrix3d A) {
	Eigen::Vector3d r;
	r << A(2, 1), A(0, 2), A(1, 0);
	double theta = r.norm();
	Eigen::Matrix3d A2;
	A2 = A / theta;

	Eigen::Matrix3d B(Eigen::Matrix3d::Zero());

	B = cos(theta) * A2 + sin(theta) * A2 * A2;

	return B;
}

Eigen::Matrix4d mathTools::expm(Eigen::Matrix4d A) {
	Eigen::Matrix4d B = Eigen::Matrix4d::Identity();
	Eigen::Vector3d r;
	r << A(2, 1), A(0, 2), A(1, 0);
	if (r.norm() > 1e-10) {
		double theta = r.norm();
		Eigen::Matrix4d Anew = A / theta;
		Eigen::Matrix3d omega = Anew.block<3, 3>(0, 0);
		Eigen::Vector3d v = Anew.block<3, 1>(0, 3);

		Eigen::Matrix3d omegaTheta = A.block<3, 3>(0, 0);
		B.block<3, 3>(0, 0) = expm(omegaTheta);

		Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
		G = G * theta + (1 - cos(theta)) * omega + (theta - sin(theta)) * omega * omega;
		B.block<3, 1>(0, 3) = G * v;
	}
	else {
		r = A.block<3, 1>(0, 3);
		B.block<3, 1>(0, 3) = r;
	}
	return B;
}

Eigen::Matrix4d mathTools::dexpm(Eigen::Matrix4d A) {
	Eigen::Matrix4d B = Eigen::Matrix4d::Zero();
	Eigen::Vector3d r;
	r << A(2, 1), A(0, 2), A(1, 0);
	if (r.norm() > 1e-10) {
		double theta = r.norm();
		Eigen::Matrix4d Anew = A / theta;
		Eigen::Matrix3d omega = Anew.block<3, 3>(0, 0);
		Eigen::Vector3d v = Anew.block<3, 1>(0, 3);

		Eigen::Matrix3d omegaTheta = A.block<3, 3>(0, 0);
		B.block<3, 3>(0, 0) = dexpm(omegaTheta);

		Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
		G = G + sin(theta) * omega + (1 - cos(theta)) * omega * omega;
		B.block<3, 1>(0, 3) = G * v;
	}
	else {
		r = A.block<3, 1>(0, 3);
		r.normalize();
		B.block<3, 1>(0, 3) = r;
	}
	return B;
}




Eigen::Matrix3d mathTools::skew(Eigen::Vector3d v) {
	Eigen::Matrix3d m;
	m << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;
	return m;
}


Eigen::Matrix4d mathTools::xi2xihat(Eigen::Matrix<double, 6, 1> xi) {
	//注意这里v和omega的顺序
	
	//std::cout << xi.block<3, 1>(0, 0).norm() << std::endl;

	Eigen::Matrix4d xihat = Eigen::Matrix4d::Zero();
	xihat.block<3, 1>(0, 3) = xi.block<3, 1>(3, 0);
	xihat.block<3, 3>(0, 0) = skew(xi.block<3, 1>(0, 0));
	return xihat;
}

Eigen::Vector3d mathTools::vex(Eigen::Matrix3d A) {
	Eigen::Vector3d v;
	v(0) = A(2, 1);
	v(1) = A(0, 2);
	v(2) = A(1, 0);
	return v;
}

Eigen::Matrix<double,6,1> mathTools::vex(Eigen::Matrix4d A) {
	//注意这里v和omega的顺序

	Eigen::Matrix<double, 6, 1> v;
	v.block<3, 1>(0, 0) = A.block<3, 1>(0, 3);
	
	Eigen::Matrix3d A1 = A.block<3, 3>(0, 0);
	Eigen::Vector3d v1 = vex(A1);

	v.block<3, 1>(3, 0) = v1;
	
	return v;
}

Eigen::Matrix3d mathTools::diffRotm(Eigen::Vector3d w, Eigen::Vector3d dw) {
	//求旋转矩阵的导数
	//R: 旋转矩阵,R=expm([w])
	//dw：w的导数

	Eigen::Matrix3d tmp = skew(w);
	Eigen::Matrix3d R = expm(tmp);
	double nor = w.norm();

	Eigen::Matrix3d A = Eigen::Matrix3d::Identity();

	if (nor != 0) {
		A = A - (1 - cos(nor)) / pow(nor, 2) * tmp + (nor - sin(nor)) / pow(nor, 3) * tmp * tmp;
	}

	Eigen::Vector3d omega = A * dw;
	Eigen::Matrix3d omegaHat = skew(omega);

	Eigen::Matrix3d dR;
	dR = R * omegaHat;

	return dR;
}