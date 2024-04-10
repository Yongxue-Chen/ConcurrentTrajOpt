#include "pathStruct.h"

toolpath::toolpath(std::vector<Eigen::Vector3d> Point, std::vector<Eigen::Vector3d> Normal) {
	this->pathPoint = Point;
	this->pathNormal = Normal;
	if (pathPoint.size() != pathNormal.size()) {
		std::cerr << "point and normal do not match!" << std::endl;
	}
	else {
		this->length = pathPoint.size();
	}
}

void toolpath::setPath(std::vector<Eigen::Vector3d> Point, std::vector<Eigen::Vector3d> Normal) {
	this->pathPoint = Point;
	this->pathNormal = Normal;
	if (pathPoint.size() != pathNormal.size()) {
		std::cerr << "point and normal do not match!" << std::endl;
	}
	else {
		this->length = pathPoint.size();
	}
}

void toolpath::getPart(toolpath* newpath, int a, int b) {
	if (b >= this->length) {
		std::cerr << "exceed the length of toolpath" << std::endl;
		return;
	}

	newpath->length = b - a + 1;

	if (b < (this->length - 1)) {
		std::vector<Eigen::Vector3d> points(this->pathPoint.begin() + a, this->pathPoint.begin() + b + 1);
		newpath->pathPoint = points;
		std::vector<Eigen::Vector3d> normals(this->pathNormal.begin() + a, this->pathNormal.begin() + b + 1);
		newpath->pathNormal = normals;
	}
	else {
		std::vector<Eigen::Vector3d> points(this->pathPoint.begin() + a, this->pathPoint.end());
		newpath->pathPoint = points;
		std::vector<Eigen::Vector3d> normals(this->pathNormal.begin() + a, this->pathNormal.end());
		newpath->pathNormal = normals;
	}

}

void toolpath::getDPath() {
	dPathPoint.resize(length);
	dPathPoint[0] = 0;
	for (int i = 1; i < length; i++) {
		Eigen::Vector3d diff = pathPoint[i] - pathPoint[i - 1];
		dPathPoint[i] = diff.norm();
	}
}

void toolpath::getDTime(double v) {
	dTimePoint.resize(length);
	dTimePoint[0] = 0;
	for (int i = 1; i < length; i++) {
		dTimePoint[i] = dPathPoint[i] / v;
	}
}

void toolpath::getDTime(Eigen::VectorXd time) {
	dTimePoint.resize(length);
	if (time.size() != length) {
		std::cerr << "time size does not match!" << std::endl;
		return;
	}
	for (int i = 0; i < length; i++) {
		dTimePoint[i] = time(i);
	}
}

void toolpath::getCosAngle() {
	cosAngle.resize(length);
	cosAngle[0] = -1;
	cosAngle[length - 1] = -1;

	for (int i = 1; i < length - 1; i++) {
		Eigen::Vector3d diff1 = pathPoint[i - 1] - pathPoint[i];
		Eigen::Vector3d diff2 = pathPoint[i + 1] - pathPoint[i];
		cosAngle[i] = diff1.normalized().dot(diff2.normalized());
	}
}