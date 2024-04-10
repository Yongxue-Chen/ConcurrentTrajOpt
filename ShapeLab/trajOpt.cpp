#include "trajOpt.h"

trajOpt::trajOpt() {
	//std::cout << "initialize trajOpt" << std::endl;


    JointMax << M_PI, 2.618, 1.309, 6.9813, M_PI / 2, 6.9813, M_PI, 100 * M_PI;
    JointMin << -M_PI, -M_PI / 2, -M_PI, -6.9813, -M_PI / 2, -6.9813, -M_PI, -100 * M_PI;

    M(6, 6) = 1.5;
    M(7, 7) = 1.5;


}

trajOpt::~trajOpt() {
	//std::cout << "delete trajOpt" << std::endl;
}

void trajOpt::setMaxDist(double x) {
	this->maxDist = x;
}


std::vector<toolpath*> trajOpt::splitPath(toolpath* layerPath) {
    std::vector<toolpath*> pathsInLayer;
    int indexBegin = 0;
    for (int i = 1; i < layerPath->length; i++) {
        Eigen::Vector3d diff = layerPath->pathPoint[i] - layerPath->pathPoint[i - 1];
        double dist = diff.norm();
        //std::cout << "dist: " << dist << std::endl;
        if (dist > this->maxDist) {

            if ((i - indexBegin) > 5) {
                toolpath* newpath = new toolpath();
                layerPath->getPart(newpath, indexBegin, i - 1);
                std::cout << "Length of the part: " << newpath->pathPoint.size() << std::endl;
                pathsInLayer.push_back(newpath);
            }

            indexBegin = i;
        }
        if (i == layerPath->length - 1) {

            if ((i - indexBegin) > 5) {
                toolpath* newpath = new toolpath();
                layerPath->getPart(newpath, indexBegin, i);
                pathsInLayer.push_back(newpath);
                std::cout << "Length of the part: " << newpath->pathPoint.size() << std::endl;
			}

        }
    }

    return pathsInLayer;
}


void trajOpt::initializePath(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool checkColi, int layerIndex) {
    std::cout << "\nInitialize tool path" << std::endl;

    errorTol = 0;
    SVMGrad svmGrad;

    if (checkColi) {
        string modelFile = "../DataSet/coliTrain/BaseColi/BaseColiFK.modelG";
        svmGrad.loadModel(modelFile);
        svmGrad.preComputeKernel(true);
    }
    

    int n = toolpath->length;
    Eigen::Vector3d uniZ = Eigen::Vector3d::UnitZ();

    std::vector<Transform3d> TYita;
    TYita.resize(nSamYita);
    std::vector<double> yitaSam;
    yitaSam.resize(nSamYita);
    double delta = 2 * M_PI / (nSamYita - 1);

    for (int i = 0; i < nSamYita; i++) {
        yitaSam[i] = delta * i + M_PI;
        TYita[i] = Transform3d::Identity();
        TYita[i].rotate(Eigen::AngleAxisd(yitaSam[i], uniZ));
    }


    toolpath->posWorkpiece.resize(n);

    std::vector<std::vector<Eigen::Matrix<double, 8, 1>>> jSys(n);
    //std::vector<std::vector<double>> yitaSave(n);

    bool flagNewPath = false;
    Eigen::Vector2d iniBC(0, 0);
    Eigen::Matrix<double, 6, 1> iniQ = Eigen::Matrix<double, 6, 1>::Zero();

#pragma omp parallel for
    for (int i = 0; i < n; i++) {
        toolpath->pathNormal[i].normalize();
        Eigen::Vector3d normal = toolpath->pathNormal[i];
        Eigen::Vector3d position = toolpath->pathPoint[i];

        if (normal(2) > (1 - 1e-5)) {
            toolpath->posWorkpiece[i] = Transform3d::Identity();
            toolpath->posWorkpiece[i].translation() = position;
        }
        else {
            Eigen::Vector3d tmp1 = uniZ.cross(normal);
            tmp1.normalize();
            double tmp2 = uniZ.dot(normal);
            tmp2 = acos(tmp2);

            Transform3d Ttmp(Eigen::AngleAxisd(tmp2, tmp1));
            Ttmp.translation() = position;
            toolpath->posWorkpiece[i] = Ttmp;
        }

        //std::cout << toolpath->posWorkpiece[i].matrix().block<3, 1>(0, 2) - normal << std::endl;
	}


    std::vector<Eigen::Vector2d> vecBC(n);
    std::vector<Eigen::Vector3d> vecXYZ(n);

    for (int i = 0; i < n; i++) {
        Eigen::Vector3d normal = toolpath->pathNormal[i];
        Eigen::Vector3d position = toolpath->pathPoint[i];

        Eigen::Vector3d xyz;
        Eigen::Vector2d BC;

        robs->getBCXYZ(normal, uniZ, position, xyz, BC, iniBC);

        vecBC[i] = BC;
        vecXYZ[i] = xyz;

        //std::cout << xyz.transpose() << std::endl;
        if (std::isnan(xyz(0)) || std::isnan(xyz(1)) || std::isnan(xyz(2))) {
			std::cerr <<i<< ": xyz is NaN" << std::endl;
            std::cout<<"BC: "<<BC.transpose()<<std::endl;
            std::cout<<robs->fkPos(BC).matrix()<<std::endl;
		}
        


        iniBC = BC;
    }

    int i = 0;
    while (i < n - 1) {

        if (std::fabs(vecBC[i](0)) < 0.0873) {

            int jtmp = n - 2;
            for (int j = i; j < n - 1; j++) {
                if (std::fabs(vecBC[j + 1](0)) > 0.0873) {
                    jtmp = j;
                    break;
                }
            }

            if (jtmp == i) {
                i++;
                continue;
            }

            int ntmp = jtmp - i + 1;
            double delta = (vecBC[jtmp](1) - vecBC[i](1)) / (ntmp - 1);

            for (int j = i; j <= jtmp; j++) {
                vecBC[j](1) = vecBC[i](1) + delta * (j - i);

                Transform3d Ttmp = robs->fkPos(vecBC[j]);
                Eigen::Vector4d xyz4 = Eigen::Vector4d::Ones();
                xyz4.block<3, 1>(0, 0) = toolpath->pathPoint[j];
                xyz4 = Ttmp.matrix() * xyz4;
                vecXYZ[j] = xyz4.block<3, 1>(0, 0);

               /* std::cout << j << ":" << std::endl;
                std::cout<<vecBC[j].transpose()<<std::endl;
                std::cout<<vecXYZ[j].transpose()<<std::endl;
                std::cout << std::endl;*/


			}

            i = jtmp + 2;

        }
        else {
            i++;
        }

    }





    for (int i = 0; i < n; i++) {
        //std::cout<<i<<std::endl;
        if (i % 100 == 0) {
			std::cout << "."<<std::flush;
		}

        Eigen::Vector3d normal = toolpath->pathNormal[i];
        Eigen::Vector3d position = toolpath->pathPoint[i];
      
        Eigen::Vector3d xyz = vecXYZ[i];
        Eigen::Vector2d BC = vecBC[i];


        Transform3d T_tool_0 = Transform3d::Identity();
        Eigen::Matrix3d Rtmp;
        Rtmp << 0, 1, 0, -1, 0, 0, 0, 0, 1;
        T_tool_0.linear() = Rtmp;
        T_tool_0.translation() = xyz;


        if (flagNewPath) {
            std::cerr << "Something wrong for flagNewPath" << std::endl;
        }
        else {
            flagNewPath = true;
        }


        std::vector<Eigen::Matrix<double, 8, 1>> jSysTmp(nSamYita);
        std::vector<bool> flagTmp(nSamYita);
        int jBegin = 0;


        double minTau = 10;
        Eigen::Matrix<double, 8, 1> jMin;
        int minIndex;

        for (int j = 0; j < nSamYita; j++) {
            flagTmp[j] = false;

            //std::cout << T_tool_0.matrix() << std::endl;

            Transform3d T_tool_world;
            T_tool_world.matrix() = T_tool_0.matrix() * TYita[j].matrix();
            robs->rotYFrame(T_tool_world);

            Transform3d Ttmp;
            Ttmp.matrix() = T_tool_world.matrix() * robs->Ttool.matrix().inverse();

            bool flag;
            Eigen::Matrix<double, 6, 1> qtmp;

            //std::cout<<T_tool_world.matrix()<<std::endl;
            //std::cout<<robs->Ttool.matrix()<<std::endl;
            //std::cout << "Ttmp: " << Ttmp.matrix() << std::endl;

            qtmp = robs->ikABB2600(flag, Ttmp, iniQ);

            if (flag) {

                if (abs(qtmp(3) - iniQ(3)) > M_PI || abs(qtmp(5) - iniQ(5)) > M_PI) {
                    //std::cout << "inverse solution not continuous!" << std::endl;
                }
                
                Eigen::Matrix<double, 8, 1> jTmp;
                jTmp.block<6, 1>(0, 0) = qtmp;
                jTmp.block<2, 1>(6, 0) = BC;

                double tau, tauBase;
                if (checkColi) {
                    tau = getFastronScore(jTmp, colli->supportData, colli->alpha, robs);

                    Eigen::VectorXd jointPosTmp = robs->Joints2Postions(jTmp, true);
                    tauBase = svmGrad.calculateGamma(jointPosTmp);
                }
                else {
                    tau = -1;
                    tauBase = -1;
                }


                if (tau < errorTol && tauBase < errorTolBase) {

                    //std::cout << "no coli" << std::endl;

                    flagTmp[j] = true;
                    jSysTmp[j] = jTmp;

                    //jSys[i].push_back(jTmp);
                    //yitaSave[i].push_back(yitaSam[j]);

                    if (flagNewPath) {
                        flagNewPath = false;
                        iniQ = qtmp;
                        jBegin = j + 1;
                        break;
                    }
                }
                else {
                    if (tau < minTau) {
						minTau = tau;
                        jMin = jTmp;
                        minIndex = j;
					}
                }

            }
        }

        //std::cout<<jBegin<<std::endl;
        if (jBegin == 0) {

            if (minTau > errorTol) {
                std::cout << " change errorTol to " << minTau << std::endl;
                errorTol = minTau;
                flagNewPath = false;
                iniQ = jMin.block<6, 1>(0, 0);
                jSys[i].push_back(jMin);
            }
            else {
                std::cout << " base coli error!" << std::endl;
            }

            //std::cerr << i << ": not enough sample 1" << std::endl;
            //std::cout<<"maxTau: "<<maxTau<<std::endl;
            
        }

        else {

#pragma omp parallel for
            for (int j = jBegin; j < nSamYita; j++) {
                bool flag4Tmp = false;

                Transform3d T_tool_world;
                T_tool_world.matrix() = T_tool_0.matrix() * TYita[j].matrix();
                robs->rotYFrame(T_tool_world);

                Transform3d Ttmp;
                Ttmp.matrix() = T_tool_world.matrix() * robs->Ttool.matrix().inverse();

                bool flag;
                Eigen::Matrix<double, 6, 1> qtmp;
                qtmp = robs->ikABB2600(flag, Ttmp, iniQ);

                if (flag) {

                    if (abs(qtmp(3) - iniQ(3)) > M_PI || abs(qtmp(5) - iniQ(5)) > M_PI) {
                        //std::cout << "inverse solution not continuous!" << std::endl;
                    }

                    Eigen::Matrix<double, 8, 1> jTmp;
                    jTmp.block<6, 1>(0, 0) = qtmp;
                    jTmp.block<2, 1>(6, 0) = BC;

                    double tau, tauBase;
                    if (checkColi) {
                        tau = getFastronScore(jTmp, colli->supportData, colli->alpha, robs);

                        Eigen::VectorXd jointPosTmp = robs->Joints2Postions(jTmp, true);
                        tauBase = svmGrad.calculateGamma(jointPosTmp);
                    }
                    else {
                        tau = -1;
                        tauBase = -1;
                    }
                    

                    if (tau < errorTol && tauBase < errorTolBase) {

                        //std::cout << "no coli" << std::endl;

                        flag4Tmp = true;
                        jSysTmp[j] = jTmp;
                    }
                }

                flagTmp[j] = flag4Tmp;
            }


            for (int j = 0; j < nSamYita; j++) {
                if (flagTmp[j]) {
                    jSys[i].push_back(jSysTmp[j]);
                    //yitaSave[i].push_back(yitaSam[j]);
                }
            }

            if (jSys[i].size() <= 0) {
                std::cerr << i << ": not enough sample 2" << std::endl;
            }

        }

    }

    std::cout << std::endl;

    toolpath->getDPath();

    if (toolpath->dTimePoint.size() == 0) {
        toolpath->getDTime(vTip);
        std::cout << "set vTip: " << vTip << std::endl;
    }


    std::vector<int> index;
    index = graphJoint(jSys);

    Joints.resize(n);

    if (!checkColi) {
        std::cout << "check collision..." << std::endl;
        for (int i = 0; i < n; i++) {

            if (i % 100 == 0) {
                std::cout << "." << std::flush;
            }

            Eigen::Matrix<double, 8, 1> tmp = jSys[i][index[i]];
            bool coli = robs->checkAllColi(tmp, layerIndex);
            //std::cout << tmp << std::endl;
            //std::cout << layerIndex << std::endl;

            if (!coli) {
                Joints[i] = tmp;
                //std::cout << i << " OK!" << std::endl;
            }
            else {
                std::sort(jSys[i].begin(), jSys[i].end(),
                    [tmp](const Eigen::Matrix<double, 8, 1>& a, const Eigen::Matrix<double, 8, 1>& b) {return (a - tmp).norm() < (b - tmp).norm(); });

                int j = 0;
                while (coli && j < (jSys[i].size() - 1)) {
                    j++;
                    tmp = jSys[i][j];
                    coli = robs->checkAllColi(tmp, layerIndex);
                    //std::cout << j << ": " << coli << std::endl;
                }

                if (coli) {
                    std::cout << i << ": no feasible solution!" << std::endl;
                    std::cout << jSys[i][0].transpose() << std::endl;
                }
                else {
                    Joints[i] = tmp;
                    //std::cout << i << " OK!" << std::endl;
                }
            }
        }
        std::cout << std::endl;

    }

    else {
        for (int i = 0; i < n; i++) {
            Joints[i] = jSys[i][index[i]];
        }
    }

    std::cout<<"Initialize tool path done\n"<<std::endl;

}

bool trajOpt::compare(Eigen::VectorXd a, Eigen::VectorXd b, Eigen::VectorXd tmp) {
    return (a - tmp).norm() < (b - tmp).norm();
}


std::vector<int> trajOpt::graphJoint(std::vector<std::vector<Eigen::Matrix<double, 8, 1>>> q) {
    int n = q.size();
    std::vector<int> jOpt(n);
    std::vector<std::vector<double>> M_dist(n), M_ind(n);

    std::vector<double> _Mdist;
    _Mdist.resize(q[0].size());
    for (int i = 0; i < q[0].size(); i++) {
        _Mdist[i] = 0;
    }
    M_dist[0] = _Mdist;
    M_ind[0] = _Mdist;

    double dcur;

    for (int k = 1; k < n; k++) {
        _Mdist.clear();
        _Mdist.resize(q[k].size());
        M_dist[k] = _Mdist;
        M_ind[k] = _Mdist;
        for (int j = 0; j < q[k].size(); j++) {
            double dmin = DBL_MAX;
            Eigen::Matrix<double, 8, 1> tmp = q[k][j];
            int jopt = q[k - 1].size();

            for (int p = 0; p < q[k - 1].size(); p++) {
                Eigen::Matrix<double, 8, 1> vJ = tmp - q[k - 1][p];
                dcur = M_dist[k - 1][p] + vJ.dot(vJ);
                if (dcur < dmin) {
                    dmin = dcur;
                    jopt = p;
                }
            }

            if (jopt == q[k - 1].size()) {
                std::cout << q[k][j].transpose() << std::endl;
                std::cout << "Joint("<<k<<","<<j<<") not good" << std::endl;
            }

            M_dist[k][j] = dmin;
            M_ind[k][j] = jopt;
        }
    }

    double dmin = DBL_MAX;
    int jopt;

    for (int j = 0; j < q[n - 1].size(); j++) {
        dcur = M_dist[n - 1][j];
        if (dcur < dmin) {
            dmin = dcur;
            jopt = j;
        }
    }

    for (int k = n - 1; k >= 0; k--) {
        jOpt[k] = jopt;
        jopt = M_ind[k][jopt];
    }

    this->status = 1;

    return jOpt;

}

Eigen::VectorXd trajOpt::jerkNum(std::vector<Eigen::VectorXd> joint, std::vector<double> dist, Eigen::Matrix<double, 5, 1>& kJerk) {

    if (joint.size() != 5 || dist.size() != 4) {
        std::cerr << "input error in jerkNum" << std::endl;
    }
    Eigen::VectorXd f0 = joint[2], f1 = joint[3], f2 = joint[4], f3 = joint[1], f4 = joint[0];
    double a = dist[2], b = dist[3], c = dist[1], d = dist[0];

    Eigen::VectorXd jerk;
    //jerk.resize(f0.size());

    double k4 = -(12 * a + 6 * b - 6 * c) / (d * (c + d) * (a + c + d) * (a + b + c + d));
    double k3 = (6 * (2 * a + b - c - d)) / (c * d * (a + c) * (a + b + c));
    double k2 = (6 * (2 * c - a + d)) / (b * (a + b) * (a + b + c) * (a + b + c + d));
    double k1 = (6 * (a + b - 2 * c - d)) / (a * b * (a + c) * (a + c + d));
    double k0 = -(6 * (2 * a + b - 2 * c - d)) / (a * c * (a + b) * (c + d));


    kJerk(0) = k4;
    kJerk(1) = k3;
    kJerk(2) = k0;
    kJerk(3) = k1;
    kJerk(4) = k2;

    jerk = k0 * f0 + k1 * f1 + k2 * f2 + k3 * f3 + k4 * f4;

    return jerk;
}

Eigen::Matrix<double, 5, 4> trajOpt::getdkJerk(std::vector<double> dist) {
	Eigen::Matrix<double, 5, 4> dkJerk;
	double a = dist[2], b = dist[3], c = dist[1], d = dist[0];

    dkJerk.setZero();
    dkJerk(0, 0) = (6 * (2 * a + b - c) * (a * a * c + 2 * a * a * d + 2 * a * c * c + 8 * a * c * d + b * a * c + 6 * a * d * d + 2 * b * a * d + pow(c, 3) + 6 * c * c * d + b * c * c + 9 * c * d * d + 4 * b * c * d + 4 * pow(d, 3) + 3 * b * d * d)) / (d * d * pow((c + d), 2) * pow((a + c + d), 2) * pow((a + b + c + d), 2));
    dkJerk(0, 1) = (6 * (2 * pow(a, 3) + 3 * a * a * b + 8 * a * a * c + 9 * a * a * d + a * b * b + 8 * a * b * c + 9 * a * b * d + 4 * a * c * c + 12 * a * c * d + 8 * a * d * d + 2 * b * b * c + 2 * b * b * d + 2 * b * c * c + 6 * b * c * d + 4 * b * d * d - 2 * pow(c, 2) - 3 * c * c * d + pow(d, 3))) / (d * pow((c + d), 2) * pow((a + c + d), 2) * pow((a + b + c + d), 2));
    dkJerk(0, 2) = -(6 * (-2 * a * a - 2 * a * b + 2 * a * c - b * b + b * c + 4 * c * c + 6 * c * d + 2 * d * d)) / (d * (c + d) * pow((a + c + d), 2) * pow((a + b + c + d), 2));
    dkJerk(0, 3) = -(6 * (2 * c - a + d)) / (d * (c + d) * (a + c + d) * pow((a + b + c + d), 2));
    dkJerk(1, 0) = -(6 * (2 * a + b - c)) / (c * d * d * (a + c) * (a + b + c));
    dkJerk(1, 1) = -(6 * (2 * pow(a, 3) + 3 * a * a * b + 8 * a * a * c - d * a * a + a * b * b + 8 * a * b * c - d * a * b + 4 * a * c * c - 4 * d * a * c + 2 * b * b * c + 2 * b * c * c - 2 * d * b * c - 2 * pow(c, 3) - 3 * d * c * c)) / (c * c * d * pow((a + c), 2) * pow((a + b + c), 2));
    dkJerk(1, 2) = (6 * (-2 * a * a - 2 * a * b + 2 * a * c + 2 * d * a - b * b + b * c + d * b + 4 * c * c + 2 * d * c)) / (c * d * pow((a + c), 2) * pow((a + b + c), 2));
    dkJerk(1, 3) = (6 * (2 * c - a + d)) / (c * d * (a + c) * pow((a + b + c), 2));
    dkJerk(2, 0) = (6 * (2 * a + b - c)) / (a * c * (a + b) * pow((c + d), 2));
    dkJerk(2, 1) = (6 * (4 * a * c + 2 * a * d + 2 * b * c + b * d - 2 * c * d - 2 * c * c - d * d)) / (a * c * c * (a + b) * pow((c + d), 2));
    dkJerk(2, 2) = -(6 * (4 * a * c - 2 * a * b + 2 * a * d + 2 * b * c + b * d - 2 * a * a - b * b)) / (a * a * c * pow((a + b), 2) * (c + d));
    dkJerk(2, 3) = -(6 * (2 * c - a + d)) / (a * c * pow((a + b), 2) * (c + d));
    dkJerk(3, 0) = -(6 * (2 * a + b - c)) / (a * b * (a + c) * pow((a + c + d), 2));
    dkJerk(3, 1) = -(6 * (4 * a * a + 2 * a * c + a * d + 2 * b * a - 2 * c * c - 2 * c * d + 2 * b * c - d * d + b * d)) / (a * b * pow((a + c), 2) * pow((a + c + d), 2));
    dkJerk(3, 2) = (6 * (-2 * pow(a, 3) + 4 * a * a * c + 2 * a * a * d - 3 * b * a * a + 8 * a * c * c + 8 * a * c * d - 4 * b * a * c + 2 * a * d * d - 2 * b * a * d + 2 * pow(c, 3) + 3 * c * c * d - b * c * c + c * d * d - b * c * d)) / (a * a * b * pow((a + c), 2) * pow((a + c + d), 2));
    dkJerk(3, 3) = (6 * (2 * c - a + d)) / (a * b * b * (a + c) * (a + c + d));
    dkJerk(4, 0) = (6 * (2 * a + b - c)) / (b * (a + b) * (a + b + c) * pow((a + b + c + d), 2));
    dkJerk(4, 1) = (6 * (4 * a * a + 6 * a * b + 2 * a * c + a * d + 2 * b * b - 2 * c * c - 2 * c * d - d * d)) / (b * (a + b) * pow((a + b + c), 2) * pow((a + b + c + d), 2));
    dkJerk(4, 2) = -(6 * (-2 * pow(a, 3) - 3 * a * a * b + 4 * a * a * c + 2 * a * a * d + 12 * a * b * c + 6 * a * b * d + 8 * a * c * c + 8 * a * c * d + 2 * a * d * d + pow(b, 3) + 8 * b * b * c + 4 * b * b * d + 9 * b * c * c + 9 * b * c * d + 2 * b * d * d + 2 * pow(c, 3) + 3 * c * c * d + c * d * d)) / (b * pow((a + b), 2) * pow((a + b + c), 2) * pow((a + b + c + d), 2));
    dkJerk(4, 3) = -(6 * (2 * c - a + d) * (pow(a, 3) + 6 * a * a * b + 2 * a * a * c + d * a * a + 9 * a * b * b + 8 * a * b * c + 4 * d * a * b + a * c * c + d * a * c + 4 * pow(b, 3) + 6 * b * b * c + 3 * d * b * b + 2 * b * c * c + 2 * d * b * c)) / (b * b * pow((a + b), 2) * pow((a + b + c), 2) * pow((a + b + c + d), 2));

    return dkJerk;
}

Eigen::VectorXd trajOpt::velNum(std::vector<Eigen::VectorXd> joint, std::vector<double> dist, Eigen::Matrix<double, 5, 1>& kVel) {

    if (joint.size() != 3 || dist.size() != 2) {
        std::cerr << "input error in velNum" << std::endl;
    }

    Eigen::VectorXd f0 = joint[0], f1 = joint[1], f2 = joint[2];
    double s0 = dist[0], s1 = dist[1];

    Eigen::VectorXd vel;
    vel.resize(f0.size());

    vel = (s0 * s0 * f2 + (s1 * s1 - s0 * s0) * f1 - s1 * s1 * f0) / s0 / s1 / (s0 + s1);

    kVel(0) = 0;
    kVel(1) = -s1 / s0 / (s0 + s1);
    kVel(2) = (s1 * s1 - s0 * s0) / s0 / s1 / (s0 + s1);
    kVel(3)= s0 / s1 / (s0 + s1);
    kVel(4) = 0;

    return vel;
}

Eigen::Matrix<double, 5, 4> trajOpt::getdkVel(std::vector<double> dist) {
    Eigen::Matrix<double, 5, 4> dkVel;
	double s0 = dist[0], s1 = dist[1];
	
    dkVel.setZero();
    dkVel(1, 1) = (s1 * (2 * s0 + s1)) / (pow(s0, 2) * pow((s0 + s1), 2));
    dkVel(1, 2) = -1 / pow(s0 + s1, 2);
    dkVel(2, 1) = -1 / pow(s0, 2);
    dkVel(2, 2) = 1 / pow(s1, 2);
    dkVel(3, 1) = 1 / pow(s0 + s1, 2);
    dkVel(3, 2) = -(s0 * (s0 + 2 * s1)) / (pow(s1, 2) * pow((s0 + s1), 2));

	return dkVel;

}

Eigen::VectorXd trajOpt::accNum(std::vector<Eigen::VectorXd> joint, std::vector<double> dist, Eigen::Matrix<double, 5, 1>& kAcc) {

    if (joint.size() != 3 || dist.size() != 2) {
        std::cerr << "input error in velNum" << std::endl;
    }

    Eigen::VectorXd f0 = joint[0], f1 = joint[1], f2 = joint[2];
    double s0 = dist[0], s1 = dist[1];

    Eigen::VectorXd acc;
    acc.resize(f0.size());

    acc = 2 * f0 / s0 / (s0 + s1) - 2 * f1 / s0 / s1 + 2 * f2 / s1 / (s0 + s1);

    kAcc(0) = 0;
    kAcc(1) = 2 / s0 / (s0 + s1);
    kAcc(2) = -2 / s0 / s1;
    kAcc(3) = 2 / s1 / (s0 + s1);
    kAcc(4) = 0;

    return acc;
}

Eigen::Matrix<double, 5, 4> trajOpt::getdkAcc(std::vector<double> dist) {
	Eigen::Matrix<double, 5, 4> dkAcc;
	double s0 = dist[0], s1 = dist[1];
	
	dkAcc.setZero();
    dkAcc(1, 1) = -(2 * (2 * s0 + s1)) / (pow(s0, 2) * pow((s0 + s1), 2));
    dkAcc(1, 2) = -2 / (s0 * pow((s0 + s1), 2));
    dkAcc(2, 1) = 2 / (pow(s0, 2) * s1);
    dkAcc(2, 2) = 2 / (pow(s1, 2) * s0);
    dkAcc(3, 1) = -2 / (s1 * pow((s0 + s1), 2));
    dkAcc(3, 2) = -(2 * (s0 + 2 * s1)) / (pow(s1, 2) * pow((s0 + s1), 2));

	return dkAcc;

}

void trajOpt::initializeOpt(toolpath* toolpath, robSystem* robs, bool optTime, Eigen::Vector4d kSet) {
    std::cout<<"\nInitialize optimization" << std::endl;
    
    if (status != 1) {
        std::cerr << "status wrong in initializeOpt" << std::endl;
    }
    
    int n = toolpath->length;

    Vel.resize(n);
    Acc.resize(n);
    Jerk.resize(n);

    kVel.resize(n);
    kAcc.resize(n);
    kJerk.resize(n);

    Vel[0] = (Joints[1] - Joints[0]) / toolpath->dTimePoint[1];
    kVel[0] << 0, 0, -1 / toolpath->dTimePoint[1], 1 / toolpath->dTimePoint[1], 0;
    Vel[n - 1] = (Joints[n - 1] - Joints[n - 2]) / toolpath->dTimePoint[n - 1];
    kVel[n - 1] << 0, -1 / toolpath->dTimePoint[n - 1], 1 / toolpath->dTimePoint[n - 1], 0, 0;

    kAcc[0] << 0, 0, 1 / pow(toolpath->dTimePoint[1], 2),
        -1 / pow(toolpath->dTimePoint[1], 2) - 1 / toolpath->dTimePoint[1] / toolpath->dTimePoint[2],
        1 / toolpath->dTimePoint[1] / toolpath->dTimePoint[2];
    Acc[0] = kAcc[0](2) * Joints[0] + kAcc[0](3) * Joints[1] + kAcc[0](4) * Joints[2];
    kAcc[n - 1] << 1 / toolpath->dTimePoint[n - 1] / toolpath->dTimePoint[n - 2],
        -1 / toolpath->dTimePoint[n - 1] / toolpath->dTimePoint[n - 2] - 1 / pow(toolpath->dTimePoint[n - 1], 2),
        1 / pow(toolpath->dTimePoint[n - 1], 2), 0, 0;
    Acc[n - 1] = kAcc[n - 1](0) * Joints[n - 3] + kAcc[n - 1](1) * Joints[n - 2] + kAcc[n - 1](2) * Joints[n - 1];


    if (optTime) {
        dkVel.resize(n);
        dkAcc.resize(n);
        dkJerk.resize(n);

        dkVel[0].setZero();
        dkVel[0](2, 2) = 1 / pow(toolpath->dTimePoint[1], 2);
        dkVel[0](3, 2) = -1 / pow(toolpath->dTimePoint[1], 2);

        dkVel[n - 1].setZero();
        dkVel[n - 1](1, 1) = 1 / pow(toolpath->dTimePoint[n - 1], 2);
        dkVel[n - 1](2, 1) = -1 / pow(toolpath->dTimePoint[n - 1], 2);

        dkAcc[0].setZero();
        dkAcc[0](2, 2) = -2 / pow(toolpath->dTimePoint[1], 3);
        dkAcc[0](3, 2) = 2 / pow(toolpath->dTimePoint[1], 3) + 1 / pow(toolpath->dTimePoint[1], 2) / toolpath->dTimePoint[2];
        dkAcc[0](3, 3) = 1 / pow(toolpath->dTimePoint[2], 2) / toolpath->dTimePoint[1];
        dkAcc[0](4, 2) = -1 / pow(toolpath->dTimePoint[1], 2) / toolpath->dTimePoint[2];
        dkAcc[0](4, 3) = -1 / pow(toolpath->dTimePoint[2], 2) / toolpath->dTimePoint[1];

        dkAcc[n - 1].setZero();
        dkAcc[n - 1](0, 0) = -1 / pow(toolpath->dTimePoint[n - 2], 2) / toolpath->dTimePoint[n - 1];
        dkAcc[n - 1](0, 1) = -1 / pow(toolpath->dTimePoint[n - 1], 2) / toolpath->dTimePoint[n - 2];
        dkAcc[n - 1](1, 0) = 1 / pow(toolpath->dTimePoint[n - 2], 2) / toolpath->dTimePoint[n - 1];
        dkAcc[n - 1](1, 1) = 1 / pow(toolpath->dTimePoint[n - 1], 2) / toolpath->dTimePoint[n - 2] + 2 / pow(toolpath->dTimePoint[n - 1], 3);
        dkAcc[n - 1](2, 1) = -2 / pow(toolpath->dTimePoint[n - 1], 3);

        Curvature.resize(n);
        Curvature(0) = 0;
        Curvature(n - 1) = 0;

        toolpath->getCosAngle();
    }


    double minV = DBL_MAX, minA = DBL_MAX, minJ = DBL_MAX;
    double maxV = -DBL_MAX, maxA = -DBL_MAX, maxJ = -DBL_MAX;
 
    for (int i = 1; i < n - 1; i++) {
        std::vector<Eigen::VectorXd> localJoint;
        std::vector<double> localDist;

        localJoint.resize(3);
        localJoint[0] = (Eigen::VectorXd)Joints[i - 1];
        //std::cout << localJoint[0](0) << " " << localJoint[0](7) << std::endl;
        localJoint[1] = Joints[i];
        //std::cout << localJoint[1](0) << " " << localJoint[1](7) << std::endl;
        localJoint[2] = Joints[i + 1];
        localDist.resize(2);
        localDist.assign(toolpath->dTimePoint.begin() + i, toolpath->dTimePoint.begin() + i + 2);

        Eigen::VectorXd X;
        X.resize(8);
        X = velNum(localJoint, localDist, kVel[i]);
        Vel[i] = X;
        double tmp = X.transpose() * M * X;
        if (tmp < minV) {
            minV = tmp;
        }

        if (tmp > maxV) {
            maxV = tmp;
        }

        X = accNum(localJoint, localDist, kAcc[i]);
        Acc[i] = X;
        tmp = X.transpose() * M * X;
        if (tmp < minA) {
            minA = tmp;
        }
        if (tmp > maxA) {
            maxA = tmp;
        }

        if (optTime) {
            dkVel[i] = getdkVel(localDist);
            dkAcc[i] = getdkAcc(localDist);

            localDist.assign(toolpath->dPathPoint.begin() + i, toolpath->dPathPoint.begin() + i + 2);
            std::vector<Eigen::VectorXd> localPos(3);
            localPos[0] = toolpath->pathPoint[i - 1];
            localPos[1] = toolpath->pathPoint[i];
            localPos[2] = toolpath->pathPoint[i + 1];

            Eigen::VectorXd pD2s;
            Eigen::Matrix<double, 5, 1> tmp;

            pD2s = accNum(localPos, localDist, tmp);
            Curvature(i) = pD2s.norm();
        }


        if (i > 1 && i < n - 2) {
            localJoint.resize(5);
            localJoint[0] = Joints[i - 2];
            localJoint[1] = Joints[i - 1];
            localJoint[2] = Joints[i];
            localJoint[3] = Joints[i + 1];
            localJoint[4] = Joints[i + 2];
            localDist.resize(4);
            localDist.assign(toolpath->dTimePoint.begin() + i - 1, toolpath->dTimePoint.begin() + i + 3);

            X = jerkNum(localJoint, localDist, kJerk[i]);
            Jerk[i] = X;
            tmp = X.transpose() * M * X;
            if (tmp < minJ) {
                minJ = tmp;
            }
            if (tmp > maxJ) {
                maxJ = tmp;
            }

            if (optTime) {
				dkJerk[i] = getdkJerk(localDist);
			}
        }

        
    }

    //Jerk[0] = Jerk[2];
    //kJerk[0] = kJerk[2];
    //Jerk[1] = Jerk[2];
    //kJerk[1] = kJerk[2];
    //Jerk[n - 2] = Jerk[n - 3];
    //kJerk[n - 2] = kJerk[n - 3];
    //Jerk[n - 1] = Jerk[n - 3];
    //kJerk[n - 1] = kJerk[n - 3];

    Jerk[0].setZero();
    Jerk[1].setZero();
    Jerk[n-2].setZero();
    Jerk[n-1].setZero();
    kJerk[0].setZero();
    kJerk[1].setZero();
    kJerk[n - 2].setZero();
    kJerk[n - 1].setZero();

    if (optTime) {
		dkJerk[0].setZero();
		dkJerk[1].setZero();
		dkJerk[n - 2].setZero();
		dkJerk[n - 1].setZero();
	}

    dVel = maxV- minV;
    dAcc = maxA - minA;
    dJerk = maxJ - minJ;

    
    if (kSet.sum() < 0.01) {
        if (optTime) {
            k1 = 0;
            k2 = 0.5;
            k3 = 1;
        }
        else {
            k1 = 0.4;
            k2 = 0.8;
            k3 = 1.2;
        }
    }
    else
    {
        k1 = kSet(0);
		k2 = kSet(1);
		k3 = kSet(2);

    }
    std::cout << "set k1 = " << k1 << ", k2 = " << k2 << ", k3 = " << k3 << std::endl;

    phi_i.resize(n);

    for (int i = 0; i < n; i++) {
        double tmp = Vel[i].transpose() * M * Vel[i];
        double phi = k1 * tmp / dVel;
        tmp= Acc[i].transpose() * M * Acc[i];
        phi = phi + k2 * tmp / dAcc;
        tmp = Jerk[i].transpose() * M * Jerk[i];
        phi = phi + k3 * tmp / dJerk;

        if (i == n - 1) {
            phi_i[i] = phi * (toolpath->dPathPoint[i]) / 2;
        }
        else {
            phi_i[i] = phi * (toolpath->dPathPoint[i] + toolpath->dPathPoint[i + 1]) / 2;
        }
        
    }

    localTime.resize(n);
    for (int i = 0; i < n; i++) {
        localTime(i) = toolpath->dTimePoint[i];
	}

    if (optTime) {
        Eigen::VectorXd vTTP(n - 1), aTTP_T(n), aTTP_N(n);
        for (int i = 0; i < n; i++) {
            if (i > 0) {
                vTTP(i - 1) = toolpath->dPathPoint[i] / localTime(i);
            }

            if ((i == 0) || (i == n - 1)) {
                aTTP_T(i) = 0;
                aTTP_N(i) = 0;
            }
            else {
                double tmp1 = toolpath->dPathPoint[i + 1] * localTime(i), tmp2 = toolpath->dPathPoint[i] * localTime(i + 1);
                aTTP_T(i) = 2 * (tmp1 - tmp2) / (localTime(i) * localTime(i + 1) * (localTime(i) + localTime(i + 1)));

                double vBarSqure = (pow(tmp1, 2) + pow(tmp2, 2) - 2 * tmp1 * tmp2 * toolpath->cosAngle[i]) / (4 * pow(localTime(i), 2) * pow(localTime(i + 1), 2));
                aTTP_N(i) = Curvature(i) * vBarSqure;
            }
        }

        velTTP = vTTP;
        acctTTP = aTTP_T;
        accnTTP = aTTP_N;
    }


    Theta.resize(5 * n);
    Theta.setZero();
    Ttcps.resize(n);
    Tpos.resize(n);

    for (int i = 0; i < n; i++) {
        Eigen::Matrix<double, 6, 1> Jtmp;
        Jtmp = Joints[i].block<6, 1>(0, 0);
        Transform3d Ttmp = robs->fkABB2600(Jtmp);
        Ttmp.matrix() = Ttmp.matrix() * robs->Ttool.matrix();
        Ttcps[i] = Ttmp;
        Eigen::Matrix3d Rtmp = Ttmp.linear();
        Eigen::Quaterniond qtmp(Rtmp);
        Rtmp=qtmp.toRotationMatrix();
        Ttcps[i].linear() = Rtmp;

        Eigen::Matrix3d wHat = mathTools::logm(Rtmp);
        Eigen::Vector3d w = mathTools::vex(wHat);

        for (int j = 0; j < 3; j++) {
            Theta(i * 5 + j) = w(j);
        }
        Theta(i * 5 + 3) = Joints[i](6);
        Theta(i * 5 + 4) = Joints[i](7);

        Tpos[i] = robs->fkPos(Joints[i].block<2, 1>(6, 0));
    }

    std::cout << "initial smoothness index: " << phi_i.sum() << std::endl;

    if (optTime) {
        std::cout << "initial time: " << localTime.sum() << std::endl;

        dTime = localTime.maxCoeff() - localTime.minCoeff();

        double objTime = localTime.sum() / dTime;
        if (phi_i.sum() / objTime > 50) {
            kTime = round(phi_i.sum() / objTime / 10);
        }
        else if (phi_i.sum() / objTime < 0.02) {
            kTime =  phi_i.sum() / objTime;
        }

        std::cout << "kTime: " << kTime << std::endl;
        
        if (kSet(3) > 0) {
            kTime = kSet(3) * kTime;
            std::cout<<"set kTime: "<<kTime<<std::endl;
		}

        //kTime = kTime * 100;
    }

    /*fileIO* IO = new fileIO();
    IO->writeVectorXd(phi_i, "../DataSet/phi_i_" + std::to_string(toolpath->length));
    IO->writeVar8(Vel, "../DataSet/Vel_" + std::to_string(toolpath->length));
    delete IO;*/

    status = 2;

    std::cout << "Initialize optimization done\n" << std::endl;

}


void trajOpt::bcd(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool checkColi, int sizeWindow, bool show) {
    std::cout << "\nBlock Coordinate Descent" << std::endl;

    if (status != 2) {
		std::cerr << "status wrong in bcd" << std::endl;
	}

    clock_t Tstart, Tfinish;
    double time;
    int n = toolpath->length;

    //std::vector<Transform3d> TtcpsSave=Ttcps, TposSave=Tpos;
    //std::vector<Eigen::Matrix<double, 8, 1>> VelSave=Vel, AccSave=Acc, JerkSave=Jerk;
    //Eigen::VectorXd phiSave=phi_i;
    Eigen::VectorXd ThetaSave0=Theta;
    std::vector<Eigen::Matrix<double, 8, 1>> JointsSave0 = Joints;


    std::cout << "initial: " << phi_i.sum() << std::endl;


    Eigen::Matrix<double, 8, 1> jMaxTmp = Jerk[0].cwiseAbs();
    for (int i = 0; i < n; i++) {
        jMaxTmp=jMaxTmp.cwiseMax(Jerk[i].cwiseAbs());
    }
    std::cout << "max jerk" << jMaxTmp.transpose() << std::endl;

    std::vector<double> iterTime, iterPhi;
    iterTime.push_back(0);
    iterPhi.push_back(phi_i.sum());

    //fileIO* IO = new fileIO();
    //IO->writeVar8(Joints, "../DataSet/Joints0.txt");
    //IO->writeVar8(Vel, "../DataSet/Vel0.txt");
    //IO->writeVar8(Acc, "../DataSet/Acc0.txt");
    //IO->writeVar8(Jerk, "../DataSet/Jerk0.txt");
    //delete IO;

    int size_bcd = sizeWindow;
    
    if (n < size_bcd) {
		size_bcd = n;
	}
    
    
    std::vector<Eigen::Vector2d> indexSave1;
    std::vector<Eigen::Vector2d> indexSave2;
    int a = 0;
    int b = size_bcd - 1;
    while (true) {
        
        indexSave1.push_back(Eigen::Vector2d(a, b));
        
        a = b + 1;
        b = a + 3;
        
        a = a - size_bcd / 2 + 2;
        b = b + size_bcd / 2 - 2;

        if (b >= n - 4) {
			b = n - 1;
            indexSave2.push_back(Eigen::Vector2d(a, b));
            break;
		}

        indexSave2.push_back(Eigen::Vector2d(a, b));


        a = a + size_bcd / 2 - 2;
        b = b - size_bcd / 2 + 2;


        a = b + 1;
        b = a + size_bcd - 1;

        if (b >= n - 4) {
            b = n - 1;
            indexSave1.push_back(Eigen::Vector2d(a, b));
            break;
        }
    }

    double Phi_1 = phi_i.sum();
    double Phi_0 = 2 * Phi_1;

    Tstart = clock();
    while ((Phi_0 - Phi_1) / Phi_0 > tau2 && iterTime.size() < 10) {
        Phi_0 = Phi_1;

        std::vector<updateInfo> InfoSave(indexSave1.size());

#pragma omp parallel for
        for (int i = 0; i < indexSave1.size(); i++) {
            std::cout << "." << std::flush;

            int start = indexSave1[i](0);
            int end = indexSave1[i](1);

            //std::cout<<i<<" "<<start<<" "<<end<<std::endl;

            updateInfo Info;

            bool success;

            local_optimal(toolpath, robs, colli, success, start, end, Info, checkColi, show);

            InfoSave[i] = Info;
        }

        for (int i = 0; i < indexSave1.size(); i++) {
            updateInfo Info = InfoSave[i];
            int start = std::max(0, Info.a - 2);
            int end = std::min(n - 1, Info.b + 2);

            std::vector<Eigen::VectorXd> thetaTmp(end - start + 1);

#pragma omp parallel for
            for (int j = start; j <= end; j++) {
                phi_i(j) = Info.phiiUpdate(j);
                Vel[j] = Info.VelUpdate[j];
                Acc[j] = Info.AccUpdate[j];
                Jerk[j] = Info.JerkUpdate[j];

                if (j >= Info.a && j <= Info.b) {
                    Joints[j] = Info.JointsUpdate[j];
                    Ttcps[j] = Info.TtcpsUpdate[j];
                    Tpos[j] = Info.TposUpdate[j];
                    
                    thetaTmp[j - start] = Info.ThetaUpdate.segment(j * 5, 5);

                    //Theta.segment(j * 5, 5) = Info.ThetaUpdate.segment(j * 5, 5);
                }
            }

            for (int j = Info.a; j <= Info.b; j++) {
                Theta.segment(j * 5, 5) = thetaTmp[j - start];
			}
        }

        //Tfinish = clock();
        //time = (double)(Tfinish - Tstart) / CLOCKS_PER_SEC;
        //Phi_1 = phi_i.sum();
        //std::cout << "BCD 5_1 time: " << time << std::endl;
        //std::cout << "Solution 5_1: " << Phi_1 << std::endl;



        InfoSave.clear();
        InfoSave.resize(indexSave2.size());

#pragma omp parallel for
        for (int i = 0; i < indexSave2.size(); i++) {
            std::cout << "." << std::flush;

            int start = indexSave2[i](0);
            int end = indexSave2[i](1);

            //std::cout << i << " " << start << " " << end << std::endl;

            bool success;

            updateInfo Info;

            local_optimal(toolpath, robs, colli, success, start, end, Info, checkColi, show);

            InfoSave[i] = Info;
        }

        for (int i = 0; i < indexSave2.size(); i++) {
            updateInfo Info = InfoSave[i];
            int start = std::max(0, Info.a - 2);
            int end = std::min(n - 1, Info.b + 2);

            std::vector<Eigen::VectorXd> thetaTmp(end - start + 1);

#pragma omp parallel for
            for (int j = start; j <= end; j++) {
                phi_i(j) = Info.phiiUpdate(j);
                Vel[j] = Info.VelUpdate[j];
                Acc[j] = Info.AccUpdate[j];
                Jerk[j] = Info.JerkUpdate[j];

                if (j >= Info.a && j <= Info.b) {
                    Joints[j] = Info.JointsUpdate[j];
                    Ttcps[j] = Info.TtcpsUpdate[j];
                    Tpos[j] = Info.TposUpdate[j];

                    thetaTmp[j - start] = Info.ThetaUpdate.segment(j * 5, 5);

                    //Theta.segment(j * 5, 5) = Info.ThetaUpdate.segment(j * 5, 5);
                }
            }

            for (int j = Info.a; j <= Info.b; j++) {
				Theta.segment(j * 5, 5) = thetaTmp[j - start];
			}
        }

        Tfinish = clock();
        time = (double)(Tfinish - Tstart) / CLOCKS_PER_SEC;
        Phi_1 = phi_i.sum();

        iterTime.push_back(time);
        iterPhi.push_back(Phi_1);

        std::cout << "\nBCD 5_2 time: " << time << std::endl;
        std::cout << "Solution 5_2: " << Phi_1 << std::endl;

        Eigen::Matrix<double, 8, 1> jMaxTmp = Jerk[0].cwiseAbs();
        for (int i = 0; i < n; i++) {
            jMaxTmp = jMaxTmp.cwiseMax(Jerk[i].cwiseAbs());
        }
        std::cout << "max jerk" << jMaxTmp.transpose() << std::endl;
	}
    
    //fileIO* IO = new fileIO();
    //IO->writeTwoVectors(iterTime, iterPhi, "../DataSet/some_results/iter_2.txt");
    //delete IO;


    //IO = new fileIO();
    //IO->writeVar8(Joints, "../DataSet/Joints1.txt");
    //IO->writeVar8(Vel, "../DataSet/Vel1.txt");
    //IO->writeVar8(Acc, "../DataSet/Acc1.txt");
    //IO->writeVar8(Jerk, "../DataSet/Jerk1.txt");
    //delete IO;


 

    //if (true) {
    //    std::cout << "start BCD 1" << std::endl;

    //    Joints = JointsSave;
    //    Ttcps = TtcpsSave;
    //    Tpos = TposSave;
    //    Vel = VelSave;
    //    Acc = AccSave;
    //    Jerk = JerkSave;
    //    phi_i = phiSave;
    //    Theta = ThetaSave;


    //    int a = 0, b = n - 1;
    //    bool succTmp;
    //    updateInfo Infotmp;
    //    Tstart = clock();
    //    local_optimal(toolpath, robs, colli, succTmp, a, b, Infotmp, true);
    //    Tfinish = clock();
    //    time = (double)(Tfinish - Tstart) / CLOCKS_PER_SEC;
    //    std::cout << "BCD 1 time: " << time << std::endl;
    //    std::cout << "Solution 1: " << Infotmp.phiiUpdate.sum() << std::endl;
    //}


    //if (!checkColi) {
        std::cout<<"check collision..."<<std::endl;
        for (int i = 0; i < n; i++) {
            if (i % 100 == 0) {
				std::cout << "." << std::flush;
			}

            Eigen::Matrix<double, 8, 1> tmp = Joints[i];
            bool coli = robs->checkAllColi(tmp);

            if (coli) {
                findFeasible(i, ThetaSave0.segment(i * 5, 5), JointsSave0[i], robs, toolpath);
            }
        }
        std::cout<<std::endl;

        std::cout << "After collision correction: " << phi_i.sum() << std::endl;
    //}

    std::cout << "BCD done\n" << std::endl;

}

void trajOpt::bcd_time(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool checkColi, int sizeWindow, bool show) {
    std::cout << "\nBlock Coordinate Descent" << std::endl;

    if (status != 2) {
        std::cerr << "status wrong in bcd" << std::endl;
    }


    clock_t Tstart, Tfinish;
    double time;
    int n = toolpath->length;


    Eigen::VectorXd ThetaSave0 = Theta;
    std::vector<Eigen::Matrix<double, 8, 1>> JointsSave0 = Joints;

    std::cout << "initial smoothness index: " << phi_i.sum() << std::endl;
    std::cout << "initial time: " << localTime.sum() << std::endl;

    std::vector<double> iterTime, iterPhi;
    iterTime.push_back(0);
    iterPhi.push_back(phi_i.sum());

    int size_bcd = sizeWindow;

    if (n < size_bcd) {
		size_bcd = n;
	}

    std::vector<Eigen::Vector2d> indexSave1;
    std::vector<Eigen::Vector2d> indexSave2;
    int a = 0;
    int b = size_bcd - 1;
    while (true) {
        indexSave1.push_back(Eigen::Vector2d(a, b));

        a = b + 1;
        b = a + 3;

        a = a - size_bcd / 2 + 2;
        b = b + size_bcd / 2 - 2;

        if (b >= n - 4) {
            b = n - 1;
            indexSave2.push_back(Eigen::Vector2d(a, b));
            break;
        }

        indexSave2.push_back(Eigen::Vector2d(a, b));


        a = a + size_bcd / 2 - 2;
        b = b - size_bcd / 2 + 2;


        a = b + 1;
        b = a + size_bcd - 1;

        if (b >= n - 4) {
            b = n - 1;
            indexSave1.push_back(Eigen::Vector2d(a, b));
            break;
        }
    }

    double Phi_1 = phi_i.sum() + kTime * localTime.sum() / dTime;
    double Phi_0 = 2 * Phi_1;

    bool flagAccN = true;

    Tstart = clock();
    while (((Phi_0 - Phi_1) / Phi_0 > tau2 || flagAccN) && iterTime.size() < 5) {
        Phi_0 = Phi_1;

        if (accnTTP.cwiseAbs().maxCoeff() < aTTPMax_N) {
            flagAccN = false;
        }


        std::vector<updateInfo> InfoSave(indexSave1.size());

#pragma omp parallel for
        for (int i = 0; i < indexSave1.size(); i++) {
            std::cout << "." << std::flush;

            int start = indexSave1[i](0);
            int end = indexSave1[i](1);

            updateInfo Info;

            bool success;

            local_optimal_time(toolpath, robs, colli, success, start, end, Info, checkColi, show);

            InfoSave[i] = Info;
        }

        for (int i = 0; i < indexSave1.size(); i++) {
            updateInfo Info = InfoSave[i];
            int start = std::max(0, Info.a - 2);
            int end = std::min(n - 1, Info.b + 2);

            std::vector<Eigen::VectorXd> thetaTmp(end - start + 1);

#pragma omp parallel for
            for (int j = start; j <= end; j++) {
                phi_i(j) = Info.phiiUpdate(j);
                Vel[j] = Info.VelUpdate[j];
                Acc[j] = Info.AccUpdate[j];
                Jerk[j] = Info.JerkUpdate[j];

                kVel[j] = Info.kVelUpdate[j];
                kAcc[j] = Info.kAccUpdate[j];
                kJerk[j] = Info.kJerkUpdate[j];

                dkVel[j] = Info.dkVelUpdate[j];
                dkAcc[j] = Info.dkAccUpdate[j];
                dkJerk[j] = Info.dkJerkUpdate[j];

                if (j >= Info.a && j <= Info.b) {
                    Joints[j] = Info.JointsUpdate[j];
                    Ttcps[j] = Info.TtcpsUpdate[j];
                    Tpos[j] = Info.TposUpdate[j];

                    localTime(j) = Info.localTimeUpdate(j);

                    thetaTmp[j - start] = Info.ThetaUpdate.segment(j * 5, 5);
                }
            }

            for (int j = Info.a; j <= Info.b; j++) {
                Theta.segment(j * 5, 5) = thetaTmp[j - start];
            }

            velTTP.segment(Info.a, Info.b - Info.a) = Info.velTTPUpdate;
            acctTTP.segment(Info.a, Info.b - Info.a + 1) = Info.acctTTPUpdate;
            accnTTP.segment(Info.a, Info.b - Info.a + 1) = Info.accnTTPUpdate;

        }

        InfoSave.clear();
        InfoSave.resize(indexSave2.size());

#pragma omp parallel for
        for (int i = 0; i < indexSave2.size(); i++) {
            std::cout << "." << std::flush;

            int start = indexSave2[i](0);
            int end = indexSave2[i](1);

            bool success;

            updateInfo Info;

            local_optimal_time(toolpath, robs, colli, success, start, end, Info, checkColi, show);

            InfoSave[i] = Info;
        }

        for (int i = 0; i < indexSave2.size(); i++) {
            updateInfo Info = InfoSave[i];
            int start = std::max(0, Info.a - 2);
            int end = std::min(n - 1, Info.b + 2);

            std::vector<Eigen::VectorXd> thetaTmp(end - start + 1);

#pragma omp parallel for
            for (int j = start; j <= end; j++) {
                phi_i(j) = Info.phiiUpdate(j);
                Vel[j] = Info.VelUpdate[j];
                Acc[j] = Info.AccUpdate[j];
                Jerk[j] = Info.JerkUpdate[j];

                kVel[j] = Info.kVelUpdate[j];
                kAcc[j] = Info.kAccUpdate[j];
                kJerk[j] = Info.kJerkUpdate[j];

                dkVel[j] = Info.dkVelUpdate[j];
                dkAcc[j] = Info.dkAccUpdate[j];
                dkJerk[j] = Info.dkJerkUpdate[j];

                if (j >= Info.a && j <= Info.b) {
                    Joints[j] = Info.JointsUpdate[j];
                    Ttcps[j] = Info.TtcpsUpdate[j];
                    Tpos[j] = Info.TposUpdate[j];

                    localTime(j) = Info.localTimeUpdate(j);

                    thetaTmp[j - start] = Info.ThetaUpdate.segment(j * 5, 5);
                }
            }

            for (int j = Info.a; j <= Info.b; j++) {
                Theta.segment(j * 5, 5) = thetaTmp[j - start];
            }

            velTTP.segment(Info.a, Info.b - Info.a) = Info.velTTPUpdate;
            acctTTP.segment(Info.a, Info.b - Info.a + 1) = Info.acctTTPUpdate;
            accnTTP.segment(Info.a, Info.b - Info.a + 1) = Info.accnTTPUpdate;
        }

        Tfinish = clock();
        time = (double)(Tfinish - Tstart) / CLOCKS_PER_SEC;
        Phi_1 = phi_i.sum() + kTime * localTime.sum() / dTime;

        iterTime.push_back(time);
        iterPhi.push_back(Phi_1);

        std::cout << "\nOptimization time: " << time << std::endl;
        std::cout << "smoothness index: " << phi_i.sum() << std::endl;
        std::cout << "printing time: " << localTime.sum() << std::endl;
        std::cout << "obj: " << Phi_1 << std::endl;


    }


    //if (!checkColi) {
        std::cout << "check collision..." << std::endl;
        for (int i = 0; i < n; i++) {
            if (i % 100 == 0) {
                std::cout << "." << std::flush;
            }

            Eigen::Matrix<double, 8, 1> tmp = Joints[i];
            bool coli = robs->checkAllColi(tmp);

            if (coli) {
                findFeasible(i, ThetaSave0.segment(i * 5, 5), JointsSave0[i], robs, toolpath);
            }
        }
        std::cout << std::endl;

        std::cout << "After collision correction: " << phi_i.sum() << std::endl;
    //}

    std::cout << "BCD done\n" << std::endl;

}

void trajOpt::local_optimal(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool& success, int a, int b, updateInfo& Info, bool checkColi, bool show) {

    std::vector<Eigen::Matrix<double,8,1>> JointsUpdate=Joints;
    std::vector<Transform3d> TtcpsUpdate=Ttcps, TposUpdate=Tpos;
    std::vector<Eigen::Matrix<double,8,1>> VelUpdate=Vel, AccUpdate=Acc, JerkUpdate=Jerk;
    Eigen::VectorXd phiiUpdate=phi_i;
    Eigen::VectorXd ThetaUpdate=Theta;

    const int aL = std::max(0, a - 2);
    const int bH = std::min(toolpath->length - 1, b + 2);
    const int nLH = bH - aL + 1;

    Eigen::VectorXd local_phi_i = phiiUpdate.segment(aL, nLH);
    double Phi_1 = local_phi_i.sum();
    double Phi_0 = 2 * Phi_1;
    double Phi_ini = Phi_1;

    clock_t Tstart, Tfinish;
    if (show) {
        Tstart = clock();
        std::cout << "start SQP  The initial: " << Phi_1 << std::endl;
    }


    Eigen::Vector3d G(0, 0, -1);

    const int n = b - a + 1;
    //const double kInfinity = std::numeric_limits<double>::infinity();

    //std::cout<<"1"<<std::endl;

    SVMGrad getBase;
    std::vector<double> tauLayer, tauBase;
    std::vector<Eigen::VectorXd> diffTauLayer, diffTauBase;
    if (checkColi) {
        tauLayer.resize(n);
        tauBase.resize(n);
        diffTauLayer.resize(n);
        diffTauBase.resize(n);

        string modelFile = "../DataSet/coliTrain/BaseColi/BaseColiFK.modelG";
        getBase.loadModel(modelFile);
        getBase.preComputeKernel(true);

        for (int i = 0; i < n; i++) {
            double tau = getFastronScore(JointsUpdate[a + i], colli->supportData, colli->alpha, diffTauLayer[i], robs);
            tauLayer[i] = tau;

            Eigen::MatrixXd posDq;
            Eigen::VectorXd tmp = robs->Joints2Postions(JointsUpdate[a + i], true, posDq);
            getBase.calculateGammaAndDerivative(tmp, tauBase[i], diffTauBase[i]);
            diffTauBase[i] = posDq.transpose() * diffTauBase[i];
        }
    }

    //std::cout<<"2"<<std::endl;

    Eigen::MatrixXd AdTheta(5 * n, 5 * n);
    AdTheta.setIdentity();
    Eigen::VectorXd UdTheta(5 * n);
    for (int i = 0; i < n; i++) {
        Eigen::VectorXd tmp(5);
        tmp << 0.01, 0.01, 0.01, 0.1, 0.1;
        UdTheta.block<5, 1>(5 * i, 0) = tmp;
    }

    success = false;

    Eigen::Matrix<double, 8, 1> maxVel, maxAcc, maxJerk;
    maxVel = vJointMax;
    maxAcc = aJointMax;
    maxJerk = jJointMax;
    for (int i = aL; i <= bH; i++) {
        maxVel = maxVel.cwiseMax(VelUpdate[i].cwiseAbs());
        maxAcc = maxAcc.cwiseMax(AccUpdate[i].cwiseAbs());
        maxJerk = maxJerk.cwiseMax(JerkUpdate[i].cwiseAbs());
    }
    //std::cout<<"maxVel: "<<maxVel.transpose()<<std::endl;
    //std::cout<<"maxAcc: "<<maxAcc.transpose()<<std::endl;
    //std::cout<<"maxJerk: "<<maxJerk.transpose()<<std::endl;

    //SQP
    while ((Phi_0 - Phi_1) / Phi_0 > tau2) {
        if (show) {
            std::cout << "in SQP" << std::endl;
        }


        Phi_0 = Phi_1;
        //std::cout << "before optimization: " << Phi_0 << std::endl;

        std::vector<Eigen::Matrix<double, 6, 3>> qDp(n), qDw(n);
        std::vector<Eigen::Matrix<double, 3, 2>> pDbc(n);
        std::vector<Eigen::Matrix<double, 6, 2>> qDbc(n);
        std::vector<Eigen::Matrix<double, 8, 5>> JointDTheta(n);


        Eigen::MatrixXd Anozzle(n, 5 * n), Anormal(n, 5 * n), Ann(n, 5 * n);
        Eigen::VectorXd Lnozzle(n), Unozzle(n), Lnormal(n), Unormal(n), Lnn(n), Unn(n);
        /*Unozzle.setZero();
        Lnormal.setZero();
        Lnn.setZero();*/
        Unozzle.setOnes();
        Unozzle = Unozzle * std::numeric_limits<double>::infinity();
        Lnormal.setOnes();
        Lnormal = -Lnormal * std::numeric_limits<double>::infinity();
        Lnn.setOnes();
        Lnn = -Lnn * std::numeric_limits<double>::infinity();

        Eigen::MatrixXd Acoli, AcoliBase;
        Eigen::VectorXd Lcoli, Ucoli, LcoliBase, UcoliBase;
        if (checkColi) {
            Acoli.resize(n, 5 * n);
            AcoliBase.resize(n, 5 * n);
            Lcoli.resize(n);
            Ucoli.resize(n);
            LcoliBase.resize(n);
            UcoliBase.resize(n);

            Ucoli.setZero();
            Acoli.setZero();
            Lcoli.setOnes();
            Lcoli = -Lcoli * std::numeric_limits<double>::infinity();
            UcoliBase.setZero();
            AcoliBase.setZero();
            LcoliBase.setOnes();
            LcoliBase = -LcoliBase * std::numeric_limits<double>::infinity();
        }
        


        double maxTau = 0;

        for (int i = a; i < b + 1; i++) {
            Eigen::Matrix<double, 6, 1> qtmp = JointsUpdate[i].block<6, 1>(0, 0);
            //Eigen::Matrix<double, 6, 6> J_base, J_bar;
            //std::vector<Eigen::Matrix<double, 6, 6>> dJ_base, dJ_bar;
            //J_base = robs->JaS_ABB(qtmp, dJ_base, robs->Ttool);
            Eigen::Matrix<double, 6, 6> J = robs->JaB_ABB(qtmp, robs->Ttool);


            Eigen::Vector3d w = ThetaUpdate.segment(i * 5, 3);
            Eigen::Vector2d BC = ThetaUpdate.segment(i * 5 + 3, 2);

            Eigen::MatrixXd NnozzleDw(3, 5 * n), NnormalDw(3, 5 * n);
            NnozzleDw.setZero();
            NnormalDw.setZero();


            for (int j = 0; j < 3; j++) {
                Eigen::Vector3d dw = Eigen::Vector3d::Zero();
                dw(j) = 1;

                Eigen::Matrix4d dT = Eigen::Matrix4d::Zero();
                dT.block<3, 1>(0, 3) = dw;
                Eigen::Matrix4d vbHat = TtcpsUpdate[i].matrix().inverse() * dT;
                Eigen::Matrix<double, 6, 1> vb = mathTools::vex(vbHat);
                qDp[i - a].block<6, 1>(0, j) = J.inverse() * vb;

                Eigen::Matrix3d dR = mathTools::diffRotm(w, dw);
                dT = Eigen::Matrix4d::Zero();
                dT.block<3, 3>(0, 0) = dR;
                vbHat = TtcpsUpdate[i].matrix().inverse() * dT;
                vb = mathTools::vex(vbHat);
                qDw[i - a].block<6, 1>(0, j) = J.inverse() * vb;

                NnozzleDw.block<3, 1>(0, 5 * (i - a) + j) = dR.block<3, 1>(0, 2);

            }

            std::vector<Eigen::Matrix4d> dTbc = robs->dfkPos(BC);
            for (int j = 0; j < 2; j++) {
                Eigen::Vector4d posiTmp = dTbc[j] * toolpath->posWorkpiece[i].matrix().block<4, 1>(0, 3);
                pDbc[i - a].block<3, 1>(0, j) = posiTmp.block<3, 1>(0, 0);

                NnormalDw.block<3, 1>(0, 5 * (i - a) + j + 3) = dTbc[j].block<3, 3>(0, 0) * toolpath->pathNormal[i];
            }


            qDbc[i - a] = qDp[i - a] * pDbc[i - a];

            JointDTheta[i - a] = Eigen::Matrix<double, 8, 5>::Zero();
            JointDTheta[i - a].block<6, 3>(0, 0) = qDw[i - a];
            JointDTheta[i - a].block<6, 2>(0, 3) = qDbc[i - a];
            JointDTheta[i - a].block<2, 2>(6, 3) = Eigen::Matrix2d::Identity();


            Eigen::Vector3d Nnormal, Nnozzle;
            Nnozzle = TtcpsUpdate[i].matrix().block<3, 1>(0, 2);
            Nnormal = TposUpdate[i].linear() * toolpath->pathNormal[i];

            Anozzle.block(i - a, 0, 1, 5 * n) = G.transpose() * NnozzleDw;
            Lnozzle(i - a) = beta - G.transpose() * Nnozzle;

            Anormal.block(i - a, 0, 1, 5 * n) = G.transpose() * NnormalDw;
            Unormal(i - a) = -alpha - G.transpose() * Nnormal;

            Ann.block(i - a, 0, 1, 5 * n) = Nnormal.transpose() * NnozzleDw + Nnozzle.transpose() * NnormalDw;
            Unn(i - a) = -gamma - Nnormal.dot(Nnozzle);


            if (checkColi) {
                double tau = tauLayer[i - a];
                if (tau > errorTol) {
                    cout << "tau: " << tau << endl;
                }

                Acoli.block(i - a, 5 * (i - a), 1, 5) = diffTauLayer[i - a].transpose() * JointDTheta[i - a];
                Ucoli(i - a) = errorTol - tau;

                AcoliBase.block(i - a, 5 * (i - a), 1, 5) = diffTauBase[i - a].transpose() * JointDTheta[i - a];
                UcoliBase(i - a) = errorTolBase - tauBase[i - a];
            }
        }
        //std::cout << "maxTau: " << maxTau << std::endl;

        Eigen::MatrixXd H(5 * n, 5 * n);
        H.setZero();
        Eigen::VectorXd f(5 * n);
        f.setZero();


        Eigen::MatrixXd Av(8 * (nLH), 5 * n), Aa(8 * (nLH), 5 * n), Aj(8 * (nLH), 5 * n), Ajoint(8 * n, 5 * n);
        Eigen::VectorXd Lv(8 * (nLH)), Uv(8 * (nLH)), La(8 * (nLH)), Ua(8 * (nLH)), Lj(8 * (nLH)), Uj(8 * (nLH)), Ljoint(8 * n), Ujoint(8 * n);


        for (int i = aL; i < bH + 1; i++) {
            Eigen::MatrixXd vDtheta(8, 5 * n), aDtheta(8, 5 * n), jDtheta(8, 5 * n);
            vDtheta.setZero();
            aDtheta.setZero();
            jDtheta.setZero();

            int j1 = 0, j2 = 5;
            if (a + 2 - i > 0) {
                j1 = a + 2 - i;
            }
            if (b + 3 - i < 5) {
                j2 = b + 3 - i;
            }

            for (int j = j1; j < j2; j++) {
                Eigen::MatrixXd jDthetaTmp(8, 5 * n);
                jDthetaTmp.setZero();
                jDthetaTmp.block<8, 5>(0, 5 * (i - a - 2 + j)) = JointDTheta[i - a - 2 + j];

                Eigen::MatrixXd eye(8, 8);
                eye.setIdentity();

                vDtheta = vDtheta + kVel[i](j) * eye * jDthetaTmp;
                aDtheta = aDtheta + kAcc[i](j) * eye * jDthetaTmp;
                jDtheta = jDtheta + kJerk[i](j) * eye * jDthetaTmp;
            }

            Eigen::MatrixXd Htmp(5 * n, 5 * n);
            Htmp.setZero();
            Htmp = k1 * vDtheta.transpose() * M * vDtheta / dVel + k2 * aDtheta.transpose() * M * aDtheta / dAcc + k3 * jDtheta.transpose() * M * jDtheta / dJerk;
            if (i == toolpath->length - 1) {
                Htmp = Htmp * toolpath->dPathPoint[i] / 2;
            }
            else {
                Htmp = Htmp * (toolpath->dPathPoint[i] + toolpath->dPathPoint[i + 1]) / 2;
            }
            Eigen::MatrixXd Htmp2 = Htmp.transpose();
            Htmp = Htmp + Htmp2;
            H = H + Htmp;

            /*Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(H);
            if (std::isnan(eigensolver.eigenvalues().minCoeff())) {
                std::cout<<"i: " <<i<<std::endl;
                std::cout<<"\nvDtheta"<<vDtheta<<std::endl;
                std::cout<<"\naDtheta"<<aDtheta<<std::endl;
                std::cout<<"\njDtheta"<<jDtheta<<std::endl;
            }*/

            Eigen::VectorXd ftmp(5 * n);
            ftmp.setZero();
            ftmp = k1 * vDtheta.transpose() * M * VelUpdate[i] / dVel + k2 * aDtheta.transpose() * M * AccUpdate[i] / dAcc + k3 * jDtheta.transpose() * M * JerkUpdate[i] / dJerk;
            if (i == toolpath->length - 1) {
                ftmp = 2 * ftmp * toolpath->dPathPoint[i] / 2;
            }
            else {
                ftmp = 2 * ftmp * (toolpath->dPathPoint[i] + toolpath->dPathPoint[i + 1]) / 2;
            }
            f = f + ftmp;


            Av.block(8 * (i - aL), 0, 8, 5 * n) = vDtheta;
            Uv.block(8 * (i - aL), 0, 8, 1) = maxVel - VelUpdate[i];
            Lv.block(8 * (i - aL), 0, 8, 1) = -maxVel - VelUpdate[i];

            Aa.block(8 * (i - aL), 0, 8, 5 * n) = aDtheta;
            Ua.block(8 * (i - aL), 0, 8, 1) = maxAcc - AccUpdate[i];
            La.block(8 * (i - aL), 0, 8, 1) = -maxAcc - AccUpdate[i];

            Aj.block(8 * (i - aL), 0, 8, 5 * n) = jDtheta;
            Uj.block(8 * (i - aL), 0, 8, 1) = maxJerk - JerkUpdate[i];
            Lj.block(8 * (i - aL), 0, 8, 1) = -maxJerk - JerkUpdate[i];

            if (i >= a && i <= b) {
                Eigen::MatrixXd jDthetaTmp(8, 5 * n);
                jDthetaTmp.setZero();
                jDthetaTmp.block<8, 5>(0, 5 * (i - a)) = JointDTheta[i - a];
                Ajoint.block(8 * (i - a), 0, 8, 5 * n) = jDthetaTmp;
                Ujoint.block(8 * (i - a), 0, 8, 1) = JointMax - JointsUpdate[i - a];
                Ljoint.block(8 * (i - a), 0, 8, 1) = JointMin - JointsUpdate[i - a];
            }

        }


        int nCons;
        if (checkColi) {
            nCons = n + n + n + n + n + 8 * nLH + 8 * nLH + 8 * nLH + 8 * n + 5 * n;
        }
        else {
            nCons = n + n + n + 8 * nLH + 8 * nLH + 8 * nLH + 8 * n + 5 * n;
        }

        
        Eigen::MatrixXd A(nCons, 5 * n);
        Eigen::VectorXd l(nCons), u(nCons);
        l.setZero();
        u.setZero();

        int tmp = 0;
        A.block(tmp, 0, n, 5 * n) = Anozzle;
        l.block(tmp, 0, n, 1) = Lnozzle;
        u.block(tmp, 0, n, 1) = Unozzle;
        tmp = tmp + n;
        A.block(tmp, 0, n, 5 * n) = Anormal;
        l.block(tmp, 0, n, 1) = Lnormal;
        u.block(tmp, 0, n, 1) = Unormal;
        tmp = tmp + n;
        A.block(tmp, 0, n, 5 * n) = Ann;
        l.block(tmp, 0, n, 1) = Lnn;
        u.block(tmp, 0, n, 1) = Unn;
        tmp = tmp + n;

        if (checkColi) {
            //layerÅö×²
            A.block(tmp, 0, n, 5 * n) = Acoli;
            l.block(tmp, 0, n, 1) = Lcoli;
            u.block(tmp, 0, n, 1) = Ucoli;
            tmp = tmp + n;

            //A.block(tmp, 0, n, 5 * n).setZero();
            //u.block(tmp, 0, n, 1).setOnes();
            A.block(tmp, 0, n, 5 * n) = AcoliBase;
            l.block(tmp, 0, n, 1) = LcoliBase;
            u.block(tmp, 0, n, 1) = UcoliBase;
            tmp = tmp + n;
        }
        
        A.block(tmp, 0, 8 * nLH, 5 * n) = Av;
        l.block(tmp, 0, 8 * nLH, 1) = Lv;
        u.block(tmp, 0, 8 * nLH, 1) = Uv;
        tmp = tmp + 8 * nLH;
        A.block(tmp, 0, 8 * nLH, 5 * n) = Aa;
        l.block(tmp, 0, 8 * nLH, 1) = La;
        u.block(tmp, 0, 8 * nLH, 1) = Ua;
        tmp = tmp + 8 * nLH;
        A.block(tmp, 0, 8 * nLH, 5 * n) = Aj;
        l.block(tmp, 0, 8 * nLH, 1) = Lj;
        u.block(tmp, 0, 8 * nLH, 1) = Uj;
        tmp = tmp + 8 * nLH;
        A.block(tmp, 0, 8 * n, 5 * n) = Ajoint;
        l.block(tmp, 0, 8 * n, 1) = Ljoint;
        u.block(tmp, 0, 8 * n, 1) = Ujoint;
        tmp = tmp + 8 * n;
        A.block(tmp, 0, 5 * n, 5 * n) = AdTheta;
        l.block(tmp, 0, 5 * n, 1) = -UdTheta;
        u.block(tmp, 0, 5 * n, 1) = UdTheta;


        Eigen::SparseMatrix<double> sparA = A.sparseView();
        sparA.makeCompressed();
        Eigen::SparseMatrix<double> sparH = H.sparseView();
        sparH.makeCompressed();

        osqp::OsqpExitCode exit_code = osqp::OsqpExitCode::kOptimal;
        while (UdTheta.norm() > tau1) {
            osqp::OsqpInstance Instance;
            Instance.objective_matrix = sparH;
            Instance.objective_vector = f;
            Instance.lower_bounds = l;
            Instance.upper_bounds = u;
            Instance.constraint_matrix = sparA;

            osqp::OsqpSolver solver;
            osqp::OsqpSettings settings;
            settings.verbose = false;

            if (show) {
                std::cout << "QP solved" << std::endl;
            }


            //time_t start, finish;
            //double time;
            //start = clock();

            auto status = solver.Init(Instance, settings);
            //std::cout<<"status: "<<status<<std::endl;
            exit_code = solver.Solve();

            //finish = clock();
            //time = (double)(finish - start) / CLOCKS_PER_SEC;
            //std::cout << "QP time: " << time << std::endl;

            if (exit_code != osqp::OsqpExitCode::kOptimal) {

                if (show) {
                    std::cerr << "\nSQP not solved for: " << a << " - " << b << std::endl;
                    std::cerr << "exit_code: " << static_cast<int>(exit_code) << std::endl;
				}
                
                
                double tmp1;
                int tmp2;
                tmp1 = u.minCoeff(&tmp2);
                if (tmp1 < 0) {
                    std::cout<<"the minimum of u is negative"<<std::endl;
                    std::cout << "u: " << tmp1 << " " << tmp2 << std::endl;
                }
                tmp1= l.maxCoeff(&tmp2);
                if (tmp1 > 0) {
                    std::cout<<"the maximum of l is positive"<<std::endl;
					std::cout << "l: " << tmp1 << " " << tmp2 << std::endl;
                }   

                break;
            }


            double optimal_objective = solver.objective_value();
            Eigen::VectorXd dTheta = solver.primal_solution();

            Eigen::VectorXd Theta2 = ThetaUpdate.block(5 * a, 0, 5 * n, 1) + dTheta;

            std::vector<Eigen::Matrix<double, 8, 1>> Joints2 = JointsUpdate;
            std::vector<Transform3d> Ttcps2 = TtcpsUpdate, Tpos2 = TposUpdate;
            std::vector<Eigen::Matrix<double, 8, 1>> Vel2 = VelUpdate, Acc2 = AccUpdate, Jerk2 = JerkUpdate;
            Eigen::VectorXd phi_i2 = phiiUpdate;
            
            std::vector<double> tauLayer2, tauBase2;
            std::vector<Eigen::VectorXd> diffTauLayer2, diffTauBase2;
            if (checkColi) {
                tauLayer2.resize(n);
                tauBase2.resize(n);
                diffTauLayer2.resize(n);
                diffTauBase2.resize(n);
            }

            bool solveFlag = false;

            for (int i = 0; i < n; i++) {
                Eigen::Vector2d BC2 = Theta2.block<2, 1>(i * 5 + 3, 0);
                Joints2[i + a].block<2, 1>(6, 0) = BC2;

                Eigen::Vector3d w2 = Theta2.block<3, 1>(i * 5, 0);
                Eigen::Matrix3d wHat2 = mathTools::skew(w2);
                Eigen::Matrix3d R2 = mathTools::expm(wHat2);


                Tpos2[i + a] = robs->fkPos(BC2);

                Eigen::Vector4d p2Tmp = Tpos2[i + a] * toolpath->posWorkpiece[i + a].matrix().block<4, 1>(0, 3);
                Eigen::Vector3d p2 = p2Tmp.block<3, 1>(0, 0);

                Ttcps2[i + a] = Transform3d::Identity();
                Ttcps2[i + a].linear() = R2;
                Ttcps2[i + a].translation() = p2;

                Transform3d Ttmp;
                Ttmp.matrix() = Ttcps2[i + a].matrix() * robs->Ttool.matrix().inverse();

                Joints2[i + a].block<6, 1>(0, 0) = robs->ikABB2600(solveFlag, Ttmp, JointsUpdate[i + a].block<6, 1>(0, 0));
                if (!solveFlag) {
                    if (show) {
                        std::cout << "ik error" << std::endl;
                    }
                    //std::cout<<"ik error"<<std::endl;
                    break;
                }

                if (!isInLimits(Joints2[i + a], JointMin, JointMax)) {
                    if (show) {
						std::cout << "joint error" << std::endl;
					}
                    //std::cout<<"joint error"<<std::endl;
                    solveFlag = false;
                    break;
                }


                Eigen::Vector3d Nnormal2, Nnozzle2;
                Nnozzle2 = Ttcps2[i + a].matrix().block<3, 1>(0, 2);
                Nnormal2 = Tpos2[i + a].linear() * toolpath->pathNormal[i + a];

                if (-Nnormal2.dot(G) < alpha || Nnozzle2.dot(G) < beta || -Nnormal2.dot(Nnozzle2) < gamma) {
                    if (show) {
						std::cout << "normal error" << std::endl;
					}
                    //std::cout << "normal error" << std::endl;
                    solveFlag = false;
                    break;
                }


                if (checkColi) {
                    double Tau2 = getFastronScore(Joints2[i + a], colli->supportData, colli->alpha, diffTauLayer2[i], robs);
                    tauLayer2[i] = Tau2;

                    if (Tau2 > errorTol) {
                        //std::cout << "Tau > error: " << Tau2 << std::endl;
                        solveFlag = false;
                        break;
                    }

                    Eigen::MatrixXd posDq;
                    Eigen::VectorXd tmp = robs->Joints2Postions(Joints2[i + a], true, posDq);
                    getBase.calculateGammaAndDerivative(tmp, tauBase2[i], diffTauBase2[i]);
                    diffTauBase2[i] = posDq.transpose() * diffTauBase2[i];

                    if (tauBase2[i] > errorTolBase) {
                        solveFlag = false;
                        break;
                    }
                }

            }

            if (!solveFlag) {
                if (show) {
                    std::cout << "cut half 1" << std::endl;
                }
                //std::cout<<"cut half"<<std::endl;
                UdTheta = 0.5 * UdTheta;
                l.block(nCons - 5 * n, 0, 5 * n, 1) = -UdTheta;
                u.block(nCons - 5 * n, 0, 5 * n, 1) = UdTheta;
                continue;
            }

            Eigen::Matrix<double, 8, 1> maxVel2, maxAcc2, maxJerk2;
            maxVel2 = vJointMax;
            maxAcc2 = aJointMax;
            maxJerk2 = jJointMax;

            for (int i = aL; i < bH + 1; i++) {
                std::vector<Eigen::Matrix<double, 8, 1>> localJoint;

                localJoint.resize(5);
                for (int j = 0; j < 5; j++) {
                    if ((i - 2 + j < 0) || (i - 2 + j > Joints2.size() - 1)) {
                        localJoint[j].resize(8);
                        localJoint[j].setZero();
                    }
                    else {
                        localJoint[j] = Joints2[i - 2 + j];
                    }
                }

                Vel2[i].setZero();
                Acc2[i].setZero();
                Jerk2[i].setZero();

                for (int j = 0; j < 5; j++) {
                    Vel2[i] = Vel2[i] + kVel[i](j) * localJoint[j];
                    Acc2[i] = Acc2[i] + kAcc[i](j) * localJoint[j];
                    Jerk2[i] = Jerk2[i] + kJerk[i](j) * localJoint[j];
                }

                maxVel2=maxVel2.cwiseMax(Vel2[i].cwiseAbs());
                maxAcc2=maxAcc2.cwiseMax(Acc2[i].cwiseAbs());
                maxJerk2=maxJerk2.cwiseMax(Jerk2[i].cwiseAbs());

                if ((!isInLimits(Vel2[i], -maxVel, maxVel)) || (!isInLimits(Acc2[i], -maxAcc, maxAcc)) || (!isInLimits(Jerk2[i], -maxJerk, maxJerk))) {
                    //std::cout<<"vel acc jerk error"<<std::endl;
                    solveFlag = false;
                    break;
                }

                double phi = 0;
                double tmp = Vel2[i].transpose() * M * Vel2[i];
                phi = k1 * tmp / dVel;
                tmp = Acc2[i].transpose() * M * Acc2[i];
                phi = phi + k2 * tmp / dAcc;
                tmp = Jerk2[i].transpose() * M * Jerk2[i];
                phi = phi + k3 * tmp / dJerk;

                if (i == toolpath->length - 1) {
                    phi_i2(i) = phi * (toolpath->dPathPoint[i]) / 2;
                }
                else {
                    phi_i2(i) = phi * (toolpath->dPathPoint[i] + toolpath->dPathPoint[i + 1]) / 2;
                }

            }

            if (!solveFlag) {
                if (show) {
                    std::cout << "cut half 2" << std::endl;
                }

                UdTheta = 0.5 * UdTheta;
                l.block(nCons - 5 * n, 0, 5 * n, 1) = -UdTheta;
                u.block(nCons - 5 * n, 0, 5 * n, 1) = UdTheta;
                continue;
            }


            Eigen::VectorXd local_phi_i2 = phi_i2.segment(aL, nLH);
            Phi_1 = local_phi_i2.sum();

            //std::cout << "after optimization: " << Phi_1 << std::endl;

            if (Phi_1 < Phi_0) {

                success = true;

                Eigen::Array<bool, Eigen::Dynamic, 1> C;
                C = (UdTheta - dTheta.cwiseAbs()).array() < (0.01 * dTheta.cwiseAbs()).array();
                Eigen::VectorXd D = C.cast<double>();

                UdTheta = UdTheta + 0.05 * D.cwiseProduct(UdTheta);

                JointsUpdate = Joints2;
                TtcpsUpdate = Ttcps2;
                TposUpdate = Tpos2;
                VelUpdate = Vel2;
                AccUpdate = Acc2;
                JerkUpdate = Jerk2;
                phiiUpdate = phi_i2;

                ThetaUpdate.block(5 * a, 0, 5 * n, 1) = Theta2;

                if (checkColi) {
                    tauLayer = tauLayer2;
                    diffTauLayer = diffTauLayer2;
                    tauBase = tauBase2;
                    diffTauBase = diffTauBase2;
                }
                

                maxVel = maxVel2;
                maxAcc = maxAcc2;
                maxJerk = maxJerk2;


                if (show) {
                    Tfinish = clock();
                    double time = (double)(Tfinish - Tstart) / CLOCKS_PER_SEC;
                    std::cout << "Time in SQP: " << time << ";  obj: " << Phi_1 << std::endl;
                }


                break;
            }
            else {
                UdTheta = 0.5 * UdTheta;
                l.block(nCons - 5 * n, 0, 5 * n, 1) = -UdTheta;
                u.block(nCons - 5 * n, 0, 5 * n, 1) = UdTheta;
                continue;
            }

        }

        if (exit_code != osqp::OsqpExitCode::kOptimal) {
            break;
        }

    }


    if (success) {
        if ((Phi_ini - Phi_1) / Phi_ini < tau2) {
            success = false;
        }
    }


    Info.JointsUpdate = JointsUpdate;
    Info.TtcpsUpdate = TtcpsUpdate;
    Info.TposUpdate = TposUpdate;
    Info.VelUpdate = VelUpdate;
    Info.AccUpdate = AccUpdate;
    Info.JerkUpdate = JerkUpdate;
    Info.phiiUpdate = phiiUpdate;
    Info.ThetaUpdate = ThetaUpdate;
    Info.a = a;
    Info.b = b;

}

void trajOpt::local_optimal_time(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool& success, int a, int b, updateInfo& Info, bool checkColi, bool show) {

    if (show) {
        std::cout<<"start: "<<a<<" - "<<b<<std::endl;
    }

    std::vector<Eigen::Matrix<double, 8, 1>> JointsUpdate = Joints;
    std::vector<Transform3d> TtcpsUpdate = Ttcps, TposUpdate = Tpos;
    std::vector<Eigen::Matrix<double, 8, 1>> VelUpdate = Vel, AccUpdate = Acc, JerkUpdate = Jerk;
    Eigen::VectorXd phiiUpdate = phi_i;
    Eigen::VectorXd ThetaUpdate = Theta;
    std::vector<Eigen::Matrix<double,5,1>> kVelUpdate = kVel, kAccUpdate = kAcc, kJerkUpdate = kJerk;
    std::vector<Eigen::Matrix<double,5,4>> dkVelUpdate = dkVel, dkAccUpdate = dkAcc, dkJerkUpdate = dkJerk;
    Eigen::VectorXd localTimeUpdate = localTime;
    Eigen::VectorXd vTTP = velTTP.segment(a, b - a), aTTP_T = acctTTP.segment(a, b - a + 1), aTTP_N = accnTTP.segment(a, b - a + 1);

    const int aL = std::max(0, a - 2);
    const int bH = std::min(toolpath->length - 1, b + 2);
    const int nLH = bH - aL + 1;
    Eigen::Vector3d G(0, 0, -1);
    const int n = b - a + 1;

    int nCons;
    if (checkColi) {
        nCons = n + n + n + n + n + 8 * nLH + 8 * nLH + 8 * nLH + 8 * n + (3 * n - 1) + (6 * n - 1);
    }
    else {
        nCons = n + n + n + 8 * nLH + 8 * nLH + 8 * nLH + 8 * n + (3 * n - 1) + (6 * n - 1);
    }

    double Phi_1 = phiiUpdate.segment(aL, nLH).sum();
    double time_1 = localTimeUpdate.segment(a + 1, n - 1).sum();

    double PHI_1 = Phi_1 + kTime * time_1 / dTime;
    double PHI_ini = PHI_1;
    double PHI_0 = 2 * PHI_1;

    clock_t Tstart, Tfinish;
    if (show) {
        Tstart = clock();
        std::cout << "start SQP  The initial: " << PHI_1 << std::endl;
    }

    SVMGrad getBase;
    std::vector<double> tauLayer, tauBase;
    std::vector<Eigen::VectorXd> diffTauLayer, diffTauBase;
    if (checkColi) {
        tauLayer.resize(n);
        tauBase.resize(n);
        diffTauLayer.resize(n);
        diffTauBase.resize(n);

        string modelFile = "../DataSet/coliTrain/BaseColi/BaseColiFK.modelG";
        getBase.loadModel(modelFile);
        getBase.preComputeKernel(true);

        for (int i = 0; i < n; i++) {
            double tau = getFastronScore(JointsUpdate[a + i], colli->supportData, colli->alpha, diffTauLayer[i], robs);
            tauLayer[i] = tau;

            Eigen::MatrixXd posDq;
            Eigen::VectorXd tmp = robs->Joints2Postions(JointsUpdate[a + i], true, posDq);
            getBase.calculateGammaAndDerivative(tmp, tauBase[i], diffTauBase[i]);
            diffTauBase[i] = posDq.transpose() * diffTauBase[i];
        }
    }

    //std::cout<<"2"<<std::endl;

    Eigen::MatrixXd AdTheta(5 * n, 5 * n);
    AdTheta.setIdentity();
    Eigen::VectorXd UdTheta(5 * n);
    for (int i = 0; i < n; i++) {
        Eigen::VectorXd tmp(5);
        tmp << 0.01, 0.01, 0.01, 0.1, 0.1;
        UdTheta.block<5, 1>(5 * i, 0) = tmp;
    }

    Eigen::MatrixXd AdTime(n - 1, n - 1);
    AdTime.setIdentity();
    Eigen::VectorXd UdTime(n - 1);
    UdTime.setOnes();
    UdTime = UdTime * (localTimeUpdate.segment(a + 1, n - 1).maxCoeff() + localTimeUpdate.segment(a + 1, n - 1).minCoeff()) / 2 / 20;

    success = false;

    Eigen::Matrix<double, 8, 1> maxVel, maxAcc, maxJerk;
    maxVel = vJointMax;
    maxAcc = aJointMax;
    maxJerk = jJointMax;
    for (int i = aL; i <= bH; i++) {
        maxVel = maxVel.cwiseMax(VelUpdate[i].cwiseAbs());
        maxAcc = maxAcc.cwiseMax(AccUpdate[i].cwiseAbs());
        maxJerk = maxJerk.cwiseMax(JerkUpdate[i].cwiseAbs());
    }
    if (show) {
        std::cout<<"maxVel: "<<maxVel.transpose()<<std::endl;
		std::cout<<"maxAcc: "<<maxAcc.transpose()<<std::endl;
		std::cout<<"maxJerk: "<<maxJerk.transpose()<<std::endl;
    }
    
    double maxVelTTP = vTTPMax, maxAccTTP_T = aTTPMax_T, maxAccTTP_N = aTTPMax_N;
    Eigen::VectorXd aTtpNLarge(n);
    aTtpNLarge = (aTTP_N.cwiseAbs().array() > maxAccTTP_N).cast<double>();
    
    PHI_1 = PHI_1 + aTtpNLarge.dot(aTTP_N.cwiseProduct(aTTP_N)) * 1e8;
    PHI_0 = 2 * PHI_1;


    //SQP
    while ((PHI_0 - PHI_1) / PHI_0 > tau2) {
        PHI_0 = PHI_1;

        std::vector<Eigen::Matrix<double, 6, 3>> qDp(n), qDw(n);
        std::vector<Eigen::Matrix<double, 3, 2>> pDbc(n);
        std::vector<Eigen::Matrix<double, 6, 2>> qDbc(n);
        std::vector<Eigen::Matrix<double, 8, 5>> JointDTheta(n);

        Eigen::MatrixXd Anozzle(n, 5 * n), Anormal(n, 5 * n), Ann(n, 5 * n);
        Eigen::VectorXd Lnozzle(n), Unozzle(n), Lnormal(n), Unormal(n), Lnn(n), Unn(n);
        Unozzle.setOnes();
        Unozzle = Unozzle * std::numeric_limits<double>::infinity();
        Lnormal.setOnes();
        Lnormal = -Lnormal * std::numeric_limits<double>::infinity();
        Lnn.setOnes();
        Lnn = -Lnn * std::numeric_limits<double>::infinity();

        Eigen::MatrixXd Acoli, AcoliBase;
        Eigen::VectorXd Lcoli, Ucoli, LcoliBase, UcoliBase;
        if (checkColi) {
            Acoli.resize(n, 5 * n);
            AcoliBase.resize(n, 5 * n);
            Lcoli.resize(n);
            Ucoli.resize(n);
            LcoliBase.resize(n);
            UcoliBase.resize(n);

            Ucoli.setZero();
            Acoli.setZero();
            Lcoli.setOnes();
            Lcoli = -Lcoli * std::numeric_limits<double>::infinity();
            UcoliBase.setZero();
            AcoliBase.setZero();
            LcoliBase.setOnes();
            LcoliBase = -LcoliBase * std::numeric_limits<double>::infinity();
        }

        double maxTau = 0;


        Eigen::MatrixXd Avttp(n - 1, n - 1), AattpT(n, n - 1), AattpN(n, n - 1);
        Eigen::VectorXd Lvttp(n - 1), Uvttp(n - 1), LattpT(n), UattpT(n), LattpN(n), UattpN(n);
        Avttp.setZero();
        AattpT.setZero();
        AattpN.setZero();
        Lvttp.setZero();
        Uvttp.setZero();
        LattpT.setZero();
        UattpT.setZero();
        LattpN.setZero();
        UattpN.setZero();



        Eigen::MatrixXd H(6 * n - 1, 6 * n - 1);
        H.setZero();
        Eigen::VectorXd f(6 * n - 1);
        f.setZero();
        f.segment(5 * n, n - 1) = kTime * Eigen::VectorXd::Ones(n - 1) / dTime;

        for (int i = a; i < b + 1; i++) {
            Eigen::Matrix<double, 6, 1> qtmp = JointsUpdate[i].block<6, 1>(0, 0);
            Eigen::Matrix<double, 6, 6> J = robs->JaB_ABB(qtmp, robs->Ttool);

            Eigen::Vector3d w = ThetaUpdate.segment(i * 5, 3);
            Eigen::Vector2d BC = ThetaUpdate.segment(i * 5 + 3, 2);

            Eigen::MatrixXd NnozzleDw(3, 5 * n), NnormalDw(3, 5 * n);
            NnozzleDw.setZero();
            NnormalDw.setZero();


            for (int j = 0; j < 3; j++) {
                Eigen::Vector3d dw = Eigen::Vector3d::Zero();
                dw(j) = 1;

                Eigen::Matrix4d dT = Eigen::Matrix4d::Zero();
                dT.block<3, 1>(0, 3) = dw;
                Eigen::Matrix4d vbHat = TtcpsUpdate[i].matrix().inverse() * dT;
                Eigen::Matrix<double, 6, 1> vb = mathTools::vex(vbHat);
                qDp[i - a].block<6, 1>(0, j) = J.inverse() * vb;


                Eigen::Matrix3d dR = mathTools::diffRotm(w, dw);
                dT = Eigen::Matrix4d::Zero();
                dT.block<3, 3>(0, 0) = dR;
                vbHat = TtcpsUpdate[i].matrix().inverse() * dT;
                vb = mathTools::vex(vbHat);
                qDw[i - a].block<6, 1>(0, j) = J.inverse() * vb;


                NnozzleDw.block<3, 1>(0, 5 * (i - a) + j) = dR.block<3, 1>(0, 2);

            }


            std::vector<Eigen::Matrix4d> dTbc = robs->dfkPos(BC);
            for (int j = 0; j < 2; j++) {

                Eigen::Vector4d posiTmp = dTbc[j] * toolpath->posWorkpiece[i].matrix().block<4, 1>(0, 3);
                pDbc[i - a].block<3, 1>(0, j) = posiTmp.block<3, 1>(0, 0);

                NnormalDw.block<3, 1>(0, 5 * (i - a) + j + 3) = dTbc[j].block<3, 3>(0, 0) * toolpath->pathNormal[i];
            }


            qDbc[i - a] = qDp[i - a] * pDbc[i - a];

            JointDTheta[i - a] = Eigen::Matrix<double, 8, 5>::Zero();
            JointDTheta[i - a].block<6, 3>(0, 0) = qDw[i - a];
            JointDTheta[i - a].block<6, 2>(0, 3) = qDbc[i - a];
            JointDTheta[i - a].block<2, 2>(6, 3) = Eigen::Matrix2d::Identity();

            Eigen::Vector3d Nnormal, Nnozzle;
            Nnozzle = TtcpsUpdate[i].matrix().block<3, 1>(0, 2);
            Nnormal = TposUpdate[i].linear() * toolpath->pathNormal[i];

            Anozzle.block(i - a, 0, 1, 5 * n) = G.transpose() * NnozzleDw;
            Lnozzle(i - a) = beta - G.transpose() * Nnozzle;

            Anormal.block(i - a, 0, 1, 5 * n) = G.transpose() * NnormalDw;
            Unormal(i - a) = -alpha - G.transpose() * Nnormal;

            Ann.block(i - a, 0, 1, 5 * n) = Nnormal.transpose() * NnozzleDw + Nnozzle.transpose() * NnormalDw;
            Unn(i - a) = -gamma - Nnormal.dot(Nnozzle);

            if (checkColi) {
                double tau = tauLayer[i - a];
                if (tau > errorTol) {
                    cout << "tau: " << tau << endl;
                }

                Acoli.block(i - a, 5 * (i - a), 1, 5) = diffTauLayer[i - a].transpose() * JointDTheta[i - a];
                Ucoli(i - a) = errorTol - tau;

                AcoliBase.block(i - a, 5 * (i - a), 1, 5) = diffTauBase[i - a].transpose() * JointDTheta[i - a];
                UcoliBase(i - a) = errorTolBase - tauBase[i - a];
            }


            double s = toolpath->dPathPoint[i], t = localTimeUpdate(i), s1, t1;
            if (i < toolpath->length - 1) {
                s1 = toolpath->dPathPoint[i + 1];
                t1 = localTimeUpdate(i + 1);
            }

            if (i == a) {
                if (i != 0) {
                    AattpT(0, 0) = -(2 * (s1 * t * t + 2 * s1 * t * t1 - s * t1 * t1)) / (t * t1 * t1 * pow((t + t1), 2));
                    UattpT(0) = maxAccTTP_T - aTTP_T(0);
                    LattpT(0) = -maxAccTTP_T - aTTP_T(0);

                    AattpN(0, 0) = -Curvature(i) * (s1 * (s1 * t - toolpath->cosAngle[i] * s * t1)) / (2 * t * pow(t1, 3));
                    UattpN(0) = maxAccTTP_N - aTTP_N(0);
                    LattpN(0) = -maxAccTTP_N - aTTP_N(0);
                }
            }
            else {
                Avttp(i - a - 1, i - a - 1) = s / pow(t, 2);
                Uvttp(i - a - 1) = vTTP(i - a - 1);
                Lvttp(i - a - 1) = vTTP(i - a - 1) - maxVelTTP;

                if (i == b) {
                    if (i != toolpath->length - 1) {
                        AattpT(i - a, n - 2) = (2 * (-s1 * t * t + 2 * s * t * t1 + s * t1 * t1)) / (t * t * t1 * pow((t + t1), 2));
                        UattpT(i - a) = maxAccTTP_T - aTTP_T(i - a);
                        LattpT(i - a) = -maxAccTTP_T - aTTP_T(i - a);

                        AattpN(i - a, n - 2) = -Curvature(i) * (s * (s * t1 - toolpath->cosAngle[i] * s1 * t)) / (2 * pow(t, 3) * t1);
                        UattpN(i - a) = maxAccTTP_N - aTTP_N(i - a);
                        LattpN(i - a) = -maxAccTTP_N - aTTP_N(i - a);
                    }
                }
                else {
                    AattpT(i - a, i - a - 1) = (2 * (-s1 * t * t + 2 * s * t * t1 + s * t1 * t1)) / (t * t * t1 * pow((t + t1), 2));
                    AattpT(i - a, i - a) = -(2 * (s1 * t * t + 2 * s1 * t * t1 - s * t1 * t1)) / (t * t1 * t1 * pow((t + t1), 2));
                    UattpT(i - a) = maxAccTTP_T - aTTP_T(i - a);
                    LattpT(i - a) = -maxAccTTP_T - aTTP_T(i - a);

                    AattpN(i - a, i - a - 1) = -Curvature(i) * (s * (s * t1 - toolpath->cosAngle[i] * s1 * t)) / (2 * pow(t, 3) * t1);
                    AattpN(i - a, i - a) = -Curvature(i) * (s1 * (s1 * t - toolpath->cosAngle[i] * s * t1)) / (2 * t * pow(t1, 3));
                    UattpN(i - a) = maxAccTTP_N - aTTP_N(i - a);
                    LattpN(i - a) = -maxAccTTP_N - aTTP_N(i - a);
                }

            }

            if (aTtpNLarge(i - a) == 1) {
                Eigen::VectorXd dAn(6 * n - 1);
                dAn.setZero();
                dAn.segment(5 * n, n - 1) = AattpN.block(i - a, 0, 1, n - 1);

                Eigen::MatrixXd Htmp(6 * n - 1, 6 * n - 1);
                Htmp = 1e8 * dAn * dAn.transpose();
                Eigen::MatrixXd Htmp2 = Htmp.transpose();
                Htmp = Htmp + Htmp2;
                H = H + Htmp;

                Eigen::VectorXd ftmp(6 * n - 1);
                ftmp = 2 * aTTP_N(i - a) * dAn * 1e8;
                f = f + ftmp;

                UattpN(i - a) = std::max(UattpN(i - a), 0.0);
                LattpN(i - a) = std::min(LattpN(i - a), 0.0);
            }



        }
        //std::cout << "maxTau: " << maxTau << std::endl;

        Eigen::MatrixXd Av(8 * (nLH), 6 * n - 1), Aa(8 * (nLH), 6 * n - 1), Aj(8 * (nLH), 6 * n - 1), Ajoint(8 * n, 5 * n);
        Eigen::VectorXd Lv(8 * (nLH)), Uv(8 * (nLH)), La(8 * (nLH)), Ua(8 * (nLH)), Lj(8 * (nLH)), Uj(8 * (nLH)), Ljoint(8 * n), Ujoint(8 * n);


        for (int i = aL; i < bH + 1; i++) {
            Eigen::MatrixXd vDtheta(8, 5 * n), aDtheta(8, 5 * n), jDtheta(8, 5 * n);
            vDtheta.setZero();
            aDtheta.setZero();
            jDtheta.setZero();

            int j1 = 0, j2 = 5;
            if (a + 2 - i > 0) {
                j1 = a + 2 - i;
            }
            if (b + 3 - i < 5) {
                j2 = b + 3 - i;
            }

            for (int j = j1; j < j2; j++) {
                Eigen::MatrixXd jDthetaTmp(8, 5 * n);
                jDthetaTmp.setZero();
                jDthetaTmp.block<8, 5>(0, 5 * (i - a - 2 + j)) = JointDTheta[i - a - 2 + j];

                Eigen::MatrixXd eye(8, 8);
                eye.setIdentity();

                vDtheta = vDtheta + kVel[i](j) * eye * jDthetaTmp;
                aDtheta = aDtheta + kAcc[i](j) * eye * jDthetaTmp;
                jDtheta = jDtheta + kJerk[i](j) * eye * jDthetaTmp;
            }

            Eigen::MatrixXd vDtime(8, n - 1), aDtime(8, n - 1), jDtime(8, n - 1);
            vDtime.setZero();
            aDtime.setZero();
            jDtime.setZero();

            if (i >= a - 1 && i <= b + 1) {
                Eigen::Matrix<double, 8, 5> localJoint;
                localJoint.setZero();
                if (i - 2 >= 0) {
                    localJoint.block<8, 1>(0, 0) = JointsUpdate[i - 2];
                }
                if (i - 1 >= 0) {
					localJoint.block<8, 1>(0, 1) = JointsUpdate[i - 1];
				}
                localJoint.block<8, 1>(0, 2) = JointsUpdate[i];
                if (i + 1 <= toolpath->length - 1) {
                    localJoint.block<8, 1>(0, 3) = JointsUpdate[i + 1];
                }
                if (i + 2 <= toolpath->length - 1) {
					localJoint.block<8, 1>(0, 4) = JointsUpdate[i + 2];
				}

                Eigen::MatrixXd kvDtime(5, n - 1), kaDtime(5, n - 1), kjDtime(5, n - 1);
                kvDtime.setZero(); kaDtime.setZero(); kjDtime.setZero();
                if (i < a + 2) {
                    kvDtime.block(0, 0, 5, i - a + 2) = dkVelUpdate[i].block(0, a + 2 - i, 5, i - a + 2);
                    kaDtime.block(0, 0, 5, i - a + 2) = dkAccUpdate[i].block(0, a + 2 - i, 5, i - a + 2);
                    kjDtime.block(0, 0, 5, i - a + 2) = dkJerkUpdate[i].block(0, a + 2 - i, 5, i - a + 2);
                }
                else if (i > b - 2) {
                    kvDtime.block(0, i - a - 2, 5, b - i + 2) = dkVelUpdate[i].block(0, 0, 5, b - i + 2);
                    kaDtime.block(0, i - a - 2, 5, b - i + 2) = dkAccUpdate[i].block(0, 0, 5, b - i + 2);
                    kjDtime.block(0, i - a - 2, 5, b - i + 2) = dkJerkUpdate[i].block(0, 0, 5, b - i + 2);
                }
                else {
                    kvDtime.block<5, 4>(0, i - a - 2) = dkVelUpdate[i];
                    kaDtime.block<5, 4>(0, i - a - 2) = dkAccUpdate[i];
                    kjDtime.block<5, 4>(0, i - a - 2) = dkJerkUpdate[i];
                }
                
                vDtime = localJoint * kvDtime;
                aDtime = localJoint * kaDtime;
                jDtime = localJoint * kjDtime;
            }


            Eigen::MatrixXd Htmp(6 * n - 1, 6 * n - 1);
            Htmp.setZero();
            Eigen::MatrixXd vDvar(8, 6 * n - 1), aDvar(8, 6 * n - 1), jDvar(8, 6 * n - 1);
            vDvar.block(0, 0, 8, 5 * n) = vDtheta;
            vDvar.block(0, 5 * n, 8, n - 1) = vDtime;
            aDvar.block(0, 0, 8, 5 * n) = aDtheta;
            aDvar.block(0, 5 * n, 8, n - 1) = aDtime;
            jDvar.block(0, 0, 8, 5 * n) = jDtheta;
            jDvar.block(0, 5 * n, 8, n - 1) = jDtime;
            Htmp = k1 * vDvar.transpose() * M * vDvar / dVel + k2 * aDvar.transpose() * M * aDvar / dAcc + k3 * jDvar.transpose() * M * jDvar / dJerk;
            if (i == toolpath->length - 1) {
                Htmp = Htmp * toolpath->dPathPoint[i] / 2;
            }
            else {
                Htmp = Htmp * (toolpath->dPathPoint[i] + toolpath->dPathPoint[i + 1]) / 2;
            }
            Eigen::MatrixXd Htmp2 = Htmp.transpose();
            Htmp = Htmp + Htmp2;
            H = H + Htmp;

            Eigen::VectorXd ftmp(5 * n + n - 1);
            ftmp.setZero();
            ftmp = k1 * vDvar.transpose() * M * VelUpdate[i] / dVel + k2 * aDvar.transpose() * M * AccUpdate[i] / dAcc + k3 * jDvar.transpose() * M * JerkUpdate[i] / dJerk;
            if (i == toolpath->length - 1) {
                ftmp = 2 * ftmp * toolpath->dPathPoint[i] / 2;
            }
            else {
                ftmp = 2 * ftmp * (toolpath->dPathPoint[i] + toolpath->dPathPoint[i + 1]) / 2;
            }
            f = f + ftmp;


            Av.block(8 * (i - aL), 0, 8, 6 * n - 1) = vDvar;
            Uv.block(8 * (i - aL), 0, 8, 1) = maxVel - VelUpdate[i];
            Lv.block(8 * (i - aL), 0, 8, 1) = -maxVel - VelUpdate[i];

            Aa.block(8 * (i - aL), 0, 8, 6 * n - 1) = aDvar;
            Ua.block(8 * (i - aL), 0, 8, 1) = maxAcc - AccUpdate[i];
            La.block(8 * (i - aL), 0, 8, 1) = -maxAcc - AccUpdate[i];

            Aj.block(8 * (i - aL), 0, 8, 6 * n - 1) = jDvar;
            Uj.block(8 * (i - aL), 0, 8, 1) = maxJerk - JerkUpdate[i];
            Lj.block(8 * (i - aL), 0, 8, 1) = -maxJerk - JerkUpdate[i];

            if (i >= a && i <= b) {
                Eigen::MatrixXd jDthetaTmp(8, 5 * n);
                jDthetaTmp.setZero();
                jDthetaTmp.block<8, 5>(0, 5 * (i - a)) = JointDTheta[i - a];
                Ajoint.block(8 * (i - a), 0, 8, 5 * n) = jDthetaTmp;
                Ujoint.block(8 * (i - a), 0, 8, 1) = JointMax - JointsUpdate[i - a];
                Ljoint.block(8 * (i - a), 0, 8, 1) = JointMin - JointsUpdate[i - a];
            }

        }


        Eigen::MatrixXd A(nCons, 6 * n - 1);
        Eigen::VectorXd l(nCons), u(nCons);
        A.setZero();
        l.setZero();
        u.setZero();

        int tmp = 0;

        A.block(tmp, 0, n, 5 * n) = Anozzle;
        l.block(tmp, 0, n, 1) = Lnozzle;
        u.block(tmp, 0, n, 1) = Unozzle;
        tmp = tmp + n;
        A.block(tmp, 0, n, 5 * n) = Anormal;
        l.block(tmp, 0, n, 1) = Lnormal;
        u.block(tmp, 0, n, 1) = Unormal;
        tmp = tmp + n;
        A.block(tmp, 0, n, 5 * n) = Ann;
        l.block(tmp, 0, n, 1) = Lnn;
        u.block(tmp, 0, n, 1) = Unn;
        tmp = tmp + n;

        if (checkColi) {
            A.block(tmp, 0, n, 5 * n) = Acoli;
            l.block(tmp, 0, n, 1) = Lcoli;
            u.block(tmp, 0, n, 1) = Ucoli;
            tmp = tmp + n;
            A.block(tmp, 0, n, 5 * n) = AcoliBase;
            l.block(tmp, 0, n, 1) = LcoliBase;
            u.block(tmp, 0, n, 1) = UcoliBase;
            tmp = tmp + n;
        }

        A.block(tmp, 0, 8 * nLH, 6 * n - 1) = Av;
        l.block(tmp, 0, 8 * nLH, 1) = Lv;
        u.block(tmp, 0, 8 * nLH, 1) = Uv;
        tmp = tmp + 8 * nLH;
        A.block(tmp, 0, 8 * nLH, 6 * n - 1) = Aa;
        l.block(tmp, 0, 8 * nLH, 1) = La;
        u.block(tmp, 0, 8 * nLH, 1) = Ua;
        tmp = tmp + 8 * nLH;
        A.block(tmp, 0, 8 * nLH, 6 * n - 1) = Aj;
        l.block(tmp, 0, 8 * nLH, 1) = Lj;
        u.block(tmp, 0, 8 * nLH, 1) = Uj;
        tmp = tmp + 8 * nLH;

        A.block(tmp, 0, 8 * n, 5 * n) = Ajoint;
        l.block(tmp, 0, 8 * n, 1) = Ljoint;
        u.block(tmp, 0, 8 * n, 1) = Ujoint;
        tmp = tmp + 8 * n;

        A.block(tmp, 5 * n, n - 1, n - 1) = Avttp;
        l.block(tmp, 0, n - 1, 1) = Lvttp;
        u.block(tmp, 0, n - 1, 1) = Uvttp;
        tmp = tmp + n - 1;
        A.block(tmp, 5 * n, n, n - 1) = AattpT;
        l.block(tmp, 0, n, 1) = LattpT;
        u.block(tmp, 0, n, 1) = UattpT;
        tmp = tmp + n;
        A.block(tmp, 5 * n, n, n - 1) = AattpN;
        l.block(tmp, 0, n, 1) = LattpN;
        u.block(tmp, 0, n, 1) = UattpN;
        tmp = tmp + n;

        A.block(tmp, 0, 5 * n, 5 * n) = AdTheta;
        l.block(tmp, 0, 5 * n, 1) = -UdTheta;
        u.block(tmp, 0, 5 * n, 1) = UdTheta;
        tmp = tmp + 5 * n;
        A.block(tmp, 5 * n, n - 1, n - 1) = AdTime;
        l.block(tmp, 0, n - 1, 1) = (-1 * localTimeUpdate.segment(a + 1, n - 1)).cwiseMax(-UdTime);
        u.block(tmp, 0, n - 1, 1) = UdTime;


        Eigen::SparseMatrix<double> sparA = A.sparseView();
        sparA.makeCompressed();
        Eigen::SparseMatrix<double> sparH = H.sparseView();
        sparH.makeCompressed();

        osqp::OsqpExitCode exit_code = osqp::OsqpExitCode::kOptimal;
        int ite_num = 0;
        while ((UdTheta.norm() > tau1 || UdTime.norm() > tau3) && (ite_num < 30)) {
            ite_num++;

            osqp::OsqpInstance Instance;
            Instance.objective_matrix = sparH;
            Instance.objective_vector = f;
            Instance.lower_bounds = l;
            Instance.upper_bounds = u;
            Instance.constraint_matrix = sparA;

            osqp::OsqpSolver solver;
            osqp::OsqpSettings settings;
            settings.verbose = false;

            auto status = solver.Init(Instance, settings);
            exit_code = solver.Solve();

            if (exit_code != osqp::OsqpExitCode::kOptimal) {

                if (show) {
                    std::cout << "\nSQP not solved for: " << a << " - " << b << std::endl;
                    std::cout << "exit_code: " << osqp::ToString(exit_code) << std::endl;
				}

                double tmp1;
                int tmp2;
                tmp1 = u.minCoeff(&tmp2);
                if (tmp1 < 0) {
                    std::cout << "the minimum of u is negative" << std::endl;
                    std::cout << "u: " << tmp1 << " " << tmp2 << std::endl;
                }
                tmp1 = l.maxCoeff(&tmp2);
                if (tmp1 > 0) {
                    std::cout << "the maximum of l is positive" << std::endl;
                    std::cout << "l: " << tmp1 << " " << tmp2 << std::endl;
                }

                break;
            }

            if (show) {
                std::cout << "QP solved" << std::endl;
            }


            double optimal_objective = solver.objective_value();
            Eigen::VectorXd dVar = solver.primal_solution();

            Eigen::VectorXd Theta2 = ThetaUpdate.block(5 * a, 0, 5 * n, 1) + dVar.segment(0, 5 * n);

            Eigen::VectorXd localTime2 = localTimeUpdate;
            localTime2.segment(a + 1, n - 1) = localTime2.segment(a + 1, n - 1) + dVar.segment(5 * n, n - 1);

            std::vector<Eigen::Matrix<double, 8, 1>> Joints2 = JointsUpdate;
            std::vector<Transform3d> Ttcps2 = TtcpsUpdate, Tpos2 = TposUpdate;
            std::vector<Eigen::Matrix<double, 8, 1>> Vel2 = VelUpdate, Acc2 = AccUpdate, Jerk2 = JerkUpdate;
            Eigen::VectorXd phi_i2 = phiiUpdate;
            std::vector<Eigen::Matrix<double, 5, 1>> kVel2 = kVelUpdate, kAcc2 = kAccUpdate, kJerk2 = kJerkUpdate;
            std::vector<Eigen::Matrix<double, 5, 4>> dkVel2 = dkVelUpdate, dkAcc2 = dkAccUpdate, dkJerk2 = dkJerkUpdate;

            std::vector<double> tauLayer2, tauBase2;
            std::vector<Eigen::VectorXd> diffTauLayer2, diffTauBase2;
            if (checkColi) {
                tauLayer2.resize(n);
                tauBase2.resize(n);
                diffTauLayer2.resize(n);
                diffTauBase2.resize(n);
            }

            Eigen::VectorXd vTTP2 = vTTP, aTTP_T2 = aTTP_T, aTTP_N2 = aTTP_N;
            //double maxVelTTP2 = vTTPMax, maxAccTTP_T2 = aTTPMax_T, maxAccTTP_N2 = aTTPMax_N;

            bool solveFlag = false, updateTimeFlag = false;

            for (int i = 0; i < n; i++) {
                Eigen::Vector2d BC2 = Theta2.block<2, 1>(i * 5 + 3, 0);
                Joints2[i + a].block<2, 1>(6, 0) = BC2;

                Eigen::Vector3d w2 = Theta2.block<3, 1>(i * 5, 0);
                Eigen::Matrix3d wHat2 = mathTools::skew(w2);
                Eigen::Matrix3d R2 = mathTools::expm(wHat2);


                Tpos2[i + a] = robs->fkPos(BC2);

                Eigen::Vector4d p2Tmp = Tpos2[i + a] * toolpath->posWorkpiece[i + a].matrix().block<4, 1>(0, 3);
                Eigen::Vector3d p2 = p2Tmp.block<3, 1>(0, 0);

                Ttcps2[i + a] = Transform3d::Identity();
                Ttcps2[i + a].linear() = R2;
                Ttcps2[i + a].translation() = p2;

                Transform3d Ttmp;
                Ttmp.matrix() = Ttcps2[i + a].matrix() * robs->Ttool.matrix().inverse();

                Joints2[i + a].block<6, 1>(0, 0) = robs->ikABB2600(solveFlag, Ttmp, JointsUpdate[i + a].block<6, 1>(0, 0));
                if (!solveFlag) {
                    if (show) {
                        std::cout << "cut because IK" << std::endl;
                    }
                    break;
                }

                if (!isInLimits(Joints2[i + a], JointMin, JointMax)) {
                    if (show) {
                        std::cout << "cut because Joint" << std::endl;
                    }
                    solveFlag = false;
                    break;
                }


                Eigen::Vector3d Nnormal2, Nnozzle2;
                Nnozzle2 = Ttcps2[i + a].matrix().block<3, 1>(0, 2);
                Nnormal2 = Tpos2[i + a].linear() * toolpath->pathNormal[i + a];

                if (-Nnormal2.dot(G) < alpha || Nnozzle2.dot(G) < beta || -Nnormal2.dot(Nnozzle2) < gamma) {
                    if (show) {
                        std::cout << "cut because angles" << std::endl;
                    }
                    solveFlag = false;
                    break;
                }


                if (checkColi) {
                    double Tau2 = getFastronScore(Joints2[i + a], colli->supportData, colli->alpha, diffTauLayer2[i], robs);
                    tauLayer2[i] = Tau2;

                    if (Tau2 > errorTol) {
                        solveFlag = false;
                        break;
                    }

                    Eigen::MatrixXd posDq;
                    Eigen::VectorXd tmp = robs->Joints2Postions(Joints2[i + a], true, posDq);
                    getBase.calculateGammaAndDerivative(tmp, tauBase2[i], diffTauBase2[i]);
                    diffTauBase2[i] = posDq.transpose() * diffTauBase2[i];

                    if (tauBase2[i] > errorTolBase) {
                        solveFlag = false;
                        break;
                    }
                }


                if (i > 0) {
                    vTTP2(i - 1) = toolpath->dPathPoint[i + a] / localTime2(i + a);

                    if (!isInLimits(vTTP2(i - 1), 0, maxVelTTP)) {
                        if (show) {
                            std::cout << "cut because vTTP" << std::endl;
                        }
                        updateTimeFlag = true;
                        break;
                    }
                }
                if ((i + a > 0) && (i + a < toolpath->length - 1)) {
                    double tmp1 = toolpath->dPathPoint[i + a + 1] * localTime2(i + a), tmp2 = toolpath->dPathPoint[i + a] * localTime2(i + a + 1);
                    aTTP_T2(i) = 2 * (tmp1 - tmp2) / (localTime2(i + a) * localTime2(i + a + 1) * (localTime2(i + a) + localTime2(i + a + 1)));

                    double vBarSquare = (pow(tmp1, 2) + pow(tmp2, 2) - 2 * tmp1 * tmp2 * toolpath->cosAngle[i + a]) / (4 * pow(localTime2(i + a), 2) * pow(localTime2(i + a + 1), 2));
                    aTTP_N2(i) = Curvature(i + a) * vBarSquare;
                }
                
                if ((!isInLimits(aTTP_T2(i), -maxAccTTP_T, maxAccTTP_T)) || (!isInLimits(aTTP_N2(i), std::min(-maxAccTTP_N, aTTP_N(i)), std::max(maxAccTTP_N, aTTP_N(i))))) {
                    if (show) {
                        std::cout << "cut because aTTP" << std::endl;
                    }
                    updateTimeFlag = true;
                    break;
                }

            }
            /*maxVelTTP2 = std::max(maxVelTTP2, vTTP2.maxCoeff());
            maxAccTTP_T2 = std::max(maxAccTTP_T2, aTTP_T2.cwiseAbs().maxCoeff());
            maxAccTTP_N2 = std::max(maxAccTTP_N2, aTTP_N2.cwiseAbs().maxCoeff());*/

            if (updateTimeFlag) {
                if (show) {
                    std::cout << "cut because time 1" << std::endl;
                }

                UdTime = 0.5 * UdTime;
                l.block(nCons - n + 1, 0, n - 1, 1) = (-UdTime).cwiseMax(-localTimeUpdate.segment(a + 1, n - 1));
                u.block(nCons - n + 1, 0, n - 1, 1) = UdTime;
                continue;
            }
            if (!solveFlag) {
                if (show) {
                    std::cout<<"cut because 1"<<std::endl;
                }

                UdTheta = 0.5 * UdTheta;
                l.block(nCons - 6 * n + 1, 0, 5 * n, 1) = -UdTheta;
                u.block(nCons - 6 * n + 1, 0, 5 * n, 1) = UdTheta;
                continue;
            }



            Eigen::Matrix<double, 8, 1> maxVel2, maxAcc2, maxJerk2;
            maxVel2 = vJointMax;
            maxAcc2 = aJointMax;
            maxJerk2 = jJointMax;

            for (int i = aL; i < bH + 1; i++) {
                std::vector<Eigen::VectorXd> localJoint(5);
                for (int j = 0; j < 5; j++) {
                    if ((i - 2 + j < 0) || (i - 2 + j > Joints2.size() - 1)) {
                        localJoint[j].resize(8);
                        localJoint[j].setZero();
                    }
                    else {
                        localJoint[j] = Joints2[i - 2 + j];
                    }
                }

                if (i >= a - 1 && i <= b + 1) {
                    if (i == 0) {
                        Vel2[0] = (Joints2[1] - Joints2[0]) / localTime2(1);
                        kVel2[0] << 0, 0, -1 / localTime2(1), 1 / localTime2(1), 0;
                        kAcc2[0] << 0, 0, 1 / pow(localTime2(1), 2),
                            -1 / pow(localTime2(1), 2) - 1 / localTime2(1) / localTime2(2),
                            1 / localTime2(1) / localTime2(2);
                        Acc2[0] = kAcc2[0](2) * Joints2[0] + kAcc2[0](3) * Joints2[1] + kAcc2[0](4) * Joints2[2];
                        dkVel2[0].setZero();
                        dkVel2[0](2, 2) = 1 / pow(localTime2(1), 2);
                        dkVel2[0](3, 2) = -1 / pow(localTime2(1), 2);
                        dkAcc2[0].setZero();
                        dkAcc2[0](2, 2) = -2 / pow(localTime2(1), 3);
                        dkAcc2[0](3, 2) = 2 / pow(localTime2(1), 3) + 1 / pow(localTime2(1), 2) / localTime2(2);
                        dkAcc2[0](3, 3) = 1 / pow(localTime2(2), 2) / localTime2(1);
                        dkAcc2[0](4, 2) = -1 / pow(localTime2(1), 2) / localTime2(2);
                        dkAcc2[0](4, 3) = -1 / pow(localTime2(2), 2) / localTime2(1);

                    }
                    else if (i == toolpath->length - 1) {
                        Vel2[i] = (Joints2[i] - Joints2[i - 1]) / localTime2(i);
                        kVel2[i] << 0, -1 / localTime2(i), 1 / localTime2(i), 0, 0;
                        kAcc2[i] << 1 / localTime2(i) / localTime2(i - 1),
                            -1 / localTime2(i) / localTime2(i - 1) - 1 / pow(localTime2(i), 2),
                            1 / pow(localTime2(i), 2), 0, 0;
                        Acc2[i] = kAcc2[i](0) * Joints2[i - 2] + kAcc2[i](1) * Joints2[i - 1] + kAcc2[i](2) * Joints2[i];
                        dkVel2[i].setZero();
                        dkVel2[i](1, 1) = 1 / pow(localTime2(i), 2);
                        dkVel2[i](2, 1) = -1 / pow(localTime2(i), 2);
                        dkAcc2[i].setZero();
                        dkAcc2[i](0, 0) = -1 / pow(localTime2(i - 1), 2) / localTime2(i);
                        dkAcc2[i](0, 1) = -1 / pow(localTime2(i), 2) / localTime2(i - 1);
                        dkAcc2[i](1, 0) = 1 / pow(localTime2(i - 1), 2) / localTime2(i);
                        dkAcc2[i](1, 1) = 1 / pow(localTime2(i), 2) / localTime2(i - 1) + 2 / pow(localTime2(i), 3);
                        dkAcc2[i](2, 1) = -2 / pow(localTime2(i), 3);
                    }
                    else {
                        std::vector<double> localDist(2);
                        localDist[0] = localTime2(i);
                        localDist[1] = localTime2(i + 1);

                        std::vector<Eigen::VectorXd> localJointTmp(3);
                        localJointTmp.assign(localJoint.begin() + 1, localJoint.begin() + 4);

                        Vel2[i] = velNum(localJointTmp, localDist, kVel2[i]);
                        Acc2[i] = accNum(localJointTmp, localDist, kAcc2[i]);
                        dkVel2[i] = getdkVel(localDist);
                        dkAcc2[i] = getdkAcc(localDist);

                        if (i > 1 && i < toolpath->length - 2) {
                            localDist.resize(4);
                            localDist[0] = localTime2(i - 1);
                            localDist[1] = localTime2(i);
                            localDist[2] = localTime2(i + 1);
                            localDist[3] = localTime2(i + 2);

                            Jerk2[i] = jerkNum(localJoint, localDist, kJerk2[i]);
                            dkJerk2[i] = getdkJerk(localDist);
                        }
                    }

                }
                else {
                    Vel2[i].setZero();
                    Acc2[i].setZero();
                    Jerk2[i].setZero();
                    
                    for (int j = 0; j < 5; j++) {
                        Vel2[i] = Vel2[i] + kVel2[i](j) * localJoint[j];
                        Acc2[i] = Acc2[i] + kAcc2[i](j) * localJoint[j];
                        Jerk2[i] = Jerk2[i] + kJerk2[i](j) * localJoint[j];
                    }
                }


                maxVel2 = maxVel2.cwiseMax(Vel2[i].cwiseAbs());
                maxAcc2 = maxAcc2.cwiseMax(Acc2[i].cwiseAbs());
                maxJerk2 = maxJerk2.cwiseMax(Jerk2[i].cwiseAbs());

                if ((!isInLimits(Vel2[i], -maxVel, maxVel)) || (!isInLimits(Acc2[i], -maxAcc, maxAcc)) || (!isInLimits(Jerk2[i], -maxJerk, maxJerk))) {
                    if (show) {
                        std::cout << "cut because VAJ: " << i << std::endl;
                        std::cout << "Vel: " << Vel2[i].transpose() << "  max:" << maxVel.transpose() << std::endl;
                        std::cout << "Acc: " << Acc2[i].transpose() << "  max:" << maxAcc.transpose() << std::endl;
                        std::cout << "Jerk: " << Jerk2[i].transpose() << "  max:" << maxJerk.transpose() << std::endl;

                        std::cout << "before: " << std::endl;
                        std::cout<< "Vel: " << VelUpdate[i].transpose() << std::endl;
                        std::cout << "Acc: " << AccUpdate[i].transpose() << std::endl;
                        std::cout << "Jerk: " << JerkUpdate[i].transpose() << std::endl;
                    }

                    solveFlag = false;

                    UdTime = 0.5 * UdTime;
                    l.block(nCons - n + 1, 0, n - 1, 1) = (-UdTime).cwiseMax(-localTimeUpdate.segment(a + 1, n - 1));
                    u.block(nCons - n + 1, 0, n - 1, 1) = UdTime;

                    break;
                }

                double phi = 0;
                double tmp = Vel2[i].transpose() * M * Vel2[i];
                phi = k1 * tmp / dVel;
                tmp = Acc2[i].transpose() * M * Acc2[i];
                phi = phi + k2 * tmp / dAcc;
                tmp = Jerk2[i].transpose() * M * Jerk2[i];
                phi = phi + k3 * tmp / dJerk;

                if (i == toolpath->length - 1) {
                    phi_i2(i) = phi * (toolpath->dPathPoint[i]) / 2;
                }
                else {
                    phi_i2(i) = phi * (toolpath->dPathPoint[i] + toolpath->dPathPoint[i + 1]) / 2;
                }

            }


            if (!solveFlag) {
                UdTheta = 0.5 * UdTheta;
                l.block(nCons - 6 * n + 1, 0, 5 * n, 1) = -UdTheta;
                u.block(nCons - 6 * n + 1, 0, 5 * n, 1) = UdTheta;    
                continue;
            }


            Eigen::VectorXd local_phi_i2 = phi_i2.segment(aL, nLH);
            Phi_1 = local_phi_i2.sum();

            Eigen::VectorXd local_time2 = localTime2.segment(a + 1, n - 1);
            time_1 = local_time2.sum();

            PHI_1 = Phi_1 + kTime * time_1 / dTime;

            Eigen::VectorXd aTtpNLarge2(n);
            aTtpNLarge2 = (aTTP_N2.cwiseAbs().array() > maxAccTTP_N).cast<double>();

            PHI_1 = PHI_1 + aTtpNLarge2.dot(aTTP_N2.cwiseProduct(aTTP_N2)) * 1e8;

            //std::cout << "after optimization: " << Phi_1 << std::endl;

            if (PHI_1 < PHI_0) {

                success = true;

                Eigen::Array<bool, Eigen::Dynamic, 1> C;
                C = (UdTheta - dVar.segment(0, 5 * n).cwiseAbs()).array() < (0.01 * dVar.segment(0, 5 * n).cwiseAbs()).array();
                Eigen::VectorXd D = C.cast<double>();
                UdTheta = UdTheta + 0.05 * D.cwiseProduct(UdTheta);

                C = (UdTime - dVar.segment(5 * n, n - 1).cwiseAbs()).array() < (0.01 * dVar.segment(5 * n, n - 1).cwiseAbs()).array();
                D = C.cast<double>();
                UdTime = UdTime + 0.05 * D.cwiseProduct(UdTime);


                JointsUpdate = Joints2;
                TtcpsUpdate = Ttcps2;
                TposUpdate = Tpos2;
                VelUpdate = Vel2;
                AccUpdate = Acc2;
                JerkUpdate = Jerk2;
                phiiUpdate = phi_i2;

                kVelUpdate = kVel2;
                kAccUpdate = kAcc2;
                kJerkUpdate = kJerk2;
                dkVelUpdate = dkVel2;
                dkAccUpdate = dkAcc2;
                dkJerkUpdate = dkJerk2;
                localTimeUpdate = localTime2;

                ThetaUpdate.block(5 * a, 0, 5 * n, 1) = Theta2;

                if (checkColi) {
                    tauLayer = tauLayer2;
                    diffTauLayer = diffTauLayer2;
                    tauBase = tauBase2;
                    diffTauBase = diffTauBase2;
                }


                maxVel = maxVel2;
                maxAcc = maxAcc2;
                maxJerk = maxJerk2;

                vTTP = vTTP2;
                aTTP_T = aTTP_T2;
                aTTP_N = aTTP_N2;
                /*maxVelTTP = maxVelTTP2;
                maxAccTTP_T = maxAccTTP_T2;
                maxAccTTP_N = maxAccTTP_N2;*/
                aTtpNLarge = aTtpNLarge2;

                if (show) {
                    Tfinish = clock();
                    double time = (double)(Tfinish - Tstart) / CLOCKS_PER_SEC;
                    std::cout << "Time in SQP: " << time << std::endl;
                    std::cout << "printing time: " << time_1 << "; smootheness: " << Phi_1 << ";  obj: " << PHI_1 << std::endl;
                    std::cout << "(PHI_0 - PHI_1) / PHI_0=" << (PHI_0 - PHI_1) / PHI_0 << std::endl;
                }


                break;
            }
            else {
                UdTheta = 0.5 * UdTheta;
                l.block(nCons - 6 * n + 1, 0, 5 * n, 1) = -UdTheta;
                u.block(nCons - 6 * n + 1, 0, 5 * n, 1) = UdTheta;

                UdTime = 0.5 * UdTime;
                l.block(nCons - n + 1, 0, n - 1, 1) = (-UdTime).cwiseMax(-localTimeUpdate.segment(a + 1, n - 1));
                u.block(nCons - n + 1, 0, n - 1, 1) = UdTime;

                continue;
            }

        }

        if (exit_code != osqp::OsqpExitCode::kOptimal) {
            break;
        }

    }


    if (success) {
        if ((PHI_ini - PHI_1) / PHI_ini < tau2) {
            success = false;
        }
    }


    Info.JointsUpdate = JointsUpdate;
    Info.TtcpsUpdate = TtcpsUpdate;
    Info.TposUpdate = TposUpdate;
    Info.VelUpdate = VelUpdate;
    Info.AccUpdate = AccUpdate;
    Info.JerkUpdate = JerkUpdate;
    Info.phiiUpdate = phiiUpdate;
    Info.ThetaUpdate = ThetaUpdate;
    Info.a = a;
    Info.b = b;

    Info.kVelUpdate = kVelUpdate;
    Info.kAccUpdate = kAccUpdate;
    Info.kJerkUpdate = kJerkUpdate;
    Info.dkVelUpdate = dkVelUpdate;
    Info.dkAccUpdate = dkAccUpdate;
    Info.dkJerkUpdate = dkJerkUpdate;
    Info.localTimeUpdate = localTimeUpdate;

    Info.velTTPUpdate = vTTP;
    Info.acctTTPUpdate = aTTP_T;
    Info.accnTTPUpdate = aTTP_N;

}



bool trajOpt::isInLimits(Eigen::VectorXd X, Eigen::VectorXd Xmin, Eigen::VectorXd Xmax) {
    Eigen::VectorXd D1, D2;
    D1 = Xmax - X;
    D2 = X - Xmin;
    bool flag;
    if (std::min(D1.minCoeff(), D2.minCoeff()) < 0) {
        flag = false;
    }
    else {
        flag = true;
    }
    return flag;
}

bool trajOpt::isInLimits(double X, double Xmin, double Xmax) {
    double D1, D2;
    D1 = Xmax - X;
    D2 = X - Xmin;
    bool flag;
    if (std::min(D1, D2) < 0) {
        flag = false;
    }
    else {
        flag = true;
    }
    return flag;
}


double trajOpt::getFastronScore(Eigen::VectorXd q, colliDataGotten supportData, Eigen::VectorXd Alpha, robSystem* robs) {
    Eigen::VectorXd posJoints = robs->Joints2PostionsBC(q, 5);

    int n = supportData.coli.size();
    Eigen::VectorXd Kfk(n);

    for (int i = 0; i < n; i++) {
        Kfk(i) = kernelRQ(posJoints, supportData.posJoints[i], 0.7, 2);
    }
    double score = Kfk.dot(Alpha);

    return score;
}

double trajOpt::kernelRQ(Eigen::VectorXd a, Eigen::VectorXd b, double gamma, double p) {
    double tmp = (a - b).norm();
    double kernel = 1 + tmp * tmp * gamma / p;
    kernel = pow(kernel, -p);
    return kernel;
}

double trajOpt::getFastronScore(Eigen::VectorXd q, colliDataGotten supportData, Eigen::VectorXd Alpha, Eigen::VectorXd& diff, robSystem* robs) {
    Eigen::MatrixXd posDjoints;
    Eigen::VectorXd posJoints = robs->Joints2PostionsBC(q, 5, posDjoints);

    int n = supportData.coli.size();
    Eigen::VectorXd Kfk(n);

    Eigen::MatrixXd KfkDposjoint(22, n);

    for (int i = 0; i < n; i++) {
        Kfk(i) = kernelRQ(posJoints, supportData.posJoints[i], 0.7, 2);

        //gamma=0.7 p=2
        double tmp = (posJoints - supportData.posJoints[i]).norm();
        Eigen::VectorXd Tmp = -2 * pow(1 + tmp * tmp * 0.7 / 2, -3) * 0.7 * tmp * (posJoints - supportData.posJoints[i]).normalized();
        KfkDposjoint.block<22, 1>(0, i) = Tmp;
    }
    double score = Kfk.dot(Alpha);

    Eigen::VectorXd scoreDposjoint(22);
    scoreDposjoint = KfkDposjoint * Alpha;

    diff = posDjoints.transpose() * scoreDposjoint;

    return score;
}

void trajOpt::findFeasible(int i, Eigen::VectorXd theta0, Eigen::VectorXd joint0, robSystem* robs, toolpath* toolpath) {
    Eigen::VectorXd theta = Theta.segment(i * 5, 5);
    Eigen::VectorXd theta1 = theta0;

    Eigen::VectorXd jointsolve = joint0;
    while ((theta - theta1).norm() > 0.01) {
        Eigen::VectorXd theta2 = (theta + theta1) / 2;

        Eigen::VectorXd joint(8);
        Eigen::Vector2d BC = theta2.segment(3, 2);
        joint.segment(6, 2) = BC;

        Eigen::Vector3d w = theta2.segment(0, 3);
        Eigen::Matrix3d wHat = mathTools::skew(w);
        Eigen::Matrix3d R = mathTools::expm(wHat);

        Eigen::Vector4d pTmp = robs->fkPos(BC) * toolpath->posWorkpiece[i].matrix().block<4, 1>(0, 3);
        Eigen::Vector3d p = pTmp.block<3, 1>(0, 0);

        Transform3d Ttmp(Transform3d::Identity());
        Ttmp.linear() = R;
        Ttmp.translation() = p;

        Transform3d Ttmp2;
        Ttmp2.matrix() = Ttmp.matrix() * robs->Ttool.matrix().inverse();

        bool flag;
        joint.segment(0, 6) = robs->ikABB2600(flag, Ttmp2, Joints[i].block<6, 1>(0, 0));

        if (!flag) {
            theta = theta2;
            continue;
        }

        flag = robs->checkAllColi(joint);

        if (flag) {
            theta = theta2;
        }
        else {
            theta1 = theta2;
            jointsolve = joint;
        }

    }

    Theta.segment(i * 5, 5) = theta1;
    Joints[i] = jointsolve;
    
    Transform3d Ttmp(Transform3d::Identity());
    Ttmp = robs->fkABB2600(jointsolve.segment(0, 6));
    Ttcps[i].matrix() = Ttmp.matrix() * robs->Ttool.matrix();
    Tpos[i] = robs->fkPos(jointsolve.segment(6, 2));

    int a = std::min(i - 2, 0);
    int b = std::min(i + 2, toolpath->length - 1);
    for (int k = a; k <= b; k++) {
        std::vector<Eigen::Matrix<double, 8, 1>> localJoint(5);

        for (int j = 0; j < 5; j++) {
            if ((k - 2 + j < 0) || (k - 2 + j > Joints.size() - 1)) {
				localJoint[j].setZero();
			}
            else {
				localJoint[j] = Joints[k - 2 + j];
			}
        }

        Vel[k].setZero();
        Acc[k].setZero();
        Jerk[k].setZero();

        for (int j = 0; j < 5; j++) {
			Vel[k] = Vel[k] + kVel[k](j) * localJoint[j];
			Acc[k] = Acc[k] + kAcc[k](j) * localJoint[j];
			Jerk[k] = Jerk[k] + kJerk[k](j) * localJoint[j];
		}

        double phi = 0;
        double tmp = Vel[k].transpose() * M * Vel[k];
        phi = k1 * tmp / dVel;
        tmp = Acc[k].transpose() * M * Acc[k];
        phi = phi + k2 * tmp / dAcc;
        tmp = Jerk[k].transpose() * M * Jerk[k];
        phi = phi + k3 * tmp / dJerk;

        if (k == toolpath->length - 1) {
			phi_i(k) = phi * (toolpath->dPathPoint[k]) / 2;
		}
        else {
			phi_i(k) = phi * (toolpath->dPathPoint[k] + toolpath->dPathPoint[k + 1]) / 2;
		}

    }

}

void trajOpt::getQVAJT(std::vector<Eigen::Matrix<double, 8, 1>>& q, std::vector<Eigen::Matrix<double, 8, 1>>& v, std::vector<Eigen::Matrix<double, 8, 1>>& a,
    std::vector<Eigen::Matrix<double, 8, 1>>& j, Eigen::VectorXd& t, Eigen::VectorXd& vT, Eigen::VectorXd& atT, Eigen::VectorXd& anT) {
    q = Joints;
    v = Vel;
    a = Acc;
    j = Jerk;
    t = localTime;

    vT = velTTP;
    atT = acctTTP;
    anT = accnTTP;
}

void trajOpt::getToTase(int& status, double& k1, double& k2, double& k3, Eigen::MatrixXd& M, Eigen::VectorXd& JointMax, Eigen::VectorXd& JointMin,
    std::vector<Eigen::Matrix<double, 5, 1>>& kJerk, std::vector<Eigen::VectorXd>& Joints, std::vector<Eigen::VectorXd>& Jerk) {
    status = this->status;
    k1 = this->k1;
    k2 = this->k2;
    k3 = this->k3;
    M = this->M;
    JointMax = this->JointMax;
    JointMin = this->JointMin;
    kJerk = this->kJerk;

    Joints.resize(this->Joints.size());
    Jerk.resize(this->Joints.size());
    for (int i = 0; i < Joints.size(); i++) {
		Joints[i] = this->Joints[i];
        Jerk[i] = this->Jerk[i];
	}
}

void trajOpt::getSolution(std::vector<Eigen::Matrix<double, 8, 1>>& q, std::vector<Transform3d>& T, Eigen::VectorXd& time) {
    q = Joints;
    T = Ttcps;
    time = localTime;
}

void trajOpt::setTipCons(double v, double at, double an) {
    vTTPMax = v;
    aTTPMax_T = at;
    aTTPMax_N = an;
    std::cout << "vTTPMax: " << vTTPMax << " aTTPMax_T: " << aTTPMax_T << " aTTPMax_N: " << aTTPMax_N << std::endl;
}

void trajOpt::setvTip(double v) {
    vTip = v;
}

void trajOpt::setM(Eigen::Matrix<double, 8, 8> Minput) {
    M = Minput;
}

void trajOpt::setAngles(double t1, double t2, double t3) {
    alpha = t1;
    beta = t2;
    gamma = t3;
    std::cout << "alpha: " << alpha << " beta: " << beta << " gamma: " << gamma << std::endl;
}

void trajOpt::setVmax(Eigen::Matrix<double, 8, 1> Vmax) {
    vJointMax = Vmax;
    std::cout << "vJointMax: " << vJointMax.transpose() << std::endl;
}

void trajOpt::setJmax(Eigen::Matrix<double, 8, 1> Jmax) {
    jJointMax = Jmax;
    std::cout << "jJointMax: " << jJointMax.transpose() << std::endl;
}



void trajOpt::test(toolpath* toolpath, robSystem* robs, colliTrain* colli, bool checkColi, bool show, int a, int b, bool boolTime) {
    std::cout << "\ntest:" << std::endl;

    if (status != 2) {
        std::cerr << "status wrong in bcd" << std::endl;
    }


    clock_t Tstart, Tfinish;
    double time;
    int n = toolpath->length;


    Eigen::VectorXd ThetaSave0 = Theta;
    std::vector<Eigen::Matrix<double, 8, 1>> JointsSave0 = Joints;

    std::cout << "initial smoothness index: " << phi_i.sum() << std::endl;
    std::cout << "initial time: " << localTime.sum() << std::endl;

    std::vector<double> iterTime, iterPhi;
    iterTime.push_back(0);
    iterPhi.push_back(phi_i.sum());


    double Phi_1 = phi_i.sum() + kTime * localTime.sum() / dTime;
    double Phi_0 = 2 * Phi_1;

    updateInfo Info;
    bool success;

    if (boolTime) {
        local_optimal_time(toolpath, robs, colli, success, a, b, Info, checkColi, show);
    }
    else {
        local_optimal(toolpath, robs, colli, success, a, b, Info, checkColi, show);
    }
    

    std::cout << "test done\n" << std::endl;

}