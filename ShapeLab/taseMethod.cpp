#include "taseMethod.h"

double sumJerk(const std::vector<double>& x, std::vector<double>& grad, void* f_data) {
    int n = x.size() / 8;
    std::vector<Eigen::VectorXd> q(n);
    for (int i = 0; i < n; i++) {
        q[i] = Eigen::Map<const Eigen::VectorXd>(&x[i * 8], 8);
    }

    infoForNlopt* fData = (infoForNlopt*)f_data;


    std::vector<Eigen::VectorXd> qOther = fData->qOther;
    std::vector<Eigen::VectorXd> tmpKJerk = fData->tmpKJerk;
    Eigen::MatrixXd M = fData->M;

    std::vector<Eigen::VectorXd> qAll(n + 4);
    for (int i = 0; i < n; i++) {
        qAll[i + 2] = q[i];
    }
    qAll[0] = qOther[0];
    qAll[1] = qOther[1];
    qAll[n + 2] = qOther[2];
    qAll[n + 3] = qOther[3];

    double sumJerk = 0;

    std::vector<Eigen::VectorXd> jerk(n);
    for (int i = 0; i < n; i++) {
        jerk[i].setZero(8);
        for (int j = 0; j < 5; j++) {
            jerk[i] += tmpKJerk[i](j) * qAll[i + j];
        }
        sumJerk += jerk[i].transpose() * M * jerk[i];
    }

    std::vector<Eigen::MatrixXd> djerk(n);
    for (int i = 0; i < n; i++) {
        djerk[i].setZero(8, 8 * n);
        djerk[i].block(0, i * 8, 8, 8) = Eigen::Matrix<double, 8, 8>::Identity() * tmpKJerk[i](2);
        if (i > 0) {
            djerk[i].block(0, (i - 1) * 8, 8, 8) = Eigen::Matrix<double, 8, 8>::Identity() * tmpKJerk[i](1);
            if (i > 1) {
                djerk[i].block(0, (i - 2) * 8, 8, 8) = Eigen::Matrix<double, 8, 8>::Identity() * tmpKJerk[i](0);
            }
        }
        if (i < n - 1) {
            djerk[i].block(0, (i + 1) * 8, 8, 8) = Eigen::Matrix<double, 8, 8>::Identity() * tmpKJerk[i](3);
            if (i < n - 2) {
                djerk[i].block(0, (i + 2) * 8, 8, 8) = Eigen::Matrix<double, 8, 8>::Identity() * tmpKJerk[i](4);
            }
        }
    }

    Eigen::MatrixXd vecGrad = Eigen::MatrixXd::Zero(1, 8 * n);
    for (int i = 0; i < n; i++) {
        vecGrad += 2 * jerk[i].transpose() * M * djerk[i];
    }

    grad.resize(8 * n);
    for (int i = 0; i < 8 * n; i++) {
        grad[i] = vecGrad(0, i);
    }

    return sumJerk;
}

double myConstraint(const std::vector<double>& x, std::vector<double>& grad, void* f_data) {
    int n = x.size() / 8;
    std::vector<Eigen::VectorXd> q(n);
    for (int i = 0; i < n; i++) {
        q[i] = Eigen::Map<const Eigen::VectorXd>(&x[i * 8], 8);
    }

    infoForNlopt* fData = (infoForNlopt*)f_data;

    robSystem* robs = fData->robs;
    std::vector<Eigen::Vector3d> pList = fData->pList;
    std::vector<Eigen::Vector3d> nList = fData->nList;
    
    double error = 0;
    for (int i = 0; i < n; i++) {
        Transform3d T = robs->fkABB2600(q[i].segment(0, 6));
        T.matrix() = T.matrix() * robs->Ttool.matrix();
        Eigen::Vector4d p = T.matrix().block(0, 3, 4, 1);
        Eigen::Vector3d tmp1 = T.matrix().block(0, 2, 3, 1);
        double tmp = Eigen::Vector3d(0, 0, -1).dot(tmp1);
        tmp = fabs(tmp - 1);
        //std::cout << "tmp1: " << tmp << std::endl;
        error += tmp;

        T = robs->fkPos(q[i].segment(6, 2));
        tmp1 = T.matrix().block(0, 0, 3, 3) * nList[i];
        tmp = Eigen::Vector3d(0, 0, 1).dot(tmp1);
        tmp = fabs(tmp - 1);
        //std::cout<< "tmp2: " << tmp << std::endl;
        error += tmp;

        p = T.matrix().inverse() * p;
        tmp = (p.segment(0, 3) - pList[i]).norm();
        //std::cout << "tmp3: " << tmp << std::endl;
        error += tmp*1000;
    }

    //std::cout << "get error: " << error << std::endl;

    grad.resize(8 * n);

#pragma omp parallel for
    for (int i = 0; i < 8 * n; i++) {
        std::vector<double> newx = x;
        newx[i] += 0.0001;

        double error1 = 0;
        {
            std::vector<Eigen::VectorXd> q(n);
            for (int j = 0; j < n; j++) {
                q[j] = Eigen::Map<const Eigen::VectorXd>(&newx[j * 8], 8);
            }

            for (int j = 0; j < n; j++) {
                Transform3d T = robs->fkABB2600(q[j].segment(0, 6));
                T.matrix() = T.matrix() * robs->Ttool.matrix();
                Eigen::Vector4d p = T.matrix().block(0, 3, 4, 1);
                Eigen::Vector3d tmp1 = T.matrix().block(0, 2, 3, 1);
                double tmp = Eigen::Vector3d(0, 0, -1).dot(tmp1);
                tmp = fabs(tmp - 1);
                error1 += tmp;

                T = robs->fkPos(q[j].segment(6, 2));
                tmp1 = T.matrix().block(0, 0, 3, 3) * nList[j];
                tmp = Eigen::Vector3d(0, 0, 1).dot(tmp1);
                tmp = fabs(tmp - 1);
                error1 += tmp;

                p = T.matrix().inverse() * p;
                tmp = (p.segment(0, 3) - pList[j]).norm();
                error1 += tmp * 1000;
            }
        }

        grad[i] = (error1 - error) / 0.0001;

	}

    //std::cout << error << std::endl;
    return error;
}


void taseMethod::initialize(trajOpt* Opt) {
	std::cout << "Initialize TASE" << std::endl;

    Opt->getToTase(status, k1, k2, k3, M, JointMax, JointMin, kJerk, Joints, Jerk);

    //std::cout<<Joints.size()<<std::endl;

    //if (k1 != 0 || k2 != 0) {
    //    std::cerr << "k1 and k2 should be 0" << std::endl;
    //}
}


void taseMethod::solve(toolpath* toolpath, robSystem* robs) {
    std::cout << "\nLocal filtering in Tase" << std::endl;

    if (status != 2) {
        std::cerr << "status wrong in bcd" << std::endl;
    }

    clock_t Tstart, Tfinish;
    double time;
    int n = toolpath->length;

    //std::cout<<n<<std::endl;

    std::vector<Eigen::VectorXd> JointsSave0 = Joints;

    //std::cout << Joints.size() << std::endl;

    Eigen::VectorXd normJerk(n);
    for (int i = 0; i < n; i++) {
        normJerk(i) = Jerk[i].transpose() * M * Jerk[i];
    }

    std::cout << "initial: " << normJerk.sum() << std::endl;

    std::vector<double> iterTime, iterPhi;
    iterTime.push_back(0);
    iterPhi.push_back(normJerk.sum());

    Eigen::VectorXd State = Eigen::VectorXd::Ones(n);
    State(0) = 0; State(1) = 0; State(n - 1) = 0; State(n - 2) = 0;

    Eigen::VectorXd toCompare(n);
    toCompare = State.cwiseProduct(normJerk);

    int maxNormJerkIndex;
    double maxNormJerk = toCompare.maxCoeff(&maxNormJerkIndex);

    int iter = 0;

    

    Tstart = clock();
    while (iter < maxIter && State.sum()>0 && Jerk[maxNormJerkIndex].cwiseAbs().maxCoeff() > 20) {
        iter++;

        std::cout << iter << " ";

        int d = 5;
        bool success = false;

        int a, b;
        while (!success && d <= dmax) {
            a = std::max(2, maxNormJerkIndex - d);
            b = std::min(n - 3, maxNormJerkIndex + d);

            localMinJerk(success, a, b, robs, toolpath);

            if (!success) {
				d += 5;
			}
            else {
                State.segment(a, b - a + 1).setOnes();
            }
        }

        if (!success) {
            State.segment(a, b - a + 1).setZero();
        }

        for (int i = 2; i < n - 2; i++) {
            normJerk(i) = Jerk[i].transpose() * M * Jerk[i];
        }

        toCompare = State.cwiseProduct(normJerk);
        maxNormJerk = toCompare.maxCoeff(&maxNormJerkIndex);

        iterPhi.push_back(normJerk.sum());

        Tfinish = clock();
        time = (double)(Tfinish - Tstart) / CLOCKS_PER_SEC;
        std::cout << "Time: " << time << "   Phi: " << normJerk.sum() << std::endl;
    }


 //   for (int i = 0; i < n; i++) {
 //       Eigen::VectorXd Jtmp = Joints[i];
 //       Transform3d Ttmp = robs->fkABB2600(Jtmp.segment(0, 6));
 //       Ttmp.matrix() = Ttmp.matrix() * robs->Ttool.matrix();
 //       
 //       Transform3d Tpos = robs->fkPos(Jtmp.segment(6, 2));
 //       Eigen::Vector3d ptmp = toolpath->pathPoint[i];
 //       Eigen::Vector4d ptmp4(ptmp(0), ptmp(1), ptmp(2), 1);
 //       ptmp4 = Tpos.matrix() * ptmp4;
	//}






    std::cout << "check collision..." << std::endl;
    for (int i = 0; i < n; i++) {
        if (i % 100 == 0) {
            std::cout << "." << std::flush;
        }

        Eigen::Matrix<double, 8, 1> tmp = Joints[i];
        bool coli = robs->checkAllColi(tmp);

        if (coli) {
            Joints[i] = JointsSave0[i];
            
            for (int j = std::max(0, i - 2); j <= std::min(n - 1, i + 2); j++) {
                Jerk[j].setZero(8);
                if (j == 0) {
                    Jerk[j] = kJerk[j](2) * Joints[j] + kJerk[j](3) * Joints[j + 1] + kJerk[j](4) * Joints[j + 2];
				}
				else if (j == 1) {
					Jerk[j] = kJerk[j](1) * Joints[j - 1] + kJerk[j](2) * Joints[j] + kJerk[j](3) * Joints[j + 1] + kJerk[j](4) * Joints[j + 2];
				}
				else if (j == n - 2) {
					Jerk[j] = kJerk[j](0) * Joints[j - 2] + kJerk[j](1) * Joints[j - 1] + kJerk[j](2) * Joints[j] + kJerk[j](3) * Joints[j + 1];
				}
				else if (j == n - 1) {
					Jerk[j] = kJerk[j](0) * Joints[j - 2] + kJerk[j](1) * Joints[j - 1] + kJerk[j](2) * Joints[j];
				}
				else {
					Jerk[j] = kJerk[j](0) * Joints[j - 2] + kJerk[j](1) * Joints[j - 1] + kJerk[j](2) * Joints[j] + kJerk[j](3) * Joints[j + 1] + kJerk[j](4) * Joints[j + 2];  
                }
			}
        }
    }
    std::cout << std::endl;


    fileIO* IO = new fileIO();
    IO->writeVar8(Joints,"../DataSet/output/JointsTase.txt");
    IO->writeVar8(Jerk, "../DataSet/output/JerkTase.txt");
    delete IO;

    std::cout << "Tase Solved\n" << std::endl;
}

void taseMethod::localMinJerk(bool &success, int a,int b, robSystem* robs, toolpath* toolpath) {

    //std::cout<<"localMinJerk: "<<a<<" "<<b<<std::endl;

    std::vector<Eigen::VectorXd> qNew = Joints;
    std::vector<Eigen::VectorXd> jerkNew = Jerk;

    int nPath = Joints.size();

    int n = b - a + 1;

    std::vector<double> x0(8 * n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < 8; j++) {
            x0[i * 8 + j] = Joints[a + i](j);
        }
    }

    infoForNlopt* fData = new infoForNlopt;
    fData->qOther.resize(4);
    fData->qOther[0] = Joints[a - 2];
    fData->qOther[1] = Joints[a - 1];
    fData->qOther[2] = Joints[b + 1];
    fData->qOther[3] = Joints[b + 2];
    fData->tmpKJerk.resize(n);
    fData->M = M;
    fData->robs = robs;
    fData->pList.resize(n);
    fData->nList.resize(n);
    for (int i = 0; i < n; i++) {
        fData->tmpKJerk[i] = kJerk[a + i];
        fData->pList[i] = toolpath->pathPoint[i + a];
        fData->nList[i] = toolpath->pathNormal[i + a];
	}

    std::vector<double> tmp;
    //double cons = myConstraint(x0, tmp, fData);
    //std::cout << "initial constraint: " << cons << std::endl;

    double inif = sumJerk(x0, tmp, fData);
    //std::cout<<"initial: "<<inif<<std::endl;

    std::vector<double> lb(8 * n), ub(8 * n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < 8; j++) {
			lb[i * 8 + j] = JointMin(j);
			ub[i * 8 + j] = JointMax(j);
		}
    }
    
    nlopt::opt opter;
    opter = nlopt::opt(nlopt::LD_MMA, 8 * n);
    opter.set_lower_bounds(lb);
    opter.set_upper_bounds(ub);
    opter.set_min_objective(sumJerk, fData);

    double consTol=myConstraint(x0, tmp, fData);
    opter.add_inequality_constraint(myConstraint, fData, consTol);
    opter.set_maxeval(2000);
    //opter.set_xtol_rel(1e-10);
    //opter.set_ftol_rel(1e-4);

    double minf;
    nlopt::result res = opter.optimize(x0, minf);

    //std::cout << a << "-" << b << ": " << res << std::endl;

    //判断是否成功，成功则更新Joints和Jerk
    if (res > 0 && minf < inif) {
        success = true;
        for (int i = 0; i < n; i++) {
            qNew[a + i] = Eigen::Map<const Eigen::VectorXd>(&x0[i * 8], 8);
        }

        std::vector<Eigen::VectorXd> qAll(n + 4);
        for (int i = 0; i < n; i++) {
            qAll[i + 2] = qNew[a + i];
        }
        qAll[0] = fData->qOther[0];
        qAll[1] = fData->qOther[1];
        qAll[n + 2] = fData->qOther[2];
        qAll[n + 3] = fData->qOther[3];

        for (int i = 0; i < n; i++) {
            jerkNew[a + i].setZero(8);
            for (int j = 0; j < 5; j++) {
                jerkNew[a + i] += fData->tmpKJerk[i](j) * qAll[i + j];
            }
        }

        Joints = qNew;
        Jerk = jerkNew;
    }
    else {
		success = false;
	}

    delete fData;
}

