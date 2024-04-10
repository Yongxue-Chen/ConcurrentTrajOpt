#ifndef __SVMGrad_H__
#define __SVMGrad_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include "armadillo"
#include <Eigen\Dense>


//using namespace arma;
using namespace std;
using namespace arma;

typedef Eigen::VectorXd vecEig;

/* SVMGrad: This structure holds the parameters of an RBF-SVM
 * D:       Datapoint Dimension
 * nSV:     # of SVs
 * b:       Offset
 * sigma:   kernel width
 * yalphas: alpha_i*yi     (1 X nSV)
 * SVs:     Support Vector (D X nSV)
*/

struct SVMGradModels{
    unsigned int D;
    unsigned int nSV;
    double sigma;
    double b;
    vec yalphas;
    mat SVs;
};

/* SVMGrad: This class computes the following function from an SMVGradModel
 *       y      = sign(Gamma(x))
 *       Gamma  = \sum_{i=1}^{N_sv}\alpha_iy_ik(x,x_i) + b
 *       DGamma = \sum_{i=1}^{N_sv}-1/2\sigma^2\alpha_iy_ik(x,x_i)(x-x_i)
*/

class SVMGrad
{
private:
    SVMGradModels SVMGradModel;
    vec    diffx;
    vec    kernelVec;
    mat    diffxMat;
    bool   storeKernel;
    double lambda; // lambda = 1/(2*sigma^2)
    double y;      // y      = sign(Gamma(x))

    inline double getKernel(vec x, unsigned int s);
    inline vec getKernelDerivative(vec x, unsigned int s);

public:       
    SVMGrad();
    SVMGrad(string& f_SVMGradmodel);
    void loadModel(string& f_SVMGradmodel);

    // Armadillo input
    void preComputeKernel(bool precompute);
    double calculateClass(vec x);
    double calculateGamma(vec x);
    vec calculateGammaDerivative(vec x);
    void calculateGammaAndDerivative(vec x, double& gamma, vec& gamma_der);

    // Eigen input
    void eigen2arma(vecEig x_in, vec& x_out);
    void arma2eigen(vec x_in, vecEig& x_out);
    double calculateGamma(vecEig x);
    vecEig calculateGammaDerivative(vecEig x);
    void calculateGammaAndDerivative(vecEig x, double& gamma, vecEig& gamma_der);

};


#endif //__SVMGrad_H__
