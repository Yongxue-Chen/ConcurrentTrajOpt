#include "svm_grad.h"


SVMGrad::SVMGrad(){

}

SVMGrad::SVMGrad(string& f_SVMGradmodel)
{

    loadModel(f_SVMGradmodel);
    storeKernel = false;
}

void SVMGrad::loadModel(string &f_SVMGradmodel){

    //cout << "SVM Model: " << f_SVMGradmodel << endl;
    ifstream fin(f_SVMGradmodel.c_str());
    unsigned int    d, s;

    // Load SVMGrad Model
    fin >> SVMGradModel.D
        >> SVMGradModel.nSV
        >> SVMGradModel.b
        >> SVMGradModel.sigma;

    /*cout << "model.D: " << SVMGradModel.D << endl
         << "model.nSV: " << SVMGradModel.nSV << endl
         << "model.b: " << SVMGradModel.b << endl
         << "model.sigma: " << SVMGradModel.sigma << endl;*/

    SVMGradModel.yalphas = zeros<colvec>(SVMGradModel.nSV);
    SVMGradModel.SVs     = zeros<mat>(SVMGradModel.D, SVMGradModel.nSV);

    for( s=0; s<SVMGradModel.nSV; s++ ){
        fin >> SVMGradModel.yalphas(s);
    }

    for( d=0; d<SVMGradModel.D; d++ ){
        for( s=0; s<SVMGradModel.nSV; s++ ){
            fin >> SVMGradModel.SVs(d,s);
        }
    }
    fin.close();


    lambda = 1/(2*SVMGradModel.sigma*SVMGradModel.sigma);
    diffx = zeros<vec>(SVMGradModel.D);

}

void SVMGrad::preComputeKernel(bool precompute){
   storeKernel = precompute;
   if (storeKernel){
            kernelVec = zeros<vec>(SVMGradModel.nSV);
            diffxMat  = zeros<mat>(SVMGradModel.D, SVMGradModel.nSV);
   }

}


double SVMGrad::calculateClass(vec x)
{
    y = calculateGamma(x);
    return y<0? -1 : y>0;
}

double SVMGrad::calculateGamma(vec x)
{

    double gamma_val = 0.0000;
    kernelVec = zeros<vec>(SVMGradModel.nSV);
    diffxMat  = zeros<mat>(SVMGradModel.D, SVMGradModel.nSV);

    for(unsigned int s=0; s<SVMGradModel.nSV; s++ ){
        // \alpha_i*y_i*exp(-lamdba*||x-x_i||^2))
        gamma_val += SVMGradModel.yalphas(s)*getKernel(x, s);
    }

    return (gamma_val + SVMGradModel.b);
}


double SVMGrad::calculateGamma(vecEig x)
{

    // Convert input to arma
    vec x_arma;
    eigen2arma(x,x_arma);

    // Compute gamma
    double gamma;
    gamma = calculateGamma(x_arma);

    return gamma;
}



vec SVMGrad::calculateGammaDerivative(vec x)
{

    vec gammaDer_val  = zeros<vec>(SVMGradModel.D);

    for(unsigned int s=0; s<SVMGradModel.nSV; s++ ){
        // [\alpha_i*y_i]*[(-2*lambda)*exp(-lamdba*||x-x_i||^2))*(x-x_i)]
        gammaDer_val += SVMGradModel.yalphas(s)*getKernelDerivative(x, s);
    }

    return gammaDer_val;
}


vecEig SVMGrad::calculateGammaDerivative(vecEig x)
{
    // Convert input to arma
    vec x_arma;
    eigen2arma(x,x_arma);

    // Compute derivative
    vec gamma_der;
    gamma_der = calculateGammaDerivative(x_arma);

    // Convert derivative to eigen
    vecEig gamma_der_eig;
    arma2eigen(gamma_der,gamma_der_eig);

    return gamma_der_eig;
}

void SVMGrad::calculateGammaAndDerivative(vec x, double& gamma, vec& gamma_der)
{
    gamma     = 0.0000;
    gamma_der  = zeros<vec>(SVMGradModel.D);

    kernelVec = zeros<vec>(SVMGradModel.nSV);
    diffxMat  = zeros<mat>(SVMGradModel.D, SVMGradModel.nSV);

    for(unsigned int s=0; s<SVMGradModel.nSV; s++ ){
        // \alpha_i*y_i*exp(-lamdba*||x-x_i||^2))
        gamma += SVMGradModel.yalphas(s)*getKernel(x, s);

        // [\alpha_i*y_i]*[(-2*lambda)*exp(-lamdba*||x-x_i||^2))*(x-x_i)]
        gamma_der += SVMGradModel.yalphas(s)*getKernelDerivative(x, s);
    }

    gamma += SVMGradModel.b;
}


void SVMGrad::calculateGammaAndDerivative(vecEig x, double& gamma, vecEig& gamma_der){

    // Convert input to arma
    vec x_arma; vec gamma_der_arma;
    eigen2arma(x,x_arma);

    // Compute gamma and derivative
    calculateGammaAndDerivative(x_arma, gamma, gamma_der_arma);

    // Convert derivative to eigen
    arma2eigen(gamma_der_arma,gamma_der);
}


inline double SVMGrad::getKernel(vec x, unsigned int s){

    double kernel = 0.0;

    // Compute vector diff
    diffx = zeros<vec>(SVMGradModel.D);
    diffx = x - SVMGradModel.SVs.col(s);

    if (storeKernel){
        diffxMat.col(s) = diffx;
    }
    // RBF Kernel Exponential
    kernel = exp(-lambda*pow(norm(diffx),2));
    if (storeKernel){
            kernelVec(s) = kernel;
    }

    return kernel;
}


inline vec SVMGrad::getKernelDerivative(vec x, unsigned int s){

    double kernel  = 0.0;
    vec kernelDer  = zeros<vec>(SVMGradModel.D);

    diffx = zeros<vec>(SVMGradModel.D);

    if (storeKernel){
        diffx = diffxMat.col(s);
        kernel = kernelVec(s);
    }
    else{
        diffx = x - SVMGradModel.SVs.col(s);
        kernel = exp(-lambda*pow(norm(diffx),2));
    }

    // RBF Kernel Derivative wrt x
    kernelDer = -2*lambda*kernel*diffx;

    return kernelDer;
}


void SVMGrad::eigen2arma(vecEig x_in, vec& x_out){
    x_out = zeros<vec>(x_in.rows());
    for (unsigned int i=0;i < x_in.rows(); i++)
        x_out(i) = x_in(i);
}

void SVMGrad::arma2eigen(vec x_in, vecEig& x_out){
    x_out.resize(x_in.size());
    for (unsigned int i=0;i < x_in.size(); i++)
        x_out(i) = x_in(i);
}

