#pragma once
#include "../QMeshLib/PolygenMesh.h"

#include <cmath>
#include "svm.h"
#include "svm_grad.h"
#include "robSystem.h"
#include "PQPLib/PQP.h"
//#include <thread>
//#include <future>

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

struct colliDataGotten {
	std::vector<int> coli;
	std::vector<Eigen::VectorXd> posJoints;
	std::vector<Eigen::VectorXd> Joints;
};


class colliTrain
{
public:
	colliTrain();
	~colliTrain() {};

	void svmtrain(int n, char** command, int D);
	void svmpredict(int n, char** command);

	colliDataGotten samFirstLayer(QMeshPatch* layer, toolpath* path, robSystem* robs, PQP_Model* layer_PQP, int typePosJoints, int indexLayer);
	colliDataGotten FastronModelUpdate(colliDataGotten data);

	colliDataGotten FastronActiveLearning(QMeshPatch* layer, int indexLayer, toolpath* path, robSystem* robs, int typePosJoints);

	Eigen::VectorXd getProxyColli(colliDataGotten supportData);
	double getProxyScore(Eigen::VectorXd q, colliDataGotten supportData, Eigen::VectorXd A, Eigen::VectorXd& diff, robSystem* robs);

	double getFastronScore(Eigen::VectorXd q, colliDataGotten supportData, Eigen::VectorXd Alpha, Eigen::VectorXd& diff, robSystem* robs);
	double getFastronScore(Eigen::VectorXd posJoints, colliDataGotten supportData, Eigen::VectorXd Alpha);

	Eigen::VectorXd alpha;
	Eigen::VectorXd F;
	colliDataGotten supportData;

	bool newFastron = true;

private:
	void parse_command_line(int n, char** command, char* input_file_name, char* model_file_name);
	void read_problem(const char* filename);
	void do_cross_validation();
	int svm_save_model_grad(const char* model_file_name, const svm_model* model, int D);
	char* readline(FILE* input);
	void predict(FILE* input, FILE* output);

	struct svm_parameter param;
	struct svm_problem prob;
	struct svm_node* x_space;
	struct svm_model* model;
	int cross_validation;
	int nr_fold;
	int max_line_len;
	char* line;


	int predict_probability;
	int max_nr_attr;
	struct svm_node* x;
	int (*info)(const char* fmt, ...) = &printf;

	void GenerateNewSample(Eigen::VectorXd a, Eigen::VectorXd b,
		std::vector<Eigen::VectorXd>& qSam, PQP_Model* layer_PQP, robSystem* robs);
	Eigen::VectorXd GenerateNewSample2(Eigen::VectorXd a, Eigen::VectorXd b,
		PQP_Model* layer_PQP, robSystem* robs);
	double distToolLayer(Eigen::Matrix<double, 8, 1> q, PQP_Model* layer_PQP, Eigen::Vector3d& q2, robSystem* robs);

	
	//Eigen::MatrixXd K;
	

	Eigen::VectorXd computeGramMatrixColumn(int i, std::vector<Eigen::VectorXd> posJoints);
	double kernelRQ(Eigen::VectorXd a, Eigen::VectorXd b, double gamma, double p);
	double kernelPH(Eigen::VectorXd a, Eigen::VectorXd b, int k);
	void removeRowAndCol(Eigen::MatrixXd& matrix, unsigned int rowToRemove, unsigned int colToRemove);
	void removeElement(Eigen::VectorXd& vector, unsigned int indexToRemove);
};
