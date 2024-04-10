#include "colliTrain.h"



//int checkLayerColinew(robSystem* robs, Eigen::Matrix<double, 8, 1> q, int index, std::promise<int>& promiseObj) {
//	std::vector<Transform3d> TransABBTmp(7);
//	Transform3d TransPosTmp;
//	robs->fkABB2600(q.block<6, 1>(0, 0), TransABBTmp);
//	TransPosTmp = robs->fkPos(q.block<2, 1>(6, 0));
//
//	int flag = -1;
//
//	fcl::BroadPhaseCollisionManagerd* manager1 = new fcl::DynamicAABBTreeCollisionManagerd();
//	fcl::BroadPhaseCollisionManagerd* manager2 = new fcl::DynamicAABBTreeCollisionManagerd();
//
//	Transform3d T1(Transform3d::Identity());
//	T1.matrix() = TransABBTmp[6].matrix() * robs->TransABB0[6].matrix().inverse();
//	manager1->registerObject(new fcl::CollisionObjectd(robs->fclABB[6], T1));
//
//	manager2->registerObject(new fcl::CollisionObjectd(robs->fclLayer[index], TransPosTmp));
//
//	manager1->setup();
//	manager2->setup();
//
//	fcl::DefaultCollisionData<double> collision_data;
//
//	manager1->collide(manager2, &collision_data, fcl::DefaultCollisionFunction);
//
//	if (collision_data.result.isCollision()) {
//		flag = 1;
//	}
//
//	delete manager1;
//	delete manager2;
//
//	promiseObj.set_value(flag);
//
//	return flag;
//}


colliTrain::colliTrain()
{
	line=NULL;
	predict_probability = 0;
	max_nr_attr = 64;
}



void colliTrain::svmtrain(int n, char** command, int D)
{
//  Usage: svm - train[options] training_set_file[model_file]
//  options :
//	-s svm_type : set type of SVM(default 0)
//	    0 --C - SVC(multi - class classification)
//	    1 --nu - SVC(multi - class classification)
//	    2 --one - class SVM
//	    3 --epsilon - SVR(regression)
//	    4 --nu - SVR(regression)
//	- t kernel_type : set type of kernel function(default 2)
//	    0 --linear : u'*v
//	    1 --polynomial : (gamma * u'*v + coef0)^degree
//	    2 --radial basis function : exp(-gamma * | u - v | ^ 2)
//		3 --sigmoid : tanh(gamma * u'*v + coef0)
//		4 --precomputed kernel(kernel values in training_set_file)
//	- d degree : set degree in kernel function(default 3)
//	- g gamma : set gamma in kernel function(default 1 / num_features)
//	- r coef0 : set coef0 in kernel function(default 0)
//	- c cost : set the parameter C of C - SVC, epsilon - SVR, and nu - SVR(default 1)
//	- n nu : set the parameter nu of nu - SVC, one - class SVM, and nu - SVR(default 0.5)
//	- p epsilon : set the epsilon in loss function of epsilon - SVR(default 0.1)
//	- m cachesize : set cache memory size in MB(default 100)
//	- e epsilon : set tolerance of termination criterion(default 0.001)
//	- h shrinking : whether to use the shrinking heuristics, 0 or 1 (default 1)
//	- b probability_estimates : whether to train a model for probability estimates, 0 or 1 (default 0)
//	- wi weight : set the parameter C of class i to weight * C, for C - SVC(default 1)
//	- v n : n - fold cross validation mode
//	- q : quiet mode(no outputs)

	//D: the number of features

	std::cout << "Training SVM" << std::endl;

	char input_file_name[1024];
	char model_file_name[1024];
	const char* error_msg;

	parse_command_line(n, command, input_file_name, model_file_name);
	read_problem(input_file_name);
	error_msg = svm_check_parameter(&prob, &param);

	if (error_msg)
	{
		fprintf(stderr, "ERROR: %s\n", error_msg);
		exit(1);
	}

	if (cross_validation)
	{
		do_cross_validation();
	}
	else
	{
		model = svm_train(&prob, &param);

		if (D > 0) {
			char model_file_name_grad[1024];
			sprintf(model_file_name_grad, "%sG", model_file_name);
			svm_save_model_grad(model_file_name_grad, model, D);
		}

		if (svm_save_model(model_file_name, model))
		{
			fprintf(stderr, "can't save model to file %s\n", model_file_name);
			exit(1);
		}
		svm_free_and_destroy_model(&model);
	}
	svm_destroy_param(&param);
	free(prob.y);
	free(prob.x);
	free(x_space);
	line = NULL;
}

void colliTrain::parse_command_line(int n, char** command, char* input_file_name, char* model_file_name) {
	//std::cout << "in parse_command_line" << std::endl;
	int i;
	void (*print_func)(const char*) = NULL;	// default printing to stdout

	// default values
	param.svm_type = C_SVC;
	param.kernel_type = RBF;
	param.degree = 3;
	param.gamma = 0;	// 1/num_features
	param.coef0 = 0;
	param.nu = 0.5;
	param.cache_size = 100;
	param.C = 1;
	param.eps = 1e-3;
	param.p = 0.1;
	param.shrinking = 1;
	param.probability = 0;
	param.nr_weight = 0;
	param.weight_label = NULL;
	param.weight = NULL;
	cross_validation = 0;

	//std::cout << "command[0][0]" << command[0][0] << std::endl;
	// parse options
	for (i = 0; i < n; i++)
	{
		if (command[i][0] != '-') {
			//std::cout << "the first command is not -" << std::endl;
			break;
		}
		if (++i >= n) {
			std::cerr << "input to svmtrain is wrong" << std::endl;
			exit(1);
		}

		//std::cout << "command[i]" << command[i] << std::endl;
		switch (command[i - 1][1])
		{
		case 's':
			//std::cout << "OK!\n s=" << command[i] << std::endl;
			param.svm_type = atoi(command[i]);
			break;
		case 't':
			param.kernel_type = atoi(command[i]);
			break;
		case 'd':
			param.degree = atoi(command[i]);
			break;
		case 'g':
			param.gamma = atof(command[i]);
			break;
		case 'r':
			param.coef0 = atof(command[i]);
			break;
		case 'n':
			param.nu = atof(command[i]);
			break;
		case 'm':
			param.cache_size = atof(command[i]);
			break;
		case 'c':
			param.C = atof(command[i]);
			break;
		case 'e':
			param.eps = atof(command[i]);
			break;
		case 'p':
			param.p = atof(command[i]);
			break;
		case 'h':
			param.shrinking = atoi(command[i]);
			break;
		case 'b':
			param.probability = atoi(command[i]);
			break;
			//case 'q':
			//	print_func = &print_null;
			//	i--;
			//	break;
		case 'v':
			cross_validation = 1;
			nr_fold = atoi(command[i]);
			if (nr_fold < 2)
			{
				fprintf(stderr, "n-fold cross validation: n must >= 2\n");
				return;
			}
			break;
		case 'w':
			++param.nr_weight;
			param.weight_label = (int*)realloc(param.weight_label, sizeof(int) * param.nr_weight);
			param.weight = (double*)realloc(param.weight, sizeof(double) * param.nr_weight);
			param.weight_label[param.nr_weight - 1] = atoi(&command[i - 1][2]);
			param.weight[param.nr_weight - 1] = atof(command[i]);
			break;
		default:
			fprintf(stderr, "Unknown option: -%c\n", command[i - 1][1]);
			return;
		}
	}

	svm_set_print_string_function(print_func);

	// determine filenames

	if (i >= n)
		std::cerr << "no input file" << std::endl;

	strcpy(input_file_name, command[i]);

	if (i < n - 1)
		strcpy(model_file_name, command[i + 1]);
	else
	{
		//char* p = strrchr(command[i], '/');
		//if (p == NULL)
		//	p = command[i];
		//else
		//	++p;
		char* p;
		p = command[i];
		sprintf(model_file_name, "%s.model", p);
	}
}

void colliTrain::read_problem(const char* filename)
{
	int max_index, inst_max_index, i;
	size_t elements, j;
	FILE* fp = fopen(filename, "r");
	char* endptr;
	char* idx, * val, * label;

	if (fp == NULL)
	{
		fprintf(stderr, "can't open input file %s\n", filename);
		exit(1);
	}

	prob.l = 0;
	elements = 0;

	max_line_len = 1024;
	line = Malloc(char, max_line_len);
	while (readline(fp) != NULL)
	{
		char* p = strtok(line, " \t"); // label

		// features
		while (1)
		{
			p = strtok(NULL, " \t");
			if (p == NULL || *p == '\n') // check '\n' as ' ' may be after the last feature
				break;
			++elements;
		}
		++elements;
		++prob.l;
	}
	rewind(fp);

	prob.y = Malloc(double, prob.l);
	prob.x = Malloc(struct svm_node*, prob.l);
	x_space = Malloc(struct svm_node, elements);

	max_index = 0;
	j = 0;
	for (i = 0; i < prob.l; i++)
	{
		inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0
		readline(fp);
		prob.x[i] = &x_space[j];
		label = strtok(line, " \t\n");
		if (label == NULL) { // empty line
			fprintf(stderr, "Wrong input format at line %d\n", i + 1);
			exit(1);
		}

		prob.y[i] = strtod(label, &endptr);
		if (endptr == label || *endptr != '\0') {
			fprintf(stderr, "Wrong input format at line %d\n", i + 1);
			exit(1);
		}

		while (1)
		{
			idx = strtok(NULL, ":");
			val = strtok(NULL, " \t");

			if (val == NULL)
				break;

			errno = 0;
			x_space[j].index = (int)strtol(idx, &endptr, 10);
			if (endptr == idx || errno != 0 || *endptr != '\0' || x_space[j].index <= inst_max_index) {
				fprintf(stderr, "Wrong input format at line %d\n", i + 1);
				exit(1);
			}
			else
				inst_max_index = x_space[j].index;

			errno = 0;
			x_space[j].value = strtod(val, &endptr);
			if (endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr))) {
				fprintf(stderr, "Wrong input format at line %d\n", i + 1);
				exit(1);
			}

			++j;
		}

		if (inst_max_index > max_index)
			max_index = inst_max_index;
		x_space[j++].index = -1;
	}

	if (param.gamma == 0 && max_index > 0)
		param.gamma = 1.0 / max_index;

	if (param.kernel_type == PRECOMPUTED)
		for (i = 0; i < prob.l; i++)
		{
			if (prob.x[i][0].index != 0)
			{
				fprintf(stderr, "Wrong input format: first column must be 0:sample_serial_number\n");
				exit(1);
			}
			if ((int)prob.x[i][0].value <= 0 || (int)prob.x[i][0].value > max_index)
			{
				fprintf(stderr, "Wrong input format: sample_serial_number out of range\n");
				exit(1);
			}
		}

	fclose(fp);
}

void colliTrain::do_cross_validation()
{
	int i;
	int total_correct = 0;
	double total_error = 0;
	double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;
	double* target = Malloc(double, prob.l);

	svm_cross_validation(&prob, &param, nr_fold, target);
	if (param.svm_type == EPSILON_SVR ||
		param.svm_type == NU_SVR)
	{
		for (i = 0; i < prob.l; i++)
		{
			double y = prob.y[i];
			double v = target[i];
			total_error += (v - y) * (v - y);
			sumv += v;
			sumy += y;
			sumvv += v * v;
			sumyy += y * y;
			sumvy += v * y;
		}
		printf("Cross Validation Mean squared error = %g\n", total_error / prob.l);
		printf("Cross Validation Squared correlation coefficient = %g\n",
			((prob.l * sumvy - sumv * sumy) * (prob.l * sumvy - sumv * sumy)) /
			((prob.l * sumvv - sumv * sumv) * (prob.l * sumyy - sumy * sumy))
		);
	}
	else
	{
		for (i = 0; i < prob.l; i++)
			if (target[i] == prob.y[i])
				++total_correct;
		printf("Cross Validation Accuracy = %g%%\n", 100.0 * total_correct / prob.l);
	}
	free(target);
}

int colliTrain::svm_save_model_grad(const char* model_file_name, const svm_model* model, int D)
{
	FILE* fp = fopen(model_file_name, "w");
	if (fp == NULL) return -1;

	//char* old_locale = setlocale(LC_ALL, NULL);
	//if (old_locale) {
	//	old_locale = strdup(old_locale);
	//}
	//setlocale(LC_ALL, "C");

	const svm_parameter& param = model->param;

	//std::cout << "D: " << D << std::endl;
	fprintf(fp, "%d\n", D);

	int l = model->l;
	fprintf(fp, "%d\n", l);

	int nr_class = model->nr_class;
	fprintf(fp, "%.17g\n", -1 * (model->rho[0]));

	double sigma = sqrt(0.5 / param.gamma);
	fprintf(fp, "%.17g\n\n", sigma);


	const double* const* sv_coef = model->sv_coef;
	const svm_node* const* SV = model->SV;

	for (int i = 0; i < l; i++) {
		fprintf(fp, "%.17g ", sv_coef[0][i]);
	}
	fprintf(fp, "\n\n");

	for (int j = 0; j < D; j++) {
		for (int i = 0; i < l; i++) {
			const svm_node* p = SV[i];
			while ((p->index < j + 1) && (p->index != -1))
				p++;
			if (p->index == j + 1)
				fprintf(fp, "%.8g ", p->value);
			else
				fprintf(fp, "0 ");
		}
		fprintf(fp, "\n");
	}

	//setlocale(LC_ALL, old_locale);
	//free(old_locale);

	if (ferror(fp) != 0 || fclose(fp) != 0) return -1;
	else return 0;
}

char* colliTrain::readline(FILE* input)
{
	int len;

	if (fgets(line, max_line_len, input) == NULL)
		return NULL;

	while (strrchr(line, '\n') == NULL)
	{
		max_line_len *= 2;
		line = (char*)realloc(line, max_line_len);
		len = (int)strlen(line);
		if (fgets(line + len, max_line_len - len, input) == NULL)
			break;
	}
	return line;
}


void colliTrain::svmpredict(int n, char** command)
{
//Usage: svm - predict[options] test_file model_file output_file
//options :
//-b probability_estimates : whether to predict probability estimates, 0 or 1 (default 0).


	FILE* input, * output;
	int i;
	// parse options
	for (i = 0; i < n; i++)
	{
		if (command[i][0] != '-') break;
		++i;
		switch (command[i - 1][1])
		{
		case 'b':
			predict_probability = atoi(command[i]);
			break;
			//case 'q':
			//	info = &print_null;
			//	i--;
			//	break;
		default:
			fprintf(stderr, "Unknown option: -%c\n", command[i - 1][1]);
			exit(1);
		}
	}

	if (i >= n - 2) {
		std::cerr << "svm train input wrong" << std::endl;
		exit(1);
	}

	input = fopen(command[i], "r");
	if (input == NULL)
	{
		fprintf(stderr, "can't open input file %s\n", command[i]);
		exit(1);
	}

	output = fopen(command[i + 2], "w");
	if (output == NULL)
	{
		fprintf(stderr, "can't open output file %s\n", command[i + 2]);
		exit(1);
	}

	if ((model = svm_load_model(command[i + 1])) == 0)
	{
		fprintf(stderr, "can't open model file %s\n", command[i + 1]);
		exit(1);
	}

	x = (struct svm_node*)malloc(max_nr_attr * sizeof(struct svm_node));
	if (predict_probability)
	{
		if (svm_check_probability_model(model) == 0)
		{
			fprintf(stderr, "Model does not support probabiliy estimates\n");
			exit(1);
		}
	}
	else
	{
		if (svm_check_probability_model(model) != 0)
			info("Model supports probability estimates, but disabled in prediction.\n");
	}

	predict(input, output);
	svm_free_and_destroy_model(&model);
	free(x);
	free(line);
	fclose(input);
	fclose(output);
	predict_probability = 0;
	max_nr_attr = 64;
}


void colliTrain::predict(FILE* input, FILE* output)
{
	int correct = 0;
	int total = 0;
	double error = 0;
	double sump = 0, sumt = 0, sumpp = 0, sumtt = 0, sumpt = 0;

	int svm_type = svm_get_svm_type(model);
	int nr_class = svm_get_nr_class(model);
	double* prob_estimates = NULL;
	int j;

	if (predict_probability)
	{
		if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
			info("Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n", svm_get_svr_probability(model));
		else if (svm_type == ONE_CLASS)
		{
			// nr_class = 2 for ONE_CLASS
			prob_estimates = (double*)malloc(nr_class * sizeof(double));
			fprintf(output, "label normal outlier\n");
		}
		else
		{
			int* labels = (int*)malloc(nr_class * sizeof(int));
			svm_get_labels(model, labels);
			prob_estimates = (double*)malloc(nr_class * sizeof(double));
			fprintf(output, "labels");
			for (j = 0; j < nr_class; j++)
				fprintf(output, " %d", labels[j]);
			fprintf(output, "\n");
			free(labels);
		}
	}

	max_line_len = 1024;
	line = (char*)malloc(max_line_len * sizeof(char));
	while (readline(input) != NULL)
	{
		int i = 0;
		double target_label, predict_label;
		char* idx, * val, * label, * endptr;
		int inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0

		label = strtok(line, " \t\n");
		if (label == NULL) {
			fprintf(stderr, "Wrong input format at line %d\n", total + 1);
			exit(1);
		}// empty line

		target_label = strtod(label, &endptr);
		if (endptr == label || *endptr != '\0') {
			fprintf(stderr, "Wrong input format at line %d\n", total + 1);
			exit(1);
		}

		while (1)
		{
			if (i >= max_nr_attr - 1)	// need one more for index = -1
			{
				max_nr_attr *= 2;
				x = (struct svm_node*)realloc(x, max_nr_attr * sizeof(struct svm_node));
			}

			idx = strtok(NULL, ":");
			val = strtok(NULL, " \t");

			if (val == NULL)
				break;
			errno = 0;
			x[i].index = (int)strtol(idx, &endptr, 10);
			if (endptr == idx || errno != 0 || *endptr != '\0' || x[i].index <= inst_max_index) {
				fprintf(stderr, "Wrong input format at line %d\n", total + 1);
				exit(1);
			}
			else
				inst_max_index = x[i].index;

			errno = 0;
			x[i].value = strtod(val, &endptr);
			if (endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr))) {
				fprintf(stderr, "Wrong input format at line %d\n", total + 1);
				exit(1);
			}

			++i;
		}
		x[i].index = -1;

		if (predict_probability && (svm_type == C_SVC || svm_type == NU_SVC || svm_type == ONE_CLASS))
		{
			predict_label = svm_predict_probability(model, x, prob_estimates);
			fprintf(output, "%g", predict_label);
			for (j = 0; j < nr_class; j++)
				fprintf(output, " %g", prob_estimates[j]);
			fprintf(output, "\n");
		}
		else
		{
			predict_label = svm_predict(model, x);
			fprintf(output, "%.17g\n", predict_label);
		}

		if (predict_label == target_label)
			++correct;
		error += (predict_label - target_label) * (predict_label - target_label);
		sump += predict_label;
		sumt += target_label;
		sumpp += predict_label * predict_label;
		sumtt += target_label * target_label;
		sumpt += predict_label * target_label;
		++total;
	}
	if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
	{
		info("Mean squared error = %g (regression)\n", error / total);
		info("Squared correlation coefficient = %g (regression)\n",
			((total * sumpt - sump * sumt) * (total * sumpt - sump * sumt)) /
			((total * sumpp - sump * sump) * (total * sumtt - sumt * sumt))
		);
	}
	else
		info("Accuracy = %g%% (%d/%d) (classification)\n",
			(double)correct / total * 100, correct, total);
	if (predict_probability)
		free(prob_estimates);
}

colliDataGotten colliTrain::samFirstLayer(QMeshPatch* layer, toolpath* path, robSystem* robs, PQP_Model* layer_PQP, int typePosJoints, int indexLayer) {

	std::vector<Eigen::VectorXd> samOnLayer;
	for (int i = 0; i < path->length; i++) {
		Eigen::VectorXd tmp(6);
		tmp.segment(0, 3) = path->pathPoint[i];
		tmp.segment(3, 3) = -path->pathNormal[i];
		samOnLayer.push_back(tmp);
	}
	std::cout << "The number of point in sample list: " << samOnLayer.size() << std::endl;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dis(0.0, 1.0);

	for (int i = 0; i < path->length; i++) {
		Eigen::Vector3d p=samOnLayer[i].block<3, 1>(0, 0);
		Eigen::Vector3d n=samOnLayer[i].block<3, 1>(3, 0);
		for (int j = 0; j < 1; j++) {
			Eigen::Vector3d ptmp;
			Eigen::Vector3d delta = Eigen::Vector3d::Random();
			ptmp = p + delta * 0.8;
			Eigen::VectorXd pp(6);
			pp.block<3, 1>(0, 0) = ptmp;
			pp.block<3, 1>(3, 0) = n;
			samOnLayer.push_back(pp);
		}
		//std::cout << i << ": " << samOnLayer.size() << std::endl;
	}

	std::cout<< "After randomly sampling: " <<samOnLayer.size()<<std::endl;


	//对采样点求逆解
	std::vector<Eigen::VectorXd> qSam;

	std::vector<std::vector<Eigen::VectorXd>> subQSamAll(samOnLayer.size());//存储每个采样点的逆解

#pragma omp parallel for
	for (int i = 0; i < samOnLayer.size(); i++) {

		std::vector<Eigen::VectorXd> subQSam;

		Eigen::Matrix<double, 6, 1> pp = samOnLayer[i];
		Transform3d T = Transform3d::Identity();
		Eigen::Vector3d p, n, tmp1, tmp2;
		p << pp(0), pp(1), pp(2);//p是点的坐标
		n << pp(3), pp(4), pp(5);//n是点的法向量
		//n = -n;
		n.normalize();
		//std::cout<<"\np:"<<p<<std::endl;
		//std::cout<<"n:"<<n<<std::endl;
		//要确认法向量方向！！！法向量向下！

		//求BC角度
		std::vector<Eigen::Vector2d> BCs;
		if (-n(2) > 1 - 1e-5) {
			Eigen::Vector2d BC;			
			BC(0) = 0;
			BC(1) = dis(gen) * 2 * M_PI - M_PI;
			BCs.push_back(BC);
			BCs.push_back(BC);
		}
		else {
			BCs = robs->ikPosA(-n, Eigen::Vector3d(0, 0, 1));
		}

		int numbers;
		if (i < path->length) {
			numbers = 15;
		}
		else {
			numbers = 1;
		}

		for (int itmp = 0; itmp < numbers; itmp++) {
			Eigen::Vector2d BC;
			if (dis(gen) > 0.5) {
				BC = BCs[0];
			}
			else {
				BC = BCs[1];
			}

			Eigen::Vector2d delta = Eigen::Vector2d::Random();//delta范围在-1到1之间
			BC = BC + delta * M_PI / 6;

			Transform3d TPOS = robs->fkPos(BC);

			//转换到工件坐标系下
			Eigen::Vector3d nNew, pNew;
			nNew = TPOS.linear() * n;//n是工件坐标系下的法向量
			pNew = TPOS.linear() * p + TPOS.translation();//p是工件坐标系下的点的坐标

			//旋转矩阵
			Eigen::Vector3d tmp(1, 0, 0);
			tmp1 = nNew.cross(tmp);
			tmp1.normalize();
			tmp2 = nNew.cross(tmp1);
			tmp2.normalize();
			Eigen::Matrix3d R;
			R << tmp1, tmp2, nNew;

			//末端位姿
			T.linear() = R;
			T.translation() = pNew;

			Eigen::VectorXd qtmp(8);
			qtmp.block<2, 1>(6, 0) = BC;


			//随机旋转
			double angle;
			angle = dis(gen) * 2 * M_PI;
			Transform3d RotzTmp(Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 0, 1)));

			Eigen::Vector3d anxisTmp;
			angle = dis(gen) * 2 * M_PI;
			anxisTmp << cos(angle), sin(angle), 0;
			angle = dis(gen) * M_PI / 4;
			Transform3d RotTmp(Eigen::AngleAxisd(angle, anxisTmp));

			RotTmp.matrix() = RotTmp.matrix() * RotzTmp.matrix();

			Transform3d T1;
			T1.matrix() = T.matrix() * RotTmp.matrix() * robs->Ttool.matrix().inverse();

			bool ik = false;
			qtmp.block<6, 1>(0, 0) = robs->ikABB2600(ik, T1);
			if (ik) {
				subQSam.push_back(qtmp);
			}
		}

		subQSamAll[i] = subQSam;
	}

	//将subQSamAll中的元素添加到qSam中
	for (int i = 0; i < subQSamAll.size(); i++) {
		for (int j = 0; j < subQSamAll[i].size(); j++) {
			qSam.push_back(subQSamAll[i][j]);//添加新的点
		}
	}
	std::cout << "1. samJoint.size=:" << qSam.size() << std::endl;


	int size1 = qSam.size();
	//检测碰撞
	std::vector<int> samColi;
	samColi.resize(qSam.size());
	std::cout << "Checking collision..." << std::endl;

	for (int i = 0; i < qSam.size(); i++) {
		bool tmp = robs->checkLayerColi(qSam[i], indexLayer);
		if (tmp == true) {
			//发生碰撞
			samColi[i] = 1;
		}
		else {
			samColi[i] = -1;
		}

		if (i % 500 == 0) {
			std::cout << "." << std::flush;
		}
		//std::cout << "." << std::flush;
	}
	std::cout << std::endl;


	//求samColi中等于1的个数
	int count = 0;
	for (int i = 0; i < samColi.size(); i++) {
		if (samColi[i] == 1) {
			count++;
		}
	}
	if (count == 0) {
		std::cout << "No collision point!" << std::endl;
	}
	else {
		std::cout << "The number of collision point:" << count << std::endl;
	}

	if (count < (qSam.size() / 1000)) {
		std::cout << "not need collision check!" << std::endl;
		colliDataGotten out;
		out.coli.push_back(0);
		return out;
	}
	

	//Upscaling
	//提高采样密度
	int k = 2;//k_NN
	double tau_a = 0.5;
	//tau_a = 2;

	std::vector<std::vector <Eigen::VectorXd>> NewqSam;
	std::vector<bool> needCheck(qSam.size(), true);//标记是否需要检查,初始化为true
	while (true) {
		NewqSam.clear();
		NewqSam.resize(qSam.size());

#pragma omp parallel for
		for (int i = 0; i < qSam.size(); i++) {
			std::vector<Eigen::VectorXd> addQSam;//存储新的点

			if (needCheck[i] == true) {//需要检查

				Eigen::Matrix<double, 8, 1> a = qSam[i];
				std::vector<std::pair<double, int>> distance(qSam.size());
				//计算距离
				for (int j = 0; j < qSam.size(); j++) {
					if (j == i) {
						distance[j] = { 0,j };
						continue;
					}
					distance[j] = { (qSam[j] - a).norm(),j };
				}
				//排序
				std::sort(distance.begin(), distance.end());

				for (int j = k; j >= 0; j--) {
					if (j == k && distance[j].first <= tau_a) {
						needCheck[i] = false;
						break;//满足条件，跳出循环
					}

					if (distance[j].first > tau_a) {
						//对每个不满足要求的点生成中间点
						//Eigen::VectorXd qNew = GenerateNewSample2(a, qSam[distance[j].second], layer_PQP, robs);
						Eigen::VectorXd qNew = (a + qSam[distance[j].second]) / 2;
						addQSam.push_back(qNew);
					}
					else {
						break;
					}
				}
			}

			NewqSam[i] = addQSam;
			//std::cout << "." << std::flush;
		}
		//std::cout << std::endl;

		int nQsam = qSam.size();
		//将NewqSam中的元素添加到qSam中
		for (int i = 0; i < NewqSam.size(); i++) {
			for (int j = 0; j < NewqSam[i].size(); j++) {
				qSam.push_back(NewqSam[i][j]);//添加新的点
				needCheck.push_back(true);//添加的点需要检查

				bool tmp = robs->checkLayerColi(qSam.back(), indexLayer);
				if (tmp) {
					samColi.push_back(1);
				}
				else {
					samColi.push_back(-1);
				}

			}

			if (i % 500 == 0) {
				std::cout << "." << std::flush;
			}
		}
		std::cout << std::endl;

		std::cout << "qSam.size=:" << qSam.size() << ";  samColi.size=:" << samColi.size() << std::endl;

		if (qSam.size() == nQsam) {
			break;//没有新的点被添加，跳出循环
		}
	}
	std::cout << "2. samJoint.size=:" << qSam.size() << std::endl;


	//refinement
	k = 1;
	double tau_b = 0.1;
	//tau_b = 0.1;
	//tau_b = 1;

	needCheck.clear();
	needCheck.resize(qSam.size(), true);//标记是否需要检查,初始化为true
	while (true) {
		NewqSam.clear();
		NewqSam.resize(qSam.size());

#pragma omp parallel for
		for (int i = 0; i < qSam.size(); i++) {
			std::vector<Eigen::VectorXd> addQSam;//存储新的点

			if (needCheck[i] == true) {
				Eigen::Matrix<double, 8, 1> a = qSam[i];

				std::vector<std::pair<double, int>> distance(qSam.size());
				//计算距离
				for (int j = 0; j < qSam.size(); j++) {
					if (j == i) {
						distance[j] = { 0,j };
						continue;
					}
					distance[j] = { (qSam[j] - a).norm(),j };
				}

				//排序
				std::sort(distance.begin(), distance.end());

				needCheck[i] = false;
				for (int j = k; j >= 0; j--) {
					if (distance[j].first > tau_b && samColi[i] != samColi[distance[j].second]) {
						//对每个不满足要求的点生成中间点
						//Eigen::VectorXd qNew = GenerateNewSample2(a, qSam[distance[j].second], layer_PQP, robs);
						Eigen::VectorXd qNew = (a + qSam[distance[j].second]) / 2;
						addQSam.push_back(qNew);

						//需要再次检查当前点
						if (!needCheck[i]) {
							needCheck[i] = true;
						}
					}
					else if (distance[j].first <= tau_b) {
						break;
					}
				}
			}
			NewqSam[i] = addQSam;
			//std::cout << "." << std::flush;
		}
		//std::cout << std::endl;

		int nQsam = qSam.size();
		//将NewqSam中的元素添加到qSam中
		for (int i = 0; i < NewqSam.size(); i++) {
			for (int j = 0; j < NewqSam[i].size(); j++) {
				qSam.push_back(NewqSam[i][j]);//添加新的点
				needCheck.push_back(true);//添加的点需要检查

				//检查碰撞
				bool tmp = robs->checkLayerColi(qSam.back(), indexLayer);
				if (tmp == true) {
					//发生碰撞
					samColi.push_back(1);
				}
				else {
					samColi.push_back(-1);
				}

			}

			if (i % 500 == 0) {
				std::cout << "." << std::flush;
			}
		}
		std::cout << std::endl;

		std::cout << "qSam.size=:" << qSam.size() << ";  samColi.size=:" << samColi.size() << std::endl;

		if (qSam.size() == nQsam) {
			break;//没有新的点被添加，跳出循环
		}

	}
	std::cout << "3. qSam.size=:" << qSam.size() << std::endl;


	//转换为关节位置
	std::vector<Eigen::VectorXd> posJoints;
	for (int i = 0; i < qSam.size(); i++) {
		//Eigen::VectorXd pos = robs->Joints2Postions(qSam[i], false);
		Eigen::VectorXd pos = robs->Joints2PostionsBC(qSam[i], typePosJoints);
		posJoints.push_back(pos);
	}

	colliDataGotten outPut;
	outPut.coli = samColi;
	outPut.posJoints = posJoints;
	outPut.Joints = qSam;

	return outPut;

}


void colliTrain::GenerateNewSample(Eigen::VectorXd a, Eigen::VectorXd b,
	std::vector<Eigen::VectorXd>& qSam, PQP_Model* layer_PQP, robSystem* robs) {

	Eigen::VectorXd qtmp = (a + b) / 2;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dis(0.0, 1.0);

	if (dis(gen) > 0.5) {
		qSam.push_back(qtmp);
	}
	else {
		Eigen::Vector3d ptmp;
		distToolLayer(qtmp, layer_PQP, ptmp, robs);


		Transform3d TRobtmp = robs->fkABB2600(qtmp.block<6, 1>(0, 0));
		TRobtmp.matrix() = TRobtmp.matrix() * robs->Ttool.matrix();
		TRobtmp.translation() = ptmp;

		bool ik = false;
		TRobtmp.matrix() = TRobtmp.matrix() * robs->Ttool.matrix().inverse();
		qtmp.block<6, 1>(0, 0) = robs->ikABB2600(ik, TRobtmp);
		if (ik) {
			qSam.push_back(qtmp);
		}
		else {
			qSam.push_back((a + b) / 2);
		}
	}
}

Eigen::VectorXd colliTrain::GenerateNewSample2(Eigen::VectorXd a, Eigen::VectorXd b,
	PQP_Model* layer_PQP, robSystem* robs) {

	Eigen::VectorXd qtmp = (a + b) / 2;

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> dis(0.0, 1.0);

	if (dis(gen) > 0.5) {
		return qtmp;
	}
	else {
		Eigen::Vector3d ptmp;
		distToolLayer(qtmp, layer_PQP, ptmp, robs);


		Transform3d TRobtmp = robs->fkABB2600(qtmp.block<6, 1>(0, 0));
		TRobtmp.matrix() = TRobtmp.matrix() * robs->Ttool.matrix();
		TRobtmp.translation() = ptmp;

		bool ik = false;
		TRobtmp.matrix() = TRobtmp.matrix() * robs->Ttool.matrix().inverse();
		qtmp.block<6, 1>(0, 0) = robs->ikABB2600(ik, TRobtmp);
		if (ik) {
			return qtmp;
		}
		else {
			return (a + b) / 2;
		}
	}
}



double colliTrain::distToolLayer(Eigen::Matrix<double, 8, 1> q, PQP_Model* layer_PQP, Eigen::Vector3d& q2, robSystem* robs) {
	Transform3d TPOS = robs->fkPos(q.block<2, 1>(6, 0));
	Transform3d TROB = robs->fkABB2600(q.block<6, 1>(0, 0));
	Transform3d TPT;//POS to Tool
	TPT.matrix() = TPOS.matrix().inverse() * TROB.matrix() * robs->Ttool.matrix();

	PQP_REAL p[3];
	p[0] = TPT.matrix()(0, 3);
	p[1] = TPT.matrix()(1, 3);
	p[2] = TPT.matrix()(2, 3);

	PQP_DistanceResult dres;
	dres.last_tri = layer_PQP->last_tri;
	PQP_Distance(&dres, layer_PQP, p, 0.0, 0.0);


	Eigen::Vector4d qtmp;
	for (int i = 0; i < 3; i++) {
		qtmp(i) = dres.P1()[i];
	}
	qtmp(3) = 1;
	qtmp = TPOS.matrix() * qtmp;
	q2 = qtmp.block<3, 1>(0, 0);


	return dres.Distance();
}

colliDataGotten colliTrain::FastronModelUpdate(colliDataGotten data) {
	std::cout << "start update" << std::endl;

	int n = data.coli.size();//数据点的数量

	Eigen::VectorXd y(n);
	for (int i = 0; i < n; i++) {
		y(i) = data.coli[i];
	}

	//std::cout<<n<<std::endl;

	if (newFastron) {
		std::cout<<"new fastron"<<std::endl;

		alpha = Eigen::VectorXd::Zero(n);
		F = Eigen::VectorXd::Zero(n);
		newFastron = false;
	}
	else {
		std::cout<<"update fastron"<<std::endl;

		//修改alpha、F
		Eigen::VectorXd alphaTmp = Eigen::VectorXd::Zero(n), Ftmp = Eigen::VectorXd::Zero(n);
		alphaTmp.segment(0, alpha.size()) = alpha;
		Ftmp.segment(0, F.size()) = F;

		for (int i = alphaTmp.size(); i < n; i++) {
			Eigen::VectorXd diff;
			Ftmp(i) = this->getFastronScore(data.posJoints[i], this->supportData, this->alpha);
		}

		alpha = alphaTmp;
		F = Ftmp;

	}

	Eigen::VectorXd alphaBefore = alpha, FBefore = F;

	int iterMax = 100000;
	double beta = 5;
	beta = 1;

	for (int iter = 1; iter <= iterMax; iter++) {
		if (iter % 1000 == 0) {
			std::cout << "iter: " << iter << ". " << std::flush;
		}

		Eigen::VectorXd tmp = y.cwiseProduct(F);
		Eigen::VectorXd::Index minIdx;
		double minVal = tmp.minCoeff(&minIdx);//最小值
		int i = minIdx;
		//cout << "i: " << i << " , minVal: " << minVal;
		if (minVal <= 0) {
			Eigen::VectorXd Kc = computeGramMatrixColumn(i, data.posJoints);
			double delta = pow(beta, 0.5 * (y(i) + 1)) * y(i) - F(i);
			alpha(i) = alpha(i) + delta;
			F = F + delta * Kc;
			//std::cout<<"F: "<<F(i)<<" "<<this->getFastronScore(data.posJoints[i], data, alpha) << std::endl;
			//std::cout<<"alpha_"<<i<<":"<<alpha(i)<<std::endl;
			//std::cout<<"  now: " << y(i) * F(i) << std::endl;
			continue;
		}

		//std::cout<<"all large than zero"<<std::endl;

		alphaBefore = alpha;
		FBefore = F;

		Eigen::VectorXd nonZero = Eigen::VectorXd::Zero(n);
		for (int j = 0; j < n; j++) {
			if (alpha(j) != 0) {
				nonZero(j) = 1;
			}
		}

		tmp = y.cwiseProduct(F - alpha);
		tmp = tmp.cwiseProduct(nonZero);
		Eigen::VectorXd::Index maxIdx;
		double maxVal = tmp.maxCoeff(&maxIdx);//最大值
		i = maxIdx;
		Eigen::VectorXd Kc = computeGramMatrixColumn(i, data.posJoints);
		//cout << "i: " << i << " , maxVal: " << maxVal << endl;
		if (maxVal > 0) {
			//F = F - alpha(i) * Kspar.col(i);
			F = F - alpha(i) * Kc;
			alpha(i) = 0;
			continue;
		}

		break;
	}

	std::cout << std::endl;

	//std::cout<<"2"<<std::endl;

	Eigen::VectorXd tmp1 = y.cwiseProduct(F), tmp2 = y.cwiseProduct(FBefore);
	//求tmp1和tmp2中小于0的元素的个数
	int count1 = 0, count2 = 0;
	for (int i = 0; i < n; i++) {
		if (tmp1(i) <= 0) {
			count1++;
		}
		if (tmp2(i) <= 0) {
			count2++;
		}
	}

	//std::cout<<"count1:"<<count1<<" count2:"<<count2<<std::endl;

	if (count1 > count2) {
		alpha = alphaBefore;
		F = FBefore;
	}

	Eigen::VectorXi rowsToKeep;

	//std::cout<<"rosToKeepSize: "<<rowsToKeep.size() << std::endl;

	colliDataGotten supportData;
	for (int i = 0; i < alpha.size(); i++) {
		if (alpha(i) != 0) {
			rowsToKeep.conservativeResize(rowsToKeep.size() + 1);
			rowsToKeep(rowsToKeep.size() - 1) = i;

			supportData.coli.push_back(data.coli[i]);
			supportData.posJoints.push_back(data.posJoints[i]);
			supportData.Joints.push_back(data.Joints[i]);
			//std::cout<<"keep:"<<i<<std::endl;
		}
	}

	//std::cout<<"rowsToKeepSize: "<<rowsToKeep.transpose() << std::endl;

	std::cout << "number of support data:" << supportData.coli.size() << std::endl;

	//Eigen::MatrixXd K2 = K(rowsToKeep, rowsToKeep);
	//K = K2;
	alpha = alpha(rowsToKeep);
	F = F(rowsToKeep);

	this->supportData = supportData;

	return supportData;
}


Eigen::VectorXd colliTrain::computeGramMatrixColumn(int i, std::vector<Eigen::VectorXd> posJoints) {
	int n= posJoints.size();
	Eigen::VectorXd Kc(n);
	for (int j = 0; j < n; j++) {
		Kc(j) = kernelRQ(posJoints[i], posJoints[j], 0.7, 2);
		//K.insert(j, i) = kernelRQ(posJoints[i], posJoints[j], 0.7, 2);
		//K(j, i) = kernelRQ(posJoints[i], posJoints[j], 0.7, 2);
	}
	return Kc;
}

double colliTrain::kernelRQ(Eigen::VectorXd a, Eigen::VectorXd b, double gamma, double p) {
	double tmp = (a - b).norm();
	double kernel = 1 + tmp * tmp * gamma / p;
	kernel = pow(kernel, -p);
	return kernel;
}

void colliTrain::removeRowAndCol(Eigen::MatrixXd& matrix, unsigned int rowToRemove, unsigned int colToRemove) {
	unsigned int numRows = matrix.rows() - 1;
	unsigned int numCols = matrix.cols() - 1;

	if (rowToRemove < numRows)
		matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols + 1) = matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols + 1);

	if (colToRemove < numCols)
		matrix.block(0, colToRemove, numRows + 1, numCols - colToRemove) = matrix.block(0, colToRemove + 1, numRows + 1, numCols - colToRemove);
	
	matrix.conservativeResize(numRows, numCols);
}

void colliTrain::removeElement(Eigen::VectorXd& vector, unsigned int indexToRemove) {
	unsigned int numElements = vector.size() - 1;

	if (indexToRemove < numElements)
		vector.segment(indexToRemove, numElements - indexToRemove) = vector.segment(indexToRemove + 1, numElements - indexToRemove);

	vector.conservativeResize(numElements);
}

double colliTrain::kernelPH(Eigen::VectorXd a, Eigen::VectorXd b, int k) {
	double r = (a - b).norm();
	if (k % 2 == 0) {
		return pow(r, k) * log(r);
	}
	else {
		return pow(r, k);
	}
}

Eigen::VectorXd colliTrain::getProxyColli(colliDataGotten supportData) {
	int n = supportData.coli.size();
	Eigen::VectorXd A(n);
	Eigen::MatrixXd Kph(n, n);
	Eigen::VectorXd Y(n);
	for (int i = 0; i < n; i++) {
		for (int j = i; j < n; j++) {
			Kph(i, j) = kernelPH(supportData.posJoints[i], supportData.posJoints[j], 1);
			Kph(j, i) = Kph(i, j);
		}
		Y(i) = supportData.coli[i];

		//std::cout<<i<<std::endl;
	}

	std::cout << "constructed" << std::endl;


	A = Kph.householderQr().solve(Y);

	//std::cout << A.maxCoeff() << std::endl;

	std::cout << "error:" << (Kph * A - Y).cwiseAbs().maxCoeff() << std::endl;


	//if (Kph.determinant() == 0) {
	//	std::cout << "singularity" << std::endl;
	//	//求广义逆
	//	Eigen::MatrixXd KphInv = Kph.completeOrthogonalDecomposition().pseudoInverse();
	//	A = KphInv * Y;
	//	std::cout << "solution:" << (Kph * A - Y).cwiseAbs().maxCoeff() << std::endl;
	//}
	//else {
	//	std::cout << "solve" << std::endl;
	//	A = Kph.inverse() * Y;
	//}

	return A;
}

double colliTrain::getProxyScore(Eigen::VectorXd q, colliDataGotten supportData, Eigen::VectorXd A, Eigen::VectorXd& diff, robSystem* robs) {
	//q:关节角度
	Eigen::MatrixXd posDjoints;

	//Eigen::VectorXd posJoints = robs->Joints2Postions(q, false, posDjoints);
	Eigen::VectorXd posJoints = robs->Joints2PostionsBC(q, 5, posDjoints);



	int n = supportData.coli.size();
	Eigen::VectorXd Kfkph(n);

	Eigen::MatrixXd KfkphDposjoint(22, n);

	for (int i = 0; i < n; i++) {
		Kfkph(i) = kernelPH(posJoints, supportData.posJoints[i], 1);

		//k=1的情况
		Eigen::VectorXd tmp = (posJoints - supportData.posJoints[i]).normalized();
		KfkphDposjoint.block<22, 1>(0, i) = tmp;
	}
	double score = Kfkph.dot(A);


	Eigen::VectorXd scoreDposjoint(22);
	scoreDposjoint = KfkphDposjoint * A;


	//diff = (scoreDposjoint.transpose() * posDjoints).transpose();
	diff = posDjoints.transpose() * scoreDposjoint;

	return score;
}

double colliTrain::getFastronScore(Eigen::VectorXd q, colliDataGotten supportData, Eigen::VectorXd Alpha, Eigen::VectorXd& diff, robSystem* robs) {
	//q:关节角度
	Eigen::MatrixXd posDjoints;
	//Eigen::VectorXd posJoints = robs->Joints2Postions(q, false, posDjoints);
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


double colliTrain::getFastronScore(Eigen::VectorXd posJoints, colliDataGotten supportData, Eigen::VectorXd Alpha) {
	
	int n = supportData.coli.size();
	Eigen::VectorXd Kfk(n);

	for (int i = 0; i < n; i++) {
		Kfk(i) = kernelRQ(posJoints, supportData.posJoints[i], 0.7, 2);
	}
	double score = Kfk.dot(Alpha);
	return score;
}


colliDataGotten colliTrain::FastronActiveLearning(QMeshPatch* layer, int indexLayer, toolpath* path, robSystem* robs, int typePosJoints) {
	
	//路径点即为采样点
	std::vector<Eigen::VectorXd> samOnLayer;
	for (int i = 0; i < path->length; i++) {
		Eigen::VectorXd tmp(6);
		tmp.segment(0, 3) = path->pathPoint[i];
		tmp.segment(3, 3) = -path->pathNormal[i];
		samOnLayer.push_back(tmp);
	}
	std::cout << "The number of point in sample list: " << samOnLayer.size() << std::endl;

	std::random_device rd;//非确定性随机数生成器
	std::mt19937 gen(rd());//使用mt19937引擎
	std::uniform_real_distribution<double> dis(0.0, 1.0);//生成0.0到1.0之间的实数

	//对每个采样点，周围随机生成新的采样点
	for (int i = 0; i < path->length; i++) {
		Eigen::Vector3d p = samOnLayer[i].block<3, 1>(0, 0);
		Eigen::Vector3d n = samOnLayer[i].block<3, 1>(3, 0);
		for (int j = 0; j < 1; j++) {
			Eigen::Vector3d ptmp;
			Eigen::Vector3d delta = Eigen::Vector3d::Random();//随机生成一个向量
			ptmp = p + delta * 0.8;//在p附近生成一个点
			Eigen::VectorXd pp(6);
			pp.block<3, 1>(0, 0) = ptmp;
			pp.block<3, 1>(3, 0) = n;
			samOnLayer.push_back(pp);
		}
		//std::cout << i << ": " << samOnLayer.size() << std::endl;
	}

	std::cout << "After randomly sampling: " << samOnLayer.size() << std::endl;


	//对采样点求逆解
	std::vector<Eigen::VectorXd> qSam;

	std::vector<std::vector<Eigen::VectorXd>> subQSamAll(samOnLayer.size());//存储每个采样点的逆解

#pragma omp parallel for
	for (int i = 0; i < samOnLayer.size(); i++) {

		std::vector<Eigen::VectorXd> subQSam;

		Eigen::Matrix<double, 6, 1> pp = samOnLayer[i];
		Transform3d T = Transform3d::Identity();
		Eigen::Vector3d p, n, tmp1, tmp2;
		p << pp(0), pp(1), pp(2);//p是点的坐标
		n << pp(3), pp(4), pp(5);//n是点的法向量
		//n = -n;
		n.normalize();
		//std::cout<<"\np:"<<p<<std::endl;
		//std::cout<<"n:"<<n<<std::endl;
		//要确认法向量方向！！！法向量向下！

		//求BC角度
		std::vector<Eigen::Vector2d> BCs;
		if (-n(2) > 1 - 1e-5) {
			Eigen::Vector2d BC;
			BC(0) = 0;
			BC(1) = dis(gen) * 2 * M_PI - M_PI;
			BCs.push_back(BC);
			BCs.push_back(BC);
		}
		else {
			BCs = robs->ikPosA(-n, Eigen::Vector3d(0, 0, 1));
		}

		int numbers;
		if (i < path->length) {
			numbers = 15;
		}
		else {
			numbers = 1;
		}

		for (int itmp = 0; itmp < numbers; itmp++) {
			Eigen::Vector2d BC;
			if (dis(gen) > 0.5) {
				BC = BCs[0];
			}
			else {
				BC = BCs[1];
			}

			Eigen::Vector2d delta = Eigen::Vector2d::Random();//delta范围在-1到1之间
			BC = BC + delta * M_PI / 6;

			Transform3d TPOS = robs->fkPos(BC);

			//转换到工件坐标系下
			Eigen::Vector3d nNew, pNew;
			nNew = TPOS.linear() * n;//n是工件坐标系下的法向量
			pNew = TPOS.linear() * p + TPOS.translation();//p是工件坐标系下的点的坐标

			//旋转矩阵
			Eigen::Vector3d tmp(1, 0, 0);
			tmp1 = nNew.cross(tmp);
			tmp1.normalize();
			tmp2 = nNew.cross(tmp1);
			tmp2.normalize();
			Eigen::Matrix3d R;
			R << tmp1, tmp2, nNew;

			//末端位姿
			T.linear() = R;
			T.translation() = pNew;

			Eigen::VectorXd qtmp(8);
			qtmp.block<2, 1>(6, 0) = BC;


			//随机旋转
			double angle;
			angle = dis(gen) * 2 * M_PI;
			Transform3d RotzTmp(Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 0, 1)));

			Eigen::Vector3d anxisTmp;
			angle = dis(gen) * 2 * M_PI;
			anxisTmp << cos(angle), sin(angle), 0;
			angle = dis(gen) * M_PI / 4;
			Transform3d RotTmp(Eigen::AngleAxisd(angle, anxisTmp));

			RotTmp.matrix() = RotTmp.matrix() * RotzTmp.matrix();

			Transform3d T1;
			T1.matrix() = T.matrix() * RotTmp.matrix() * robs->Ttool.matrix().inverse();

			bool ik = false;
			qtmp.block<6, 1>(0, 0) = robs->ikABB2600(ik, T1);
			if (ik) {
				subQSam.push_back(qtmp);
			}
		}

		subQSamAll[i] = subQSam;
	}

	//将subQSamAll中的元素添加到qSam中
	for (int i = 0; i < subQSamAll.size(); i++) {
		for (int j = 0; j < subQSamAll[i].size(); j++) {
			qSam.push_back(subQSamAll[i][j]);//添加新的点
		}
	}

	std::cout << "1. samJoint.size=:" << qSam.size() << std::endl;

    //在每个支持点附近生成新点
	subQSamAll.clear();
	subQSamAll.resize(supportData.coli.size());

#pragma omp parallel for
	for (int i = 0; i < supportData.coli.size(); i++) {

		std::vector<Eigen::VectorXd> subQSam;

		Transform3d T = robs->fkABB2600(supportData.Joints[i].block<6, 1>(0, 0));
		T.matrix() = T.matrix() * robs->Ttool.matrix();
		Eigen::Vector3d ptmp = T.translation();

		//std::cout<<"in support: "<<i<<std::endl;

		for (int j = 0; j < 1; j++) {

			Eigen::Vector3d rand = Eigen::Vector3d::Random(3, 1);//rand是随机向量，每个元素在-1到1之间
			Eigen::Vector3d ptmpnew;
			ptmpnew = ptmp + rand * 0.2;
			Transform3d Tnew = T;
			Tnew.translation() = ptmpnew;

			Eigen::VectorXd qtmp(8);
			qtmp.block<2, 1>(6, 0) = supportData.Joints[i].block<2, 1>(6, 0);

			bool ik = false;
			int itmp = 0;
			while (!ik && itmp < 5) {
				itmp += 1;

				double angle = dis(gen) * 2 * M_PI / 36 - M_PI / 36;
				Transform3d RotzTmp(Eigen::AngleAxisd(angle, Eigen::Vector3d(0, 0, 1)));

				Eigen::Vector3d anxisTmp;
				angle = dis(gen) * 2 * M_PI;
				anxisTmp << cos(angle), sin(angle), 0;
				angle = dis(gen) * M_PI / 36;
				Transform3d RotTmp(Eigen::AngleAxisd(angle, anxisTmp));

				RotTmp.matrix() = RotTmp.matrix() * RotzTmp.matrix();

				Tnew.matrix() = Tnew.matrix() * RotTmp.matrix() * robs->Ttool.matrix().inverse();

				qtmp.block<6, 1>(0, 0) = robs->ikABB2600(ik, Tnew, supportData.Joints[i].block<6, 1>(0, 0));
			}

			if (ik) {
				subQSam.push_back(qtmp);
			}
		}

		subQSamAll[i] = subQSam;
	}

	//将subQSamAll中的元素添加到qSam中
	for (int i = 0; i < subQSamAll.size(); i++) {
		for (int j = 0; j < subQSamAll[i].size(); j++) {
			qSam.push_back(subQSamAll[i][j]);//添加新的点
		}
	}
	std::cout << "2. samJoint.size=:" << qSam.size() << std::endl;


	colliDataGotten newData = supportData;

	for (int i = 0; i < qSam.size(); i++) {
		newData.Joints.push_back(qSam[i]);

		Eigen::VectorXd pos = robs->Joints2PostionsBC(qSam[i], typePosJoints);
		newData.posJoints.push_back(pos);

		newData.coli.push_back(-1);
	}

	std::cout << "check collision ..." << std::endl;
	for (int i = 0; i < newData.coli.size(); i++) {
		if (newData.coli[i] == -1) {
			bool tmp = robs->checkLayerColi(newData.Joints[i], indexLayer);
			if (tmp == true) {
				//发生碰撞
				newData.coli[i] = 1;
			}
		}

		if (i % 1000 == 0) {
			std::cout << "." << std::flush;
		}
	}
	std::cout << std::endl;


	std::cout<<"complete ActiveLearning"<<std::endl;

	return newData;
}