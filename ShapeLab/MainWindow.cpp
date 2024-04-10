#include "stdafx.h"

#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QFileDialog>
#include <QtDebug>
#include <QDesktopWidget>
#include <QCoreApplication>
#include <QMimeData>
#include <QTreeView>
#include <QThread>
#include <QTimer>
#include <QDateTime>
#include <QMessageBox>
#include <QScreen>
#include <QStyleFactory>
#include <fstream>

#include "../GLKLib/GLKCameraTool.h"
#include "../GLKLib/InteractiveTool.h"
#include "../GLKLib/GLKMatrixLib.h"
#include "../GLKLib/GLKGeometry.h"
#include "../QMeshLib/QMeshPatch.h"
#include "../QMeshLib/QMeshTetra.h"
#include "../QMeshLib/QMeshFace.h"
#include "../QMeshLib/QMeshEdge.h"
#include "../QMeshLib/QMeshNode.h"

#include "alphanum.hpp"
#include <dirent.h>

#include <random>

#include "trajOpt.h"
#include "pathStruct.h"
#include "fileIO.h"
#include "mathTools.h"
#include "robSystem.h"
#include "colliTrain.h"
#include "taseMethod.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	QApplication::setStyle(QStyleFactory::create("Fusion"));

    signalMapper = new QSignalMapper(this);
    addToolBar(ui->toolBar);
    addToolBar(ui->navigationToolBar);
    addToolBar(ui->selectionToolBar);

    createTreeView();
    createActions();

    pGLK = new GLKLib();
    ui->horizontalLayout->addWidget(pGLK);
    ui->horizontalLayout->setMargin(0);
    pGLK->setFocus();

    pGLK->clear_tools();
    pGLK->set_tool(new GLKCameraTool(pGLK,ORBITPAN));

    this->setDefault();
	
	//connect timer with timer function
	//connect(&Gcode_timer, SIGNAL(timeout()), this, SLOT(doTimerGcodeMoving()));
}

MainWindow::~MainWindow()
{
    delete robs;
    delete colli;
    delete Opt;

    for (int i = 0; i < pathsInLayer.size(); i++) {
        delete pathsInLayer[i];
    }
    pathsInLayer.clear();


    delete ui;

    
}

void MainWindow::createActions()
{
    // file IO
    connect(ui->actionOpen, SIGNAL(triggered(bool)), this, SLOT(open()));
    connect(ui->actionSave, SIGNAL(triggered(bool)), this, SLOT(save()));
	connect(ui->actionSaveSelection, SIGNAL(triggered(bool)), this, SLOT(saveSelection()));
	connect(ui->actionReadSelection, SIGNAL(triggered(bool)), this, SLOT(readSelection()));

    // navigation
    connect(ui->actionFront, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionBack, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionTop, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionBottom, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionLeft, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionRight, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionIsometric, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_In, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_Out, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_All, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionZoom_Window, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    signalMapper->setMapping (ui->actionFront, 0);
    signalMapper->setMapping (ui->actionBack, 1);
    signalMapper->setMapping (ui->actionTop, 2);
    signalMapper->setMapping (ui->actionBottom, 3);
    signalMapper->setMapping (ui->actionLeft, 4);
    signalMapper->setMapping (ui->actionRight, 5);
    signalMapper->setMapping (ui->actionIsometric, 6);
    signalMapper->setMapping (ui->actionZoom_In, 7);
    signalMapper->setMapping (ui->actionZoom_Out, 8);
    signalMapper->setMapping (ui->actionZoom_All, 9);
    signalMapper->setMapping (ui->actionZoom_Window, 10);

    // view
    connect(ui->actionShade, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionMesh, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionProfile, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionFaceNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionNodeNormal, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    signalMapper->setMapping (ui->actionShade, 20);
    signalMapper->setMapping (ui->actionMesh, 21);
    signalMapper->setMapping (ui->actionNode, 22);
    signalMapper->setMapping (ui->actionProfile, 23);
    signalMapper->setMapping (ui->actionFaceNormal, 24);
    signalMapper->setMapping (ui->actionNodeNormal, 25);
    ui->actionShade->setChecked(true);

    connect(ui->actionShifttoOrigin, SIGNAL(triggered(bool)), this, SLOT(shiftToOrigin()));

    // select
    connect(ui->actionSelectNode, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionSelectEdge, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
    connect(ui->actionSelectFace, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectFix, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));
	connect(ui->actionSelectHandle, SIGNAL(triggered(bool)), signalMapper, SLOT(map()));

	signalMapper->setMapping (ui->actionSelectNode, 30);
    signalMapper->setMapping (ui->actionSelectEdge, 31);
    signalMapper->setMapping (ui->actionSelectFace, 32);
	signalMapper->setMapping(ui->actionSelectFix, 33);
	signalMapper->setMapping(ui->actionSelectHandle, 34);


    connect (signalMapper, SIGNAL(mapped(int)), this, SLOT(signalNavigation(int)));

	//Button
	//connect(ui->pushButton_ShowAllLayers, SIGNAL(released()), this, SLOT(viewAllWaypointLayers()));
	//connect(ui->spinBox_ShowLayerIndex, SIGNAL(valueChanged(int)), this, SLOT(changeWaypointDisplay()));

    connect(ui->pushButton_Opt, SIGNAL(released()), this, SLOT(trajOptimization()));
    connect(ui->pushButton_loadRob, SIGNAL(released()), this, SLOT(loadRob()));
    connect(ui->pushButton_getBaseColi, SIGNAL(released()), this, SLOT(getBaseColi()));
    connect(ui->pushButton_trainBaseColi, SIGNAL(released()), this, SLOT(trainBaseColi()));
    //connect(ui->pushButton_tmp, SIGNAL(released()), this, SLOT(tmp()));
    //connect(ui->pushButton_default,SIGNAL(released()),this,SLOT(setDefault()));
    connect(ui->pushButton_inputPathLayer, SIGNAL(released()), this, SLOT(inputPathLayer()));
    connect(ui->pushButton_iniOpt, SIGNAL(released()), this, SLOT(iniOpt()));
    //connect(ui->pushButton_test, SIGNAL(released()), this, SLOT(test()));
    
}

void MainWindow::open()
{
    QString filenameStr = QFileDialog::getOpenFileName(this, tr("Open File,"), "..", tr(""));
    QFileInfo fileInfo(filenameStr);
    QString fileSuffix = fileInfo.suffix();
    QByteArray filenameArray = filenameStr.toLatin1();
    char *filename = filenameArray.data();

    // set polygen name
    std::string strFilename(filename);
    std::size_t foundStart = strFilename.find_last_of("/");
    std::size_t foundEnd = strFilename.find_last_of(".");
    std::string modelName;
    modelName = strFilename.substr(0,foundEnd);
    modelName = modelName.substr(foundStart+1);
    
    if (QString::compare(fileSuffix,"obj") == 0){
        PolygenMesh *polygenMesh = new PolygenMesh(UNDEFINED);
        polygenMesh->ImportOBJFile(filename,modelName);
        polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
        pGLK->AddDisplayObj(polygenMesh,true);
        polygenMeshList.AddTail(polygenMesh);
    }

	else if (QString::compare(fileSuffix, "tet") == 0) {
		PolygenMesh *polygenMesh = new PolygenMesh(TET);
		std::cout << filename << std::endl;
		std::cout << modelName << std::endl;
		polygenMesh->ImportTETFile(filename, modelName);
		polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
		pGLK->AddDisplayObj(polygenMesh, true);
		polygenMeshList.AddTail(polygenMesh);
	}

    updateTree();

    shiftToOrigin();
    pGLK->refresh(true);
}

void MainWindow::save()
{
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	if (!polygenMesh)
		return;
	QString filenameStr = QFileDialog::getSaveFileName(this, tr("OBJ File Export,"), "..", tr("OBJ(*.obj)"));
	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();

	if (QString::compare(fileSuffix, "obj") == 0) {
		QFile exportFile(filenameStr);
		if (exportFile.open(QFile::WriteOnly | QFile::Truncate)) {
			QTextStream out(&exportFile);
			for (GLKPOSITION posMesh = polygenMesh->GetMeshList().GetHeadPosition(); posMesh != nullptr;) {
				QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetNext(posMesh);
				for (GLKPOSITION posNode = patch->GetNodeList().GetHeadPosition(); posNode != nullptr;) {
					QMeshNode *node = (QMeshNode*)patch->GetNodeList().GetNext(posNode);
					double xx, yy, zz;
					node->GetCoord3D(xx, yy, zz);
					float r, g, b;
					node->GetColor(r, g, b);
					out << "v " << xx << " " << yy << " " << zz << " " << node->value1 << endl;
				}
				for (GLKPOSITION posFace = patch->GetFaceList().GetHeadPosition(); posFace != nullptr;) {
					QMeshFace *face = (QMeshFace*)patch->GetFaceList().GetNext(posFace);
					out << "f " << face->GetNodeRecordPtr(0)->GetIndexNo() << " " << face->GetNodeRecordPtr(1)->GetIndexNo() << " " << face->GetNodeRecordPtr(2)->GetIndexNo() << endl;
				}
			}
		}
		exportFile.close();
	}
}

void MainWindow::saveSelection()
{
	//printf("%s exported\n", Model->ModelName);

	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char * c = filename.c_str();
	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char output_filename[256];
	strcpy(output_filename, "..\\selection_file\\");
	strcat(output_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(output_filename, filetype);

	ofstream nodeSelection(output_filename);
	if (!nodeSelection)
		cerr << "Sorry!We were unable to build the file NodeSelect!\n";
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		nodeSelection << CheckNode->GetIndexNo() << ":";
		//for the selection of fixing part
		if (CheckNode->isFixed == true) nodeSelection << "1:";
		else nodeSelection << "0:";

		//for the selection of hard part
		if (CheckNode->isHandle == true) nodeSelection << "1:" <<std::endl;
		else nodeSelection << "0:" << std::endl;
	}

	nodeSelection.close();
	printf("Finish output selection \n");
}

void MainWindow::readSelection()
{
	PolygenMesh *polygenMesh = getSelectedPolygenMesh();
	if (!polygenMesh)
		polygenMesh = (PolygenMesh*)polygenMeshList.GetHead();
	QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();

	std::string filename = polygenMesh->getModelName();
	const char * c = filename.c_str();

	char *cstr = new char[filename.length() + 1];
	strcpy(cstr, filename.c_str());

	const char * split = ".";
	char* p = strtok(cstr, split);

	char input_filename[256];
	strcpy(input_filename, "..\\selection_file\\");
	strcat(input_filename, cstr);
	char filetype[64];
	strcpy(filetype, ".txt");
	strcat(input_filename, filetype);

	ifstream nodeSelect(input_filename);
	if (!nodeSelect)
		cerr << "Sorry!We were unable to open the file!\n";
	vector<int> NodeIndex(patch->GetNodeNumber()), checkNodeFixed(patch->GetNodeNumber()), checkNodeHandle(patch->GetNodeNumber());
	//string line;
	int LineIndex1 = 0;
	string sss;
	while (getline(nodeSelect, sss)){
		const char * c = sss.c_str();
		sscanf(c, "%d:%d:%d", &NodeIndex[LineIndex1], &checkNodeFixed[LineIndex1], &checkNodeHandle[LineIndex1]);
		LineIndex1++;
	}

	nodeSelect.close();
	for (GLKPOSITION Pos = patch->GetNodeList().GetHeadPosition(); Pos;) {
		QMeshNode *CheckNode = (QMeshNode*)patch->GetNodeList().GetNext(Pos);
		if (checkNodeFixed[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isFixed = true;
		if (checkNodeHandle[CheckNode->GetIndexNo() - 1] == 1) CheckNode->isHandle = true;
	}

	for (GLKPOSITION Pos = patch->GetFaceList().GetHeadPosition(); Pos != NULL;)
	{
		QMeshFace* Face = (QMeshFace*)patch->GetFaceList().GetNext(Pos);
		if (Face->GetNodeRecordPtr(0)->isHandle == true &&
			Face->GetNodeRecordPtr(1)->isHandle == true &&
			Face->GetNodeRecordPtr(2)->isHandle == true)
			Face->isHandleDraw = true;
		else Face->isHandleDraw = false;

		if (Face->GetNodeRecordPtr(0)->isFixed == true &&
			Face->GetNodeRecordPtr(1)->isFixed == true &&
			Face->GetNodeRecordPtr(2)->isFixed == true)
			Face->isFixedDraw = true;
		else Face->isFixedDraw = false;
	}
	printf("Finish input selection \n");
	pGLK->refresh(true);

}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
	//QMouseEvent *e = (QMouseEvent*)event;
	//QPoint pos = e->pos();
	//cout << "Mouse position updated" << endl;
	//double wx, wy, wz;
	//pGLK->screen_to_wcl(100.0, 100.0, wx, wy, wz);
	//ui->CorrdinateMouse->setText(QString("X = %1").arg(wx));

	//QString text;
	//text = QString("%1 X %2").arg(event->pos().x()).arg(event->pos().y());
	///** Update the info text */
	//ui->statusBar->showMessage(text);
}

void MainWindow::signalNavigation(int flag)
{
    if (flag <= 10)
        pGLK->setNavigation(flag);
    if (flag >=20 && flag <=25){
        pGLK->setViewModel(flag-20);
        switch (flag) {
        case 20:
            ui->actionShade->setChecked(pGLK->getViewModel(0));
            break;
        case 21:
            ui->actionMesh->setChecked(pGLK->getViewModel(1));
            break;
        case 22:
            ui->actionNode->setChecked(pGLK->getViewModel(2));
            break;
        case 23:
            ui->actionProfile->setChecked(pGLK->getViewModel(3));
            break;
        case 24:
            ui->actionFaceNormal->setChecked(pGLK->getViewModel(4));
            break;
        case 25:
            ui->actionNodeNormal->setChecked(pGLK->getViewModel(5));
            break;
        }
    }
  //  if (flag==30 || flag==31 || flag==32 || flag == 33 || flag == 34){
  //      InteractiveTool *tool;
  //      switch (flag) {
  //      case 30:
  //          tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NODE, ui->boxDeselect->isChecked());
  //          break;
  //      case 31:
  //          tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), EDGE, ui->boxDeselect->isChecked());
  //          break;
  //      case 32:
  //          tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FACE, ui->boxDeselect->isChecked());
  //          break;
		//case 33:
		//	tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), FIX, ui->boxDeselect->isChecked());
		//	break;
		//case 34:
		//	tool = new InteractiveTool(pGLK, &polygenMeshList, (GLKMouseTool*)pGLK->GetCurrentTool(), NHANDLE, ui->boxDeselect->isChecked());
		//	break;
  //      }
  //      pGLK->set_tool(tool);
  //  }
}

void MainWindow::shiftToOrigin()
{
    
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls())
        event->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent *event)
{
    QString filenameStr;
    foreach (const QUrl &url, event->mimeData()->urls())
        filenameStr = url.toLocalFile();
    QByteArray filenameArray = filenameStr.toLatin1();
    char *filename = filenameArray.data();

    PolygenMesh *polygenMesh = new PolygenMesh(UNDEFINED);

    // set polygen name
    std::string strFilename(filename);
    std::size_t foundStart = strFilename.find_last_of("/");
    std::size_t foundEnd = strFilename.find_last_of(".");
    std::string modelName;
    modelName = strFilename.substr(0,foundEnd);
    modelName = modelName.substr(foundStart+1);
    int i = 0;
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygen = (PolygenMesh*)polygenMeshList.GetNext(pos);
        std::string name = (polygen->getModelName()).substr(0,(polygen->getModelName()).find(' '));
        if (name == modelName)
            i++;
    }
    if (i > 0)
        modelName += " "+std::to_string(i);

	QFileInfo fileInfo(filenameStr);
	QString fileSuffix = fileInfo.suffix();
	if (QString::compare(fileSuffix, "obj") == 0) {
		polygenMesh->ImportOBJFile(filename, modelName);
	}
	else if (QString::compare(fileSuffix, "tet") == 0) {
		polygenMesh->ImportTETFile(filename, modelName);
        polygenMesh->meshType = TET;
	}
	polygenMesh->m_bVertexNormalShading = false;	
    polygenMesh->BuildGLList(polygenMesh->m_bVertexNormalShading);
    pGLK->AddDisplayObj(polygenMesh,true);
    polygenMeshList.AddTail(polygenMesh);
    
    updateTree();
}

void MainWindow::createTreeView()
{
    treeModel = new QStandardItemModel();
    ui->treeView->setModel(treeModel);
    ui->treeView->setHeaderHidden(true);
    ui->treeView->setContextMenuPolicy(Qt::CustomContextMenu);
    ui->treeView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->treeView->expandAll();
}

void MainWindow::updateTree()
{
    treeModel->clear();
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        QString modelName = QString::fromStdString(polygenMesh->getModelName());
        QStandardItem *modelListItem = new QStandardItem(modelName);
        modelListItem->setCheckable(true);
        modelListItem->setCheckState(Qt::Checked);
        treeModel->appendRow(modelListItem);
    }
	pGLK->refresh(true);
}

PolygenMesh *MainWindow::getSelectedPolygenMesh()
{
    if (!treeModel->hasChildren())
        return nullptr;
    QModelIndex index = ui->treeView->currentIndex();
    QString selectedModelName = index.data(Qt::DisplayRole).toString();
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr;){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        QString modelName = QString::fromStdString(polygenMesh->getModelName());
        if (QString::compare(selectedModelName,modelName) == 0)
            return polygenMesh;
    }
    return nullptr;
}

void MainWindow::on_pushButton_clearAll_clicked()
{
    int i = 0;
    for (GLKPOSITION pos=polygenMeshList.GetHeadPosition(); pos!=nullptr; i++){
        PolygenMesh *polygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        QMeshPatch *patch = (QMeshPatch*)polygenMesh->GetMeshList().GetHead();
        if (i<2)
            continue;
        for (GLKPOSITION pos2=patch->GetFaceList().GetHeadPosition(); pos2!=nullptr;){
            QMeshFace *face = (QMeshFace*)patch->GetFaceList().GetNext(pos2);
            face->m_nIdentifiedPatchIndex = 0;
        }
    }
    pGLK->refresh(true);
}

void MainWindow::on_treeView_clicked(const QModelIndex &index)
{
    ui->treeView->currentIndex();
    QStandardItem *modelListItem = treeModel->itemFromIndex(index);
    ui->treeView->setCurrentIndex(index);
    PolygenMesh *polygenMesh = getSelectedPolygenMesh();
    if (modelListItem->checkState() == Qt::Checked)
        polygenMesh->bShow = true;
    else
        polygenMesh->bShow = false;
    pGLK->refresh(true);
}

PolygenMesh* MainWindow::_detectPolygenMesh(mesh_type type) {

    PolygenMesh* detectedMesh = NULL;
    for (GLKPOSITION pos = polygenMeshList.GetHeadPosition(); pos != nullptr;) {
        PolygenMesh* thispolygenMesh = (PolygenMesh*)polygenMeshList.GetNext(pos);
        if (thispolygenMesh->meshType == type) {
            detectedMesh = thispolygenMesh; break;
        }
    }
    return detectedMesh;
}


PolygenMesh* MainWindow::_buildPolygenMesh(mesh_type type, std::string name) {

    PolygenMesh* newMesh = new PolygenMesh(type);
    newMesh->setModelName(name);
    newMesh->BuildGLList(newMesh->m_bVertexNormalShading);
    pGLK->AddDisplayObj(newMesh, true);
    polygenMeshList.AddTail(newMesh);
    updateTree();
    return newMesh;

}

void MainWindow::setDefault() {
    ui->lineEdit_SorceDataDir->setText("../DataSet/models/freeformBig/");
    ui->doubleSpinBox_hPos->setValue(6.6 + 5 + 1);

    /*ui->lineEdit_toolTipX->setText("151.762");
    ui->lineEdit_toolTipY->setText("-88.265");
    ui->lineEdit_toolTipZ->setText("188.612");*/
    ui->lineEdit_toolTipX->setText("151.585");
    ui->lineEdit_toolTipY->setText("-88.7572");
    ui->lineEdit_toolTipZ->setText("189.022");

    ui->spinBox_setLayer->setValue(49);
    ui->doubleSpinBox_vTip->setValue(20);
    ui->lineEdit_consVt->setText("60");
    ui->lineEdit_consAt->setText("200");
    ui->lineEdit_consAn->setText("200");
    ui->doubleSpinBox_enhancePos->setValue(1.2);
    ui->doubleSpinBox_alpha->setValue(0.94);
    ui->doubleSpinBox_beta->setValue(0.99);
    ui->lineEdit_gamma->setText("0.98");
    ui->doubleSpinBox_vMaxRob->setValue(0.5);
    ui->doubleSpinBox_vMaxPos->setValue(1);
    ui->doubleSpinBox_jMaxRob->setValue(20);
    ui->doubleSpinBox_jMaxPos->setValue(20);
    ui->doubleSpinBox_kv->setValue(0);
    ui->doubleSpinBox_ka->setValue(0.5);
    ui->doubleSpinBox_kj->setValue(1);

    ui->lineEdit_kt->setText("1");

    ui->doubleSpinBox_sepPath->setValue(3);

    ui->checkBox_enhancePos->setChecked(true);

    ui->pushButton_getBaseColi->setDisabled(true);
    ui->pushButton_trainBaseColi->setDisabled(true);

    ui->pushButton_iniOpt->setDisabled(true);
    ui->pushButton_Opt->setDisabled(true);

    ui->lineEdit_sizeWindow->setText("100");

    ui->checkBox_checkCollision->setDisabled(true);
}




void::MainWindow::loadRob() {
    Eigen::Vector3d pToolTip;
    pToolTip << ui->lineEdit_toolTipX->text().toDouble(), ui->lineEdit_toolTipY->text().toDouble(), ui->lineEdit_toolTipZ->text().toDouble();
    delete robs;
    robs = new robSystem(ui->doubleSpinBox_hPos->value(), pToolTip);


    fileIO* IO_operator = new fileIO();

    //导入IRB2600模型
    PolygenMesh* Rob_Model = this->_detectPolygenMesh(ROBOT);
    std::string FileDir = "../DataSet/RobSys/IRB2600/";

    if (Rob_Model == NULL) {
        Rob_Model = this->_buildPolygenMesh(ROBOT, "ABBrobot");
        IO_operator->readRobotData(Rob_Model, robs, FileDir);
    }
    else {
        IO_operator->readRobotData(robs, FileDir);
    }

    //导入变位机模型
    PolygenMesh* posModel = this->_detectPolygenMesh(POSITIONER);
    FileDir = "../DataSet/RobSys/a250Model/";
    if (posModel == NULL) {
        posModel = this->_buildPolygenMesh(POSITIONER, "ABBpositioner");
        IO_operator->readPosData(posModel, robs, FileDir);
    }
    else {
        IO_operator->readPosData(robs, FileDir);
    }

    //设置变位机初始位姿
    robs->setInitialPos(posModel);

    delete IO_operator;

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();

    std::cout << "robs loaded!" << std::endl;
}

void MainWindow::getBaseColi() {
    std::cout<<"\n\nstart get base collision data"<<std::endl;

    //以下变量可以简化为7自由度，因为C轴角度不影响碰撞检测

    //生成随机数种子
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);

    Eigen::VectorXd JointMax, JointMin;
    JointMax.resize(8);
    JointMin.resize(8);
    JointMax << M_PI/2, 2.618, 1.309, 6.9813, M_PI / 2, 6.9813, M_PI, 6 * M_PI;
    JointMin << -M_PI/2, -M_PI / 2, -M_PI, -6.9813, -M_PI / 2, -6.9813, -M_PI, -6 * M_PI;


    std::vector<Eigen::Matrix<double, 8, 1>> samJoint;
    std::vector<double> samColi;

    //随机采样
    for (int i = 0; i < 1000; i++) {
        Eigen::Matrix<double, 8, 1> q;

        for (int j = 0; j < 8; j++) {
			q(j) = dis(gen) * (JointMax(j) - JointMin(j)) + JointMin(j);
		}

        samJoint.push_back(q);

	}
    std::cout<<"1. samJoint.size=:"<<samJoint.size()<<std::endl;


    //Upscaling
    //提高采样密度
    int k = 5;//k_NN
    double tau_a = 0.5;

    for (int i = 0; ; i++) {

        if (i == samJoint.size()) {
            std::cout << "2. samJoint.size=:" << samJoint.size() << std::endl;
            break;
        }

        Eigen::Matrix<double, 8, 1> a = samJoint[i];

        std::vector<std::pair<double, int>> distance(samJoint.size());
        //计算距离
        for (int j = 0; j < samJoint.size(); j++) {

            if (j == i) {
                distance[j] = { 0,j };
                continue;
            }
            distance[j] = { (samJoint[j] - a).norm(),j };
        }

        //排序
        std::sort(distance.begin(), distance.end());


        for (int j = k; j >= 0; j--) {
            
            if (j == k) {
                if (distance[j].first > tau_a) {
                    i--;//不满足条件，下一轮继续检查当前点
                }
                else {
                    break;//满足条件，跳出循环
                }
            }


            if (distance[j].first > tau_a) {
                //对每个不满足要求的点生成中间点
                Eigen::Matrix<double, 8, 1> qNew = (samJoint[distance[j].second] + a) / 2;
                samJoint.push_back(qNew);
            }
            else {
                break;
            }
        }

    }

    //检测碰撞
    samColi.resize(samJoint.size());
//#pragma omp parallel for
    for (int i = 0; i < samJoint.size(); i++) {
        bool tmp = robs->checkBaseColi(samJoint[i]);
        if (tmp == true) {
            //发生碰撞
            std::cout<<"collision:"<<i << std::endl;
            samColi[i] = 1;
        }
        else {
            samColi[i] = -1;
        }
    }



    //refinement
    double tau_b = 0.05;
    for (int i = 0; ; i++) {

        if (i == samJoint.size()) {
            std::cout << "3. samJoint.size=:" << samJoint.size() << std::endl;
            break;
        }

        Eigen::Matrix<double, 8, 1> a = samJoint[i];

        std::vector<std::pair<double, int>> distance(samJoint.size());
        //计算距离
        for (int j = 0; j < samJoint.size(); j++) {

            if (j == i) {
                distance[j] = { 0,j };
                continue;
            }
            distance[j] = { (samJoint[j] - a).norm(),j };
        }


        //排序
        std::sort(distance.begin(), distance.end());


        bool checkAgain = false;
        for (int j = k; j >= 0; j--) {

            if (distance[j].first > tau_b && samColi[i] != samColi[distance[j].second]) {
                //对每个不满足要求的点生成中间点
                Eigen::Matrix<double, 8, 1> qNew = (samJoint[distance[j].second] + a) / 2;
                samJoint.push_back(qNew);

                //检查碰撞
                bool tmp = robs->checkBaseColi(qNew);
                if (tmp == true) {
                    //发生碰撞
                    samColi.push_back(1);
                }
                else {
                    samColi.push_back(-1);
                }

                //需要再次检查当前点
                if (!checkAgain) {
                    checkAgain = true;
                    i--;
                }
            }
            else if (distance[j].first <= tau_b) {
                break;
            }
        }

    }

    //output dataset for svm training
    //string filename = "../DataSet/coliTrain/BaseColi/BaseColi";
    //fileIO* IO_operator = new fileIO();
    //IO_operator->writeBaseColi(samJoint, samColi, filename);
    //delete IO_operator;



    //output dataset for svm training new(FK kernel)
    string filename = "../DataSet/coliTrain/BaseColi/BaseColiFK";
    std::vector<Eigen::VectorXd> posJoints;
    fileIO* IO_operator = new fileIO();
    for (int i = 0; i < samJoint.size(); i++) {
		Eigen::VectorXd q = samJoint[i];
        Eigen::VectorXd pos = robs->Joints2Postions(q, true);
		posJoints.push_back(pos);
	}
    IO_operator->writeBaseColi(posJoints, samColi, filename);
    delete IO_operator;

	std::cout<<"base collision data obtained!"<<std::endl;
}

void MainWindow::trainBaseColi() {
    std::cout << "\n\nstart train base collision proxy model" << std::endl;

    colliTrain* colli = new colliTrain();

    //train
    //-g,-h的说明见libsvm的README/colli::svmtrain的函数内部
    char* svmcommand[] = { (char*)"-g",(char*)"0.7",(char*)"-h",(char*)"0",(char*)"../DataSet/coliTrain/BaseColi/BaseColiFK"};
    int length_of_command;
    length_of_command = sizeof(svmcommand) / sizeof(char*);
    std::cout << "length of command: " << length_of_command << std::endl;
    colli->svmtrain(length_of_command, svmcommand, 19);


    //test_predict
    char* svmpredict_command[3];
    svmpredict_command[0] = (char*)"../DataSet/coliTrain/BaseColi/BaseColiFK";
    svmpredict_command[1] = (char*)"../DataSet/coliTrain/BaseColi/BaseColiFK.model";
    svmpredict_command[2] = (char*)"../DataSet/coliTrain/BaseColi/BaseColiFK_predict";
    length_of_command = sizeof(svmpredict_command) / sizeof(char*);
    colli->svmpredict(length_of_command, svmpredict_command);


    //test svmGrad
    string dataFile = "../DataSet/coliTrain/BaseColi/BaseColiFK";
    string modelFile = "../DataSet/coliTrain/BaseColi/BaseColiFK.modelG";
    SVMGrad svmGrad(modelFile);
    svmGrad.preComputeKernel(true);

    std::vector<Eigen::VectorXd> Joints;
    std::vector<double> labels;
    fileIO* IO_operator = new fileIO();
    IO_operator->readJointsFromSVMData(Joints, labels, dataFile);
    delete IO_operator;

    for (int i = 0; i < 100; i++) {
        Eigen::VectorXd q = Joints[i];
        double gamma=svmGrad.calculateGamma(q);
        std::cout << i << ": " << gamma << " " << labels[i] << std::endl;
    }

    delete colli;
    std::cout << "end of train" << std::endl;
}

void MainWindow::tmp() {
    std::cout<<"\n\nstart tmp"<<std::endl;
    taseMethod* tase1 = new taseMethod();
    tase1->test = 1;
    taseMethod* tase2 = new taseMethod(*tase1);
    tase2->test = 2;

    std::cout<<"tase 1"<<tase1->test<<std::endl;
    std::cout<<"tase 2"<<tase2->test<<std::endl;

    delete tase1;
    delete tase2;
}

void MainWindow::inputPathLayer() {
    std::string baseFile = ui->lineEdit_SorceDataDir->text().toStdString();
    int layerIndex = ui->spinBox_setLayer->value();
    bool checkColi = ui->checkBox_checkCollision->isChecked();

    std::cout << "\nWorking on layer: " << layerIndex << std::endl;
    iLayer = to_string(layerIndex);

    //输入layer
    PolygenMesh* Layers = this->_detectPolygenMesh(CURVED_LAYER);
    PQP_Model* layer_PQP = new PQP_Model();

    if (Layers == NULL) {//没有layer,则新建
        Layers = this->_buildPolygenMesh(CURVED_LAYER, "Layers");
    }

    std::string FileDir = baseFile + "layer/";
    std::ifstream infile(FileDir + iLayer + ".obj");
    if (!infile.good()) {
        std::cout << "no " << layerIndex << ".obj" << std::endl;
        iLayer = iLayer + "S";
        std::ifstream infile2(FileDir + iLayer + ".obj");
        if (!infile2.good()) {
            std::cerr << "something wrong of the input layer" << std::endl;
        }
    }

    fileIO* IO_operator = new fileIO();
    if (!isLayerLoaded) {
        //读取layer
        IO_operator->readLayerData(Layers, robs, layer_PQP, FileDir, iLayer);
        isLayerLoaded = true;

        //读取路径
        toolpath* layerPath = new toolpath();
        PolygenMesh* Paths = this->_detectPolygenMesh(Tool_PATH);
        if (Paths == NULL) {
            Paths = this->_buildPolygenMesh(Tool_PATH, "Paths");
        }
        std::string pathDir = baseFile + "waypoint/";
        IO_operator->inputLayerPath(layerPath, pathDir, iLayer, Paths);
        std::cout << "length of path: " << layerPath->pathPoint.size() << std::endl;

        //当前patch
        QMeshPatch* layerPatch = (QMeshPatch*)Layers->GetMeshList().GetTail();
        robs->Trans_mesh(layerPatch, robs->TransPos0[3], false);
        //构造碰撞检测模型
        delete colli;
        colli = new colliTrain();
        if (checkColi) {

            std::cout << "start construct collision proxy" << std::endl;

            colliDataGotten colliData;
            colliDataGotten supportData;

            //第一层采样
            colliData = colli->samFirstLayer(layerPatch, layerPath, robs, layer_PQP, 5, layerIndex);

            //IO_operator->readColliData(colliData, "../DataSet/coliTrain/colliData_" + iLayer);

            if (colliData.coli[0] == 0) {
                checkColi = false;
            }
            else {
                IO_operator->writeColliData(colliData, "../DataSet/coliTrain/colliData_" + iLayer);

                //构造碰撞检测模型
                supportData = colli->FastronModelUpdate(colliData);
                IO_operator->writeColliData(supportData, "../DataSet/coliTrain/supportData_" + iLayer);
                IO_operator->writeVectorXd(colli->alpha, "../DataSet/coliTrain/supportAlpha_" + iLayer);
                IO_operator->writeVectorXd(colli->F, "../DataSet/coliTrain/supportF_" + iLayer);

                std::cout << "constructe collision model" << std::endl;
            }



            /*IO_operator->readColliData(colliData, "../DataSet/coliTrain/colliData_" + iLayer);

            IO_operator->readColliData(supportData, "../DataSet/coliTrain/supportData_" + iLayer);
            colli->supportData = supportData;
            Eigen::VectorXd Alpha;
            IO_operator->readVectorXd(Alpha, "../DataSet/coliTrain/supportAlpha_" + iLayer);
            colli->alpha = Alpha;
            Eigen::VectorXd F;
            IO_operator->readVectorXd(F, "../DataSet/coliTrain/supportF_" + iLayer);
            colli->F = F;
            colli->newFastron = false;*/

            std::cout << "constructe collision model" << std::endl;
        }

        trajOpt* Opt = new trajOpt();
        Opt->setMaxDist(ui->doubleSpinBox_sepPath->value());
        //拆分独立路径
        pathsInLayer = Opt->splitPath(layerPath);
        delete Opt;
        std::cout << "number of paths: " << pathsInLayer.size() << std::endl;

        ui->lcdNumber_pathNumber->display(double(pathsInLayer.size()));

        ui->spinBox_pathIndex->setMaximum(pathsInLayer.size() - 1);

        delete layerPath;
    }
    else {
        IO_operator->readLayerData(robs, FileDir, iLayer);
    }
    delete IO_operator;


    delete layer_PQP;

    ui->pushButton_iniOpt->setEnabled(true);
}

void MainWindow::iniOpt() {
    std::cout << "\n\ninitial optimization" << std::endl;

    int i = ui->spinBox_pathIndex->value();
    int layerIndex = ui->spinBox_setLayer->value();
    fileIO* IO_operator = new fileIO();
    bool checkColi = ui->checkBox_checkCollision->isChecked();
    std::string baseFile = ui->lineEdit_SorceDataDir->text().toStdString();

    delete Opt;
    Opt = new trajOpt();

    //set time
    Eigen::VectorXd time;

    if (ui->checkBox_inputTime->isChecked()) {
        IO_operator->readVectorXd(time, baseFile + "timenew.txt");
        std::cout << time.size() << std::endl;
        pathsInLayer[i]->getDTime(time);
    }


    //set tooltip velocity and constraint
    Opt->setvTip(ui->doubleSpinBox_vTip->value());
    Opt->setTipCons(ui->lineEdit_consVt->text().toDouble(), ui->lineEdit_consAt->text().toDouble(), ui->lineEdit_consAn->text().toDouble());
    //Opt->setTipCons(30, 350, 350);

    //set matrix M
    Eigen::Matrix<double, 8, 8> M = Eigen::Matrix<double, 8, 8>::Identity();
    if (ui->checkBox_enhancePos->isChecked()) {
        M(6, 6) = ui->doubleSpinBox_enhancePos->value();
        M(7, 7) = ui->doubleSpinBox_enhancePos->value();
    }
    Opt->setM(M);

    Opt->setAngles(ui->doubleSpinBox_alpha->value(), ui->doubleSpinBox_beta->value(), ui->lineEdit_gamma->text().toDouble());

    //set jMax
    Eigen::Matrix<double, 8, 1> jMax = Eigen::Matrix<double, 8, 1>::Ones();
    jMax = ui->doubleSpinBox_jMaxRob->value() * jMax;
    jMax(6) = ui->doubleSpinBox_jMaxPos->value();
    jMax(7) = ui->doubleSpinBox_jMaxPos->value();
    Opt->setJmax(jMax);

    //set vMax
    Eigen::Matrix<double, 8, 1> vMax = Eigen::Matrix<double, 8, 1>::Ones();
    vMax = ui->doubleSpinBox_vMaxRob->value() * vMax;
    vMax(6) = ui->doubleSpinBox_vMaxPos->value();
    vMax(7) = ui->doubleSpinBox_vMaxPos->value();
    Opt->setVmax(vMax);


    //1. 路径初始化
    Opt->initializePath(pathsInLayer[i], robs, colli, false, layerIndex);

    //2. 优化初始化
    Eigen::Vector4d coef;
    coef << ui->doubleSpinBox_kv->value(), ui->doubleSpinBox_ka->value(), ui->doubleSpinBox_kj->value(), ui->lineEdit_kt->text().toDouble();
    Opt->initializeOpt(pathsInLayer[i], robs, ui->checkBox_optTime->isChecked(), coef);


    /*std::vector<Eigen::Matrix<double, 8, 1>> q, v, a, j;
    Eigen::VectorXd t, vT, atT, anT;

    Opt->getQVAJT(q, v, a, j, t, vT, atT, anT);
    IO_operator->writeVar8(q, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_q0.txt");
    IO_operator->writeVar8(v, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_v0.txt");
    IO_operator->writeVar8(a, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_a0.txt");
    IO_operator->writeVar8(j, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_j0.txt");
    IO_operator->writeVectorXd(t, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_t0.txt");
    IO_operator->writeVectorXd(vT, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_vT0.txt");
    IO_operator->writeVectorXd(atT, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_atT0.txt");
    IO_operator->writeVectorXd(anT, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_anT0.txt");*/
     

    //std::vector<Eigen::Matrix<double, 8, 1>> Joints;
    //std::vector<Transform3d> Ttcps;       

    //Opt->getSolution(Joints, Ttcps, time);
    //IO_operator->writeVar8(Joints, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_Joints0.txt");
    //IO_operator->writeVectorXd(time, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_time0.txt");
    //IO_operator->writeTransform3d(Ttcps, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_Ttcps0.txt");

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();

    delete IO_operator;

    ui->pushButton_Opt->setEnabled(true);
}


void MainWindow::trajOptimization() {
    std::cout << "\n\ninitial optimization" << std::endl;


    fileIO* IO_operator = new fileIO();
    int i = ui->spinBox_pathIndex->value();
    bool checkColi = ui->checkBox_checkCollision->isChecked();

     
    trajOpt* Opt2 = new trajOpt(*Opt);
     //3. Block Coordinate Descent
    int sizeWindow = ui->lineEdit_sizeWindow->text().toInt();
    bool showDetail = ui->checkBox_showDetail->isChecked();

    if (ui->checkBox_optTime->isChecked()) {
        Opt2->bcd_time(pathsInLayer[i], robs, colli, checkColi, sizeWindow, showDetail);
    }
    else {
        Opt2->bcd(pathsInLayer[i], robs, colli, checkColi, sizeWindow, showDetail);
    }       

    //std::vector<Eigen::Matrix<double, 8, 1>> q, v, a, j;
    //Eigen::VectorXd t, vT, atT, anT;
    // 
    //Opt2->getQVAJT(q, v, a, j, t, vT, atT, anT);
    //IO_operator->writeVar8(q, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_q.txt");
    //IO_operator->writeVar8(v, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_v.txt");
    //IO_operator->writeVar8(a, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_a.txt");
    //IO_operator->writeVar8(j, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_j.txt");

    //IO_operator->writeVectorXd(t, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_t.txt");
    //IO_operator->writeVectorXd(vT, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_vT.txt");
    //IO_operator->writeVectorXd(atT, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_atT.txt");
    //IO_operator->writeVectorXd(anT, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_anT.txt");


    //4. 输出路径
    //文件1：变位机角度、机器人末端位姿
    //文件2：机器人关节角度
    std::vector<Eigen::Matrix<double, 8, 1>> Joints;
    std::vector<Transform3d> Ttcps;
    Eigen::VectorXd time;

    Opt2->getSolution(Joints, Ttcps, time);
    IO_operator->writeVar8(Joints, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_Joints.txt");
    //IO_operator->writeVectorXd(time, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_time.txt");
    IO_operator->writeTransform3d(Ttcps, "../DataSet/output/layer" + iLayer + "_" + to_string(i) + "_Ttcps.txt");
            
            

    pGLK->refresh(true);
    pGLK->Zoom_All_in_View();

    delete IO_operator;  
    delete Opt2;

    std::cout << "end of Opt" << std::endl;
}

