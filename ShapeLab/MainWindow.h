#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSignalMapper>
#include <QStandardItemModel>
#include "../GLKLib/GLKLib.h"
#include "../QMeshLib/PolygenMesh.h"
#include <omp.h>
#include <QTimer>
#include <QLabel>

#include "robSystem.h"
#include "pathStruct.h"
#include "colliTrain.h"
#include "trajOpt.h"

#define PI		3.141592654
#define DEGREE_TO_ROTATE(x)		0.0174532922222*x
#define ROTATE_TO_DEGREE(x)		57.295780490443*x

using namespace std;

class DeformTet;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
	// Qtimer - defined function
    //void doTimerGcodeMoving();

private:
    Ui::MainWindow *ui;
    GLKLib *pGLK;

    /* add for Gcode generation */
    //QTimer Gcode_timer; //Gcode Simulation timer
    //int gocodetimerItertime;
    //int simuLayerInd;
    //Eigen::MatrixXf Gcode_Table;
    //unsigned int operationTime = 0;
    /* ------------------------ */
	GLKObList polygenMeshList;

private:
    void createActions();
    void createTreeView();
	void showTetraDeformationRatio();
	void MoveHandleRegion();
	void QTgetscreenshoot();

    PolygenMesh *getSelectedPolygenMesh();

    QSignalMapper *signalMapper;
    QStandardItemModel *treeModel;

	DeformTet *Deformation;

private:
    PolygenMesh* _detectPolygenMesh(mesh_type type);
    PolygenMesh* _buildPolygenMesh(mesh_type type, std::string name);


protected:
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);

private slots:
    void open();
    void save();
	void saveSelection();
	void readSelection();

    void signalNavigation(int flag);
    void shiftToOrigin();
    void updateTree();
	void mouseMoveEvent(QMouseEvent *event);
    void on_pushButton_clearAll_clicked();
    void on_treeView_clicked(const QModelIndex &index);

    // 18.11.23 by Yongxue
    void trajOptimization();
    void loadRob();
    void getBaseColi();
    void trainBaseColi();
    void tmp();
    void setDefault();
    void inputPathLayer();
    void iniOpt();


	/*This is Nano Printing*/

    /*This is for Display*/
    //void changeWaypointDisplay();
    //void viewAllWaypointLayers();

private:
    robSystem* robs = new robSystem(12.6, Eigen::Vector3d(151.762, -88.265, 188.612));
    std::vector<toolpath*> pathsInLayer;
    colliTrain* colli = new colliTrain();
    trajOpt* Opt = new trajOpt();

    bool isLayerLoaded = false;
    std::string iLayer;

};

#endif // MAINWINDOW_H
