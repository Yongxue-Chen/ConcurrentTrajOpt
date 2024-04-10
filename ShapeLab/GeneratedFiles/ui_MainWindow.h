/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpen;
    QAction *actionFront;
    QAction *actionBack;
    QAction *actionTop;
    QAction *actionBottom;
    QAction *actionLeft;
    QAction *actionRight;
    QAction *actionIsometric;
    QAction *actionZoom_In;
    QAction *actionZoom_Out;
    QAction *actionZoom_All;
    QAction *actionZoom_Window;
    QAction *actionShade;
    QAction *actionMesh;
    QAction *actionNode;
    QAction *actionSave;
    QAction *actionSelectNode;
    QAction *actionSelectFace;
    QAction *actionShifttoOrigin;
    QAction *actionProfile;
    QAction *actionFaceNormal;
    QAction *actionNodeNormal;
    QAction *actionSelectEdge;
    QAction *actionGenerate;
    QAction *actionTest_1;
    QAction *actionSelectFix;
    QAction *actionSelectHandle;
    QAction *actionSaveSelection;
    QAction *actionReadSelection;
    QAction *actionSelectChamber;
    QAction *actionExport_to_Abaqus_model;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout;
    QToolBar *navigationToolBar;
    QStatusBar *statusBar;
    QToolBar *selectionToolBar;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_MANY_3DP_CNC_CAM;
    QFrame *line;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label;
    QDoubleSpinBox *doubleSpinBox_hPos;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_2;
    QLineEdit *lineEdit_toolTipX;
    QLineEdit *lineEdit_toolTipY;
    QLineEdit *lineEdit_toolTipZ;
    QPushButton *pushButton_loadRob;
    QFrame *line_3;
    QHBoxLayout *horizontalLayout_13;
    QPushButton *pushButton_getBaseColi;
    QPushButton *pushButton_trainBaseColi;
    QFrame *line_2;
    QHBoxLayout *horizontalLayout_16;
    QLabel *label_PosNorFile;
    QLineEdit *lineEdit_SorceDataDir;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QSpinBox *spinBox_setLayer;
    QSpacerItem *horizontalSpacer_2;
    QCheckBox *checkBox_checkCollision;
    QHBoxLayout *horizontalLayout_14;
    QLabel *label_15;
    QDoubleSpinBox *doubleSpinBox_sepPath;
    QPushButton *pushButton_inputPathLayer;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QLCDNumber *lcdNumber_pathNumber;
    QLabel *label_4;
    QSpinBox *spinBox_pathIndex;
    QHBoxLayout *horizontalLayout_6;
    QCheckBox *checkBox_optTime;
    QCheckBox *checkBox_inputTime;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_6;
    QDoubleSpinBox *doubleSpinBox_vTip;
    QLabel *label_7;
    QLineEdit *lineEdit_consVt;
    QLineEdit *lineEdit_consAt;
    QLineEdit *lineEdit_consAn;
    QHBoxLayout *horizontalLayout_8;
    QCheckBox *checkBox_enhancePos;
    QSpacerItem *horizontalSpacer;
    QLabel *label_8;
    QDoubleSpinBox *doubleSpinBox_enhancePos;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_9;
    QDoubleSpinBox *doubleSpinBox_alpha;
    QDoubleSpinBox *doubleSpinBox_beta;
    QLineEdit *lineEdit_gamma;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_10;
    QDoubleSpinBox *doubleSpinBox_vMaxRob;
    QLabel *label_11;
    QDoubleSpinBox *doubleSpinBox_vMaxPos;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_12;
    QDoubleSpinBox *doubleSpinBox_jMaxRob;
    QLabel *label_13;
    QDoubleSpinBox *doubleSpinBox_jMaxPos;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_14;
    QDoubleSpinBox *doubleSpinBox_kv;
    QDoubleSpinBox *doubleSpinBox_ka;
    QDoubleSpinBox *doubleSpinBox_kj;
    QLineEdit *lineEdit_kt;
    QPushButton *pushButton_iniOpt;
    QHBoxLayout *horizontalLayout_15;
    QCheckBox *checkBox_showDetail;
    QHBoxLayout *horizontalLayout_17;
    QHBoxLayout *horizontalLayout_18;
    QLabel *label_16;
    QLineEdit *lineEdit_sizeWindow;
    QSpacerItem *horizontalSpacer_4;
    QPushButton *pushButton_Opt;
    QSpacerItem *verticalSpacer;
    QTreeView *treeView;
    QPushButton *pushButton_clearAll;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuView;
    QMenu *menuSelect;
    QToolBar *toolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1331, 1070);
        MainWindow->setMinimumSize(QSize(0, 0));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        MainWindow->setFont(font);
        MainWindow->setMouseTracking(true);
        MainWindow->setFocusPolicy(Qt::StrongFocus);
        MainWindow->setAcceptDrops(true);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/resource/Open Folder.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen->setIcon(icon);
        actionFront = new QAction(MainWindow);
        actionFront->setObjectName(QString::fromUtf8("actionFront"));
        actionFront->setCheckable(false);
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/resource/Front View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFront->setIcon(icon1);
        actionBack = new QAction(MainWindow);
        actionBack->setObjectName(QString::fromUtf8("actionBack"));
        actionBack->setCheckable(false);
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/resource/Back View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBack->setIcon(icon2);
        actionTop = new QAction(MainWindow);
        actionTop->setObjectName(QString::fromUtf8("actionTop"));
        actionTop->setCheckable(false);
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/resource/Top View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionTop->setIcon(icon3);
        actionBottom = new QAction(MainWindow);
        actionBottom->setObjectName(QString::fromUtf8("actionBottom"));
        actionBottom->setCheckable(false);
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/resource/Bottom View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionBottom->setIcon(icon4);
        actionLeft = new QAction(MainWindow);
        actionLeft->setObjectName(QString::fromUtf8("actionLeft"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/resource/Left View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionLeft->setIcon(icon5);
        actionRight = new QAction(MainWindow);
        actionRight->setObjectName(QString::fromUtf8("actionRight"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/resource/Right View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionRight->setIcon(icon6);
        actionIsometric = new QAction(MainWindow);
        actionIsometric->setObjectName(QString::fromUtf8("actionIsometric"));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/resource/Isometric View.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionIsometric->setIcon(icon7);
        actionZoom_In = new QAction(MainWindow);
        actionZoom_In->setObjectName(QString::fromUtf8("actionZoom_In"));
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/resource/Zoom In.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_In->setIcon(icon8);
        actionZoom_Out = new QAction(MainWindow);
        actionZoom_Out->setObjectName(QString::fromUtf8("actionZoom_Out"));
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/resource/Zoom Out.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Out->setIcon(icon9);
        actionZoom_All = new QAction(MainWindow);
        actionZoom_All->setObjectName(QString::fromUtf8("actionZoom_All"));
        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/resource/Zoom All.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_All->setIcon(icon10);
        actionZoom_Window = new QAction(MainWindow);
        actionZoom_Window->setObjectName(QString::fromUtf8("actionZoom_Window"));
        QIcon icon11;
        icon11.addFile(QString::fromUtf8(":/resource/Zoom Window.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Window->setIcon(icon11);
        actionShade = new QAction(MainWindow);
        actionShade->setObjectName(QString::fromUtf8("actionShade"));
        actionShade->setCheckable(true);
        QIcon icon12;
        icon12.addFile(QString::fromUtf8(":/resource/Shade.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShade->setIcon(icon12);
        actionMesh = new QAction(MainWindow);
        actionMesh->setObjectName(QString::fromUtf8("actionMesh"));
        actionMesh->setCheckable(true);
        QIcon icon13;
        icon13.addFile(QString::fromUtf8(":/resource/Mesh.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionMesh->setIcon(icon13);
        actionNode = new QAction(MainWindow);
        actionNode->setObjectName(QString::fromUtf8("actionNode"));
        actionNode->setCheckable(true);
        QIcon icon14;
        icon14.addFile(QString::fromUtf8(":/resource/Node.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNode->setIcon(icon14);
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        QIcon icon15;
        icon15.addFile(QString::fromUtf8(":/resource/Save as.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave->setIcon(icon15);
        actionSelectNode = new QAction(MainWindow);
        actionSelectNode->setObjectName(QString::fromUtf8("actionSelectNode"));
        QIcon icon16;
        icon16.addFile(QString::fromUtf8(":/resource/selectNode.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectNode->setIcon(icon16);
        actionSelectFace = new QAction(MainWindow);
        actionSelectFace->setObjectName(QString::fromUtf8("actionSelectFace"));
        QIcon icon17;
        icon17.addFile(QString::fromUtf8(":/resource/selectFace.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFace->setIcon(icon17);
        actionShifttoOrigin = new QAction(MainWindow);
        actionShifttoOrigin->setObjectName(QString::fromUtf8("actionShifttoOrigin"));
        actionProfile = new QAction(MainWindow);
        actionProfile->setObjectName(QString::fromUtf8("actionProfile"));
        actionProfile->setCheckable(true);
        QIcon icon18;
        icon18.addFile(QString::fromUtf8(":/resource/Profile.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionProfile->setIcon(icon18);
        actionFaceNormal = new QAction(MainWindow);
        actionFaceNormal->setObjectName(QString::fromUtf8("actionFaceNormal"));
        actionFaceNormal->setCheckable(true);
        QIcon icon19;
        icon19.addFile(QString::fromUtf8(":/resource/FaceNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionFaceNormal->setIcon(icon19);
        actionNodeNormal = new QAction(MainWindow);
        actionNodeNormal->setObjectName(QString::fromUtf8("actionNodeNormal"));
        actionNodeNormal->setCheckable(true);
        QIcon icon20;
        icon20.addFile(QString::fromUtf8(":/resource/NodeNormal.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNodeNormal->setIcon(icon20);
        actionSelectEdge = new QAction(MainWindow);
        actionSelectEdge->setObjectName(QString::fromUtf8("actionSelectEdge"));
        QIcon icon21;
        icon21.addFile(QString::fromUtf8(":/resource/selectEdge.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectEdge->setIcon(icon21);
        actionGenerate = new QAction(MainWindow);
        actionGenerate->setObjectName(QString::fromUtf8("actionGenerate"));
        actionTest_1 = new QAction(MainWindow);
        actionTest_1->setObjectName(QString::fromUtf8("actionTest_1"));
        actionSelectFix = new QAction(MainWindow);
        actionSelectFix->setObjectName(QString::fromUtf8("actionSelectFix"));
        QIcon icon22;
        icon22.addFile(QString::fromUtf8(":/resource/selectFix.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectFix->setIcon(icon22);
        actionSelectHandle = new QAction(MainWindow);
        actionSelectHandle->setObjectName(QString::fromUtf8("actionSelectHandle"));
        QIcon icon23;
        icon23.addFile(QString::fromUtf8(":/resource/selectHandle.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelectHandle->setIcon(icon23);
        actionSaveSelection = new QAction(MainWindow);
        actionSaveSelection->setObjectName(QString::fromUtf8("actionSaveSelection"));
        QIcon icon24;
        icon24.addFile(QString::fromUtf8(":/resource/SaveSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionSaveSelection->setIcon(icon24);
        actionReadSelection = new QAction(MainWindow);
        actionReadSelection->setObjectName(QString::fromUtf8("actionReadSelection"));
        QIcon icon25;
        icon25.addFile(QString::fromUtf8(":/resource/InputSelection.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionReadSelection->setIcon(icon25);
        actionSelectChamber = new QAction(MainWindow);
        actionSelectChamber->setObjectName(QString::fromUtf8("actionSelectChamber"));
        actionExport_to_Abaqus_model = new QAction(MainWindow);
        actionExport_to_Abaqus_model->setObjectName(QString::fromUtf8("actionExport_to_Abaqus_model"));
        actionExport_to_Abaqus_model->setCheckable(false);
        QIcon icon26;
        icon26.addFile(QString::fromUtf8(":/resource/abaqus logo.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionExport_to_Abaqus_model->setIcon(icon26);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setMouseTracking(true);
        centralWidget->setAcceptDrops(true);
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        MainWindow->setCentralWidget(centralWidget);
        navigationToolBar = new QToolBar(MainWindow);
        navigationToolBar->setObjectName(QString::fromUtf8("navigationToolBar"));
        navigationToolBar->setMovable(false);
        navigationToolBar->setIconSize(QSize(25, 25));
        navigationToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, navigationToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);
        selectionToolBar = new QToolBar(MainWindow);
        selectionToolBar->setObjectName(QString::fromUtf8("selectionToolBar"));
        selectionToolBar->setMovable(false);
        selectionToolBar->setIconSize(QSize(25, 25));
        selectionToolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, selectionToolBar);
        dockWidget = new QDockWidget(MainWindow);
        dockWidget->setObjectName(QString::fromUtf8("dockWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(dockWidget->sizePolicy().hasHeightForWidth());
        dockWidget->setSizePolicy(sizePolicy);
        dockWidget->setMinimumSize(QSize(300, 984));
        dockWidget->setMaximumSize(QSize(300, 524287));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        dockWidgetContents->setLayoutDirection(Qt::LeftToRight);
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_MANY_3DP_CNC_CAM = new QLabel(dockWidgetContents);
        label_MANY_3DP_CNC_CAM->setObjectName(QString::fromUtf8("label_MANY_3DP_CNC_CAM"));
        QFont font1;
        font1.setPointSize(10);
        label_MANY_3DP_CNC_CAM->setFont(font1);

        verticalLayout_2->addWidget(label_MANY_3DP_CNC_CAM);

        line = new QFrame(dockWidgetContents);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label = new QLabel(dockWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_2->addWidget(label);

        doubleSpinBox_hPos = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_hPos->setObjectName(QString::fromUtf8("doubleSpinBox_hPos"));

        horizontalLayout_2->addWidget(doubleSpinBox_hPos);


        verticalLayout_2->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_2 = new QLabel(dockWidgetContents);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_3->addWidget(label_2);

        lineEdit_toolTipX = new QLineEdit(dockWidgetContents);
        lineEdit_toolTipX->setObjectName(QString::fromUtf8("lineEdit_toolTipX"));

        horizontalLayout_3->addWidget(lineEdit_toolTipX);

        lineEdit_toolTipY = new QLineEdit(dockWidgetContents);
        lineEdit_toolTipY->setObjectName(QString::fromUtf8("lineEdit_toolTipY"));

        horizontalLayout_3->addWidget(lineEdit_toolTipY);

        lineEdit_toolTipZ = new QLineEdit(dockWidgetContents);
        lineEdit_toolTipZ->setObjectName(QString::fromUtf8("lineEdit_toolTipZ"));

        horizontalLayout_3->addWidget(lineEdit_toolTipZ);


        verticalLayout_2->addLayout(horizontalLayout_3);

        pushButton_loadRob = new QPushButton(dockWidgetContents);
        pushButton_loadRob->setObjectName(QString::fromUtf8("pushButton_loadRob"));

        verticalLayout_2->addWidget(pushButton_loadRob);

        line_3 = new QFrame(dockWidgetContents);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_3);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setSpacing(6);
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        pushButton_getBaseColi = new QPushButton(dockWidgetContents);
        pushButton_getBaseColi->setObjectName(QString::fromUtf8("pushButton_getBaseColi"));

        horizontalLayout_13->addWidget(pushButton_getBaseColi);

        pushButton_trainBaseColi = new QPushButton(dockWidgetContents);
        pushButton_trainBaseColi->setObjectName(QString::fromUtf8("pushButton_trainBaseColi"));

        horizontalLayout_13->addWidget(pushButton_trainBaseColi);


        verticalLayout_2->addLayout(horizontalLayout_13);

        line_2 = new QFrame(dockWidgetContents);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_2);

        horizontalLayout_16 = new QHBoxLayout();
        horizontalLayout_16->setSpacing(6);
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        label_PosNorFile = new QLabel(dockWidgetContents);
        label_PosNorFile->setObjectName(QString::fromUtf8("label_PosNorFile"));
        QFont font2;
        font2.setPointSize(8);
        font2.setBold(true);
        font2.setWeight(75);
        label_PosNorFile->setFont(font2);

        horizontalLayout_16->addWidget(label_PosNorFile);

        lineEdit_SorceDataDir = new QLineEdit(dockWidgetContents);
        lineEdit_SorceDataDir->setObjectName(QString::fromUtf8("lineEdit_SorceDataDir"));

        horizontalLayout_16->addWidget(lineEdit_SorceDataDir);


        verticalLayout_2->addLayout(horizontalLayout_16);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_3 = new QLabel(dockWidgetContents);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_4->addWidget(label_3);

        spinBox_setLayer = new QSpinBox(dockWidgetContents);
        spinBox_setLayer->setObjectName(QString::fromUtf8("spinBox_setLayer"));

        horizontalLayout_4->addWidget(spinBox_setLayer);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_2);

        checkBox_checkCollision = new QCheckBox(dockWidgetContents);
        checkBox_checkCollision->setObjectName(QString::fromUtf8("checkBox_checkCollision"));

        horizontalLayout_4->addWidget(checkBox_checkCollision);


        verticalLayout_2->addLayout(horizontalLayout_4);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setSpacing(6);
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        label_15 = new QLabel(dockWidgetContents);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        horizontalLayout_14->addWidget(label_15);

        doubleSpinBox_sepPath = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_sepPath->setObjectName(QString::fromUtf8("doubleSpinBox_sepPath"));

        horizontalLayout_14->addWidget(doubleSpinBox_sepPath);


        verticalLayout_2->addLayout(horizontalLayout_14);

        pushButton_inputPathLayer = new QPushButton(dockWidgetContents);
        pushButton_inputPathLayer->setObjectName(QString::fromUtf8("pushButton_inputPathLayer"));

        verticalLayout_2->addWidget(pushButton_inputPathLayer);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_5 = new QLabel(dockWidgetContents);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_5->addWidget(label_5);

        lcdNumber_pathNumber = new QLCDNumber(dockWidgetContents);
        lcdNumber_pathNumber->setObjectName(QString::fromUtf8("lcdNumber_pathNumber"));

        horizontalLayout_5->addWidget(lcdNumber_pathNumber);

        label_4 = new QLabel(dockWidgetContents);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_5->addWidget(label_4);

        spinBox_pathIndex = new QSpinBox(dockWidgetContents);
        spinBox_pathIndex->setObjectName(QString::fromUtf8("spinBox_pathIndex"));

        horizontalLayout_5->addWidget(spinBox_pathIndex);


        verticalLayout_2->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        checkBox_optTime = new QCheckBox(dockWidgetContents);
        checkBox_optTime->setObjectName(QString::fromUtf8("checkBox_optTime"));

        horizontalLayout_6->addWidget(checkBox_optTime);

        checkBox_inputTime = new QCheckBox(dockWidgetContents);
        checkBox_inputTime->setObjectName(QString::fromUtf8("checkBox_inputTime"));

        horizontalLayout_6->addWidget(checkBox_inputTime);


        verticalLayout_2->addLayout(horizontalLayout_6);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_6 = new QLabel(dockWidgetContents);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_7->addWidget(label_6);

        doubleSpinBox_vTip = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_vTip->setObjectName(QString::fromUtf8("doubleSpinBox_vTip"));

        horizontalLayout_7->addWidget(doubleSpinBox_vTip);

        label_7 = new QLabel(dockWidgetContents);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        horizontalLayout_7->addWidget(label_7);

        lineEdit_consVt = new QLineEdit(dockWidgetContents);
        lineEdit_consVt->setObjectName(QString::fromUtf8("lineEdit_consVt"));

        horizontalLayout_7->addWidget(lineEdit_consVt);

        lineEdit_consAt = new QLineEdit(dockWidgetContents);
        lineEdit_consAt->setObjectName(QString::fromUtf8("lineEdit_consAt"));

        horizontalLayout_7->addWidget(lineEdit_consAt);

        lineEdit_consAn = new QLineEdit(dockWidgetContents);
        lineEdit_consAn->setObjectName(QString::fromUtf8("lineEdit_consAn"));

        horizontalLayout_7->addWidget(lineEdit_consAn);


        verticalLayout_2->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        checkBox_enhancePos = new QCheckBox(dockWidgetContents);
        checkBox_enhancePos->setObjectName(QString::fromUtf8("checkBox_enhancePos"));

        horizontalLayout_8->addWidget(checkBox_enhancePos);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer);

        label_8 = new QLabel(dockWidgetContents);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_8->addWidget(label_8);

        doubleSpinBox_enhancePos = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_enhancePos->setObjectName(QString::fromUtf8("doubleSpinBox_enhancePos"));

        horizontalLayout_8->addWidget(doubleSpinBox_enhancePos);


        verticalLayout_2->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_9 = new QLabel(dockWidgetContents);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        horizontalLayout_9->addWidget(label_9);

        doubleSpinBox_alpha = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_alpha->setObjectName(QString::fromUtf8("doubleSpinBox_alpha"));

        horizontalLayout_9->addWidget(doubleSpinBox_alpha);

        doubleSpinBox_beta = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_beta->setObjectName(QString::fromUtf8("doubleSpinBox_beta"));

        horizontalLayout_9->addWidget(doubleSpinBox_beta);

        lineEdit_gamma = new QLineEdit(dockWidgetContents);
        lineEdit_gamma->setObjectName(QString::fromUtf8("lineEdit_gamma"));

        horizontalLayout_9->addWidget(lineEdit_gamma);


        verticalLayout_2->addLayout(horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_10 = new QLabel(dockWidgetContents);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        horizontalLayout_10->addWidget(label_10);

        doubleSpinBox_vMaxRob = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_vMaxRob->setObjectName(QString::fromUtf8("doubleSpinBox_vMaxRob"));

        horizontalLayout_10->addWidget(doubleSpinBox_vMaxRob);

        label_11 = new QLabel(dockWidgetContents);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        horizontalLayout_10->addWidget(label_11);

        doubleSpinBox_vMaxPos = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_vMaxPos->setObjectName(QString::fromUtf8("doubleSpinBox_vMaxPos"));

        horizontalLayout_10->addWidget(doubleSpinBox_vMaxPos);


        verticalLayout_2->addLayout(horizontalLayout_10);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_12 = new QLabel(dockWidgetContents);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        horizontalLayout_11->addWidget(label_12);

        doubleSpinBox_jMaxRob = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_jMaxRob->setObjectName(QString::fromUtf8("doubleSpinBox_jMaxRob"));

        horizontalLayout_11->addWidget(doubleSpinBox_jMaxRob);

        label_13 = new QLabel(dockWidgetContents);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        horizontalLayout_11->addWidget(label_13);

        doubleSpinBox_jMaxPos = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_jMaxPos->setObjectName(QString::fromUtf8("doubleSpinBox_jMaxPos"));

        horizontalLayout_11->addWidget(doubleSpinBox_jMaxPos);


        verticalLayout_2->addLayout(horizontalLayout_11);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        label_14 = new QLabel(dockWidgetContents);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        horizontalLayout_12->addWidget(label_14);

        doubleSpinBox_kv = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_kv->setObjectName(QString::fromUtf8("doubleSpinBox_kv"));

        horizontalLayout_12->addWidget(doubleSpinBox_kv);

        doubleSpinBox_ka = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_ka->setObjectName(QString::fromUtf8("doubleSpinBox_ka"));

        horizontalLayout_12->addWidget(doubleSpinBox_ka);

        doubleSpinBox_kj = new QDoubleSpinBox(dockWidgetContents);
        doubleSpinBox_kj->setObjectName(QString::fromUtf8("doubleSpinBox_kj"));

        horizontalLayout_12->addWidget(doubleSpinBox_kj);

        lineEdit_kt = new QLineEdit(dockWidgetContents);
        lineEdit_kt->setObjectName(QString::fromUtf8("lineEdit_kt"));

        horizontalLayout_12->addWidget(lineEdit_kt);


        verticalLayout_2->addLayout(horizontalLayout_12);

        pushButton_iniOpt = new QPushButton(dockWidgetContents);
        pushButton_iniOpt->setObjectName(QString::fromUtf8("pushButton_iniOpt"));

        verticalLayout_2->addWidget(pushButton_iniOpt);

        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setSpacing(6);
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        checkBox_showDetail = new QCheckBox(dockWidgetContents);
        checkBox_showDetail->setObjectName(QString::fromUtf8("checkBox_showDetail"));

        horizontalLayout_15->addWidget(checkBox_showDetail);


        verticalLayout_2->addLayout(horizontalLayout_15);

        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setSpacing(6);
        horizontalLayout_17->setObjectName(QString::fromUtf8("horizontalLayout_17"));

        verticalLayout_2->addLayout(horizontalLayout_17);

        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setSpacing(6);
        horizontalLayout_18->setObjectName(QString::fromUtf8("horizontalLayout_18"));
        label_16 = new QLabel(dockWidgetContents);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        horizontalLayout_18->addWidget(label_16);

        lineEdit_sizeWindow = new QLineEdit(dockWidgetContents);
        lineEdit_sizeWindow->setObjectName(QString::fromUtf8("lineEdit_sizeWindow"));

        horizontalLayout_18->addWidget(lineEdit_sizeWindow);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_18->addItem(horizontalSpacer_4);

        pushButton_Opt = new QPushButton(dockWidgetContents);
        pushButton_Opt->setObjectName(QString::fromUtf8("pushButton_Opt"));

        horizontalLayout_18->addWidget(pushButton_Opt);


        verticalLayout_2->addLayout(horizontalLayout_18);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        treeView = new QTreeView(dockWidgetContents);
        treeView->setObjectName(QString::fromUtf8("treeView"));
        treeView->setEnabled(true);
        treeView->setProperty("showDropIndicator", QVariant(true));
        treeView->setIndentation(5);
        treeView->header()->setVisible(false);

        verticalLayout_2->addWidget(treeView);

        pushButton_clearAll = new QPushButton(dockWidgetContents);
        pushButton_clearAll->setObjectName(QString::fromUtf8("pushButton_clearAll"));

        verticalLayout_2->addWidget(pushButton_clearAll);

        dockWidget->setWidget(dockWidgetContents);
        MainWindow->addDockWidget(Qt::RightDockWidgetArea, dockWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1331, 22));
        menuBar->setLayoutDirection(Qt::LeftToRight);
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        menuSelect = new QMenu(menuBar);
        menuSelect->setObjectName(QString::fromUtf8("menuSelect"));
        MainWindow->setMenuBar(menuBar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        toolBar->setMovable(false);
        toolBar->setFloatable(false);
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);

        navigationToolBar->addAction(actionFront);
        navigationToolBar->addAction(actionBack);
        navigationToolBar->addAction(actionTop);
        navigationToolBar->addAction(actionBottom);
        navigationToolBar->addAction(actionLeft);
        navigationToolBar->addAction(actionRight);
        navigationToolBar->addAction(actionIsometric);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionZoom_In);
        navigationToolBar->addAction(actionZoom_Out);
        navigationToolBar->addAction(actionZoom_All);
        navigationToolBar->addAction(actionZoom_Window);
        navigationToolBar->addSeparator();
        navigationToolBar->addAction(actionShade);
        navigationToolBar->addAction(actionMesh);
        navigationToolBar->addAction(actionNode);
        navigationToolBar->addAction(actionProfile);
        navigationToolBar->addAction(actionFaceNormal);
        navigationToolBar->addAction(actionNodeNormal);
        selectionToolBar->addAction(actionSaveSelection);
        selectionToolBar->addAction(actionReadSelection);
        selectionToolBar->addSeparator();
        selectionToolBar->addAction(actionSelectNode);
        selectionToolBar->addAction(actionSelectEdge);
        selectionToolBar->addAction(actionSelectFace);
        selectionToolBar->addAction(actionSelectFix);
        selectionToolBar->addAction(actionSelectHandle);
        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuBar->addAction(menuSelect->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuFile->addAction(actionSaveSelection);
        menuFile->addAction(actionReadSelection);
        menuView->addAction(actionFront);
        menuView->addAction(actionBack);
        menuView->addAction(actionTop);
        menuView->addAction(actionBottom);
        menuView->addAction(actionLeft);
        menuView->addAction(actionRight);
        menuView->addAction(actionIsometric);
        menuView->addSeparator();
        menuView->addAction(actionZoom_In);
        menuView->addAction(actionZoom_Out);
        menuView->addAction(actionZoom_All);
        menuView->addAction(actionZoom_Window);
        menuView->addSeparator();
        menuView->addAction(actionShade);
        menuView->addAction(actionMesh);
        menuView->addAction(actionNode);
        menuView->addAction(actionProfile);
        menuView->addSeparator();
        menuView->addAction(actionShifttoOrigin);
        menuSelect->addAction(actionSelectNode);
        menuSelect->addAction(actionSelectEdge);
        menuSelect->addAction(actionSelectFace);
        menuSelect->addSeparator();
        menuSelect->addAction(actionSelectFix);
        menuSelect->addAction(actionSelectHandle);
        menuSelect->addSeparator();
        toolBar->addAction(actionOpen);
        toolBar->addAction(actionSave);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        actionOpen->setText(QCoreApplication::translate("MainWindow", "Open", nullptr));
        actionFront->setText(QCoreApplication::translate("MainWindow", "Front", nullptr));
        actionBack->setText(QCoreApplication::translate("MainWindow", "Back", nullptr));
        actionTop->setText(QCoreApplication::translate("MainWindow", "Top", nullptr));
        actionBottom->setText(QCoreApplication::translate("MainWindow", "Bottom", nullptr));
        actionLeft->setText(QCoreApplication::translate("MainWindow", "Left", nullptr));
        actionRight->setText(QCoreApplication::translate("MainWindow", "Right", nullptr));
        actionIsometric->setText(QCoreApplication::translate("MainWindow", "Isometric", nullptr));
        actionZoom_In->setText(QCoreApplication::translate("MainWindow", "Zoom In", nullptr));
        actionZoom_Out->setText(QCoreApplication::translate("MainWindow", "Zoom Out", nullptr));
        actionZoom_All->setText(QCoreApplication::translate("MainWindow", "Zoom All", nullptr));
        actionZoom_Window->setText(QCoreApplication::translate("MainWindow", "Zoom Window", nullptr));
        actionShade->setText(QCoreApplication::translate("MainWindow", "Shade", nullptr));
        actionMesh->setText(QCoreApplication::translate("MainWindow", "Mesh", nullptr));
        actionNode->setText(QCoreApplication::translate("MainWindow", "Node", nullptr));
        actionSave->setText(QCoreApplication::translate("MainWindow", "Save", nullptr));
        actionSelectNode->setText(QCoreApplication::translate("MainWindow", "Node", nullptr));
        actionSelectFace->setText(QCoreApplication::translate("MainWindow", "Face", nullptr));
        actionShifttoOrigin->setText(QCoreApplication::translate("MainWindow", "Shift to Origin", nullptr));
        actionProfile->setText(QCoreApplication::translate("MainWindow", "Profile", nullptr));
        actionFaceNormal->setText(QCoreApplication::translate("MainWindow", "FaceNormal", nullptr));
        actionNodeNormal->setText(QCoreApplication::translate("MainWindow", "NodeNormal", nullptr));
        actionSelectEdge->setText(QCoreApplication::translate("MainWindow", "Edge", nullptr));
        actionGenerate->setText(QCoreApplication::translate("MainWindow", "Generate", nullptr));
        actionTest_1->setText(QCoreApplication::translate("MainWindow", "Test_1", nullptr));
        actionSelectFix->setText(QCoreApplication::translate("MainWindow", "Fix", nullptr));
        actionSelectHandle->setText(QCoreApplication::translate("MainWindow", "Handle & Rigid", nullptr));
        actionSaveSelection->setText(QCoreApplication::translate("MainWindow", "Save selection", nullptr));
        actionReadSelection->setText(QCoreApplication::translate("MainWindow", "Read selection", nullptr));
        actionSelectChamber->setText(QCoreApplication::translate("MainWindow", "Select Chamber (SORO)", nullptr));
        actionExport_to_Abaqus_model->setText(QCoreApplication::translate("MainWindow", "Export to Abaqus model", nullptr));
        navigationToolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "navigationToolBar", nullptr));
        selectionToolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "selectionToolBar", nullptr));
        label_MANY_3DP_CNC_CAM->setText(QCoreApplication::translate("MainWindow", "Trajectory Optimisation", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "hPos:", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "toolTip:", nullptr));
        pushButton_loadRob->setText(QCoreApplication::translate("MainWindow", "Load Robots", nullptr));
        pushButton_getBaseColi->setText(QCoreApplication::translate("MainWindow", "Get Base Collision Data", nullptr));
        pushButton_trainBaseColi->setText(QCoreApplication::translate("MainWindow", "Train Base Collision Proxy", nullptr));
        label_PosNorFile->setText(QCoreApplication::translate("MainWindow", "File Dir:", nullptr));
        lineEdit_SorceDataDir->setText(QCoreApplication::translate("MainWindow", "test", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Layer:", nullptr));
        checkBox_checkCollision->setText(QCoreApplication::translate("MainWindow", "checkCollision", nullptr));
        label_15->setText(QCoreApplication::translate("MainWindow", "maxDist in a single path:", nullptr));
        pushButton_inputPathLayer->setText(QCoreApplication::translate("MainWindow", "inputPathAndLayer", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "PathNumber:", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "PathIndex", nullptr));
        checkBox_optTime->setText(QCoreApplication::translate("MainWindow", "OptTime", nullptr));
        checkBox_inputTime->setText(QCoreApplication::translate("MainWindow", "inputTime", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "vTip:", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "consTip:", nullptr));
        checkBox_enhancePos->setText(QCoreApplication::translate("MainWindow", "enhancePos", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "enhanceValue:", nullptr));
        label_9->setText(QCoreApplication::translate("MainWindow", "angleCons:", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "vMaxRob:", nullptr));
        label_11->setText(QCoreApplication::translate("MainWindow", "vMaxPos:", nullptr));
        label_12->setText(QCoreApplication::translate("MainWindow", "jMaxRob:", nullptr));
        label_13->setText(QCoreApplication::translate("MainWindow", "jMaxPos:", nullptr));
        label_14->setText(QCoreApplication::translate("MainWindow", "k:v,a,j,t", nullptr));
        pushButton_iniOpt->setText(QCoreApplication::translate("MainWindow", "Initialize Optimization", nullptr));
        checkBox_showDetail->setText(QCoreApplication::translate("MainWindow", "ShowDetails", nullptr));
        label_16->setText(QCoreApplication::translate("MainWindow", "sizeWindow:", nullptr));
        pushButton_Opt->setText(QCoreApplication::translate("MainWindow", "Optimization", nullptr));
        pushButton_clearAll->setText(QCoreApplication::translate("MainWindow", "Clear All", nullptr));
        menuFile->setTitle(QCoreApplication::translate("MainWindow", "File", nullptr));
        menuView->setTitle(QCoreApplication::translate("MainWindow", "View", nullptr));
        menuSelect->setTitle(QCoreApplication::translate("MainWindow", "Select", nullptr));
        toolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
