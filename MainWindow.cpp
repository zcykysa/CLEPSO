#include "MainWindow.h"
#include "QPushButton.h"
#include "QFileDialog.h"
#include "vtkSTLReader.h"
#include "vtkTriangleFilter.h"
#include "vtkPolyDataConnectivityFilter.h"
#include "QFile.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderWindow.h"
#include "vtkCamera.h"
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkSeedRepresentation.h>
#include <vtkSeedWidget.h>
#include <vtkProperty2D.h>
#include <vtkCallbackCommand.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3DMapper.h>
#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkCellArray.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyDataWriter.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkTubeFilter.h"
#include "vtkParametricSpline.h"
#include "vtkParametricFunctionSource.h"
#include <vtkSphereSource.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkOBBTree.h>
#include <vtkNamedColors.h>
#include <vtkOutlineFilter.h>
#include "OrthoData.h"
#include <GroupModelObservation.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include "arch.h"
#include "Gum.h"
#include "Tooth.h"
#include <vtkAutoInit.h>
#include <new.h>
#include <GroupManager.h>
#include <CVtkInteractorStyle.h>
#include <GroupContrast.h>
#include <RendererContainer.h>
#include <GroupMoveCalculation.h>
#include <GroupOcclusion.h>
#include <GroupPathPlanningPSO.h>
#include <GroupGridLine.h>
#include <QColor>
#include <QPalette>
#include <QPointF>
#include <QPainter>
#include <QPaintEvent>
#include <QVTKOpenGLNativeWidget.h>
#include <MyGLWidget.h>

//#include <vtkAxesActor.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
//VTK_MODULE_INIT(vtkRenderingFreeType);




MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	

	Init();


	connect(ui.pushButton, &QPushButton::clicked, this, &MainWindow::LoadData);
	connect(ui.Hide_UpperTeeth, &QPushButton::clicked, this, &MainWindow::M_HideUpperTeeth); //隐藏上颌
	connect(ui.UpperTeeth, &QPushButton::clicked, this, &MainWindow::M_ViewUpperTeeth); //上颌
	connect(ui.UpperSurface, &QPushButton::clicked, this, &MainWindow::M_ViewUpperSurface); //上颌颌面
	connect(ui.Left, &QPushButton::clicked, this, &MainWindow::M_ViewLeft); //左侧
	connect(ui.Right, &QPushButton::clicked, this, &MainWindow::M_ViewRight); //右侧
	connect(ui.Front, &QPushButton::clicked, this, &MainWindow::M_ViewFront); //正面
	connect(ui.Back, &QPushButton::clicked, this, &MainWindow::M_ViewBack); //后视
	connect(ui.LowerSurface, &QPushButton::clicked, this, &MainWindow::M_ViewLowerSurface); //下颌颌面
	connect(ui.LowerTeeth, &QPushButton::clicked, this, &MainWindow::M_ViewLowerTeeth); //下颌
	connect(ui.Hide_LowerTeeth, &QPushButton::clicked, this, &MainWindow::M_HideLowerTeeth); //隐藏下颌
	connect(ui.FDI, &QPushButton::clicked, this, &MainWindow::MFDI_Mark); //FDI牙位标记
	connect(ui.Contrast, &QPushButton::clicked, this, &MainWindow::OnContrast); //对照
	connect(ui.Movement, &QPushButton::clicked, this, &MainWindow::OnMoveCalculation); //平移量计算
	//connect(ui.Animation, &QPushButton::clicked, this, &MainWindow::Animation); //
	connect(ui.Occlusion, &QPushButton::clicked, this, &MainWindow::Occlusion_Show); //咬合显示
	connect(ui.IPSO_SA_Ours, &QPushButton::clicked, this, &MainWindow::Path_IPSOSA_Ours); //路径规划——粒子群算法
	connect(ui.PSO_phase1, &QPushButton::clicked, this, &MainWindow::PSO_Phase1);//可视化阶段1——起始点
	connect(ui.PSO_phase3, &QPushButton::clicked, this, &MainWindow::PSO_Phase3);
	connect(ui.PSO_phase5, &QPushButton::clicked, this, &MainWindow::PSO_Phase5);
	connect(ui.PSO_phase7, &QPushButton::clicked, this, &MainWindow::PSO_Phase7);
	connect(ui.PSO_phase9, &QPushButton::clicked, this, &MainWindow::PSO_Phase9);
	connect(ui.PSO_phase11, &QPushButton::clicked, this, &MainWindow::PSO_Phase11);
	connect(ui.PSO_phase13, &QPushButton::clicked, this, &MainWindow::PSO_Phase13);
	connect(ui.PSO_phase15, &QPushButton::clicked, this, &MainWindow::PSO_Phase15);
	connect(ui.PSO_phase17, &QPushButton::clicked, this, &MainWindow::PSO_Phase17);
	connect(ui.PSO_phase19, &QPushButton::clicked, this, &MainWindow::PSO_Phase19);
	connect(ui.PSO_phase21, &QPushButton::clicked, this, &MainWindow::PSO_Phase21);//路径21——终点
	connect(ui.PSO_phase2, &QPushButton::clicked, this, &MainWindow::PSO_Phase2);
	connect(ui.PSO_phase4, &QPushButton::clicked, this, &MainWindow::PSO_Phase4);
	connect(ui.PSO_phase6, &QPushButton::clicked, this, &MainWindow::PSO_Phase6);
	connect(ui.PSO_phase8, &QPushButton::clicked, this, &MainWindow::PSO_Phase8);
	connect(ui.PSO_phase10, &QPushButton::clicked, this, &MainWindow::PSO_Phase10);
	connect(ui.PSO_phase12, &QPushButton::clicked, this, &MainWindow::PSO_Phase12);
	connect(ui.PSO_phase14, &QPushButton::clicked, this, &MainWindow::PSO_Phase14);
	connect(ui.PSO_phase16, &QPushButton::clicked, this, &MainWindow::PSO_Phase16);
	connect(ui.PSO_phase18, &QPushButton::clicked, this, &MainWindow::PSO_Phase18);
	connect(ui.PSO_phase20, &QPushButton::clicked, this, &MainWindow::PSO_Phase20);
	connect(ui.PSO_phase22, &QPushButton::clicked, this, &MainWindow::PSO_Phase22);
	connect(ui.PSO_phase23, &QPushButton::clicked, this, &MainWindow::PSO_Phase23);
	connect(ui.PSO_phase24, &QPushButton::clicked, this, &MainWindow::PSO_Phase24);
	connect(ui.PSO_phase25, &QPushButton::clicked, this, &MainWindow::PSO_Phase25);



	connect(ui.PSO, &QPushButton::clicked, this, &MainWindow::Path_PSO);
	connect(ui.NSMPSO_Xu, &QPushButton::clicked, this, &MainWindow::Path_NSMPSO_Xu);
	connect(ui.IPSO_SA, &QPushButton::clicked, this, &MainWindow::Path_IPSOSA);
	connect(ui.CSPSO, &QPushButton::clicked, this, &MainWindow::Path_CSPSO);
	connect(ui.MOPSO, &QPushButton::clicked, this, &MainWindow::Path_MOPSO);
	connect(ui.MDC_DPSO, &QPushButton::clicked, this, &MainWindow::Path_MDC_DPSO);
	connect(ui.MCMOPSO, &QPushButton::clicked, this, &MainWindow::Path_MCMOPSO);


	connect(ui.GridLine, &QPushButton::clicked, this, &MainWindow::GridLine_Show); //网格显示

}

MainWindow::~MainWindow()
{
	cout << "主窗口析构" << endl;
}


void MainWindow::InitGroup()
{
	GroupBase* observe = new GroupModelObservation(Group_ModelObservation); //模型观察功能块
	m_GroupManager->AddGroup(observe);

	GroupBase* contrast = new GroupContrast(Group_Contrast);//对照功能
	m_GroupManager->AddGroup(contrast);

	GroupBase* fdiMark = new GroupFdiMark(Group_FdiMark);  //牙位标记功能
	m_GroupManager->AddGroup(fdiMark);

	GroupBase* movecalculation = new GroupMoveCalculation(Group_MoveCalculation);//平移量计算功能
	m_GroupManager->AddGroup(movecalculation);

	GroupBase* occlusion_Show = new GroupOcclusion(Group_Occlusion);
	m_GroupManager->AddGroup(occlusion_Show);

	
	GroupBase* pathPSO = new GroupPathPlanningPSO(Group_PathPSO);
	m_GroupManager->AddGroup(pathPSO);
	GroupBase* phase1 = new GroupPathPlanningPSO(Group_Phase1);
	m_GroupManager->AddGroup(phase1);
	GroupBase* phase3 = new GroupPathPlanningPSO(Group_Phase3);
	m_GroupManager->AddGroup(phase3);
	GroupBase* phase5 = new GroupPathPlanningPSO(Group_Phase5);
	m_GroupManager->AddGroup(phase5);
	GroupBase* phase7 = new GroupPathPlanningPSO(Group_Phase7);
	m_GroupManager->AddGroup(phase7);
	GroupBase* phase9 = new GroupPathPlanningPSO(Group_Phase9);
	m_GroupManager->AddGroup(phase9);
	GroupBase* phase11 = new GroupPathPlanningPSO(Group_Phase11);
	m_GroupManager->AddGroup(phase11);
	GroupBase* phase13 = new GroupPathPlanningPSO(Group_Phase13);
	m_GroupManager->AddGroup(phase13);
	GroupBase* phase15 = new GroupPathPlanningPSO(Group_Phase15);
	m_GroupManager->AddGroup(phase15);
	GroupBase* phase17 = new GroupPathPlanningPSO(Group_Phase17);
	m_GroupManager->AddGroup(phase17);
	GroupBase* phase19 = new GroupPathPlanningPSO(Group_Phase19);
	m_GroupManager->AddGroup(phase19);
	GroupBase* phase21 = new GroupPathPlanningPSO(Group_Phase21);
	m_GroupManager->AddGroup(phase21);
	GroupBase* phase2 = new GroupPathPlanningPSO(Group_Phase2);
	m_GroupManager->AddGroup(phase2);
	GroupBase* phase4 = new GroupPathPlanningPSO(Group_Phase4);
	m_GroupManager->AddGroup(phase4);
	GroupBase* phase6 = new GroupPathPlanningPSO(Group_Phase6);
	m_GroupManager->AddGroup(phase6);
	GroupBase* phase8 = new GroupPathPlanningPSO(Group_Phase8);
	m_GroupManager->AddGroup(phase8);
	GroupBase* phase10 = new GroupPathPlanningPSO(Group_Phase10);
	m_GroupManager->AddGroup(phase10);
	GroupBase* phase12 = new GroupPathPlanningPSO(Group_Phase12);
	m_GroupManager->AddGroup(phase12);
	GroupBase* phase14 = new GroupPathPlanningPSO(Group_Phase14);
	m_GroupManager->AddGroup(phase14);
	GroupBase* phase16 = new GroupPathPlanningPSO(Group_Phase16);
	m_GroupManager->AddGroup(phase16);
	GroupBase* phase18 = new GroupPathPlanningPSO(Group_Phase18);
	m_GroupManager->AddGroup(phase18);
	GroupBase* phase20 = new GroupPathPlanningPSO(Group_Phase20);
	m_GroupManager->AddGroup(phase20);
	GroupBase* phase22 = new GroupPathPlanningPSO(Group_Phase22);
	m_GroupManager->AddGroup(phase22);
	GroupBase* phase23 = new GroupPathPlanningPSO(Group_Phase23);
	m_GroupManager->AddGroup(phase23);
	GroupBase* phase24 = new GroupPathPlanningPSO(Group_Phase24);
	m_GroupManager->AddGroup(phase24);
	GroupBase* phase25 = new GroupPathPlanningPSO(Group_Phase25);
	m_GroupManager->AddGroup(phase25);
	



	GroupBase* gridline = new GroupGridLine(Group_GridLine);
	m_GroupManager->AddGroup(gridline);
	


}

void MainWindow::Init()
{

	m_GroupManager = GroupManager::GetInstance();

	//QSharedPointer的使用方式
	my_OrthoData = QSharedPointer<OrthoData>(new OrthoData());
	m_GroupManager->SetOrthoData(my_OrthoData.get());

	

	//实例化类后，自动实现了对应构造函数
	m_RendererContainer = QSharedPointer<RendererContainer>(new RendererContainer(this->ui.openGLWidget->renderWindow()));
	m_GroupManager->SetRenderContainer(m_RendererContainer.get());


	InitGroup();

	SetThreeCoordinate();  //设置世界坐标系(左下角)
}

void MainWindow::LoadData()
{
	QFileDialog* dir = new QFileDialog(this);
	QString fileName = dir->getExistingDirectory(this, "请选择数据...", "D://");

	if (fileName.size() == 0)
		return;
	int n = fileName.lastIndexOf('/');
	QString caseName = fileName.right(fileName.size() - n - 1);
	QString path = fileName + "/";
	my_OrthoData->LoadData(path.toStdString());  //传入牙颌数据路径 (这里已经设置好了所有的牙齿牙龈对象)
	my_OrthoData->AddActors(m_RendererContainer->GetRenderer()); //添加牙颌对象 (这里也就是将牙齿牙龈取出来)
	m_RendererContainer->Render();
}

void MainWindow::WidgetsOn() {

}

void MainWindow::view1() {
}

void MainWindow::curve() {
}
void MainWindow::OBB() {
}


void MainWindow::M_HideLowerTeeth() {  //隐藏下颌

	GroupModelObservation* observe = (GroupModelObservation*)m_GroupManager->GetGroup(Group_ModelObservation);
	observe->HideLowerTeeth();
}

void MainWindow::M_ShowLowerTeeth() {  //展示下颌

	GroupModelObservation* observe = (GroupModelObservation*)m_GroupManager->GetGroup(Group_ModelObservation);
	observe->ShowLowerTeeth();
}

void MainWindow::M_HideUpperTeeth() {  //隐藏上颌

	GroupModelObservation* observe = (GroupModelObservation*)m_GroupManager->GetGroup(Group_ModelObservation);
	observe->HideUpperTeeth();
}

void MainWindow::M_ShowUpperTeeth() {  //展示上颌

	GroupModelObservation* observe = (GroupModelObservation*)m_GroupManager->GetGroup(Group_ModelObservation);
	observe->ShowUpperTeeth();
}


void MainWindow::M_ViewUpperTeeth() { //上颌
	M_ShowUpperTeeth();
}

void MainWindow::M_ViewUpperSurface() { //上颌颌面
	GroupModelObservation* observe = (GroupModelObservation*)m_GroupManager->GetGroup(Group_ModelObservation);
	observe->ViewUpperSurface();
	M_HideLowerTeeth(); //隐藏下颌
	M_ShowUpperTeeth();
}

void MainWindow::M_ViewLeft() { //左侧
	GroupModelObservation* observe = (GroupModelObservation*)m_GroupManager->GetGroup(Group_ModelObservation);
	observe->ViewLeft();
}

void MainWindow::M_ViewRight() {  //右侧
	GroupModelObservation* observe = (GroupModelObservation*)m_GroupManager->GetGroup(Group_ModelObservation);
	observe->ViewRight();
}

void MainWindow::M_ViewBack() {  //后视
	GroupModelObservation* observe = (GroupModelObservation*)m_GroupManager->GetGroup(Group_ModelObservation);
	observe->ViewBack();
}

void MainWindow::M_ViewFront() {  //正面
	GroupModelObservation* observe = (GroupModelObservation*)m_GroupManager->GetGroup(Group_ModelObservation);
	observe->ViewFront();
}

void MainWindow::M_ViewLowerSurface() {  //下颌颌面
	GroupModelObservation* observe = (GroupModelObservation*)m_GroupManager->GetGroup(Group_ModelObservation);
	observe->ViewLowerSurface();
	M_HideUpperTeeth();
	M_ShowLowerTeeth();
}

void MainWindow::M_ViewLowerTeeth() {   //下颌
	M_ShowLowerTeeth();
}



void MainWindow::MFDI_Mark() { //FDI牙位标记

	GroupFdiMark* fMark = (GroupFdiMark*)m_GroupManager->GetGroup(Group_FdiMark);
	fMark->FdiMark();
	if (fMark->GetIsGroupOpen()) {
		fMark->CloseGroup();
	}
	else {
		fMark->OpenGroup();
	}
}

void MainWindow::SetThreeCoordinate() {  //设置左下角的世界坐标系
	vtkAxesActor* iconActor = vtkAxesActor::New();
	vtkOrientationMarkerWidget* orientationWidget = vtkOrientationMarkerWidget::New();
	orientationWidget->SetOutlineColor(1, 1, 1);//(0.93, 0.57, 0.13);
	orientationWidget->SetOrientationMarker(iconActor);
	orientationWidget->SetInteractor(m_RendererContainer->GetRenWinInteractor());
	orientationWidget->SetEnabled(1); //坐标系是否可见
	orientationWidget->InteractiveOff(); //坐标系是否可移动
}

void MainWindow::OnContrast() {//对照
	GroupContrast* contrast = (GroupContrast*)m_GroupManager->GetGroup(Group_Contrast);
	contrast->Tooth_Contrast();
	if (contrast->GetIsGroupOpen()) {
		contrast->CloseGroup();
	}
	else {
		contrast->OpenGroup();
	}
}

void MainWindow::OnMoveCalculation() {//平移量计算
	GroupMoveCalculation* movecalculation = (GroupMoveCalculation*)m_GroupManager->GetGroup(Group_MoveCalculation);
	movecalculation->MoveCalculation();
	if (movecalculation->GetIsGroupOpen()) {
		movecalculation->CloseGroup();
	}
	else {
		movecalculation->OpenGroup();
	}
}

//咬合显示
//按照牙齿FDI编号，1区牙齿主要和4区牙齿产生碰撞，2区牙齿主要和3区牙齿产生碰撞
void MainWindow::Occlusion_Show()
{
	GroupOcclusion* occlusion_Show = (GroupOcclusion*)m_GroupManager->GetGroup(Group_Occlusion);

	//定义vector容器 通过调用Get_TeethId() 存放上颌或者下颌的牙齿编号
	std::vector<ToothId>list = my_OrthoData->Get_TeethId(Arch_Upper);
	std::vector<ToothId>list1 = my_OrthoData->Get_TeethId(Arch_Lower);

	//首先初始化，否则无法推导auto类型
	auto it = Tooth_NULL;
	ToothId Tooth1;
	ToothId Tooth2;
	//循环去读取上颌中的牙齿编号
	for (auto it : list)
	{
		//先取上颌的1颗牙齿的牙齿编号作为待检测的对象之一
		Tooth1 = it;
		//如果这颗牙齿编号对应的牙齿为空的话跳过，继续取下一颗牙齿
		if (my_OrthoData->GetArch(Arch_Upper)->Get_Tooth(Tooth1) == NULL) continue;

		//在下颌数据中取上颌的这颗牙齿对应的牙齿编号
		Tooth2 = list1[it - 1];

		//如果取到牙齿编号对应的牙齿不为空，则对这两颗牙齿进行碰撞检测
		if (my_OrthoData->GetArch(Arch_Lower)->Get_Tooth(Tooth2) != NULL)
		{

			Tooth* T1 = my_OrthoData->GetArch(Arch_Upper)->Get_Tooth(Tooth1);
			Tooth* T2 = my_OrthoData->GetArch(Arch_Lower)->Get_Tooth(Tooth2);
			occlusion_Show->Collide(T1, T2);

		}
		//重置Tooth2为Tooth_NULL
		Tooth2 = Tooth_NULL;
		
		//在下颌数据中取上颌的这颗牙齿对应的牙齿的左边牙齿编号
		Tooth2 = list1[it - 2];

		//如果取到牙齿编号对应的牙齿不为空，则对上下颌的这两颗牙齿进行碰撞检测
		if (my_OrthoData->GetArch(Arch_Lower)->Get_Tooth(Tooth2) != NULL)

		{
			Tooth* T1 = my_OrthoData->GetArch(Arch_Upper)->Get_Tooth(Tooth1);
			Tooth* T2 = my_OrthoData->GetArch(Arch_Lower)->Get_Tooth(Tooth2);
			occlusion_Show->Collide(T1, T2);
		}
		//重置Tooth2为Tooth_NULL
		Tooth2 = Tooth_NULL;

		//在下颌数据中取上颌的这颗牙齿对应的牙齿的右边牙齿编号
		Tooth2 = list1[it];
		//如果取到牙齿编号对应的牙齿不为空，则对上下颌的这两颗牙齿进行碰撞检测
		if (my_OrthoData->GetArch(Arch_Lower)->Get_Tooth(Tooth2) != NULL)
		{
			Tooth* T1 = my_OrthoData->GetArch(Arch_Upper)->Get_Tooth(Tooth1);
			Tooth* T2 = my_OrthoData->GetArch(Arch_Lower)->Get_Tooth(Tooth2);
			occlusion_Show->Collide(T1, T2);
		}
	}
	if (occlusion_Show->GetIsGroupOpen()) 
	{
		occlusion_Show->CloseGroup();
	}
	else 
	{
		occlusion_Show->OpenGroup();
	}
}




void MainWindow::GridLine_Show() {//网格显示
	GroupGridLine* gridline = (GroupGridLine*)m_GroupManager->GetGroup(Group_GridLine);
	gridline->OnGridLine();
	if (gridline->GetIsGroupOpen()) {
		gridline->CloseGroup();
	}
	else {
		gridline->OpenGroup();
	}
}
//void MainWindow::GridLine_Show() {//网格显示
//	//MyGLWidget* gridline2;
//	//gridline2->paintGL();
//	
//}


void MainWindow::Path_IPSOSA_Ours() {
	freopen("D://1111牙齿及目标位//实验结果3//resultdata21.txt", "w", stdout);
	GroupPathPlanningPSO* IPSOSA_Ours = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_PathPSO);
	IPSOSA_Ours->PathPlan_Huxue();
	if (IPSOSA_Ours->GetIsGroupOpen()) {
		IPSOSA_Ours->CloseGroup();
	}
	else {
		IPSOSA_Ours->OpenGroup();
	}
	//fclose(stdout);//关闭文件
}
void MainWindow::Path_PSO() {
	freopen("D://1111牙齿及目标位//实验结果2//resultdata21.txt", "w", stdout);
	GroupPathPlanningPSO* pathPSO = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_PathPSO);
	pathPSO->PathPlan_PSO();
	if (pathPSO->GetIsGroupOpen()) {
		pathPSO->CloseGroup();
	}
	else {
		pathPSO->OpenGroup();
	}
	//fclose(stdout);//关闭文件
}
void MainWindow::Path_NSMPSO_Xu() {
	freopen("D://1111牙齿及目标位//实验结果2//resultdata21.txt", "w", stdout);
	GroupPathPlanningPSO* NSMPSO_Xu= (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_PathPSO);
	NSMPSO_Xu->PathPlan_NSMPSO_Xu();
	if (NSMPSO_Xu->GetIsGroupOpen()) {
		NSMPSO_Xu->CloseGroup();
	}
	else {
		NSMPSO_Xu->OpenGroup();
	}
	//fclose(stdout);//关闭文件
}
void MainWindow::Path_IPSOSA() {
	freopen("D://1111牙齿及目标位//实验结果2//resultdata21.txt", "w", stdout);
	GroupPathPlanningPSO* IPSOSA = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_PathPSO);
	IPSOSA->PathPlan_IPSOSA();
	if (IPSOSA->GetIsGroupOpen()) {
		IPSOSA->CloseGroup();
	}
	else {
		IPSOSA->OpenGroup();
	}
	//fclose(stdout);//关闭文件
}

void MainWindow::Path_CSPSO() {
	freopen("D://1111牙齿及目标位//实验结果2//resultdata21.txt", "w", stdout);
	GroupPathPlanningPSO* CSPSO = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_PathPSO);
	CSPSO->PathPlan_CSPSO();
	if (CSPSO->GetIsGroupOpen()) {
		CSPSO->CloseGroup();
	}
	else {
		CSPSO->OpenGroup();
	}
	//fclose(stdout);//关闭文件
}
void MainWindow::Path_MOPSO() {
	//freopen("E://1111牙齿及目标位//实验结果//resultdata3_2.txt", "w", stdout);
	GroupPathPlanningPSO* MOPSO = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_PathPSO);
	MOPSO->PathPlan_MOPSO();
	if (MOPSO->GetIsGroupOpen()) {
		MOPSO->CloseGroup();
	}
	else {
		MOPSO->OpenGroup();
	}
	//fclose(stdout);//关闭文件
}
void MainWindow::Path_MDC_DPSO() {
	//freopen("E://1111牙齿及目标位//实验结果//resultdata3_2.txt", "w", stdout);
	GroupPathPlanningPSO* MDC_DPSO = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_PathPSO);
	MDC_DPSO->PathPlan_MDC_DPSO();
	if (MDC_DPSO->GetIsGroupOpen()) {
		MDC_DPSO->CloseGroup();
	}
	else {
		MDC_DPSO->OpenGroup();
	}
	//fclose(stdout);//关闭文件
}
void MainWindow::Path_MCMOPSO() {
	freopen("D://1111牙齿及目标位//实验结果2//resultdata20.txt", "w", stdout);
	GroupPathPlanningPSO* MCMOPSO = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_PathPSO);
	MCMOPSO->PathPlan_Ours();
	if (MCMOPSO->GetIsGroupOpen()) {
		MCMOPSO->CloseGroup();
	}
	else {
		MCMOPSO->OpenGroup();
	}
	//fclose(stdout);//关闭文件
}



//可视化7个阶段（6个中间路径点）
void MainWindow::PSO_Phase1() {//可视化阶段1——起始点
	GroupPathPlanningPSO* phase1 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase1);
	phase1->OrthoPhase(0);
	if (phase1->GetIsGroupOpen()) {
		phase1->CloseGroup();
	}
	else {
		phase1->OpenGroup();
	}
}
void MainWindow::PSO_Phase2() {//可视化阶段1——起始点
	GroupPathPlanningPSO* phase2 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase2);
	phase2->OrthoPhase(1);
	if (phase2->GetIsGroupOpen()) {
		phase2->CloseGroup();
	}
	else {
		phase2->OpenGroup();
	}
}
void MainWindow::PSO_Phase3() {//路径点3
	GroupPathPlanningPSO* phase3 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase3);
	phase3->OrthoPhase(2);
	if (phase3->GetIsGroupOpen()) {
		phase3->CloseGroup();
	}
	else {
		phase3->OpenGroup();
	}
}
void MainWindow::PSO_Phase4() {
	GroupPathPlanningPSO* phase4 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase4);
	phase4->OrthoPhase(3);
	if (phase4->GetIsGroupOpen()) {
		phase4->CloseGroup();
	}
	else {
		phase4->OpenGroup();
	}
}

void MainWindow::PSO_Phase5() {//路径点5
	GroupPathPlanningPSO* phase5 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase5);
	phase5->OrthoPhase(4);
	if (phase5->GetIsGroupOpen()) {
		phase5->CloseGroup();
	}
	else {
		phase5->OpenGroup();
	}
}
void MainWindow::PSO_Phase6() {
	GroupPathPlanningPSO* phase6 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase6);
	phase6->OrthoPhase(5);
	if (phase6->GetIsGroupOpen()) {
		phase6->CloseGroup();
	}
	else {
		phase6->OpenGroup();
	}
}

void MainWindow::PSO_Phase7() {//路径点7
	GroupPathPlanningPSO* phase7 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase7);
	phase7->OrthoPhase(6);
	if (phase7->GetIsGroupOpen()) {
		phase7->CloseGroup();
	}
	else {
		phase7->OpenGroup();
	}
}

void MainWindow::PSO_Phase8() {
	GroupPathPlanningPSO* phase8 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase8);
	phase8->OrthoPhase(7);
	if (phase8->GetIsGroupOpen()) {
		phase8->CloseGroup();
	}
	else {
		phase8->OpenGroup();
	}
}
void MainWindow::PSO_Phase9() {//路径点9
	GroupPathPlanningPSO* phase9 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase9);
	phase9->OrthoPhase(8);
	if (phase9->GetIsGroupOpen()) {
		phase9->CloseGroup();
	}
	else {
		phase9->OpenGroup();
	}
}
void MainWindow::PSO_Phase10() {
	GroupPathPlanningPSO* phase10 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase10);
	phase10->OrthoPhase(9);
	if (phase10->GetIsGroupOpen()) {
		phase10->CloseGroup();
	}
	else {
		phase10->OpenGroup();
	}
}
void MainWindow::PSO_Phase11() {//路径点11
	GroupPathPlanningPSO* phase11 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase11);
	phase11->OrthoPhase(10);
	if (phase11->GetIsGroupOpen()) {
		phase11->CloseGroup();
	}
	else {
		phase11->OpenGroup();
	}
}
void MainWindow::PSO_Phase12() {
	GroupPathPlanningPSO* phase12 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase12);
	phase12->OrthoPhase(10);
	if (phase12->GetIsGroupOpen()) {
		phase12->CloseGroup();
	}
	else {
		phase12->OpenGroup();
	}
}

void MainWindow::PSO_Phase13() {//路径点13
	GroupPathPlanningPSO* phase13 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase13);
	phase13->OrthoPhase(12);
	if (phase13->GetIsGroupOpen()) {
		phase13->CloseGroup();
	}
	else {
		phase13->OpenGroup();
	}
}
void MainWindow::PSO_Phase14() {
	GroupPathPlanningPSO* phase14 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase14);
	phase14->OrthoPhase(13);
	if (phase14->GetIsGroupOpen()) {
		phase14->CloseGroup();
	}
	else {
		phase14->OpenGroup();
	}
}
void MainWindow::PSO_Phase15() {//路径点15
	GroupPathPlanningPSO* phase15 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase15);
	phase15->OrthoPhase(14);
	if (phase15->GetIsGroupOpen()) {
		phase15->CloseGroup();
	}
	else {
		phase15->OpenGroup();
	}
}
void MainWindow::PSO_Phase16() {
	GroupPathPlanningPSO* phase16 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase16);
	phase16->OrthoPhase(15);
	if (phase16->GetIsGroupOpen()) {
		phase16->CloseGroup();
	}
	else {
		phase16->OpenGroup();
	}
}
void MainWindow::PSO_Phase17() {//路径点17
	GroupPathPlanningPSO* phase17 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase17);
	phase17->OrthoPhase(16);
	if (phase17->GetIsGroupOpen()) {
		phase17->CloseGroup();
	}
	else {
		phase17->OpenGroup();
	}
}
void MainWindow::PSO_Phase18() {
	GroupPathPlanningPSO* phase18 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase18);
	phase18->OrthoPhase(17);
	if (phase18->GetIsGroupOpen()) {
		phase18->CloseGroup();
	}
	else {
		phase18->OpenGroup();
	}
}

void MainWindow::PSO_Phase19() {//路径点19
	GroupPathPlanningPSO* phase19 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase19);
	phase19->OrthoPhase(18);
	if (phase19->GetIsGroupOpen()) {
		phase19->CloseGroup();
	}
	else {
		phase19->OpenGroup();
	}
}
void MainWindow::PSO_Phase20() {
	GroupPathPlanningPSO* phase20 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase20);
	phase20->OrthoPhase(19);
	if (phase20->GetIsGroupOpen()) {
		phase20->CloseGroup();
	}
	else {
		phase20->OpenGroup();
	}
}

void MainWindow::PSO_Phase21() {//路径点21——终点
	GroupPathPlanningPSO* phase21 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase21);
	phase21->OrthoPhase(20);
	if (phase21->GetIsGroupOpen()) {
		phase21->CloseGroup();
	}
	else {
		phase21->OpenGroup();
	}
}
void MainWindow::PSO_Phase22() {
	GroupPathPlanningPSO* phase22 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase22);
	phase22->OrthoPhase(21);
	if (phase22->GetIsGroupOpen()) {
		phase22->CloseGroup();
	}
	else {
		phase22->OpenGroup();
	}
}
void MainWindow::PSO_Phase23() {
	GroupPathPlanningPSO* phase23 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase23);
	phase23->OrthoPhase(22);
	if (phase23->GetIsGroupOpen()) {
		phase23->CloseGroup();
	}
	else {
		phase23->OpenGroup();
	}
}
void MainWindow::PSO_Phase24() {
	GroupPathPlanningPSO* phase24 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase24);
	phase24->OrthoPhase(23);
	if (phase24->GetIsGroupOpen()) {
		phase24->CloseGroup();
	}
	else {
		phase24->OpenGroup();
	}
}
void MainWindow::PSO_Phase25() {
	GroupPathPlanningPSO* phase25 = (GroupPathPlanningPSO*)m_GroupManager->GetGroup(Group_Phase25);
	phase25->OrthoPhase(24);
	if (phase25->GetIsGroupOpen()) {
		phase25->CloseGroup();
	}
	else {
		phase25->OpenGroup();
	}
}