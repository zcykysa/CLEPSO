#include "GroupOcclusion.h"
#include "GroupBase.h"
#include <vtkCollisionDetectionFilter.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include "MainWindow.h"
#include "Common.h"
#include <vtkTextActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkTextProperty.h>
#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include "RendererContainer.h"
#include <vtkAlgorithm.h>
#include <vtkRenderWindow.h>
#include <OrthoData.h>

class OrthoData;

//构造函数
GroupOcclusion::GroupOcclusion(GroupType type) : GroupBase(type) {

	m_GroupType = type;

}
//析构函数
GroupOcclusion::~GroupOcclusion()
{
}



//实现碰撞的主要算法
void GroupOcclusion::Collide(Tooth* T1, Tooth* T2) {
	
	// 获取T1和T2对应的PolyData数据
	vtkPolyData* p1 = T1->GetToothPolyData();
	vtkPolyData* p2 = T2->GetToothPolyData();
	
	//contactMode:碰撞模式
	int contactMode = 0;

	vtkNew<vtkMatrix4x4> matrix1;
	vtkNew<vtkTransform> transform0;

	//设置输入数据
	vtkNew<vtkCollisionDetectionFilter> collide;
	//输入第一个polyData数据
	collide->SetInputData(0, p1);
	//对第一个输入数据进行空间几何变换
	collide->SetTransform(0, transform0);
	//输入第二个polyData数据
	collide->SetInputData(1, p2);
	//对第二个数据进行矩阵变换
	collide->SetMatrix(1, matrix1);
	//设置并获取OBB公差
	collide->SetBoxTolerance(0.001);
	//设置并获取单元格公差
	collide->SetCellTolerance(0.001);
	//设置并获取每个OBB中的单元格数
	collide->SetNumberOfCellsPerNode(2);
	
	//根据contactMode选择不同的碰撞模式
	if (contactMode == 0)
	{
		
		collide->SetCollisionModeToAllContacts();
	}
	else if (contactMode == 1)
	{
		collide->SetCollisionModeToFirstContact();
	}
	else
	{
		collide->SetCollisionModeToHalfContacts();
	}
	collide->GenerateScalarsOn();

	//Visualize 可视化
	//颜色类
	vtkNew<vtkNamedColors> colors;
	
	//获取T1对应的actor数据,并做出相关处理
	vtkActor* actor1 = T1->GetToothActor();
	actor1->GetProperty()->BackfaceCullingOn();
	actor1->SetUserTransform(transform0);
	
	//获取T2对应的actor数据,并做出相关处理
	vtkActor* actor2 = T2->GetToothActor();
	//开启背面裁剪/剔除
	actor2->GetProperty()->BackfaceCullingOn();
	actor2->SetUserMatrix(matrix1);
	
	//需要显示的碰撞部分
	vtkNew<vtkPolyDataMapper> mapper3;
	mapper3->SetInputConnection(collide->GetContactsOutputPort());
	mapper3->SetResolveCoincidentTopologyToPolygonOffset();

	vtkNew<vtkActor> actor3;
	actor3->SetMapper(mapper3);
	actor3->GetProperty()->SetColor(colors->GetColor3d("red").GetData());
	actor3->GetProperty()->SetLineWidth(10.0);
	
	//将碰撞区域的actor数据存储在vector容器中
	colActor.push_back(actor3);

	//获取Renderer,并将actor1,actor2,actor3添加进去 
	vtkSmartPointer <vtkRenderer> m_Renderer = m_RenderContainer->GetRenderer();
	m_Renderer->UseHiddenLineRemovalOn();
	m_Renderer->AddActor(actor1);
	m_Renderer->AddActor(actor2);
	m_Renderer->AddActor(actor3);
	m_Renderer->UseHiddenLineRemovalOn();

	// Move the first object
	int numSteps = 100;
	double dx = 10.0 / static_cast<double>(numSteps) * 0.5;//dx=10/2000
	//使用Translate移动对象
	transform0->Translate(-40, 42.0, -18.0);

	m_RenderContainer->Render();


	for (int i = 0; i < numSteps; ++i)
	{
		transform0->Translate(0.5, -0.5, dx);
		m_Renderer->ResetCameraClippingRange();
		m_RenderContainer->Render();
		if (collide->GetNumberOfContacts() > 0)
		{
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(3000 / numSteps));
	}

	m_RenderContainer->Render();

}
//打开功能
void GroupOcclusion::OpenGroup()
{
	//将m_IsOpen设置为true
	m_IsOpen = true;
	cout << "OpenGroup()重写" << endl;

}

//关闭功能
void GroupOcclusion::CloseGroup()
{
	//将m_IsOpen设置为false
	m_IsOpen = false;
	//通过从vector容器中取出碰撞区域的actor数据，并将其Remove来达到关闭功能的效果
	for (auto it = colActor.begin(); it != colActor.end(); it++)
	{

		vtkSmartPointer<vtkActor> actor = *it;
		m_RenderContainer->GetRenderer(MainRenderer)->RemoveActor(actor);

	}
	colActor.clear();
	cout << "vector.size()(Close) = " << colActor.size() << endl;
	m_RenderContainer->Render();
	cout << "CloseGroup()重写" << endl;
}