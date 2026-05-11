#include "Arch.h"
#include "Common.h"
#include "Gum.h"
#include "Tooth.h"
#include"vtkActor.h"
#include"vtkRenderer.h"

//构造函数
Arch::Arch(ArchType type) 
{
	m_ArchType = type;   
	my_Gum = NULL;
	m_ArchActor = NULL;
}

Arch::~Arch()
{
}


//-------------------以下四个函数暂时针对的是没分离的牙颌数据----------------------------------------------

void Arch::SetArchActor(vtkSmartPointer<vtkActor>actor) //设置ArchActor
{
	m_ArchActor = actor;
}

vtkSmartPointer<vtkActor> Arch::GetArchActor() //获取ArchActor
{
	return m_ArchActor;
}

void Arch::AddArchActor(vtkSmartPointer<vtkRenderer>ren) //添加ArchActor到指定ren中
{
	if(m_ArchActor!=NULL)
		ren->AddActor(m_ArchActor);
}

void Arch::RemoveArchActor(vtkSmartPointer<vtkRenderer>ren) //移除ArchActor
{
	if (m_ArchActor != NULL)
		ren->RemoveActor(m_ArchActor);
}

//保存ArchPolyData
void Arch::SetArchPolyData(vtkSmartPointer<vtkPolyData>polydata)
{
	m_ArchPolyData = polydata;
}

//获取ArchPolyData
vtkSmartPointer<vtkPolyData> Arch::GetArchPolyData()
{
	return m_ArchPolyData;
}

//-------------------------------------以下两函数是添加牙齿和牙龈actor到ren中------------------------------

//向指定ren渲染器中添加牙齿actor
void Arch::AddToothActor(vtkSmartPointer<vtkRenderer>ren)
{
	for (auto it = my_Tooth.begin(); it != my_Tooth.end(); it++)
	{
		//map是以键值对的形式来存储的，it->second指向存储的牙齿指针，再调用GetToothActor()获取牙齿
		ren->AddActor(it->second->GetToothActor());
	}
}

//向指定ren渲染器中添加牙龈actor
void Arch::AddToothGum(vtkSmartPointer<vtkRenderer>ren)
{
	if(my_Gum!=NULL)
	{
		ren->AddActor(my_Gum->GetGumActor()); 
	}
}


//---------------------以下两函数是 创建牙齿指针放到对应map容器中 以及 获取指定牙齿指针 ---------------------

void Arch::Creat_Tooth(ToothId id) //创建Tooth指针
{
	my_Tooth[id] = new Tooth(id); //创建单颗牙齿对象放入牙齿map容器中
}


Tooth* Arch::Get_Tooth(ToothId id) //获取指定枚举值的牙齿对象指针，并返回
{
	if (my_Tooth.find(id)!= my_Tooth.end())
	{
		Tooth* T1 = my_Tooth[id];
		return T1;
	}
	return NULL;
}


//------------------------------以下两函数是 创建牙龈对象 以及 获取牙龈对象----------------------------------

void Arch::Creat_Gum() //创建牙龈对象
{
	my_Gum = new Gum(m_ArchType); 
}


Gum* Arch::Get_Gum() //获取牙龈 
{
	return my_Gum;  
}

