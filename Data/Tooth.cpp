#include "Tooth.h"
#include "Common.h"
#include<vtkPolyData.h>
#include<vtkActor.h>
#include"vtkSmartPointer.h"
//有参构造函数
Tooth::Tooth(ToothId id)
{
	m_Toothid = id;
}
//析构函数
Tooth::~Tooth()
{

}
//设置toothActor
void Tooth::SetToothActor(vtkSmartPointer<vtkActor> tooth)
{
	tooth_Actor = tooth;

}

//获取toothActor
vtkSmartPointer <vtkActor> Tooth:: GetToothActor()
{
	return tooth_Actor;
}

//设置toothPolyData
void Tooth::SetToothPolyData(vtkSmartPointer<vtkPolyData> tooth)
{
	tooth_PolyData = tooth;

}

//获取toothPolyData
vtkSmartPointer <vtkPolyData> Tooth::GetToothPolyData()
{
	return tooth_PolyData;
}