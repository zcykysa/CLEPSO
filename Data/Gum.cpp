#include "Gum.h"
#include "vtkPolyData.h"
#include "vtkActor.h"

enum  ArchType :int;

//构造函数
Gum::Gum(ArchType type)
{
	m_ArchType = type;
}
Gum::~Gum()
{

}
//保存牙龈(actor类型)
void Gum::SetGumActor(vtkSmartPointer<vtkActor> gum)
{
	gum_Actor = gum;


}
//获取牙龈(actor类型)
vtkSmartPointer <vtkActor> Gum::GetGumActor()
{
	return gum_Actor;

}

//保存牙龈(PolyData类型)
void Gum::SetGumPolyData(vtkSmartPointer<vtkPolyData> gum)
{
	gum_PolyData = gum;
}
//获取牙龈(PolyData类型)
vtkSmartPointer <vtkPolyData> Gum::GetGumPolyData()
{
	return gum_PolyData;
}