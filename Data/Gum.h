/******************************************************************************

版权所有 (C), 2023-2025, 西安科技大学

******************************************************************************
文 件 名   :Gum.h
生成日期   :2023年2月17日
最近修改   :
功能描述   :牙龈数据类
函数列表   :
修改历史   :
1.日   期  :2023年2月17日
作     者  :刘曼
修改内容   :
******************************************************************************/


#pragma once
#include<vtkSmartPointer.h>

enum  ArchType :int;
class vtkPolyData;
class vtkActor;
//牙龈类
class Gum
{
private:
	double x, y, z;//牙龈中心点坐标
	bool exist;//牙龈是否存在
	vtkSmartPointer<vtkPolyData> gum_PolyData;//牙龈PolyData类型的数据
	vtkSmartPointer<vtkActor> gum_Actor;//牙龈Actor类型的数据
	ArchType m_ArchType;

public:
	Gum (ArchType);
	~Gum();

	//保存gum_Actor
	void SetGumActor(vtkSmartPointer<vtkActor>);

	//获取gum_Actor
	vtkSmartPointer <vtkActor> GetGumActor();

	//保存gum_PolyData
	void SetGumPolyData(vtkSmartPointer<vtkPolyData>);

	//获取gum_PolyData
	vtkSmartPointer <vtkPolyData> GetGumPolyData();
	
};

