/******************************************************************************

版权所有 (C), 2023-2025, 西安科技大学

******************************************************************************
文 件 名   :Tooth.h
生成日期   :2023年2月17日
最近修改   :
功能描述   :牙齿数据类
函数列表   :
修改历史   :
1.日   期  :2023年2月17日
作     者  :刘曼
修改内容   :
******************************************************************************/


#pragma once

#include"vtkSmartPointer.h"

enum  ToothId :int;
class vtkPolyData;
class vtkActor;
//单颗牙齿类
class Tooth
{
private:
	ToothId m_Toothid;//牙齿FDI编号(枚举)
	vtkSmartPointer<vtkPolyData> tooth_PolyData;//toothPolyData类型数据    
	vtkSmartPointer<vtkActor> tooth_Actor;//toothActor类型数据

public:
	Tooth(ToothId id);
	~Tooth();

	//设置/获取tooth_Actor
	void SetToothActor(vtkSmartPointer<vtkActor>);
	vtkSmartPointer<vtkActor> GetToothActor();

	//设置/获取tooth_PolyData
	void SetToothPolyData(vtkSmartPointer<vtkPolyData>);
	vtkSmartPointer<vtkPolyData> GetToothPolyData();
};
