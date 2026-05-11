/******************************************************************************

版权所有 (C), 2023-2025, 西安科技大学

******************************************************************************
文 件 名   :Arch.h
生成日期   :2023年2月17日
最近修改   :
功能描述   :牙颌数据类
函数列表   :
修改历史   :
1.日   期  :2023年2月17日
作     者  :刘曼
修改内容   :
******************************************************************************/



#pragma once
#include <map>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

enum  ArchType :int;
enum  ToothId :int;
class Tooth;
class vtkActor;
class vtkRenderer;
class Gum;
//arch类 16个牙齿 1个牙龈
class Arch
{

private:
	//成员变量包括牙龈数据，牙齿数据，牙颌数据
	Gum *my_Gum;
	std::map<ToothId, Tooth*>my_Tooth; //用于存放对应的牙齿指针
	ArchType m_ArchType; //表示上下颌

	vtkSmartPointer<vtkActor>m_ArchActor;  //存的是整个上/下颌的Actor数据(牙齿牙龈没分离) 
	vtkSmartPointer<vtkPolyData>m_ArchPolyData;  //存储整个上/下颌的PolyData数据(牙齿牙龈没分离)
public:
	Arch(ArchType);
	~Arch();

	//设置Arch数据(actor类型)
	void SetArchActor(vtkSmartPointer<vtkActor>actor);
	//获取Arch数据(actor类型)
	vtkSmartPointer<vtkActor> GetArchActor();

	//设置Arch数据(polydata类型)
	void SetArchPolyData(vtkSmartPointer<vtkPolyData>polydata);
	//获取Arch数据(actor类型)
	vtkSmartPointer<vtkPolyData> GetArchPolyData();

	//添加Arch数据(actor类型)
	void AddArchActor(vtkSmartPointer<vtkRenderer>ren);
	//移除Arch数据(actor类型)
	void RemoveArchActor(vtkSmartPointer<vtkRenderer>ren);


	//添加Tooth数据(actor类型)
	void AddToothActor(vtkSmartPointer<vtkRenderer>ren);
	//添加Gum数据(actor类型)
	void AddToothGum(vtkSmartPointer<vtkRenderer>ren);

	
	
	//创建牙齿
	void Creat_Tooth(ToothId);
	//获取牙齿
	Tooth* Get_Tooth(ToothId);

	//创建牙龈
	void Creat_Gum();
	//获取牙龈
	Gum* Get_Gum();

	
};
