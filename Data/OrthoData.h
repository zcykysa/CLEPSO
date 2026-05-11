/******************************************************************************

版权所有 (C), 2023-2025, 西安科技大学

******************************************************************************
文 件 名   :OrthoData.h
生成日期   :2023年2月17日
最近修改   :
功能描述   :主数据类
函数列表   :
修改历史   :
1.日   期  :2023年2月17日
作     者  :刘曼
修改内容   :
******************************************************************************/

#pragma once
#include<map>
#include "Arch.h"
#include "Gum.h"
#include "Tooth.h"
#include"vtkSmartPointer.h"
#include <vector>


enum  ArchType;
enum  GumType;
enum  ToothType;
class vtkRenderer;

class OrthoData           
{
private:
     
	std::map<ArchType, Arch*>m_Arch; //用于存放上/下牙颌总数据的map容器(Actor类型)

	//数据路径
	std::string m_CasePath;

public:
	OrthoData();
	~OrthoData();

	
	std::string GetCasePath(); //获取数据路径

	
	void LoadData(std::string path); //数据路径作为参数加载数据

	
	void AddActors(vtkSmartPointer<vtkRenderer>ren); //向 渲染窗口ren 中添加牙齿和牙龈actor

	Arch* GetArch(ArchType);  //该函数实现功能：获取当前上颌/下颌数据(Actor类型)
	Tooth* GetTooth(ToothId id);  //该函数实现功能：获取当前上颌/下颌数据(Actor类型)
	
	Arch* GetArchPolyData(ArchType); //该函数实现功能：获取当前上颌/下颌数据(PolyData类型)

	std::vector<ToothId> Get_TeethId(ArchType);  //分别获取上/下颌的牙齿编号

	bool Is_Tooth(vtkSmartPointer<vtkActor> actor);//判断是否是牙齿
	bool Is_Gam(vtkSmartPointer<vtkActor> actor);//判断是否是牙龈
	void SetMap(ToothId i, int j, std::vector<double> a);
	bool GetMap(ToothId i, int j, std::vector<double>& axesx);
	void SetPSOGbestx(ToothId i, std::vector<std::vector<double>> gbestx);
	bool GetPSOGbestx(ToothId i, std::vector<std::vector<double>>& gbestx);
	void SetCenterBefore(ToothId i, std::vector<double> cen);
	bool GetCenterBefore(ToothId i, std::vector<double>& center);
	void SetCenterAfter(ToothId i, std::vector<double> cen);
	bool GetCenterAfter(ToothId i, std::vector<double>& center);
	void SetPathToothActor(int i, std::vector<vtkSmartPointer<vtkActor>> init);
	bool GetPathToothActor(int i, std::vector<vtkSmartPointer<vtkActor>>& init);


private:
	
	bool LoadTooth(); //加载牙齿数据
	
	bool LoadGum(); //加载牙龈数据
	
	bool LoadArch(); //加载Arch数据

private:
	std::map<ToothId, std::vector<double>> toothcenter_before;
	std::map<ToothId, std::vector<double>> toothcenter_after;
	std::vector<vtkSmartPointer<vtkActor>> InitToothActor;
	std::vector<vtkSmartPointer<vtkActor>> TargetToothActor;
	std::map<ToothId, std::vector<double>> AxesX;
	std::map<ToothId, std::vector<double>> AxesY;
	std::map<ToothId, std::vector<double>> AxesZ;
	std::map<ToothId, std::vector<std::vector<double>>> m_pso_gbestx;

};