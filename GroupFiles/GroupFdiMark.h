/******************************************************************************

版权所有 (C), 2023-2025, 西安科技大学

******************************************************************************
文 件 名   :GroupFdiMark.h
生成日期   :2023年3月9日
最近修改   :
功能描述   :FDI标记类
函数列表   :
修改历史   :
1.日   期  :2023年3月9日
作     者  :彭龙
修改内容   :
******************************************************************************/

#pragma once
#include "GroupBase.h"
#include "Arch.h"
#include "Common.h"
#include "OrthoData.h"
#include "Gum.h"
#include "QSharedPointer.h"
#include "RendererContainer.h"
#include <iostream>
using namespace std;

class GroupBase;

class GroupFdiMark :public GroupBase
{
public:
	GroupFdiMark(GroupType);
	~GroupFdiMark();

	virtual void OpenGroup() override;
	virtual void CloseGroup() override;

	void Fdi(double follPos_X, double follPos_Y, double follPos_Z,
		double rotX, double rotY, double rotZ, const char* orthNumber);

	void FdiMark();

	void OnRightButtonDown();

private:
	std::vector<vtkSmartPointer<vtkActor>>fdiActor;  //作用：便于actor的添加与删除
};

