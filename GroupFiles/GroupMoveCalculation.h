/******************************************************************************

版权所有 (C), 2023-2025, 西安科技大学

******************************************************************************
文 件 名   :GroupMoveCalculation.h
生成日期   :2023年3月8日
最近修改   :
功能描述   :牙颌平移量计算类
函数列表   :
修改历史   :
1.日   期  :2023年3月8日
作     者  :胡雪
修改内容   :
******************************************************************************/

#pragma once
#include <iostream>
using namespace std;
#include "vtkSmartPointer.h"
#include "GroupBase.h"
#include "vtkActor.h"
#include "OrthoData.h"
#include "Arch.h"
#include "Common.h"
#include "vtkProperty.h"
#include "RendererContainer.h"
#include "QSharedPointer.h"

class GroupMoveCalculation : public GroupBase
{
public:
	GroupMoveCalculation(GroupType type); //设置无参构造函数
	~GroupMoveCalculation();

	virtual void OpenGroup() override; //实现父类中的纯虚函数（virtual 可以省略）
	virtual void CloseGroup() override;

	void MoveCalculation();//牙颌移动量计算

	bool OnMouseDoublePress(MouseEvent  type) override;//重写的鼠标双击事件

private:
	vtkActor* LastPickedActor;
	vtkProperty* LastPickedProperty;

};

