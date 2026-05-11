
#pragma once
#include <iostream>
using namespace std;
#include "vtkSmartPointer.h"
#include "GroupBase.h"
#include "vtkActor.h"
#include "OrthoData.h"
#include "Arch.h"
#include "Common.h"
#include "MainWindow.h"
#include <vtkLineSource.h>
#include <vtkLine.h>
#include <vtkDataSet.h>
#include <vtkPolyData.h>
#include <vtkDataSetMapper.h>
#include <QPen.h>
#include <vtkRenderer.h>


class RendererContainer;
class OrthoData;

class GroupGridLine : public GroupBase
{
public:
	GroupGridLine(GroupType type); //设置无参构造函数
	~GroupGridLine();

	virtual void OpenGroup() override; //实现父类中的纯虚函数（virtual 可以省略）
	virtual void CloseGroup() override;

	void OnGridLine();


private:
	//std::vector<vtkSmartPointer<vtkActor>> lineactor;
	vtkSmartPointer<vtkPoints>         LinePoints;
	vtkSmartPointer<vtkCellArray>      LineCellArray;
	vtkSmartPointer<vtkPolyData>       LinePolyData;
	vtkSmartPointer<vtkDataSetMapper>  LinePolyMapper;
	vtkSmartPointer<vtkRenderer>       ToothRenderer;
	vtkSmartPointer<vtkRenderer>       LineRenderer;
	//int minGap = 5;                              // 最小线间隔
	//int maxGap = 50;                              // 最大线间隔
	//float gap = 5;                               // 当前线间隔
	//float startX = 0;                             // 竖线初始x坐标（从窗口中间开始）
	//float startY = 0;                            // 横线初始y坐标（从窗口中间开始）
	//QPen lightPen = 1;                           // 细一点的画笔
	//QPen darkPen = 2;                            // 粗一点的画笔
	

};

