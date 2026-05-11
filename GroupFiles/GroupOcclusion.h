#pragma once
#include "GroupBase.h"
#include "GroupManager.h"
#include <QObject>
#include <vtkAlgorithm.h>
#include <vtkPolyData.h>
#include "RendererContainer.h"
#include "Tooth.h"
#include "RendererContainer.h"
using namespace std;

class RendererContainer;
class OrthoData;

class GroupOcclusion :public GroupBase
{
private:
	std::vector<vtkSmartPointer<vtkActor>> colActor;
public:

	bool m_IsOpen;
	//构造函数
	GroupOcclusion(GroupType type);

	//析构函数
	~GroupOcclusion();

	//碰撞实现
	void Collide(Tooth* T1, Tooth* T2);

	virtual void OpenGroup() override; //实现父类中的纯虚函数（virtual 可以省略）
	virtual void CloseGroup() override;
};



