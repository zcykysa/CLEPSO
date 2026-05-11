#include "GroupMoveCalculation.h"
#include <vtkCellCenters.h>
#include "vtkActor.h"
#include "vtkPolyData.h"
#include "vtkMapper.h"
#include <vtkPropPicker.h>
#include <vtkProPerty.h>

GroupMoveCalculation::GroupMoveCalculation(GroupType type) : GroupBase(type) {
	m_GroupType = type;
	LastPickedActor = NULL;
	LastPickedProperty = vtkProperty::New();
}

GroupMoveCalculation::~GroupMoveCalculation() {
	LastPickedProperty->Delete();
}

void GroupMoveCalculation::MoveCalculation() {//牙颌移动量计算
	
}

bool GroupMoveCalculation::OnMouseDoublePress(MouseEvent  type) {
	// 鼠标点击位置
	int* clickPos = m_RenderContainer->GetRenWinInteractor()->GetEventPosition();
	// 在鼠标点击位置拾取
	vtkSmartPointer<vtkPropPicker>  picker =
		vtkSmartPointer<vtkPropPicker>::New();
	picker->Pick(clickPos[0], clickPos[1], 0, m_RenderContainer->GetRenderer());//Pick()实现拾取功能
	//virtual int Pick (double selectionX, double selectionY, double selectionZ, vtkRenderer *renderer)

	//生成世界坐标系统的坐标值
	double* pos = picker->GetPickPosition();
	//重置所选Actor的属性
	if (this->LastPickedActor)
	{
		this->LastPickedActor->GetProperty()->DeepCopy(this->LastPickedProperty);
	}
	this->LastPickedActor = picker->GetActor();
	if (this->LastPickedActor)
	{
		if (m_OrthoData->Is_Tooth(this->LastPickedActor)) {
			//保存所选的Actor属性，以备下次恢复
			this->LastPickedProperty->DeepCopy(this->LastPickedActor->GetProperty());
			//通过更改所选Actor的属性为红色来高亮显示该Actor
			this->LastPickedActor->GetProperty()->SetColor(0.8, 0.0, 0.0);
			this->LastPickedActor->GetProperty()->SetDiffuse(1.0);
			this->LastPickedActor->GetProperty()->SetSpecular(0.0);
			m_RenderContainer->Render();
		}
	}
	return true;
}


void GroupMoveCalculation::OpenGroup() {
	m_IsOpen = true;
	cout << "OpenGroup()" << endl;
	m_RenderContainer->Render();
}

void GroupMoveCalculation::CloseGroup() {
	m_IsOpen = false;
	if (this->LastPickedActor != NULL)  this->LastPickedActor->SetProperty(LastPickedProperty);//功能关闭时恢复原始颜色
	cout << "CloseGroup()" << endl;
	m_RenderContainer->Render();
}
