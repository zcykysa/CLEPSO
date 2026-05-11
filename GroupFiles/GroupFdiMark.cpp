#include "GroupFdiMark.h"
#include "vtkNamedColors.h"
#include "vtkVectorText.h"
#include "vtkPolyDataMapper.h"
#include "vtkFollower.h"
#include "vtkProperty.h"
#include "vtkAbstractPicker.h"
#include "vtkSphereSource.h"

GroupFdiMark::GroupFdiMark(GroupType type) : GroupBase(type) {

	m_GroupType = type;


}
GroupFdiMark::~GroupFdiMark() {

}

void GroupFdiMark::Fdi(double follPos_X, double follPos_Y, double follPos_Z,
	double rotX, double rotY, double rotZ, const char* orthNumber) {

	vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();

	vtkSmartPointer<vtkVectorText> textSource = vtkSmartPointer<vtkVectorText>::New();
	textSource->SetText(orthNumber);
	textSource->Update();

	vtkSmartPointer<vtkPolyDataMapper> textMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	textMapper->SetInputData(textSource->GetOutput());

	vtkSmartPointer<vtkFollower> follower = vtkSmartPointer<vtkFollower>::New();  //vtkActor
	follower->SetMapper(textMapper);
	follower->SetPosition(follPos_X, follPos_Y, follPos_Z);
	follower->GetProperty()->SetColor(colors->GetColor3d("Black").GetData());
	follower->SetScale(2.5, 2.8, 1.9);
	follower->RotateX(rotX);
	follower->RotateY(rotY);
	follower->RotateY(rotZ);

	fdiActor.push_back(follower);

}

void GroupFdiMark::FdiMark() {

	Fdi(-5.5, -22.3, 14, 55.5, 15, -10.5, "11");
	Fdi(-13.1, -15.6, 11, 75, 8, -60, "12");
	Fdi(-19, -8.9, 11, 99, 8, -60, "13");
	Fdi(-23.5, -1.6, 10.5, 99, 8, -60, "14");
	Fdi(-27.5, 6.15, 10.5, 99, 1, -60, "15");
	Fdi(-31, 15.1, 12.7, 99, -5, -60, "16");

	Fdi(5.5, -21.25, 14, 55.5, 25, -10.5, "21");
	Fdi(13.8, -15.5, 11.8, 85, 85, -10.5, "22");
	Fdi(18, -8.2, 11.2, 95, 72, -10.5, "23");
	Fdi(20.5, -0.5, 10, 100, 72, -10.5, "24");
	Fdi(23, 7.7, 9, 100, 85, -10.5, "25");
	Fdi(25.5, 19, 11, 100, 95, -10.5, "26");

	Fdi(-4, -18.3, 3, 89, 15, -10.5, "41");
	Fdi(-10.5, -17.5, 1.7, 89, 12, -10.5, "42");
	Fdi(-17, -11.5, 0.4, 80, -29.4, -10.5, "43");
	Fdi(-21, -5.3, -0.45, 75, -40, -10.5, "44");
	Fdi(-26.6, 1.5, -0.5, 75, -40, -10.5, "45");
	Fdi(-31, 10.7, -0.3, 72, -40, -10.5, "46");

	Fdi(1.4, -17.63, 3, 89, 15, -10.5, "31");
	Fdi(7.2, -16.1, 3, 89, 25, -10.5, "32");
	Fdi(13.6, -12.25, 2.5, 87, 50, -10.5, "33");
	Fdi(17.6, -5.64, 1.5, 85, 65, -10.5, "34");
	Fdi(21.5, 1.965, 1.5, 78, 65, 10, "35");
	Fdi(24.1, 11.5, 2.3, 80, 65, 10, "36");

}

void GroupFdiMark::OnRightButtonDown() {

	vtkSmartPointer<vtkRenderWindowInteractor> rwi = m_RenderContainer->GetRenWinInteractor();
	//打印鼠标右键像素位置
	cout << "Mouse Coordinates: " << rwi->GetEventPosition()[0]
		<< " " << rwi->GetEventPosition()[1] << endl;

	//注册拾取点函数
	rwi->GetPicker()->Pick(
		rwi->GetEventPosition()[0],
		rwi->GetEventPosition()[1],
		0,
		m_RenderContainer->GetRenderer(MainRenderer)
	);

	//获取拾取点的三维坐标
	double picked[3];
	rwi->GetPicker()->GetPickPosition(picked);
	std::cout << "Marker Coordinates: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;

	vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->SetRadius(3);
	sphereSource->Update();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(sphereSource->GetOutputPort());

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->SetPosition(picked);
	actor->SetScale(0.05);
	actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
	m_RenderContainer->GetRenderer(MainRenderer)->AddActor(actor);
}



void GroupFdiMark::OpenGroup() { //打开功能组
	m_IsOpen = true;
	for (auto it = fdiActor.begin(); it != fdiActor.end(); ++it) {
		vtkSmartPointer<vtkActor> actor = *it;
		m_RenderContainer->GetRenderer(MainRenderer)->AddActor(actor);
	}
	cout << "vector.size()(Open) = " << fdiActor.size() << endl;
	m_RenderContainer->Render();
}

void GroupFdiMark::CloseGroup() {  //关闭功能组
	m_IsOpen = false;
	for (auto it = fdiActor.begin(); it != fdiActor.end(); ++it) {
		vtkSmartPointer<vtkActor> actor = *it;
		m_RenderContainer->GetRenderer(MainRenderer)->RemoveActor(actor);
	}
	fdiActor.clear();
	cout << "vector.size()(Close) = " << fdiActor.size() << endl;
	m_RenderContainer->Render();
}