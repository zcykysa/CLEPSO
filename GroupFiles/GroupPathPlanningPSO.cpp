#include "GroupPathPlanningPSO.h"
#include <map>
#include "vtkPolyDataMapper.h"
#include "vtkPolyData.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkActor.h"
#include "vtkProperty.h"
#include "OrthoData.h"
#include "QSharedPointer.h"
#include "Arch.h"
#include "Common.h"
#include "GroupBase.h"
#include "RendererContainer.h"
#include "iostream"
#include "ctime"
#include "cstdlib"
#include <vtkPropPicker.h>
#include <vtkProPerty.h>
#include <vtkOutlineFilter.h>
#include <vtkCollisionDetectionFilter.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <fstream>
#include <vector>
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include <vtkArrowSource.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkVertex.h>
#include <vtkDoubleArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkGlyph3D.h>
#include <iostream>
#include <cmath>
#include <vtkMath.h>
#include <math.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkFeatureEdges.h>
#include "QSharedPointer.h"
#include <Common.h>
#include <iomanip>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include<ctime>
#include <vector>
#include <vtkOBBTree.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkBooleanOperationPolyDataFilter.h>

using namespace std;
#define N  999 //精度为小数点后面3位

GroupPathPlanningPSO::GroupPathPlanningPSO(GroupType type) : GroupBase(type) {
	m_GroupType = type;
}

GroupPathPlanningPSO::~GroupPathPlanningPSO() {

}







void GroupPathPlanningPSO::RotateXYZeulerAngle() {// 计算旋转量
	Eigen::Matrix3d rotation_matrix; rotation_matrix << 0.933091, -0.188831, -0.306078, 0.229071, 0.968152, 0.10104, 0.277251, -0.164393, 0.946629;//初始化旋转矩阵
	Eigen::Vector3d eulerAngle = rotation_matrix.eulerAngles(2, 0, 1);//旋转矩阵转欧拉角(Z-X-Y(2-0-1)，即RPY)
	cout<< eulerAngle [0]<<"," << eulerAngle[1] << "," << eulerAngle[2]  << endl;
}


void GroupPathPlanningPSO::RotateVYZValue(int id, double r11, double r12, double r13, double r21, double r22, double r23, double r31, double r32, double r33) {//计算旋转量
	//double rotate_matrix[3][3] = { {r11, r12, r13} ,
	//	                             {r21,r22, r23},
	//	                             {r31, r32, r33 } };
	 
	
	const double PI = 3.141592653;
	float sy = sqrt(r11 * r11 + r21 * r21);

	bool singular = sy < 1e-6; // If

	float x, y, z;
	x = asin(r32);
	y = atan2(-r31, r33);
	z = atan2(-r12, r22);
	after[id][3] = x;
	after[id][4] = y;
	after[id][5] = z;
	/*if (!singular)
	{
		x = atan2(r32, r33);
		y = atan2(-1 * r31, sy);
		z = atan2(r21, r11);
	}
	else
	{
		x = atan2(-1 * r23, r22);
		y = atan2(-1 * r31, sy);
		z = 0;
	}*/
	
	
	/*if (!singular)
	{
		x = atan2(r32, r33) / PI * 180;
		y = atan2(-1 * r31,  sy) / PI * 180;
		z = atan2(r21, r11) / PI * 180;
	}
	else
	{
		x = atan2(-1 * r23, r22) / PI * 180;
		y = atan2(-1 * r31, sy) / PI * 180;
		z = 0;
	}*/


	////反正切atan2(r21, r11)
	//const double PI = 3.141592653;
	//double z = atan2(r21, r11) / PI * 180;//z旋转量
	//double y = atan2(-1 * r31, sqrt(r31 * r31 + r33 * r33)) / PI * 180;//y旋转量
	//double x = atan2(r32, r33) / PI * 180;//x旋转量

	/*cout << "toothid:"<< id <<endl;
	cout << "z旋转量:" << z << endl;
	cout << "y旋转量:" << y << endl;
	cout << "x旋转量:" << x << endl;*/

	cout << "after[" << id << "][3]=gbestx[" << id << "][n][3] = " << x << ";" << endl;
	cout << "after[" << id << "][4]=gbestx[" << id << "][n][4] = " << y << ";" << endl;
	cout << "after[" << id << "][5]=gbestx[" << id << "][n][5] = " << z << ";" << endl;

	
}

void GroupPathPlanningPSO::MatrixDecomposition() {
	////取测试的单个牙齿
	//Tooth* tooth1 = m_OrthoData->GetTooth((ToothId)7);
	////使用vtkMatrix4x4矩阵使其变换到理想位置
	//vtkSmartPointer<vtkMatrix4x4> matriedec = vtkSmartPointer<vtkMatrix4x4>::New();
	//for (int j = 0; j < 4; j++) {
	//	for (int k = 0; k < 4; k++) {
	//		matriedec->SetElement(j, k, Matrix[i](j, k));
	//	}
	//}
	//matriedec->DecomposeMatrix();

	//tooth1->GetToothActor()->SetUserMatrix(matriedec);
}




//void grouppathplanningpso::Axes(){
//
//	double tooth_center[16][3];
//	double tooth_x[16][3];
//	double tooth_y[16][3];
//	double tooth_z[16][3];
//	double tooth_normalizex[16][3];
//	double tooth_normalizey[16][3];
//	double tooth_normalizez[16][3];
//	const int targettoothid = 7;
//	for (int p = 2; p < 16; p++)
//	{
//		tooth* tooth = m_orthodata->gettooth((toothid)p);
//		if (p == targettoothid) {
//			// 显示目标牙齿
//			tooth->gettoothactor()->setvisibility(true);
//			// 可选：突出显示目标牙齿（设置颜色和透明度）
//			tooth->gettoothactor()->getproperty()->setopacity(1.0);
//			tooth->gettoothactor()->getproperty()->setcolor(0.8, 0.8, 0.8);
//		}
//		else {
//			// 隐藏其他牙齿
//			tooth->gettoothactor()->setvisibility(false);
//		}
//		// 记录所有牙齿中心点（即使隐藏也需要用于坐标计算）
//		auto center = tooth->gettoothactor()->getcenter();
//		tooth_center[p][0] = center[0];
//		tooth_center[p][1] = center[1];
//		tooth_center[p][2] = center[2];
//	}
//	//for (int p = 2; p < 16; p++)
//	//{
//	//	//取测试的单个牙齿,记录中心点
//	//	tooth* tooth = m_orthodata->gettooth((toothid)p);
//	//	tooth->gettoothactor()->setvisibility(true);
//	//	auto center = tooth->gettoothactor()->getcenter();
//	//	tooth_center[p][0] = center[0];
//	//	tooth_center[p][1] = center[1];
//	//	tooth_center[p][2] = center[2];
//	//}
//	//for (int p = 5; p < 8; p++)
//	//{
//	//	//取测试的单个牙齿,记录中心点
//	//	tooth* tooth = m_orthodata->gettooth((toothid)p);
//	//	tooth->gettoothactor()->setvisibility(true);
//	//	tooth->gettoothactor()->getproperty()->setopacity(0.6);
//	//	//取测试的单个牙齿
//	//	if (p == 6) {
//	//		////修改透明度和颜色
//	//		tooth->gettoothactor()->getproperty()->setopacity(0.6);
//	//		tooth->gettoothactor()->getproperty()->setcolor(1, 0, 0);
//	//	}
//	//	
//	//	auto center = tooth->gettoothactor()->getcenter();
//	//	tooth_center[p][0] = center[0];
//	//	tooth_center[p][1] = center[1];
//	//	tooth_center[p][2] = center[2];
//	//}
//
//	////右侧最后一个牙齿
//	////x轴
//	tooth_x[7][0] = tooth_center[2][0] - tooth_center[3][0];
//	tooth_x[7][1] = tooth_center[2][1] - tooth_center[3][1];
//	tooth_x[7][2] = tooth_center[2][2] - tooth_center[3][2];
//	////计算x轴向量模长—xd
//	double xx = pow(tooth_x[7][0], 2);//pow()幂次方函数
//	double xy = pow(tooth_x[7][1], 2);
//	double xz = pow(tooth_x[7][2], 2);
//	double xd = sqrt(xx + xy + xz);//sqrt()平方根函数
//	//计算x轴单位向量——normalizex[3]
//	tooth_normalizex[7][0] = tooth_x[7][0] / xd;
//	tooth_normalizex[7][1] = tooth_x[7][1] / xd;
//	tooth_normalizez[7][2] = tooth_x[7][2] / xd;
// //   //------------------z轴-----------------------------------
//	////计算临时z轴上两个点的方向向量(终点 - 起点)——directioninterimz[3]
//	tooth_z[7][0] = 0;
//	tooth_z[7][1] = 0;
//	tooth_z[7][2] = -1;
//	////计算临时z轴向量模长——zinterimd
//	double zinterimx = pow(tooth_z[7][0], 2);//pow()幂次方函数
//	double zinterimy = pow(tooth_z[7][1], 2);
//	double zinterimz = pow(tooth_z[7][2], 2);
//	double zinterimd = sqrt(zinterimx + zinterimy + zinterimz);//sqrt()平方根函数
//	////计算临时z轴单位向量（为了和normalizex叉乘得到y轴单位向量）——normalizeinterimz[3]
//	tooth_normalizez[7][0] = tooth_z[7][0] / zinterimd;  // 修正原代码错误
//	tooth_normalizez[7][1] = tooth_z[7][1] / zinterimd;  // 修正原代码错误
//	tooth_normalizez[7][2] = tooth_z[7][2] / zinterimd;
//
//	////叉乘计算y轴单位向量——normalizey[3]
//	tooth_normalizey[7][0] = tooth_normalizex[7][1] * tooth_normalizez[7][2] - tooth_normalizex[7][2] * tooth_normalizez[7][1];
//	tooth_normalizey[7][1] = tooth_normalizex[7][2] * tooth_normalizez[7][0] - tooth_normalizex[7][0] * tooth_normalizez[7][2];
//	tooth_normalizey[7][2] = tooth_normalizex[7][0] * tooth_normalizez[7][1] - tooth_normalizex[7][1] * tooth_normalizez[7][0];
//	//
//	////---------------------z轴----------------------------------
//	////叉乘计算z轴单位向量normalizez[3]
//	tooth_normalizez[7][0] = tooth_normalizex[7][1] * tooth_normalizey[7][2] - tooth_normalizex[7][2] * tooth_normalizey[7][1];
//	tooth_normalizez[7][1] = tooth_normalizex[7][2] * tooth_normalizey[7][0] - tooth_normalizex[7][0] * tooth_normalizey[7][2];
//	tooth_normalizez[7][2] = tooth_normalizex[7][0] * tooth_normalizey[7][1] - tooth_normalizex[7][1] * tooth_normalizey[7][0];
//
//	//vtkdoublearray* normalsx = vtkdoublearray::new();  //创建法向量属性，存入向量的朝向target
//	//normalsx->setnumberofcomponents(3);
//	//normalsx->insertnexttuple(tooth_normalizex[2]);
//	//vtkdoublearray* normalsy = vtkdoublearray::new();  //创建法向量属性，存入向量的朝向target
//	//normalsy->setnumberofcomponents(3);
//	//normalsy->insertnexttuple(tooth_normalizey[2]);
//	//vtkdoublearray* normalsz = vtkdoublearray::new();  //创建法向量属性，存入向量的朝向target
//	//normalsz->setnumberofcomponents(3);
//	//normalsz->insertnexttuple(tooth_normalizez[2]);
//
//	//
//
//
//	//
//	vtkpoints* points = vtkpoints::new();  //记录起点坐标
//	points->insertnextpoint(tooth_center[7]);
//	vtkvertex* vertex = vtkvertex::new();  //建立起点的拓扑(不建立拓扑的话是不行的)
//	vertex->getpointids()->setnumberofids(points->getnumberofpoints());
//	for (int ij = 0; ij < points->getnumberofpoints(); ij++) {
//		vertex->getpointids()->setid(ij, ij);
//	}
//	// 存储轴向量（方向）
//	vtkdoublearray* normalsx = vtkdoublearray::new();
//	normalsx->setnumberofcomponents(3);
//	normalsx->insertnexttuple(tooth_normalizex[7]);
//
//	vtkdoublearray* normalsy = vtkdoublearray::new();
//	normalsy->setnumberofcomponents(3);
//	normalsy->insertnexttuple(tooth_normalizey[7]);
//
//	vtkdoublearray* normalsz = vtkdoublearray::new();
//	normalsz->setnumberofcomponents(3);
//	normalsz->insertnexttuple(tooth_normalizez[7]);
//
//	// 存储箭头长度（标量）
//	vtkdoublearray* scalars = vtkdoublearray::new();
//	scalars->setnumberofcomponents(1);
//	scalars->insertnexttuple1(100);  // 箭头长度缩放因子
//
//	// 封装几何数据
//	vtksmartpointer<vtkcellarray> vertices = vtksmartpointer<vtkcellarray>::new();
//	vertices->insertnextcell(vertex);
//
//	vtkpolydata* polydatax = vtkpolydata::new();
//	polydatax->setpoints(points);
//	polydatax->setverts(vertices);
//	polydatax->getpointdata()->setnormals(normalsx);
//	polydatax->getpointdata()->setscalars(scalars);
//
//	vtkpolydata* polydatay = vtkpolydata::new();
//	polydatay->setpoints(points);
//	polydatay->setverts(vertices);
//	polydatay->getpointdata()->setnormals(normalsy);
//	polydatay->getpointdata()->setscalars(scalars);
//
//	vtkpolydata* polydataz = vtkpolydata::new();
//	polydataz->setpoints(points);
//	polydataz->setverts(vertices);
//	polydataz->getpointdata()->setnormals(normalsz);
//	polydataz->getpointdata()->setscalars(scalars);
//
//	// ------------------------生成箭头并显示----------------------------
//	vtkarrowsource* arrow = vtkarrowsource::new();
//	arrow->update();
//
//	// x轴箭头（红色）
//	vtkglyph3d* glyphx = vtkglyph3d::new();
//	glyphx->setinputdata(polydatax);
//	glyphx->setsourcedata(arrow->getoutput());
//	glyphx->setscalefactor(0.2);  // 箭头大小，可根据牙齿尺寸调整
//	glyphx->setvectormodetousenormal();
//	glyphx->update();
//
//	vtksmartpointer<vtkpolydatamapper> glyphmapperx = vtksmartpointer<vtkpolydatamapper>::new();
//	glyphmapperx->setinputdata(glyphx->getoutput());
//	glyphmapperx->scalarvisibilityoff();
//
//	vtksmartpointer<vtkactor> glyphactorx = vtksmartpointer<vtkactor>::new();
//	glyphactorx->setmapper(glyphmapperx);
//	glyphactorx->getproperty()->setcolor(1, 0, 0);  // x轴：红色
//
//	// y轴箭头（绿色）
//	vtkglyph3d* glyphy = vtkglyph3d::new();
//	glyphy->setinputdata(polydatay);
//	glyphy->setsourcedata(arrow->getoutput());
//	glyphy->setscalefactor(0.2);
//	glyphy->setvectormodetousenormal();
//	glyphy->update();
//
//	vtksmartpointer<vtkpolydatamapper> glyphmappery = vtksmartpointer<vtkpolydatamapper>::new();
//	glyphmappery->setinputdata(glyphy->getoutput());
//	glyphmappery->scalarvisibilityoff();
//
//	vtksmartpointer<vtkactor> glyphactory = vtksmartpointer<vtkactor>::new();
//	glyphactory->setmapper(glyphmappery);
//	glyphactory->getproperty()->setcolor(0, 1, 0);  // y轴：绿色
//
//	// z轴箭头（蓝色）
//	vtkglyph3d* glyphz = vtkglyph3d::new();
//	glyphz->setinputdata(polydataz);
//	glyphz->setsourcedata(arrow->getoutput());
//	glyphz->setscalefactor(0.2);
//	glyphz->setvectormodetousenormal();
//	glyphz->update();
//
//	vtksmartpointer<vtkpolydatamapper> glyphmapperz = vtksmartpointer<vtkpolydatamapper>::new();
//	glyphmapperz->setinputdata(glyphz->getoutput());
//	glyphmapperz->scalarvisibilityoff();
//
//	vtksmartpointer<vtkactor> glyphactorz = vtksmartpointer<vtkactor>::new();
//	glyphactorz->setmapper(glyphmapperz);
//	glyphactorz->getproperty()->setcolor(0, 0, 1);  // z轴：蓝色
//
//	// 添加到渲染器并更新显示
//	m_rendercontainer->getrenderer()->addactor(glyphactorx);
//	m_rendercontainer->getrenderer()->addactor(glyphactory);
//	m_rendercontainer->getrenderer()->addactor(glyphactorz);
//	//vtkdoublearray* scalars = vtkdoublearray::new();  //创建标量属性，存入向量的长度length
//	//scalars->setnumberofcomponents(1);
//	////scalars->setname("scalars");
//	//scalars->insertnexttuple1(100);
//
//
//
//	//vtksmartpointer<vtkcellarray> vertices =
//	//	vtksmartpointer<vtkcellarray>::new();
//	//vertices->insertnextcell(vertex);	//将建立的拓扑用vtkcellarray封装，用于赋予vtkpolydata
//
//	//vtkpolydata* polydatax = vtkpolydata::new();  //创建几何实体
//	//polydatax->setpoints(points);	//赋予起点
//	//polydatax->setverts(vertices);	//赋予拓扑
//	//polydatax->getpointdata()->setnormals(normalsx);	//赋予向量朝向 
//	//polydatax->getpointdata()->setscalars(scalars);	//赋予向量长度
//	//vtkpolydata* polydatay = vtkpolydata::new();  //创建几何实体
//	//polydatay->setpoints(points);	//赋予起点
//	//polydatay->setverts(vertices);	//赋予拓扑
//	//polydatay->getpointdata()->setnormals(normalsy);	//赋予向量朝向 
//	//polydatay->getpointdata()->setscalars(scalars);	//赋予向量长度
//	//vtkpolydata* polydataz = vtkpolydata::new();  //创建几何实体
//	//polydataz->setpoints(points);	//赋予起点
//	//polydataz->setverts(vertices);	//赋予拓扑
//	//polydataz->getpointdata()->setnormals(normalsz);	//赋予向量朝向 
//	//polydataz->getpointdata()->setscalars(scalars);	//赋予向量长度
//
//	//vtkarrowsource* arrow = vtkarrowsource::new();
//	//arrow->update();
//
//	//vtkglyph3d* glyphx = vtkglyph3d::new();
//	//glyphx->setinputdata(polydatax);
//	//glyphx->setsourcedata(arrow->getoutput());
//	//glyphx->setscalefactor(0.1);
//	//glyphx->setvectormodetousenormal();
//	//glyphx->update();
//	//vtkglyph3d* glyphy = vtkglyph3d::new();
//	//glyphy->setinputdata(polydatay);
//	//glyphy->setsourcedata(arrow->getoutput());
//	//glyphy->setscalefactor(0.1);
//	//glyphy->setvectormodetousenormal();
//	//glyphy->update();
//	//vtkglyph3d* glyphz = vtkglyph3d::new();
//	//glyphz->setinputdata(polydataz);
//	//glyphz->setsourcedata(arrow->getoutput());
//	//glyphz->setscalefactor(0.1);
//	//glyphz->setvectormodetousenormal();
//	//glyphz->update();
//
//	//vtksmartpointer<vtkpolydatamapper> glyphmapperx =
//	//	vtksmartpointer<vtkpolydatamapper>::new();
//	//glyphmapperx->setinputdata(glyphx->getoutput());
//	//glyphmapperx->scalarvisibilityoff();
//	//vtksmartpointer<vtkactor> glyphactorx =
//	//	vtksmartpointer<vtkactor>::new();
//	//glyphactorx->setmapper(glyphmapperx);
//	//glyphactorx->getproperty()->setcolor(1, 0, 0);
//
//	//vtksmartpointer<vtkpolydatamapper> glyphmappery =
//	//	vtksmartpointer<vtkpolydatamapper>::new();
//	//glyphmappery->setinputdata(glyphy->getoutput());
//	//glyphmappery->scalarvisibilityoff();
//	//vtksmartpointer<vtkactor> glyphactory =
//	//	vtksmartpointer<vtkactor>::new();
//	//glyphactory->setmapper(glyphmappery);
//	//glyphactory->getproperty()->setcolor(0, 1, 0);
//
//	//vtksmartpointer<vtkpolydatamapper> glyphmapperz =
//	//	vtksmartpointer<vtkpolydatamapper>::new();
//	//glyphmapperz->setinputdata(glyphz->getoutput());
//	//glyphmapperz->scalarvisibilityoff();
//	//vtksmartpointer<vtkactor> glyphactorz =
//	//	vtksmartpointer<vtkactor>::new();
//	//glyphactorz->setmapper(glyphmapperz);
//	//glyphactorz->getproperty()->setcolor(0, 0, 1);
//
//
//	//
//
//
//	// m_rendercontainer->getrenderer()->addactor(glyphactorx);
//	// m_rendercontainer->getrenderer()->addactor(glyphactory);
//	// m_rendercontainer->getrenderer()->addactor(glyphactorz);
//
//	// //右侧
//	//for (int p = 8; p > 2; p--)
//	//{
//	//	//------------------------x轴----------------------------
//	//	//计算x轴上两个点的方向向量(终点 - 起点)—direction[3]
//	//	tooth_x[p][0] = tooth_center[p][0] - tooth_center[p - 1][0];;
//	//	tooth_x[p][1] = tooth_center[p][1] - tooth_center[p - 1][1];
//	//	tooth_x[p][2] = tooth_center[p][2] - tooth_center[p - 1][2];
//	//}
//	////左侧
//	//for (int p = 9; p < 16; p++)
//	//{
//	//	//取测试的单个牙齿
//	//	tooth* tooth = m_orthodata->gettooth((toothid)p);
//	//	tooth->gettoothactor()->setvisibility(true);
//	//	auto center = tooth->gettoothactor()->getcenter();
//	//	tooth_center[p][0] = center[0];
//	//	tooth_center[p][1] = center[1];
//	//	tooth_center[p][2] = center[2];
//	//}
//	//for (int p = 2; p < 16; p++)
//	//{
//	//	//取测试的单个牙齿
//	//	tooth* tooth1 = m_orthodata->gettooth((toothid)3);
//	//	tooth1->gettoothactor()->setvisibility(true);
//	//	/*vtksmartpointer<vtkpolydatamapper> mapper =
//	//		vtksmartpointer<vtkpolydatamapper>::new();
//	//	mapper->setinputdata(tooth1->gettoothpolydata());
//	//	vtksmartpointer<vtkactor>  actortest =
//	//		vtksmartpointer<vtkactor>::new();
//	//	actortest->setmapper(mapper);*/
//	//	//修改透明度和颜色
//	//	//actortest->getproperty()->setopacity(0.5);
//	//	//actortest->getproperty()->setcolor(1, 0, 0);
//	//	//m_rendercontainer->getrenderer(mainrenderer)->addactor(actortest);
//
//
//	//	//坐标轴
//	//	auto center = tooth1->gettoothactor()->getcenter();//原actor中心点
//	//	//保存畸形牙齿数据—初始牙齿编码
//	//	//auto center = tooth->gettoothactor()->getcenter();//原actor中心点
//	//	double cen[3] = { center[0], center[1], center[2]-0.5 };
//	//	vtkpoints* points = vtkpoints::new();  //记录起点坐标
//	//	points->insertnextpoint(cen);
//	//	vtkvertex* vertex = vtkvertex::new();  //建立起点的拓扑(不建立拓扑的话是不行的)
//	//	vertex->getpointids()->setnumberofids(points->getnumberofpoints());
//	//	for (int ij = 0; ij < points->getnumberofpoints(); ij++) {
//	//		vertex->getpointids()->setid(ij, ij);
//	//	}
//	//	////double normalizex[3] = { tooth_center[p][0] - tooth_center[p + 1][0], tooth_center[p][1] - tooth_center[p + 1][1], tooth_center[p][1] - tooth_center[p + 1][1] };
//	//	//double normalizex[3] = { tooth_center[p + 1][0] - tooth_center[p][0], 0, tooth_center[p + 1][1] - tooth_center[p][1] };
//	//	//////计算x轴向量模长—xd
//	//	//double xx = pow(normalizex[0], 2);//pow()幂次方函数
//	//	//double xy = pow(normalizex[1], 2);
//	//	//double xz = pow(normalizex[2], 2);
//	//	//double xd = sqrt(xx + xy + xz);//sqrt()平方根函数
//	//	////计算x轴单位向量——normalizex[3]
//	//	//normalizex[0] = sqrt(xx) / xd;
//	//	//normalizex[1] = sqrt(xy) / xd;
//	//	//normalizex[2] = sqrt(xz) / xd;
//	//	//
//	//	//double normalizey[3] = { 0, -1, 0 };
//	//	//double normalizez[3] = { normalizex[1] * normalizey[2] - normalizex[2] * normalizey[1],
//	//	//	normalizex[2] * normalizey[0] - normalizex[0] * normalizey[2],
//	//	//	normalizex[0] * normalizey[1] - normalizex[1] * normalizey[0] };
//	//	////---------------------z轴----------------------------------
//	//	////叉乘计算z轴单位向量normalizez[3]
//	//	///*tooth_normalizez[2][0] = tooth_normalizex[2][1] * tooth_normalizey[2][2] - tooth_normalizex[2][2] * tooth_normalizey[2][1];
//	//	//tooth_normalizez[2][1] = tooth_normalizex[2][2] * tooth_normalizey[2][0] - tooth_normalizex[2][0] * tooth_normalizey[2][2];
//	//	//tooth_normalizez[2][2] = tooth_normalizex[2][0] * tooth_normalizey[2][1] - tooth_normalizex[2][1] * tooth_normalizey[2][0];*/
//	//	//double normalizex[3] = { 0.5, 0, 1 };//6
//	//	//double normalizey[3] = { 0, -1, 0 };//6
//	//	//double normalizez[3] = { -1, 0, 0.5 };//6
//	//	double normalizex[3] = { 0.5, 0, 1 };
//	//	double normalizey[3] = { 0, -1, 0 };
//	//	double normalizez[3] = { -1, 0, 0.5 };
//
//	//	vtkdoublearray* normalsx = vtkdoublearray::new();  //创建法向量属性，存入向量的朝向target
//	//	normalsx->setnumberofcomponents(3);
//	//	normalsx->insertnexttuple(normalizex);
//	//	vtkdoublearray* normalsy = vtkdoublearray::new();  //创建法向量属性，存入向量的朝向target
//	//	normalsy->setnumberofcomponents(3);
//	//	normalsy->insertnexttuple(normalizey);
//	//	vtkdoublearray* normalsz = vtkdoublearray::new();  //创建法向量属性，存入向量的朝向target
//	//	normalsz->setnumberofcomponents(3);
//	//	normalsz->insertnexttuple(normalizez);
//
//	//	vtkdoublearray* scalars = vtkdoublearray::new();  //创建标量属性，存入向量的长度length
//	//	scalars->setnumberofcomponents(1);
//	//	//scalars->setname("scalars");
//	//	scalars->insertnexttuple1(100);
//
//
//	//	vtksmartpointer<vtkcellarray> vertices =
//	//		vtksmartpointer<vtkcellarray>::new();
//	//	vertices->insertnextcell(vertex);	//将建立的拓扑用vtkcellarray封装，用于赋予vtkpolydata
//
//	//	vtkpolydata* polydatax = vtkpolydata::new();  //创建几何实体
//	//	polydatax->setpoints(points);	//赋予起点
//	//	polydatax->setverts(vertices);	//赋予拓扑
//	//	polydatax->getpointdata()->setnormals(normalsx);	//赋予向量朝向 
//	//	polydatax->getpointdata()->setscalars(scalars);	//赋予向量长度
//	//	vtkpolydata* polydatay = vtkpolydata::new();  //创建几何实体
//	//	polydatay->setpoints(points);	//赋予起点
//	//	polydatay->setverts(vertices);	//赋予拓扑
//	//	polydatay->getpointdata()->setnormals(normalsy);	//赋予向量朝向 
//	//	polydatay->getpointdata()->setscalars(scalars);	//赋予向量长度
//	//	vtkpolydata* polydataz = vtkpolydata::new();  //创建几何实体
//	//	polydataz->setpoints(points);	//赋予起点
//	//	polydataz->setverts(vertices);	//赋予拓扑
//	//	polydataz->getpointdata()->setnormals(normalsz);	//赋予向量朝向 
//	//	polydataz->getpointdata()->setscalars(scalars);	//赋予向量长度
//
//	//	vtkarrowsource* arrow = vtkarrowsource::new();
//	//	arrow->update();
//
//	//	vtkglyph3d* glyphx = vtkglyph3d::new();
//	//	glyphx->setinputdata(polydatax);
//	//	glyphx->setsourcedata(arrow->getoutput());
//	//	glyphx->setscalefactor(0.1);
//	//	glyphx->setvectormodetousenormal();
//	//	glyphx->update();
//	//	vtkglyph3d* glyphy = vtkglyph3d::new();
//	//	glyphy->setinputdata(polydatay);
//	//	glyphy->setsourcedata(arrow->getoutput());
//	//	glyphy->setscalefactor(0.1);
//	//	glyphy->setvectormodetousenormal();
//	//	glyphy->update();
//	//	vtkglyph3d* glyphz = vtkglyph3d::new();
//	//	glyphz->setinputdata(polydataz);
//	//	glyphz->setsourcedata(arrow->getoutput());
//	//	glyphz->setscalefactor(0.1);
//	//	glyphz->setvectormodetousenormal();
//	//	glyphz->update();
//
//	//	vtksmartpointer<vtkpolydatamapper> glyphmapperx =
//	//		vtksmartpointer<vtkpolydatamapper>::new();
//	//	glyphmapperx->setinputdata(glyphx->getoutput());
//	//	glyphmapperx->scalarvisibilityoff();
//	//	vtksmartpointer<vtkactor> glyphactorx =
//	//		vtksmartpointer<vtkactor>::new();
//	//	glyphactorx->setmapper(glyphmapperx);
//	//	glyphactorx->getproperty()->setcolor(1, 0, 0);//红色
//
//	//	vtksmartpointer<vtkpolydatamapper> glyphmappery =
//	//		vtksmartpointer<vtkpolydatamapper>::new();
//	//	glyphmappery->setinputdata(glyphy->getoutput());
//	//	glyphmappery->scalarvisibilityoff();
//	//	vtksmartpointer<vtkactor> glyphactory =
//	//		vtksmartpointer<vtkactor>::new();
//	//	glyphactory->setmapper(glyphmappery);
//	//	glyphactory->getproperty()->setcolor(0, 1, 0);//绿
//
//	//	vtksmartpointer<vtkpolydatamapper> glyphmapperz =
//	//		vtksmartpointer<vtkpolydatamapper>::new();
//	//	glyphmapperz->setinputdata(glyphz->getoutput());
//	//	glyphmapperz->scalarvisibilityoff();
//	//	vtksmartpointer<vtkactor> glyphactorz =
//	//		vtksmartpointer<vtkactor>::new();
//	//	glyphactorz->setmapper(glyphmapperz);
//	//	glyphactorz->getproperty()->setcolor(0, 0, 1);//蓝
//
//
//
//
//
//	//	m_rendercontainer->getrenderer()->addactor(glyphactorx);
//	//	m_rendercontainer->getrenderer()->addactor(glyphactory);
//	//	m_rendercontainer->getrenderer()->addactor(glyphactorz);
//
//
//
//
//
//	//}
//	 //------------------------------------------------------旋转分量测试-----------------------------------------------------
//	m_rendercontainer->render();//可视化牙齿目标位
//
//}
void GroupPathPlanningPSO::Axes() {
	double Tooth_center[32][3];
	double Tooth_X[32][3];
	double Tooth_Y[32][3];
	double Tooth_Z[32][3];
	double Tooth_normalizeX[32][3];
	double Tooth_normalizeY[32][3];
	double Tooth_normalizeZ[32][3];
	// 只显示第17颗牙齿，其余隐藏
	const int targetToothId = 17; // 目标牙齿编号（上颌右侧第三磨牙，见图2c）
	for (int p = 17; p <= 31; p++) {
		Tooth* tooth = m_OrthoData->GetTooth((ToothId)p);
		if (!tooth) continue;
		if (p == targetToothId) {
			tooth->GetToothActor()->SetVisibility(true);
			tooth->GetToothActor()->GetProperty()->SetOpacity(1.0);
			tooth->GetToothActor()->GetProperty()->SetColor(0.8, 0.8, 0.8); // 浅灰
		}
		else {
			tooth->GetToothActor()->SetVisibility(false);
		}
	}

	Tooth* targetTooth = m_OrthoData->GetTooth((ToothId)targetToothId);
	if (!targetTooth) return;

	// 1. 获取第17颗牙齿的中心
	double center[3];
	{
		auto c = targetTooth->GetToothActor()->GetCenter();
		center[0] = c[0];
		center[1] = c[1];
		center[2] = c[2];
	}

	// 2. 构建局部坐标系，参考论文图2(c)和正交性
	// X轴（红）：近远中方向（指向16号牙齿中心）
	double X[3] = { 1, 0, 0 };
	Tooth* neighbor16 = m_OrthoData->GetTooth((ToothId)16);
	if (neighbor16) {
		auto c16 = neighbor16->GetToothActor()->GetCenter();
		X[0] = c16[0] - center[0];
		X[1] = c16[1] - center[1];
		X[2] = c16[2] - center[2];
	}
	double X_norm = sqrt(X[0] * X[0] + X[1] * X[1] + X[2] * X[2]);
	double X_dir[3] = { X[0] / X_norm, X[1] / X_norm, X[2] / X_norm };

	// Y轴（绿）：牙冠到牙龈方向（上颌磨牙近似世界Z轴正向）
	double Y[3] = { 0, 0, 1 }; // 可以根据模型调整
	double Y_norm = sqrt(Y[0] * Y[0] + Y[1] * Y[1] + Y[2] * Y[2]);
	double Y_dir[3] = { Y[0] / Y_norm, Y[1] / Y_norm, Y[2] / Y_norm };

	// Z轴（蓝）：正交方向（X×Y，指向腭/舌侧）
	double Z_dir[3];
	Z_dir[0] = X_dir[1] * Y_dir[2] - X_dir[2] * Y_dir[1];
	Z_dir[1] = X_dir[2] * Y_dir[0] - X_dir[0] * Y_dir[2];
	Z_dir[2] = X_dir[0] * Y_dir[1] - X_dir[1] * Y_dir[0];
	double Z_norm = sqrt(Z_dir[0] * Z_dir[0] + Z_dir[1] * Z_dir[1] + Z_dir[2] * Z_dir[2]);
	Z_dir[0] /= Z_norm; Z_dir[1] /= Z_norm; Z_dir[2] /= Z_norm;

	// 3. 箭头长度与比例，适合牙齿展示
	double bounds[6];
	targetTooth->GetToothActor()->GetBounds(bounds);
	double maxDim = std::max({ bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4] });
	double arrowLength = maxDim * 0.3; // 箭头为包围盒最大边长的30%

	// 4. VTK点和法向量
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(center);

	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkVertex> vertex = vtkSmartPointer<vtkVertex>::New();
	vertex->GetPointIds()->SetNumberOfIds(1);
	vertex->GetPointIds()->SetId(0, 0);
	vertices->InsertNextCell(vertex);

	// X轴
	vtkSmartPointer<vtkDoubleArray> normalsX = vtkSmartPointer<vtkDoubleArray>::New();
	normalsX->SetNumberOfComponents(3);
	normalsX->InsertNextTuple(X_dir);
	vtkSmartPointer<vtkDoubleArray> scalars = vtkSmartPointer<vtkDoubleArray>::New();
	scalars->SetNumberOfComponents(1);
	scalars->InsertNextTuple1(arrowLength);
	vtkSmartPointer<vtkPolyData> polydataX = vtkSmartPointer<vtkPolyData>::New();
	polydataX->SetPoints(points);
	polydataX->SetVerts(vertices);
	polydataX->GetPointData()->SetNormals(normalsX);
	polydataX->GetPointData()->SetScalars(scalars);

	// Y轴
	vtkSmartPointer<vtkDoubleArray> normalsY = vtkSmartPointer<vtkDoubleArray>::New();
	normalsY->SetNumberOfComponents(3);
	normalsY->InsertNextTuple(Y_dir);
	vtkSmartPointer<vtkPolyData> polydataY = vtkSmartPointer<vtkPolyData>::New();
	polydataY->SetPoints(points);
	polydataY->SetVerts(vertices);
	polydataY->GetPointData()->SetNormals(normalsY);
	polydataY->GetPointData()->SetScalars(scalars);

	// Z轴
	vtkSmartPointer<vtkDoubleArray> normalsZ = vtkSmartPointer<vtkDoubleArray>::New();
	normalsZ->SetNumberOfComponents(3);
	normalsZ->InsertNextTuple(Z_dir);
	vtkSmartPointer<vtkPolyData> polydataZ = vtkSmartPointer<vtkPolyData>::New();
	polydataZ->SetPoints(points);
	polydataZ->SetVerts(vertices);
	polydataZ->GetPointData()->SetNormals(normalsZ);
	polydataZ->GetPointData()->SetScalars(scalars);

	// 箭头模型
	vtkSmartPointer<vtkArrowSource> arrow = vtkSmartPointer<vtkArrowSource>::New();
	arrow->SetTipLength(0.3);
	arrow->SetTipRadius(0.08);
	arrow->SetShaftRadius(0.02);
	arrow->Update();

	// X轴（红色）
	vtkSmartPointer<vtkGlyph3D> glyphX = vtkSmartPointer<vtkGlyph3D>::New();
	glyphX->SetInputData(polydataX);
	glyphX->SetSourceData(arrow->GetOutput());
	glyphX->SetScaleFactor(1.0); // 已用scalars控制长度
	glyphX->SetVectorModeToUseNormal();
	glyphX->Update();
	vtkSmartPointer<vtkPolyDataMapper> glyphMapperX = vtkSmartPointer<vtkPolyDataMapper>::New();
	glyphMapperX->SetInputData(glyphX->GetOutput());
	glyphMapperX->ScalarVisibilityOff();
	vtkSmartPointer<vtkActor> glyphActorX = vtkSmartPointer<vtkActor>::New();
	glyphActorX->SetMapper(glyphMapperX);
	glyphActorX->GetProperty()->SetColor(1, 0, 0);

	// Y轴（绿色）
	vtkSmartPointer<vtkGlyph3D> glyphY = vtkSmartPointer<vtkGlyph3D>::New();
	glyphY->SetInputData(polydataY);
	glyphY->SetSourceData(arrow->GetOutput());
	glyphY->SetScaleFactor(1.0);
	glyphY->SetVectorModeToUseNormal();
	glyphY->Update();
	vtkSmartPointer<vtkPolyDataMapper> glyphMapperY = vtkSmartPointer<vtkPolyDataMapper>::New();
	glyphMapperY->SetInputData(glyphY->GetOutput());
	glyphMapperY->ScalarVisibilityOff();
	vtkSmartPointer<vtkActor> glyphActorY = vtkSmartPointer<vtkActor>::New();
	glyphActorY->SetMapper(glyphMapperY);
	glyphActorY->GetProperty()->SetColor(0, 1, 0);

	// Z轴（蓝色）
	vtkSmartPointer<vtkGlyph3D> glyphZ = vtkSmartPointer<vtkGlyph3D>::New();
	glyphZ->SetInputData(polydataZ);
	glyphZ->SetSourceData(arrow->GetOutput());
	glyphZ->SetScaleFactor(1.0);
	glyphZ->SetVectorModeToUseNormal();
	glyphZ->Update();
	vtkSmartPointer<vtkPolyDataMapper> glyphMapperZ = vtkSmartPointer<vtkPolyDataMapper>::New();
	glyphMapperZ->SetInputData(glyphZ->GetOutput());
	glyphMapperZ->ScalarVisibilityOff();
	vtkSmartPointer<vtkActor> glyphActorZ = vtkSmartPointer<vtkActor>::New();
	glyphActorZ->SetMapper(glyphMapperZ);
	glyphActorZ->GetProperty()->SetColor(0, 0, 1);

	// 5. 添加Actor到渲染器
	vtkRenderer* renderer = m_RenderContainer->GetRenderer();
	renderer->AddActor(glyphActorX);
	renderer->AddActor(glyphActorY);
	renderer->AddActor(glyphActorZ);

	// 6. 调整相机：居中并适合显示
	double center_bbox[3] = {
		(bounds[0] + bounds[1]) / 2,
		(bounds[2] + bounds[3]) / 2,
		(bounds[4] + bounds[5]) / 2
	};
	renderer->GetActiveCamera()->SetFocalPoint(center_bbox);
	renderer->GetActiveCamera()->SetPosition(center_bbox[0], center_bbox[1], center_bbox[2] + 2 * maxDim);
	renderer->GetActiveCamera()->SetViewUp(0, 1, 0);
	renderer->GetActiveCamera()->Zoom(1.5);
	renderer->ResetCamera();
	m_RenderContainer->Render();
}


//
// 
void GroupPathPlanningPSO::BuildToothLocalAxes() {
	const int targetToothId = 8; // 目标牙齿编号
	// 1. 仅显示目标牙齿
	for (int p = 2; p < 16; p++) {
		Tooth* tooth = m_OrthoData->GetTooth((ToothId)p);
		if (p == targetToothId) {
			tooth->GetToothActor()->SetVisibility(true);
			tooth->GetToothActor()->GetProperty()->SetOpacity(1.0);
			tooth->GetToothActor()->GetProperty()->SetColor(0.8, 0.8, 0.8);
		}
		else {
			tooth->GetToothActor()->SetVisibility(false);
		}
	}

	Tooth* targetTooth = m_OrthoData->GetTooth((ToothId)targetToothId);
	vtkPolyData* toothPoly = vtkPolyData::SafeDownCast(
		targetTooth->GetToothActor()->GetMapper()->GetInput()
	);
	if (!toothPoly) return;

	// 2. 获取牙齿中心
	double* c = targetTooth->GetToothActor()->GetCenter();
	double center[3] = { c[0], c[1], c[2] };

	// ----------------- 需提前获得全口牙弓点集和咬合平面法向 -----------------
	// 这里仅做示例：假定咬合平面近似为Z轴，牙弓切线近似为X轴，Y轴为右手正交
	double zAxis[3] = { 0, 0, 1 }; // Z轴：咬合垂直向，正方向指向上颌
	double xAxis[3] = { 1, 0, 0 }; // X轴：牙弓前后向，正方向指向远中
	double yAxis[3];

	// 保证归一化
	vtkMath::Normalize(zAxis);
	vtkMath::Normalize(xAxis);
	vtkMath::Cross(zAxis, xAxis, yAxis);
	vtkMath::Normalize(yAxis);

	// ------- 将绿色箭头（Y轴）反向 -------
	yAxis[0] = -yAxis[0];
	yAxis[1] = -yAxis[1];
	yAxis[2] = -yAxis[2];

	// 3. VTK箭头生成与显示
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(center);

	vtkSmartPointer<vtkVertex> vertex = vtkSmartPointer<vtkVertex>::New();
	vertex->GetPointIds()->SetNumberOfIds(1);
	vertex->GetPointIds()->SetId(0, 0);

	vtkSmartPointer<vtkDoubleArray> normalsX = vtkSmartPointer<vtkDoubleArray>::New();
	normalsX->SetNumberOfComponents(3);
	normalsX->InsertNextTuple(xAxis);
	vtkSmartPointer<vtkDoubleArray> normalsY = vtkSmartPointer<vtkDoubleArray>::New();
	normalsY->SetNumberOfComponents(3);
	normalsY->InsertNextTuple(yAxis);
	vtkSmartPointer<vtkDoubleArray> normalsZ = vtkSmartPointer<vtkDoubleArray>::New();
	normalsZ->SetNumberOfComponents(3);
	normalsZ->InsertNextTuple(zAxis);

	double bounds[6];
	targetTooth->GetToothActor()->GetBounds(bounds);
	double maxDim = std::max({ bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4] });
	double arrowLength = maxDim * 1.2;

	vtkSmartPointer<vtkDoubleArray> scalars = vtkSmartPointer<vtkDoubleArray>::New();
	scalars->SetNumberOfComponents(1);
	scalars->InsertNextTuple1(arrowLength);

	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	vertices->InsertNextCell(vertex);

	auto makePoly = [&](vtkDoubleArray* nrm) {
		vtkSmartPointer<vtkPolyData> poly = vtkSmartPointer<vtkPolyData>::New();
		poly->SetPoints(points);
		poly->SetVerts(vertices);
		poly->GetPointData()->SetNormals(nrm);
		poly->GetPointData()->SetScalars(scalars);
		return poly;
	};
	auto polydataX = makePoly(normalsX);
	auto polydataY = makePoly(normalsY);
	auto polydataZ = makePoly(normalsZ);

	vtkSmartPointer<vtkArrowSource> arrow = vtkSmartPointer<vtkArrowSource>::New();
	arrow->SetTipLength(0.45);
	arrow->SetTipRadius(0.06);
	arrow->SetShaftRadius(0.018);
	arrow->SetTipResolution(20);
	arrow->SetShaftResolution(20);
	arrow->Update();

	auto makeGlyph = [&](vtkPolyData* poly) {
		vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
		glyph->SetInputData(poly);
		glyph->SetSourceConnection(arrow->GetOutputPort());
		glyph->SetScaleFactor(1.0);
		glyph->SetVectorModeToUseNormal();
		glyph->Update();
		return glyph;
	};

	auto makeActor = [&](vtkGlyph3D* glyph, double r, double g, double b) {
		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(glyph->GetOutputPort());
		mapper->ScalarVisibilityOff();
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(mapper);
		actor->GetProperty()->SetColor(r, g, b);
		return actor;
	};

	// X-红，Y-绿（反向），Z-蓝
	auto glyphX = makeGlyph(polydataX);
	auto glyphY = makeGlyph(polydataY);
	auto glyphZ = makeGlyph(polydataZ);

	auto actorX = makeActor(glyphX, 1, 0, 0);
	auto actorY = makeActor(glyphY, 0, 1, 0);
	auto actorZ = makeActor(glyphZ, 0, 0, 1);

	m_RenderContainer->GetRenderer()->AddActor(actorX);
	m_RenderContainer->GetRenderer()->AddActor(actorY);
	m_RenderContainer->GetRenderer()->AddActor(actorZ);
	m_RenderContainer->Render();
}
// 尖牙
// void GroupPathPlanningPSO::Axes() {
	// const int TARGET_TOOTH_ID = 6;         // 目标牙（例如尖牙）
	// const int NEIGHBOR_TOOTH_ID = 7;       // 邻牙（用于定义X轴方向）

	// double targetCenter[3] = { 0.0 };
	// double neighborCenter[3] = { 0.0 };
	// vtkSmartPointer<vtkPolyData> targetPolyData = nullptr;
	// Tooth* targetTooth = nullptr;

	1. 收集目标牙和邻牙数据，并隐藏其他牙齿
	// for (int toothId = 2; toothId < 16; ++toothId) {
		// Tooth* tooth = m_OrthoData->GetTooth((ToothId)toothId);
		// if (!tooth) continue;

		// if (toothId == TARGET_TOOTH_ID) {
			// double* center = tooth->GetToothActor()->GetCenter();
			// targetCenter[0] = center[0];
			// targetCenter[1] = center[1];
			// targetCenter[2] = center[2];
			// targetTooth = tooth;

			// targetPolyData = vtkSmartPointer<vtkPolyData>::New();
			// targetPolyData->DeepCopy(tooth->GetToothPolyData());

			显示目标牙
			// tooth->GetToothActor()->SetVisibility(1);
		// }
		// else {
			隐藏其他牙
			// tooth->GetToothActor()->SetVisibility(0);

			// if (toothId == NEIGHBOR_TOOTH_ID) {
				// double* center = tooth->GetToothActor()->GetCenter();
				// neighborCenter[0] = center[0];
				// neighborCenter[1] = center[1];
				// neighborCenter[2] = center[2];
			// }
		// }
	// }

	// if (!targetTooth || !targetPolyData || targetPolyData->GetNumberOfPoints() == 0) {
		// m_RenderContainer->Render();
		// return;
	// }

	2. 原点
	// double origin[3] = { targetCenter[0], targetCenter[1], targetCenter[2] };

	3. Z轴：牙尖方向
	// double cuspPoint[3] = { 0.0 };
	// double maxZ = -VTK_DOUBLE_MAX;
	// for (vtkIdType i = 0; i < targetPolyData->GetNumberOfPoints(); ++i) {
		// double point[3];
		// targetPolyData->GetPoint(i, point);
		// if (point[2] > maxZ) {
			// maxZ = point[2];
			// cuspPoint[0] = point[0];
			// cuspPoint[1] = point[1];
			// cuspPoint[2] = point[2];
		// }
	// }
	// double zAxis[3];
	// vtkMath::Subtract(cuspPoint, origin, zAxis);
	// vtkMath::Normalize(zAxis);

	4. X轴：邻牙方向
	// double xAxis[3];
	// vtkMath::Subtract(neighborCenter, origin, xAxis);
	// vtkMath::Normalize(xAxis);

	5. Y轴：Z × X
	// double yAxis[3];
	// vtkMath::Cross(zAxis, xAxis, yAxis);
	// vtkMath::Normalize(yAxis);

	=== 反转绿色坐标轴 ===
	// yAxis[0] = -yAxis[0];
	// yAxis[1] = -yAxis[1];
	// yAxis[2] = -yAxis[2];

	6. 修正X轴，确保正交
	// vtkMath::Cross(yAxis, zAxis, xAxis);
	// vtkMath::Normalize(xAxis);

	==============================
	可视化箭头
	==============================
	// double bounds[6];
	// targetTooth->GetToothActor()->GetBounds(bounds);
	// double toothSize = std::max({
		// bounds[1] - bounds[0],
		// bounds[3] - bounds[2],
		// bounds[5] - bounds[4]
		// });
	// double arrowLength = toothSize * 1.0;

	// vtkSmartPointer<vtkPoints> originPoints = vtkSmartPointer<vtkPoints>::New();
	// originPoints->InsertNextPoint(origin);

	// vtkSmartPointer<vtkVertex> vertex = vtkSmartPointer<vtkVertex>::New();
	// vertex->GetPointIds()->SetId(0, 0);

	// vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	// vertices->InsertNextCell(vertex);

	// auto createNormalArray = [](const double axis[3]) {
		// vtkSmartPointer<vtkDoubleArray> normals = vtkSmartPointer<vtkDoubleArray>::New();
		// normals->SetNumberOfComponents(3);
		// normals->InsertNextTuple(axis);
		// return normals;
	// };
	// auto xNormals = createNormalArray(xAxis);
	// auto yNormals = createNormalArray(yAxis);
	// auto zNormals = createNormalArray(zAxis);

	// vtkSmartPointer<vtkDoubleArray> scalars = vtkSmartPointer<vtkDoubleArray>::New();
	// scalars->SetNumberOfComponents(1);
	// scalars->InsertNextTuple1(arrowLength);

	// auto createPolyData = [&](vtkDoubleArray* normals) {
		// vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
		// polyData->SetPoints(originPoints);
		// polyData->SetVerts(vertices);
		// polyData->GetPointData()->SetNormals(normals);
		// polyData->GetPointData()->SetScalars(scalars);
		// return polyData;
	// };
	// auto xPolyData = createPolyData(xNormals);
	// auto yPolyData = createPolyData(yNormals);
	// auto zPolyData = createPolyData(zNormals);

	// vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();
	// arrowSource->SetTipLength(0.3);
	// arrowSource->SetTipRadius(0.08);
	// arrowSource->SetShaftRadius(0.02);
	// arrowSource->SetTipResolution(24);
	// arrowSource->SetShaftResolution(24);
	// arrowSource->Update();

	// auto createGlyph = [&](vtkPolyData* polyData) {
		// vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
		// glyph->SetInputData(polyData);
		// glyph->SetSourceConnection(arrowSource->GetOutputPort());
		// glyph->SetScaleFactor(1.0);
		// glyph->SetVectorModeToUseNormal();
		// glyph->Update();
		// return glyph;
	// };
	// auto xGlyph = createGlyph(xPolyData);
	// auto yGlyph = createGlyph(yPolyData);
	// auto zGlyph = createGlyph(zPolyData);

	// auto createActor = [&](vtkGlyph3D* glyph, double r, double g, double b) {
		// vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		// mapper->SetInputConnection(glyph->GetOutputPort());
		// mapper->ScalarVisibilityOff();

		// vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		// actor->SetMapper(mapper);
		// actor->GetProperty()->SetColor(r, g, b);
		// return actor;
	// };

	// auto xActor = createActor(xGlyph, 1.0, 0.0, 0.0);  // 红: X
	// auto yActor = createActor(yGlyph, 0.0, 1.0, 0.0);  // 绿: Y (反向后显示)
	// auto zActor = createActor(zGlyph, 0.0, 0.0, 1.0);  // 蓝: Z

	// m_RenderContainer->GetRenderer()->AddActor(xActor);
	// m_RenderContainer->GetRenderer()->AddActor(yActor);
	// m_RenderContainer->GetRenderer()->AddActor(zActor);
	// m_RenderContainer->Render();
// }


//下牙颌
//void GroupPathPlanningPSO::Axes() {
//	// 设置目标牙齿编号，改为第三颗牙齿
//	const int targetToothId = 8; // 3号为尖牙
//
//	// 其它牙齿隐藏，仅目标牙齿全显
//	for (int p = 2; p < 16; p++) {
//		Tooth* tooth = m_OrthoData->GetTooth((ToothId)p);
//		if (!tooth) continue;
//		if (p == targetToothId) {
//			tooth->GetToothActor()->SetVisibility(true);
//			tooth->GetToothActor()->GetProperty()->SetOpacity(1.0);
//			tooth->GetToothActor()->GetProperty()->SetColor(0.8, 0.8, 0.8);
//		}
//		else {
//			tooth->GetToothActor()->SetVisibility(false);
//		}
//	}
//
//	Tooth* targetTooth = m_OrthoData->GetTooth((ToothId)targetToothId);
//	if (!targetTooth) return;
//
//	// 1. 获取目标牙齿中心
//	double center[3];
//	{
//		auto c = targetTooth->GetToothActor()->GetCenter();
//		center[0] = c[0];
//		center[1] = c[1];
//		center[2] = c[2];
//	}
//
//	// 2. 计算局部坐标系
//	// Z轴：牙齿长轴（尖牙近似世界Z轴正方向）
//	double Z[3] = { 0, 0, 1 }; // 可根据需要替换为实际牙齿主轴
//	double Z_len = sqrt(Z[0] * Z[0] + Z[1] * Z[1] + Z[2] * Z[2]);
//	double Z_dir[3] = { Z[0] / Z_len, Z[1] / Z_len, Z[2] / Z_len };
//
//	// X轴：指向邻牙（通常为4号牙齿）
//	int neighborId = targetToothId + 1;  // 近中方向（可根据实际排布调整）
//	if (neighborId >= 16) neighborId = targetToothId - 1;
//	Tooth* neighborTooth = m_OrthoData->GetTooth((ToothId)neighborId);
//	double X[3] = { 1, 0, 0 }; // fallback
//	if (neighborTooth) {
//		auto nc = neighborTooth->GetToothActor()->GetCenter();
//		X[0] = nc[0] - center[0];
//		X[1] = nc[1] - center[1];
//		X[2] = nc[2] - center[2];
//	}
//	// X正交化
//	double dotXZ = X[0] * Z_dir[0] + X[1] * Z_dir[1] + X[2] * Z_dir[2];
//	double X_proj[3] = { X[0] - dotXZ * Z_dir[0], X[1] - dotXZ * Z_dir[1], X[2] - dotXZ * Z_dir[2] };
//	double X_len = sqrt(X_proj[0] * X_proj[0] + X_proj[1] * X_proj[1] + X_proj[2] * X_proj[2]);
//	double X_dir[3] = { X_proj[0] / X_len, X_proj[1] / X_len, X_proj[2] / X_len };
//
//	// Y轴：右手系正交，并反向（与论文图一致）
//	double Y_dir[3];
//	Y_dir[0] = Z_dir[1] * X_dir[2] - Z_dir[2] * X_dir[1];
//	Y_dir[1] = Z_dir[2] * X_dir[0] - Z_dir[0] * X_dir[2];
//	Y_dir[2] = Z_dir[0] * X_dir[1] - Z_dir[1] * X_dir[0];
//	// 反向Y
//	Y_dir[0] = -Y_dir[0];
//	Y_dir[1] = -Y_dir[1];
//	Y_dir[2] = -Y_dir[2];
//
//	// 3. 箭头长度适配牙齿体积 (适中)
//	double bounds[6];
//	targetTooth->GetToothActor()->GetBounds(bounds);
//	double maxDim = std::max({ bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4] });
//	double arrowLength = maxDim * 2;
//
//	// 4. VTK数据结构
//	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//	points->InsertNextPoint(center);
//	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
//	vtkSmartPointer<vtkVertex> vertex = vtkSmartPointer<vtkVertex>::New();
//	vertex->GetPointIds()->SetNumberOfIds(1);
//	vertex->GetPointIds()->SetId(0, 0);
//	vertices->InsertNextCell(vertex);
//
//	vtkSmartPointer<vtkDoubleArray> normalsX = vtkSmartPointer<vtkDoubleArray>::New();
//	normalsX->SetNumberOfComponents(3);
//	normalsX->InsertNextTuple(X_dir);
//	vtkSmartPointer<vtkDoubleArray> normalsY = vtkSmartPointer<vtkDoubleArray>::New();
//	normalsY->SetNumberOfComponents(3);
//	normalsY->InsertNextTuple(Y_dir);
//	vtkSmartPointer<vtkDoubleArray> normalsZ = vtkSmartPointer<vtkDoubleArray>::New();
//	normalsZ->SetNumberOfComponents(3);
//	normalsZ->InsertNextTuple(Z_dir);
//
//	vtkSmartPointer<vtkDoubleArray> scalars = vtkSmartPointer<vtkDoubleArray>::New();
//	scalars->SetNumberOfComponents(1);
//	scalars->InsertNextTuple1(arrowLength);
//
//	vtkSmartPointer<vtkPolyData> polydataX = vtkSmartPointer<vtkPolyData>::New();
//	polydataX->SetPoints(points);
//	polydataX->SetVerts(vertices);
//	polydataX->GetPointData()->SetNormals(normalsX);
//	polydataX->GetPointData()->SetScalars(scalars);
//	vtkSmartPointer<vtkPolyData> polydataY = vtkSmartPointer<vtkPolyData>::New();
//	polydataY->SetPoints(points);
//	polydataY->SetVerts(vertices);
//	polydataY->GetPointData()->SetNormals(normalsY);
//	polydataY->GetPointData()->SetScalars(scalars);
//	vtkSmartPointer<vtkPolyData> polydataZ = vtkSmartPointer<vtkPolyData>::New();
//	polydataZ->SetPoints(points);
//	polydataZ->SetVerts(vertices);
//	polydataZ->GetPointData()->SetNormals(normalsZ);
//	polydataZ->GetPointData()->SetScalars(scalars);
//
//	// 箭头模型适度（可自行调整为细长效果）
//	vtkSmartPointer<vtkArrowSource> arrow = vtkSmartPointer<vtkArrowSource>::New();
//	arrow->SetTipLength(0.3);
//	arrow->SetTipRadius(0.12);
//	arrow->SetShaftRadius(0.03);
//	arrow->Update();
//
//	// Glyph整体缩放适中
//	vtkSmartPointer<vtkGlyph3D> glyphX = vtkSmartPointer<vtkGlyph3D>::New();
//	glyphX->SetInputData(polydataX);
//	glyphX->SetSourceData(arrow->GetOutput());
//	glyphX->SetScaleFactor(1.0);
//	glyphX->SetVectorModeToUseNormal();
//	glyphX->Update();
//	vtkSmartPointer<vtkPolyDataMapper> glyphMapperX = vtkSmartPointer<vtkPolyDataMapper>::New();
//	glyphMapperX->SetInputData(glyphX->GetOutput());
//	glyphMapperX->ScalarVisibilityOff();
//	vtkSmartPointer<vtkActor> glyphActorX = vtkSmartPointer<vtkActor>::New();
//	glyphActorX->SetMapper(glyphMapperX);
//	glyphActorX->GetProperty()->SetColor(1, 0, 0);//红
//
//	vtkSmartPointer<vtkGlyph3D> glyphY = vtkSmartPointer<vtkGlyph3D>::New();
//	glyphY->SetInputData(polydataY);
//	glyphY->SetSourceData(arrow->GetOutput());
//	glyphY->SetScaleFactor(1.0);
//	glyphY->SetVectorModeToUseNormal();
//	glyphY->Update();
//	vtkSmartPointer<vtkPolyDataMapper> glyphMapperY = vtkSmartPointer<vtkPolyDataMapper>::New();
//	glyphMapperY->SetInputData(glyphY->GetOutput());
//	glyphMapperY->ScalarVisibilityOff();
//	vtkSmartPointer<vtkActor> glyphActorY = vtkSmartPointer<vtkActor>::New();
//	glyphActorY->SetMapper(glyphMapperY);
//	glyphActorY->GetProperty()->SetColor(0, 1, 0);//绿
//
//	vtkSmartPointer<vtkGlyph3D> glyphZ = vtkSmartPointer<vtkGlyph3D>::New();
//	glyphZ->SetInputData(polydataZ);
//	glyphZ->SetSourceData(arrow->GetOutput());
//	glyphZ->SetScaleFactor(1.0);
//	glyphZ->SetVectorModeToUseNormal();
//	glyphZ->Update();
//	vtkSmartPointer<vtkPolyDataMapper> glyphMapperZ = vtkSmartPointer<vtkPolyDataMapper>::New();
//	glyphMapperZ->SetInputData(glyphZ->GetOutput());
//	glyphMapperZ->ScalarVisibilityOff();
//	vtkSmartPointer<vtkActor> glyphActorZ = vtkSmartPointer<vtkActor>::New();
//	glyphActorZ->SetMapper(glyphMapperZ);
//	glyphActorZ->GetProperty()->SetColor(0, 0, 1);//蓝
//
//	m_RenderContainer->GetRenderer()->AddActor(glyphActorX);
//	m_RenderContainer->GetRenderer()->AddActor(glyphActorY);
//	m_RenderContainer->GetRenderer()->AddActor(glyphActorZ);
//
//	// 相机自动聚焦（如需可恢复下方注释代码）
//	//double bbox_center[3] = {
//	//    (bounds[0] + bounds[1]) / 2,
//	//    (bounds[2] + bounds[3]) / 2,
//	//    (bounds[4] + bounds[5]) / 2
//	//};
//	//m_RenderContainer->GetRenderer()->GetActiveCamera()->SetFocalPoint(bbox_center);
//	//m_RenderContainer->GetRenderer()->GetActiveCamera()->SetPosition(bbox_center[0], bbox_center[1], bbox_center[2] + 2 * maxDim);
//	//m_RenderContainer->GetRenderer()->GetActiveCamera()->SetViewUp(0, 1, 0);
//	//m_RenderContainer->GetRenderer()->GetActiveCamera()->Zoom(1);
//	//m_RenderContainer->GetRenderer()->ResetCamera();
//
//	m_RenderContainer->Render();
//}

void GroupPathPlanningPSO::Grow_AxesZ() {
	/*
	2-15   tooth_grow_center[toothid][center]
	*/
	double tooth_grow_center[16][3];
	tooth_grow_center[2][0]=-5.7;
	tooth_grow_center[2][1]=2.3;
	tooth_grow_center[2][2]=0;
	tooth_grow_center[3][0]=-5.7;
	tooth_grow_center[3][1]=2.3;
	tooth_grow_center[3][2]=2;
	tooth_grow_center[4][0]=-5;
	tooth_grow_center[4][1]=2.3;
	tooth_grow_center[4][2]=2;
	tooth_grow_center[5][0]=-4.8;
	tooth_grow_center[5][1]=2.3;
	tooth_grow_center[5][2]=2;
	tooth_grow_center[6][0] = 10;
	tooth_grow_center[6][1] = 10;
	tooth_grow_center[6][2] = 10;
	tooth_grow_center[7][0]=-0.5;
	tooth_grow_center[7][1]=2.8;
	tooth_grow_center[7][2]=3;
	tooth_grow_center[8][0] = -2.4; 
	tooth_grow_center[8][1]=2.8;
	tooth_grow_center[8][2]=4;
	tooth_grow_center[9][0]=-0.5;//8，9中切牙
	tooth_grow_center[9][1]=2.8;
	tooth_grow_center[9][2]=3.1;
	tooth_grow_center[10][0]=0.5;
	tooth_grow_center[10][1]=2.8;
	tooth_grow_center[10][2]=3.2;
	tooth_grow_center[11][0]=3.5;
	tooth_grow_center[11][1]=2.8;
	tooth_grow_center[11][2]=1.2;
	tooth_grow_center[12][0]=4.5;
	tooth_grow_center[12][1]=2.8;
	tooth_grow_center[12][2]=1.2;
	tooth_grow_center[13][0]=4.5;
	tooth_grow_center[13][1]=2.8;
	tooth_grow_center[13][2]=1.2;
	tooth_grow_center[14][0]=5.3;
	tooth_grow_center[14][1]=2.8;
	tooth_grow_center[14][2]=1.2;
	tooth_grow_center[15][0]=5.3;
	tooth_grow_center[15][1]=2.8;
	tooth_grow_center[15][2]=1.2;
	

	for (int p = 2; p < 16; p++)
	{
		//取测试的单个牙齿
		Tooth* tooth = m_OrthoData->GetTooth((ToothId)p);
		auto center = tooth->GetToothActor()->GetCenter();//原actor中心点

		// 定义点和向量
		double point[3] = { center[0]+ tooth_grow_center[p][0], center[1]+ tooth_grow_center[p][1], center[2]+ tooth_grow_center[p][2] }; // 箭头的起点
		double vector[3] = { 1.0, 1.0, 0.0 }; // 箭头的方向向量

		// 创建箭头源
		vtkNew<vtkArrowSource> arrowSource;
		arrowSource->SetTipResolution(20); // 设置箭头尖端的分辨率
		arrowSource->SetShaftResolution(20); // 设置箭杆的分辨率
		arrowSource->SetTipLength(0.2); // 设置箭头尖端的长度（相对于总长度的比例）
		arrowSource->SetTipRadius(0.12); // 设置箭头尖端的半径
		arrowSource->SetShaftRadius(0.05); // 设置箭杆的半径

		// 创建变换对象
		vtkNew<vtkTransform> transform;
		transform->Translate(point[0], point[1], point[2]); // 将箭头移动到指定的点
		//transform->RotateZ(-90.0); // 绕X轴逆时针旋转90度
		//transform->RotateWXYZ(vtkMath::DegreesFromRadians(acos(vector[2])), -vector[1], vector[0], 0.0); // 旋转箭头以对齐向量
		//if (vector[2] < 0) {
		//	transform->RotateWXYZ(180, 0.0, 0.0, 1.0);
		//}

		// 计算从Z轴 (0, 0, 1) 到目标方向向量的旋转
		double zAxis[3] = { 0.0, 0.0, 1.0 };
		double axis[3];
		double angle;
		vtkMath::Cross(zAxis, vector, axis); // 计算旋转轴
		vtkMath::Normalize(axis); // 归一化旋转轴
		angle = acos(vtkMath::Dot(zAxis, vector)); // 计算旋转角度

		// 如果旋转轴是零向量（即方向向量与Z轴平行），则不需要旋转
		if (vtkMath::Norm(axis) > 1e-6)
		{
			transform->RotateWXYZ(vtkMath::DegreesFromRadians(angle), axis[0], axis[1], axis[2]);
		}

		// 旋转箭头以对齐方向向量
		if (vector[2] != 1.0) {
			double angle = acos(vector[2]);
			transform->RotateWXYZ(vtkMath::DegreesFromRadians(angle), -vector[1], vector[0], 0.0);
			if (vector[2] < 0) {
				transform->RotateWXYZ(180, 0.0, 0.0, 1.0);
			}
		}

		// 缩放箭头以放大
		double scale_factor = 7.0; // 放大倍数
		transform->Scale(scale_factor, scale_factor, scale_factor);

		// 应用变换到箭头
		vtkNew<vtkTransformPolyDataFilter> transformFilter;
		transformFilter->SetInputConnection(arrowSource->GetOutputPort());
		transformFilter->SetTransform(transform);
		transformFilter->Update();

		// 创建mapper和actor来显示箭头
		vtkNew<vtkPolyDataMapper> mapper;
		mapper->SetInputConnection(transformFilter->GetOutputPort());

		vtkNew<vtkActor> actor;
		actor->SetMapper(mapper);
		actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色
		m_RenderContainer->GetRenderer()->AddActor(actor);
	}
	m_RenderContainer->Render();
}



void GroupPathPlanningPSO::TestBox() {

	for (int p = 18; p < 32; p++)
	{
		//取测试的单个牙齿
		Tooth* tooth1 = m_OrthoData->GetTooth((ToothId)p);
		vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(tooth1->GetToothPolyData());
		vtkSmartPointer<vtkActor>  actorTest =
			vtkSmartPointer<vtkActor>::New();
		actorTest->SetMapper(mapper);
		//修改透明度和颜色
		//actorTest->GetProperty()->SetOpacity(0.5);
		//actorTest->GetProperty()->SetColor(1, 0, 0);
		m_RenderContainer->GetRenderer(MainRenderer)->AddActor(actorTest);


		//包围盒
		/*vtkSmartPointer<vtkOutlineFilter> outline =
			vtkSmartPointer<vtkOutlineFilter>::New();
		outline->SetInputData(tooth1->GetToothPolyData());*/
		//outline->SetInputData(dynamic_cast<vtkPolyData*>(picker->GetActor()->GetMapper()->GetInput()));
		vtkNew<vtkNamedColors> colors;

		//vtkSmartPointer<vtkPolyDataMapper> outlineMapper =
		//	vtkSmartPointer<vtkPolyDataMapper>::New();
		//outlineMapper->SetInputConnection(outline->GetOutputPort());
		//vtkSmartPointer<vtkActor> outlineActor =
		//	vtkSmartPointer<vtkActor>::New();
		//outlineActor->SetMapper(outlineMapper);
		////outlineActor->GetProperty()->SetColor(1, 0, 0);
		//outlineActor->GetProperty()->SetColor(colors->GetColor4d("black").GetData());
		//outlineActor->GetProperty()->SetOpacity(.5);
		//m_RenderContainer->GetRenderer(MainRenderer)->AddActor(outlineActor);


		int maxLevel = 12;
		// Create the tree
		vtkNew<vtkOBBTree> obbTree;
		obbTree->SetDataSet(tooth1->GetToothPolyData());
		obbTree->SetMaxLevel(maxLevel);
		obbTree->BuildLocator();
		

		double corner[3] = { 0.0, 0.0, 0.0 };
		double max[3] = { 0.0, 0.0, 0.0 };
		double mid[3] = { 0.0, 0.0, 0.0 };
		double min[3] = { 0.0, 0.0, 0.0 };
		double size[3] = { 0.0, 0.0, 0.0 };

		obbTree->ComputeOBB(tooth1->GetToothPolyData(), corner, max, mid, min, size);
		// Initialize the representation
		vtkNew<vtkPolyData> polydata;
		obbTree->GenerateRepresentation(8, polydata);
		vtkNew<vtkPolyDataMapper> obbtreeMapper;
		obbtreeMapper->SetInputData(polydata);

		vtkNew<vtkActor> obbtreeActor;
		obbtreeActor->SetMapper(obbtreeMapper);
		obbtreeActor->GetProperty()->SetInterpolationToFlat();
		obbtreeActor->GetProperty()->SetOpacity(.5);
		obbtreeActor->GetProperty()->EdgeVisibilityOn();
		obbtreeActor->GetProperty()->SetColor(
		colors->GetColor4d("Orange").GetData());
	
		//m_RenderContainer->GetRenderer()->AddActor(obbtreeActor);
		m_RenderContainer->GetRenderer(MainRenderer)->AddActor(obbtreeActor);

		vtkNew<vtkSliderRepresentation2D> sliderRep;
		sliderRep->SetMinimumValue(0);
		sliderRep->SetMaximumValue(obbTree->GetLevel());
		sliderRep->SetValue(obbTree->GetLevel() / 2);
		sliderRep->SetTitleText("Level");
		sliderRep->GetPoint1Coordinate()->SetCoordinateSystemToNormalizedDisplay();
		sliderRep->GetPoint1Coordinate()->SetValue(.2, .2);
		sliderRep->GetPoint2Coordinate()->SetCoordinateSystemToNormalizedDisplay();
		sliderRep->GetPoint2Coordinate()->SetValue(.8, .2);
		sliderRep->SetSliderLength(0.075);
		sliderRep->SetSliderWidth(0.05);
		sliderRep->SetEndCapLength(0.05);
		/*sliderRep->GetTitleProperty()->SetColor(
			colors->GetColor3d("Beige").GetData());
		sliderRep->GetCapProperty()->SetColor(
			colors->GetColor3d("MistyRose").GetData());
		sliderRep->GetSliderProperty()->SetColor(
			colors->GetColor3d("LightBlue").GetData());
		sliderRep->GetSelectedProperty()->SetColor(
			colors->GetColor3d("Violet").GetData());*/

		vtkNew<vtkSliderWidget> sliderWidget;
		sliderWidget->SetInteractor(m_RenderContainer->GetRenWinInteractor());
		sliderWidget->SetRepresentation(sliderRep);
		sliderWidget->SetAnimationModeToAnimate();
		sliderWidget->EnabledOn();
		
		/*vtkNew<vtkSliderCallback> callback;
		callback->OBBTree = obbTree;
		callback->PolyData = polydata;
		callback->Renderer = renderer;
		callback->Execute(sliderWidget, 0, 0);

		sliderWidget->AddObserver(vtkCommand::InteractionEvent, callback);*/
		
		
		
		
	}









	

	 //------------------------------------------------------旋转分量测试-----------------------------------------------------
	m_RenderContainer->Render();//可视化牙齿目标位

}

// 辅助函数：获取立方体的一条边的两个端点
void GroupPathPlanningPSO::GetEdgePoints(double bounds[6], int edgeIndex, double p0[3], double p1[3]) {
	static const int edges[12][2] = {
		{0, 1}, {1, 2}, {2, 3}, {3, 0},
		{4, 5}, {5, 6}, {6, 7}, {7, 4},
		{0, 4}, {1, 5}, {2, 6}, {3, 7}
	};

	for (int i = 0; i < 3; ++i) {
		p0[i] = (edges[edgeIndex][0] & (1 << i)) ? bounds[2 * i + 1] : bounds[2 * i];
		p1[i] = (edges[edgeIndex][1] & (1 << i)) ? bounds[2 * i + 1] : bounds[2 * i];
	}
}

void GroupPathPlanningPSO::OBBTree_Occ() {

	for (int p = 2; p < 16; p++)
	{
		//取测试的单个牙齿
		Tooth* tooth1 = m_OrthoData->GetTooth((ToothId)p);
		if (tooth1) continue;
		//vtkSmartPointer<vtkPolyDataMapper> mapper =
		//	vtkSmartPointer<vtkPolyDataMapper>::New();
		//mapper->SetInputData(tooth1->GetToothPolyData());
		//vtkSmartPointer<vtkActor>  actorTest =
		//	vtkSmartPointer<vtkActor>::New();
		//actorTest->SetMapper(mapper);

		////修改透明度和颜色
		//if (p = 11) {
		//	actorTest->GetProperty()->SetOpacity(0.5);
		//	actorTest->GetProperty()->SetColor(1, 0, 0);
		//}
		//m_RenderContainer->GetRenderer(MainRenderer)->AddActor(actorTest);
	
		vtkNew<vtkNamedColors> colors;

		//int maxLevel = 12;
		// Create the tree
		vtkNew<vtkOBBTree> obbTree;
		obbTree->SetDataSet(tooth1->GetToothPolyData());
		//obbTree->SetMaxLevel(maxLevel);
		obbTree->BuildLocator();
		


		double corner[3] = { 0.0, 0.0, 0.0 };
		double max[3] = { 0.0, 0.0, 0.0 };
		double mid[3] = { 0.0, 0.0, 0.0 };
		double min[3] = { 0.0, 0.0, 0.0 };
		double size[3] = { 0.0, 0.0, 0.0 };

		obbTree->ComputeOBB(tooth1->GetToothPolyData(), corner, max, mid, min, size);
		// Initialize the representation
		vtkNew<vtkPolyData> polydata;
		obbTree->GenerateRepresentation(8, polydata);
		vtkNew<vtkPolyDataMapper> obbtreeMapper;
		obbtreeMapper->SetInputData(polydata);

		vtkNew<vtkActor> obbtreeActor;
		obbtreeActor->SetMapper(obbtreeMapper);
		obbtreeActor->GetProperty()->SetInterpolationToFlat();
		obbtreeActor->GetProperty()->SetOpacity(.5);
		obbtreeActor->GetProperty()->EdgeVisibilityOn();
		obbtreeActor->GetProperty()->SetColor(
			colors->GetColor4d("SpringGreen").GetData());

		//m_RenderContainer->GetRenderer()->AddActor(obbtreeActor);

		////碰撞检测
		//double min[3], max[3];
		//if (obbTree->IntersectWithOBBTree(obbTree2->GetToothPolyData()->GetBounds(), min, max))
		//{
		//	// 如果碰撞，将actor2的网格颜色设置为红色
		//	actor2->GetProperty()->SetColor(1.0, 0.0, 0.0);
		//}

	}

	Tooth* tooth1 = m_OrthoData->GetTooth((ToothId)7);
	Tooth* tooth2 = m_OrthoData->GetTooth((ToothId)8);
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->RotateX(2);
	transform->RotateY(2);
	transform->RotateZ(0);
	//transform->Translate(1,0.2,0);//平移	
	transform->Update();
	tooth1->GetToothActor()->SetUserTransform(transform);
	tooth1->GetToothActor()->SetVisibility(true);

	vtkNew<vtkOBBTree> obbTree1;
	obbTree1->SetDataSet(tooth1->GetToothPolyData());
	obbTree1->BuildLocator();
	vtkNew<vtkOBBTree> obbTree2;
	obbTree2->SetDataSet(tooth2->GetToothPolyData());
	obbTree2->BuildLocator();

	//// 准备用于存储碰撞点的vtkPolyData
	//vtkSmartPointer<vtkPoints> collisionPoints = vtkSmartPointer<vtkPoints>::New();
	//vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

	//// 遍历第一个OBB树的所有单元格
	//for (vtkIdType i = 0; i < tooth1->GetToothPolyData()->GetNumberOfCells(); ++i) {
	//	double bounds[6];
	//	tooth1->GetToothPolyData()->GetCellBounds(i, bounds);

	//	// 检查当前单元格的每条边是否与第二个OBB树相交
	//	for (int edgeIndex = 0; edgeIndex < 12; ++edgeIndex) { // 一个立方体有12条边
	//		double p0[3], p1[3];
	//		GetEdgePoints(bounds, edgeIndex, p0, p1); // 辅助函数，获取立方体的一条边的两个端点

	//		double t, x[3], pcoords[3];
	//		int subId;
	//		vtkIdType cellId;
	//		vtkCell* cell;

	//		if (obbTree2->IntersectWithLine(p0, p1, 0.0, t, x, pcoords, subId, cellId, cell)) {
	//			// 添加相交点到新的polydata
	//			vtkIdType newPointId = collisionPoints->InsertNextPoint(x);
	//			vertices->InsertNextCell(1);
	//			vertices->InsertCellPoint(newPointId);
	//		}
	//	}
	//}

	//// 遍历第一个OBB树的所有单元格
	//for (vtkIdType i = 0; i < tooth1->GetToothPolyData()->GetNumberOfCells(); ++i) {
	//	double bounds[6];
	//	tooth1->GetToothPolyData()->GetCellBounds(i, bounds);

	//	// 检查当前单元格与第二个OBB树是否相交
	//	if (obbTree2->IntersectWithLine(bounds, 0.0)) {
	//		double t, x[3], pcoords[3];
	//		int subId;
	//		vtkSmartPointer<vtkIdList> cellIds = vtkSmartPointer<vtkIdList>::New();

	//		// 获取相交的单元格ID
	//		if (obbTree2->IntersectWithLine(bounds, 0.0, cellIds, t, x, pcoords, subId)) {
	//			for (vtkIdType j = 0; j < cellIds->GetNumberOfIds(); ++j) {
	//				vtkIdType id = cellIds->GetId(j);
	//				// 获取相交单元格的点
	//				vtkCell* cell = tooth2->GetToothPolyData()->GetCell(id);
	//				for (int k = 0; k < cell->GetNumberOfPoints(); ++k) {
	//					double point[3];
	//					cell->GetPoints()->GetPoint(k, point);
	//					// 添加点到新的polydata
	//					vtkIdType newPointId = collisionPoints->InsertNextPoint(point);
	//					// 添加顶点
	//					vertices->InsertNextCell(1);
	//					vertices->InsertCellPoint(newPointId);
	//				}
	//			}
	//		}
	//	}
	//}

	//// 创建一个新的vtkPolyData来存储碰撞点
	//vtkSmartPointer<vtkPolyData> collisionPolyData = vtkSmartPointer<vtkPolyData>::New();
	//collisionPolyData->SetPoints(collisionPoints);
	//collisionPolyData->SetVerts(vertices);

	//// 创建球体源和glyph3D来表示碰撞点
	//vtkNew<vtkSphereSource> sphereSource;
	//sphereSource->SetRadius(0.5); // 调整半径以适应你的场景
	//sphereSource->Update();

	//vtkNew<vtkGlyph3D> glyph3D;
	//glyph3D->SetInputData(collisionPolyData);
	//glyph3D->SetSourceConnection(sphereSource->GetOutputPort());
	//glyph3D->ScalingOff();
	//glyph3D->Update();

	//// 创建mapper和actor来显示碰撞点
	//vtkNew<vtkPolyDataMapper> mapper;
	//mapper->SetInputConnection(glyph3D->GetOutputPort());

	//vtkNew<vtkActor> actor;
	//actor->SetMapper(mapper);
	//actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色

	//// 创建渲染器、渲染窗口和交互器
	//vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	//vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	//renderWindow->AddRenderer(renderer);
	//vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	//interactor->SetRenderWindow(renderWindow);

	//// 添加原始物体
	//vtkNew<vtkPolyDataMapper> tooth1Mapper;
	//tooth1Mapper->SetInputData(tooth1PolyData);

	//vtkNew<vtkActor> tooth1Actor;
	//tooth1Actor->SetMapper(tooth1Mapper);
	//tooth1Actor->GetProperty()->SetColor(0.5, 0.5, 0.5); // 灰色

	//vtkNew<vtkPolyDataMapper> tooth2Mapper;
	//tooth2Mapper->SetInputData(tooth2PolyData);

	//vtkNew<vtkActor> tooth2Actor;
	//tooth2Actor->SetMapper(tooth2Mapper);
	//tooth2Actor->GetProperty()->SetColor(0.5, 0.5, 0.5); // 灰色

	//// 添加物体和碰撞点到渲染器
	//renderer->AddActor(tooth1Actor);
	//renderer->AddActor(tooth2Actor);
	//renderer->AddActor(actor);

	//// 遍历第一个OBB树的所有叶节点
	//for (int i = 0; i < obbTree1->GetNumberOfNodes(); ++i) {
	//	if (obbTree1->IsLeafNode(i)) {
	//		vtkSmartPointer<vtkPolyData> obb1 = CreateOBB(obbTree1, i);

	//		// 创建mapper和actor
	//		vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
	//		mapper1->SetInputData(obb1);

	//		vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor>::New();
	//		actor1->SetMapper(mapper1);
	//		actor1->GetProperty()->SetColor(0.8, 0.8, 0.8); // 默认颜色

	//		// 检查与第二个OBB树的所有叶节点是否相交
	//		for (int j = 0; j < obbTree2->GetNumberOfNodes(); ++j) {
	//			if (obbTree2->IsLeafNode(j)) {
	//				if (obbTree1->IntersectWithLine(obb1->GetBounds(), 0.0, nullptr, nullptr, nullptr, nullptr)) {
	//					// 如果相交，设置为红色
	//					actor1->GetProperty()->SetColor(1.0, 0.0, 0.0);
	//				}
	//			}
	//		}

	//		// 将actor添加到渲染器
	//		renderer->AddActor(actor1);
	//	}
	//}

	//// 同样处理第二个OBB树
	//for (int j = 0; j < obbTree2->GetNumberOfNodes(); ++j) {
	//	if (obbTree2->IsLeafNode(j)) {
	//		vtkSmartPointer<vtkPolyData> obb2 = CreateOBB(obbTree2, j);

	//		// 创建mapper和actor
	//		vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
	//		mapper2->SetInputData(obb2);

	//		vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
	//		actor2->SetMapper(mapper2);
	//		actor2->GetProperty()->SetColor(0.8, 0.8, 0.8); // 默认颜色

	//		// 将actor添加到渲染器
	//		renderer->AddActor(actor2);
	//	}
	//}

	// 创建一个新的vtkPolyData来存储相交部分
	vtkSmartPointer<vtkPolyData> intersectionPolyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkCellArray> cells2 = vtkSmartPointer<vtkCellArray>::New();

	// 准备用于存储交点信息的变量
	vtkSmartPointer<vtkIdList> cellIds = vtkSmartPointer<vtkIdList>::New();
	double t, x[3], pcoords[3];
	int subId;

	// 遍历第一个OBB树的所有单元格
	for (vtkIdType i = 0; i < obbTree1->GetDataSet()->GetNumberOfCells(); ++i) {
		// 获取当前单元格
		vtkCell* cell1 = obbTree1->GetDataSet()->GetCell(i);

		for (int j = 0; j < cell1->GetNumberOfEdges(); ++j) {
			vtkCell* edge = cell1->GetEdge(j);
			double p0[3], p1[3];
			edge->GetPoints()->GetPoint(0, p0);
			edge->GetPoints()->GetPoint(1, p1);

			// 检查这条边是否与第二个OBB树相交
			if (obbTree2->IntersectWithLine(p0, p1, 0.0, t, x, pcoords, subId)) {
				// 将相交点添加到新的polydata
				vtkIdType newPointId = points->InsertNextPoint(x);
				cells->InsertNextCell(1);
				cells->InsertCellPoint(newPointId);
			}
		}
	}
	// 遍历第二个OBB树的所有单元格
	for (vtkIdType i = 0; i < obbTree2->GetDataSet()->GetNumberOfCells(); ++i) {
		// 获取当前单元格
		vtkCell* cell1 = obbTree2->GetDataSet()->GetCell(i);

		for (int j = 0; j < cell1->GetNumberOfEdges(); ++j) {
			vtkCell* edge = cell1->GetEdge(j);
			double p0[3], p1[3];
			edge->GetPoints()->GetPoint(0, p0);
			edge->GetPoints()->GetPoint(1, p1);

			// 检查这条边是否与第二个OBB树相交
			if (obbTree1->IntersectWithLine(p0, p1, 0.0, t, x, pcoords, subId)) {
				// 将相交点添加到新的polydata
				vtkIdType newPointId = points->InsertNextPoint(x);
				cells->InsertNextCell(1);
				cells->InsertCellPoint(newPointId);
			}
		}
	}

	// 设置新的polydata的点和单元格
	intersectionPolyData->SetPoints(points);
	intersectionPolyData->SetVerts(cells);

	// 创建球体源和glyph3D来表示碰撞点
	vtkNew<vtkSphereSource> sphereSource;
	sphereSource->SetRadius(1.2); // 调整半径以适应你的场景
	sphereSource->Update();

	vtkNew<vtkGlyph3D> glyph3D;
	glyph3D->SetInputData(intersectionPolyData);
	glyph3D->SetSourceConnection(sphereSource->GetOutputPort());
	glyph3D->ScalingOff();
	glyph3D->Update();

	// 创建mapper和actor来显示碰撞点
	vtkNew<vtkPolyDataMapper> mapper;
	mapper->SetInputConnection(glyph3D->GetOutputPort());

	vtkNew<vtkActor> actor;
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色

	//// 创建mapper和actor来显示相交部分
	//vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	//mapper->SetInputData(intersectionPolyData);

	//vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	//actor->SetMapper(mapper);
	//actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色

	m_RenderContainer->GetRenderer()->AddActor(actor);//可视化牙齿目标位

	//


	//// 创建一个用于相交测试的OBB
	//double bounds[6]; // 用来存储第二个OBB的边界
	//obbTree2->GetDataSet()->GetBounds(bounds);
	//vtkNew<vtkOBBTree> tempOBB;
	//tempOBB->SetDataSet(obbTree2->GetDataSet());
	//tempOBB->BuildLocator();
	//// 遍历vtkOBBTree叶节，并修改颜色
	//for (int i = 0; i < obbTree1->GetNumberOfLeafNodes(); ++i) {//GetNumberOfCellsPerNode
	//	vtkOBBTreeNode* node1 = obbTree1->GetLeafNode(i);
	//	for (int j = 0; j < obbTree2->GetNumberOfLeafNodes(); ++j) {
	//		vtkOBBTreeNode* node2 = obbTree2->GetLeafNode(j);
	//		if (obbTree1->Intersect(node1, obbTree2, node2)) {
	//			// 如果叶节相交，修改叶节颜色
	//			vtkPolyData* poly1 = static_cast<vtkPolyData*>(node1->GetData());
	//			vtkPolyData* poly2 = static_cast<vtkPolyData*>(node2->GetData());

	//			vtkSmartPointer<vtkProperty> property1 = vtkSmartPointer<vtkProperty>::New();
	//			vtkSmartPointer<vtkProperty> property2 = vtkSmartPointer<vtkProperty>::New();

	//			property1->SetColor(1.0, 0.0, 0.0); // 红色
	//			property2->SetColor(0.0, 1.0, 0.0); // 绿色

	//			// 在这里可以将property应用到对应的actor上
	//		}
	//	}
	//}
	//// 遍历vtkOBBTree叶节，并修改颜色
	//for (int i = 0; i < obbTree1->GetNumberOfLeafNodes(); ++i) {
	//	vtkOBBTreeNode* node1 = obbTree1->GetLeafNode(i);
	//	for (int j = 0; j < obbTree2->GetNumberOfLeafNodes(); ++j) {
	//		vtkOBBTreeNode* node2 = obbTree2->GetLeafNode(j);
	//		if (obbTree1->Intersect(node1, obbTree2, node2)) {
	//			// 如果叶节相交，修改叶节颜色
	//			vtkPolyData* poly1 = static_cast<vtkPolyData*>(node1->GetData());
	//			vtkPolyData* poly2 = static_cast<vtkPolyData*>(node2->GetData());

	//			vtkSmartPointer<vtkProperty> property1 = vtkSmartPointer<vtkProperty>::New();
	//			vtkSmartPointer<vtkProperty> property2 = vtkSmartPointer<vtkProperty>::New();

	//			property1->SetColor(1.0, 0.0, 0.0); // 红色
	//			property2->SetColor(0.0, 1.0, 0.0); // 绿色

	//			// 在这里可以将property应用到对应的actor上
	//		}
	//	}
	//}

	m_RenderContainer->Render();//可视化牙齿目标位

}




void  GroupPathPlanningPSO::ReadMatrixFile() { //获取理想位置Matrix矩阵
	//std::map<ToothId, Eigen::Matrix4f> Matrix; //用于存放牙齿对应的矩阵
	//下面for的终止条件中的6表示txt文件中矩阵的个数
	char* filePath = "D:\\1111牙齿及目标位\\data3\\TargetPosition3.txt";//------------------------------更改---------------------------
	ifstream fileStream;
	fileStream.open(filePath);
	if (fileStream.fail())//文件打开失败:返回0
	{
		cout << "fail to open file" << endl;
	}
	//data3
	else//文件存在
	{
		for (int p = 2; p < 16; p++)
		{
			Eigen::Matrix4f M;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					fileStream >> M(i, j);
				}
			}
			Matrix[p] = M;
		}
		for (int p = 18; p < 32; p++)
		{
			Eigen::Matrix4f M;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					fileStream >> M(i, j);
				}
			}
			Matrix[p] = M;
		}
	}
	//data5
	//else//文件存在
	//{
	//	for (int p = 2; p < 16; p++)
	//	{
	//		Eigen::Matrix4f M;
	//		for (int i = 0; i < 4; i++) {
	//			for (int j = 0; j < 4; j++) {
	//				fileStream >> M(i, j);
	//			}
	//		}
	//		Matrix[p] = M;
	//	}
	//	for (int p = 17; p < 33; p++)
	//	{
	//		Eigen::Matrix4f M;
	//		for (int i = 0; i < 4; i++) {
	//			for (int j = 0; j < 4; j++) {
	//				fileStream >> M(i, j);
	//			}
	//		}
	//		Matrix[p] = M;
	//	}
	//}
	////data7下颌
	//char* filePath = "E:\\1111牙齿及目标位\\data7\\TargetPosition.txt";//------------------------------更改---------------------------
	//ifstream fileStream;
	//fileStream.open(filePath);
	//if (fileStream.fail())//文件打开失败:返回0
	//{
	//	cout << "fail to open file" << endl;
	//}
	//else//文件存在
	//{
	//	/*for (int p = 2; p < 16; p++)
	//	{
	//		Eigen::Matrix4f M;
	//		for (int i = 0; i < 4; i++) {
	//			for (int j = 0; j < 4; j++) {
	//				fileStream >> M(i, j);
	//			}
	//		}
	//		Matrix[p] = M;
	//	}*/
	//	for (int p = 18; p < 32; p++)
	//	{
	//		Eigen::Matrix4f M;
	//		for (int i = 0; i < 4; i++) {
	//			for (int j = 0; j < 4; j++) {
	//				fileStream >> M(i, j);
	//			}
	//		}
	//		Matrix[p] = M;
	//	}
	//}
	fileStream.close();//关闭文件
}

void GroupPathPlanningPSO::GetpositionMatrix() {
	for (int p = 2; p < 16; p++)
	{
		for (int i = 0; i < 3; i++) {
			after[p][i] = Matrix[p](i,3);
		}
	}
	for (int p = 18; p < 32; p++)
	{
		for (int i = 0; i < 3; i++) {
			after[p][i] = Matrix[p](i, 3);
		}
	}
}

void GroupPathPlanningPSO::ToothData() {//读取牙齿数据并保存
	std::vector<vtkSmartPointer<vtkActor>> init_tooth;
	std::vector<vtkSmartPointer<vtkActor>> target_tooth;
	//for (int i = 2; i < 16; i++)
	for (int i = 18; i < 32; i++)
	{
		Tooth* tooth = m_OrthoData->GetTooth((ToothId)i);
		if (tooth == NULL) continue;
		tooth->GetToothActor()->SetVisibility(true);
		//tooth->GetToothActor()->GetProperty()->SetColor(0.2,0.5,0.73);
		init_tooth.push_back(tooth->GetToothActor());
		//保存畸形牙齿数据—初始牙齿编码
		auto center1 = tooth->GetToothActor()->GetCenter();//原actor中心点
		before[i][0] = center1[0];//当前位置（第i个牙齿,每个牙齿xyz3坐标）
		before[i][1] = center1[1];
		before[i][2] = center1[2];
		before[i][3] = 0;
		before[i][4] = 0;
		before[i][5] = 0;
		cout << "牙齿"<<i << "矫正前中心位置:" << before[i][0] << "，" << before[i][1] << "，" << before[i][2] << endl;
		//保存初始牙齿中心点到OrthoData中
		std::vector<double> bef;
		for (int d = 0; d < 6; d++) {//d维
			bef.push_back(before[i][d]);
		}
		m_OrthoData->SetCenterBefore((ToothId)i,bef);

		//添加初始牙齿到renderer，便于窗口可视化对比
		//vtkSmartPointer<vtkPolyDataMapper> mapper =
		//	vtkSmartPointer<vtkPolyDataMapper>::New();
		//mapper->SetInputData(tooth->GetToothPolyData());
		//vtkSmartPointer<vtkActor>  actor_before =
		//	vtkSmartPointer<vtkActor>::New();
		//actor_before->SetMapper(mapper);
		////修改透明度和颜色
		///*actor_before->GetProperty()->SetOpacity(0.5);
		//actor_before->GetProperty()->SetColor(1, 0, 0);*/
		//m_RenderContainer->GetRenderer(MainRenderer)->AddActor(actor_before);
		

		////保存各个牙齿坐标轴
		//int temp1 = i + 1;
		//if (temp1 = 15)	temp1 = 14;
		//if (temp1 = 31) temp1 = 30;
		//Tooth* tooth2 = m_OrthoData->GetTooth((ToothId)temp1);//取第i+1个牙齿
		//if (tooth2 == NULL) continue;
		////三个点center1，center2，center3
		//auto center2 = tooth2->GetToothActor()->GetCenter();//下一个牙齿中心点
		//double center3[3] = { center1[0], center1[1], center1[2] - 10 };//建立中心点下方的一个点（上颌）——计算Y轴单位向量需要使用的中间临时Z轴点
		////------------------------X轴----------------------------
		////计算X轴上两个点的方向向量(终点 - 起点)—Direction[3]
		//double DirectionX[3];
		//DirectionX[0] = center2[0] - center1[0];
		//DirectionX[1] = center2[1] - center1[1];
		//DirectionX[2] = center2[2] - center1[2];
		////计算X轴向量模长—Xd
		//double Xx = pow(DirectionX[0], 2);//pow()幂次方函数
		//double Xy = pow(DirectionX[1], 2);
		//double Xz = pow(DirectionX[2], 2);
		//double Xd = sqrt(Xx + Xy + Xz);//sqrt()平方根函数
		////计算X轴单位向量——normalizeX[3]
		//double normalizeX[3] = { sqrt(Xx) / Xd, sqrt(Xy) / Xd, sqrt(Xz) / Xd };
		////保存第i个牙齿的X轴
		//std::vector<double> norX = { normalizeX[0] ,normalizeX[1] ,normalizeX[2] };
		//m_OrthoData->SetMap((ToothId)i, 1, norX);
		////------------------Y轴-----------------------------------
		////计算临时Z轴上两个点的方向向量(终点 - 起点)——DirectionInterimZ[3]
		//double DirectionInterimZ[3];
		//DirectionInterimZ[0] = center3[0] - center1[0];
		//DirectionInterimZ[1] = center3[1] - center1[1];
		//DirectionInterimZ[2] = center3[2] - center1[2];
		////计算临时Z轴向量模长——ZInterimd
		//double ZInterimx = pow(DirectionInterimZ[0], 2);//pow()幂次方函数
		//double ZInterimy = pow(DirectionInterimZ[1], 2);
		//double ZInterimz = pow(DirectionInterimZ[2], 2);
		//double ZInterimd = sqrt(ZInterimx + ZInterimy + ZInterimz);//sqrt()平方根函数
		////计算临时Z轴单位向量（为了和normalizeX叉乘得到Y轴单位向量）——normalizeInterimZ[3]
		//double normalizeInterimZ[3] = { sqrt(ZInterimx) / ZInterimd, sqrt(ZInterimy) / ZInterimd, sqrt(ZInterimz) / ZInterimd };

		////叉乘计算Y轴单位向量——normalizeY[3]
		//double normalizeY[3] = { normalizeX[1] * normalizeInterimZ[2] - normalizeX[2] * normalizeInterimZ[1],
		//						normalizeX[2] * normalizeInterimZ[0] - normalizeX[0] * normalizeInterimZ[2],
		//						normalizeX[0] * normalizeInterimZ[1] - normalizeX[1] * normalizeInterimZ[0]
		//};
		////保存第i个牙齿的Y轴
		//std::vector<double> norY = { normalizeY[0] ,normalizeY[1] ,normalizeY[2] };
		//m_OrthoData->SetMap((ToothId)i,2, norY);
		////---------------------Z轴----------------------------------
		////叉乘计算Z轴单位向量normalizeZ[3]
		//double normalizeZ[3] = { normalizeX[1] * normalizeY[2] - normalizeX[2] * normalizeY[1],
		//						normalizeX[2] * normalizeY[0] - normalizeX[0] * normalizeY[2],
		//						normalizeX[0] * normalizeY[1] - normalizeX[1] * normalizeY[0]
		//};
		////保存第i个牙齿的Z轴
		//std::vector<double> norZ = { normalizeZ[0] ,normalizeZ[1] ,normalizeZ[2] };
		//m_OrthoData->SetMap((ToothId)i, 3, norZ);

		//使用vtkMatrix4x4矩阵使其变换到理想位置
		vtkSmartPointer<vtkMatrix4x4>mat = vtkSmartPointer<vtkMatrix4x4>::New();
		for (int j = 0; j < 4; j++) {
			for (int k = 0; k < 4; k++) {
				mat->SetElement(j, k, Matrix[i](j, k));
			}
		}
		tooth->GetToothActor()->SetUserMatrix(mat);
		target_tooth.push_back(tooth->GetToothActor());
		//保存理想牙齿数据—目标牙齿编码
		auto center = tooth->GetToothActor()->GetCenter();//现actor中心点
		after[i][0] = gbestx[i][n][0] = center[0];
		after[i][1] = gbestx[i][n][1] = center[1];
		after[i][2] = gbestx[i][n][2] = center[2];
		after[i][3] = gbestx[i][n][3] = 0;//暂定旋转编码为0
		after[i][4] = gbestx[i][n][4] = 0;
		after[i][5] = gbestx[i][n][5] = 0;
		cout << "牙齿" << i << "矫正后中心位置:" << after[i][0] << "，" << after[i][1] << "，" << after[i][2] << endl;

		//保存初始牙齿中心点到OrthoData中
		std::vector<double> aft;
		for (int d = 0; d < 6; d++) {//d维
			bef.push_back(after[i][d]);
		}
		m_OrthoData->SetCenterAfter((ToothId)i, aft);

		tooth->GetToothActor()->SetVisibility(true);
		////添加理想位置牙齿牙齿到renderer，便于窗口可视化对比
		//	vtkSmartPointer<vtkPolyDataMapper> mapper =
		//	vtkSmartPointer<vtkPolyDataMapper>::New();
		//mapper->SetInputData(tooth->GetToothPolyData());
		//vtkSmartPointer<vtkActor>  actor_before =
		//	vtkSmartPointer<vtkActor>::New();
		//actor_before->SetMapper(mapper);
		////修改透明度和颜色
		//actor_before->GetProperty()->SetOpacity(0.5);
		//actor_before->GetProperty()->SetColor(1, 0, 0);
		//m_RenderContainer->GetRenderer(MainRenderer)->AddActor(actor_before);
		//m_RenderContainer->Render();//可视化牙齿目标位
		//tooth->GetToothActor()->GetProperty()->SetOpacity(1);
		//if (i == 25) {

		//	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		//	tooth->GetToothActor()->SetUserTransform(transform);
		//	transform->Update();
		//	auto centerTest = tooth->GetToothActor()->GetCenter();
		//	vtkTransform* transform1 = (vtkTransform*)tooth->GetToothActor()->GetUserTransform();
		//	transform1->InternalTransformPoint(centerTest, centerTest);
		//	//actorTest->SetOrientation(axis_y[0], axis_y[1], axis_y[2]);//设置方位
		//	tooth->GetToothActor()->SetOrigin(before[i][0], before[i][1], before[i][2]);//设置旋转中心
		//	vtkSmartPointer<vtkTransform> transform2 = vtkSmartPointer<vtkTransform>::New();
		//	transform2->Translate(centerTest[0], centerTest[1], centerTest[2]);
		//	transform2->RotateWXYZ(-0.2, 1, 0, 0);
		//	transform2->RotateWXYZ(0.2, 0, 1, 0);
		//	transform2->RotateWXYZ(0, 0, 0, 1);
		//	//transform2->RotateWXYZ(0, 1, 0, 0);
		//	//transform2->RotateWXYZ(0 ,0, 1, 0);
		//	//transform2->RotateWXYZ(0, 0, 0, 1);
		//	transform2->Translate(-centerTest[0], -centerTest[1], -centerTest[2]);

		//	transform2->Translate(after[i][0] - before[i][0], after[i][1] - before[i][1], after[i][2] - before[i][2]);//平移	
		//	transform2->Update();
		//	transform1->Concatenate(transform2);
		//	tooth->GetToothActor()->SetUserTransform(transform1);
		//
		//}
		
	}

	
	//保存初始牙齿actor到OrthoData中，方便可视化调用
	m_OrthoData->SetPathToothActor(1, init_tooth);
	init_tooth.clear();
	//保存理想牙齿actor到OrthoData中，方便可视化调用
	m_OrthoData->SetPathToothActor(2, target_tooth);
	target_tooth.clear();

	
	m_RenderContainer->Render();//可视化牙齿目标位
}

void  GroupPathPlanningPSO::RotateData_data3() {
	//RotateVYZValue(2, 1, -5.13345e-08, -4.36767e-09, 5.13345e-08, 1, 1.30332e-07, 4.36766e-09, -1.30332e-07, 1);
	//RotateVYZValue(3, 1, 0, 0, 0, 1, 0, 0, 0, 1);
	//RotateVYZValue(4, 1, 0, 0, 0, 1, 0, 0, 0, 1);
	//RotateVYZValue(5, 0.992891, 0.0085858, -0.118714, -0.0117075, 0.999603, -0.0256236, 0.118447, -0.0779188, 0.960658);
	//RotateVYZValue(6, 0.963265, 0.0120724, 0.268282, -0.0325017, 0.996887, 0.0718382, -0.26658, -0.0779188, 0.960658);//计算旋转量
	//RotateVYZValue(7, 0.933091, -0.188831, -0.306078, 0.229071, 0.968152, 0.10104, 0.277251, -0.164393, 0.946629);
	//RotateVYZValue(8, 0.997112, 0.05158, -0.055734, -0.043884, 0.990354, 0.131431, 0.0619756, -0.128605, 0.989757);
	//RotateVYZValue(9, 0.985874, 0.113178, 0.123462, -0.0825586, 0.969749, -0.229719, -0.145726, 0.216281, 0.965394);
	//RotateVYZValue(10, 0.990033, 0.00519984, 0.140743, -0.00134157, 0.999621, -0.0274947, -0.140832, 0.0270318, 0.989664);
	//RotateVYZValue(11, 0.876599, -0.251016, -0.410568, 0.225621, 0.967975, -0.110086, 0.425053, 0.00386843, 0.90516);
	//RotateVYZValue(12, 0.998325, -0.0319623, 0.0482271, 0.0353763, 0.9968, -0.0716827, -0.0457816, 0.0732687, 0.996261);
	//RotateVYZValue(13, 0.993058, 0.109851, 0.0420527, -0.105219, 0.989419, -0.0998955, -0.0525814, 0.0947773, 0.994109);
	//RotateVYZValue(14, 0.991702, -0.0135525, 0.12784, 0.0176715, 0.999359, -0.031141, -0.127336, 0.0331417, 0.991306);
	//RotateVYZValue(15, 0.994874, -0.0191346, 0.0992996, 0.0165106, 0.999494, 0.02718, -0.0997695, -0.0254012, 0.994686);
	//RotateVYZValue(18, 1, 0, 0, 0, 1, 0, 0, 0, 1);
	//RotateVYZValue(19, 1, 0, 0, 0, 1, 0, 0, 0, 1);
	//RotateVYZValue(20, 0.994551, -0.0162122, 0.102985, 0.0169954, 0.999833, -0.00673266, -0.102859, 0.00844625, 0.99466);
	//RotateVYZValue(21, 1, 0, 0, 0, 1, 0, 0, 0, 1);
	//RotateVYZValue(22, 0.998771, 0.0492042, -0.00595636, -0.0493536, 0.998381, -0.0282705, 0.00455568, 0.0285298, 0.999583);
	//RotateVYZValue(23, 0.739427, -0.421033, 0.525337, 0.513318, 0.857473, -0.035285, -0.435606, 0.295756, 0.850162);
	//RotateVYZValue(24, 0.957717, -0.190595, 0.215527, 0.234941, 0.950478, -0.203458, -0.166076, 0.245492, 0.955067);
	//RotateVYZValue(25, 0.964628, -0.038785, -0.260745, -0.0229082, 0.973042, -0.229486, 0.262616, 0.227342, 0.937736);
	//RotateVYZValue(26, 0.653803, 0.197359, -0.730473, -0.336416, 0.94054, -0.0469912, 0.677765, 0.276466, 0.681323);
	//RotateVYZValue(27, 0.988135, 0.0120251, -0.153117, -0.0218293, 0.997805, -0.0625115, 0.15203, 0.0651123, 0.986229);
	//RotateVYZValue(28, 0.97747, -0.18397, -0.103472, 0.175128, 0.980519, -0.0889507, 0.11782, 0.0688258, 0.990647);
	//RotateVYZValue(29, 1, 0, 0, 0, 1, 0, 0, 0, 1);
	//RotateVYZValue(30, 1, 0, 0, 0, 1, 0, 0, 0, 1);
	//RotateVYZValue(31, 1, 0, 0, 0, 1, 0, 0, 0, 1);
	//重置理想牙齿旋转分量编码
	/*after[2][3] = gbestx[2][7][3] = -1.30332e-07;
	after[2][4] = gbestx[2][7][4] = -4.36766e-09;
	after[2][5] = gbestx[2][7][5] = 5.13345e-08;
	after[3][3] = gbestx[3][7][3] = 0;
	after[3][4] = gbestx[3][7][4] = -0;
	after[3][5] = gbestx[3][7][5] = 0;
	after[4][3] = gbestx[4][7][3] = 0;
	after[4][4] = gbestx[4][7][4] = -0;
	after[4][5] = gbestx[4][7][5] = 0;
	after[5][3] = gbestx[5][7][3] = -0.0809327;
	after[5][4] = gbestx[5][7][4] = -0.118726;
	after[5][5] = gbestx[5][7][5] = -0.0117908;
	after[6][3] = gbestx[6][7][3] = -0.0809327;
	after[6][4] = gbestx[6][7][4] = 0.269843;
	after[6][5] = gbestx[6][7][5] = -0.0337284;
	after[7][3] = gbestx[7][7][3] = -0.171947;
	after[7][4] = gbestx[7][7][4] = -0.280932;
	after[7][5] = gbestx[7][7][5] = 0.240736;
	after[8][3] = gbestx[8][7][3] = -0.129212;
	after[8][4] = gbestx[8][7][4] = -0.0620154;
	after[8][5] = gbestx[8][7][5] = -0.0439827;
	after[9][3] = gbestx[9][7][3] = 0.220395;
	after[9][4] = gbestx[9][7][4] = 0.146247;
	after[9][5] = gbestx[9][7][5] = -0.0835466;
	after[10][3] = gbestx[10][7][3] = 0.0273073;
	after[10][4] = gbestx[10][7][4] = 0.141302;
	after[10][5] = gbestx[10][7][5] = -0.00135508;
	after[11][3] = gbestx[11][7][3] = 0.00427373;
	after[11][4] = gbestx[11][7][4] = -0.43902;
	after[11][5] = gbestx[11][7][5] = 0.251914;
	after[12][3] = gbestx[12][7][3] = 0.0734115;
	after[12][4] = gbestx[12][7][4] = 0.0457976;
	after[12][5] = gbestx[12][7][5] = 0.0354208;
	after[13][3] = gbestx[13][7][3] = 0.0950516;
	after[13][4] = gbestx[13][7][4] = 0.0526057;
	after[13][5] = gbestx[13][7][5] = -0.105561;
	after[14][3] = gbestx[14][7][3] = 0.0334199;
	after[14][4] = gbestx[14][7][4] = 0.127683;
	after[14][5] = gbestx[14][7][5] = 0.0178175;
	after[15][3] = gbestx[15][7][3] = -0.0255314;
	after[15][4] = gbestx[15][7][4] = 0.0999357;
	after[15][5] = gbestx[15][7][5] = 0.0165941;
	after[18][3] = gbestx[18][7][3] = 0;
	after[18][4] = gbestx[18][7][4] = -0;
	after[18][5] = gbestx[18][7][5] = 0;
	after[19][3] = gbestx[19][7][3] = 0;
	after[19][4] = gbestx[19][7][4] = -0;
	after[19][5] = gbestx[19][7][5] = 0;
	after[20][3] = gbestx[20][7][3] = 0.00849139;
	after[20][4] = gbestx[20][7][4] = 0.103041;
	after[20][5] = gbestx[20][7][5] = 0.0170869;
	after[21][3] = gbestx[21][7][3] = 0;
	after[21][4] = gbestx[21][7][4] = -0;
	after[21][5] = gbestx[21][7][5] = 0;
	after[22][3] = gbestx[22][7][3] = 0.028534;
	after[22][4] = gbestx[22][7][4] = -0.0045557;
	after[22][5] = gbestx[22][7][5] = -0.0493742;
	after[23][3] = gbestx[23][7][3] = 0.334787;
	after[23][4] = gbestx[23][7][4] = 0.450711;
	after[23][5] = gbestx[23][7][5] = 0.60683;
	after[24][3] = gbestx[24][7][3] = 0.251595;
	after[24][4] = gbestx[24][7][4] = 0.166849;
	after[24][5] = gbestx[24][7][5] = 0.240563;
	after[25][3] = gbestx[25][7][3] = 0.237848;
	after[25][4] = gbestx[25][7][4] = -0.265732;
	after[25][5] = gbestx[25][7][5] = -0.0237438;
	after[26][3] = gbestx[26][7][3] = 0.385478;
	after[26][4] = gbestx[26][7][4] = -0.744719;
	after[26][5] = gbestx[26][7][5] = -0.475222;
	after[27][3] = gbestx[27][7][3] = 0.0659258;
	after[27][4] = gbestx[27][7][4] = -0.152622;
	after[27][5] = gbestx[27][7][5] = -0.0220878;
	after[28][3] = gbestx[28][7][3] = 0.0693641;
	after[28][4] = gbestx[28][7][4] = -0.118094;
	after[28][5] = gbestx[28][7][5] = 0.177284;
	after[29][3] = gbestx[29][7][3] = 0;
	after[29][4] = gbestx[29][7][4] = -0;
	after[29][5] = gbestx[29][7][5] = 0;
	after[30][3] = gbestx[30][7][3] = 0;
	after[30][4] = gbestx[30][7][4] = -0;
	after[30][5] = gbestx[30][7][5] = 0;
	after[31][3] = gbestx[31][7][3] = 0;
	after[31][4] = gbestx[31][7][4] = -0;
	after[31][5] = gbestx[31][7][5] = 0;*/
	//重置理想牙齿旋转分量编码
	//6-14,22-28
	after[2][3] = gbestx[2][n][3] = 0;
	after[2][4] = gbestx[2][n][4] = -0;
	after[2][5] = gbestx[2][n][5] = 0;
	after[3][3] = gbestx[3][n][3] = 0;
	after[3][4] = gbestx[3][n][4] = -0;
	after[3][5] = gbestx[3][n][5] = 0;
	after[4][3] = gbestx[4][n][3] = 0;
	after[4][4] = gbestx[4][n][4] = -0;
	after[4][5] = gbestx[4][n][5] = 0;
	after[5][3] = gbestx[5][n][3] = 0;
	after[5][4] = gbestx[5][n][4] = -0;
	after[5][5] = gbestx[5][n][5] = 0;
	after[6][3] = gbestx[6][n][3] = -1;
	after[6][4] = gbestx[6][n][4] = -5;
	after[6][5] = gbestx[6][n][5] = 4;
	after[7][3] = gbestx[7][n][3] = -1;
	after[7][4] = gbestx[7][n][4] = -1;
	after[7][5] = gbestx[7][n][5] = 1;
	after[8][3] = gbestx[8][n][3] = 16;
	after[8][4] = gbestx[8][n][4] = 0;
	after[8][5] = gbestx[8][n][5] = 2;
	after[9][3] = gbestx[9][n][3] = 23;
	after[9][4] = gbestx[9][n][4] = 25;
	after[9][5] = gbestx[9][n][5] = -5;
	after[10][3] = gbestx[10][n][3] = 2;
	after[10][4] = gbestx[10][n][4] = -1;
	after[10][5] = gbestx[10][n][5] = -5;
	after[11][3] = gbestx[11][n][3] = -2;
	after[11][4] = gbestx[11][n][4] = 3;
	after[11][5] = gbestx[11][n][5] = 5;
	after[12][3] = gbestx[12][n][3] = -0.5;
	after[12][4] = gbestx[12][n][4] = -0.5;
	after[12][5] = gbestx[12][n][5] = -0.5;
	after[13][3] = gbestx[13][n][3] = 2;
	after[13][4] = gbestx[13][n][4] = 2;
	after[13][5] = gbestx[13][n][5] = -6;
	after[14][3] = gbestx[14][n][3] = 0;
	after[14][4] = gbestx[14][n][4] = 3;
	after[14][5] = gbestx[14][n][5] = 1;
	after[15][3] = gbestx[15][n][3] = 0;
	after[15][4] = gbestx[15][n][4] = -0;
	after[15][5] = gbestx[15][n][5] = 0;
	after[18][3] = gbestx[18][n][3] = 0;
	after[18][4] = gbestx[18][n][4] = -0;
	after[18][5] = gbestx[18][n][5] = 0;
	after[19][3] = gbestx[19][n][3] = 0;
	after[19][4] = gbestx[19][n][4] = -0;     
	after[19][5] = gbestx[19][n][5] = 0;
	after[20][3] = gbestx[20][n][3] = 0;
	after[20][4] = gbestx[20][n][4] = -0;
	after[20][5] = gbestx[20][n][5] = 0;
	after[21][3] = gbestx[21][n][3] = 0;
	after[21][4] = gbestx[21][n][4] = -0;
	after[21][5] = gbestx[21][n][5] = 0;
	after[22][3] = gbestx[22][n][3] = 2;
	after[22][4] = gbestx[22][n][4] = 0;
	after[22][5] = gbestx[22][n][5] = -1;
	after[23][3] = gbestx[23][n][3] = -8;
	after[23][4] = gbestx[23][n][4] = 32;
	after[23][5] = gbestx[23][n][5] = 18;
	after[24][3] = gbestx[24][n][3] = 6;
	after[24][4] = gbestx[24][n][4] = 4;
	after[24][5] = gbestx[24][n][5] = 10;
	after[25][3] = gbestx[25][n][3] = 8;
	after[25][4] = gbestx[25][n][4] = -6;
	after[25][5] = gbestx[25][n][5] = 0;
	after[26][3] = gbestx[26][n][3] = 0;
	after[26][4] = gbestx[26][n][4] = -20;
	after[26][5] = gbestx[26][n][5] = -10;
	/*after[23][3] = gbestx[23][n][3] = -4;
	after[23][4] = gbestx[23][n][4] = 16;
	after[23][5] = gbestx[23][n][5] = 20;
	after[24][3] = gbestx[24][n][3] = 10;
	after[24][4] = gbestx[24][n][4] = 5;
	after[24][5] = gbestx[24][n][5] = 0;
	after[25][3] = gbestx[25][n][3] = 5;
	after[25][4] = gbestx[25][n][4] = -7;
	after[25][5] = gbestx[25][n][5] = 3.5;
	after[26][3] = gbestx[26][n][3] = 0;
	after[26][4] = gbestx[26][n][4] = -20;
	after[26][5] = gbestx[26][n][5] = -12;*/
	after[27][3] = gbestx[27][n][3] = 1;
	after[27][4] = gbestx[27][n][4] = 0;
	after[27][5] = gbestx[27][n][5] = 1;
	after[28][3] = gbestx[28][n][3] = 0.5;
	after[28][4] = gbestx[28][n][4] = 0;
	after[28][5] = gbestx[28][n][5] = 7;
	after[29][3] = gbestx[29][n][3] = 0;
	after[29][4] = gbestx[29][n][4] = -0;
	after[29][5] = gbestx[29][n][5] = 0;
	after[30][3] = gbestx[30][n][3] = 0;
	after[30][4] = gbestx[30][n][4] = -0;
	after[30][5] = gbestx[30][n][5] = 0;
	after[31][3] = gbestx[31][n][3] = 0;
	after[31][4] = gbestx[31][n][4] = -0;
	after[31][5] = gbestx[31][n][5] = 0;
	
}
void  GroupPathPlanningPSO::RotateData_data4() {
	//RotateVYZValue(2, 1, 0, 0, 0, 1, 0, 0, 0, 1);
	//RotateVYZValue(3, 0.993886, -0.0462925, 0.10024, 0.0603053, 0.988083, -0.141617, -0.0924895, 0.146796, 0.984833);
	//RotateVYZValue(4, 0.999683, 0.0245458, -0.00557907, -0.0248578, 0.997559, -0.065247, 0.00396392, 0.065365, 0.997854);
	//RotateVYZValue(5, 0.999066, 0.0185925, -0.0390081, -0.0198657, 0.999274, -0.0325095, 0.0383753, 0.0332541, 0.99871);
	//RotateVYZValue(6, 0.977548, -0.0831389, -0.193615, 0.0834843, 0.996489, -0.00638882, 0.193467, -0.00991846, 0.981057);//计算旋转量
	//RotateVYZValue(7, 0.952537, 0.222785, 0.207459, -0.19669, 0.970541, -0.139151, -0.232348, 0.0917414, 0.968296);
	//RotateVYZValue(8, 0.797883, -0.273537, -0.537179, 0.20972, 0.961412, -0.178059, 0.565156, 0.0294127, 0.82446);
	//RotateVYZValue(9, 0.978988, -0.157769, -0.129197, 0.122779, 0.961903, -0.244268, 0.162813, 0.223273, 0.961062);
	//RotateVYZValue(10, 0.998828, 0.0476877, 0.00824606, -0.0470721, 0.996883, -0.0633169, -0.0112398, 0.0628545, 0.997959);
	//RotateVYZValue(11, 0.984962, -0.0541525, -0.164064, 0.0481484, 0.998025, -0.0403572, 0.165926, 0.0318509, 0.985624);
	//RotateVYZValue(12, 0.999849, 0.00914037, 0.0148029, -0.00911988, 0.999957, -0.00145084, -0.0148156, 0.00131562, 0.999889);
	//RotateVYZValue(13, 0.99844, -0.0236198, -0.0506022, 0.0201992, 0.997544, -0.0670739, 0.0520622, 0.0659471, 0.996464);
	//RotateVYZValue(14, 0.996062, -0.0700101, -0.054397, 0.0695107, 0.99752, -0.0110212, 0.0550337, 0.00719659, 0.998459);
	//RotateVYZValue(15, 0.980346, 0.175494, 0.0901251, -0.174355, 0.984471, -0.02042, -0.0923091, 0.00430486, 0.995721);
	//RotateVYZValue(18, 0.996443, -0.0335984, 0.0772834, 0.0313968, 0.999071, 0.029529, -0.0782037, -0.0269975, 0.996572);
	//RotateVYZValue(19, 0.987989, 0.051281, 0.145767, -0.0683179, 0.991085, 0.114385, -0.138602, -0.12297, 0.982684);
	//RotateVYZValue(20, 0.996196, -0.024709, 0.0835693, 0.0245977, 0.999695, 0.00236107, -0.0836021, -0.000296475, 0.996499);
	//RotateVYZValue(21, 0.996072, 0.0316946, -0.0826805, -0.0323271, 0.999457, -0.00632326, 0.0824352, 0.00897125, 0.996556);
	//RotateVYZValue(22, 0.85534, 0.222088, -0.46805, -0.247505, 0.968858, 0.00741562, 0.455121, 0.109502, 0.883671);
	//RotateVYZValue(23, 0.99545, -0.0776, 0.0552888, 0.0928083, 0.921032, -0.378267, -0.0215692, 0.381677, 0.924044);
	//RotateVYZValue(24, .984057, 0.0235977, -0.176282, -0.0434262, 0.993039, -0.109486, 0.172471, 0.115396, 0.978232);
	//RotateVYZValue(25, 0.998979, -0.00671911, 0.0446687, 0.0182691, 0.964488, -0.263494, -0.041312, 0.264041, 0.963626);
	//RotateVYZValue(26, 0.96783, 0.143963, -0.206346, -0.190358, 0.955258, -0.226377, 0.164523, 0.258375, 0.951932);
	//RotateVYZValue(27, 0.995991, -0.0743484, 0.0497392, 0.0824618, 0.978618, -0.188434, -0.0346659, 0.19178, 0.980825);
	//RotateVYZValue(28, 0.999848, -0.00478466, 0.0167746, 0.00479393, 0.999988, -0.000512379, -0.016772, 0.000592718, 0.999859);
	//RotateVYZValue(29, 0.999848, -0.00310753, 0.0171707, 0.00311289, 0.999995, -0.000285609, -0.0171697, 0.000339015, 0.999853);
	//RotateVYZValue(30, 0.990734, -0.0693628, -0.11677, 0.0697666, 0.997563, -0.000630427, 0.116529, -0.00752204, 0.993159);
	//RotateVYZValue(31, 0.999961, -0.00823611, -0.0030747, 0.00842914, 0.997568, 0.0691893, 0.00249738, -0.0692126, 0.997599);

	after[2][3] = gbestx[2][n][3] = 0;
	after[2][4] = gbestx[2][n][4] = -0;
	after[2][5] = gbestx[2][n][5] = -0;
	after[3][3] = gbestx[3][n][3] = 0;
	after[3][4] = gbestx[3][n][4] = 0;
	after[3][5] = gbestx[3][n][5] = 0;
	after[4][3] = gbestx[4][n][3] = 0;
	after[4][4] = gbestx[4][n][4] = 0;
	after[4][5] = gbestx[4][n][5] = 0;
	after[5][3] = gbestx[5][n][3] = 0;
	after[5][4] = gbestx[5][n][4] = 0;
	after[5][5] = gbestx[5][n][5] = 0;
	after[6][3] = gbestx[6][n][3] = 0;
	after[6][4] = gbestx[6][n][4] = -0.19;
	after[6][5] = gbestx[6][n][5] = -0.18;//;0.083239;
	after[7][3] = gbestx[7][n][3] = 0.091;
	after[7][4] = gbestx[7][n][4] = 0.23;
	after[7][5] = gbestx[7][n][5] = -0.22;
	after[8][3] = gbestx[8][n][3] = 8.02;// 0.0294169;
	after[8][4] = gbestx[8][n][4] = -25.60;// -0.600919;
	after[8][5] = gbestx[8][n][5] = 17.27;// 0.277191;
	after[9][3] = gbestx[9][n][3] = 13.22;// 0.225171;
	after[9][4] = gbestx[9][n][4] = -8.16;// -0.167816;
	after[9][5] = gbestx[9][n][5] = 5.16;// 0.16257;
	after[10][3] = gbestx[10][n][3] = 2;
	after[10][4] = gbestx[10][n][4] = 0;
	after[10][5] = gbestx[10][n][5] = 0;
	after[11][3] = gbestx[11][n][3] = 0;
	after[11][4] = gbestx[11][n][4] = 0;
	after[11][5] = gbestx[11][n][5] = 0;
	after[12][3] = gbestx[12][n][3] = 0;
	after[12][4] = gbestx[12][n][4] = 0;
	after[12][5] = gbestx[12][n][5] =0;
	after[13][3] = gbestx[13][n][3] = 0;
	after[13][4] = gbestx[13][n][4] =0;
	after[13][5] = gbestx[13][n][5] = 0;
	after[14][3] = gbestx[14][n][3] = 0;
	after[14][4] = gbestx[14][n][4] = 0;
	after[14][5] = gbestx[14][n][5] = 0;
	after[15][3] = gbestx[15][n][3] = 1;
	after[15][4] = gbestx[15][n][4] = 5;
	after[15][5] = gbestx[15][n][5] = -17;
	after[18][3] = gbestx[18][n][3] = 0;
	after[18][4] = gbestx[18][n][4] = 0;
	after[18][5] = gbestx[18][n][5] = 0;
	after[19][3] = gbestx[19][n][3] = 0;
	after[19][4] = gbestx[19][n][4] = 0;
	after[19][5] = gbestx[19][n][5] = 0;
	after[20][3] = gbestx[20][n][3] = 0;
	after[20][4] = gbestx[20][n][4] = 0;
	after[20][5] = gbestx[20][n][5] = 0;
	after[21][3] = gbestx[21][n][3] = 0.;
	after[21][4] = gbestx[21][n][4] = 0.;
	after[21][5] = gbestx[21][n][5] = 0.;
	after[22][3] = gbestx[22][n][3] = 5;
	after[22][4] = gbestx[22][n][4] = -20;
	after[22][5] = gbestx[22][n][5] = -5;
	after[23][3] = gbestx[23][n][3] = 14;
	after[23][4] = gbestx[23][n][4] = 1;
	after[23][5] = gbestx[23][n][5] = 4;
	after[24][3] = gbestx[24][n][3] = 4;
	after[24][4] = gbestx[24][n][4] = -2;
	after[24][5] = gbestx[24][n][5] = -0.5;
	after[25][3] = gbestx[25][n][3] = 12;
	after[25][4] = gbestx[25][n][4] = 0;
	after[25][5] = gbestx[25][n][5] = 0;
	after[26][3] = gbestx[26][n][3] = 12;
	after[26][4] = gbestx[26][n][4] = -4;
	after[26][5] = gbestx[26][n][5] = -4;
	after[27][3] = gbestx[27][n][3] = 5;
	after[27][4] = gbestx[27][n][4] = 0;
	after[27][5] = gbestx[27][n][5] = 2;
	after[28][3] = gbestx[28][n][3] = 0;
	after[28][4] = gbestx[28][n][4] = 0;
	after[28][5] = gbestx[28][n][5] = 0;
	after[29][3] = gbestx[29][n][3] = 0;
	after[29][4] = gbestx[29][n][4] = 0;
	after[29][5] = gbestx[29][n][5] = 0;
	after[30][3] = gbestx[30][n][3] = 0;
	after[30][4] = gbestx[30][n][4] = 0;
	after[30][5] = gbestx[30][n][5] = 0;
	after[31][3] = gbestx[31][n][3] = 0;
	after[31][4] = gbestx[31][n][4] = 0;
	after[31][5] = gbestx[31][n][5] = 0;

}

void GroupPathPlanningPSO::RotateData_data5() {
	//RotateVYZValue(2, 0.99861, -0.028137, -0.0445671, 0.0262163, 0.998726, -0.0431097, 0.0457234, 0.0418814, 0.998076);
	//RotateVYZValue(3, 0.989483, 0.0146661, 0.143907, -0.00553525, 0.997957, -0.0636462, -0.144546, 0.0621802, 0.987542);
	//RotateVYZValue(4, 0.982965, -0.180649, 0.033849, 0.18379, 0.965132, -0.18639, 0.00100248, 0.189436, 0.981893);
	//RotateVYZValue(5, 0.928566, 0.127227, -0.348681, -0.10513, 0.9911, 0.0816641, 0.355967, -0.0391737, 0.933677);
	//RotateVYZValue(6, 0.993933, 0.0988286, 0.0482661, -0.0998285, 0.994828, 0.018759, -0.0461625, -0.0234635, 0.998658);//计算旋转量
	//RotateVYZValue(7, 0.97552, -0.0214298, -0.218866, 0.0255275, 0.999547, 0.0159113, 0.218426, -0.0211089, 0.975625);
	//RotateVYZValue(8, 0.997553, 0.0362973, -0.0597612, -0.0285173, 0.991589, 0.126244, 0.0638409, -0.124231, 0.990197);
	//RotateVYZValue(9, 0.978773, -0.020316, 0.203939, -0.0199927, 0.980864, 0.193663, -0.203971, -0.19363, 0.959637);
	//RotateVYZValue(10, 0.996956, -0.0731726, 0.0269252, 0.0770906, 0.976778, -0.199904, -0.0116725, 0.201371, 0.979446);
	//RotateVYZValue(11, 0.981318, -0.0896943, 0.170205, 0.0861129, 0.995883, 0.028324, -0.172045, -0.013138, 0.985001);
	//RotateVYZValue(12, 0.999814, 0.0116396, -0.015351, -0.0138111, 0.9886, -0.149929, 0.0134309, 0.150113, 0.988578);
	//RotateVYZValue(13, 0.991369, -0.0285493, -0.127952, 0.00853491, 0.987985, -0.154317, 0.13082, 0.151893, 0.979701);
	//RotateVYZValue(14, 0.987985, -0.137236, -0.071076, 0.129625, 0.986255, -0.102464, 0.0841608, 0.0920195, 0.992194);
	//RotateVYZValue(15, 0.991605, -0.104482, 0.07618, 0.105072, 0.994457, -0.00377282, -0.0753636, 0.0117456, 0.997087);
	after[2][3] = gbestx[2][n][3] = 0.0418937;
	after[2][4] = gbestx[2][n][4] = -0.0457795;
	after[2][5] = gbestx[2][n][5] = 0.0281654;
	after[3][3] = gbestx[3][n][3] = 0.0622203;
	after[3][4] = gbestx[3][n][4] = 0.145337;
	after[3][5] = gbestx[3][n][5] = -0.0146951;
	after[4][3] = gbestx[4][n][3] = 0.190588;
	after[4][4] = gbestx[4][n][4] = -0.00102097;
	after[4][5] = gbestx[4][n][5] = 0.185034;
	after[5][3] = gbestx[5][n][3] = -0.0391837;
	after[5][4] = gbestx[5][n][4] = -0.364241;
	after[5][5] = gbestx[5][n][5] = -0.127671;
	after[6][3] = gbestx[6][n][3] = -0.0234657;
	after[6][4] = gbestx[6][n][4] = 0.0461917;
	after[6][5] = gbestx[6][n][5] = -0.0990175;
	after[7][3] = gbestx[7][n][3] = -0.0211105;
	after[7][4] = gbestx[7][n][4] = -0.220251;
	after[7][5] = gbestx[7][n][5] = 0.0214362;
	after[8][3] = gbestx[8][n][3] = -0.124553;
	after[8][4] = gbestx[8][n][4] = -0.0643838;
	after[8][5] = gbestx[8][n][5] = -0.0365889;
	after[9][3] = gbestx[9][n][3] = -0.194861;
	after[9][4] = gbestx[9][n][4] = 0.209433;
	after[9][5] = gbestx[9][n][5] = 0.0207094;
	after[10][3] = gbestx[10][n][3] = 0.202757;
	after[10][4] = gbestx[10][n][4] = 0.0119169;
	after[10][5] = gbestx[10][n][5] = 0.0747726;
	after[11][3] = gbestx[11][n][3] = -0.0131384;
	after[11][4] = gbestx[11][n][4] = 0.17292;
	after[11][5] = gbestx[11][n][5] = 0.0898227;
	after[12][3] = gbestx[12][n][3] = 0.150683;
	after[12][4] = gbestx[12][n][4] = -0.0135852;
	after[12][5] = gbestx[12][n][5] = -0.0117733;
	after[13][3] = gbestx[13][n][3] = 0.152483;
	after[13][4] = gbestx[13][n][4] = -0.132745;
	after[13][5] = gbestx[13][n][5] = 0.0288885;
	after[14][3] = gbestx[14][n][3] = 0.0921499;
	after[14][4] = gbestx[14][n][4] = -0.0846204;
	after[14][5] = gbestx[14][n][5] = 0.138261;
	after[15][3] = gbestx[15][n][3] = 0.0117459;
	after[15][4] = gbestx[15][n][4] = 0.0754403;
	after[15][5] = gbestx[15][n][5] = 0.10468;
}
void GroupPathPlanningPSO::RotateData_data7() {
	/*RotateVYZValue(18, 1, 0, 0, 0, 1, 0, 0, 0, 1);
	RotateVYZValue(19, 0.971284, -0.191031,  0.141828, 0.149053,  0.953184,  0.263103, -0.185449, -0.234408,  0.954286);
	RotateVYZValue(20, 0.956386, -0.200832,  0.212115, 0.144704,  0.956529,  0.253205, -0.253746, -0.211467,  0.943872);
	RotateVYZValue(21, 0.959596, -0.215757,  0.18062, 0.165625,  0.952032,  0.257302, -0.227471, -0.216991,  0.949301);
	RotateVYZValue(22, 0.976774, -0.0931654,  0.19296, 0.0657552,  0.987407,  0.143886, -0.203935, -0.127856,  0.970599);
	RotateVYZValue(23, 0.995941,  0.0235273, - 0.0868814, -0.0315599,  0.995234, - 0.0922705, 0.0842964,  0.0946379,  0.991936);
	RotateVYZValue(24, 0.99367,  0.0218881,  0.110186, -0.0378006,  0.988787,  0.144472, -0.105788, -0.147722,  0.983355);
	RotateVYZValue(25, 0.937801,  0.234353, - 0.256141, -0.152042,  0.940511,  0.303846, 0.31211, - 0.246003,  0.917644);
	RotateVYZValue(26, 0.985766, - 0.0505724,  0.160339, 0.0489947,  0.998704,  0.0137807, -0.160828, - 0.00572874,  0.986966);
	RotateVYZValue(27, 0.972085, - 0.0889731,  0.217106, 0.0751068,  0.994622,  0.0713219, -0.222284, - 0.0530249,  0.973539);
	RotateVYZValue(28, 0.990142,  0.0584213,  0.127305, -0.0819032,  0.978777,  0.187851, -0.113629, - 0.196426,  0.973912);
	RotateVYZValue(29, 0.982872,  0.133863, - 0.126662, -0.104042,  0.970349,  0.218171, 0.152111, - 0.201256,  0.967656);
	RotateVYZValue(30, 0.925667,  0.21212, - 0.313283, -0.139173,  0.960897,  0.239391, 0.351812, - 0.177996 , 0.918992);
	RotateVYZValue(31, 0.973606,  0.042946, - 0.224157, 0.02968,  0.949974,  0.310917, 0.226296, - 0.309364,  0.923626);*/
	/*after[18][3] = gbestx[18][n][3] = 0;
	after[18][4] = gbestx[18][n][4] = -0;
	after[18][5] = gbestx[18][n][5] = -0;
	after[19][3] = gbestx[19][n][3] = -0.23661;
	after[19][4] = gbestx[19][n][4] = 0.19194;
	after[19][5] = gbestx[19][n][5] = 0.197793;
	after[20][3] = gbestx[20][n][3] = -0.213076;
	after[20][4] = gbestx[20][n][4] = 0.262626;
	after[20][5] = gbestx[20][n][5] = 0.206953;
	after[21][3] = gbestx[21][n][3] = -0.218731;
	after[21][4] = gbestx[21][n][4] = 0.235185;
	after[21][5] = gbestx[21][n][5] = 0.222863;
	after[22][3] = gbestx[22][n][3] = -0.128207;
	after[22][4] = gbestx[22][n][4] = 0.2071;
	after[22][5] = gbestx[22][n][5] = 0.0940751;
	after[23][3] = gbestx[23][n][3] = 0.0947797;
	after[23][4] = gbestx[23][n][4] = -0.084778;
	after[23][5] = gbestx[23][n][5] = -0.0236356;
	after[24][3] = gbestx[24][n][3] = -0.148265;
	after[24][4] = gbestx[24][n][4] = 0.107166;
	after[24][5] = gbestx[24][n][5] = -0.0221327;
	after[25][3] = gbestx[25][n][3] = -0.248554;
	after[25][4] = gbestx[25][n][4] = -0.327847;
	after[25][5] = gbestx[25][n][5] = -0.244203;
	after[26][3] = gbestx[26][n][3] = -0.00572877;
	after[26][4] = gbestx[26][n][4] = 0.161532;
	after[26][5] = gbestx[26][n][5] = 0.0505948;
	after[27][3] = gbestx[27][n][3] = -0.0530498;
	after[27][4] = gbestx[27][n][4] = 0.224478;
	after[27][5] = gbestx[27][n][5] = 0.0892167;
	after[28][3] = gbestx[28][n][3] = -0.197712;
	after[28][4] = gbestx[28][n][4] = 0.116148;
	after[28][5] = gbestx[28][n][5] = -0.0596173;
	after[29][3] = gbestx[29][n][3] = -0.20264;
	after[29][4] = gbestx[29][n][4] = -0.155919;
	after[29][5] = gbestx[29][n][5] = -0.137088;
	after[30][3] = gbestx[30][n][3] = -0.17895;
	after[30][4] = gbestx[30][n][4] = -0.365612;
	after[30][5] = gbestx[30][n][5] = -0.217268;
	after[31][3] = gbestx[31][n][3] = -0.314524;
	after[31][4] = gbestx[31][n][4] = -0.240275;
	after[31][5] = gbestx[31][n][5] = -0.0451768;*/
	after[18][3] = gbestx[18][n][3] = 0;
	after[18][4] = gbestx[18][n][4] = -0;
	after[18][5] = gbestx[18][n][5] = -0;
	after[19][3] = gbestx[19][n][3] = -5;
	after[19][4] = gbestx[19][n][4] = 2;
	after[19][5] = gbestx[19][n][5] = 6;
	after[20][3] = gbestx[20][n][3] = -2;
	after[20][4] = gbestx[20][n][4] = 4;
	after[20][5] = gbestx[20][n][5] = 4;
	after[21][3] = gbestx[21][n][3] = -4;
	after[21][4] = gbestx[21][n][4] = 1;
	after[21][5] = gbestx[21][n][5] = 6;
	after[22][3] = gbestx[22][n][3] = -6;
	after[22][4] = gbestx[22][n][4] = 6;
	after[22][5] = gbestx[22][n][5] = 0;
	after[23][3] = gbestx[23][n][3] = 2.5;
	after[23][4] = gbestx[23][n][4] = -2;
	after[23][5] = gbestx[23][n][5] = 0;
	after[24][3] = gbestx[24][n][3] = -6;
	after[24][4] = gbestx[24][n][4] = 4;
	after[24][5] = gbestx[24][n][5] = 0;
	after[25][3] = gbestx[25][n][3] = -18;
	after[25][4] = gbestx[25][n][4] = -4;
	after[25][5] = gbestx[25][n][5] = -2;
	after[26][3] = gbestx[26][n][3] = 0;
	after[26][4] = gbestx[26][n][4] = 0;
	after[26][5] = gbestx[26][n][5] = 0;
	after[27][3] = gbestx[27][n][3] = 0;
	after[27][4] = gbestx[27][n][4] = 1;
	after[27][5] = gbestx[27][n][5] = 0;
	after[28][3] = gbestx[28][n][3] = -8;
	after[28][4] = gbestx[28][n][4] = 2;
	after[28][5] = gbestx[28][n][5] = 2;
	after[29][3] = gbestx[29][n][3] = 0;
	after[29][4] = gbestx[29][n][4] = -4;
	after[29][5] = gbestx[29][n][5] = -4;
	after[30][3] = gbestx[30][n][3] = -12;
	after[30][4] = gbestx[30][n][4] = -8;
	after[30][5] = gbestx[30][n][5] = -8;
	after[31][3] = gbestx[31][n][3] = -2;
	after[31][4] = gbestx[31][n][4] = -5;
	after[31][5] = gbestx[31][n][5] = 0;

	
}
void GroupPathPlanningPSO::RotateData_data8() {
	after[18][3] = gbestx[18][n][3] = -5;
	after[18][4] = gbestx[18][n][4] = 10; 
	after[18][5] = gbestx[18][n][5] = 0;
	after[19][3] = gbestx[19][n][3] = -5;
	after[19][4] = gbestx[19][n][4] = 2;
	after[19][5] = gbestx[19][n][5] = 6;
	after[20][3] = gbestx[20][n][3] = -2;
	after[20][4] = gbestx[20][n][4] = 4;
	after[20][5] = gbestx[20][n][5] = 4;
	after[21][3] = gbestx[21][n][3] = -4;
	after[21][4] = gbestx[21][n][4] = 1;
	after[21][5] = gbestx[21][n][5] = 6;
	after[22][3] = gbestx[22][n][3] = -6;
	after[22][4] = gbestx[22][n][4] = 6;
	after[22][5] = gbestx[22][n][5] = 0;
	after[23][3] = gbestx[23][n][3] = 2.5;
	after[23][4] = gbestx[23][n][4] = -2;
	after[23][5] = gbestx[23][n][5] = 0;
	after[24][3] = gbestx[24][n][3] = -6;
	after[24][4] = gbestx[24][n][4] = 8;
	after[24][5] = gbestx[24][n][5] = 6;
	after[25][3] = gbestx[25][n][3] = -18;
	after[25][4] = gbestx[25][n][4] = -4;
	after[25][5] = gbestx[25][n][5] = -2;
	after[26][3] = gbestx[26][n][3] = 0;
	after[26][4] = gbestx[26][n][4] = 0;
	after[26][5] = gbestx[26][n][5] = 0;
	after[27][3] = gbestx[27][n][3] = 0;
	after[27][4] = gbestx[27][n][4] = 1;
	after[27][5] = gbestx[27][n][5] = 0;
	after[28][3] = gbestx[28][n][3] = -8;
	after[28][4] = gbestx[28][n][4] = 2;
	after[28][5] = gbestx[28][n][5] = 2;
	after[29][3] = gbestx[29][n][3] = 0;
	after[29][4] = gbestx[29][n][4] = -4;
	after[29][5] = gbestx[29][n][5] = -4;
	after[30][3] = gbestx[30][n][3] = -12;
	after[30][4] = gbestx[30][n][4] = -8;
	after[30][5] = gbestx[30][n][5] = -8;
	after[31][3] = gbestx[31][n][3] = -2;
	after[31][4] = gbestx[31][n][4] = -5;
	after[31][5] = gbestx[31][n][5] = 0;


}
void GroupPathPlanningPSO::ToothTarget() {//测试最终位置数据
	//for (int i = 1; i <= 15; i++) {
	//	Tooth* tooth = m_OrthoData->GetTooth((ToothId)i);
	//	if (tooth == NULL) continue;
	//	vtkSmartPointer<vtkPolyDataMapper> mapper =
	//		vtkSmartPointer<vtkPolyDataMapper>::New();
	//	mapper->SetInputData(tooth->GetToothPolyData());
	//	vtkSmartPointer<vtkActor>  actor =
	//		vtkSmartPointer<vtkActor>::New();
	//	actor->SetMapper(mapper);
	//	//修改透明度和颜色
	//	actor->GetProperty()->SetOpacity(1);
	//	actor->GetProperty()->SetColor(1, 0, 0);
	//	m_RenderContainer->GetRenderer(MainRenderer)->AddActor(actor);

	//	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	//	transform->Translate(before[i][0], before[i][1], before[i][2]);
	//	transform->RotateWXYZ(after[i][3], 1, 0, 0);
	//	transform->RotateWXYZ(after[i][4], 0, 1, 0);
	//	transform->RotateWXYZ(after[i][5], 0, 0, 1);
	//	transform->Translate(-before[i][0], -before[i][1], -before[i][2]);
	//	transform->Translate(after[i][0] - before[i][0], after[i][1] - before[i][1], after[i][2] - before[i][2]);//平移	
	//	transform->Update();
	//	actor->SetUserTransform(transform);
	//}
	//
	
	for (int i = 18; i <= 31; i++) {
		Tooth* tooth = m_OrthoData->GetTooth((ToothId)i);
		if (tooth == NULL) continue;
		
		
		//使用vtkMatrix4x4矩阵使其变换到理想位置
    	vtkSmartPointer<vtkMatrix4x4> matriedec = vtkSmartPointer<vtkMatrix4x4>::New();
	    for (int j = 0; j < 3; j++) {
		    for (int k = 0; k < 3; k++) {
		    	matriedec->SetElement(j, k, Matrix[i](j, k));
		    }
	    }
		matriedec->SetElement(0, 3, 0);
		matriedec->SetElement(1, 3, 0);
		matriedec->SetElement(2, 3, 0);
		matriedec->SetElement(3, 0, 0);
		matriedec->SetElement(3, 1, 0);
		matriedec->SetElement(3, 2, 0);
		matriedec->SetElement(3, 3, 1);
		tooth->GetToothActor()->SetUserMatrix(matriedec);
		//vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		////transform->Translate(0, 0, 0);
		////transform->Translate(after[i][0], after[i][1], after[i][2]);//平移
		////myTrans->Translate(origin[0], origin[1], origin[2]);
		//transform->RotateY(after[i][4]);
		//transform->RotateX(after[i][3]);
		//transform->RotateZ(after[i][5]);
		//transform->Translate(after[i][0] - before[i][0], after[i][1] - before[i][1], after[i][2] - before[i][2]);//平移	
		//transform->Translate(after[i][0], after[i][1], after[i][2]);//平移
		/*transform1->RotateWXYZ(orthophase[k][3], axes_x[0], axes_x[1], axes_x[2]);
		transform1->RotateWXYZ(orthophase[k][4], axes_y[0], axes_y[1], axes_y[2]);
		transform1->RotateWXYZ(orthophase[k][5], axes_z[0], axes_z[1], axes_z[2]);*/
		//transform->Update();
		//tooth->GetToothActor()->SetUserTransform(transform);
		tooth->GetToothActor()->SetVisibility(true);
		//vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		//transform->Translate(centerTest[0], centerTest[1], centerTest[2]);
		//transform->RotateWXYZ(0, 0, 1, 0);
		//transform->RotateWXYZ(90, 1, 0, 0);
		//transform->RotateWXYZ(0, 0, 0, 1);
		//transform->Translate(-centerTest[0], -centerTest[1], -centerTest[2]);
		////transform->Translate(after[7][0] - before[7][0], after[7][1] - before[7][1], after[7][2] - before[7][2]);//平移	
		//transform->Update();
		//actorTest->SetUserTransform(transform);

		
	}

	m_RenderContainer->Render();
}



void GenerateCosineSequence(double* seq, int length) {
        seq[0] = (double)rand() / RAND_MAX;  // 初始值∈[0,1)
        for (int i = 1; i < length; i++) {
            seq[i] = k * cos(PI * seq[i-1]);  // 余弦混沌映射：a_{m+1}=k*cos(π*a_m)
        }
 }

void GroupPathPlanningPSO::Orthodontic(particle* part, int it, double y) {
	// 算法参数（保持自适应策略）
	const double w = 0.5;
	const double Wmax = 0.8;
	const double Wmin = 0.4;
	const double Bmax = 1;
	const double Bmin = 0.7;
	const int c1 = 1.4;       // 个体认知因子
	const int c2 = 1.8;       // 社会引导因子
	double r1 = (2 * (rand() % (N + 1) / (float)(N + 1))) - 1;  // 随机数[-1,1]
	double r2 = (2 * (rand() % (N + 1) / (float)(N + 1))) - 1;  // 随机数[-1,1]

	// 遍历上颌（a=0）和下颌（a=1）
	for (int a = 0; a < 2; a++) {
		int startJ, endJ;
		if (a == 0) {
			startJ = 2;   // 上颌牙齿ID范围：2-15
			endJ = 16;
		}
		else {
			startJ = 18;  // 下颌牙齿ID范围：18-31
			endJ = 32;
		}

		// 遍历当前颌的每个牙齿
		for (int j = startJ; j < endJ; j++) {
			// 跳过无效牙齿（未加载或不存在的牙齿）
			Tooth* tooth = m_OrthoData->GetTooth((ToothId)j);
			if (!tooth) {
				cout << "跳过无效牙齿ID：" << j << endl;
				continue;
			}

			// 遍历每个路径阶段（k=2到k<n，中间路径点）
			for (int k = 2; k < n; k++) {
				// --------------------------
				// 1. 平移维度更新（0-2：x、y、z）
				// --------------------------
				for (int d = 0; d < 3; d++) {
					// 自适应速度更新公式
					// 惯性项：随迭代从1线性衰减到0
					double inertia = 1 - (it * 1.0 / (iter_max - 1));
					// 个体认知项：sin因子（随迭代从0增至1）
					double sinFactor = sin(it * PI * 0.5 / (iter_max - 1));
					// 社会引导项：cos因子（随迭代从1减至0）
					double cosFactor = cos(it * PI * 0.5 / (iter_max - 1));

					// 速度更新（融合惯性、个体最优、全局最优）
					part->v[j][k][d] =
						inertia * part->v[j][k][d] +
						sinFactor * c1 * r1 * (part->bestx[j][k][d] - part->x[j][k][d]) +
						cosFactor * c2 * r2 * (gbestx[j][k][d] - part->x[j][k][d]);

					// 位置更新（当前位置 + 速度）
					part->x[j][k][d] += part->v[j][k][d];

					// 平移边界约束（基于初始与目标位置的动态范围）
					double diff = abs(before[j][d] - after[j][d]);  // 初始到目标的总平移差
					// 动态调整边界：随迭代接近目标，范围逐渐缩小
					double minTrans = (before[j][d] + after[j][d]) / 2
						- 0.5 * (1 - it / (double)(iter_max - 1))  // 随迭代衰减的范围
						- 0.2 * diff  // 与总平移差相关的范围
						- 1;          // 固定偏移
					double maxTrans = (before[j][d] + after[j][d]) / 2
						+ 0.5 * (1 - it / (double)(iter_max - 1))
						+ 0.2 * diff
						+ 1;
					// 强制位置在边界内
					part->x[j][k][d] = max(min(part->x[j][k][d], maxTrans), minTrans);
				}

				// --------------------------
				// 关键修复：三维综合平移约束（单步≤0.3mm）
				// --------------------------
				double dx = part->x[j][k][0] - part->x[j][k - 1][0];  // x方向单步平移
				double dy = part->x[j][k][1] - part->x[j][k - 1][1];  // y方向单步平移
				double dz = part->x[j][k][2] - part->x[j][k - 1][2];  // z方向单步平移
				double step3D = sqrt(dx * dx + dy * dy + dz * dz);        // 三维综合平移量

				if (step3D > 0.3) {  // 超过单步最大允许平移（0.3mm）
					double scale = 0.3 / step3D;  // 缩放因子（将综合平移量缩至0.3mm）
					// 按比例调整三维坐标，确保单步综合平移不超标
					part->x[j][k][0] = part->x[j][k - 1][0] + dx * scale;
					part->x[j][k][1] = part->x[j][k - 1][1] + dy * scale;
					part->x[j][k][2] = part->x[j][k - 1][2] + dz * scale;
				}

				// --------------------------
				// 2. 旋转维度更新（3-5：绕x、y、z轴旋转）
				// --------------------------
				for (int d = 3; d < 6; d++) {
					// 自适应速度更新（与平移维度策略一致）
					double inertia = 1 - (it * 1.0 / (iter_max - 1));
					double sinFactor = sin(it * PI * 0.5 / (iter_max - 1));
					double cosFactor = cos(it * PI * 0.5 / (iter_max - 1));

					// 速度更新
					part->v[j][k][d] =
						inertia * part->v[j][k][d] +
						sinFactor * c1 * r1 * (part->bestx[j][k][d] - part->x[j][k][d]) +
						cosFactor * c2 * r2 * (gbestx[j][k][d] - part->x[j][k][d]);

					// 位置更新
					part->x[j][k][d] += part->v[j][k][d];

					// 旋转边界约束（基于初始与目标位置的动态范围）
					double diff = abs(before[j][d] - after[j][d]);  // 初始到目标的总旋转差
					// 动态调整边界：随迭代接近目标，范围逐渐缩小
					double minRot = (before[j][d] + after[j][d]) / 2
						- 0.5 * (1 - it / (double)(iter_max - 1))
						- 0.2 * diff
						- 1;
					double maxRot = (before[j][d] + after[j][d]) / 2
						+ 0.5 * (1 - it / (double)(iter_max - 1))
						+ 0.2 * diff
						+ 1;
					// 强制旋转在边界内
					part->x[j][k][d] = max(min(part->x[j][k][d], maxRot), minRot);
				}

				// --------------------------
				// 旋转单步约束（单步≤2°，转换为弧度）
				// --------------------------
				double rotX = part->x[j][k][3] - part->x[j][k - 1][3];  // x轴单步旋转
				double rotY = part->x[j][k][4] - part->x[j][k - 1][4];  // y轴单步旋转
				double rotZ = part->x[j][k][5] - part->x[j][k - 1][5];  // z轴单步旋转
				const double MAX_ROT_STEP = 2 * PI / 180;  // 2°对应的弧度（≈0.0349）

				// 检查x轴旋转是否超标
				if (abs(rotX) > MAX_ROT_STEP) {
					part->x[j][k][3] = part->x[j][k - 1][3] + (rotX > 0 ? MAX_ROT_STEP : -MAX_ROT_STEP);
				}
				// 检查y轴旋转是否超标
				if (abs(rotY) > MAX_ROT_STEP) {
					part->x[j][k][4] = part->x[j][k - 1][4] + (rotY > 0 ? MAX_ROT_STEP : -MAX_ROT_STEP);
				}
				// 检查z轴旋转是否超标
				if (abs(rotZ) > MAX_ROT_STEP) {
					part->x[j][k][5] = part->x[j][k - 1][5] + (rotZ > 0 ? MAX_ROT_STEP : -MAX_ROT_STEP);
				}
			}
		}
	}
}

void GroupPathPlanningPSO::OrthodonticPSO(particle* part, int it, double y) {//每个牙齿每次循环迭代移动后的相关坐标
	//初始化相关参数
	const double w = 0.9;
	const int c1 = 2;
	const int c2 = 2;
	//double r1 =  rand() % (N + 1) / (float)(N + 1);//r1为随机数，取值范围[-1,1]
	//double r2 =  rand() % (N + 1) / (float)(N + 1);//r2为随机数，取值范围[-1,1]
	// 修改随机数生成逻辑（在OrthodonticPSO中）
	double r1 = (rand() % (2 * (N + 1)) / (float)(N + 1)) - 1; // 范围：[-1,1]
	double r2 = (rand() % (2 * (N + 1)) / (float)(N + 1)) - 1;

	double temp;
	////传统粒子群算法进行更新
	for (int j = 2; j < 16; j++) {//m个牙齿
		for (int k = 2; k < n; k++) {//m*n个阶段//阶段1忽略
			for (int d = 0; d < 6; d++) {//d维
				part->v[j][k][d] = w * part->v[j][k][d] + c1 * r1 * (part->bestx[j][k][d] - part->x[j][k][d]) + c2 * r2 * (gbestx[j][k][d] - part->x[j][k][d]);
				part->x[j][k][d] = part->x[j][k][d] + part->v[j][k][d];
			}
		}
	}
	for (int j = 18; j < 32; j++) {//m个牙齿
		for (int k = 2; k < n; k++) {//m*n个阶段//阶段1忽略
			for (int d = 0; d < 6; d++) {//d维
				part->v[j][k][d] = w * part->v[j][k][d] + c1 * r1 * (part->bestx[j][k][d] - part->x[j][k][d]) + c2 * r2 * (gbestx[j][k][d] - part->x[j][k][d]);
				part->x[j][k][d] = part->x[j][k][d] + part->v[j][k][d];
			}
		}
	}

}




int GroupPathPlanningPSO::round_double(double number)
{
	return (number > 0.0) ? (number + 0.5) : (number - 0.5);
}

//void GroupPathPlanningPSO::OrthodonticXYZ(particle* part, int it, double y) {
//	for (int b = 0; b < 2; b++) {
//		for (int s = 2 + 16 * b; s < 16 + 16 * b; s++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
//			for (int u = 1; u < n; u++) {
//				//超出边界，重新更新位置
//				for (int m = 0; m < 3; m++) {//  3 维
//					double diff = abs(before[s][m] - after[s][m]);
//					double min = (before[s][m] + after[s][m]) / 2 - 0.8 * diff;
//					double max = (before[s][m] + after[s][m]) / 2 + 0.8 * diff;
//					if (part->x[s][u][m] < min || part->x[s][u][m] > max)    Orthodontic(part, it, y);
//				}
//				/*std::vector<double> before1;
//				m_OrthoData->GetCenterBefore((ToothId)s, before1);
//				std::vector<double> after1;
//				m_OrthoData->GetCenterAfter((ToothId)s, after1);*/
//				//for (int m = 0; m < 3; m++) {//  3 维
//				//	double diff = abs(before1[m] - after1[m]);
//				//	double min = (before1[m] + after1[m]) / 2 - 0.8 * diff;
//				//	double max = (before1[m] + after1[m]) / 2 + 0.8 * diff;
//				//	if (p->x[s][u][m] < min || p->x[s][u][m] > max)
//				//		Orthodontic(p, iter, Z[z]);
//				//}
//			}
//		}
//	}
//}
void GroupPathPlanningPSO::Orthodontic2(particle* part, int it) {//每个牙齿每次循环迭代移动后的相关坐标
	//初始化相关参数
	//it是当前迭代次数，iter_max是总迭代次数
	double R = rand() % (N + 1) / (float)(N + 1);//随机数R:0<R<1, 
	double Wmax = 0.8;//w为惯性权重，取值范围[0,1],粒子速度更新公式使用
	double Wmin = 0.4;
	double Bmax = 1;
	double Bmin = 0.7;
	int c1 = 2;//c1、c2为学习因子，粒子速度更新公式使用
	int c2 = 2;
	double r1 = (2 * (rand() % (N + 1) / (float)(N + 1))) - 1;
	double r2 = (5 * (rand() % (N + 1) / (float)(N + 1))) - 2.5;
	//进行更新
	for (int j = 1; j < m; j++) {//m个牙齿
		for (int k = 1; k < n; k++) {//m*n个阶段
			for (int d = 0; d < 6; d++) {//d维
				if (R < 0.95) {
					part->v[j][k][d] = (Wmax - ((Wmax - Wmin) * sin(it * PI * 0.5 / (iter_max - 1)))) * part->v[j][k][d] + c1 * r1 * (part->bestx[j][k][d] - part->x[j][k][d]) + (Bmax - ((Bmax - Bmin) * cos(it * PI * 0.5 / (iter_max - 1)))) * c2 * r2 * (gbestx[j][k][d] - part->x[j][k][d]);
					part->x[j][k][d] = part->x[j][k][d] + part->v[j][k][d];
				}
				else
				{
					part->v[j][k][d] = Wmax * part->v[j][k][d] + c1 * r1 * (part->bestx[j][k][d] - part->x[j][k][d]) + c2 * r2 * (gbestx[j][k][d] - part->x[j][k][d]);
					part->x[j][k][d] = part->x[j][k][d] + part->v[j][k][d];
				}
			}
		}
		//cout << "更新sin(it * PI :" << part->v[j][4][4] << endl;
	}
}

double GroupPathPlanningPSO::OBB(particle* part, int j) {
	double fc = 0;
	for (int k = 2; k < n; k++) {//循环每个牙齿的每个的中间路径点
		int tooth1 = j - 1;
		int tooth2 = j + 1;
		Tooth* T = m_OrthoData->GetTooth((ToothId)j);
		Tooth* Tleft = m_OrthoData->GetTooth((ToothId)tooth1);
		Tooth* Tright = m_OrthoData->GetTooth((ToothId)tooth2);
		//添加初始牙齿到renderer，便于窗口可视化对比
		vtkSmartPointer<vtkPolyDataMapper> mapperT =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapperT->SetInputData(T->GetToothPolyData());
		vtkSmartPointer<vtkActor>  actorT =
			vtkSmartPointer<vtkActor>::New();
		actorT->SetMapper(mapperT);


		//根据平移旋转量 更新 第j个牙齿和相邻牙齿的polydata
		vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		transform->Translate(before[j][0], before[j][1], before[j][2]);
		transform->RotateWXYZ(part->x[j][k][3], 1, 0, 0);
		transform->RotateWXYZ(part->x[j][k][4], 0, 1, 0);
		transform->RotateWXYZ(part->x[j][k][5], 0, 0, 1);
		transform->Translate(-before[j][0], -before[j][1], -before[j][2]);
		transform->Translate(part->x[j][k][0] - before[j][0], part->x[j][k][1] - before[j][1], part->x[j][k][2] - before[j][2]);//平移	
		transform->Update();
		vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyData = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		transformPolyData->SetInputData(T->GetToothPolyData());
		//transformPolyData->SetInputConnection(mapperT->GetOutputPort());
		transformPolyData->SetTransform(transform);
		transformPolyData->Update();


		if (Tleft)
		{
			vtkSmartPointer<vtkPolyDataMapper> mapperTleft =
				vtkSmartPointer<vtkPolyDataMapper>::New();
			mapperTleft->SetInputData(Tleft->GetToothPolyData());
			vtkSmartPointer<vtkActor>  actorTleft =
				vtkSmartPointer<vtkActor>::New();
			actorTleft->SetMapper(mapperTleft);
			vtkSmartPointer<vtkTransform> transformLeft = vtkSmartPointer<vtkTransform>::New();
			transformLeft->Translate(before[j][0], before[j][1], before[j][2]);
			transformLeft->RotateWXYZ(part->x[tooth1][k][3], 1, 0, 0);
			transformLeft->RotateWXYZ(part->x[tooth1][k][4], 0, 1, 0);
			transformLeft->RotateWXYZ(part->x[tooth1][k][5], 0, 0, 1);
			transformLeft->Translate(-before[j][0], -before[j][1], -before[j][2]);
			transformLeft->Translate(part->x[tooth1][k][0] - before[tooth1][0], part->x[tooth1][k][1] - before[tooth1][1], part->x[tooth1][k][2] - before[tooth1][2]);//平移
			transformLeft->Update();
			vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyDataLeft = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformPolyDataLeft->SetInputData(Tleft->GetToothPolyData());
			//transformPolyDataLeft->SetInputConnection(mapperTleft->GetOutputPort());
			transformPolyDataLeft->SetTransform(transformLeft);
			transformPolyDataLeft->Update();
			if (CollisionOn(transformPolyDataLeft->GetOutput(), transformPolyData->GetOutput(), j, k)) {
				fc = 10000000;//发生碰撞
				continue;
			}
			//else {
			//	for (int p = 1; p < 4; p++) {
			//		//添加初始牙齿到renderer，便于窗口可视化对比
			//		vtkSmartPointer<vtkPolyDataMapper> mapperT1 =
			//			vtkSmartPointer<vtkPolyDataMapper>::New();
			//		mapperT1->SetInputData(T->GetToothPolyData());
			//		vtkSmartPointer<vtkActor>  actorT1 =
			//			vtkSmartPointer<vtkActor>::New();
			//		actorT1->SetMapper(mapperT1);

			//		//根据平移旋转量 更新 第j个牙齿和相邻牙齿的polydata
			//		vtkSmartPointer<vtkTransform> transform1 = vtkSmartPointer<vtkTransform>::New();
			//		transform1->Translate(before[j][0], before[j][1], before[j][2]);
			//		transform1->RotateWXYZ((part->x[j][k][3]) * p / 4, 1, 0, 0);
			//		transform1->RotateWXYZ((part->x[j][k][4]) * p / 4, 0, 1, 0);
			//		transform1->RotateWXYZ((part->x[j][k][5]) * p / 4, 0, 0, 1);
			//		transform1->Translate(-before[j][0], -before[j][1], -before[j][2]);
			//		transform1->Translate((part->x[j][k][0] - before[j][0]) * p / 4, (part->x[j][k][1] - before[j][1]) * p / 4, (part->x[j][k][2] - before[j][2]) * p / 4);//平移	
			//		transform1->Update();
			//		vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyData1 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//		transformPolyData1->SetInputData(T->GetToothPolyData());
			//		//transformPolyData->SetInputConnection(mapperT->GetOutputPort());
			//		transformPolyData1->SetTransform(transform1);
			//		transformPolyData1->Update();

			//		vtkSmartPointer<vtkPolyDataMapper> mapperTleft1 =
			//			vtkSmartPointer<vtkPolyDataMapper>::New();
			//		mapperTleft1->SetInputData(Tleft->GetToothPolyData());
			//		vtkSmartPointer<vtkActor>  actorTleft1 =
			//			vtkSmartPointer<vtkActor>::New();
			//		actorTleft1->SetMapper(mapperTleft1);
			//		vtkSmartPointer<vtkTransform> transformLeft1 = vtkSmartPointer<vtkTransform>::New();
			//		transformLeft1->Translate(before[j][0], before[j][1], before[j][2]);
			//		transformLeft1->RotateWXYZ((part->x[tooth1][k][3]) * p / 4, 1, 0, 0);
			//		transformLeft1->RotateWXYZ((part->x[tooth1][k][4]) * p / 4, 0, 1, 0);
			//		transformLeft1->RotateWXYZ((part->x[tooth1][k][5]) * p / 4, 0, 0, 1);
			//		transformLeft1->Translate(-before[j][0], -before[j][1], -before[j][2]);
			//		transformLeft1->Translate((part->x[tooth1][k][0] - before[tooth1][0]) * p / 4, (part->x[tooth1][k][1] - before[tooth1][1])* p / 4, (part->x[tooth1][k][2] - before[tooth1][2]) * p / 4);//平移
			//		transformLeft1->Update();
			//		vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyDataLeft1 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//		transformPolyDataLeft1->SetInputData(Tleft->GetToothPolyData());
			//		//transformPolyDataLeft->SetInputConnection(mapperTleft->GetOutputPort());
			//		transformPolyDataLeft1->SetTransform(transformLeft1);
			//		transformPolyDataLeft1->Update();
			//		if (CollisionOn(transformPolyDataLeft1->GetOutput(), transformPolyData1->GetOutput(), j, k)) {
			//			fc = 100000;//发生碰撞
			//			continue;
			//		}
			//	}
			//	
			//
			//
			//}


			//vtkSmartPointer<vtkTransform> transform2 = vtkSmartPointer<vtkTransform>::New();
			//transform2->Translate(-1 * (part->x[j][k][0] - before[j][0]), -1 * (part->x[j][k][1] - before[j][1]), -1 * (part->x[j][k][2] - before[j][2]));//平移
			//transform2->Translate(before[j][0], before[j][1], before[j][2]);
			//transform2->RotateWXYZ(-1 * (part->x[j][k][5]), 0, 0, 1);
			//transform2->RotateWXYZ(-1 * (part->x[j][k][4]), 0, 1, 0);
			//transform2->RotateWXYZ(-1 * (part->x[j][k][3]), 1, 0, 0);
			//transform2->Translate(-before[j][0], -before[j][1], -before[j][2]);
			//transform2->Update();
			//vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyData2 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//transformPolyData2->SetInputData(T->GetToothPolyData());
			////transformPolyData2->SetInputConnection(mapperT->GetOutputPort());
			//transformPolyData2->SetTransform(transform2);
			//transformPolyData2->Update();
			//vtkSmartPointer<vtkTransform> transformLeft2 = vtkSmartPointer<vtkTransform>::New();
			//transformLeft2->RotateX(-1 * (part->x[tooth1][k][3]));
			//transformLeft2->RotateY(-1 * (part->x[tooth1][k][4]));
			//transformLeft2->RotateZ(-1 * (part->x[tooth1][k][5]));
			//transformLeft2->Translate(-1 * (part->x[tooth1][k][0] - before[tooth1][0]), -1 * (part->x[tooth1][k][1] - before[tooth1][1]), -1 * (part->x[tooth1][k][2] - before[tooth1][2]));//平移
			//transformLeft2->Update();
			//vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyDataLeft2 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//transformPolyDataLeft2->SetInputData(Tleft->GetToothPolyData());
			////transformPolyDataLeft2->SetInputConnection(mapperTleft->GetOutputPort());
			//transformPolyDataLeft2->SetTransform(transformLeft2);
			//transformPolyDataLeft2->Update();
			//vtkSmartPointer<vtkFeatureEdges> featureEdges = vtkSmartPointer<vtkFeatureEdges>::New();
			//featureEdges->SetInputData(transformPolyData->GetOutput());
			//featureEdges->BoundaryEdgesOn();
			//featureEdges->FeatureEdgesOff();
			//featureEdges->ManifoldEdgesOff();
			//featureEdges->NonManifoldEdgesOff();
			//featureEdges->Update();
			//int numberOfOpenEdges = featureEdges->GetOutput()->GetNumberOfCells();
			//if (numberOfOpenEdges)
			//{
			//	std::cout << "该模型不是封闭的......." << std::endl;
			//}
			//else
			//{
			//	std::cout << "该模型是封闭的......." << std::endl;
			//}
			//vtkSmartPointer<vtkPolyData> points = vtkSmartPointer<vtkPolyData>::New();
			//points->SetPoints(transformPolyData->GetOutput()->GetPoints());
			//vtkSmartPointer<vtkSelectEnclosedPoints> selectEnclosedPoints = vtkSmartPointer<vtkSelectEnclosedPoints>::New();
			//selectEnclosedPoints->SetInputData(points);
			//selectEnclosedPoints->SetSurfaceData(transformPolyDataLeft->GetOutput());
			//selectEnclosedPoints->SetTolerance(0.00001);//设置容差
			//selectEnclosedPoints->Update();
			//bool inside = false;

			//for (int i = 0; i < transformPolyData->GetOutput()->GetNumberOfPoints(); ++i)
			//{
			//	if (selectEnclosedPoints->IsInside(i) == 1)
			//	{
			//		//发生碰撞
			//		inside = true;
			//     std::cout << "发生碰撞 << std::endl;
			//		break;
			//	}
			//}

		}
		if (Tright)
		{
			vtkSmartPointer<vtkPolyDataMapper> mapperTright =
				vtkSmartPointer<vtkPolyDataMapper>::New();
			mapperTright->SetInputData(Tright->GetToothPolyData());
			vtkSmartPointer<vtkActor>  actorTright =
				vtkSmartPointer<vtkActor>::New();
			actorTright->SetMapper(mapperTright);
			vtkSmartPointer<vtkTransform> transformRight = vtkSmartPointer<vtkTransform>::New();
			transformRight->Translate(before[j][0], before[j][1], before[j][2]);
			transformRight->RotateWXYZ(part->x[tooth2][k][3], 1, 0, 0);
			transformRight->RotateWXYZ(part->x[tooth2][k][4], 0, 1, 0);
			transformRight->RotateWXYZ(part->x[tooth2][k][5], 0, 0, 1);
			transformRight->Translate(-before[j][0], -before[j][1], -before[j][2]);
			transformRight->Translate(part->x[tooth2][k][0] - before[tooth2][0], part->x[tooth2][k][1] - before[tooth2][1], part->x[tooth2][k][2] - before[tooth2][2]);//平移
			transformRight->Update();
			vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyDataRight = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformPolyDataRight->SetInputData(Tright->GetToothPolyData());
			//transformPolyDataRight->SetInputConnection(mapperTright->GetOutputPort());
			transformPolyDataRight->SetTransform(transformRight);
			transformPolyDataRight->Update();
			if (CollisionOn(transformPolyDataRight->GetOutput(), transformPolyData->GetOutput(), j, k)) {
				fc = 10000000; //发生碰撞
				continue;
			}
			//else {
			//	for (int q = 1; q < 4; q++) {
			//		//添加初始牙齿到renderer，便于窗口可视化对比
			//		vtkSmartPointer<vtkPolyDataMapper> mapperT1 =
			//			vtkSmartPointer<vtkPolyDataMapper>::New();
			//		mapperT1->SetInputData(T->GetToothPolyData());
			//		vtkSmartPointer<vtkActor>  actorT1 =
			//			vtkSmartPointer<vtkActor>::New();
			//		actorT1->SetMapper(mapperT1);

			//		//根据平移旋转量 更新 第j个牙齿和相邻牙齿的polydata
			//		vtkSmartPointer<vtkTransform> transform1 = vtkSmartPointer<vtkTransform>::New();
			//		transform1->Translate(before[j][0], before[j][1], before[j][2]);
			//		transform1->RotateWXYZ((part->x[j][k][3]) * q / 4, 1, 0, 0);
			//		transform1->RotateWXYZ((part->x[j][k][4]) * q / 4, 0, 1, 0);
			//		transform1->RotateWXYZ((part->x[j][k][5]) * q / 4, 0, 0, 1);
			//		transform1->Translate(-before[j][0], -before[j][1], -before[j][2]);
			//		transform1->Translate((part->x[j][k][0] - before[j][0]) * q / 4, (part->x[j][k][1] - before[j][1]) * q / 4, (part->x[j][k][2] - before[j][2]) * q / 4);//平移	
			//		transform1->Update();
			//		vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyData1 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//		transformPolyData1->SetInputData(T->GetToothPolyData());
			//		//transformPolyData->SetInputConnection(mapperT->GetOutputPort());
			//		transformPolyData1->SetTransform(transform1);
			//		transformPolyData1->Update();

			//		vtkSmartPointer<vtkPolyDataMapper> mapperTright1 =
			//			vtkSmartPointer<vtkPolyDataMapper>::New();
			//		mapperTright1->SetInputData(Tright->GetToothPolyData());
			//		vtkSmartPointer<vtkActor>  actorTright1 =
			//			vtkSmartPointer<vtkActor>::New();
			//		actorTright1->SetMapper(mapperTright1);
			//		vtkSmartPointer<vtkTransform> transformRight1 = vtkSmartPointer<vtkTransform>::New();
			//		transformRight1->Translate(before[j][0], before[j][1], before[j][2]);
			//		transformRight1->RotateWXYZ((part->x[tooth2][k][3])* q / 4, 1, 0, 0);
			//		transformRight1->RotateWXYZ((part->x[tooth2][k][4])* q / 4, 0, 1, 0);
			//		transformRight1->RotateWXYZ((part->x[tooth2][k][5])* q / 4, 0, 0, 1);
			//		transformRight1->Translate(-before[j][0], -before[j][1], -before[j][2]);
			//		transformRight1->Translate((part->x[tooth2][k][0] - before[tooth2][0])* q / 4, (part->x[tooth2][k][1] - before[tooth2][1])* q / 4, (part->x[tooth2][k][2] - before[tooth2][2])* q / 4);//平移
			//		transformRight1->Update();
			//		vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyDataRight1 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//		transformPolyDataRight1->SetInputData(Tright->GetToothPolyData());
			//		//transformPolyDataRight->SetInputConnection(mapperTright->GetOutputPort());
			//		transformPolyDataRight1->SetTransform(transformRight);
			//		transformPolyDataRight1->Update();
			//		if (CollisionOn(transformPolyDataRight1->GetOutput(), transformPolyData1->GetOutput(), j, k)) {
			//			fc = 100000; //发生碰撞
			//			continue;
			//		}
			//	}
			//}
			//vtkSmartPointer<vtkTransform> transform3 = vtkSmartPointer<vtkTransform>::New();
			//transform3->RotateX(-1 * (part->x[j][k][3]));
			//transform3->RotateY(-1 * (part->x[j][k][4]));
			//transform3->RotateZ(-1 * (part->x[j][k][5]));
			//transform3->Translate(-1 * (part->x[j][k][0] - before[j][0]), -1 * (part->x[j][k][1] - before[j][1]), -1 * (part->x[j][k][2] - before[j][2]));//平移
			//transform3->Update();
			//vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyData3 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//transformPolyData3->SetInputData(T->GetToothPolyData());
			////transformPolyData3->SetInputConnection(mapperT->GetOutputPort());
			//transformPolyData3->SetTransform(transform3);
			//transformPolyData3->Update();
			//vtkSmartPointer<vtkTransform> transformRight2 = vtkSmartPointer<vtkTransform>::New();
			//transformRight2->RotateX(-1 * (part->x[tooth2][k][3]));
			//transformRight2->RotateY(-1 * (part->x[tooth2][k][4]));
			//transformRight2->RotateZ(-1 * (part->x[tooth2][k][5]));
			//transformRight2->Translate(-1 * (part->x[tooth2][k][0] - before[tooth2][0]), -1 * (part->x[tooth2][k][1] - before[tooth2][1]), -1 * (part->x[tooth2][k][2] - before[tooth2][2]));//平移
			//transformRight2->Update();
			//vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyDataRight2 = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			//transformPolyDataRight2->SetInputData(Tright->GetToothPolyData());
			////transformPolyDataRight2->SetInputConnection(mapperTright->GetOutputPort());
			//transformPolyDataRight2->SetTransform(transformRight2);
			//transformPolyDataRight2->Update();
		}
	}
	return fc;

}


double GroupPathPlanningPSO::GbestpOBB(tempgbest* part, int j) {
	double fc = 0;
	for (int k = 1; k < n; k++) {//循环每个牙齿的每个的中间路径点
		int tooth1 = j - 1;
		int tooth2 = j + 1;
		Tooth* T = m_OrthoData->GetTooth((ToothId)j);
		Tooth* Tleft = m_OrthoData->GetTooth((ToothId)tooth1);
		Tooth* Tright = m_OrthoData->GetTooth((ToothId)tooth2);
		//添加初始牙齿到renderer，便于窗口可视化对比
		vtkSmartPointer<vtkPolyDataMapper> mapperT =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapperT->SetInputData(T->GetToothPolyData());
		vtkSmartPointer<vtkActor>  actorT =
			vtkSmartPointer<vtkActor>::New();
		actorT->SetMapper(mapperT);


		//根据平移旋转量 更新 第j个牙齿和相邻牙齿的polydata
		vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		transform->Translate(before[j][0], before[j][1], before[j][2]);
		transform->RotateWXYZ(part->Tempgbestx[j][k][3], 1, 0, 0);
		transform->RotateWXYZ(part->Tempgbestx[j][k][4], 0, 1, 0);
		transform->RotateWXYZ(part->Tempgbestx[j][k][5], 0, 0, 1);
		transform->Translate(-before[j][0], -before[j][1], -before[j][2]);
		transform->Translate(part->Tempgbestx[j][k][0] - before[j][0], part->Tempgbestx[j][k][1] - before[j][1], part->Tempgbestx[j][k][2] - before[j][2]);//平移	
		transform->Update();
		vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyData = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		transformPolyData->SetInputData(T->GetToothPolyData());
		//transformPolyData->SetInputConnection(mapperT->GetOutputPort());
		transformPolyData->SetTransform(transform);
		transformPolyData->Update();


		if (Tleft)
		{
			vtkSmartPointer<vtkPolyDataMapper> mapperTleft =
				vtkSmartPointer<vtkPolyDataMapper>::New();
			mapperTleft->SetInputData(Tleft->GetToothPolyData());
			vtkSmartPointer<vtkActor>  actorTleft =
				vtkSmartPointer<vtkActor>::New();
			actorTleft->SetMapper(mapperTleft);
			vtkSmartPointer<vtkTransform> transformLeft = vtkSmartPointer<vtkTransform>::New();
			transformLeft->Translate(before[j][0], before[j][1], before[j][2]);
			transformLeft->RotateWXYZ(part->Tempgbestx[tooth1][k][3], 1, 0, 0);
			transformLeft->RotateWXYZ(part->Tempgbestx[tooth1][k][4], 0, 1, 0);
			transformLeft->RotateWXYZ(part->Tempgbestx[tooth1][k][5], 0, 0, 1);
			transformLeft->Translate(-before[j][0], -before[j][1], -before[j][2]);
			transformLeft->Translate(part->Tempgbestx[tooth1][k][0] - before[tooth1][0], part->Tempgbestx[tooth1][k][1] - before[tooth1][1], part->Tempgbestx[tooth1][k][2] - before[tooth1][2]);//平移
			transformLeft->Update();
			vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyDataLeft = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformPolyDataLeft->SetInputData(Tleft->GetToothPolyData());
			//transformPolyDataLeft->SetInputConnection(mapperTleft->GetOutputPort());
			transformPolyDataLeft->SetTransform(transformLeft);
			transformPolyDataLeft->Update();
			if (CollisionOn(transformPolyDataLeft->GetOutput(), transformPolyData->GetOutput(), j, k)) {
				fc = 100000;//发生碰撞
				continue;
			}

		}
		if (Tright)
		{
			vtkSmartPointer<vtkPolyDataMapper> mapperTright =
				vtkSmartPointer<vtkPolyDataMapper>::New();
			mapperTright->SetInputData(Tright->GetToothPolyData());
			vtkSmartPointer<vtkActor>  actorTright =
				vtkSmartPointer<vtkActor>::New();
			actorTright->SetMapper(mapperTright);
			vtkSmartPointer<vtkTransform> transformRight = vtkSmartPointer<vtkTransform>::New();
			transformRight->Translate(before[j][0], before[j][1], before[j][2]);
			transformRight->RotateWXYZ(part->Tempgbestx[tooth2][k][3], 1, 0, 0);
			transformRight->RotateWXYZ(part->Tempgbestx[tooth2][k][4], 0, 1, 0);
			transformRight->RotateWXYZ(part->Tempgbestx[tooth2][k][5], 0, 0, 1);
			transformRight->Translate(-before[j][0], -before[j][1], -before[j][2]);
			transformRight->Translate(part->Tempgbestx[tooth2][k][0] - before[tooth2][0], part->Tempgbestx[tooth2][k][1] - before[tooth2][1], part->Tempgbestx[tooth2][k][2] - before[tooth2][2]);//平移
			transformRight->Update();
			vtkSmartPointer<vtkTransformPolyDataFilter> transformPolyDataRight = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
			transformPolyDataRight->SetInputData(Tright->GetToothPolyData());
			//transformPolyDataRight->SetInputConnection(mapperTright->GetOutputPort());
			transformPolyDataRight->SetTransform(transformRight);
			transformPolyDataRight->Update();
			if (CollisionOn(transformPolyDataRight->GetOutput(), transformPolyData->GetOutput(), j, k)) {
				fc = 1000000; //发生碰撞
				continue;
			}
		}
	}
	return fc;

}

bool GroupPathPlanningPSO::CollisionOn(vtkSmartPointer<vtkPolyData> p1, vtkSmartPointer<vtkPolyData> p2, int j, int k) {//判断该牙齿是否发生碰撞
	//contactMode:碰撞模式
	int contactMode = 0;
	vtkMatrix4x4* matrix0 = vtkMatrix4x4::New();
	vtkMatrix4x4* matrix1 = vtkMatrix4x4::New();
	vtkNew<vtkTransform> transform0;
	vtkNew<vtkTransform> transform1;

	//设置输入数据
	vtkNew<vtkCollisionDetectionFilter> collide;
	collide->DebugOn();///////
	//输入第一个polyData数据
	collide->SetInputData(0, p1);
	collide->SetMatrix(0, matrix0);////////
	//对第一个输入数据进行空间几何变换
	//collide->SetTransform(0, transform0);
	//输入第二个polyData数据
	collide->SetInputData(1, p2);
	//对第二个数据进行矩阵变换
	collide->SetMatrix(1, matrix1);
	//设置并获取OBB公差
	collide->SetBoxTolerance(0.0);
	//设置并获取单元格公差
	collide->SetCellTolerance(0.0);
	//设置并获取每个OBB中的单元格数
	collide->SetNumberOfCellsPerNode(2);



	//根据contactMode选择不同的碰撞模式
	if (contactMode == 0)
	{
		collide->SetCollisionModeToAllContacts();
	}
	else if (contactMode == 1)
	{
		collide->SetCollisionModeToFirstContact();
	}
	else
	{
		collide->SetCollisionModeToHalfContacts();
	}
	collide->GenerateScalarsOn();
	collide->Update();

	//不发生碰撞collide->GetNumberOfContacts() = 0
	//设置阈值400
	int obb = collide->GetNumberOfContacts();
	if (obb > 750) {
		cout << "牙齿" << j << "阶段" << k << "发生碰撞数collide->GetNumberOfContacts:" << collide->GetNumberOfContacts() << endl;
		return true;//发生碰撞
	}
	else {
		return false;//不发生碰撞
	}
	//return false;//不发生碰撞
}


double GroupPathPlanningPSO::Move(particle* part, int j) {//单个牙齿移动量计算
	double fm = 0;
	for (int k = 2; k <= n; k++){
		fm = fm + sqrt(pow(part->x[j][k][0] - part->x[j][k - 1][0], 2) + pow(part->x[j][k][1] - part->x[j][k - 1][1], 2) + pow(part->x[j][k][2] - part->x[j][k - 1][2], 2));
		//if (k == 1) { //第一个路径点 - 初始位置  //x[j][0][d] = after[j][0][d]
		//	fm = fm + sqrt(pow(part->x[j][k][0] - before[j][0], 2) + pow(part->x[j][k][1] - before[j][1], 2) + pow(part->x[j][k][2] - before[j][2], 2));
		//}
		//else {
		//	fm = fm + sqrt(pow(part->x[j][k][0] - part->x[j][k - 1][1], 2) + pow(part->x[j][k][1] - part->x[j][k - 1][0], 2) + pow(part->x[j][k][2] - part->x[j][k - 1][2], 2));
		//}
	}
	return fm;
}
double GroupPathPlanningPSO::Rotate(particle* part, int j) {//单个牙齿旋转量计算
	double fr = 0;
	for (int k = 2; k <= n; k++) {
		fr = fr + abs(part->x[j][k][3] - part->x[j][k - 1][3]) + abs(part->x[j][k][4] - part->x[j][k - 1][4]) + abs(part->x[j][k][5] - part->x[j][k - 1][5]);
		//if (k == 1) {//第一个路径点 - 初始位置  //x[j][0][d] = after[j][0][d]
		//	fr = fr + abs(part->x[j][k][3]) + abs(part->x[j][k][4]) + abs(part->x[j][k][5]);
		//}
		//else {
		//	fr = fr + abs(part->x[j][k][3] - part->x[j][k - 1][3] ) + abs(part->x[j][k][4] - part->x[j][k - 1][4] ) + abs(part->x[j][k][5] - part->x[j][k - 1][5] );
		//}
	}
	return fr;
}
bool GroupPathPlanningPSO::Constraint(particle* part, int j) {//约束条件--移动量旋转量
	// 文档要求：单步平移≤0.3mm，单步旋转≤2°
	for (int k = 2; k <= n; k++) {
		// 平移量（单位：mm）
		double trans = sqrt(
			pow(part->x[j][k][0] - part->x[j][k - 1][0], 2) +
			pow(part->x[j][k][1] - part->x[j][k - 1][1], 2) +
			pow(part->x[j][k][2] - part->x[j][k - 1][2], 2)
		);
		// 旋转量（转换为角度，单位：°）
		double rot =
			abs((part->x[j][k][3] / PI * 180) - (part->x[j][k - 1][3] / PI * 180)) +
			abs((part->x[j][k][4] / PI * 180) - (part->x[j][k - 1][4] / PI * 180)) +
			abs((part->x[j][k][5] / PI * 180) - (part->x[j][k - 1][5] / PI * 180));

		// 不符合文档约束则返回false
		if (trans > 3 || rot > 7) { // 修正为0.3mm和2°
			return false;
		}
	}
	return true; // 符合约束
}


double GroupPathPlanningPSO::Fitness(particle* part) {
	double w1 = 0.6; // 提高平移权重
	double w2 = 0.3;
	double w3 = 0.1; // 碰撞惩罚权重
	double f1 = 0, f2 = 0, f3 = 0;

	for (int j = 2; j < 16; j++) { // 上颌牙齿
		Tooth* tooth = m_OrthoData->GetTooth((ToothId)j);
		if (!tooth) continue;

		f1 += Move(part, j); // 平移量
		f2 += Rotate(part, j); // 旋转量

		// 关键修复：碰撞惩罚与平移量关联（碰撞时放大平移的权重）
		if (!Constraint(part, j) || OBB(part, j) > 0) {
			f3 += 10000 * (1 + f1); // 平移越大，碰撞惩罚越重
		}
	}

	// 适应度 = 平移（主） + 旋转（次） + 碰撞惩罚（与平移关联）
	double fitness = w1 * f1 + w2 * f2 + w3 * f3;
	return fitness;
}
double GroupPathPlanningPSO::GbestFitness(tempgbest* part) {//适应度函数=目标函数
	double w1 = 0.4118;//权重1 - 单个牙齿总移动量
	double w2 = 0.3431;//权重2 - 单个牙齿总旋转量
	double w3 = 0.2451;//权重3 - 碰撞检测
	double f1 = 0;//约束条件 - 单个牙齿总移动量
	double f2 = 0;//约束条件 - 单个牙齿总旋转量
	double f3 = 1000000;//罚函数约束条件 - 碰撞检测
	double f4 = 0;//罚函数约束条件 - 平移旋转
	double move = 0;//单个牙齿单个阶段的移动量
	double rotate = 0;//单个牙齿单个阶段的旋转量

	double cons = 0;//判断移动量和旋转量是否符合约束条件得出的判断值
	double Cons = 1;//根据约束条件得出的Cons值来选择是否改变适应度函数值使其适应度下降
	double fitness;//适应度函数值
	for (int a = 0; a < 1; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
			for (int d = 0; d < 6; d++) {
				part->Tempgbestx[j][25][d] = gbestx[j][n][d];
			}
			//平移量计算
			double fm = 0;
			for (int k = 2; k <= n; k++) {
				fm = fm + sqrt(pow(part->Tempgbestx[j][k][0] - part->Tempgbestx[j][k - 1][0], 2) + pow(part->Tempgbestx[j][k][1] - part->Tempgbestx[j][k - 1][1], 2) + pow(part->Tempgbestx[j][k][2] - part->Tempgbestx[j][k - 1][2], 2));
			}
			f1 = f1 + fm;//总平移量
		    //旋转量计算
			double fr = 0;
			for (int k = 2; k <= n; k++) {
				fr = fr + abs(part->Tempgbestx[j][k][3] - part->Tempgbestx[j][k - 1][3]) + abs(part->Tempgbestx[j][k][4] - part->Tempgbestx[j][k - 1][4]) + abs(part->Tempgbestx[j][k][5] - part->Tempgbestx[j][k - 1][5]);
			}
			f2 = f2 + fr;//总旋转量

			//平移和旋转约束条件
			for (int k = 2; k <= n; k++) {
				double m = sqrt(pow(part->Tempgbestx[j][k][0] - part->Tempgbestx[j][k - 1][0], 2) + pow(part->Tempgbestx[j][k][1] - part->Tempgbestx[j][k - 1][1], 2) + pow(part->Tempgbestx[j][k][2] - part->Tempgbestx[j][k - 1][2], 2));
				double r = abs((part->Tempgbestx[j][k][3] / PI * 180) - (part->Tempgbestx[j][k - 1][3] / PI * 180)) + abs((part->Tempgbestx[j][k][4] / PI * 180) - (part->Tempgbestx[j][k - 1][4] / PI * 180)) + abs((part->Tempgbestx[j][k][5] / PI * 180) - (part->Tempgbestx[j][k - 1][5] / PI * 180));
				if (m > 3 || r > 7)
					f4 = 1000000;//不符合约束条件
			}
			if(f4 == 0)    f3 = GbestpOBB(part, j);
		}
	}
	fitness = w1 * f1 + w2 * f2 + w3 * f3 + f4;//适应度函数值

	return fitness;
}

double GroupPathPlanningPSO::GbestMove(tempgbest* part) {//适应度函数=目标函数
	double f1 = 0;//约束条件 - 单个牙齿总移动量
	for (int a = 0; a < 1; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
			for (int d = 0; d < 6; d++) {
				part->Tempgbestx[j][25][d] = gbestx[j][n][d];
			}
			//平移量计算
			double fm = 0;
			for (int k = 2; k <= n; k++) {
				fm = fm + sqrt(pow(part->Tempgbestx[j][k][0] - part->Tempgbestx[j][k - 1][0], 2) + pow(part->Tempgbestx[j][k][1] - part->Tempgbestx[j][k - 1][1], 2) + pow(part->Tempgbestx[j][k][2] - part->Tempgbestx[j][k - 1][2], 2));
			}
			f1 = f1 + fm;//总平移量
			
		}
	}
	return f1;
}

double GroupPathPlanningPSO::GbestRotate(tempgbest* part) {//适应度函数=目标函数
	double f2 = 0;//约束条件 - 单个牙齿总旋转量
	for (int a = 0; a < 1; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
			for (int d = 0; d < 6; d++) {
				part->Tempgbestx[j][25][d] = gbestx[j][n][d];
			}
			//旋转量计算
			double fr = 0;
			for (int k = 2; k <= n; k++) {
				fr = fr + abs(part->Tempgbestx[j][k][3] - part->Tempgbestx[j][k - 1][3]) + abs(part->Tempgbestx[j][k][4] - part->Tempgbestx[j][k - 1][4]) + abs(part->Tempgbestx[j][k][5] - part->Tempgbestx[j][k - 1][5]);
			}
			f2 = f2 + fr;//总旋转量
		}
	}
	return f2;
}


double GroupPathPlanningPSO::GbestFit(tempgbest* part) {//适应度函数=目标函数
	double w1 = 0.518;//权重1 - 单个牙齿总移动量
	double w2 = 0.482;//权重2 - 单个牙齿总旋转量
	double f1 = 0;//约束条件 - 单个牙齿总移动量
	double f2 = 0;//约束条件 - 单个牙齿总旋转量

	double fitness1 = 0;//适应度函数值
	for (int a = 0; a < 1; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
			for (int d = 0; d < 6; d++) {
				part->Tempgbestx[j][25][d] = gbestx[j][n][d];
			}
			//平移量计算
			double fm = 0;
			for (int k = 2; k <= n; k++) {
				fm = fm + sqrt(pow(part->Tempgbestx[j][k][0] - part->Tempgbestx[j][k - 1][0], 2) + pow(part->Tempgbestx[j][k][1] - part->Tempgbestx[j][k - 1][1], 2) + pow(part->Tempgbestx[j][k][2] - part->Tempgbestx[j][k - 1][2], 2));
			}
			f1 = f1 + fm;//总平移量
			//旋转量计算
			double fr = 0;
			for (int k = 2; k <= n; k++) {
				fr = fr + abs(part->Tempgbestx[j][k][3] - part->Tempgbestx[j][k - 1][3]) + abs(part->Tempgbestx[j][k][4] - part->Tempgbestx[j][k - 1][4]) + abs(part->Tempgbestx[j][k][5] - part->Tempgbestx[j][k - 1][5]);
			}
			f2 = f2 + fr;//总旋转量

		}
	}
	fitness1 = w1 * f1 + w2 * f2 ;//适应度函数值

	return fitness1;
}

double GroupPathPlanningPSO::GbestMove_upper(tempgbest* part) {//适应度函数=目标函数
	double f1 = 0;//
	for (int j = 2; j < 16; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
		for (int d = 0; d < 6; d++) {
			part->Tempgbestx[j][25][d] = after[j][d];
			part->Tempgbestx[j][1][d] = before[j][d];
		}
		//平移量计算
		double fm = 0;
		for (int k = 2; k <= n; k++) {
			fm = fm + sqrt(pow(part->Tempgbestx[j][k][0] - part->Tempgbestx[j][k - 1][0], 2) + pow(part->Tempgbestx[j][k][1] - part->Tempgbestx[j][k - 1][1], 2) + pow(part->Tempgbestx[j][k][2] - part->Tempgbestx[j][k - 1][2], 2));
			//if (k == 1) { //第一个路径点 - 初始位置  //x[j][0][d] = after[j][0][d]
			//	fm = fm + sqrt(pow(part->Tempgbestx[j][k][0] - before[j][0], 2) + pow(part->Tempgbestx[j][k][1] - before[j][1], 2) + pow(part->Tempgbestx[j][k][2] - before[j][2], 2));
			//}
			//else {
			//	fm = fm + sqrt(pow(part->Tempgbestx[j][k][0] - part->Tempgbestx[j][k - 1][0], 2) + pow(part->Tempgbestx[j][k][1] - part->Tempgbestx[j][k - 1][1], 2) + pow(part->Tempgbestx[j][k][2] - part->Tempgbestx[j][k - 1][2], 2));
			//}
		}
		cout << "牙齿" << j << "平移量:" << fm << endl;
		f1 = f1 + fm;//总平移量
	}
	return f1;
}
double GroupPathPlanningPSO::GbestMove_upper1(tempgbest* part) {//适应度函数=目标函数
	double f1 = 0;//约束条件 - 单个牙齿总移动量
	for (int a = 0; a < 1; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
			for (int d = 0; d < 6; d++) {
				part->Tempgbestx[j][25][d] = gbestx[j][n][d];
			}
			//平移量计算
			double fm = 0;
			for (int k = 2; k <= n; k++) {
				fm = fm + sqrt(pow(part->Tempgbestx[j][k][0] - part->Tempgbestx[j][k - 1][0], 2));
			}
			f1 = f1 + fm;//总平移量

		}
	}
	return f1;
}
double GroupPathPlanningPSO::GbestMove_upper2(tempgbest* part) {//适应度函数=目标函数
	double f1 = 0;//约束条件 - 单个牙齿总移动量
	for (int a = 0; a < 1; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
			for (int d = 0; d < 6; d++) {
				part->Tempgbestx[j][25][d] = gbestx[j][n][d];
			}
			//平移量计算
			double fm = 0;
			for (int k = 2; k <= n; k++) {
				fm = fm + sqrt(pow(part->Tempgbestx[j][k][1] - part->Tempgbestx[j][k - 1][1], 2));
			}
			f1 = f1 + fm;//总平移量

		}
	}
	return f1;
}
double GroupPathPlanningPSO::GbestMove_upper3(tempgbest* part) {//适应度函数=目标函数
	double f1 = 0;//约束条件 - 单个牙齿总移动量
	for (int a = 0; a < 1; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
			for (int d = 0; d < 6; d++) {
				part->Tempgbestx[j][25][d] = gbestx[j][n][d];
			}
			//平移量计算
			double fm = 0;
			for (int k = 2; k <= n; k++) {
				fm = fm + sqrt(pow(part->Tempgbestx[j][k][2] - part->Tempgbestx[j][k - 1][2], 2));
			}
			f1 = f1 + fm;//总平移量

		}
	}
	return f1;
}

double GroupPathPlanningPSO::GbestRotate_upper(tempgbest* part) {//适应度函数=目标函数
	double f2 = 0;//约束条件 - 单个牙齿总旋转量
	for (int a = 0; a < 1; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
			for (int d = 0; d < 6; d++) {
				part->Tempgbestx[j][25][d] = after[j][d];
				part->Tempgbestx[j][1][d] = before[j][d];
			}
			//旋转量计算
			double fr = 0;
			for (int k = 2; k <= n; k++) {
				fr = fr + abs(part->Tempgbestx[j][k][3] - part->Tempgbestx[j][k - 1][3]) + abs(part->Tempgbestx[j][k][4] - part->Tempgbestx[j][k - 1][4]) + abs(part->Tempgbestx[j][k][5] - part->Tempgbestx[j][k - 1][5]);
				//if (k == 1) {//第一个路径点 - 初始位置  //x[j][0][d] = after[j][0][d]
				//	fr = fr + abs(part->Tempgbestx[j][k][3]) + abs(part->Tempgbestx[j][k][4]) + abs(part->Tempgbestx[j][k][5]);
				//}
				//else {
				//	fr = fr + abs(part->Tempgbestx[j][k][3] - part->Tempgbestx[j][k - 1][3]) + abs(part->Tempgbestx[j][k][4] - part->Tempgbestx[j][k - 1][4]) + abs(part->Tempgbestx[j][k][5] - part->Tempgbestx[j][k - 1][5]);
				//}
			}
			cout << "牙齿" << j << "旋转量:" << fr << endl;
			f2 = f2 + fr;//总旋转量

		}
	}
	return f2;
}
double GroupPathPlanningPSO::GbestRotate_upper4(tempgbest* part) {//适应度函数=目标函数
	double f2 = 0;//约束条件 - 单个牙齿总旋转量
	for (int a = 0; a < 1; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
			for (int d = 0; d < 6; d++) {
				part->Tempgbestx[j][25][d] = gbestx[j][n][d];
			}
			//旋转量计算
			double fr = 0;
			for (int k = 2; k <= n; k++) {
				fr = fr + abs(part->Tempgbestx[j][k][3] - part->Tempgbestx[j][k - 1][3]);
			}
				
			f2 = f2 + fr;//总旋转量
		}
	}
	return f2;
}

double GroupPathPlanningPSO::GbestRotate_upper5(tempgbest* part) {//适应度函数=目标函数
	double f2 = 0;//约束条件 - 单个牙齿总旋转量
	for (int a = 0; a < 1; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
			for (int d = 0; d < 6; d++) {
				part->Tempgbestx[j][25][d] = gbestx[j][n][d];
			}
			//旋转量计算
			double fr = 0;
			for (int k = 2; k <= n; k++) {
				fr = fr + abs(part->Tempgbestx[j][k][4] - part->Tempgbestx[j][k - 1][4]);
			}
			f2 = f2 + fr;//总旋转量
		}
	}
	return f2;
}

double GroupPathPlanningPSO::GbestRotate_upper6(tempgbest* part) {//适应度函数=目标函数
	double f2 = 0;//约束条件 - 单个牙齿总旋转量
	for (int a = 0; a < 1; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
			for (int d = 0; d < 6; d++) {
				part->Tempgbestx[j][25][d] = gbestx[j][25][d];
			}
			//旋转量计算
			double fr = 0;
			for (int k = 2; k <= n; k++) {
				fr = fr + abs(part->Tempgbestx[j][k][5] - part->Tempgbestx[j][k - 1][5]);
			}
				
			f2 = f2 + fr;//总旋转量
		}
	}
	return f2;
}

double GroupPathPlanningPSO::GbestFit_upper(tempgbest* part) {//适应度函数=目标函数
	double w1 = 0.518;//权重1 - 单个牙齿总移动量
	double w2 = 0.482;//权重2 - 单个牙齿总旋转量
	double f1 = 0;//约束条件 - 单个牙齿总移动量
	double f2 = 0;//约束条件 - 单个牙齿总旋转量

	double fitness1 = 0;//适应度函数值
	for (int j = 2; j < 16; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
		for (int d = 0; d < 6; d++) {
			part->Tempgbestx[j][25][d] = after[j][d];
			part->Tempgbestx[j][1][d] = before[j][d];
		}
		//平移量计算
		double fm = 0;
		double fr = 0;
		for (int k = 2; k <= n; k++) {
			fm = fm + sqrt(pow(part->Tempgbestx[j][k][0] - part->Tempgbestx[j][k - 1][0], 2) + pow(part->Tempgbestx[j][k][1] - part->Tempgbestx[j][k - 1][1], 2) + pow(part->Tempgbestx[j][k][2] - part->Tempgbestx[j][k - 1][2], 2));
			fr = fr + abs(part->Tempgbestx[j][k][3] - part->Tempgbestx[j][k - 1][3]) + abs(part->Tempgbestx[j][k][4] - part->Tempgbestx[j][k - 1][4]) + abs(part->Tempgbestx[j][k][5] - part->Tempgbestx[j][k - 1][5]);
		}
		f1 = f1 + fm;//总平移量
		f2 = f2 + fr;//
	}
	fitness1 = w1 * f1 + w2 * f2;//适应度函数值
	cout << "牙齿上颌适应值:" << fitness1 << endl;
	return fitness1;
}

double GroupPathPlanningPSO::GbestMove_lower(tempgbest* part) {//适应度函数=目标函数
	double f1 = 0;//约束条件 - 单个牙齿总移动量
	for (int j = 18; j < 32; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
		for (int d = 0; d < 6; d++) {
			part->Tempgbestx[j][25][d] = after[j][d];
			part->Tempgbestx[j][1][d] = before[j][d];
		}
		//平移量计算
		double fm = 0;
		for (int k = 2; k <= n; k++) {
			fm = fm + sqrt(pow(part->Tempgbestx[j][k][0] - part->Tempgbestx[j][k - 1][0], 2) + pow(part->Tempgbestx[j][k][1] - part->Tempgbestx[j][k - 1][1], 2) + pow(part->Tempgbestx[j][k][2] - part->Tempgbestx[j][k - 1][2], 2));
		}
		f1 = f1 + fm;//总平移量
	}
	return f1;
}

double GroupPathPlanningPSO::GbestRotate_lower(tempgbest* part) {//适应度函数=目标函数
	double f2 = 0;//约束条件 - 单个牙齿总旋转量
	for (int j = 18; j < 32; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
		for (int d = 0; d < 6; d++) {
			part->Tempgbestx[j][25][d] = gbestx[j][n][d];
		}
		//旋转量计算
		double fr = 0;
		for (int k = 2; k <= n; k++) {
			fr = fr + abs(part->Tempgbestx[j][k][3] - part->Tempgbestx[j][k - 1][3]) + abs(part->Tempgbestx[j][k][4] - part->Tempgbestx[j][k - 1][4]) + abs(part->Tempgbestx[j][k][5] - part->Tempgbestx[j][k - 1][5]);
		}

		f2 = f2 + fr;//总旋转量
	}
	return f2;
}


double GroupPathPlanningPSO::GbestFit_lower(tempgbest* part) {//适应度函数=目标函数
	double w1 = 0.518;//权重1 - 单个牙齿总移动量
	double w2 = 0.482;//权重2 - 单个牙齿总旋转量
	double f1 = 0;//约束条件 - 单个牙齿总移动量
	double f2 = 0;//约束条件 - 单个牙齿总旋转量

	double fitness1 = 0;//适应度函数值
	for (int j = 18; j < 32; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
			// n = 7是理想位置点需单独考虑中间路径点6-7的计算
		for (int d = 0; d < 6; d++) {
			part->Tempgbestx[j][25][d] = gbestx[j][n][d];
		}
		//平移量计算
		double fm = 0;
		for (int k = 1; k <= n; k++) {
			fm = fm + sqrt(pow(part->Tempgbestx[j][k][0] - part->Tempgbestx[j][k - 1][0], 2) + pow(part->Tempgbestx[j][k][1] - part->Tempgbestx[j][k - 1][1], 2) + pow(part->Tempgbestx[j][k][2] - part->Tempgbestx[j][k - 1][2], 2));
		}

		f1 = f1 + fm;//总平移量
		//旋转量计算
		double fr = 0;
		for (int k = 1; k <= n; k++) {
			fr = fr + abs(part->Tempgbestx[j][k][3] - part->Tempgbestx[j][k - 1][3]) + abs(part->Tempgbestx[j][k][4] - part->Tempgbestx[j][k - 1][4]) + abs(part->Tempgbestx[j][k][5] - part->Tempgbestx[j][k - 1][5]);
		}

		f2 = f2 + fr;//总旋转量

	}
	fitness1 = w1 * f1 + w2 * f2;//适应度函数值
	cout << "下颌平移量:" << f1 << endl;
	cout << "下颌旋转量:" << f2 << endl;
	cout << "下颌适应度值:" << fitness1 << endl;

	return fitness1;
}

double GroupPathPlanningPSO::Pbest(particle* part) {//适应度函数=目标函数
	double w1 = 0.518; // 平移权重
	double w2 = 0.482; // 旋转权重
	double totalTrans = 0; // 上下颌总平移量（上颌+下颌）
	double totalRot = 0;   // 上下颌总旋转量（上颌+下颌）

	// 文档要求：需同时处理上颌和下颌案例
	// a=0：上颌（牙齿ID 2-15）；a=1：下颌（牙齿ID 18-31）
	for (int a = 0; a < 2; a++) {
		int startJ, endJ;
		if (a == 0) {
			startJ = 2;  // 上颌起始ID
			endJ = 16;   // 上颌结束ID（2-15）
		}
		else {
			startJ = 18; // 下颌起始ID
			endJ = 32;   // 下颌结束ID（18-31）
		}

		// 累加当前颌的平移和旋转量
		for (int j = startJ; j < endJ; j++) {
			// 强制最终位置与全局最优一致
			for (int d = 0; d < 6; d++) {
				part->x[j][25][d] = gbestx[j][25][d];
			}

			// 计算当前牙齿的总平移量（累加各阶段平移）
			double toothTrans = 0;
			for (int k = 2; k <= n; k++) {
				toothTrans += sqrt(
					pow(part->x[j][k][0] - part->x[j][k - 1][0], 2) +
					pow(part->x[j][k][1] - part->x[j][k - 1][1], 2) +
					pow(part->x[j][k][2] - part->x[j][k - 1][2], 2)
				);
			}
			totalTrans += toothTrans;

			// 计算当前牙齿的总旋转量（累加各阶段旋转）
			double toothRot = 0;
			for (int k = 2; k <= n; k++) {
				toothRot +=
					abs(part->x[j][k][3] - part->x[j][k - 1][3]) +
					abs(part->x[j][k][4] - part->x[j][k - 1][4]) +
					abs(part->x[j][k][5] - part->x[j][k - 1][5]);
			}
			totalRot += toothRot;
		}
	}

	// 适应度函数=平移权重×总平移 + 旋转权重×总旋转（符合文档目标函数设计）
	double fitness = w1 * totalTrans + w2 * totalRot;

	// 输出修正为真实上下颌总量（上颌+下颌）
	cout << "上下颌平移量:" << totalTrans << endl;
	cout << "上下颌旋转量:" << totalRot << endl;
	cout << "上下颌适应度值:" << fitness << endl;
	return fitness;
}
double GroupPathPlanningPSO::Pbest_upper(particle* part) {
	double w1 = 0.6; // 提高平移权重，优先减少平移
	double w2 = 0.4;
	double f1 = 0; // 上颌总平移量
	double f2 = 0; // 上颌总旋转量

	for (int j = 2; j < 16; j++) { // 上颌牙齿（2-15）
		// 关键修复：跳过无效牙齿（未加载或不存在的牙齿）
		Tooth* tooth = m_OrthoData->GetTooth((ToothId)j);
		if (!tooth) {
			cout << "跳过无效牙齿：" << j << endl;
			continue; // 不计算无效牙齿的平移/旋转量
		}

		// 强制最终位置与全局最优一致
		for (int d = 0; d < 6; d++) {
			part->x[j][25][d] = gbestx[j][25][d];
		}

		// 平移量计算（仅有效牙齿）
		double fm = 0;
		for (int k = 2; k <= n; k++) {
			// 计算当前步的三维平移距离
			double dx = part->x[j][k][0] - part->x[j][k - 1][0];
			double dy = part->x[j][k][1] - part->x[j][k - 1][1];
			double dz = part->x[j][k][2] - part->x[j][k - 1][2];
			fm += sqrt(dx * dx + dy * dy + dz * dz); // 累加三维平移距离
		}
		f1 += fm;

		// 旋转量计算（仅有效牙齿）
		double fr = 0;
		for (int k = 2; k <= n; k++) {
			fr += abs(part->x[j][k][3] - part->x[j][k - 1][3]) +
				abs(part->x[j][k][4] - part->x[j][k - 1][4]) +
				abs(part->x[j][k][5] - part->x[j][k - 1][5]);
		}
		f2 += fr;
	}

	double fitness = w1 * f1 + w2 * f2;
	cout << "上颌平移量:" << f1 << endl;
	cout << "上颌旋转量:" << f2 << endl;
	cout << "上颌适应度值:" << fitness << endl;
	return fitness;
}
double GroupPathPlanningPSO::Pbest_lower(particle* part) {//适应度函数=目标函数
	//double w1 = 0.4118;//权重1 - 单个牙齿总移动量
	//double w2 = 0.3431;//权重2 - 单个牙齿总旋转量
	//double w3 = 0.2451;//权重3 - 碰撞检测
	double w1 = 0.518;//权重1 - 单个牙齿总移动量
	double w2 = 0.482;//权重2 - 单个牙齿总旋转量
	double f1 = 0;//约束条件 - 单个牙齿总移动量
	double f2 = 0;//约束条件 - 单个牙齿总旋转量
	double f3 = 10000000;//约束条件 - 碰撞检测

	double fitness = 0;//适应度函数值
	for (int j = 18; j < 32; j++) {//循环上颌/下颌每个牙齿(m = 牙齿id)
		for (int d = 0; d < 6; d++) {
			part->x[j][25][d] = gbestx[j][25][d];
		}
		//平移量计算
		double fm = 0;
		for (int k = 2; k <= n; k++) {
			fm = fm + sqrt(pow(part->x[j][k][0] - part->x[j][k - 1][0], 2) + pow(part->x[j][k][1] - part->x[j][k - 1][1], 2) + pow(part->x[j][k][2] - part->x[j][k - 1][2], 2));
		}
		f1 = f1 + fm;//总平移量
		//旋转量计算
		double fr = 0;
		for (int k = 2; k <= n; k++) {
			fr = fr + abs(part->x[j][k][3] - part->x[j][k - 1][3]) + abs(part->x[j][k][4] - part->x[j][k - 1][4]) + abs(part->x[j][k][5] - part->x[j][k - 1][5]);
		}
		f2 = f2 + fr;//总旋转量
	}
	fitness = w1 * f1 + w2 * f2;//适应度函数值

	cout << "下颌平移量:" << f1 << endl;
	cout << "下颌旋转量:" << f2 << endl;
	cout << "下颌适应度值:" << fitness << endl;
	return fitness;

}

void GroupPathPlanningPSO::Fitness_jizhun() {
	clock_t start_timeSphere = clock();
	//Test_PSO();
	Test_Ours();
	//Test_NSMPSO();
	//Test_PSOSA();
	//Test_CMPSO();
	/*Test_MOPSO();
	Test_MDC_DPSO();
	Test_MCMOPSO();*/
	clock_t end_timeSphere = clock();
	cout << "test_Sphere耗时"<<(double)(end_timeSphere - start_timeSphere) / CLOCKS_PER_SEC << "秒" << endl;
	cout << "-------------------------------------------------------------------------------------------------" <<endl;
	//OnGA();
	//test_f* test_Michalewicz = &t;
	//clock_t start_timeMichalewicz = clock();
	//Test_jizhun(test_Michalewicz);
	//clock_t end_timeMichalewicz = clock();
	//cout << "test_Michalewicz耗时" << (double)(end_timeMichalewicz - start_timeMichalewicz) / CLOCKS_PER_SEC << "秒" << endl;
	//cout << "-------------------------------------------------------------------------------------------------" << endl;

	//test_f* test_Rosenbrock = &t;
	//clock_t start_timeRosenbrock = clock();
	//Test_jizhun(test_Rosenbrock);
	//clock_t end_timeRosenbrock = clock();
	//cout << "test_Rosenbrock耗时" << (double)(end_timeRosenbrock - start_timeRosenbrock) / CLOCKS_PER_SEC << "秒" << endl;
	//cout << "-------------------------------------------------------------------------------------------------" << endl;

	//test_f* test_Step = &t;
	//clock_t start_timeStep = clock();
	//Test_jizhun(test_Step);
	//clock_t end_timeStep = clock();
	//cout << "test_Step耗时" << (double)(end_timeStep - start_timeStep) / CLOCKS_PER_SEC << "秒" << endl;
	//cout << "-------------------------------------------------------------------------------------------------" << endl;

	//test_f* test_Noise = &t;
	//clock_t start_timeNoise = clock();
	//Test_jizhun(test_Noise);
	//clock_t end_timeNoise = clock();
	//cout << "test_Noise耗时" << (double)(end_timeNoise - start_timeNoise) / CLOCKS_PER_SEC << "秒" << endl;
	//cout << "-------------------------------------------------------------------------------------------------" << endl;

	//test_f* test_Zakharov = &t;
	//clock_t start_timeZakharov = clock();
	//Test_jizhun(test_Zakharov);
	//clock_t end_timeZakharov = clock();
	//cout << "test_Zakharov耗时" << (double)(end_timeZakharov - start_timeZakharov) / CLOCKS_PER_SEC << "秒" << endl;
	//cout << "-------------------------------------------------------------------------------------------------" << endl;


}


double GroupPathPlanningPSO::Test_jizhun(test_f* part) {
	double value_jizhun = Sphere(part);
	//double value_jizhun = Quadric(part);
	//double value_jizhun = Rosenbrock(part);
	//double value_jizhun = Step(part);
	//double value_jizhun = Noise(part);
	//double value_jizhun = Rastrigin(part);
	//double value_jizhun = Zakharov(part);
	//double value_jizhun = Ackle(part);
	//double value_jizhun = Griewank(part);
	//double value_jizhun = Booth(part);
	//double value_jizhun = Schaffer(part);
	return value_jizhun;
}

//基准函数测试算法性能
double  GroupPathPlanningPSO::Sphere(test_f* part){
	double test_fit = 0;
	for (int i = 0; i < 20; i++) {
		test_fit = test_fit + pow(part->test_x[i], 2);
	}
	return test_fit;
}
double GroupPathPlanningPSO::Sphere2(test222* part) {
	double test_fit = 0;
	for (int i = 0; i < 20; i++) {
		test_fit = test_fit + pow(part->test_x[i], 2);
	}
	return test_fit;
}
double  GroupPathPlanningPSO::Quadric(test_f* part){
	double test_fit = 0;
	for (int i = 0; i < 20; i++) {
		double test_fit2 = 0;
		for (int j = 0; j < 20; j++) {
			test_fit2 = test_fit2 + part->test_x[j];
		}
		test_fit = test_fit + test_fit2 * test_fit2;
	}
	return test_fit;
}
double  GroupPathPlanningPSO::Rosenbrock(test_f* part){
	double test_fit = 0;
	for (int i = 0; i < 19; i++) {
		test_fit = test_fit + 100 * pow((part->test_x[i+1] - part->test_x[i] * part->test_x[i]), 2) + pow((part->test_x[i] - 1), 2);
	}
	return test_fit;
}
double  GroupPathPlanningPSO::Step(test_f* part){
	double test_fit = 0;
	for (int i = 0; i < 20; i++) {
		test_fit = test_fit + pow(abs(part->test_x[i] + 0.5), 2);
	}
	return test_fit;
}
double  GroupPathPlanningPSO::Noise(test_f* part){
	double test_fit = 0;
	for (int i = 0; i < 20; i++) {
		test_fit = test_fit + i *  pow(part->test_x[i], 4);
	}
	double fit = test_fit + (rand() % (N + 1) / (float)(N + 1));
	return fit;
}
double  GroupPathPlanningPSO::Zakharov(test_f* part){
	double test_fit1 = 0;
	double test_fit2 = 0;
	double test_fit3 = 0;
	double test_fit = 0;
	for (int i = 0; i < 20; i++) {
		test_fit1 = test_fit1 + pow(part->test_x[i], 2);
		test_fit2 = test_fit2 + i * pow(part->test_x[i], 2);
		test_fit3 = test_fit3 + 0.5 * i * pow(part->test_x[i], 2);
	}
	test_fit2 = pow(test_fit2, 2);
	test_fit3 = pow(test_fit3, 4);
	test_fit = test_fit1 + test_fit2 + test_fit3;
	return test_fit;
}
double GroupPathPlanningPSO::Booth(test_f* part) {
	double test_fit = 0;
	test_fit = pow(part->test_x[1]+ part->test_x2[1] - 7, 2) + pow(2 * part->test_x[1] + part->test_x2[1] - 5, 2);
	return test_fit;
}

double GroupPathPlanningPSO::Schaffer(test_f* part) {
	double test_fit = 0;
	test_fit = 0.5 + ((pow((sin(sqrt(pow(part->test_x[1], 2) + pow(part->test_x[2], 2)))), 2)) - 0.5) / (pow((1 + 0.001 * (pow(part->test_x[1], 2) + pow(part->test_x[2], 2))), 2)); 
	return test_fit;
}


double  GroupPathPlanningPSO::Rastrigin(test_f* part) {
	double test_fit = 0;
	for (int i = 0; i < 20; i++) {
		test_fit = test_fit + pow(part->test_x[i], 2) - 10 * cos(2 * PI * part->test_x[i]) + 10;
	}
	return test_fit;
}

double GroupPathPlanningPSO::Ackle(test_f* part) {
	double test_fit1 = 0;
	double test_fit2 = 0;
	double test_fit = 0;
	for (int i = 0; i < 20; i++) {
		test_fit1 = test_fit1 + pow(part->test_x[i], 2);
		test_fit2 = test_fit2 + cos(2 * PI * part->test_x[i]);
	}
	test_fit = - 20 * log(- 0.2 * sqrt( test_fit1 / 20)) - log(test_fit2 / 20) + 20 + exp(1);
	return test_fit;

	/*double test_fit = 0;
	test_fit = pow(part->test_x[i], 2) + pow(part->test_x[i], 2);
	for (int i = 0; i < 20; i++) {
		test_fit = test_fit - part->test_x[i] * sin(sqrt(abs(part->test_x[i])));
	}
	return test_fit;*/
}

double GroupPathPlanningPSO::Griewank(test_f* part) {
	double test_fit1 = 0;
	double test_fit2 = 0;
	double test_fit = 0;
	for (int i = 0; i < 20; i++) {
		test_fit1 = test_fit1 + pow(part->test_x[i], 2);
		test_fit2 = test_fit2 * cos(part->test_x[i] / sqrt(i));
	}
	test_fit = test_fit1 / 4000 - test_fit2 + 1;
	return test_fit;
}

double  GroupPathPlanningPSO::Alpine(test_f* part) {
	double test_fit = 0;
	for (int i = 0; i < 20; i++) {
		test_fit = test_fit + abs(part->test_x[i] * sin(part->test_x[i]) + 0.1 *part->test_x[i]);

	}
	return test_fit;
}




//可视化7个阶段（6个中间路径点）
void GroupPathPlanningPSO::OrthoPhase(int k) {
	//全局最优位置gbestx
	for (int i = 18; i <= 31; i++) 
	//for (int i = 2; i <= 15; i++) 
	{
		Tooth* tooth = m_OrthoData->GetTooth((ToothId)i);
		if (tooth == NULL) continue;
		std::vector<std::vector<double >> orthophase;
		m_OrthoData->GetPSOGbestx((ToothId)i, orthophase);
		
		cout << "第" << i << "个牙齿的最优第" << k + 1 << "个路径点位置：" << orthophase[k][0] << "," << orthophase[k][1] << "," << orthophase[k][2] << "," << orthophase[k][3] << "," << orthophase[k][4] << "," << orthophase[k][5] << endl;
		/*std::vector<double> axes_x;
		m_OrthoData->GetMap((ToothId)i, 1, axes_x);
		std::vector<double> axes_y;
		m_OrthoData->GetMap((ToothId)i, 2, axes_y);
		std::vector<double> axes_z;
		m_OrthoData->GetMap((ToothId)i, 3, axes_z);*/
		std::vector<double> before;
		m_OrthoData->GetCenterBefore((ToothId)i, before);

		vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		transform->Translate(before[0], before[1], before[2]);
		transform->RotateWXYZ(orthophase[k][3], 1, 0, 0);
		transform->RotateWXYZ(orthophase[k][4], 0, 1, 0);
		transform->RotateWXYZ(orthophase[k][5], 0, 0, 1);
		transform->Translate(-before[0], -before[1], -before[2]);
		transform->Translate(orthophase[k][0] - before[0], orthophase[k][1] - before[1], orthophase[k][2] - before[2]);//平移	
		/*transform1->RotateWXYZ(orthophase[k][3], axes_x[0], axes_x[1], axes_x[2]);
		transform1->RotateWXYZ(orthophase[k][4], axes_y[0], axes_y[1], axes_y[2]);
		transform1->RotateWXYZ(orthophase[k][5], axes_z[0], axes_z[1], axes_z[2]);*/
		transform->Update();
		tooth->GetToothActor()->SetUserTransform(transform);
		tooth->GetToothActor()->SetVisibility(true);
		orthophase.clear();
		/*axes_x.clear();
		axes_y.clear();
		axes_z.clear();*/
		before.clear();
	}
	m_RenderContainer->Render();
}

void GroupPathPlanningPSO::PathPlan_Ours()
{
	ReadMatrixFile();//获取理想位置Matrix矩阵
	ToothData();//读取牙齿数据并保存牙齿初始位编码数据和目标位编码数据
	//RotateData_data3();//读取旋转分量
	//RotateData_data5();
	//RotateData_data7();
	RotateData_data8();
	Init_OurPSO();
	OnOurPSO();
}
void GroupPathPlanningPSO::Orthodontic5(particle* part, int it, double y) {
	// ----- 1. 动态惯性权重 w(t) 设计 -----
	// 参数定义
	const double Wmax = 0.8;
	const double Wmin = 0.4;
	const double mu = 0.5; // 可调参数μ，论文推荐0.5
	const int T = iter_max; // 最大迭代次数T

	// ----- 2. 混沌变量 zt（Logistic混沌映射） -----
	static double zt = 0.7; // 初值，可随机[0,1]，每50次重置避免周期性
	if (it % 50 == 0) {
		zt = (rand() % 10001) / 10000.0;
		if (zt < 0.01) zt = 0.01; // 避免极小值/零
		if (zt > 0.99) zt = 0.99;
	}
	zt = 4 * zt * (1 - zt);

	// ----- 3. 调控因子 α(t) -----
	double alpha = 1.0;
	if (it < T / 3) {
		// 迭代初期，抗扰动：sin²型
		alpha = pow(sin(PI * it / (2.0 * T)), 2);
	}
	else if (it < 2 * T / 3) {
		// 迭代中期，平衡：sin型
		alpha = sin(PI * it / T);
	}
	else {
		// 迭代后期，聚焦：cos²型
		alpha = pow(cos(PI * it / (2.0 * T)), 2);
	}

	// ----- 4. w(t) 综合惯性权重 -----
	double expA = exp(-double(it) / T) - exp(-1.0); // 分子
	double expB = exp(-1.0 / T) - exp(-1.0);        // 分母
	double smoothW = (expB == 0) ? 0 : (expA / expB);
	double w = Wmin + (Wmax - Wmin) * smoothW
		+ alpha * Wmax * mu * zt * (1 - zt);

	// ----- 5. 其它参数（学习因子等） -----
	double p = (rand() % 10001) / 10000.0; // 随机数[0,1]
	double c1, c2;
	if (p < 0.5) {
		c1 = 2.0 * sin(2 * PI * (1 - it / double(iter_max)));
		c2 = 2.0 * cos(2 * it * PI / double(iter_max));
	}
	else {
		c1 = 2.0 * cos(2 * PI * (1 - it / double(iter_max)));
		c2 = 2.0 * sin(2 * it * PI / double(iter_max));
	}

	// ----- 6. 主循环 -----
	for (int a = 0; a < 2; a++) {
		int startJ, endJ;
		if (a == 0) {
			startJ = 2;   // 上颌牙齿ID范围：2-15
			endJ = 16;
		}
		else {
			startJ = 18;  // 下颌牙齿ID范围：18-31
			endJ = 32;
		}

		for (int j = startJ; j < endJ; j++) {
			Tooth* tooth = m_OrthoData->GetTooth((ToothId)j);
			if (!tooth) {
				cout << "跳过无效牙齿ID：" << j << endl;
				continue;
			}

			for (int k = 2; k < n; k++) {
				// ---- 1. 平移（0,1,2） ----
				for (int d = 0; d < 3; d++) {
					double inertia = w; // 用动态w(t)替代原惯性因子
					double sinFactor = sin(it * PI * 0.5 / (iter_max - 1));
					double cosFactor = cos(it * PI * 0.5 / (iter_max - 1));

					double r1 = (2 * (rand() % (N + 1) / (float)(N + 1))) - 1;
					double r2 = (2 * (rand() % (N + 1) / (float)(N + 1))) - 1;

					part->v[j][k][d] =
						inertia * part->v[j][k][d]
						+ sinFactor * c1 * r1 * (part->bestx[j][k][d] - part->x[j][k][d])
						+ cosFactor * c2 * r2 * (gbestx[j][k][d] - part->x[j][k][d]);

					part->x[j][k][d] += part->v[j][k][d];

					// 动态平移边界约束
					double diff = abs(before[j][d] - after[j][d]);
					double minTrans = (before[j][d] + after[j][d]) / 2
						- 0.5 * (1 - it / (double)(iter_max - 1))
						- 0.2 * diff
						- 1;
					double maxTrans = (before[j][d] + after[j][d]) / 2
						+ 0.5 * (1 - it / (double)(iter_max - 1))
						+ 0.2 * diff
						+ 1;
					part->x[j][k][d] = max(min(part->x[j][k][d], maxTrans), minTrans);
				}

				// ---- 单步3D平移约束 ----
				double dx = part->x[j][k][0] - part->x[j][k - 1][0];
				double dy = part->x[j][k][1] - part->x[j][k - 1][1];
				double dz = part->x[j][k][2] - part->x[j][k - 1][2];
				double step3D = sqrt(dx * dx + dy * dy + dz * dz);
				if (step3D > 0.3) {
					double scale = 0.3 / step3D;
					part->x[j][k][0] = part->x[j][k - 1][0] + dx * scale;
					part->x[j][k][1] = part->x[j][k - 1][1] + dy * scale;
					part->x[j][k][2] = part->x[j][k - 1][2] + dz * scale;
				}

				// ---- 2. 旋转（3,4,5） ----
				for (int d = 3; d < 6; d++) {
					double inertia = w; // 用动态w(t)替代原惯性因子
					double sinFactor = sin(it * PI * 0.5 / (iter_max - 1));
					double cosFactor = cos(it * PI * 0.5 / (iter_max - 1));

					double r1 = (2 * (rand() % (N + 1) / (float)(N + 1))) - 1;
					double r2 = (2 * (rand() % (N + 1) / (float)(N + 1))) - 1;

					part->v[j][k][d] =
						inertia * part->v[j][k][d]
						+ sinFactor * c1 * r1 * (part->bestx[j][k][d] - part->x[j][k][d])
						+ cosFactor * c2 * r2 * (gbestx[j][k][d] - part->x[j][k][d]);

					part->x[j][k][d] += part->v[j][k][d];

					double diff = abs(before[j][d] - after[j][d]);
					double minRot = (before[j][d] + after[j][d]) / 2
						- 0.5 * (1 - it / (double)(iter_max - 1))
						- 0.2 * diff
						- 1;
					double maxRot = (before[j][d] + after[j][d]) / 2
						+ 0.5 * (1 - it / (double)(iter_max - 1))
						+ 0.2 * diff
						+ 1;
					part->x[j][k][d] = max(min(part->x[j][k][d], maxRot), minRot);
				}

				// ---- 单步旋转约束 ----
				double rotX = part->x[j][k][3] - part->x[j][k - 1][3];
				double rotY = part->x[j][k][4] - part->x[j][k - 1][4];
				double rotZ = part->x[j][k][5] - part->x[j][k - 1][5];
				const double MAX_ROT_SUM = 2 * PI / 180;
				double rotSum = abs(rotX) + abs(rotY) + abs(rotZ);
				if (rotSum > MAX_ROT_SUM) {
					double scale = MAX_ROT_SUM / rotSum;
					// 按比例缩小三轴旋转步长
					part->x[j][k][3] = part->x[j][k - 1][3] + rotX * scale;
					part->x[j][k][4] = part->x[j][k - 1][4] + rotY * scale;
					part->x[j][k][5] = part->x[j][k - 1][5] + rotZ * scale;
				}
			}
			// ====== 检查终点是否收敛到目标点（只检测不强制） ======
			bool reached_goal = true;
			for (int d = 0; d < 6; d++) {
				if (fabs(part->x[j][n][d] - after[j][d]) > 1e-5) { // 允许极小误差
					reached_goal = false;
					break;
				}
			}
			if (reached_goal) {
				std::cout << "Tooth " << j << " 到达终点！ " << it << std::endl;
			}
			//若未到达，输出偏差
			else if (it == iter_max - 1) {
				std::cout << "Tooth " << j << " final deviation: ";
				for (int d = 0; d < 6; d++) {
					std::cout << (part->x[j][n][d] - after[j][d]) << " ";
				}
				std::cout << std::endl;
			}
		}
	}
}

void GroupPathPlanningPSO::Init_OurPSO() {//初始化粒子群（实现论文线性插值+余弦扰动）
// 初始化全局最优适应度
	gbestf = 1000000;
	pbestf_it = 10000;
	gbestf_it = 10000;
	pbestf_it_upper = 10000;
	pbestf_it_lower = 10000;
	gbestf_it_upper = 10000;
	gbestf_it_lower = 10000;

	srand(time(NULL));  // 随机数种子

	// 遍历每个粒子（共z个）
	for (int i = 0; i < z; i++) {
		particle* p = &swarm[i];
		p->bestf = 1000000;  // 初始化粒子历史最优适应度

		// 处理上颌牙齿（2-15）
		for (int j = 2; j < 16; j++) {
			// 1. 初始化起点（k=1）和终点（k=25）
			for (int d = 0; d < 6; d++) {
				p->x[j][1][d] = before[j][d];   // 起点=初始位置
				p->x[j][n][d] = after[j][d];    // 终点=目标位置
				// 初始速度（小随机值，避免波动过大）
				p->v[j][1][d] = (rand() % 100 / 100.0) / 5;
				p->v[j][n][d] = (rand() % 100 / 100.0) / 5;
			}

			// 2. 生成余弦干扰序列（全局探索：混沌映射）
			double cosSeq[25] = { 0 };  // 干扰序列（长度=路径点数量）
			cosSeq[0] = rand() % 100 / 100.0;  // 初始值∈[0,1)
			for (int s = 1; s < n - 1; s++) {  // 生成后续序列
				cosSeq[s] = k_cos * cos(PI * cosSeq[s - 1]);  // 余弦混沌映射
			}

			// 3. 生成正弦干扰序列（局部微调：基于余弦序列的波动）
			double sinSeq[25] = { 0 };
			for (int s = 0; s < n - 1; s++) {
				// 基于余弦序列生成正弦扰动（保持局部相关性，避免随机跳跃）
				sinSeq[s] = k_sin * sin(2 * PI * cosSeq[s] + (rand() % 100 / 100.0));
			}

			// 4. 初始化中间路径点（k=2到k=24）：线性插值+双重扰动
			for (int k = 2; k < n; k++) {
				// （1）线性插值基础位置（锁定核心方向）
				double ratio = (k - 1.0) / (n - 1);  // 阶段比例（0~1）
				double linearPos[6];
				for (int d = 0; d < 6; d++) {
					linearPos[d] = before[j][d] + (after[j][d] - before[j][d]) * ratio;
				}

				// （2）叠加余弦扰动（全局探索：控制幅度）
				int seqIdx = k - 2;  // 干扰序列索引（对应当前路径点）
				double cosDisturb[6];
				for (int d = 0; d < 6; d++) {
					// 平移维度（d=0-2）扰动幅度稍大，旋转维度（d=3-5）稍小
					cosDisturb[d] = (d < 3) ? cosSeq[seqIdx] * 0.15 : cosSeq[seqIdx] * 0.05;
				}

				// （3）叠加正弦扰动（局部微调：补充细节）
				double sinDisturb[6];
				for (int d = 0; d < 6; d++) {
					// 平移维度（d=0-2）微调幅度小，旋转维度（d=3-5）稍大（适应临床旋转需求）
					sinDisturb[d] = (d < 3) ? sinSeq[seqIdx] * 0.05 : sinSeq[seqIdx] * 0.1;
				}

				// （4）计算最终位置并限制总扰动（避免超出临床安全范围）
				for (int d = 0; d < 6; d++) {
					p->x[j][k][d] = linearPos[d] + cosDisturb[d] + sinDisturb[d];

					// 总扰动幅度控制（平移≤0.3mm，旋转≤2°）
					double totalDisturb = fabs(p->x[j][k][d] - linearPos[d]);
					if (d < 3 && totalDisturb > MAX_STEP_TRANS) {  // 平移维度
						p->x[j][k][d] = linearPos[d] + MAX_STEP_TRANS * (p->x[j][k][d] - linearPos[d]) / totalDisturb;
					}
					else if (d >= 3 && totalDisturb > MAX_STEP_ROT) {  // 旋转维度
						p->x[j][k][d] = linearPos[d] + MAX_STEP_ROT * (p->x[j][k][d] - linearPos[d]) / totalDisturb;
					}

					// 初始化速度（小随机值）
					p->v[j][k][d] = (rand() % 100 / 100.0) / 5;
				}
			}
		}

		// 处理下颌牙齿（18-31，逻辑同上颌）
		for (int j = 18; j < 32; j++) {
			// 1. 起点和终点初始化
			for (int d = 0; d < 6; d++) {
				p->x[j][1][d] = before[j][d];
				p->x[j][n][d] = after[j][d];
				p->v[j][1][d] = p->v[j][n][d] = (rand() % 100 / 100.0) / 5;
			}

			// 2. 生成余弦干扰序列
			double cosSeq[25] = { 0 };
			cosSeq[0] = rand() % 100 / 100.0;
			for (int s = 1; s < n - 1; s++) {
				cosSeq[s] = k_cos * cos(PI * cosSeq[s - 1]);
			}

			// 3. 生成正弦干扰序列
			double sinSeq[25] = { 0 };
			for (int s = 0; s < n - 1; s++) {
				sinSeq[s] = k_sin * sin(2 * PI * cosSeq[s] + (rand() % 100 / 100.0));
			}

			// 4. 初始化中间路径点（线性插值+双重扰动）
			for (int k = 2; k < n; k++) {
				double ratio = (k - 1.0) / (n - 1);
				double linearPos[6];
				for (int d = 0; d < 6; d++) {
					linearPos[d] = before[j][d] + (after[j][d] - before[j][d]) * ratio;
				}

				int seqIdx = k - 2;
				double cosDisturb[6], sinDisturb[6];
				for (int d = 0; d < 6; d++) {
					cosDisturb[d] = (d < 3) ? cosSeq[seqIdx] * 0.15 : cosSeq[seqIdx] * 0.05;
					sinDisturb[d] = (d < 3) ? sinSeq[seqIdx] * 0.05 : sinSeq[seqIdx] * 0.1;
					p->x[j][k][d] = linearPos[d] + cosDisturb[d] + sinDisturb[d];

					// 扰动幅度限制
					double totalDisturb = fabs(p->x[j][k][d] - linearPos[d]);
					if (d < 3 && totalDisturb > MAX_STEP_TRANS) {
						p->x[j][k][d] = linearPos[d] + MAX_STEP_TRANS * (p->x[j][k][d] - linearPos[d]) / totalDisturb;
					}
					else if (d >= 3 && totalDisturb > MAX_STEP_ROT) {
						p->x[j][k][d] = linearPos[d] + MAX_STEP_ROT * (p->x[j][k][d] - linearPos[d]) / totalDisturb;
					}

					p->v[j][k][d] = (rand() % 100 / 100.0) / 5;
				}
			}
		}

		// 5. 计算初始适应度并更新最优解
		p->fitness = Fitness(p);  // 调用适应度函数

		// 更新粒子历史最优
		if (p->fitness < p->bestf) {
			for (int j = 2; j < 32; j++)
				for (int k = 1; k <= n; k++)
					for (int d = 0; d < 6; d++)
						p->bestx[j][k][d] = p->x[j][k][d];
			p->bestf = p->fitness;
		}

		// 更新全局最优
		if (p->fitness < gbestf) {
			for (int j = 2; j < 32; j++)
				for (int k = 1; k <= n; k++)
					for (int d = 0; d < 6; d++)
						gbestx[j][k][d] = p->x[j][k][d];
			gbestf = p->fitness;
		}
	}

	cout << "初始化完成，全局最优适应度：" << gbestf << endl;
}

void GroupPathPlanningPSO::OnOurPSO() {
	srand(time(NULL));
	for (int q = 0; q < 100; ++q) {
		double U1 = rand() * 1.0f / RAND_MAX;
		double U2 = rand() * 1.0f / RAND_MAX;
		Z[q] = sqrt(-2 * log(U1)) * cos(2 * PI * U2);
	}

	// 初始化分群（高/中/低适应度群）
	int highGroup[z] = { 0 }, midGroup[z] = { 0 }, lowGroup[z] = { 0 };
	double fitList[z] = { 0 };
	int highNum = z * 0.2;
	int midNum = z * 0.6;
	int lowNum = z - highNum - midNum;
	std::vector<int> idx(z - 1);

	// 初始适应度统计与分群
	for (int i = 1; i < z; i++) {
		particle* p = &swarm[i];
		fitList[i] = Fitness(p);
		idx[i - 1] = i;
	}
	std::sort(idx.begin(), idx.end(), [&](int a, int b) { return fitList[a] < fitList[b]; });
	for (int i = 0; i < highNum; i++) highGroup[idx[i]] = 1;
	for (int i = highNum; i < highNum + midNum; i++) midGroup[idx[i]] = 1;
	for (int i = highNum + midNum; i < z - 1; i++) lowGroup[idx[i]] = 1;

	for (int iter = 1; iter < iter_max; iter++) {
		// 每5代重新适应度统计分群
		if (iter % 5 == 0) {
			for (int i = 1; i < z; i++) fitList[i] = Fitness(&swarm[i]);
			std::sort(idx.begin(), idx.end(), [&](int a, int b) { return fitList[a] < fitList[b]; });
			std::fill(highGroup, highGroup + z, 0);
			std::fill(midGroup, midGroup + z, 0);
			std::fill(lowGroup, lowGroup + z, 0);
			for (int i = 0; i < highNum; i++) highGroup[idx[i]] = 1;
			for (int i = highNum; i < highNum + midNum; i++) midGroup[idx[i]] = 1;
			for (int i = highNum + midNum; i < z - 1; i++) lowGroup[idx[i]] = 1;
		}
		for (int i = 1; i < z; i++) {
			particle* p = &swarm[i];

			// 计算phase
			double phase = iter * PI * 0.5 / (iter_max - 1);
			double w, c1, c2;

			// == 高级子群D1 ==
			if (highGroup[i]) {
				w = 0.4 - iter * 1.0 / (iter_max - 1) * 0.2;
				c1 = 1.8 + 0.2 * sin(phase);
				c2 = 1.0 + 0.2 * cos(phase);
				for (int j = 2; j < 32; j++) {
					for (int k = 2; k < n; k++) {
						int seqIdx = k - 2;
						for (int d = 0; d < 6; d++) {
							double r1 = ((rand() % 100) / 100.0) * 2 - 1;
							double r2 = ((rand() % 100) / 100.0) * 2 - 1;
							double v = w * p->v[j][k][d]
								+ sin(phase) * c1 * r1 * (p->bestx[j][k][d] - p->x[j][k][d])
								+ cos(phase) * c2 * r2 * (gbestx[j][k][d] - p->x[j][k][d]);
							p->v[j][k][d] = v;
							double delta = 0.03 * sin(seqIdx) + 0.03;
							p->x[j][k][d] += 0.3 * v + delta;
							if (d < 3 && fabs(p->x[j][k][d] - p->x[j][k - 1][d]) > 0.1)
								p->x[j][k][d] = p->x[j][k - 1][d] + (p->x[j][k][d] > p->x[j][k - 1][d] ? 0.1 : -0.1);
							if (d >= 3 && fabs(p->x[j][k][d] - p->x[j][k - 1][d]) > 0.5 * PI / 180)
								p->x[j][k][d] = p->x[j][k - 1][d] + (p->x[j][k][d] > p->x[j][k - 1][d] ? 0.5 * PI / 180 : -0.5 * PI / 180);
						}
						if (!Constraint(p, j)) {
							for (int d = 0; d < 6; d++) p->x[j][k][d] = p->x[j][k - 1][d];
						}
					}
				}
			}
			// == 中级子群D2 ==
			else if (midGroup[i]) {
				w = 0.9 - iter * 1.0 / (iter_max - 1) * 0.2;
				c1 = 1.4 + 0.4 * sin(phase);
				c2 = 1.4 + 0.4 * cos(phase);
				for (int j = 2; j < 32; j++) {
					for (int k = 2; k < n; k++) {
						int seqIdx = k - 2;
						for (int d = 0; d < 6; d++) {
							double r1 = ((rand() % 100) / 100.0) * 2 - 1;
							double r2 = ((rand() % 100) / 100.0) * 2 - 1;
							double v = w * p->v[j][k][d]
								+ sin(phase) * c1 * r1 * (p->bestx[j][k][d] - p->x[j][k][d])
								+ cos(phase) * c2 * r2 * (gbestx[j][k][d] - p->x[j][k][d]);
							p->v[j][k][d] = v;
							double delta = 0.1 * cos(seqIdx) + 0.1;
							p->x[j][k][d] += v + delta;
							if (d < 3 && fabs(p->x[j][k][d] - p->x[j][k - 1][d]) > 0.2)
								p->x[j][k][d] = p->x[j][k - 1][d] + (p->x[j][k][d] > p->x[j][k - 1][d] ? 0.2 : -0.2);
							if (d >= 3 && fabs(p->x[j][k][d] - p->x[j][k - 1][d]) > 1 * PI / 180)
								p->x[j][k][d] = p->x[j][k - 1][d] + (p->x[j][k][d] > p->x[j][k - 1][d] ? 1 * PI / 180 : -1 * PI / 180);
						}
					}
				}
			}
			// == 低级子群D3 ==
			else if (lowGroup[i]) {
				w = 0.9 - iter * 1.0 / (iter_max - 1) * 0.2;
				c1 = 1.0 + 0.2 * sin(phase);
				c2 = 1.8 + 0.2 * cos(phase);
				if (iter % 10 == 0) {
					for (int j = 2; j < 32; j++) {
						for (int k = 2; k < n; k++) {
							for (int d = 0; d < 6; d++) {
								double k_cos = 0.8, k_sin = 0.6;
								p->x[j][k][d] = k_cos * cos(iter * PI / (iter_max - 1)) + k_sin * sin(iter * PI / (iter_max - 1));
								if (d < 3 && fabs(p->x[j][k][d] - p->x[j][k - 1][d]) > 2.0)
									p->x[j][k][d] = p->x[j][k - 1][d] + (p->x[j][k][d] > p->x[j][k - 1][d] ? 2.0 : -2.0);
								if (d >= 3 && fabs(p->x[j][k][d] - p->x[j][k - 1][d]) > 10 * PI / 180)
									p->x[j][k][d] = p->x[j][k - 1][d] + (p->x[j][k][d] > p->x[j][k - 1][d] ? 10 * PI / 180 : -10 * PI / 180);
							}
						}
					}
				}
				else {
					for (int j = 2; j < 32; j++) {
						for (int k = 2; k < n; k++) {
							for (int d = 0; d < 6; d++) {
								double r1 = ((rand() % 100) / 100.0) * 2 - 1;
								double r2 = ((rand() % 100) / 100.0) * 2 - 1;
								double v = w * p->v[j][k][d]
									+ sin(phase) * c1 * r1 * (p->bestx[j][k][d] - p->x[j][k][d])
									+ cos(phase) * c2 * r2 * (gbestx[j][k][d] - p->x[j][k][d]);
								p->v[j][k][d] = v;
								p->x[j][k][d] += v;
								if (d < 3 && fabs(p->x[j][k][d] - p->x[j][k - 1][d]) > 2.0)
									p->x[j][k][d] = p->x[j][k - 1][d] + (p->x[j][k][d] > p->x[j][k - 1][d] ? 2.0 : -2.0);
								if (d >= 3 && fabs(p->x[j][k][d] - p->x[j][k - 1][d]) > 10 * PI / 180)
									p->x[j][k][d] = p->x[j][k - 1][d] + (p->x[j][k][d] > p->x[j][k - 1][d] ? 10 * PI / 180 : -10 * PI / 180);
							}
						}
					}
				}
			}
			double pbest = Pbest(p);
			if (pbest < pbestf_it) pbestf_it = pbest;
			if (pbest < gbestf_it) gbestf_it = pbest;
			std::cout << "第" << i << "个粒子第" << iter << "次迭代的pbest适应度值：" << pbest << std::endl;
			std::cout << "第" << i << "个粒子第" << iter << "次迭代的粒子历史最优解pbestf_it值：" << pbestf_it << std::endl;
			std::cout << "第" << i << "个粒子第" << iter << "次迭代的粒子全局最优解gbestf_it值：" << gbestf_it << std::endl;

			double pbest_upper = Pbest_upper(p);
			if (pbest_upper < pbestf_it_upper) pbestf_it_upper = pbest_upper;
			if (pbest_upper < gbestf_it_upper) gbestf_it_upper = pbest_upper;
			std::cout << "第" << i << "个粒子第" << iter << "次迭代的pbest_upper上颌适应度值：" << pbest_upper << std::endl;
			std::cout << "第" << i << "个粒子第" << iter << "次迭代的粒子历史最优解pbestf_it_upper 值：" << pbestf_it_upper << std::endl;
			std::cout << "第" << i << "个粒子第" << iter << "次迭代的粒子全局最优解gbestf_it_upper 值：" << gbestf_it_upper << std::endl;

			double pbest_lower = Pbest_lower(p);
			if (pbest_lower < pbestf_it_lower) pbestf_it_lower = pbest_lower;
			if (pbest_lower < gbestf_it_lower) gbestf_it_lower = pbest_lower;
			std::cout << "第" << i << "个粒子第" << iter << "次迭代的pbest_lower下颌适应度值值：" << pbest_lower << std::endl;
			std::cout << "第" << i << "个粒子第" << iter << "次迭代的粒子历史最优解pbestf_it_lower值：" << pbestf_it_lower << std::endl;
			std::cout << "第" << i << "个粒子第" << iter << "次迭代的粒子全局最优解gbestf_it_lower值：" << gbestf_it_lower << std::endl;

			p->fitness = Fitness(p);
			if (p->fitness < p->bestf) {
				for (int j = 1; j < m; j++)
					for (int k = 1; k < n; k++)
						for (int d = 0; d < 6; d++)
							p->bestx[j][k][d] = p->x[j][k][d];
				p->bestf = p->fitness;
			}
			if (p->fitness < gbestf) {
				for (int j = 1; j < m; j++)
					for (int k = 1; k < n; k++)
						for (int d = 0; d < 6; d++)
							gbestx[j][k][d] = p->x[j][k][d];
				gbestf = p->fitness;
			}
			std::cout << "第" << i << "个粒子第" << iter << "次迭代带碰撞检测的的p->fitness值：" << p->fitness << std::endl;
			std::cout << "第" << i << "个粒子第" << iter << "次迭代的带碰撞检测的粒子历史最优解p->bestf值：" << p->bestf << std::endl;
			std::cout << "第" << i << "个粒子第" << iter << "次迭代的带碰撞检测的粒子全局最优解gbestf值：" << gbestf << std::endl;
			std::cout << "----------------------------------------------------------------------------------" << std::endl;

			// 输出每个牙齿所有路径点
			for (int j = 2; j < 32; j++)
				for (int k = 1; k <= n; k++) {
					std::cout << "粒子" << i << "迭代" << iter << "牙齿" << j << "路径点" << k << "位置: ";
					for (int d = 0; d < 6; d++) {
						std::cout << p->x[j][k][d];
						if (d < 5) std::cout << ",";
					}
					std::cout << std::endl;
				}
		}
		// 子群协作学习（每10代一次）
		if (iter % 10 == 0) {
			// 1. 随机选出3个D1模板粒子
			std::vector<int> templateIdx;
			while (templateIdx.size() < 3) {
				int tidx = idx[rand() % highNum];
				if (std::find(templateIdx.begin(), templateIdx.end(), tidx) == templateIdx.end())
					templateIdx.push_back(tidx);
			}

			// 2. D2中50%粒子局部学习（如第5-10阶段）
			int midLearnNum = midNum * 0.5;
			for (int nmid = 0; nmid < midLearnNum; nmid++) {
				int i = idx[highNum + nmid];
				particle* p = &swarm[i];
				int temp = templateIdx[rand() % templateIdx.size()];
				// 学习模板的第5-10阶段（局部路径）
				for (int j = 2; j < 32; j++)
					for (int k = 5; k <= 10; k++)
						for (int d = 0; d < 6; d++)
							p->x[j][k][d] = swarm[temp].bestx[j][k][d];
				p->fitness = Fitness(p);
				if (p->fitness < p->bestf) {
					for (int j = 1; j < m; j++)
						for (int k = 1; k < n; k++)
							for (int d = 0; d < 6; d++)
								p->bestx[j][k][d] = p->x[j][k][d];
					p->bestf = p->fitness;
				}
				if (p->fitness < gbestf) {
					for (int j = 1; j < m; j++)
						for (int k = 1; k < n; k++)
							for (int d = 0; d < 6; d++)
								gbestx[j][k][d] = p->x[j][k][d];
					gbestf = p->fitness;
				}
			}

			// 3. D3中30%粒子全局学习（所有阶段）
			int lowLearnNum = lowNum * 0.3;
			for (int nlow = 0; nlow < lowLearnNum; nlow++) {
				int i = idx[highNum + midNum + nlow];
				particle* p = &swarm[i];
				int temp = templateIdx[rand() % templateIdx.size()];
				for (int j = 2; j < 32; j++)
					for (int k = 1; k <= n; k++)
						for (int d = 0; d < 6; d++)
							p->x[j][k][d] = swarm[temp].bestx[j][k][d];
				p->fitness = Fitness(p);
				if (p->fitness < p->bestf) {
					for (int j = 1; j < m; j++)
						for (int k = 1; k < n; k++)
							for (int d = 0; d < 6; d++)
								p->bestx[j][k][d] = p->x[j][k][d];
					p->bestf = p->fitness;
				}
				if (p->fitness < gbestf) {
					for (int j = 1; j < m; j++)
						for (int k = 1; k < n; k++)
							for (int d = 0; d < 6; d++)
								gbestx[j][k][d] = p->x[j][k][d];
					gbestf = p->fitness;
				}
			}
		}
	}
	// 输出全局最优路径
	std::cout << "全局最优适应度：" << gbestf << std::endl;
	tempgbest* t = &tp;
	for (int j = 1; j < 32; j++) {
		for (int k = 1; k < n + 1; k++) {
			for (int d = 0; d < 6; d++) t->Tempgbestx[j][k][d] = gbestx[j][k][d];
		}
	}
	for (int a = 0; a < 2; a++) {
		for (int j = 2 + 16 * a; j < 16 + 16 * a; j++) {
			std::vector<std::vector<double>> gbestphase;
			for (int k = 1; k <= n; k++) {
				std::cout << "第" << j << "个牙齿的最优第" << k << "个路径点位置：" << gbestx[j][k][0] << "," << gbestx[j][k][1] << "," << gbestx[j][k][2] << "," << gbestx[j][k][3] << "," << gbestx[j][k][4] << "," << gbestx[j][k][5] << std::endl;
				std::vector<double> gbestphase1;
				for (int d = 0; d < 6; d++) gbestphase1.push_back(gbestx[j][k][d]);
				gbestphase.push_back(gbestphase1);
				gbestphase1.clear();
			}
			m_OrthoData->SetPSOGbestx((ToothId)j, gbestphase);
			gbestphase.clear();
		}
	}
	double gbestmove = GbestMove(t);
	double gbestrotate = GbestRotate(t);
	double fitnessvalue = GbestFit(t);
	double gbestmove_upper = GbestMove_upper(t);
	std::cout << "上颌最终最优解平移量gbestmove_upper: " << gbestmove_upper << std::endl;
	double gbestrotate_upper = GbestRotate_upper(t);
	double fitnessvalue_upper = GbestFit_upper(t);
	double gbestmove_lower = GbestMove_lower(t);
	double gbestrotate_lower = GbestRotate_lower(t);
	double fitnessvalue_lower = GbestFit_lower(t);

	double m1 = GbestMove_upper1(t);
	double m2 = GbestMove_upper2(t);
	double m3 = GbestMove_upper3(t);
	double r1 = GbestRotate_upper4(t);
	double r2 = GbestRotate_upper5(t);
	double r3 = GbestRotate_upper6(t);

	std::cout << "带碰撞检测的全局最优适应度为：" << gbestf << std::endl;
	std::cout << "最终最优解平移量gbestmove：" << gbestmove << std::endl;
	std::cout << "最终最优解旋转量gbestrotate: " << gbestrotate << std::endl;
	std::cout << "最终最优解适应度值fitnessvalue：" << fitnessvalue << std::endl;
	std::cout << "上颌最终最优解平移量gbestmove：" << gbestmove_upper << std::endl;
	std::cout << "上颌最终最优解旋转量gbestrotate: " << gbestrotate_upper << std::endl;
	std::cout << "上颌最终最优解适应度值fitnessvalue：" << fitnessvalue_upper << std::endl;
	std::cout << "-------------------------------------------------------" << std::endl;
	std::cout << "上颌m1" << m1 << std::endl;
	std::cout << "上颌m2" << m2 << std::endl;
	std::cout << "上颌m3" << m3 << std::endl;
	std::cout << "上颌r1" << r1 << std::endl;
	std::cout << "上颌r2" << r2 << std::endl;
	std::cout << "上颌r3" << r3 << std::endl;
}


void GroupPathPlanningPSO::OpenGroup() {
	m_IsOpen = true;
	cout << "打开功能" << endl;	
	
}

void GroupPathPlanningPSO::CloseGroup() {
	m_IsOpen = false;
	cout << "关闭功能" << endl;
	
}

