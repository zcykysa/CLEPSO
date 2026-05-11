#include "OrthoData.h"
#include <iostream>
#include "QFileDialog.h"
#include "vtkSTLReader.h"
#include "vtkTriangleFilter.h"
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include<map>
#include"Arch.h"
#include "Tooth.h"
#include "Gum.h"
#include "Common.h"
#include"vtkSmartPointer.h"
#include"QFile.h"
#include <vector>
using namespace std;

OrthoData::OrthoData()
{
	//创建相应的上/下牙颌对象，并存入m_Arch容器中
	m_Arch[Arch_Upper] = new Arch(Arch_Upper); 
	m_Arch[Arch_Lower] = new Arch(Arch_Lower);
	
}

OrthoData::~OrthoData()
{
	
	delete m_Arch[Arch_Upper]; //删除相应的对象
	delete m_Arch[Arch_Lower];
	
	m_Arch.clear();            //清除容器中所有的对象
	
}


Arch* OrthoData::GetArch(ArchType type) {  //获取Arch指针
	Arch* arch = m_Arch[type];
	return arch;
}

std::string OrthoData::GetCasePath() //获取数据路径
{
	return m_CasePath;
}

void OrthoData::LoadData(std::string path) //传入数据路径，调用相应函数去加载数据
{
	m_CasePath = path;
	//LoadArch();  //这块加载的是没有分离的上下牙颌数据
	LoadTooth();
	//LoadGum();
}

//-----------------------------------------------------------------------------------------------------------

bool OrthoData::LoadTooth() //加载牙齿数据
{
	/*
		Models文件夹中存着:没有分离的上下牙颌数据(Mandibular.stl<下>，Maxillary.stl<上>)
						   已经分离的各个单颗牙齿数据(Tooth_2.stl------>Tooth_31.stl)
						   其中单颗牙齿数据少了5，12，16,17，21，28.stl
						   没有牙齿的上下牙龈数据(Tooth_LowerJaw.stl<下>，Tooth_UpperJaw.stl<上>)
	*/
	QString path = QString(m_CasePath.c_str()); 
	QString filePath = path + "tooth/";    //获取当前文件夹所在路径
	//for (int i = 2; i <= 15; i++)//data3
	for (int i = 18; i <= 31; i++)//data7
	{
		QString strToothid = QString::number(i);
		QString fileName = filePath + "tooth_" + strToothid + ".stl"; //获取对应牙齿文件名//data3
		//QString fileName = filePath + "tooth_" + strToothid + ".stl"; //获取对应牙齿文件名//data7
		QFile dir(fileName);
		if (!dir.exists()) continue; //若文件名不存在，则停止执行进入下一次循环

		vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
		reader->SetFileName(fileName.toStdString().c_str());
		reader->Update();

		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(reader->GetOutputPort());

		vtkSmartPointer<vtkActor>actor = vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(mapper);

		ToothId id = (ToothId)i;  //这里只是为了下面调用i枚举值方便

		//将1-16号牙齿存入Arch_Upper
		if (i <= 16)
		{
			//通过m_Arch[Arch_Upper]获取Arch*，创建对应牙齿指针，放入到my_Tooth容器中
			m_Arch[Arch_Upper]->Creat_Tooth(id);

			//通过m_Arch[Arch_Upper]获取Arch*，调用Get_tooth(id)返回对应id的Tooth*，
			//接着设置上述生成的actor对象
			m_Arch[Arch_Upper]->Get_Tooth(id)->SetToothActor(actor);
			//存储PolyData数据
			m_Arch[Arch_Upper]->Get_Tooth(id)->SetToothPolyData(reader->GetOutput());
			
		//注：以上两行代码已经将不同枚举值的Arch* 对应的my_Tooth容器中每个对应的Tooth*设置了其对应的牙齿actor(这很重要)
		}
		//17-31号牙齿存入Arch_Lower
		else
		{
			m_Arch[Arch_Lower]->Creat_Tooth(id);
			m_Arch[Arch_Lower]->Get_Tooth(id)->SetToothActor(actor);
			
			m_Arch[Arch_Lower]->Get_Tooth(id)->SetToothPolyData(reader->GetOutput());

		}
	}
	return true;
}

bool OrthoData::LoadGum() //加载牙龈数据
{
	QString path = QString(m_CasePath.c_str());
	QString filePath = path + "Models/";

	for (int i = 0; i < 2; i++)
	{
		QString strArch = "Tooth_UpperJaw.stl"; //该.stl是牙齿与牙龈分离的(只有牙龈模型)上牙龈数据
		ArchType type = Arch_Upper;
		if (i == 1)
		{
			strArch = "Tooth_LowerJaw.stl";  //该.stl是牙齿与牙龈分离的(只有牙龈模型)下牙龈数据
			type = Arch_Lower;
		}

		QString fileName = filePath + strArch;

		QFile dir(fileName);
		if (!dir.exists())continue;

		vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
		reader->SetFileName(fileName.toStdString().c_str());
		reader->Update();

		vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(reader->GetOutputPort());

		vtkSmartPointer<vtkActor>actor = vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(mapper);

		//m_Arch[type]返回一个Arch*，调用Creat_Gum()实例化一个牙龈对象
		m_Arch[type]->Creat_Gum();

		//m_Arch[type]返回一个Arch*，调用GetGum()返回当前已经实例化的Gum*
		//接着通过SetGumActor()设置上述生成的actor对象
		m_Arch[type]->Get_Gum()->SetGumActor(actor);

	    //存储PolyData数据
		m_Arch[type]->Get_Gum()->SetGumPolyData(reader->GetOutput());
		//注：上述两行代码也已经为不同枚举值 所对应的Arch*里的Gum指针 设置了其对应的actor对象(这很重要)
	}
	return true;
}

bool OrthoData::LoadArch() //加载Arch数据
{
	QString path = QString(m_CasePath.c_str());

	QString filePath = path + "Models/";

	for (int i = 0; i < 2; i++)
	{
		QString strArch = "Maxillary.stl";  //牙齿与牙龈没有分离的上牙颌数据
		ArchType type = Arch_Upper;
		if (i == 1)
		{
			strArch = "Mandibular.stl";  //牙齿与牙龈没有分离的下牙颌数据
			type = Arch_Lower;
		}

		QString fileName = filePath + strArch;

		QFile file(fileName);
		if (!file.exists())continue;
		vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
		reader->SetFileName(fileName.toStdString().c_str());
		reader->Update();

		vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(reader->GetOutputPort());

		vtkSmartPointer<vtkActor>actor = vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(mapper);

		//m_Arch[type]返回一个Arch*，将生成的actor设置到 对应的枚举值 所对应的Arch指针里的m_ArchActor中
		m_Arch[type]->SetArchActor(actor); 
		m_Arch[type]->SetArchPolyData(reader->GetOutput());
		
	}
	return true;
}

//-------------------------------------------------------------------------------------------------------


//注：此函数是建立在上述已经将所有牙齿和牙龈的actor设置到了map容器中各枚举值所对应的value值中去了
//    因此下述代码直接调用Add函数 获取到所设置的每个actor后放入ren中
void OrthoData::AddActors(vtkSmartPointer<vtkRenderer>ren) //向指定ren中添加牙齿和牙龈actor
{
	for (int i = 0; i < 2; i++)
	{
		ArchType type = Arch_Upper;    //获取上牙颌(枚举)
		if (i == 1)
		{
			type = Arch_Lower;        //获取下牙颌(枚举)
		}
		//my_arch[type]->AddArchActor(ren);

		//m_Arch[type]返回Arch*，调用AddToothActor()通过m_Tooth容器的迭代器将牙齿加入到渲染窗口中
		m_Arch[type]->AddToothActor(ren);  

		//m_Arch[type]返回Arch*，调用AddToothGum()通过实例化的my_gum对象将牙龈加入到渲染窗口中
		m_Arch[type]->AddToothGum(ren);    
	}
}

//获取Arch数据(polydata)
Arch* OrthoData::GetArchPolyData(ArchType type)
{
	Arch* arch = m_Arch[type];
	return arch;
}


//获取上下牙颌的牙齿编号
vector<ToothId> OrthoData::Get_TeethId(ArchType type)
{
	//创建vector容器，存放上/下颌的ToothId
	std::vector<ToothId>m_ToothId;
	if (type == Arch_Upper)
	{
		//创建临时数组，存放上颌牙齿的ToothId
		ToothId  ToothId_Upper[16] = { Tooth_18 ,Tooth_17, Tooth_16, Tooth_15, Tooth_14, Tooth_13, Tooth_12, Tooth_11,
									   Tooth_21, Tooth_22, Tooth_23, Tooth_24, Tooth_25, Tooth_26, Tooth_27, Tooth_28 };

		//使用循环将数组中的ToothId存入vector容器中
		for (int i = 0; i < sizeof(ToothId_Upper); i++)
		{

			m_ToothId.push_back(ToothId_Upper[i]);

		}
	}
	else if (type == Arch_Lower)
	{
		//创建临时数组，存放下颌牙齿的ToothId
		ToothId  ToothId_Lower[16] = { Tooth_48, Tooth_47, Tooth_46, Tooth_45, Tooth_44, Tooth_43, Tooth_42, Tooth_41,
									   Tooth_31, Tooth_32, Tooth_33, Tooth_34, Tooth_35, Tooth_36, Tooth_37, Tooth_38 };

		//使用循环将数组中的ToothId存入vector容器中
		for (int i = 0; i < sizeof(ToothId_Lower); i++)
		{

			m_ToothId.push_back(ToothId_Lower[i]);

		}

	}
	else
	{
		//创建临时数组，存放双颌牙齿的ToothId
		ToothId  ToothId_Double[32] = { Tooth_18 ,Tooth_17, Tooth_16, Tooth_15, Tooth_14, Tooth_13, Tooth_12, Tooth_11,
										Tooth_21, Tooth_22, Tooth_23, Tooth_24, Tooth_25, Tooth_26, Tooth_27, Tooth_28,
										Tooth_38, Tooth_37, Tooth_36, Tooth_35, Tooth_34, Tooth_33, Tooth_32, Tooth_31,
										Tooth_41, Tooth_42, Tooth_43, Tooth_44, Tooth_45, Tooth_46, Tooth_47, Tooth_48 };

		//使用循环将数组中的ToothId存入vector容器中
		for (int i = 0; i < sizeof(ToothId_Double); i++)
		{

			m_ToothId.push_back(ToothId_Double[i]);

		}
	}
	
	return m_ToothId;


}

bool OrthoData::Is_Tooth(vtkSmartPointer<vtkActor> actor) {//判断是否是牙齿
	for (int i = 1; i <= 16; i++) {
		Tooth* tooth_Upper = m_Arch[Arch_Upper]->Get_Tooth((ToothId)i);
		if (tooth_Upper == NULL) continue;
		if (tooth_Upper->GetToothPolyData() == actor->GetMapper()->GetInput())//只能用vtkPolyData型数据判断，不能用vtkActor型数据判断
		{
			return true;
			break;
		}
	}
	for (int i = 17; i <= 31; i++) {
		Tooth* tooth_Lower = m_Arch[Arch_Lower]->Get_Tooth((ToothId)i);
		if (tooth_Lower == NULL) continue;
		if (tooth_Lower->GetToothPolyData() == actor->GetMapper()->GetInput())
		{
			return true;
			break;
		}
	}
	return false;
}
bool OrthoData::Is_Gam(vtkSmartPointer<vtkActor> actor) {//判断是否是牙龈
	Gum* jaw_Upper = m_Arch[Arch_Upper]->Get_Gum();
	if (jaw_Upper->GetGumPolyData() == actor->GetMapper()->GetInput()) {
		return true;
	}

	Gum* jaw_Lower = m_Arch[Arch_Lower]->Get_Gum();
	if (jaw_Lower->GetGumPolyData() == actor->GetMapper()->GetInput()) {
		return true;
	}
	return false;
}

void OrthoData::SetMap(ToothId i,int j, std::vector<double> a) {
	if (j == 1)  AxesX[i] = a;
	if (j == 2)  AxesY[i] = a;
	if (j == 3)  AxesZ[i] = a;
}

bool OrthoData::GetMap(ToothId i, int j,std::vector<double>&axesx) {
	if (j == 1) {
		if (AxesX.find(i) != AxesX.end())
		{
			axesx = AxesX[i];
			return  true;
		}
	}
	if (j == 2) {
		if (AxesY.find(i) != AxesY.end())
		{
			axesx = AxesY[i];
			return  true;
		}
	}
	if (j == 3) {
		if (AxesZ.find(i) != AxesZ.end())
		{
			axesx = AxesZ[i];
			return  true;
		}
	}
	return false;
}


 Tooth* OrthoData:: GetTooth(ToothId id)
{
	 if (id >= 1 && id <= 16) {
		 return  m_Arch[Arch_Upper]->Get_Tooth(id);
	 }
	 if (id >= 17 && id <= 32) {
		 return  m_Arch[Arch_Lower]->Get_Tooth(id);
	 }
	 return NULL;
}

 void OrthoData::SetPSOGbestx(ToothId i, std::vector<std::vector<double>> x) {
	 m_pso_gbestx[i] = x;
 }
 bool OrthoData::GetPSOGbestx(ToothId i, std::vector<std::vector<double>>& gbestx) {
	 if (m_pso_gbestx.find(i) != m_pso_gbestx.end())
	 {
		 gbestx = m_pso_gbestx[i];
		 return  true;
	 }
	 return false;
 }

 void OrthoData::SetCenterBefore(ToothId i, std::vector<double> cen) {
	 toothcenter_before[i] = cen;
 }
 bool OrthoData::GetCenterBefore(ToothId i, std::vector<double>& center) {
	 if (toothcenter_before.find(i) != toothcenter_before.end())
	 {
		 center = toothcenter_before[i];
		 return  true;
	 }
	 return false;
 }

 void OrthoData::SetCenterAfter(ToothId i, std::vector<double> cen) {
	 toothcenter_after[i] = cen;
 }
 bool OrthoData::GetCenterAfter(ToothId i, std::vector<double>& center) {
	 if (toothcenter_after.find(i) != toothcenter_after.end())
	 {
		 center = toothcenter_after[i];
		 return  true;
	 }
	 return false;
 }

 void OrthoData::SetPathToothActor(int i, std::vector<vtkSmartPointer<vtkActor>> init) {
	 if(i = 1)  InitToothActor = init;
	 if(i = 2)  TargetToothActor = init;
 }
 bool OrthoData::GetPathToothActor(int i, std::vector<vtkSmartPointer<vtkActor>>& init) {
	 if (i = 1)   init = InitToothActor;
	 if (i = 2)   init = TargetToothActor;
	 return true;
 }