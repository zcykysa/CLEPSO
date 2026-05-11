/******************************************************************************

版权所有 (C), 2023-2025, 西安科技大学

******************************************************************************
文 件 名   :MainWindow.h
生成日期   :2023年2月17日
最近修改   :
功能描述   :主窗口类
函数列表   :
修改历史   :
1.日   期  :2023年2月17日
作     者  :刘曼
修改内容   :
******************************************************************************/

#pragma once
using namespace std;
#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSmartPointer.h>
#include "GroupFdiMark.h"
#include "Common.h"
#include <map>
#include <vector>
#include <QPen>
#include <QPainter>
#include <QPaintEvent>

class QPushButton;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class OpenGLWidget;
class OrthoData;
class GroupModelObservation;
class GroupFdiMark;
class Renderer;
class Common;
class GroupManager;
class GroupContrast;
class RendererContainer;
class GroupMoveCalculation;
class GroupPathPlanningPSO;
class GroupGridLine;
class MyGLWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    void WidgetsOn();

    void view1();
    void OnLeftButtonDownS();
    //void mouseReleaseEvent(QMouseEvent* ev);
    //void mouseDoubleClickEvent(QMouseEvent* ev);
    void curve();
    void OBB();
    Ui::MainWindow ui;

 private:
 
    QSharedPointer<RendererContainer> m_RendererContainer; //设置(Renderer, RenderWindow, RenderWindowInteractor)
    QSharedPointer<OrthoData> my_OrthoData;
    GroupManager* m_GroupManager;
    MyGLWidget* gridline2;
 


private:
    void Init();
    void InitGroup();
    void SetThreeCoordinate();  //设置左下角的世界坐标系

private slots:
    void LoadData();
    void M_HideLowerTeeth();  //隐藏下颌
    void M_ShowLowerTeeth();  //显示下颌
    void M_HideUpperTeeth();  //隐藏上颌
    void M_ShowUpperTeeth();  //显示上颌

    void M_ViewUpperTeeth(); //上颌
    void M_ViewUpperSurface(); //上颌颌面
    void M_ViewLeft();   //左侧
    void M_ViewRight();  //右侧
    void M_ViewBack();   //后视
    void M_ViewFront(); //正面
    void M_ViewLowerSurface(); //下颌颌面
    void M_ViewLowerTeeth(); //下颌

    void MFDI_Mark(); //牙位标记
    void OnContrast();//对照
    void OnMoveCalculation();//平移量计算

    //void Animation();//动画

    void Occlusion_Show();//咬合显示
    
    void GridLine_Show();//网格显示
    
    
   
    void Path_IPSOSA_Ours();//牙齿路径规划—路径规划
    void Path_PSO();
    void Path_NSMPSO_Xu();
    void Path_IPSOSA();
    void Path_CSPSO();
    void Path_MOPSO();
    void Path_MDC_DPSO();
    void Path_MCMOPSO();
  

    ////可视化7个阶段（7个中间路径点）
    void PSO_Phase1();//可视化阶段1——起始点
    void PSO_Phase3();//
    void PSO_Phase5();//
    void PSO_Phase7();//
    void PSO_Phase9();//
    void PSO_Phase11();//
    void PSO_Phase13();//
    void PSO_Phase15();//
    void PSO_Phase17();//
    void PSO_Phase19();//
    void PSO_Phase21();//路径25——终点

    void PSO_Phase2();//可视化阶段1——起始点
    void PSO_Phase4();//
    void PSO_Phase6();//
    void PSO_Phase8();//
    void PSO_Phase10();//
    void PSO_Phase12();//
    void PSO_Phase14();//
    void PSO_Phase16();//
    void PSO_Phase18();//
    void PSO_Phase20();//
    void PSO_Phase22();//
    void PSO_Phase23();//
    void PSO_Phase24();//
    void PSO_Phase25();//

    
};
