#include "vtkSmartPointer.h"
#include "GroupBase.h"
#include "vtkActor.h"
#include "OrthoData.h"
#include "Arch.h"
#include "Common.h"
#include "MainWindow.h"
#include <iostream>
#include <vector>
#include"time.h"
#include"math.h"
#include <stdio.h>
#include "vtkProperty.h"
#include "RendererContainer.h"
#include "QSharedPointer.h"
#include <fstream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <map>
#include <MainWindow.h>




using namespace std;

class RendererContainer;
class OrthoData;
class MainWindow;

//初始化参数
//上颌2-15
//下颌18-31
const int z = 40;//粒子个数20-----const恒定变量值——种群数.  初始，1，5，9，13，17，21，25
const int iter_max = 5;//最大迭代次数30
const int m = 32;//牙齿id
const int n = 25;//牙齿矫正中间20个路径点（21个阶段）
const double PI = 3.141592653;
const double MAX_STEP_TRANS = 0.3; // 单步最大平移（论文2.4.1节）
const double MAX_STEP_ROT = 2.0;   // 单步最大旋转（论文2.4.1节）
const double k = 0.5;
const double k_cos = 0.3;          // 余弦扰动系数（全局探索）
const double k_sin = 0.1;          // 正弦扰动系数（局部微调）




class GroupPathPlanningPSO : public GroupBase
{
public:
	GroupPathPlanningPSO(GroupType type); //设置无参构造函数
	~GroupPathPlanningPSO();
	virtual void OpenGroup() override; //实现父类中的纯虚函数（virtual 可以省略）
	virtual void CloseGroup() override;


	double pbestf_it;//全局最优适应度
	double gbestf_it;//全局最优适应度
	double pbestf_it_upper;//全局最优适应度
	double pbestf_it_lower;//全局最优适应度
	double gbestf_it_upper;//全局最优适应度
	double gbestf_it_lower;//全局最优适应度

	double before[m][6];//初始位牙齿编码
	double after[m][6];//目标位牙齿编码
	double gbestx[m][n + 1][6];//全局最优位置（包括最后理想位置阶段）
	double gbestf;//全局最优适应度
	double Z[400];// 均值为0，方差为1的正态分布
	double Y[400];// 均值为1，方差为4的正态分布
	double D[z + 1];//基于维度的全局最优解更新，记录每个粒子的pbest，之后排序以便使用

	//CSPSO
	double  Pbestx_sum[m][n + 1][6];;
	double  S[m][n + 1][6];;//Pbestx_sum平均值

	struct particle {	//定义一个粒子
		double x[m][n + 1][6];//粒子位置（m个牙齿，n个阶段，每个牙齿xyz3坐标）
		double v[m][n + 1][6];//粒子速度（m个牙齿，n个阶段，每个牙齿xyz3坐标）
		double bestx[m][n + 1][6];//当前粒子历史最优位置
		double fitness;//当前粒子适应度
		double bestf;//历史最优适应度
		double fx;//路径长度
		int Stagnation;            // 停滞计数器（新增）
	}swarm[z];//定义粒子群

	struct tempgbest {	//temp辅助求解全局最优解
		double Tempbestx[m][n + 1][6];
		double Tempgbestx[m][n + 1][6];
		double TT_bestx[20];
		double TT_bestx2[20];
		double TT_gestx[20];
		double TT_gestx2[20];
		double TT_gestf;
	}tp;
	enum SubgroupType { D1, D2, D3 };




	void TestBox();//OBBTree包围盒树
	void OBBTree_Occ();//OBBTree进行碰撞检测
	void GetEdgePoints(double bounds[6], int edgeIndex, double p0[3], double p1[3]);
	void Axes();//建立局部坐标系
	void Grow_AxesZ();
	void Test();//测试计算旋转分量
	void Test2();//测试计算旋转分量
	void Test3();//调整之后最终位置
	void Test4();//局部坐标系𬌗平面
	void RotateVYZValue(int id, double r11, double r12, double r13, double r21, double r22, double r23, double r31, double r32, double r33);//计算旋转量
	void RotateXYZeulerAngle();//计算旋转量
	void MatrixDecomposition();

	void PathPlan_Ours();//
	void PathPlan_Huxue();//粒子群算法路径规划--主实现函数
	void PathPlan_PSO();//对比1
	void PathPlan_CMPSO();//对比2
	void PathPlan_NSMPSO_Xu();//对比3
	void PathPlan_IPSOSA();//对比4

	void PathPlan_MOPSO();
	void PathPlan_MCMOPSO();

	void PathPlan_MDC_DPSO();

	void ReadMatrixFile();//获取理想位置Matrix矩阵
	void GetpositionMatrix();
	void ToothTarget();//测试最终位置数据
	void RotateData_data3();
	void RotateData_data4();
	void RotateData_data5();
	void RotateData_data7();
	void RotateData_data8();
	void ToothData();//读取牙齿数据并保存牙齿初始位编码数据和目标位编码数据
	void GenerateCosineSequence(double* seq, int length);
	void Init_ARLAPSO(); //初始化粒子群
	void Init_PSO();//初始化粒子群
	void Init_OurPSO();//初始化粒子群
	void Init_NSMPSO_Xu();//初始化粒子群

	void OnARLAPSO();//PSO路径规划算法
	void OnPSO();//PSO路径规划算法
	void OnNSMPSO();//PSO路径规划算法
	void OnCMPSO();
	void OnPSOSA();
	void OnOurPSO();

	void Orthodontic(particle* part, int it, double y);//阶段一每个牙齿每次循环迭代更新后的位置和速度
	void Orthodontic5(particle* part, int it, double y);//阶段一每个牙齿每次循环迭代更新后的位置和速度
	void OrthodonticPSO(particle* part, int it, double y);
	void OrthodonticCSPSO(particle* part, int it, double y);
	void OrthodonticNSMPSO(particle* part, int it, double y);

	//void OrthodonticXYZ(particle* part, int it, double y);
	void Orthodontic2(particle* part, int it);//阶段二每个牙齿每次循环迭代更新后的位置和速度
	double OBB(particle* part, int j);//碰撞检测约束条件
	double GbestpOBB(tempgbest* part, int j);//碰撞检测约束条件
	bool CollisionOn(vtkSmartPointer<vtkPolyData> pd1, vtkSmartPointer<vtkPolyData> pd2, int j, int k);//判断相邻牙齿是否发生碰撞
	double Move(particle* part, int j);//单个牙齿单个阶段移动量计算
	double Rotate(particle* part, int j);//单个牙齿单个阶段旋转量计算
	bool Constraint(particle* part, int j);//约束条件--移动量旋转量
	double Fitness(particle* part);//计算适应度函数（目标函数）
	double GbestFitness(tempgbest* part);

	int round_double(double number);
	double GbestMove(tempgbest* part);
	double GbestRotate(tempgbest* part);
	double GbestFit(tempgbest* part);
	double Pbest(particle* part);
	double Pbest_upper(particle* part);
	double Pbest_lower(particle* part);

	double GbestMove_upper(tempgbest* part);
	double GbestMove_upper1(tempgbest* part);
	double GbestMove_upper2(tempgbest* part);
	double GbestMove_upper3(tempgbest* part);
	double GbestRotate_upper(tempgbest* part);
	double GbestRotate_upper4(tempgbest* part);
	double GbestRotate_upper5(tempgbest* part);
	double GbestRotate_upper6(tempgbest* part);
	double GbestFit_upper(tempgbest* part);
	double GbestMove_lower(tempgbest* part);
	double GbestRotate_lower(tempgbest* part);
	double GbestFit_lower(tempgbest* part);


	void EnforceStepConstraints(particle* original, particle* trial, int j, int k);
	void LocalExploration(particle* p);
	void PathCompression(particle* p);
	void DEStrategy(particle* p, int iter);
	//基准函数

	double Dim = 20;//维度
	double maxgen = 500;//迭代次数50
	double sizepop = 50;//种群规模50
	struct test_f {	//temp辅助求解全局最优解;
		double test_v[20];
		double test_v2[20];
		double test_x[20];
		double test_x2[20];
		double test_fitness;
		double bestf;
		double bestx[20];
		double bestx2[20];
	}t[50];
	struct test222 {	//temp辅助求解全局最优解;
		double test_v[20];
		double test_x[20];
		double test_fitness;
		double bestf;
		double bestx[20];
	}t2;
	//初始化变量
	double ggbestf;
	double ggbestx[20];
	double ggbestx2[20];
	double TT[50];

	void Fitness_jizhun();
	//对比实验
	void Test_Ours();//本文所提算法
	void Test_PSO();//对比1
	void Test_CSPSO();//对比2
	void Test_NSMPSO();//对比4
	void Test_PSOSA();//对比4
	void Test_MOPSO();
	void Test_MCMOPSO();


	void Test_MDC_DPSO();
	//void Test_GA();

	double Test_jizhun(test_f* part);
	//Sphere函数
	double Sphere(test_f* part);
	double Sphere2(test222* part);
	double Quadric(test_f* part);
	double Rosenbrock(test_f* part);
	double Step(test_f* part);
	double Noise(test_f* part);
	double Rastrigin(test_f* part);
	double Zakharov(test_f* part);
	double Booth(test_f* part);
	double Schaffer(test_f* part);

	double Ackle(test_f* part);
	double Griewank(test_f* part);
	double Alpine(test_f* part);


	//可视化7个阶段（6个中间路径点）
	void OrthoPhase(int k);


	


private:
	std::map<int, Eigen::Matrix4f> Matrix; //用于存放初始位到目标位对应的牙齿Matrix变换矩阵

};


