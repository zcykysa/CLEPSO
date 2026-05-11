/******************************************************************************

版权所有 (C), 2023-2025, 西安科技大学

******************************************************************************
文 件 名   :Common.h
生成日期   :2023年2月17日
最近修改   :
功能描述   :公共数据类
函数列表   :
修改历史   :
1.日   期  :2023年2月17日
作     者  :刘曼
修改内容   :
******************************************************************************/

#pragma once

enum  ArchType:int
{ 
	Arch_null=-1,
	Arch_Upper,   //上颌
	Arch_Lower,   //下颌
	Arch_Double   //双颌
};

enum  ToothId:int   //FDI牙齿编号
{
	Tooth_NULL=0,
	Tooth_18=1,	Tooth_17, Tooth_16, Tooth_15, Tooth_14, Tooth_13, Tooth_12, Tooth_11,
	Tooth_21, Tooth_22, Tooth_23, Tooth_24, Tooth_25, Tooth_26, Tooth_27, Tooth_28,
	Tooth_38, Tooth_37, Tooth_36, Tooth_35, Tooth_34, Tooth_33, Tooth_32, Tooth_31,
	Tooth_41, Tooth_42, Tooth_43, Tooth_44, Tooth_45, Tooth_46, Tooth_47, Tooth_48,
};

enum  MouseEvent :int
{
	Mouse_NULL=0, 
	Mouse_Left,
	Mouse_Right,
	Mouse_Middle,
};

enum RendererId : int   //渲染场景
{
	Renderer_NULL = 0,
	MainRenderer = 1,
	FirstRenderer = 2,
	SecondRenderer=3,
};