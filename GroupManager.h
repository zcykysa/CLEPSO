#pragma once
 
#include <vtkSmartPointer.h>
#include <map>
#include <vector>
#include <list>

enum GroupType : unsigned int;
enum MouseEvent : int;
enum ArchType : int;
class OrthoData;
class GroupBase;
class RendererContainer;

class GroupManager
{
public:
	static GroupManager* GetInstance();
	GroupBase* GetGroup(const GroupType&type);

	void OpenGroup(const GroupType& type);//打开功能
	void CloseGroup(const GroupType& type);//关闭功能

	void OnMousePress(MouseEvent  type);//鼠标单击
	void OnMouseRelease(MouseEvent  type);//鼠标释放
	void OnMouseMove();//鼠标移动
	bool OnMousePressMove(MouseEvent type);//鼠标按下移动
	void OnMouseDoublePress(MouseEvent  type);//鼠标双击
	void OnKeyDown();//键盘按下
	void OnKeyUp();//键盘释放
	bool OnMouseWheelRolling(bool isForward);//鼠标滚轮
	void RegisterGenerator(GroupBase* group);
//	void SetRenderWindowManager(RenderWindowManager* rwm);

	void SetOrthoData(OrthoData* orthoData);//调用牙齿数据

	void Render();
	void AddGroup(GroupBase* group);//插入新的牙齿数据并重新建立优先级列表

	void SetRenderContainer(RendererContainer* rendererContainer);

private:
	void CreateGroupListBaseOnPriority();//新建牙齿优先级列表

private:
	GroupManager();
	~GroupManager();

	std::map<GroupType, GroupBase*> mGroupList;
	std::vector<GroupBase*> mGroupListBaseOnPriority;
	OrthoData* mOrthoData;
	RendererContainer* m_RenderContainer;
};

