#include "GroupManager.h"
#include "OrthoData.h"
#include "GroupBase.h"
#include "Common.h"
#include <iostream>
#include <algorithm>
#include "RendererContainer.h"
#include "vtkActorCollection.h"

GroupManager::GroupManager()
{
}

GroupManager::~GroupManager()
{
	auto iter = mGroupList.begin();
	while (iter != mGroupList.end())
	{
		GroupBase* fb = iter->second;
		iter = mGroupList.erase(iter); //删除iter迭代器所指的元素，同时返回下一个元素的迭代器
		delete fb;
		fb = nullptr;
	}
}

GroupManager* GroupManager::GetInstance()
{
	static GroupManager groupManager;
	return &groupManager;
}

GroupBase* GroupManager::GetGroup(const GroupType& type) //获取某个功能块
{
	GroupBase* result = nullptr;
	auto iter = mGroupList.find(type);  //寻找键为type对应的value值，若存在则返回该键对应value值的迭代器
	if (iter != mGroupList.end()) //不为空的情况下
	{
		result = iter->second;   //找到则将value值赋值给数据类型为GroupBase的result指针变量
	}
	
	return result;
}

void GroupManager::OpenGroup(const GroupType& type)//打开功能
{
	GroupBase* group = this->GetGroup(type);
	if (group == nullptr)
	{
		return;
	}

	group->OpenGroup();
}

void GroupManager::CloseGroup(const GroupType& type)//关闭功能
{
	GroupBase* group = this->GetGroup(type);
	if (group == nullptr)
	{
		return;
	}

	group->CloseGroup();
}

void GroupManager::OnMousePress(MouseEvent  e)//鼠标按下
{
	for (size_t i = 0; i < mGroupListBaseOnPriority.size(); ++i)
	{
		GroupBase* fb = mGroupListBaseOnPriority[i];
		if (fb->GetIsGroupOpen() && fb->OnMousePress(e))
		{
			return;
		}
	}
}
void GroupManager::OnMouseDoublePress(MouseEvent e)//鼠标双击
{
	for (size_t i = 0; i < mGroupListBaseOnPriority.size(); ++i)
	{
		GroupBase* fb = mGroupListBaseOnPriority[i];
		if (fb->GetIsGroupOpen() && fb->OnMouseDoublePress(e))
		{
			
			return;
		}
	}

}
void GroupManager::OnMouseRelease(MouseEvent e)//鼠标释放
{
	for (size_t i = 0; i < mGroupListBaseOnPriority.size(); ++i)
	{
		GroupBase* fb = mGroupListBaseOnPriority[i];
		if (fb->GetIsGroupOpen() && fb->OnMouseRelease(e))
		{
			return;
		}
	}
}

void GroupManager::OnKeyDown()//键盘按下
{
	for (size_t i = 0; i < mGroupListBaseOnPriority.size(); ++i)
	{
		GroupBase* fb = mGroupListBaseOnPriority[i];
		if (fb->GetIsGroupOpen() && fb->OnKeyPress())
		{
			return;
		}
	}
}

void GroupManager::OnKeyUp()//键盘释放
{
	for (size_t i = 0; i < mGroupListBaseOnPriority.size(); ++i)
	{
		GroupBase* fb = mGroupListBaseOnPriority[i];
		if (fb->GetIsGroupOpen() && fb->OnKeyRelease())
		{
			return;
		}
	}	
}
void GroupManager::OnMouseMove()//鼠标移动
{
	for (size_t i = 0; i < mGroupListBaseOnPriority.size(); ++i)
	{
		GroupBase* fb = mGroupListBaseOnPriority[i];
		if (fb->GetIsGroupOpen() && fb->OnMouseMove())
		{
			return;
		}
	}
}
bool GroupManager::OnMousePressMove(MouseEvent e)//鼠标按下移动
{
	for (size_t i = 0; i < mGroupListBaseOnPriority.size(); ++i)
	{
		GroupBase* fb = mGroupListBaseOnPriority[i];
		if (fb->GetIsGroupOpen() && fb->OnMousePressMove(e))
		{
			return true;
		}
	}
	return false;
}


bool GroupManager::OnMouseWheelRolling(bool isForward)//鼠标滚轮
{
	for (size_t i = 0; i < mGroupListBaseOnPriority.size(); ++i)
	{
		GroupBase* fb = mGroupListBaseOnPriority[i];
		if (fb->GetIsGroupOpen() && fb->OnMouseWheelRolling(isForward))
		{
			return false;
		}
	}
}


void GroupManager::SetRenderContainer(RendererContainer* rendererContainer)
{
	m_RenderContainer = rendererContainer;
	auto iter = mGroupList.begin();
	while (iter != mGroupList.end())
	{
		GroupBase* fb = iter->second;
		fb->SetGroupManager(this);
		fb->SetRenderContainer(rendererContainer);
		iter++;
	}
}

void GroupManager::SetOrthoData(OrthoData* orthoData)//调用牙齿数据
{
	mOrthoData = orthoData;
	auto iter = mGroupList.begin();
	while (iter != mGroupList.end())
	{
		GroupBase* fb = iter->second;
		fb->SetGroupManager(this);
		fb->SetOrthoData(orthoData);
		iter++;
	}
}

void GroupManager::Render()
{
	m_RenderContainer->Render();
}

void GroupManager::AddGroup(GroupBase* group)//插入新的牙齿数据并重新建立优先级列表
{
	mGroupList.insert(std::pair<GroupType, GroupBase*>(group->GetGroupType(), group));//插入新的牙齿数据
	group->SetOrthoData(mOrthoData);
	group->SetRenderContainer(m_RenderContainer);
	CreateGroupListBaseOnPriority();//新建优先级列表
}

void GroupManager::CreateGroupListBaseOnPriority()//新建优先级列表
{
	mGroupListBaseOnPriority.clear();//将当前列表数据清空

	auto iter = mGroupList.begin();
	while (iter != mGroupList.end())//将新元素逐步添加到列表中
	{
		mGroupListBaseOnPriority.push_back(iter->second);
		iter++;
	}

	sort(mGroupListBaseOnPriority.begin(), mGroupListBaseOnPriority.end(), [](GroupBase* left, GroupBase* right)->bool {
		return left->GetGroupPriority() < right->GetGroupPriority();
		});
}