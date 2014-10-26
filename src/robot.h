#include "header.h"

#include <iostream>
#include <cstdio>
#include <cstring>
#include <ctime>

int init();
void release();
int main(int argc, char** argv );

class robotActionNode
{
public:
	int lastTime;
	string data;
};

class robotAction
{
	list<robotActionNode> nodeList;
	list<robotActionNode>::iterator it;
	bool isActive;
	bool isAutoCycle;//是否自动循环
public :
	robotAction(string);
	void update();//状态更新，在住循环中调用
	bool getIsActive();//获得当前动作组是否在动作
	bool getIsAutoCycle();
	void setIsAutoCycle(bool);
	void reset();//重置动作组
	void active();//开始动作
};
