//common header file
#ifdef  __INCLUDED__
	//为什么我要这样愚蠢……
#else
	#include <stdio.h>
	#include <string.h>
	#include <errno.h>
	//STL
	#include <map>
	#include <list>
	#include <queue>
	#include <string>
	//xml解析库
	#include "tinyxml.h"
	#define __INCLUDED__

	//下面的是一些开关
	//#define __DEBUG__
	#ifdef __DEBUG__
		#define SHOW_GUI
		//#define __DEBUG_SERIAL__
		#define __DEBUG_ACTION_LOAD__
		#define __DEBUG_INIT__
		#define __DEBUG_IMG_PROC__
		//#define __DEBUG_IMG_PROC_CALL__
		#define __DEBUG_STEP__
	#endif

	#define CAPTURE_FROM_WEBCAM
	#define BLACK_LINE_SIGNAL 127 /*这个是在二值图中检测用的，请不要在二值图中自然的出现这个数值*/
	#define NO_LINE_SIGNAL 50
	#define NOT_LINE 255

	#define ACTION_STAND_STAND 0
	#define ACTION_STAND_TO_LEFT 1
	#define ACTION_STAND_TO_RIGHT 2
	#define ACTION_WALK_LEFT 3
	#define ACTION_WALK_RIGHT 4
	#define ACTION_LEFT_TO_STAND 5
	#define ACTION_RIGHT_TO_STAND 6
	#define ACTION_TURN_LEFT 7
	#define ACTION_TURN_RIGHT 8

	#define STATUE_UNDEF 0
	#define STATUE_STAND 1
	#define STATUE_LEFT_AHEAD 2
	#define STATUE_RIGHT_AHEAD 3

	using namespace std;
#endif

