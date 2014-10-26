//common header file
#ifdef  __INCLUDED__
	
#else
	#include <stdio.h>
	#include <string.h>
	#include <errno.h>
	#include <map>
	#include <list>
	#include <string>
	#include "tinyxml.h"
	#define __INCLUDED__
	#define SHOW_GUI
	#define __DEBUG__
	//#define CAPTURE_FROM_WEBCAM

	#define ACTION_STAND 0
	#define ACTION_FIRST_STEP 1
	#define ACTION_WALK 2
	#define ACTION_LEFT 3
	#define ACTION_RIGHT 4
	#define ACTION_STOP 5

	using namespace std;
#endif

