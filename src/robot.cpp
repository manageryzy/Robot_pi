//#include "opencv2/opencv.hpp"
//#include "opencv2/contrib/contrib.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include "robot.h"
#include "serialOut.h"
#include "opencvCapture.h"
#include "confReader.h"
#include <pthread.h>
#include <unistd.h>

/////////////////////////////////////////////
//global flags
bool shouldCaptule = false,
	shouldExit = false;

string ImageFileName,robotStandFileName,robotGoFileName,robotWalkFileName,robotLeftFileName,robotRightFileName,robotStopFileName;
int ImageFilterSize=3;

TiXmlDocument
	robotStand,//站立
	robotGo,//第一步
	robotWalk,//行走步态
	robotLeft,//左转步态
	robotRight,//右转步态
	robotStop;//停止步态

robotAction * actions[6];
int nowAction;

/**
 * 初始化
 */
int init()
{
	//----------------------------
	//读取配置文件
	confReader RobotConfReader("./conf/robot.conf");

#ifndef CAPTURE_FROM_WEBCAM
	ImageFileName = RobotConfReader.getConf("image_location");
	if(ImageFileName=="null")
	{
		puts("error in getting image_location!I Quit!");
		return 1;
	}
#endif

	//设置高斯滤波大小
	ImageFilterSize = std::atoi( RobotConfReader.getConf("image_filter_size").c_str());
	if(RobotConfReader.getConf("image_filter_size")=="null")
	{
		puts("error in getting image_filter_size");
		ImageFilterSize=3;
	}

	//------------------------------
	//读取步态文件
	puts("------------------------------");
	puts("now loading step file!\n");

	//站立姿态
	puts("loading step file 'stand'");
	robotStandFileName = RobotConfReader.getConf("stand");
	if(robotStandFileName=="null")
	{
		puts("error in getting stand step");
		return 1;
	}
	actions[ACTION_STAND]=new robotAction(robotStandFileName);
	puts("step file :stand loaded!");

	//启动步伐
	puts("loading step file 'go'");
	robotGoFileName = RobotConfReader.getConf("go");
	if(robotGoFileName=="null")
	{
		puts("error in getting go step");
		return 1;
	}
	actions[ACTION_FIRST_STEP] = new robotAction(robotGoFileName);
	puts("step file :'go' loaded");

	//行走步伐
	puts("loading step file 'walk'");
	robotWalkFileName = RobotConfReader.getConf("walk");
	if(robotWalkFileName=="null")
	{
		puts("error in getting walk step");
		return 1;
	}
	actions[ACTION_WALK] = new robotAction(robotWalkFileName);
	puts("step file :'walk' loaded");

	//左转步态
	puts("loading step file 'left'");
	robotLeftFileName = RobotConfReader.getConf("left");
	if(robotLeftFileName=="null")
	{
		puts("error in getting left step");
		return 1;
	}
	actions[ACTION_LEFT] = new robotAction(robotLeftFileName);
	puts("step file :'left' loaded");

	//右转步态
	puts("loading step file 'right'");
	robotRightFileName = RobotConfReader.getConf("right");
	if(robotRightFileName=="null")
	{
		puts("error in getting right step");
		return 1;
	}
	actions[ACTION_RIGHT] = new robotAction(robotRightFileName);
	puts("step file :'right' loaded");

	//停止步态
	puts("loading step file 'stop'");
	robotStopFileName = RobotConfReader.getConf("stop");
	if(robotStopFileName=="null")
	{
		puts("error in getting stop step");
		return 1;
	}
	actions[ACTION_STOP] = new robotAction(robotStopFileName);
	puts("step file :'stop' loaded");
	puts("\nAll the step file has been loaded successfully!\n");

	nowAction = ACTION_STAND;

	//-----------------------------------
	//串口初始化
	if(SerialInit()!=0)
	{
		puts("error in init serial!Quit..");
		return 1;
	}

	//------------------------------------
	//opencv初始化

	//开启opencv串口线程，防止在多线程的时候出来各种各样的诡异的异常
	cvStartWindowThread();

	//初始化opencv捕获
	if(opencvCaptureInit())
	{
		puts("error in init opencv capture");
		return 2;
	}

	//根据编译开关打开窗口
#ifdef SHOW_GUI
	cvNamedWindow("capture", CV_WINDOW_AUTOSIZE);
	puts("created window capture");
#endif

	shouldCaptule = true;

	return 0;

}

/**
 * 垃圾回收释放资源
 */
void release()
{
	//关闭串口
	closeSerial();

	//停止捕获
	opencvCaptureClose();

	//关闭窗口
#ifdef SHOW_GUI
	cvDestroyWindow("capture");
	puts("destroied window capture");
#endif

}

int main(int argc, char** argv )
{

	IplImage *img, //原始图像
		*img_gray, //灰度图像
		*img_smooth,//滤波图像
		*img_twovalue,//二值化图像
		*img_canny; //canny 检测之后的图像

	if(init()!=0)
		exit(-1);

	puts("init finished!");

	while (1)
	{
		if(shouldCaptule)
		{
			shouldCaptule = false;

			#ifdef CAPTURE_FROM_WEBCAM
				img = myCaptureImage();
			#else
				puts(ImageFileName.c_str());
				img = cvLoadImage(ImageFileName.c_str(),-1);
			#endif

			#ifdef __DEBUG__
				puts("captured!");
			#endif

//			img_gray=cvCloneImage(img);
//			#ifdef __DEBUG__
//				puts("img_gray=cvCloneImage(img);");
//			#endif

			//--------------------------
			//图像灰度化
			{
				//创建灰度图像
				img_gray=cvCreateImage(cvGetSize(img),img->depth,1);
				#ifdef __DEBUG__
					puts("img_gray=cvCreateImage(cvGetSize(img),img->depth,1);");
				#endif

				//色彩空间转换
				cvCvtColor(img,img_gray,CV_RGB2GRAY);
				#ifdef __DEBUG__
					puts("cvCvtColor(img,img_gray,CV_RGB2GRAY);");
				#endif
			}

			//---------------------------
			//平滑滤波
			{
				//创建smooth的图像
				img_smooth=cvCreateImage(cvGetSize(img),img->depth,1);
				#ifdef __DEBUG__
					puts("img_smooth=cvCreateImage(cvGetSize(img),img->depth,1);");
				#endif

				//进行平滑滤波
				cvSmooth(img_gray,img_smooth,CV_GAUSSIAN,ImageFilterSize,0,0,0);
				#ifdef __DEBUG__
					puts("cvSmooth(img_gray,img_smooth);");
				#endif
			}

			//---------------------------
			//二值化
			{
				//创建二值化图像
				img_twovalue = cvCreateImage(cvGetSize(img),img->depth,1);
				#ifdef __DEBUG__
					puts("img_twovalue = cvCreateImage(cvGetSize(img),img->depth,1);");
				#endif

				int autoOstu = otsu(img_gray);
				#ifdef __DEBUG__
					puts("int autoOstu = otsu(img_gray);");
				#endif

				cvThreshold(img_smooth,img_twovalue,autoOstu,150,CV_THRESH_BINARY);
				#ifdef __DEBUG__
					puts("cvThreshold(img_smooth,img_twovalue,autoOstu,150,CV_THRESH_BINARY);");
				#endif

				//自带的自适应二值化方法，由于效果不好，放弃
//				cvAdaptiveThreshold(img_smooth,img_twovalue,255);
//				#ifdef __DEBUG__
//					puts("cvAdaptiveThreshold(img_smooth,img_twovalue,255);");
//				#endif
			}

			//----------------------------
			//canny 变换，用来和二值化图像一起使用来进行判断
			{
				//创建canny变换的图像
				img_canny = cvCreateImage(cvGetSize(img),img->depth,1);
				#ifdef __DEBUG__
					puts("img_canny =cvCreateImage(cvGetSize(img),img->depth,1);");
				#endif

				//进行变换
				cvCanny(img_gray,img_canny,30,50,3);
				#ifdef __DEBUG__
					puts("cvCanny(img_gray,img_canny,30,50,3);");
				#endif
			}

			//对二值图进行变换，找出有黑线的部分
			{
				findBlackLine(img_twovalue,img_canny,300,30);
			}

			//-----------------------------
			//显示图像
			#ifdef SHOW_GUI
				#ifdef __DEBUG__
					puts("show captured!");
				#endif
				cvShowImage ("capture", img_twovalue);
			#endif

			//-----------------------------
			//垃圾回收
			cvReleaseImage(&img);
			cvReleaseImage(&img_gray);
			cvReleaseImage(&img_smooth);
			cvReleaseImage(&img_twovalue);
			cvReleaseImage(&img_canny);
		}

		//延迟10ms
		usleep(10000);

		//更新步态
		actions[nowAction]->update();
		if(!actions[nowAction]->getIsActive())//查询步态是否完成
		{

		}

		if (cvWaitKey(1) == 27 || shouldExit)
			break;
	}

	puts("quit!");

	release();

	return 0;
}


void robotAction::active()
{
	this->isActive = true;
}

bool robotAction::getIsActive()
{
	return this->isActive;
}

bool robotAction::getIsAutoCycle()
{
	return this->isAutoCycle;
}

void robotAction::reset()
{
	this->it = this->nodeList.begin();
	isActived = false;
}

robotAction::robotAction(string doc)
{
	it = nodeList.begin();
	isActive = false;
	isAutoCycle = false;
	TiXmlDocument docDom;
	if(!docDom.LoadFile(doc.c_str()))
	{
		puts("error in reading xml file!");
	}

	TiXmlElement * root= docDom.RootElement();

	for( TiXmlNode*  item = root->FirstChild("Table1");
	         item;
	         item = item->NextSibling() ) {
		robotActionNode node;
		try{
			TiXmlNode * nowNode = item->FirstChild("Time");
			if(nowNode == NULL)
			{
				puts("error in getting time node of the step!Ignored!");
				continue;
			}
			if(nowNode->ToElement()->GetText()==0)
			{
				throw 0;
			}
			sscanf(nowNode->ToElement()->GetText()+1,"%d",&node.lastTime);

			nowNode = item->FirstChild("Move");
			if(nowNode == NULL)
			{
				puts("error in getting command node of the step!");
				continue;
			}
			if(nowNode->ToElement()->GetText()==0)
			{
				throw 0;
			}
			node.data = nowNode->ToElement()->GetText();

			nodeList.push_back(node);
		}
		catch (...) {
			puts("Something wrong in dealing xml!ignored!");
		}
	}

	isActived = false;
}

void robotAction::setIsAutoCycle(bool isAutoCycle)
{
	this->isAutoCycle = isAutoCycle;
}

void robotAction::update()
{
	if(!isActive)//非活动状态，退出程序
		return;
	if(this->isActived)//本条动作已经激活过了
	{
		if(clock()-this->ActiveTime > this->it->lastTime)
		{
			this->isActived = false;
			if(this->it == this->nodeList.end())//步态完成
			{
				shouldCaptule = true;//在步态完成之后再主循环的下一次循环的时候捕获图片
				if(this->isAutoCycle)
				{
					it = this->nodeList.begin();
				}
				else
				{
					isActive = false;
				}
			}
		}
	}
	else
	{
		serialSendString(this->it->data.c_str());
		this->isActived = true;
		this->ActiveTime = clock();
	}
}


