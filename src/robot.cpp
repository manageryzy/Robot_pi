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
	shouldExit = false,
	captureFinished = false;

confReader * RobotConfReader;

string ImageFileName;
int ImageFilterSize=3;

int Line1X,LineFindingError;
int line1=0;//两条检测线的结果
int MaxLine1,MinLine1;

TiXmlDocument
	robotStand,//站立
	robotGo,//第一步
	robotWalk,//行走步态
	robotLeft,//左转步态
	robotRight,//右转步态
	robotStop;//停止步态

robotAction * actions[9];
int nowAction;
queue<int> actionQueue;


int loadStep(int index,const char * conf_name)
{
	string filename;
	printf("loading step file '%s'\n",conf_name);
	filename = RobotConfReader->getConf(conf_name);
	if(filename=="null")
	{
		printf("error in getting %s step\n",conf_name);
		return 1;
	}
	actions[index]=new robotAction(filename);
	printf("step file :%s loaded!",conf_name);
	return 0;
}

/**
 * 初始化
 */
int init()
{
	//----------------------------
	//读取配置文件
	RobotConfReader = new confReader("./conf/robot.conf");

#ifndef CAPTURE_FROM_WEBCAM
	ImageFileName = RobotConfReader->getConf("image_location");
	if(ImageFileName=="null")
	{
		puts("error in getting image_location!I Quit!");
		return 1;
	}
#endif

	//设置高斯滤波大小
	ImageFilterSize = std::atoi( RobotConfReader->getConf("image_filter_size").c_str());
	if(RobotConfReader->getConf("image_filter_size")=="null")
	{
		puts("error in getting image_filter_size");
		ImageFilterSize=3;
	}

	//读取直线1的位置
	Line1X = std::atoi( RobotConfReader->getConf("line1").c_str());
	if(RobotConfReader->getConf("line1")=="null")
	{
		puts("error in getting line1");
		return 1;
	}
	MaxLine1 = std::atoi( RobotConfReader->getConf("maxline1").c_str());
	if(RobotConfReader->getConf("maxline1")=="null")
	{
		puts("error in getting maxline1");
		return 1;
	}
	MinLine1 = std::atoi( RobotConfReader->getConf("minline1").c_str());
	if(RobotConfReader->getConf("minline1")=="null")
	{
		puts("error in getting minline1");
		return 1;
	}



	//读取巡线误差
	LineFindingError = std::atoi( RobotConfReader->getConf("lineError").c_str());
	if(RobotConfReader->getConf("lineError")=="null")
	{
		puts("error in getting lineError");
		LineFindingError = 10;
	}

	//------------------------------
	//读取步态文件
	puts("------------------------------");
	puts("now loading step file!\n");

	if(loadStep(ACTION_STAND_STAND,"stand")!=0)
	{
		puts("error in read step!Quit");
		return 1;
	}

	if(loadStep(ACTION_STAND_TO_LEFT,"standToLeft")!=0)
	{
		puts("error in read step!Quit");
		return 1;
	}

	if(loadStep(ACTION_STAND_TO_RIGHT,"standToRight")!=0)
	{
		puts("error in read step!Quit");
		return 1;
	}

	if(loadStep(ACTION_WALK_LEFT,"walkLeft")!=0)
	{
		puts("error in read step!Quit");
		return 1;
	}

	if(loadStep(ACTION_WALK_RIGHT,"walkRight")!=0)
	{
		puts("error in read step!Quit");
		return 1;
	}

	if(loadStep(ACTION_LEFT_TO_STAND,"leftToStand")!=0)
	{
		puts("error in read step!Quit");
		return 1;
	}

	if(loadStep(ACTION_RIGHT_TO_STAND,"rightToStand")!=0)
	{
		puts("error in read step!Quit");
		return 1;
	}

	if(loadStep(ACTION_TURN_LEFT,"turnLeft")!=0)
	{
		puts("error in read step!Quit");
		return 1;
	}

	if(loadStep(ACTION_TURN_RIGHT,"turnRight")!=0)
	{
		puts("error in read step!Quit");
		return 1;
	}

	puts("\nAll the step file has been loaded successfully!\n");



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
	nowAction = ACTION_STAND_STAND;
	actions[nowAction]->active();

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

				printf("ostu = %d\n",autoOstu);
				if(autoOstu > 110)
				{
					puts("pure white!");
					if(actionQueue.empty())
					{
						switch(getStatue())
						{
						case STATUE_STAND:
							actionQueue.push(ACTION_TURN_RIGHT);
							break;
						case STATUE_LEFT_AHEAD:
							actionQueue.push(ACTION_LEFT_TO_STAND);
							actionQueue.push(ACTION_TURN_RIGHT);
							break;
						case STATUE_RIGHT_AHEAD:
							actionQueue.push(ACTION_RIGHT_TO_STAND);
							actionQueue.push(ACTION_TURN_RIGHT);
							break;
						default:
							puts("ERROR!UNDEF STATUE!");
							actionQueue.push(ACTION_STAND_STAND);
							actionQueue.push(ACTION_TURN_RIGHT);
							break;
						}
					}
					autoOstu = 70;
				}
				cvThreshold(img_smooth,img_twovalue,autoOstu,150,CV_THRESH_BINARY);
				#ifdef __DEBUG__
					printf("cvThreshold(img_smooth,img_twovalue,%d,150,CV_THRESH_BINARY);\n",autoOstu);
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
				line1 = 1024;
				line1 = findBlackLine(img_twovalue,img_canny,img,Line1X,LineFindingError);
				cvCircle(img,cvPoint(Line1X,line1),10,cvScalar(255,0,0,0.5));
			}

			//-----------------------------
			//显示图像
			#ifdef SHOW_GUI
				#ifdef __DEBUG__
					puts("show captured!");
				#endif
				cvShowImage ("capture", img);
			#endif

			//-----------------------------
			//垃圾回收
			cvReleaseImage(&img);
			cvReleaseImage(&img_gray);
			cvReleaseImage(&img_smooth);
			cvReleaseImage(&img_twovalue);
			cvReleaseImage(&img_canny);

			captureFinished = true;
		}

		//延迟10ms
		usleep(10000);

		//更新步态
		actions[nowAction]->update();
		if(!actions[nowAction]->getIsActive())//查询步态是否完成
		{
			puts("step finish!");
			if(actionQueue.empty())
			{
				puts("new action group");
				if(captureFinished)
				{
					//进行步态的选择啦
					if(line1>=MaxLine1)//左转
					{
						switch(getStatue())
						{
						case STATUE_LEFT_AHEAD:
							actionQueue.push(ACTION_LEFT_TO_STAND);
							break;
						case STATUE_RIGHT_AHEAD:
							actionQueue.push(ACTION_RIGHT_TO_STAND);
							break;
						case STATUE_STAND:break;
						default:
							puts("UNKONW STATUE");
							actionQueue.push(ACTION_STAND_STAND);
							break;
						}
						actionQueue.push(ACTION_TURN_LEFT);
						actionQueue.push(ACTION_STAND_TO_LEFT);
					}
					else if(line1<MinLine1)//右转
					{
						switch(getStatue())
						{
						case STATUE_STAND:
							actionQueue.push(ACTION_TURN_RIGHT);
							break;
						case STATUE_LEFT_AHEAD:
							actionQueue.push(ACTION_LEFT_TO_STAND);
							actionQueue.push(ACTION_TURN_RIGHT);
							break;
						case STATUE_RIGHT_AHEAD:
							actionQueue.push(ACTION_RIGHT_TO_STAND);
							actionQueue.push(ACTION_TURN_RIGHT);
							break;
						default:
							puts("ERROR!UNDEF STATUE!");
							actionQueue.push(ACTION_STAND_STAND);
							actionQueue.push(ACTION_TURN_RIGHT);
							break;
						}
						actionQueue.push(ACTION_STAND_TO_RIGHT);
					}
					else
					{
						//根据当前状态继续步态
						switch(getStatue())
						{
						case STATUE_LEFT_AHEAD:
							nowAction = ACTION_WALK_RIGHT;
							actions[nowAction]->active();
							break;
						case STATUE_RIGHT_AHEAD:
							nowAction = ACTION_WALK_LEFT;
							actions[nowAction]->active();
							break;
						case STATUE_STAND:
							nowAction = ACTION_STAND_TO_LEFT;
							actions[nowAction]->active();
							break;
						default:
							nowAction = ACTION_STAND_STAND;
							actions[nowAction]->active();
							break;
						}
					}
				}
				else
				{
					shouldCaptule = true;
					captureFinished = false;
				}
			}
			else//指令队列还有指令，等待指令结束
			{
				puts("action group now finish!");
				nowAction = actionQueue.front();
				actionQueue.pop();
				actions[nowAction]->active();
			}
			printf("%d\n",nowAction);
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
	this->reset();
	this->isActive = true;
	this->ActiveTime = clock();
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
			string timeStr;
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
			timeStr = nowNode->ToElement()->GetText();

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
			node.data.append(1,' ');
			node.data.append(timeStr);
			node.data.append(1,'\r');
			node.data.append(1,'\n');

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
		if(this->it == this->nodeList.end())//干掉步态动作组完成之后的那个等待
		{
			shouldCaptule = true;//在步态完成之后再主循环的下一次循环的时候捕获图片
			if(this->isAutoCycle)
			{
				it = this->nodeList.begin();
			}

			isActive = false;
			return ;
		}
		if((clock()-this->ActiveTime) > ((long)this->it->lastTime*200))
		{

			if(++this->it == this->nodeList.end())//步态完成
			{
				shouldCaptule = true;//在步态完成之后再主循环的下一次循环的时候捕获图片
				if(this->isAutoCycle)
				{
					it = this->nodeList.begin();
				}

				isActive = false;
				return ;
			}
			this->isActived = false; // 脑残无药医也
		}
	}
	else
	{
		serialSendString(this->it->data.c_str());
		this->isActived = true;
		this->ActiveTime = clock();
	}
}

void robotAction::stop()
{
	this->isActive = false;
}


int getStatue()
{
	switch(nowAction)
	{
	case ACTION_LEFT_TO_STAND:
	case ACTION_RIGHT_TO_STAND:
	case ACTION_STAND_STAND:
	case ACTION_TURN_LEFT:
	case ACTION_TURN_RIGHT:
		return STATUE_STAND;
	case ACTION_STAND_TO_LEFT:
	case ACTION_WALK_LEFT:
		return STATUE_LEFT_AHEAD;
	case ACTION_STAND_TO_RIGHT:
	case ACTION_WALK_RIGHT:
		return STATUE_RIGHT_AHEAD;
	}
	return STATUE_UNDEF;
}
