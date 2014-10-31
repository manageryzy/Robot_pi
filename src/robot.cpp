//#include "opencv2/opencv.hpp"
//#include "opencv2/contrib/contrib.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include "robot.h"
#include "serialOut.h"
#include "opencvCapture.h"
#include "confReader.h"
#include <pthread.h>
#include <unistd.h>
#include <wiringPi.h>
#include <math.h>

/////////////////////////////////////////////
//global flags
bool shouldCaptule = false,
	shouldExit = false,
	captureFinished = false,
	tooClose = false;

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
bool isWhiteScreen = false;
int nowAction;
queue<int> actionQueue;

float lineK = 0;

int CalcDes(float x1,float y1,float x2,float y2,float x0,float y0)
{
	return abs((int)(((y2-y1)*(x0)/(x2-x1)-y0+y1-(y2-y1)*(x1)/(x2-x1))/sqrtf((y2-y1)/(x2-x1)*(y2-y1)/(x2-x1)+1)));
}

int pointDes(float x1,float y1,float x2,float y2)
{
	return (int)sqrtf((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

int minDes(float x1,float y1,float x2,float y2,float x0,float y0)
{
	int des1 = pointDes(x1,y1,x0,y0);
	int des2 = pointDes(x2,y2,x0,y0);
	int linePointDes = CalcDes(x1,y1,x2,y2,x0,y0);
	int minDes = des1<des2?des1:des2;
	if(minDes<linePointDes)
	{
		return 255;/*抛弃这个点*/
	}
	else
		return linePointDes;

}


int loadStep(int index,const char * conf_name)
{
	string filename;
	#ifdef __DEBUG_ACTION_LOAD__
		printf("loading step file '%s'\n",conf_name);
	#endif
	filename = RobotConfReader->getConf(conf_name);
	if(filename=="null")
	{
		printf("error in getting %s step\n",conf_name);
		return 1;
	}
	actions[index]=new robotAction(filename);
	#ifdef __DEBUG_ACTION_LOAD__
		printf("step file :%s loaded!\n",filename.c_str());
	#endif
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
	Line1X = std::atoi( RobotConfReader->getConf("line1").c_str())/4;
	if(RobotConfReader->getConf("line1")=="null")
	{
		puts("error in getting line1");
		return 1;
	}
	MaxLine1 = std::atoi( RobotConfReader->getConf("maxline1").c_str())/4;
	if(RobotConfReader->getConf("maxline1")=="null")
	{
		puts("error in getting maxline1");
		return 1;
	}
	MinLine1 = std::atoi( RobotConfReader->getConf("minline1").c_str())/4;
	if(RobotConfReader->getConf("minline1")=="null")
	{
		puts("error in getting minline1");
		return 1;
	}



	//读取巡线误差
	LineFindingError = std::atoi( RobotConfReader->getConf("lineError").c_str())/4;
	if(RobotConfReader->getConf("lineError")=="null")
	{
		puts("error in getting lineError");
		LineFindingError = 10;
	}

	//------------------------------
	//读取步态文件
	#ifdef __DEBUG_INIT__
		puts("------------------------------");
		puts("now loading step file!\n");
	#endif

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
	#ifdef __DEBUG_INIT__
		puts("\nAll the step file has been loaded successfully!\n");
	#endif


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
		cvNamedWindow("two", CV_WINDOW_AUTOSIZE);
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
		cvDestroyWindow("two");
		puts("destroied window capture");
	#endif

}

int main(int argc, char** argv )
{

	IplImage
		*img_raw,//缩放之前的图像
		*img, //原始图像
		*img_gray, //灰度图像
		*img_smooth,//滤波图像
		*img_twovalue,//二值化图像
		*img_canny; //canny 检测之后的图像
	CvSeq* lines = 0;
	CvMemStorage* storage = cvCreateMemStorage(0);

	if(init()!=0)
		exit(-1);

	#ifdef __DEBUG_INIT__
		puts("init finished!");
	#endif

	while(digitalRead(1)==0)//等待开关被触发
	{
		sleep(1);
	}

	while (1)
	{
		if(shouldCaptule)
		{
			shouldCaptule = false;

			#ifdef CAPTURE_FROM_WEBCAM
				img_raw = myCaptureImage();
			#else
				puts(ImageFileName.c_str());
				img_raw = cvLoadImage(ImageFileName.c_str(),-1);
			#endif

			img = cvCreateImage(cvSize(cvGetSize(img_raw).width/4,cvGetSize(img_raw).height/4),img_raw->depth,3);
			cvResize(img_raw,img);

			#ifdef __DEBUG_IMG_PROC__
				puts("pic captured!");
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
				#ifdef __DEBUG_IMG_PROC_CALL__
					puts("img_gray=cvCreateImage(cvGetSize(img),img->depth,1);");
				#endif

				//色彩空间转换
				cvCvtColor(img,img_gray,CV_RGB2GRAY);
				#ifdef __DEBUG_IMG_PROC_CALL__
					puts("cvCvtColor(img,img_gray,CV_RGB2GRAY);");
				#endif
			}

			//---------------------------
			//平滑滤波
			{
				//创建smooth的图像
				img_smooth=cvCreateImage(cvGetSize(img),img->depth,1);
				#ifdef __DEBUG_IMG_PROC_CALL__
					puts("img_smooth=cvCreateImage(cvGetSize(img),img->depth,1);");
				#endif

				//进行平滑滤波
				cvSmooth(img_gray,img_smooth,CV_GAUSSIAN,ImageFilterSize,0,0,0);
				#ifdef __DEBUG_IMG_PROC_CALL__
					puts("cvSmooth(img_gray,img_smooth);");
				#endif
			}

			//---------------------------
			//二值化
			{
				//创建二值化图像
				img_twovalue = cvCreateImage(cvGetSize(img),img->depth,1);
				#ifdef __DEBUG_IMG_PROC_CALL__
					puts("img_twovalue = cvCreateImage(cvGetSize(img),img->depth,1);");
				#endif

				int autoOstu ;
//				autoOstu = otsu(img_gray);
//				#ifdef __DEBUG_IMG_PROC_CALL__
//					puts("int autoOstu = otsu(img_gray);");
//				#endif
//
//				#ifdef __DEBUG_IMG_PROC__
//					printf("ostu = %d\n",autoOstu);
//				#endif

				autoOstu = 90;

//				if(autoOstu > 110)
//				{
//					#ifdef __DEBUG_IMG_PROC__
//						puts("pure white!");
//					#endif
//
//					isWhiteScreen = true;
//					autoOstu = 70;
//				}
//				else
//				{
//					isWhiteScreen = false;
//				}
				cvThreshold(img_smooth,img_twovalue,autoOstu,150,CV_THRESH_BINARY);
				#ifdef __DEBUG_IMG_PROC_CALL__
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
				#ifdef __DEBUG_IMG_PROC_CALL__
					puts("img_canny =cvCreateImage(cvGetSize(img),img->depth,1);");
				#endif

				//进行变换
				cvCanny(img_gray,img_canny,30,50,3);
				#ifdef __DEBUG_IMG_PROC_CALL__
					puts("cvCanny(img_gray,img_canny,30,50,3);");
				#endif
			}

			//对二值图进行变换，找出有黑线的部分
			{
				line1 = 1024;
				line1 = findBlackLine(img_twovalue,img_canny,img,Line1X,LineFindingError);
				cvCircle(img,cvPoint(Line1X,line1),10,cvScalar(255,0,0,0.5));
			}

			lines = cvHoughLines2( img_canny, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 50, 50, 10 );
			lineK = 0;
			for(int i = 0; i < lines->total; i++ )
			{
				CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
				if(line[0].x!=line[1].x)
				{
					if(minDes(line[0].x,line[0].y,line[1].x,line[1].y,Line1X,line1)<10)
					{
						#ifdef __DEBUG_IMG_PROC__
							printf("%f\n",CalcDes(line[0].x,line[0].y,line[1].x,line[1].y,Line1X,line1));
						#endif

						#ifdef SHOW_GUI
							cvLine( img, line[0], line[1], CV_RGB(255,0,0), 3, CV_AA, 0 );
						#endif

						lineK = (float)(line[0].y-line[1].y)/(line[0].x-line[1].x);
					}
				}
			}

			isWhiteScreen = false;
			tooClose = false;
			float per = (float)4*isTooClose(img_gray,Line1X)/(img->height*img->width);
			if(per>0.9)isWhiteScreen = true;

			#ifdef __DEBUG_IMG_PROC__
				printf("per1:%f ",per);
			#endif

			per = (float)isTooCloseB(img_twovalue)/(img->width);
			if(per<0.6)tooClose = true;

			#ifdef __DEBUG_IMG_PROC__
				printf("per2:%f\n",per);
			#endif

			//-----------------------------
			//显示图像
			#ifdef SHOW_GUI
				puts("show captured!");
				cvShowImage ("capture", img);
				cvShowImage ("two", img_twovalue);
			#endif

			//-----------------------------
			//垃圾回收
			cvReleaseImage(&img_raw);
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
			#ifdef __DEBUG_STEP__
				puts("step finish!");
			#endif

			if(actionQueue.empty())
			{
				#ifdef __DEBUG_STEP__
					puts("new action group");
				#endif
				if(captureFinished)
				{
					captureFinished =false;
					//进行步态的选择啦
					if(tooClose&&line1<10)//过于靠近黑线，危险
					{
						#ifdef __DEBUG_STEP__
							puts("danger*****************************");
						#endif
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
							#ifdef __DEBUG__
								puts("UNKONW STATUE");
							#endif
							actionQueue.push(ACTION_STAND_STAND);
							break;
						}
						actionQueue.push(ACTION_TURN_LEFT);
						actionQueue.push(ACTION_TURN_LEFT);
					}
					else if(isWhiteScreen)//白屏被认定
					{
						#ifdef __DEBUG_STEP__
							puts("--------:white screen---------------");
						#endif

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
							#ifdef __DEBUG__
								puts("ERROR!UNDEF STATUE!");
							#endif
							actionQueue.push(ACTION_STAND_STAND);
							actionQueue.push(ACTION_TURN_RIGHT);
							break;
						}
						actionQueue.push(ACTION_STAND_TO_RIGHT);
						actionQueue.push(ACTION_WALK_LEFT);
					}
					else if(line1>=MaxLine1||/**/lineK<-0.3)//左转
					{
						#ifdef __DEBUG_STEP__
							puts("--------:turn left------------------");
							printf("line k: %f",lineK);
							if(line1<MinLine1)
							{
								puts("line1>=MaxLine1||");
							}
							else
							{
								puts("lineK<-0.3");
							}
						#endif

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
							#ifdef __DEBUG__
								puts("UNKONW STATUE");
							#endif
							actionQueue.push(ACTION_STAND_STAND);
							break;
						}
						actionQueue.push(ACTION_TURN_LEFT);
					}
					else if(/*line1<MinLine1||*/lineK>0.3)//右转
					{
						#ifdef __DEBUG_STEP__
							puts("--------:turn right-----------------");
							printf("line k: %f",lineK);
							if(line1>=MaxLine1)
							{
								puts("line1<MinLine1");
							}
							else
							{
								puts("lineK>0.3");
							}
						#endif
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
							#ifdef __DEBUG__
								puts("ERROR!UNDEF STATUE!");
							#endif
							actionQueue.push(ACTION_STAND_STAND);
							actionQueue.push(ACTION_TURN_RIGHT);
							break;
						}
						actionQueue.push(ACTION_STAND_TO_RIGHT);
					}
					else
					{
						//根据当前状态继续步态
						#ifdef __DEBUG_STEP__
							puts("--------:walk------------------");
						#endif
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
				puts("action group not finish!");
				nowAction = actionQueue.front();
				actionQueue.pop();
				actions[nowAction]->active();
			}
			#ifdef __DEBUG_STEP__
				switch(nowAction)
				{
				case ACTION_LEFT_TO_STAND:
					puts("ACTION_LEFT_TO_STAND");
					break;
				case ACTION_RIGHT_TO_STAND:
					puts("ACTION_RIGHT_TO_STAND");
					break;
				case ACTION_STAND_STAND:
					puts("ACTION_STAND_STAND");
					break;
				case ACTION_STAND_TO_LEFT:
					puts("ACTION_STAND_TO_LEFT");
					break;
				case ACTION_STAND_TO_RIGHT:
					puts("ACTION_STAND_TO_RIGHT");
					break;
				case ACTION_TURN_LEFT:
					puts("ACTION_TURN_LEFT");
					break;
				case ACTION_TURN_RIGHT:
					puts("ACTION_TURN_RIGHT");
					break;
				case ACTION_WALK_LEFT:
					puts("ACTION_WALK_LEFT");
					break;
				case ACTION_WALK_RIGHT:
					puts("ACTION_WALK_RIGHT");
					break;
				}
			#endif
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
		if((clock()-this->ActiveTime) > ((long)this->it->lastTime*150))
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
