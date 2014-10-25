//#include "opencv2/opencv.hpp"
//#include "opencv2/contrib/contrib.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include "robot.h"
#include "serialOut.h"
#include "opencvCapture.h"
#include "confReader.h"
#include <pthread.h>

/////////////////////////////////////////////
//global flags
bool shouldCaptule = false,
	shouldExit = false;

/**
 * 初始化
 */
int init()
{
	confReader RobotConfReader("./conf/robot.conf");

	if(SerialInit()!=0)
	{
		puts("error in init serial!Quit..");
		return 1;
	}

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
		*img_canny; //canny 检测之后的图像

	if(init()!=0)
		exit(-1);

	puts("init finished!");

	while (1)
	{
		if(shouldCaptule)
		{
			shouldCaptule = false;
			img = myCaptureImage();

			#ifdef __DEBUG__
				puts("captured!");
			#endif

			img_gray=cvCloneImage(img);
			#ifdef __DEBUG__
				puts("img_gray=cvCloneImage(img);");
			#endif

			img_gray=cvCreateImage(cvGetSize(img),img->depth,1);
			#ifdef __DEBUG__
				puts("img_gray=cvCreateImage(cvGetSize(img),img->depth,1);");
			#endif

			img_canny =cvCreateImage(cvGetSize(img),img->depth,1);
			#ifdef __DEBUG__
				puts("img_canny =cvCreateImage(cvGetSize(img),img->depth,1);");
			#endif

			cvCvtColor(img,img_gray,CV_RGB2GRAY);
			#ifdef __DEBUG__
				puts("cvCvtColor(img,img_gray,CV_RGB2GRAY);");
			#endif

			cvCanny(img_gray,img_canny,30,50,3);
			#ifdef __DEBUG__
				puts("cvCanny(img_gray,img_canny,30,50,3);");
			#endif

			#ifdef SHOW_GUI
				#ifdef __DEBUG__
					puts("show captured!");
				#endif
				cvShowImage ("capture", img_canny);
			#endif

			cvReleaseImage(&img);
			cvReleaseImage(&img_gray);
			cvReleaseImage(&img_canny);
		}

		puts("before sleep");
		sleep(1);
		puts("after sleep");

		if (cvWaitKey(1) == 27 || shouldExit)
			break;
	}

	puts("quit!");

	release();

	return 0;
}
