//#include "opencv2/opencv.hpp"
//#include "opencv2/contrib/contrib.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include "robot.h"
#include "serialOut.h"
#include "opencv.h"
#include "confReader.h"
#include <pthread.h>


int main(int argc, char** argv )
{
	int dropFrames=0;
	IplImage *img,*img_gray,*img_canny;

	ASDFrameSequencer *sequencer;
	
	cvStartWindowThread();
	
	//img_gray=cvCreateImage(cvSize(320,280),IPL_DEPTH_16U,1);
	
	sequencer = new ASDFrameSequencerWebCam();
        (dynamic_cast<ASDFrameSequencerWebCam*>(sequencer))->open(-1);

	if (! sequencer->isOpen())
	{
	std::cout << std::endl << "Error: Cannot initialize the default Webcam" << std::endl << std::endl;
	return -1;
	}

#ifdef SHOW_GUI
	cvNamedWindow("capture", CV_WINDOW_AUTOSIZE);
	puts("created window capture");
#endif
	
	while ((img = sequencer->getNextImage()) != 0)
	{
		if(dropFrames++%5!=0)
		{
			continue;
		}
		else
		{
			dropFrames = 1;
		}
		img_gray=cvCloneImage(img);
		img_gray=cvCreateImage(cvGetSize(img),img->depth,1);
		img_canny =cvCreateImage(cvGetSize(img),img->depth,1);
		cvCvtColor(img,img_gray,CV_RGB2GRAY);
		cvCanny(img_gray,img_canny,30,50,3);

#ifdef SHOW_GUI
		cvShowImage ("capture", img_canny);
#endif

		cvReleaseImage(&img);
		cvReleaseImage(&img_gray);
		cvReleaseImage(&img_canny);
		puts("before sleep");
		sleep(1);
		puts("after sleep");
		if (cvWaitKey(1) == 27)
			break;
	}

	sequencer->close();
	delete sequencer;

	//cvReleaseImage(&filterMask);
#ifdef SHOW_GUI
	cvDestroyWindow("capture");
	puts("destroied window capture");
#endif
	
	return 0;
}
