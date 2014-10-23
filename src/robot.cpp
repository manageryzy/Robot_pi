//#include "opencv2/opencv.hpp"
//#include "opencv2/contrib/contrib.hpp"
//#include "opencv2/highgui/highgui.hpp"
#include "robot.h"
#include "serialOut.h"
#include "opencv.h"
#include "confReader.h"


int main(int argc, char** argv )
{
	IplImage *img,*img_gray,*img_canny;

	ASDFrameSequencer *sequencer;
	
	//img_gray=cvCreateImage(cvSize(320,280),IPL_DEPTH_16U,1);
	
	sequencer = new ASDFrameSequencerWebCam();
        (dynamic_cast<ASDFrameSequencerWebCam*>(sequencer))->open(-1);

        if (! sequencer->isOpen())
        {
		std::cout << std::endl << "Error: Cannot initialize the default Webcam" << std::endl << std::endl;
		return -1;
        }
	
	cvNamedWindow("capture", CV_WINDOW_AUTOSIZE);
	
	while ((img = sequencer->getNextImage()) != 0)
	{
		img_gray=cvCloneImage(img);
		img_gray=cvCreateImage(cvGetSize(img),img->depth,1);
		img_canny =cvCreateImage(cvGetSize(img),img->depth,1);
		cvCvtColor(img,img_gray,CV_RGB2GRAY);
		cvCanny(img_gray,img_canny,30,50,3);
		cvShowImage ("capture", img_canny);
		cvReleaseImage(&img);
		cvReleaseImage(&img_gray);
		cvReleaseImage(&img_canny);
		if (cvWaitKey(1) == 27)
			break;
	}

	sequencer->close();
	delete sequencer;

	//cvReleaseImage(&filterMask);

	cvDestroyWindow("capture");
	
	return 0;
}