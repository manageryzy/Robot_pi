#include "opencvCapture.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif


//-------------------- ASDFrameHolder -----------------------//
ASDFrameHolder::ASDFrameHolder( )
{
    image = NULL;
    timeStamp = 0;
}

ASDFrameHolder::~ASDFrameHolder( )
{
    cvReleaseImage(&image);
}

void ASDFrameHolder::assignFrame(IplImage *sourceImage, double frameTime)
{
    if (image != NULL)
    {
        cvReleaseImage(&image);
        image = NULL;
    }

    image = cvCloneImage(sourceImage);
    timeStamp = frameTime;
}

IplImage *ASDFrameHolder::getImage()
{
    return image;
}

double ASDFrameHolder::getTimeStamp()
{
    return timeStamp;
}

void ASDFrameHolder::setImage(IplImage *sourceImage)
{
    image = sourceImage;
}


//-------------------- ASDFrameSequencer -----------------------//

ASDFrameSequencer::~ASDFrameSequencer()
{
    close();
}

IplImage *ASDFrameSequencer::getNextImage()
{
    return NULL;
}

void ASDFrameSequencer::close()
{

}

bool ASDFrameSequencer::isOpen()
{
    return false;
}

void ASDFrameSequencer::getFrameCaption(char* /*caption*/) {
    return;
}

IplImage* ASDCVFrameSequencer::getNextImage()
{
    IplImage *image;

    image = cvQueryFrame(capture);

    if (image != NULL)
    {
        return cvCloneImage(image);
    }
    else
    {
        return NULL;
    }
}

void ASDCVFrameSequencer::close()
{
    if (capture != NULL)
    {
        cvReleaseCapture(&capture);
    }
}

bool ASDCVFrameSequencer::isOpen()
{
    return (capture != NULL);
}


//-------------------- ASDFrameSequencerWebCam -----------------------//

bool ASDFrameSequencerWebCam::open(int cameraIndex)
{
    close();

    capture = cvCaptureFromCAM(cameraIndex);

    if (!capture)
    {
        return false;
    }
    else
    {
        return true;
    }
}


//-------------------- ASDFrameSequencerVideoFile -----------------------//

bool ASDFrameSequencerVideoFile::open(const char *fileName)
{
    close();

    capture = cvCaptureFromFile(fileName);
    if (!capture)
    {
        return false;
    }
    else
    {
        return true;
    }
}


//-------------------- ASDFrameSequencerImageFile -----------------------//

void ASDFrameSequencerImageFile::open(const char *fileNameMask, int startIndex, int endIndex)
{
    nCurrentIndex = startIndex-1;
    nStartIndex = startIndex;
    nEndIndex = endIndex;

    std::sprintf(sFileNameMask, "%s", fileNameMask);
}

void ASDFrameSequencerImageFile::getFrameCaption(char *caption) {
    std::sprintf(caption, sFileNameMask, nCurrentIndex);
}

IplImage* ASDFrameSequencerImageFile::getNextImage()
{
    char fileName[2048];

    nCurrentIndex++;

    if (nCurrentIndex > nEndIndex)
        return NULL;

    std::sprintf(fileName, sFileNameMask, nCurrentIndex);

    IplImage* img = cvLoadImage(fileName);

    return img;
}

void ASDFrameSequencerImageFile::close()
{
    nCurrentIndex = nEndIndex+1;
}

bool ASDFrameSequencerImageFile::isOpen()
{
    return (nCurrentIndex <= nEndIndex);
}

//----------------------------------------------------------
//上面的是库里面的文件，下面的是自定义的。或许写成一个类更好，但是懒得改了
ASDFrameSequencer *sequencer;

//初始化
int opencvCaptureInit()
{
	//img_gray=cvCreateImage(cvSize(320,280),IPL_DEPTH_16U,1);

	sequencer = new ASDFrameSequencerWebCam();
		(dynamic_cast<ASDFrameSequencerWebCam*>(sequencer))->open(-1);

	if (! sequencer->isOpen())
	{
		std::cout << std::endl << "Error: Cannot initialize the default Webcam" << std::endl << std::endl;
		return -1;
	}

	return 0;
}

//捕获图像，并且丢弃掉一定数量的数据
IplImage * myCaptureImage()
{
	int dropFrames=0;
	IplImage *img;

	while ((img = sequencer->getNextImage()) != 0)
	{
		if(dropFrames++<=6)
		{
			continue;
		}
		return img;
	}
}

//释放资源
void opencvCaptureClose()
{
	sequencer->close();
	delete sequencer;
}
