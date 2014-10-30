#include "header.h"

#include <iostream>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <cv.h>
#include <highgui.h>


class ASDFrameHolder
{
private:
    IplImage *image;
    double timeStamp;

public:
    ASDFrameHolder();
    virtual ~ASDFrameHolder();
    virtual void assignFrame(IplImage *sourceImage, double frameTime);
    inline IplImage *getImage();
    inline double getTimeStamp();
    virtual void setImage(IplImage *sourceImage);
};

class ASDFrameSequencer
{
public:
    virtual ~ASDFrameSequencer();
    virtual IplImage *getNextImage();
    virtual void close();
    virtual bool isOpen();
    virtual void getFrameCaption(char *caption);
};

class ASDCVFrameSequencer : public ASDFrameSequencer
{
protected:
    CvCapture *capture;

public:
    virtual IplImage *getNextImage();
    virtual void close();
    virtual bool isOpen();
};

class ASDFrameSequencerWebCam : public ASDCVFrameSequencer
{
public:
    virtual bool open(int cameraIndex);
};

class ASDFrameSequencerVideoFile : public ASDCVFrameSequencer
{
public:
    virtual bool open(const char *fileName);
};

class ASDFrameSequencerImageFile : public ASDFrameSequencer {
private:
    char sFileNameMask[2048];
    int nCurrentIndex, nStartIndex, nEndIndex;

public:
    virtual void open(const char *fileNameMask, int startIndex, int endIndex);
    virtual void getFrameCaption(char *caption);
    virtual IplImage *getNextImage();
    virtual void close();
    virtual bool isOpen();
};


int opencvCaptureInit();
IplImage * myCaptureImage();
void opencvCaptureClose();
int otsu(const IplImage *src_image); //大津法求阈值
const int findBlackLine(const IplImage *twoValue,const IplImage *sobel,IplImage *img,int lineX,int errorSize = 10);
bool isToolClose(const IplImage *twoValue,int lineX);
