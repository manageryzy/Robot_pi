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

int otsu(const IplImage *src_image) //大津法求阈值
{
    double sum = 0.0;
    double w0 = 0.0;
    double w1 = 0.0;
    double u0_temp = 0.0;
    double u1_temp = 0.0;
    double u0 = 0.0;
    double u1 = 0.0;
    double delta_temp = 0.0;
    double delta_max = 0.0;

    //src_image灰度级
    int pixel_count[256]={0};
    float pixel_pro[256]={0};
    int threshold = 0;
    uchar* data = (uchar*)src_image->imageData;
    //统计每个灰度级中像素的个数
    for(int i = 0; i < src_image->height; i++)
    {
        for(int j = 0;j < src_image->width;j++)
        {
            pixel_count[(int)data[i * src_image->width + j]]++;
            sum += (int)data[i * src_image->width + j];
        }
    }
    //计算每个灰度级的像素数目占整幅图像的比例
    for(int i = 0; i < 256; i++)
    {
    pixel_pro[i] = (float)pixel_count[i] / ( src_image->height * src_image->width );
    }
    //遍历灰度级[0,255],寻找合适的threshold
    for(int i = 0; i < 256; i++)
    {
        w0 = w1 = u0_temp = u1_temp = u0 = u1 = delta_temp = 0;
        for(int j = 0; j < 256; j++)
        {
            if(j <= i)   //背景部分
            {
                w0 += pixel_pro[j];
                u0_temp += j * pixel_pro[j];
            }
            else   //前景部分
            {
                w1 += pixel_pro[j];
                u1_temp += j * pixel_pro[j];
            }
        }
        u0 = u0_temp / w0;
        u1 = u1_temp / w1;
        delta_temp = (float)(w0 *w1* pow((u0 - u1), 2)) ;
        if(delta_temp > delta_max)
        {
            delta_max = delta_temp;
            threshold = i;
        }
    }
    return threshold;
}
