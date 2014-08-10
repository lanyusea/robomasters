#include <iostream>
#include <opencv2/legacy/legacy.hpp>
#include <stack>
#include <queue>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>//TODO
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc_c.h>

using namespace std;

#define Fx 817.74458
#define Fy 818.04921
#define Cx 314.85848
#define Cy 227.90983
#define K1 0.02061
#define K2 -0.29551
#define P1 0.00385
#define P2 0.00028
#define defaultNbSamples 20
#define defaultReqMatches 2
#define defaultRadius 20
#define defaultSubsamplingFactor 16
#define background 0
#define foreground 255
#define T_ROW_SEGMENT_THRESHOLD 50
#define T_COL_SEGMENT_THRESHOLD 50
#define T_VOTE_STHRESHOLD 0.2
#define T_SUM_STHRESHOLD 19
#define T_COLOUR_THRESHOLD 1
#define T_IMAGE_WIDTH 640
#define T_IMAGE_HEIGHT 480
#define T_RECT_MERGE   20
#define T_RED_A     5
#define T_RED_B     170
#define T_RED_C     0
#define T_RED_D     175
#define T_RED_E     180
#define T_BLUE_LOW     102
#define T_BLUE_HIGH     110
#define max_corners 50
IplImage* RgbImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
IplImage* GrayImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* LastGrayImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* DiffImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* ThresholdImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* VibeImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* TickImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* TmpImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* EDImage = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);
IplImage* pImgTst = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
IplImage* pImgHsv = cvCreateImage(cvSize(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), IPL_DEPTH_8U, 3);


struct SObstacle
{
    CvPoint FrPt;
    CvPoint NrPt;
    CvPoint CnPt;
    int nWidth;
    int nHeight;
    float fDist;
    float fHAngle;
    float fVAngle;
    float fVx;
    float fVy;
};

bool IntsectRect(SObstacle sObs1, SObstacle sObs2)
{
    SObstacle TmpObs;
    TmpObs = sObs2;
    if ((sObs1.FrPt.y >= sObs2.NrPt.y && sObs2.FrPt.y >= sObs1.NrPt.y) && (sObs1.FrPt.x >= sObs2.NrPt.x && sObs2.FrPt.x >= sObs1.NrPt.x))
    {
        return true;
    }
    else
        return false;
}