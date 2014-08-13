#include <iostream>
#include <opencv2/legacy/legacy.hpp>
#include <stack>
#include <queue>
#include <opencv2/opencv.hpp>//TODO
#include <opencv2/imgproc/imgproc_c.h>
#include <SerialStream.h>
using namespace std;

#define Fx 817.74458
#define Fy 818.04921
#define Cx 314.85848
#define Cy 227.90983
#define K1 0.02061
#define K2 -0.29551
#define P1 0.00385
#define P2 0.00028


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

