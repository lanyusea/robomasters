#include "color.h"
#include <iostream>
//#include <BoundingBox.h>
using namespace std;
using namespace cv;
using namespace LibSerial;

CvMat inImage;
CvMat outImage;
cv_bridge::CvImagePtr cv_ptr;
IplImage* pLabelImg = NULL;
vector<SObstacle> vSObs, vRltObs, vTmpObs, vPreObs;
time_t c_start, c_end;
CvScalar sColour;
int nH, nS, nV, nStep;
unsigned char* DstData;
std_msgs::Header img_header;

long tPre = 0, tCur;
int nDeltaT;
CvSeq *pcvSeq = NULL;
CvRect tRect;
SObstacle sTmpObs;
int nHangle = 0;
float fLW;
int nIndex = 0;
IplImage *pOutlineImage = cvCreateImage(cvGetSize(TmpImage), IPL_DEPTH_8U, 3);
CvMemStorage *pcvMStorage = cvCreateMemStorage();
ros::Publisher pub1;

SerialStream serial_port;
//void msgPublish(int x, int y, int width, int height, float confidence) { //confidence is not used here
//    tld_msgs::BoundingBox msg;
//    msg.header = img_header; //Add the Header of the last image processed
//    msg.x = x;
//    msg.y = y;
//    msg.width = width;
//    msg.height = height;
//    msg.confidence = confidence;
//    pub1.publish(msg);
//}

void serialInit() {
    if (serial_port.IsOpen())
        serial_port.Close();
    serial_port.Open("/dev/ttyUSB0");//TODO
    if ( ! serial_port.good() )
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
            << "Error: Could not open serial port."
            << std::endl ;
        exit(1) ;
    }

    // Set the baud rate of the serial port.
    serial_port.SetBaudRate( SerialStreamBuf::BAUD_57600) ; //TODO
    //int rate = serial_port.BaudRate();
    //cout << SerialStreamBuf::BAUD_115200<<endl;
    //cout << rate <<endl;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the baud rate." <<
            std::endl ;
        exit(1) ;
    }
    //
    // Set the number of data bits.
    //
    serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the character size." <<
            std::endl ;
        exit(1) ;
    }
    //
    // Disable parity.
    //
    serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not disable the parity." <<
            std::endl ;
        exit(1) ;
    }
    //
    // Set the number of stop bits.
    //
    serial_port.SetNumOfStopBits( 1 ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the number of stop bits."
            << std::endl ;
        exit(1) ;
    }
    //
    // Turn off hardware flow control.
    //
    serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not use hardware flow control."
            << std::endl ;
        exit(1) ;
    }

}


void serialSend(int16_t yaw, int16_t pixel) {
    //cout << yaw << ",";
    //cout << std::hex << yaw <<endl;
    //cout << pitch <<",";
    //cout << std::hex << pitch << endl;
    uint16_t yawStart = yaw >> 8;
    uint16_t yawEnd = yaw & 0xFF;
    uint16_t pixelStart = pixel >> 8;
    uint16_t pixelEnd = pixel & 0xFF;

    serial_port << std::hex << 0xAA
        << std::hex << 0x55
        << std::hex <<yawEnd
        << std::hex <<yawStart
        << std::hex <<pixelEnd
        << std::hex <<pixelStart
        << std::hex <<0xBB;

//    cout << std::hex << 0xAA
//        << std::hex << 0x55
//        << std::hex << yawEnd
//        << std::hex << yawStart
//        << std::hex << pitchEnd
//        << std::hex << pitchStart
//        << std::hex << 0xBB << endl;


}
float RectDist(SObstacle & sObs1, SObstacle sObs2)
{
    float fDis,dX,dY;
    if (sObs1.FrPt.x >= sObs2.NrPt.x && sObs2.FrPt.x >= sObs1.NrPt.x)
    {
        if (sObs1.FrPt.y >= sObs2.NrPt.y && sObs2.FrPt.y >= sObs1.NrPt.y)
        {
            fDis = 0;
        }
        else
        {
            fDis = (sObs1.FrPt.y < sObs2.NrPt.y) ? (sObs2.NrPt.y - sObs1.FrPt.y) : (sObs1.NrPt.y - sObs2.FrPt.y);
        }
    }
    else
    {
        if (sObs1.FrPt.y >= sObs2.NrPt.y && sObs2.FrPt.y >= sObs1.NrPt.y)
        {
            fDis = (sObs1.FrPt.x < sObs2.NrPt.x) ? (sObs2.NrPt.x - sObs1.FrPt.x) : (sObs1.NrPt.x - sObs2.FrPt.x);
        }
        else
        {
            dX = (sObs1.FrPt.x < sObs2.NrPt.x) ? (sObs2.NrPt.x - sObs1.FrPt.x) : (sObs1.NrPt.x - sObs2.FrPt.x);
            dY = (sObs1.FrPt.y < sObs2.NrPt.y) ? (sObs2.NrPt.y - sObs1.FrPt.y) : (sObs1.NrPt.y - sObs2.FrPt.y);
            fDis = sqrt(dX * dX + dY * dY);
        }
    }
    if (fDis < T_RECT_MERGE)
    {
        sObs1.FrPt.x = max(sObs1.FrPt.x, sObs2.FrPt.x);
        sObs1.FrPt.y = max(sObs1.FrPt.y, sObs2.FrPt.y);
        sObs1.NrPt.x = min(sObs1.NrPt.x, sObs2.NrPt.x);
        sObs1.NrPt.y = min(sObs1.NrPt.y, sObs2.NrPt.y);
        sObs1.CnPt.x = (sObs1.NrPt.x + sObs1.FrPt.x) / 2;
        sObs1.CnPt.y = (sObs1.NrPt.y + sObs1.FrPt.y) / 2;
        sObs1.nWidth = sObs1.FrPt.x - sObs1.NrPt.x;
        sObs1.nHeight = sObs1.FrPt.y - sObs1.NrPt.y;
        sObs1.fHAngle = 35 * (2 * (float)sObs1.CnPt.x / T_IMAGE_WIDTH - 1);
    }

    return fDis;
}
void Tracking(vector<SObstacle> vPreObs, vector<SObstacle> & vCurObs, int nDeltaT)
{
    SObstacle TmpObs;


    for (unsigned int nI = 0; nI < vCurObs.size(); nI++)
    {
        for (unsigned int nJ = 0; nJ < vPreObs.size(); nJ++)
        {
            TmpObs = vPreObs[nJ];
            if (TmpObs.fVx > 0)
                TmpObs.FrPt.x += TmpObs.fVx * nDeltaT/1000;
            else
                TmpObs.NrPt.x += TmpObs.fVx * nDeltaT/1000;
            if (TmpObs.fVy > 0)
                TmpObs.FrPt.y += TmpObs.fVy * nDeltaT/1000;
            else
                TmpObs.NrPt.y += TmpObs.fVy * nDeltaT/1000;
            if (IntsectRect(vCurObs[nI], TmpObs))
            {
                //RltObs.nHeight = (nTolHeight - qObs.front().nHeight + vCurObs[nI].nHeight) / nQueueCount;
                //RltObs.nWidth = (nTolWidth - qObs.front().nWidth + vCurObs[nI].nWidth) / nQueueCount;
                vCurObs[nI].nHeight = (vPreObs[nJ].nHeight + vCurObs[nI].nHeight) / 2;
                vCurObs[nI].nWidth = (vPreObs[nJ].nWidth + vCurObs[nI].nWidth) / 2;
                vCurObs[nI].FrPt.x = vCurObs[nI].CnPt.x + vCurObs[nI].nWidth / 2;
                vCurObs[nI].FrPt.y = vCurObs[nI].CnPt.y + vCurObs[nI].nHeight / 2;
                vCurObs[nI].NrPt.x = vCurObs[nI].CnPt.x - vCurObs[nI].nWidth / 2;
                vCurObs[nI].NrPt.y = vCurObs[nI].CnPt.y - vCurObs[nI].nHeight / 2;
                vCurObs[nI].fVx = (float)(vCurObs[nI].CnPt.x - vPreObs[nJ].CnPt.x) * 1000 / nDeltaT;
                vCurObs[nI].fVy = (float)(vCurObs[nI].CnPt.y - vPreObs[nJ].CnPt.y) * 1000 / nDeltaT;
                //vRltObs.push_back(RltObs);
                //bPreState = FALSE;
            }
        }
    }
}

void MergeObs(vector<SObstacle> &vSrcObsList, vector<SObstacle>& vSRltObsList) {
    //SObstacleInGridMap STmpGridObs;
    //vector<SObstacleInfo> vTmpObsList;
    int nNum;
    nNum = vSrcObsList.size();
    bool bKeepWork = true;
    //BOOL bFlag[200];
    vector<bool> bFlag;
    vSRltObsList.clear();

    if (nNum < 2)
    {
        vSRltObsList.push_back(vSrcObsList[0]);
    }
    else
    {
        for (unsigned int nI = 0; nI < vSrcObsList.size(); nI++)
        {
            bFlag.push_back(true);
        }
        while (bKeepWork)
        {
            bKeepWork = false;
            for (unsigned int nI = 0; nI < vSrcObsList.size(); nI++)
            {
                if (bFlag[nI])
                {
                    for (int nJ = nI + 1; nJ < nNum; nJ++)
                    {
                        if (bFlag[nJ])
                        {
                            if (RectDist(vSrcObsList[nI], vSrcObsList[nJ]) < T_RECT_MERGE)
                            {
                                bFlag[nJ] = false;
                                bKeepWork = true;
                            }
                        }
                    }
                }
            }
        }

        for (int nI = 0; nI < nNum; nI++)
        {
            if (bFlag[nI])
            {
                vSRltObsList.push_back(vSrcObsList.at(nI));
            }
        }
        bFlag.clear();
    }

}


void imageCallback(const sensor_msgs::ImageConstPtr& img) {
    //credit to QI Xiaolin
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    inImage = cv_ptr->image;
    img_header = cv_ptr->header;

    //while(cv_ptr){
        nIndex +=1;
        tCur = clock();
        nDeltaT = tCur - tPre;
        cvCopy(&inImage,RgbImage);
        cvCopy(&inImage,pImgTst);

        cvRectangle(TmpImage,cvPoint(0,0),cvPoint(T_IMAGE_WIDTH, T_IMAGE_HEIGHT), cvScalar(0,0,0), CV_FILLED, 8, 0);
        cvCvtColor(RgbImage,pImgHsv, CV_BGR2HSV);
        DstData = (unsigned char*)TmpImage->imageData;
        nStep = TmpImage->widthStep / sizeof(unsigned char);

        for (int nI = 0; nI < T_IMAGE_HEIGHT; nI++)
        {
            for (int nJ = 0; nJ < T_IMAGE_WIDTH; nJ++)
            {
                sColour = cvGet2D(pImgHsv, nI, nJ);
                nH = (int)sColour.val[0];
                nS = (int)sColour.val[1];
                nV = (int)sColour.val[2];
                if (nV > 46 && nS > 43)
                {
                    if (nH < T_RED_A || nH > T_RED_B)
                    {
                        DstData[nI*nStep + nJ] = 255;
                    }

                }
            }
        }
        cvErode(TmpImage, EDImage, NULL, 3);
        cvDilate(EDImage, TmpImage, NULL, 1);

        cvDilate(TmpImage, EDImage, NULL, 1);
        cvErode(EDImage, TickImage, NULL, 1);


        cvFindContours(TickImage, pcvMStorage, &pcvSeq, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

        vSObs.clear();
        vRltObs.clear();
        for (; pcvSeq != NULL; pcvSeq = pcvSeq->h_next)
        {
            CvRect rect = cvBoundingRect(pcvSeq, 0);
            memset(&sTmpObs, 0, sizeof(sTmpObs));
            sTmpObs.NrPt.x = rect.x;
            sTmpObs.NrPt.y = rect.y;
            sTmpObs.FrPt.x = rect.x + rect.width;
            sTmpObs.FrPt.y = rect.y + rect.height;
            sTmpObs.CnPt.x = (sTmpObs.NrPt.x + sTmpObs.FrPt.x) / 2;
            sTmpObs.CnPt.y = (sTmpObs.NrPt.y + sTmpObs.FrPt.y) / 2;
            sTmpObs.nHeight = rect.height;
            sTmpObs.nWidth = rect.width;
            sTmpObs.fHAngle = 35 * (2 * (float)sTmpObs.CnPt.x / T_IMAGE_WIDTH - 1);
            vSObs.push_back(sTmpObs);
        }

        if (vSObs.size() > 0)
        {
            MergeObs(vSObs, vRltObs);
            vTmpObs.clear();
            for (int nI = 0; nI < vRltObs.size(); nI++)
            {
                if (vRltObs[nI].nHeight * vRltObs[nI].nWidth > 800)
                {
                    vTmpObs.push_back(vRltObs[nI]);
                    //vSObs.erase(vSObs.begin() + nI);
                }
            }
            vRltObs.clear();
            for (int nI = 0; nI < vTmpObs.size(); nI++)
            {
                fLW = ((float)(vTmpObs[nI].nHeight)) / vTmpObs[nI].nWidth;

                if (fLW < 0.7 || fLW > 1.43)
                {
                    vRltObs.push_back(vTmpObs[nI]);
                }
                else
                {
                    nIndex--;
                }
            }
            Tracking(vPreObs, vRltObs, nDeltaT);
            //msgPublish(vRltObs.CnPt.x,vRltObs.CnPt.y,vRltObs.nWidth,vRltObs.nHeight);
            if (vRltObs.size() > 0)
            {
                nHangle = (short int)(vRltObs[0].fHAngle * 100 + vRltObs[0].fVx * nDeltaT / 1000);

                serialSend((int16_t)nHangle,(int16_t)vRltObs[0].CnPt.x);
            }
            else
            {
                serialSend((int16_t)20000,(int16_t)20000);
            }
            for (unsigned int nI = 0; nI < vRltObs.size(); nI++)
            {
                //cout << vRltObs[nI].fVx << ", " << vRltObs[nI].fVy << endl;
                cvRectangle(RgbImage, vRltObs[nI].FrPt, vRltObs[nI].NrPt, cvScalar(0, 0, 255));
            }
        }
        vPreObs.clear();
        vPreObs = vRltObs;

        cvShowImage("TickWhite", TmpImage);

        cvShowImage("RGB", RgbImage);
        //cvShowImage("Rlt", pOutlineImage);
        tPre = tCur;
        //cout << tCur - tPre << endl;
        cvWaitKey(1);
        DstData = NULL;
        pcvSeq = NULL;
        vTmpObs.clear();
        vRltObs.clear();
        vSObs.clear();
        cvReleaseImage(&pOutlineImage);
    //}


}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"color_detect");
    ros::NodeHandle nh;

    serialInit();

    vTmpObs.clear();
    vPreObs.clear();

    cvNamedWindow("RGB", CV_WINDOW_AUTOSIZE);
    RgbImage->origin = IPL_ORIGIN_TL;
    pImgTst->origin = IPL_ORIGIN_TL;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imgSub;
    //ros::Publisher pub1 = nh.advertise<tld_msgs::BoundingBox>("color_detect", 1000, true);
    imgSub = it.subscribe("/gnd_cam/image0", 20, imageCallback);

    ros::spin();
    return 0;
}
