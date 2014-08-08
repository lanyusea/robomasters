#include "color.h"
#include <iostream>
using namespace std;
using namespace cv;

CvMat inImage;
CvMat outImage;
cv_bridge::CvImagePtr cv_ptr;
IplImage* pLabelImg = NULL;
	vector<SObstacle> vSObs, vRltObs, vTmpObs, vPreObs;
	time_t c_start, c_end;
	CvScalar sColour;
	int nH, nS, nV, nStep;
	unsigned char* DstData;

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
		sObs1.fHAngle = 37.5 * (2 * (float)sObs1.CnPt.x / T_IMAGE_WIDTH - 1);
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
				TmpObs.FrPt.x += TmpObs.fVx * nDeltaT;
			else
				TmpObs.NrPt.x += TmpObs.fVx * nDeltaT;
			if (TmpObs.fVy > 0)
				TmpObs.FrPt.y += TmpObs.fVy * nDeltaT;
			else
				TmpObs.NrPt.y += TmpObs.fVy * nDeltaT;
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

void MergeObs(vector<SObstacle> &vSrcObsList, vector<SObstacle>& vSRltObsList)
{
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
	cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	inImage = cv_ptr->image;
	
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

			if (vRltObs.size() > 0)
			{
				nHangle = (short int)(vRltObs[0].fHAngle * 100 + vRltObs[0].fVx * nDeltaT / 1000);
//send serial				mPort.SendAngle(nHangle, 800);
				//cout << nHangle << endl;
			}
			else
			{
//send serial				mPort.SendAngle(20000, 20000);
			}
			for (unsigned int nI = 0; nI < vRltObs.size(); nI++)
			{
				cout << vRltObs[nI].fVx << ", " << vRltObs[nI].fVy << endl;
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

	//credit to QI Xiaolin
	//IplImage* pFrame0 = NULL;
	

	
	

	vTmpObs.clear();
	vPreObs.clear();

	cvNamedWindow("RGB", CV_WINDOW_AUTOSIZE);
	RgbImage->origin = IPL_ORIGIN_TL;
	pImgTst->origin = IPL_ORIGIN_TL;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imgSub;
	imgSub = it.subscribe("/gnd_cam/image0", 20, imageCallback);

	ros::spin();
    return 0;
}
