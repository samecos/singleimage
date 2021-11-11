#pragma once
#include "CaliStruct.h"

//计算单张影像覆盖率
double CalSingleImageCoverage(DetectCircleBoard dBoard, Size imgSize,Size pixRes,Mat& coverImg)
{
	int coverW = cvRound(imgSize.width / pixRes.width)+2;
	int coverH = cvRound(imgSize.height / pixRes.height) + 2;

	Mat srcCoverImg = Mat::zeros(Size(coverW, coverH), CV_8UC1);
	for (int i = 0; i < dBoard.pts.size(); i++)
	{
		int ptX = cvRound(dBoard.pts[i].imgCoor.x / pixRes.width);
		int ptY = cvRound(dBoard.pts[i].imgCoor.y / pixRes.height);
		int size = cvCeil(dBoard.pts[i].size / pixRes.width);
		Rect rc = Rect(ptX - size / 2, ptY - size / 2, size, size);		
		srcCoverImg(rc) += 1;
	}
	normalize(srcCoverImg, coverImg, 0, 255, NORM_MINMAX,CV_8UC1);
	int cnt = countNonZero(srcCoverImg);
	return ((double)cnt / ((double)coverW * coverH));
}

//所有影像投到一张影像上计算覆盖率
double CalCoverageUseAllImage(vector<DetectCircleBoard>& vBoards,Size imgSize, Size pixRes, Mat& coverImg)
{
	int coverW = cvRound(imgSize.width / pixRes.width) + 1;
	int coverH = cvRound(imgSize.height / pixRes.height) + 1;

	Mat srcCoverImg = Mat::zeros(Size(coverW, coverH), CV_8UC1);
	for (int i = 0; i < vBoards.size(); i++)
	{
		for (int j = 0; j < vBoards[i].pts.size(); j++)
		{
			int ptX = cvRound(vBoards[i].pts[j].imgCoor.x / pixRes.width);
			int ptY = cvRound(vBoards[i].pts[j].imgCoor.y / pixRes.height);
			int size = cvCeil(vBoards[i].pts[j].size / pixRes.width);
			if ((ptX + size / 2 - coverW + 1) > 0)
				ptX = coverW - 1 - size/2;
			if ((ptY + size / 2 - coverH + 1) > 0)
				ptY = coverH - 1 - size / 2;

			if ((ptX - size / 2 ) < 0)
				ptX = size / 2;
			if ((ptY - size / 2 ) < 0)
				ptY = size / 2;

			Rect rc = Rect(ptX - size / 2, ptY - size / 2, size, size);
			srcCoverImg(rc) += 1;
		}
	}
	normalize(srcCoverImg, coverImg, 0, 255, NORM_MINMAX, CV_8UC1);
	int cnt = countNonZero(srcCoverImg);
	return ((double)cnt / ((double)coverW * coverH));
}



