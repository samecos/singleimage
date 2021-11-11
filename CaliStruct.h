#pragma once
#include <ctype.h>
#include <stdarg.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <io.h>
#include <stdio.h>
#include <direct.h>
#include <stdlib.h>
#include <vector>
#include "opencv2/imgproc/types_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "RotationCalc.h"

using namespace std;
using namespace cv;

const double PI = 3.14159265358979323846;
const int unused = -99999;

const string bmpType = "bmp";
const string jpgType = "jpg";
const string subPixType = "pdf";
const string caliBoardPtType = "cbt";

Size imageSize = Size(unused, unused);

//检校点类型，使用imgcoor，fitimgcoor，eccentricImgCoor进行检校计算
enum class CaliPtType 
{ 
	KeyPt = 1,FitPt,EccentriPt
};

//旋转参数类型，输出的旋转参数是旋转矩阵，旋转向量，欧拉角或者四元数
enum class RotationType
{
	RotMatrix = 1,RotVec,Eulor, Quaternion
};

//相机类型，投影相机，点云相机，纹理相机
enum class CameraType
{
	PROJECTOR = 1,CLOUD,TEXTURE,
};

//画点的颜色
const int colorCnt = 18;
Scalar ptColor[18] = {
Scalar(0,0,255),Scalar(0,255,0), Scalar(255,0,0), 
Scalar(169,169,169),Scalar(0,0,139), Scalar(0,69,255), 
Scalar(30,105,210),Scalar(10,215,255), Scalar(0,255,255), 
Scalar(0,128,128),Scalar(144,238,144), Scalar(139,139,0), 
Scalar(230,216,173),Scalar(130,0,75), Scalar(128,0,128), 
Scalar(203,192,255),Scalar(147,20,255), Scalar(238,130,238)
};

//排序
bool sortPairIdX(const pair<float, int>& p1, const pair<float, int>& p2)
{
	if (p1.first - p2.first < 0)
		return true;
	else
		return false;
}

//按照关键点大小排序
bool sortKeyPointBySize(const KeyPoint& kp1, const KeyPoint& kp2)
{
	if (kp1.size - kp2.size > 0)
		return true;
	else
		return false;
}

//计算距离
double calcLength(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

//删除文件夹
void DeleteDir(const string& strDirPath)
{
	string strPathTmp = strDirPath + "\\*.*";
	intptr_t lFile;
	string sAddPath;
	struct _finddata_t sDate;
	lFile = _findfirst(strPathTmp.c_str(), &sDate);
	if (lFile == -1)
	{
		printf("read file fail!\n");
		return;
	}

	intptr_t hFile = 0;
	while ((hFile = _findnext(lFile, &sDate)) != -1)
	{
		if (sDate.attrib == _A_SUBDIR)
		{
			sAddPath = strDirPath;
			sAddPath += "/";
			sAddPath += sDate.name;
			if (string(sDate.name) == "." || string(sDate.name) == "..")
			{
				continue;
			}
			DeleteDir(sAddPath);
		}
		else
		{
			string strFile = strDirPath + "\\" + string(sDate.name);
			remove(strFile.c_str());
		}
	}

	_findclose(lFile);
	//删除文件夹
	_rmdir(strDirPath.c_str());
}

//生成指定文件夹
void CreateFolder(string folderPath)
{
	if (_access(folderPath.c_str(), 0) == -1)
		_mkdir(folderPath.c_str());
	else
	{
		DeleteDir(folderPath);
		_mkdir(folderPath.c_str());
	}
}

//1	0	0
//0	cos(angle) sin(angle)
//0	-sin(angle) cos(angle)
void RotationX(double angle, Mat& mR)
{
	mR = Mat::zeros(3, 3, CV_64FC1);
	double R[9];
	memcpy(R, mR.data, 9 * sizeof(double));
	R[0] = 1.0; R[8] = R[4] = cos(angle);
	R[5] = sin(angle);
	R[7] = -R[5];
	memcpy(mR.data, R, 9 * sizeof(double));
}

//cos(angle)	0	-sin(angle)
//0	1	0
//sin(angle)	0	cos(angle)
void RotationY(double angle, Mat& mR)
{
	mR = Mat::zeros(3, 3, CV_64FC1);
	double R[9];
	memcpy(R, mR.data, 9 * sizeof(double));
	R[8] = R[0] = cos(angle);
	R[4] = 1.0;
	R[2] = -sin(angle);
	R[6] = -R[2];
	memcpy(mR.data, R, 9 * sizeof(double));
}

//cos(angle)	sin(angle)	0
//-sin(angle)	cos(angle)	0
//0	0	1
void RotationZ(double angle, Mat& mR)
{
	mR = Mat::zeros(3, 3, CV_64FC1);
	double R[9];
	memcpy(R, mR.data, 9 * sizeof(double));
	R[4] = R[0] = cos(angle);
	R[8] = 1.0;
	R[1] = sin(angle);
	R[3] = -R[1];
	memcpy(mR.data, R, 9 * sizeof(double));
}

void Matrix2Eulor(double* R, int rotateOrder, double& eulor1, double& eulor2, double& eulor3)
{
	switch (rotateOrder)
	{
		//第一类：第一次和第三次转动绕相同轴
	case 121:
	{
		eulor2 = acos(R[0]); double temp = sin(eulor2);
		eulor1 = atan2(R[1] * temp, -R[2] * temp); eulor3 = atan2(R[3] * temp, R[6] * temp);
		break;
	}
	case 131:
	{
		eulor2 = acos(R[0]); double temp = sin(eulor2);
		eulor1 = atan2(R[2] * temp, -R[1] * temp); eulor3 = atan2(R[6] * temp, R[3] * temp);
		break;
	}
	case 212:
	{
		eulor2 = acos(R[4]); double temp = sin(eulor2);
		eulor1 = atan2(R[3] * temp, -R[5] * temp); eulor3 = atan2(R[1] * temp, R[7] * temp);
		break;
	}
	case 232:
	{
		eulor2 = acos(R[4]); double temp = sin(eulor2);
		eulor1 = atan2(R[5] * temp, -R[3] * temp); eulor3 = atan2(R[7] * temp, R[1] * temp);
		break;
	}
	case 313:
	{
		eulor2 = acos(R[8]); double temp = sin(eulor2);
		eulor1 = atan2(R[6] * temp, -R[7] * temp); eulor3 = atan2(R[2] * temp, R[5] * temp);
		break;
	}
	case 323:
	{
		eulor2 = acos(R[8]); double temp = sin(eulor2);
		eulor1 = atan2(R[7] * temp, -R[6] * temp); eulor3 = atan2(R[5] * temp, R[2] * temp);
		break;
	}
	//第二类：每次转动绕不同轴
	case 123:
	{
		eulor2 = asin(R[6]); double temp = cos(eulor2);
		eulor1 = atan2(-R[7] * temp, R[8] * temp); eulor3 = atan2(-R[3] * temp, R[0] * temp);
		break;
	}
	case 132:
	{
		eulor2 = asin(-R[3]); double temp = cos(eulor2);
		eulor1 = atan2(R[5] * temp, R[4] * temp); eulor3 = atan2(R[6] * temp, R[0] * temp);
		break;
	}
	case 213:
	{
		eulor2 = asin(-R[7]); double temp = cos(eulor2);
		eulor1 = atan2(R[6] * temp, R[8] * temp); eulor3 = atan2(R[1] * temp, R[4] * temp);
		break;
	}
	case 231:
	{
		eulor2 = asin(R[1]); double temp = cos(eulor2);
		eulor1 = atan2(-R[2] * temp, R[0] * temp); eulor3 = atan2(-R[7] * temp, R[4] * temp);
		break;

	}
	case 312:
	{
		eulor2 = asin(R[5]); double temp = cos(eulor2);
		eulor1 = atan2(-R[3] * temp, R[4] * temp); eulor3 = atan2(R[2] * temp, R[8] * temp);
		break;
	}
	case 321:
	{
		eulor2 = asin(-R[2]); double temp = cos(eulor2);
		eulor1 = atan2(R[1] * temp, R[0] * temp); eulor3 = atan2(R[5] * temp, R[8] * temp);
		break;
	}
	}
}

void Eulor2Matrix(double eulor1, double eulor2, double eulor3, int rotateOrder, double* R)
{
	Mat mR1, mR2, mR3, mRTemp, mR;
	switch (rotateOrder)
	{
	case 121:
	{
		RotationX(eulor1, mR1); RotationY(eulor2, mR2); RotationX(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	case 131:
	{
		RotationX(eulor1, mR1); RotationZ(eulor2, mR2); RotationX(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	case 212:
	{
		RotationY(eulor1, mR1); RotationX(eulor2, mR2); RotationY(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	case 232:
	{
		RotationY(eulor1, mR1); RotationZ(eulor2, mR2); RotationY(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	case 313:
	{
		RotationZ(eulor1, mR1); RotationX(eulor2, mR2); RotationZ(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	case 323:
	{
		RotationZ(eulor1, mR1); RotationY(eulor2, mR2); RotationZ(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	case 123:
	{
		RotationX(eulor1, mR1); RotationY(eulor2, mR2); RotationZ(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	case 132:
	{
		RotationX(eulor1, mR1); RotationZ(eulor2, mR2); RotationY(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	case 213:
	{
		RotationY(eulor1, mR1); RotationX(eulor2, mR2); RotationZ(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	case 231:
	{
		RotationY(eulor1, mR1); RotationZ(eulor2, mR2); RotationX(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	case 312:
	{
		RotationZ(eulor1, mR1); RotationX(eulor2, mR2); RotationY(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	case 321:
	{
		RotationZ(eulor1, mR1); RotationY(eulor2, mR2); RotationX(eulor3, mR3);
		mRTemp = mR2 * mR1;		mR = mR3 * mRTemp;
		memcpy(R, mR.data, 9 * sizeof(double));
		break;
	}
	default:
	{
		cout << "No such Eulor2Matrix!" << endl;
		break;
	}
	}
}

//圆点标定板参数
struct CircleBoardPara
{
	int nBoardW, nBoardH;//标定板尺寸，总宽度（mm）和总高度（mm）
	int nInterSize, nWidth, nHeight;//点间隔（mm），总列数，总行数
	int nBigCircle;//大圆个数
	int nBigD, nSmallD;//大圆直径（mm），小圆直径（mm）
	float fRes=unused;//投影分辨率（mm）如0.005，即重采样时1个像素的大小
};

//圆形标定点结构
struct CirclePoint
{
	int id=unused;
	Point2i idx = Point(unused, unused);//行列号，列在前，行在后，列向右为正，行向下为正
	Point2f imgCoor=Point2f(unused, unused);//keypoint中坐标
	Point2f fitImgCoor = Point2f(unused, unused);//subpix拟合椭圆中心坐标
	Point2f eccentricImgCoor = Point2f(unused, unused);//偏心改正后的坐标
	vector<Point2f> cntr;//subpix圆形边缘点
	RotatedRect rrcEllipse;//椭圆外包围矩形
	float size = unused;//keypoint中尺寸

	void CalcId(CircleBoardPara para)
	{
		int xAdd = (para.nWidth - 1) / 2;
		int yAdd = (para.nHeight - 1) / 2;

		if (idx.x != unused)
			this->id = (idx.y + yAdd) * para.nWidth + (idx.x + xAdd);
	}
};

//按照ID排序
bool sortCirclePointByID(const CirclePoint& cp1, const CirclePoint& cp2)
{
	if (cp1.id - cp2.id < 0)
		return true;
	else
		return false;
}

//按照标定板行列号排序
bool sortCirclePointByIdxRowCol(const CirclePoint& cp1, const CirclePoint& cp2)
{
	if (cp1.idx.y - cp2.idx.y < 0)
		return true;
	else if (cp1.idx.y - cp2.idx.y == 0 && cp1.idx.x - cp2.idx.x < 0)
		return true;
	else
		return false;
}

//按照标定板影像坐标排序
bool sortCirclePointByImgX(const CirclePoint& cp1, const CirclePoint& cp2)
{
	if (cp1.imgCoor.x - cp2.imgCoor.x < 0)
		return true;
	else
		return false;
}

//按照标定板影像坐标排序
bool sortCirclePointByImgY(const CirclePoint& cp1, const CirclePoint& cp2)
{
	if (cp1.imgCoor.y - cp2.imgCoor.y < 0)
		return true;
	else
		return false;
}

//直线参数，一般形式AX+BY+CZ=0
//点斜式y-pty=k(x-ptx)
struct LinePara
{
	double k, ptX, ptY;
	double A, B, C;
};

//提取的圆点标定板结构
struct DetectCircleBoard
{
	vector<CirclePoint> pts;//所有点

	CirclePoint A, B, C, D, E;//大圆
	CirclePoint origin;//圆点

	string boardId = "";//用影像名称作为提取标定板的ID

	void ClearData()
	{
		vector<CirclePoint>().swap(pts);
		vector<Point2f>().swap(A.cntr);
		vector<Point2f>().swap(B.cntr);
		vector<Point2f>().swap(C.cntr);
		vector<Point2f>().swap(D.cntr);
		vector<Point2f>().swap(E.cntr);
		vector<Point2f>().swap(origin.cntr);

		A.idx = Point2i(unused, unused);
		B.idx = Point2i(unused, unused);
		C.idx = Point2i(unused, unused);
		D.idx = Point2i(unused, unused);
		E.idx = Point2i(unused, unused);
		origin.idx = Point2i(unused, unused);
	}

	void FillCircleInfoFrmPts(CirclePoint& pt)
	{
		for (int i = 0; i < pts.size(); i++)
		{
			if ((pts[i].idx.x - pt.idx.x) == 0 &&
				(pts[i].idx.y - pt.idx.y) == 0)
			{
				pt.cntr = pts[i].cntr;
				pt.fitImgCoor = pts[i].fitImgCoor;
				pt.imgCoor = pts[i].imgCoor;
				pt.size = pts[i].size;
			}
		}
	}

	void CalcAllId(CircleBoardPara bPara)
	{
		int ptCnt = pts.size();
		for (int i = 0; i < ptCnt; i++)
		{
			pts[i].CalcId(bPara);
		}
	}

	void TransBoardPt(const Mat& trans)
	{
		int ptCnt = (int)pts.size();
		Mat a, b, c, y1, y2, y3;
		for (int i = 0; i < ptCnt; i++)
		{
			//Point2f img = pts[i].imgCoor;
			Point2f fit = pts[i].fitImgCoor;
			//Point2f eccentric = pts[i].eccentricImgCoor;
			
			//a = (Mat_<double>(3, 1) << img.x, img.y, 1);
			b = (Mat_<double>(3, 1) << fit.x, fit.y, 1);
			//c = (Mat_<double>(3, 1) << eccentric.x, eccentric.y, 1);
			//y1 = trans * a;
			y2 = trans * b;
			//y3 = trans * c;

			//pts[i].imgCoor = Point2f(y1.at<double>(0, 0) / y1.at<double>(2, 0), y1.at<double>(1, 0) / y1.at<double>(2, 0));
			pts[i].fitImgCoor = Point2f(y2.at<double>(0, 0) / y2.at<double>(2, 0), y2.at<double>(1, 0) / y2.at<double>(2, 0));			
			//pts[i].eccentricImgCoor = Point2f(y3.at<double>(0, 0) / y3.at<double>(2, 0), y3.at<double>(1, 0) / y3.at<double>(2, 0));

			int cntr = (int)pts[i].cntr.size();
			for (int j=0;j<cntr;j++)
			{
				Point2f img = pts[i].cntr[j];
				a = (Mat_<double>(3, 1) << img.x, img.y, 1);
				y1 = trans * a;
				pts[i].cntr[j] = Point2f(y1.at<double>(0, 0) / y1.at<double>(2, 0), y1.at<double>(1, 0) / y1.at<double>(2, 0));
			}
			pts[i].rrcEllipse = fitEllipse(pts[i].cntr);
			//pts[i].fitImgCoor = pts[i].rrcEllipse.center;
		}
	}

	void SetID(string imgName)
	{
		boardId = imgName;
	}
};

//检校影像数据结构，单帧或者多帧
struct CalibImageData
{
	vector<vector<Point2f>> imgPts;//单相机多影像
	vector<vector<Point3f>> objPts;//多影像

	CaliPtType ptType;//检校点类型，三种

	//清空数据
	void ClearData()
	{
		if (imgPts.size() > 0)
		{
			for (int i = 0; i < imgPts.size(); i++)
			{
				vector<Point2f>().swap(imgPts[i]);
			}
			vector<vector<Point2f>>().swap(imgPts);
		}

		if (objPts.size() > 0)
		{
			for (int i = 0; i < objPts.size(); i++)
			{
				vector<Point3f>().swap(objPts[i]);
			}
			vector<vector<Point3f>>().swap(objPts);
		}
	}

	void TransSingleBoard2CaliImage(CircleBoardPara bPara, DetectCircleBoard dBoard,CaliPtType type)
	{
		ptType = type;
		
		int ptCnt = dBoard.pts.size();
		double ratio = bPara.nInterSize;

		this->ClearData();
		vector<Point2f> img; vector<Point3f> obj;
		for (int i = 0; i < ptCnt; i++)
		{
			Point2i idx = dBoard.pts[i].idx;
			if (idx.x == unused || idx.y == unused)
				continue;
			Point3f ptCoor = Point3f((double)(idx.x) * ratio, (double)(idx.y) * ratio, 0.0);

			if(ptType== CaliPtType::KeyPt)
				img.emplace_back(dBoard.pts[i].imgCoor);
			else if(ptType == CaliPtType::FitPt)
				img.emplace_back(dBoard.pts[i].fitImgCoor);
			else if(ptType == CaliPtType::EccentriPt)
				img.emplace_back(dBoard.pts[i].eccentricImgCoor);

			obj.emplace_back(ptCoor);
		}
		this->imgPts.emplace_back(img);
		this->objPts.emplace_back(obj);
	}

	void TransAllBoard2CaliImage(CircleBoardPara bPara, int camNo,vector<vector<DetectCircleBoard>> vBoard, CaliPtType type)
	{
		int camCnt = vBoard.size();
		assert(camNo-camCnt<0 && camNo>=0);

		ptType = type;
		int imgCnt = vBoard[camNo].size();
		double ratio = bPara.nInterSize;
		ClearData();
		for (int n = 0; n < imgCnt; n++)
		{
			int ptCnt = vBoard[camNo][n].pts.size();
			if (ptCnt == 0)
				break;

			vector<Point2f> img; vector<Point3f> obj;
			for (int i = 0; i < ptCnt; i++)
			{
				Point2i idx = vBoard[camNo][n].pts[i].idx;
				if (idx.x == unused || idx.y == unused)
					continue;

				Point3f ptCoor = Point3f((double)(idx.x) * ratio, (double)(idx.y) * ratio, 0.0);

				if (ptType == CaliPtType::KeyPt)
					img.emplace_back(vBoard[camNo][n].pts[i].imgCoor);
				else if (ptType == CaliPtType::FitPt)
					img.emplace_back(vBoard[camNo][n].pts[i].fitImgCoor);
				else if (ptType == CaliPtType::EccentriPt)
					img.emplace_back(vBoard[camNo][n].pts[i].eccentricImgCoor);

				obj.emplace_back(ptCoor);
			}

			imgPts.emplace_back(img);
			objPts.emplace_back(obj);
		}
	}
};

//立体检校数据结构，左相机总是0号相机
struct StereoCalibImageData
{
	vector<vector<Point2f>> imgLPts,imgRPts;//左相机多影像，右相机多影像
	vector<vector<Point3f>> objPts;//多影像
	CaliPtType ptType;//检校点类型，三种

	//清空数据
	void ClearData()
	{
		if (imgLPts.size() > 0)
		{
			for (int i = 0; i < imgLPts.size(); i++)
			{
				vector<Point2f>().swap(imgLPts[i]);
			}
			vector<vector<Point2f>>().swap(imgLPts);
		}

		if (imgRPts.size() > 0)
		{
			for (int i = 0; i < imgRPts.size(); i++)
			{
				vector<Point2f>().swap(imgRPts[i]);
			}
			vector<vector<Point2f>>().swap(imgRPts);
		}

		if (objPts.size() > 0)
		{
			for (int i = 0; i < objPts.size(); i++)
			{
				vector<Point3f>().swap(objPts[i]);
			}
			vector<vector<Point3f>>().swap(objPts);
		}
	}

	void TransBoard2StereoCaliData(CircleBoardPara bPara, vector<DetectCircleBoard> boardL, vector<DetectCircleBoard> boardR, CaliPtType type)
	{
		ptType = type;
		double ratio = bPara.nInterSize;

		this->ClearData();
		
		int imgLCnt = boardL.size();
		int imgRCnt = boardR.size();
		
		string idL = "", idR = "";
		int ptLCnt = 0, ptRCnt = 0;
		int ptLId =0,ptRId =0, lastPos = 0;
		vector<Point2f> imgL, imgR;
		vector<Point3f> obj;
		Point2i idx; Point3f ptCoor;
		for (int i = 0; i <imgLCnt ; i++)
		{
			idL = boardL[i].boardId;

			vector<Point2f>().swap(imgL); vector<Point2f>().swap(imgR);
			vector<Point3f>().swap(obj);
			for (int j = 0; j < imgRCnt; j++)
			{
				idR = boardR[j].boardId;
				if (strcmp(idL.c_str(), idR.c_str()) == 0)
				{
					ptLCnt = boardL[i].pts.size();
					ptRCnt = boardR[j].pts.size();
					if (ptLCnt == 0 || ptRCnt == 0)
						continue;

					for (int k1 = 0; k1 < ptLCnt; k1++)
					{
						ptLId = boardL[i].pts[k1].id;
						for (int k2 = lastPos; k2 < ptRCnt; k2++)
						{
							ptRId = boardR[j].pts[k2].id;
							if (ptRId - ptLId == 0)
							{
								idx = boardL[i].pts[k1].idx;
								ptCoor = Point3f((double)(idx.x) * ratio, (double)(idx.y) * ratio, 0.0);
								obj.emplace_back(ptCoor);
								if (ptType == CaliPtType::KeyPt)
								{
									imgL.emplace_back(boardL[i].pts[k1].imgCoor);
									imgR.emplace_back(boardR[j].pts[k2].imgCoor);
								}
								else if (ptType == CaliPtType::FitPt)
								{
									imgL.emplace_back(boardL[i].pts[k1].fitImgCoor);
									imgR.emplace_back(boardR[j].pts[k2].fitImgCoor);
								}
								else if (ptType == CaliPtType::EccentriPt)
								{
									imgL.emplace_back(boardL[i].pts[k1].eccentricImgCoor);
									imgR.emplace_back(boardR[j].pts[k2].eccentricImgCoor);
								}

								//lastPos = k2;
								break;
							}
						}
					}

					if (imgL.size() > 0 && imgR.size()>0&& obj.size()>0)
					{
						imgLPts.push_back(imgL);
						imgRPts.push_back(imgR);
						objPts.push_back(obj);
					}
					break;
				}
			}
		}
	}
};

//反投影数据,完成数据转换，反投影变换和投影变换
struct unProjectTransData
{
	vector<Point2f> srcImgPt, unprojImgPt;
	Size unprojImgSize;

	void ClearData()
	{
		if (srcImgPt.size() > 0)
		{
			vector<Point2f>().swap(srcImgPt);
		}

		if (unprojImgPt.size() > 0)
		{
			vector<Point2f>().swap(unprojImgPt);
		}
	}

	void TransBoard2UnprojData(CircleBoardPara& bPara,Size srcImgSize, DetectCircleBoard dBoard)
	{
		int xAdd = (bPara.nWidth- 1) / 2;
		int yAdd = (bPara.nHeight- 1) / 2;
		int ptCnt = dBoard.pts.size();
		//bPara.fRes = (float)cvFloor((((float)bPara.nInterSize*(maxX - minX-1)) /(float)srcImgSize.width)*100)/100.0;
		float ratio = bPara.nInterSize / bPara.fRes;
		float zeroAdd =  bPara.nBigD / bPara.fRes;

		double minX = DBL_MAX, minY = DBL_MAX;
		double maxX = DBL_MIN, maxY = DBL_MIN;
		for (int i = 0; i < ptCnt; i++)
		{
			Point2i idx = dBoard.pts[i].idx;
			Point2f dstImgPt = Point2f((double)(idx.x + xAdd) * ratio+ zeroAdd, (double)(idx.y + yAdd) * ratio+ zeroAdd);
			if (dstImgPt.x - minX < 0)
				minX = dstImgPt.x;
			if (dstImgPt.y - minY < 0)
				minY = dstImgPt.y;

			if (dstImgPt.x - maxX > 0)
				maxX = dstImgPt.x;
			if (dstImgPt.y - maxY > 0)
				maxY = dstImgPt.y;
		}

		double subX = 0;// minX - 1.0;
		double subY = 0;// minY - 1.0;

		this->srcImgPt.clear(); this->unprojImgPt.clear();
		for (int i = 0; i < ptCnt; i++)
		{
			Point2i idx = dBoard.pts[i].idx;
			Point2f dstImgPt = Point2f((double)(idx.x + xAdd) * ratio + zeroAdd-subX, (double)(idx.y + yAdd) * ratio + zeroAdd-subY);
			this->srcImgPt.emplace_back(dBoard.pts[i].fitImgCoor);
			this->unprojImgPt.emplace_back(dstImgPt);

			if (dstImgPt.x - minX < 0)
				minX = dstImgPt.x;
			if (dstImgPt.y - minY < 0)
				minY = dstImgPt.y;
			if (dstImgPt.x - maxX > 0)
				maxX = dstImgPt.x;
			if (dstImgPt.y - maxY > 0)
				maxY = dstImgPt.y;
		}

		unprojImgSize = Size(cvCeil(maxX + zeroAdd), cvCeil(maxY + zeroAdd));
	}

	void DoSingleImgUnproj(const Mat& srcImg, Mat& unprjImg)
	{
		Mat unprojTrans = findHomography(this->srcImgPt, this->unprojImgPt);
		warpPerspective(srcImg, unprjImg, unprojTrans, unprojImgSize,INTER_LINEAR);
	}

	void DoSingleImgReproj(const Mat& srcImg, Mat& reprjImg, vector<Point2f>& reprojImgPt)
	{
		Mat reprojTrans = findHomography(this->unprojImgPt,this->srcImgPt);
		warpPerspective(srcImg, reprjImg, reprojTrans, srcImg.size(),INTER_LINEAR);

		int l = (int)srcImgPt.size();
		vector<Point2f>().swap(reprojImgPt);
		for (int j = 0; j < l; j++) 
		{
			Mat a = (Mat_<double>(3, 1) << srcImgPt[j].x, srcImgPt[j].y, 1);
			Mat y = reprojTrans * a;
			reprojImgPt.emplace_back(Point2f(y.at<double>(0, 0) / y.at<double>(2, 0), y.at<double>(1, 0) / y.at<double>(2, 0)));
		}
	}

	void GetReprojTrans(Mat& reprojTrans)
	{
		reprojTrans = findHomography(this->unprojImgPt, this->srcImgPt);
	}
};

//检校误差数据
struct CalibErrorData
{
	vector<vector<Mat>> imgResiduals;//反投到影像上的残差，先相机后影像
	vector<vector<double>> imgRMS;//统计中误差，先相机后影像
	vector<double> camRMS;//相机检校中误差
	CaliPtType ptType;//检校点类型，三种
	int camCnt;

	//清空数据
	void ClearData()
	{
		if (imgResiduals.size() > 0)
		{
			for (int i = 0; i < imgResiduals.size(); i++)
			{
				vector<Mat>().swap(imgResiduals[i]);
				vector<double>().swap(imgRMS[i]);
			}
			vector<vector<Mat>>().swap(imgResiduals);
			vector<vector<double>>().swap(imgRMS);
			vector<double>().swap(camRMS);
		}
	}

	void SetData(int Cnt)
	{
		camCnt = Cnt;
		imgResiduals.resize(camCnt);
		imgRMS.resize(camCnt);
		camRMS.resize(camCnt);
		for (int i = 0; i < camCnt; i++)
			camRMS[i] = 0.0;
	}

	void AddImgData(int camNo, const Mat& oneRes, const double oneRms)
	{
		assert(camNo >= 0 && camNo - camCnt < 0);

		imgResiduals[camNo].push_back(oneRes.clone());
		imgRMS[camNo].push_back(oneRms);
		camRMS[camNo] += oneRms * oneRms;
	}

	void CalCameraRMS()
	{
		for (int i = 0; i < camCnt; i++)
			camRMS[i] = sqrt(camRMS[i] / camCnt);
	}
};

struct CameraInitial
{
	float focal;
	pair<float, Size> camInfo;
};

struct Pos
{
	double R[9] = { 1,0,0,0,1,0,0,0,1 };
	double T[3] = { 0,0,0 };
	double Eulor[3] = { 0,0,0 };
	double rvec[3] = { 0,0,0 };

	void Mat2Eulor()
	{
		Matrix2Eulor(R, 213, Eulor[0], Eulor[1], Eulor[2]);
		Mat2RVec();
	}

	void Eulor2Mat()
	{
		Eulor2Matrix(Eulor[0], Eulor[1], Eulor[2], 213, R);
		Mat2RVec();
	}

	void RVec2Mat()
	{
		Mat rv_Mat = Mat(Size(1, 3), CV_64FC1, rvec);
		Mat R_Mat = Mat(Size(3, 3), CV_64FC1, R);
		Rodrigues(rv_Mat, R_Mat);
	}

	void Mat2RVec()
	{
		Mat rv_Mat = Mat(Size(1, 3), CV_64FC1, rvec);
		Mat R_Mat = Mat(Size(3, 3), CV_64FC1, R);
		Rodrigues(R_Mat, rv_Mat);
	}
};

struct InnPara
{
	double fx, fy, cx, cy, k1, k2, k3, p1, p2;
};

struct SingleCameraPara
{
	InnPara camInn;
	vector<Pos> camPosList;
};

struct MultiCameraPara
{
	vector<SingleCameraPara> cameraParaList;
	vector<Pos> relPos2Cam0List;
};

//单相机检校
struct SingleCalibCamera
{
	//检校结果，外参
	vector<Mat> tvecsMat, rvecsMat;
	//检校结果，内参
	Mat cameraMatrix, distCoeffs;
	//相机类型
	CameraType camType;
	//检校误差数据
	CalibErrorData errData;

	//消除畸变相关
	Mat mapX, mapY,R;
	//检校参数
	int flag = 0;
	TermCriteria criteria = TermCriteria(
		TermCriteria::COUNT + TermCriteria::EPS,
		30, DBL_EPSILON);
	//初始化检校参数
	void InitCalibration(int f, TermCriteria c)
	{
		flag = f;
		criteria = c;
	}

	//获取内参结果
	void GetCameraPara(InnPara& para)
	{
		para.fx = cameraMatrix.at<double>(0, 0);
		para.fy = cameraMatrix.at<double>(1, 1);
		para.cx = cameraMatrix.at<double>(0, 2);
		para.cy = cameraMatrix.at<double>(1, 2);
		para.k1 = distCoeffs.at<double>(0, 0);
		para.k2 = distCoeffs.at<double>(0, 1);
		para.k3 = distCoeffs.at<double>(0, 4);
		para.p1 = distCoeffs.at<double>(0, 2);
		para.p2 = distCoeffs.at<double>(0, 3);
	}

	//执行检校计算
	double DoCalibration(CalibImageData data,Size imageSize)
	{
		assert(data.imgPts.size() > 0 && data.objPts.size() > 0 && data.imgPts.size() == data.objPts.size());

		tvecsMat.clear(); rvecsMat.clear();
		double rms = calibrateCamera(data.objPts, data.imgPts, imageSize, 
			cameraMatrix, distCoeffs, rvecsMat, tvecsMat,
			flag,criteria);

		errData.ClearData();
		errData.SetData(1);

		double total_err = 0, err = 0;
		vector <Point2f> image_points2; // 保存重新计算得到的投影点
		for (int i = 0; i < data.imgPts.size(); i++) {
			vector<Point3f> tmpPointSet = data.objPts[i];
			projectPoints(tmpPointSet, rvecsMat[i], tvecsMat[i], 
				cameraMatrix, distCoeffs, image_points2);

			vector<Point2f> tmpImagePoint = data.imgPts[i];
			Mat tmpImagePointMat = Mat(1, tmpImagePoint.size(), CV_32FC2);
			Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
			int ln = (int)tmpImagePoint.size();
			for (int j = 0; j < ln; j++) {
				image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
				tmpImagePointMat.at<Vec2f>(0, j) = Vec2f(tmpImagePoint[j].x, tmpImagePoint[j].y);
			}

			err = norm(image_points2Mat, tmpImagePointMat, NORM_L2);
			err /= ln;

			errData.AddImgData(0,image_points2Mat - tmpImagePointMat,sqrt(err));
			total_err += err;			
		}

		total_err /= data.imgPts.size();
		errData.CalCameraRMS();

		return total_err;
	}

	double DoCalibration(CameraInitial initial,CalibImageData data, Size imageSize)
	{
		assert(data.imgPts.size() > 0 && data.objPts.size() > 0 && data.imgPts.size() == data.objPts.size());

		cameraMatrix = Mat::eye(3,3,CV_64FC1);
		cameraMatrix.at<double>(0, 0) = initial.focal / initial.camInfo.first;
		cameraMatrix.at<double>(1, 1) = initial.focal / initial.camInfo.first;
		cameraMatrix.at<double>(0, 2) = imageSize.width/2;
		cameraMatrix.at<double>(1, 2) = imageSize.height/2;

		tvecsMat.clear(); rvecsMat.clear();
		double rms = calibrateCamera(data.objPts, data.imgPts, imageSize,
			cameraMatrix, distCoeffs, rvecsMat, tvecsMat,
			flag, criteria);

		errData.ClearData();
		errData.SetData(1);

		double total_err = 0, err = 0;
		vector <Point2f> image_points2; // 保存重新计算得到的投影点
		for (int i = 0; i < data.imgPts.size(); i++) {
			vector<Point3f> tmpPointSet = data.objPts[i];
			projectPoints(tmpPointSet, rvecsMat[i], tvecsMat[i],
				cameraMatrix, distCoeffs, image_points2);

			vector<Point2f> tmpImagePoint = data.imgPts[i];
			Mat tmpImagePointMat = Mat(1, tmpImagePoint.size(), CV_32FC2);
			Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
			int ln = (int)tmpImagePoint.size();
			for (int j = 0; j < ln; j++) {
				image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
				tmpImagePointMat.at<Vec2f>(0, j) = Vec2f(tmpImagePoint[j].x, tmpImagePoint[j].y);
			}

			err = norm(image_points2Mat, tmpImagePointMat, NORM_L2);
			err /= ln;

			errData.AddImgData(0, image_points2Mat - tmpImagePointMat, sqrt(err));
			total_err += err;
		}

		total_err /= data.imgPts.size();
		errData.CalCameraRMS();

		return total_err;
	}

	//初始化畸变纠正参数
	void InitUndistor(Size imageSize)
	{
		assert(imageSize.width > 0 && imageSize.height > 0);

		mapX = Mat(imageSize, CV_32FC1);
		mapY = Mat(imageSize, CV_32FC1);
		R = Mat::eye(3, 3, CV_32F);
		initUndistortRectifyMap(this->cameraMatrix, this->distCoeffs, R, this->cameraMatrix, imageSize, CV_32FC1, mapX, mapY);
	}

	//初始化指定影像的带旋转的畸变纠正参数
	void InitUndistor(Size imageSize, int imgNum)
	{
		assert(imgNum >= 0 && imgNum - tvecsMat.size() < 0);

		mapX = Mat(imageSize, CV_32FC1);
		mapY = Mat(imageSize, CV_32FC1);
		R = rvecsMat[imgNum];// Mat::eye(3, 3, CV_32F);
		initUndistortRectifyMap(this->cameraMatrix, this->distCoeffs, R, this->cameraMatrix, imageSize, CV_32FC1, mapX, mapY);
	}

	//执行单张影像的畸变纠正
	void DoSingleUndistort(const Mat& srcImg,Mat& undistortImg)
	{
		assert(mapX.size().width > 0 && mapX.size().height > 0);
		assert(mapY.size().width > 0 && mapY.size().height > 0);

		remap(srcImg, undistortImg, mapX, mapY, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
	}
};

//多相机检校，包括双相机及两个以上的相机
struct MultiCalibCameras
{
	//单相机检校
	vector<SingleCalibCamera> cameras;
	//相对参数，均是相对于0号相机
	vector<Mat> relR, relT;
	//检校误差数据
	CalibErrorData errData;

	//立体检校参数
	int flag = CALIB_FIX_INTRINSIC;
	TermCriteria criteria = TermCriteria(
		TermCriteria::COUNT + TermCriteria::EPS,
		20000, DBL_EPSILON);

	//初始化相机列表
	void InitCameraList(const vector<SingleCalibCamera> camList)
	{
		int camCnt = camList.size();
		if(cameras.size()>0)
			vector<SingleCalibCamera>().swap(cameras);
		
		cameras.insert(cameras.end(),camList.begin(),camList.end());
	}

	//初始化单相机检校的参数
	void InitSingleCalibration(int f, TermCriteria c)
	{
		for (int i = 0; i < cameras.size(); i++)
		{
			cameras[i].InitCalibration(f, c);
		}
	}

	//初始化立体机检校的参数
	void InitStereoCalibration(int f, TermCriteria c)
	{
		flag = f;
		criteria = c;
	}

	//获取指定相机的内参
	void GetCameraPara(int camNo,InnPara& para)
	{
		int camCnt = cameras.size();
		assert(camNo < camCnt&& camNo >= 0);
		cameras[camNo].GetCameraPara(para);
	}

	//获取所有相机的内参
	void GetCameraPara(vector<InnPara>& paraList)
	{
		int camCnt = cameras.size();
		if (paraList.size() > 0)
			vector<InnPara>().swap(paraList);

		InnPara para;
		for (int i = 0; i < camCnt; i++)
		{
			cameras[i].GetCameraPara(para);
			paraList.push_back(para);
		}
	}

	//获取指定相机相对于0号相机的相对姿态
	void GetOneRelCamPos(int camNo, Pos& relPos)
	{
		int camCnt = cameras.size();
		assert(camNo < camCnt&& camNo > 0);

		relPos.T[0] = relT[camNo - 1].at<double>(0, 0);
		relPos.T[1] = relT[camNo - 1].at<double>(0, 1);
		relPos.T[2] = relT[camNo - 1].at<double>(0, 2);
		memcpy(relPos.R,relR[camNo - 1].data,9*sizeof(double));
		relPos.Mat2Eulor(); 
		relPos.Mat2RVec();
	}

	//获取所有相机相对于0号相机的相对姿态
	void GetAllRelCamPos(vector<Pos>& relPosList)
	{
		int relCnt = relT.size();

		if(relPosList.size()>0)
			vector<Pos>().swap(relPosList);

		Pos relPos;
		for (int i = 0; i < relT.size(); i++)
		{
			relPos.T[0] = relT[i].at<double>(0, 0);
			relPos.T[1] = relT[i].at<double>(0, 1);
			relPos.T[2] = relT[i].at<double>(0, 2);
			memcpy(relPos.R, relR[i].data, 9 * sizeof(double));
			relPos.Mat2Eulor();
			relPos.Mat2RVec();

			relPosList.push_back(relPos);
		}
	}

	//执行单相机检校
	double DoSingleCalibration(int camNo,CalibImageData data, Size imageSize)
	{
		int camCnt = cameras.size();
		assert(camNo < camCnt&& camNo >= 0);
		cameras[camNo].tvecsMat.clear(); cameras[camNo].rvecsMat.clear();
		double rms = cameras[camNo].DoCalibration(data,imageSize);
		return rms;
	}
	double DoSingleCalibration(int camNo,CameraInitial initial, CalibImageData data, Size imageSize)
	{
		int camCnt = cameras.size();
		assert(camNo < camCnt&& camNo >= 0);

		double rms = cameras[camNo].DoCalibration(initial,data, imageSize);
		return rms;
	}

	//执行立体检校，即先单相机再相对
	double DoStereoCalibration(vector<CalibImageData> allSingleData, vector<StereoCalibImageData> allStereoData,vector<Size> allImgSize)
	{
		int camCnt = cameras.size();
		assert(camCnt - allSingleData.size() == 0 && camCnt - allImgSize.size() == 0);

		//double rmsSingle=0.0;
		//for (int i = 0; i < camCnt; i++)
		//{
		//	double rms = cameras[i].DoCalibration(allSingleData[i], allImgSize[i]);
		//	rmsSingle += rms*rms;
		//}
		//rmsSingle = sqrt(rmsSingle / camCnt);

		Mat E, F;
		Mat R = Mat_<float>::zeros(3,3);
		Mat T = Mat_<float>::zeros(3,1);
		double rmsStereo = 0.0;
		for (int i = 1; i < camCnt; i++)
		{
			double rms2 = stereoCalibrate(allStereoData[i-1].objPts,allStereoData[i-1].imgLPts,allStereoData[i-1].imgRPts,
				cameras[0].cameraMatrix,cameras[0].distCoeffs,cameras[i].cameraMatrix,cameras[i].distCoeffs,
				allImgSize[i],R,T,E,F,flag,criteria);
			rmsStereo += rms2 * rms2;
			relR.push_back(R); 
			relT.push_back(T);
		}
		rmsStereo = sqrt(rmsStereo / ((double)camCnt - 1));
		return rmsStereo;
	}

	double DoStereoCalibration(vector<CameraInitial> initials,vector<CalibImageData> allSingleData, vector<StereoCalibImageData> allStereoData, vector<Size> allImgSize)
	{
		int camCnt = cameras.size();
		assert(camCnt - allSingleData.size() == 0 && camCnt - allImgSize.size() == 0);

		double rmsSingle = 0.0;
		for (int i = 0; i < camCnt; i++)
		{
			double rms = cameras[i].DoCalibration(initials[i],allSingleData[i], allImgSize[i]);
			rmsSingle += rms * rms;
		}
		rmsSingle = sqrt(rmsSingle / camCnt);

		Mat E, F;
		Mat R = Mat_<float>::zeros(3, 3);
		Mat T = Mat_<float>::zeros(3, 1);
		double rmsStereo = 0.0;
		for (int i = 1; i < camCnt; i++)
		{
			double rms2 = stereoCalibrate(allStereoData[i - 1].objPts, allStereoData[i - 1].imgLPts, allStereoData[i - 1].imgRPts,
				cameras[0].cameraMatrix, cameras[0].distCoeffs, cameras[i].cameraMatrix, cameras[i].distCoeffs,
				allImgSize[i], R, T, E, F, flag, criteria);
			rmsStereo += rms2 * rms2;
			relR.push_back(R);
			relT.push_back(T);
		}
		rmsStereo = sqrt(rmsStereo / ((double)camCnt - 1));
	}

	//初始化指定相机的畸变纠正参数
	void InitUndistor(int camNo, Size imageSize)
	{
		int camCnt = cameras.size();
		assert(camNo < camCnt&& camNo >= 0);
		cameras[camNo].InitUndistor(imageSize);
	}

	//初始化指定相机指定影像的带旋转的畸变纠正参数
	void InitUndistor(int camNo, Size imageSize, int imgNum)
	{
		int camCnt = cameras.size();
		assert(camNo < camCnt&& camNo >= 0);
		cameras[camNo].InitUndistor(imageSize, imgNum);
	}

	//执行单张影像的畸变纠正
	void DoSingleUndistort(int camNo, const Mat& srcImg, Mat& undistortImg)
	{
		int camCnt = cameras.size();
		assert(camNo < camCnt&& camNo >= 0);
		cameras[camNo].DoSingleUndistort(srcImg, undistortImg);
	}
};

struct CaliProject
{
	string imgFolder = "";
	string tmpFolder = "";
	string coverFolder = "";
	string pdfFolder = "";
	string txtFolder = "";
	string iterprojFolder = "";

	vector<string> imgFolders, tmpFolders, coverFolders, pdfFolders, txtFolders, iterprojFolders;
	CircleBoardPara bPara;
	bool bCreateSub = true;
	Size coverageSize = Size(1, 1);
	int maxIterCnt = 10;
	int camCnt = 1;

	//斑点检测阈值
	SimpleBlobDetector::Params params;
	//斑点检测指针
	Ptr<SimpleBlobDetector> detector;

	//处理的影像数目
	int procImgCnt;

	vector<CameraInitial> caliInitials;

	//初始化斑点检测
	void InitBlobDetector()
	{
		//阈值控制
		params.thresholdStep = 5;
		params.minThreshold = 10;
		params.maxThreshold = 200;
		params.filterByColor = true;
		params.blobColor = 0;
		//像素面积大小控制
		params.filterByArea = true;
		params.minArea = 50;
		params.maxArea = 30000;
		//形状（凸）
		params.filterByCircularity = false;
		params.minCircularity = 0.7;
		//形状（凹）
		params.filterByConvexity = false;
		params.minConvexity = 0.9;
		//形状（园）
		params.filterByInertia = true;
		params.minInertiaRatio = 0.4;

		detector = SimpleBlobDetector::create(params);
		procImgCnt = 0;
	}

	//初始化子文件夹
	void InitDetectSubDir(int camCnt)
	{
		vector<string>().swap(tmpFolders);
		vector<string>().swap(coverFolders);
		vector<string>().swap(pdfFolders);
		vector<string>().swap(txtFolders);
		vector<string>().swap(iterprojFolders);

		tmpFolders.resize(camCnt);
		coverFolders.resize(camCnt);
		pdfFolders.resize(camCnt);
		txtFolders.resize(camCnt);
		iterprojFolders.resize(camCnt);

		for (int i = 0; i < camCnt; i++)
		{
			tmpFolder = imgFolder + "temp\\";
			coverFolder = imgFolder + "cover\\";
			pdfFolder = imgFolder + "subpix\\";
			txtFolder = imgFolder + "cbt\\";
			iterprojFolder = imgFolder + "iterProj\\";

			tmpFolders[i] = imgFolders[i] + "temp\\";
			coverFolders[i] = imgFolders[i] + "cover\\";
			pdfFolders[i] = imgFolders[i] + "subpix\\";
			txtFolders[i] = imgFolders[i] + "cbt\\";
			iterprojFolders[i] = imgFolders[i] + "iterProj\\";

			if (bCreateSub)
			{
				CreateFolder(tmpFolders[i]);
				CreateFolder(coverFolders[i]);
				CreateFolder(pdfFolders[i]);
				CreateFolder(txtFolders[i]);
				CreateFolder(iterprojFolders[i]);
			}
		}
	}
};