#pragma once
#include "CaliStruct.h"

//找出5个大圆和原点
void FindFiveBigCircleAndOrigin(CircleBoardPara bPara,DetectCircleBoard& dBoard)
{
    //距离最远的是BE，最近的是CD，剩下的是A
    double maxDist = 0.0; pair<int, int> pDistMax;
    double minDist = DBL_MAX; pair<int, int>pDistMin;
    for (int i = 0; i < bPara.nBigCircle; i++)
    {
        for (int j = i + 1; j < bPara.nBigCircle; j++)
        {
            double dist = calcLength(dBoard.pts[i].imgCoor.x, dBoard.pts[i].imgCoor.y, 
                dBoard.pts[j].imgCoor.x, dBoard.pts[j].imgCoor.y);

            if (dist - maxDist > 0)
            {
                maxDist = dist;
                pDistMax.first = i;
                pDistMax.second = j;
            }
            if (dist - minDist < 0)
            {
                minDist = dist;
                pDistMin.first = i;
                pDistMin.second = j;
            }
        }
    }

    int idxA = 10 - (pDistMin.first+pDistMin.second) - (pDistMax.first+pDistMax.second);
    dBoard.A = dBoard.pts[idxA];
    dBoard.A.idx = Point2i(0, 2);
    dBoard.A.CalcId(bPara);
    dBoard.pts[idxA].idx = dBoard.A.idx;
    dBoard.pts[idxA].CalcId(bPara);

    //CD中距离A较大的是C，剩下的是D
    double Dist1 = calcLength(dBoard.pts[pDistMin.first].imgCoor.x, 
        dBoard.pts[pDistMin.first].imgCoor.y,dBoard.A.imgCoor.x,dBoard.A.imgCoor.y);
    double Dist2 = calcLength(dBoard.pts[pDistMin.second].imgCoor.x,
        dBoard.pts[pDistMin.second].imgCoor.y, dBoard.A.imgCoor.x, dBoard.A.imgCoor.y);
    int idxC = (Dist1 - Dist2) > 0 ? pDistMin.first : pDistMin.second;
    int idxD = (Dist1 - Dist2) > 0 ? pDistMin.second : pDistMin.first;

    dBoard.C = dBoard.pts[idxC];
    dBoard.C.idx = Point2i(-1, -2);
    dBoard.C.CalcId(bPara);
    dBoard.pts[idxC].idx = dBoard.C.idx;
    dBoard.pts[idxC].CalcId(bPara);

    dBoard.D = dBoard.pts[idxD];
    dBoard.D.idx = Point2i(0, -2);
    dBoard.D.CalcId(bPara);
    dBoard.pts[idxD].idx = dBoard.D.idx;
    dBoard.pts[idxD].CalcId(bPara);

    //BE中距离C较近的是B，剩下的是E
    Dist1 = calcLength(dBoard.pts[pDistMax.first].imgCoor.x,
        dBoard.pts[pDistMax.first].imgCoor.y, dBoard.C.imgCoor.x, dBoard.C.imgCoor.y);
    Dist2 = calcLength(dBoard.pts[pDistMax.second].imgCoor.x,
        dBoard.pts[pDistMax.second].imgCoor.y, dBoard.C.imgCoor.x, dBoard.C.imgCoor.y);
    int idxB= (Dist1 - Dist2) > 0 ? pDistMax.second : pDistMax.first;
    int idxE = (Dist1 - Dist2) > 0 ? pDistMax.first : pDistMax.second;

    dBoard.B = dBoard.pts[idxB];
    dBoard.B.idx = Point2i(-3, 0);
    dBoard.B.CalcId(bPara);
    dBoard.pts[idxB].idx = dBoard.B.idx;
    dBoard.pts[idxB].CalcId(bPara);

    dBoard.E = dBoard.pts[idxE];
    dBoard.E.idx = Point2i(3, 0);
    dBoard.E.CalcId(bPara);
    dBoard.pts[idxE].idx = dBoard.E.idx;
    dBoard.pts[idxE].CalcId(bPara);

    //距离ABDE最近的是原点
    int idxO = -1; minDist = DBL_MAX;
    double XA = dBoard.A.imgCoor.x; double YA = dBoard.A.imgCoor.y;
    double XB = dBoard.B.imgCoor.x; double YB = dBoard.B.imgCoor.y;
    double XD = dBoard.D.imgCoor.x; double YD = dBoard.D.imgCoor.y;
    double XE = dBoard.E.imgCoor.x; double YE = dBoard.E.imgCoor.y;

    for (int i=0;i<dBoard.pts.size();i++)
    {
        //X坐标在BE之间
        if ((dBoard.pts[i].imgCoor.x-XB)*(dBoard.pts[i].imgCoor.x - XE)<0)
        {
        //Y坐标在AD之间
            if ((dBoard.pts[i].imgCoor.y - YA) * (dBoard.pts[i].imgCoor.y - YD) < 0)
            {
                double distA = calcLength(dBoard.pts[i].imgCoor.x, dBoard.pts[i].imgCoor.y, XA, YA);
                double distB = calcLength(dBoard.pts[i].imgCoor.x, dBoard.pts[i].imgCoor.y, XB, YB);
                double distD = calcLength(dBoard.pts[i].imgCoor.x, dBoard.pts[i].imgCoor.y, XD, YD);
                double distE = calcLength(dBoard.pts[i].imgCoor.x, dBoard.pts[i].imgCoor.y, XE, YE);

                if ((distA+distB+distD+distE)-minDist<0)
                {
                    minDist = distA + distB + distD + distE;
                    idxO = i;
                }
            }
        }
    }

    dBoard.origin = dBoard.pts[idxO];
    dBoard.origin.idx = Point2i(0, 0);
    dBoard.origin.CalcId(bPara);
    dBoard.pts[idxO].idx = dBoard.origin.idx;
    dBoard.pts[idxO].CalcId(bPara);
}

//通过共线的两点，找到直线上其他点，距离限值delta，并确定行列号
void FindLinePtWith2Point(int nBigCnt,DetectCircleBoard& dBoard, CirclePoint pt1, CirclePoint pt2,double delta,vector<CirclePoint>& linePt,vector<int>& lineIdx)
{
    double A = (double)pt2.imgCoor.y - pt1.imgCoor.y;
    double B = (double)pt1.imgCoor.x - pt2.imgCoor.x;
    double C = (double)pt2.imgCoor.x * pt1.imgCoor.y - pt1.imgCoor.x * pt2.imgCoor.y;
    double AABB = sqrt(A * A + B * B);

    linePt.clear();
    lineIdx.clear();
    for (int i= 0;i<dBoard.pts.size();i++)
    {
        double dist2Line = fabs(A*dBoard.pts[i].imgCoor.x+B*dBoard.pts[i].imgCoor.y+C) / AABB;
        if (dist2Line - delta < 0)
        {
            linePt.emplace_back(dBoard.pts[i]);
            lineIdx.emplace_back(i);
        }
    }
    return;
}

void GetHoriCircleLineWith2Point(DetectCircleBoard& dBoard, vector<CirclePoint>& linePt, vector<int> ptIdx, CirclePoint pt1, CirclePoint pt2)
{
    double dist = calcLength(pt1.imgCoor.x, pt1.imgCoor.y, pt2.imgCoor.x, pt2.imgCoor.y);
    double oneColLen = dist / abs(pt1.idx.x - pt2.idx.x);

    int row = pt1.idx.y; int col = 0;
    vector<int> preCol;
    for (int i = 0; i < linePt.size(); i++)
    {
        if (fabs(linePt[i].imgCoor.x - pt1.imgCoor.x) - fabs(linePt[i].imgCoor.x - pt2.imgCoor.x) < 0)
        {
            dist = (linePt[i].imgCoor.x - pt1.imgCoor.x > 0) ?
                calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, pt1.imgCoor.x, pt1.imgCoor.y) :
                -calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, pt1.imgCoor.x, pt1.imgCoor.y);
            col = cvRound(dist / oneColLen) + pt1.idx.x;
        }
        else
        {
            dist = (linePt[i].imgCoor.x - pt2.imgCoor.x > 0) ?
                calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, pt2.imgCoor.x, pt2.imgCoor.y) :
                -calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, pt2.imgCoor.x, pt2.imgCoor.y);
            col = cvRound(dist / oneColLen) + pt2.idx.x;
        }

        if (i >= 1)
        {
            bool bSame = false;
            for (int j = 0; j < preCol.size(); j++)
            {
                if (preCol[j] - col == 0)
                {
                    bSame = true;
                    col = col + 1;
                    break;
                }
            }
        }
        int idx = ptIdx[i];
        dBoard.pts[idx].idx = Point(col, row);
        linePt[i].idx = Point(col, row);

        preCol.emplace_back(col);
    }
    return;
}

void GetHoriCircleLineWith2Point2(int nHalfBoardW,DetectCircleBoard& dBoard, vector<CirclePoint>& linePt, vector<int> ptIdx, CirclePoint pt1, CirclePoint pt2)
{
    float x1 = pt1.imgCoor.x,x2 = pt2.imgCoor.x;
    float col1 = pt1.idx.x,col2 = pt2.idx.x;
    
    int row = pt1.idx.y; 
    vector<int> preCol;
    for (int i = 0; i < linePt.size(); i++)
    {
        float x = linePt[i].imgCoor.x;
        int tmpCol = cvRound(((x - x2) * col1 - (x - x1) * col2) / (x1 - x2));
        if (abs(tmpCol) - nHalfBoardW > 0)
            tmpCol = (tmpCol > 0) ? nHalfBoardW : -nHalfBoardW;
        if (i >= 1)
        {
            bool bSame = false;
            for (int j = 0; j < preCol.size(); j++)
            {
                if (preCol[j] - tmpCol == 0)
                {
                    bSame = true;
                    tmpCol = tmpCol + 1;
                    break;
                }
            }
        }

        int idx = ptIdx[i];
        dBoard.pts[idx].idx = Point(tmpCol, row);
        linePt[i].idx = Point(tmpCol, row);
        preCol.emplace_back(tmpCol);
    }

    return;
}

void GetHoriCircleLineWith2Point3(int nHalfBoardW, DetectCircleBoard& dBoard, vector<CirclePoint>& linePt, vector<int> ptIdx, CirclePoint pt1, CirclePoint pt2)
{
    float x1 = dBoard.A.imgCoor.x, x2 = dBoard.B.imgCoor.x, x3 = dBoard.C.imgCoor.x;
    float x4 = dBoard.D.imgCoor.x, x5 = dBoard.E.imgCoor.x, x6 = dBoard.origin.imgCoor.x;

    float col1 = dBoard.A.idx.x, col2 = dBoard.B.idx.x, col3 = dBoard.C.idx.x;
    float col4 = dBoard.D.idx.x, col5 = dBoard.E.idx.x, col6 = dBoard.origin.idx.x;

    int row = pt1.idx.y;
    vector<int> preCol;
    float x; double dist1, dist2;
    int tmpCol1, tmpCol2, tmpCol;
    for (int i = 0; i < linePt.size(); i++)
    {
        x = linePt[i].imgCoor.x;
        dist1 = calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, dBoard.C.imgCoor.x, dBoard.C.imgCoor.y);
        dist2 = calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, dBoard.D.imgCoor.x, dBoard.D.imgCoor.y);
        tmpCol1 = cvFloor(((x - x3) * col4 - (x - x4) * col3) / (x4 - x3));

        dist1 = calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, dBoard.B.imgCoor.x, dBoard.B.imgCoor.y);
        dist2 = calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, dBoard.E.imgCoor.x, dBoard.E.imgCoor.y);
        tmpCol2 = cvFloor(((x - x2) * col5 - (x - x5) * col2) / (x5 - x2));
        tmpCol = (tmpCol1 - tmpCol2) == 0 ? tmpCol1 : (tmpCol1+tmpCol2)/2;

        if (abs(tmpCol) - nHalfBoardW > 0)
            tmpCol = (tmpCol > 0) ? nHalfBoardW : -nHalfBoardW;
        if (i >= 1)
        {
            bool bSame = false;
            for (int j = 0; j < preCol.size(); j++)
            {
                if (preCol[j] - tmpCol == 0)
                {
                    bSame = true;
                    tmpCol = tmpCol + 1;
                    break;
                }
            }
        }

        int idx = ptIdx[i];
        dBoard.pts[idx].idx = Point(tmpCol, row);
        linePt[i].idx = Point(tmpCol, row);
        preCol.emplace_back(tmpCol);
    }

    return;
}

void GetHoriCircleLineWith2PointUseHomo(Mat homo,CircleBoardPara bPara,DetectCircleBoard& dBoard, vector<CirclePoint>& linePt, vector<int> ptIdx, CirclePoint pt1, CirclePoint pt2)
{
    int xAdd = (bPara.nWidth + 1) / 2;
    int yAdd = (bPara.nHeight + 1) / 2;
    int nHalfBoardW = (bPara.nWidth - 1) / 2;

    int row = pt1.idx.y,tmpCol;
    Mat a, y; Point2f prePt;
    for (int i = 0; i < linePt.size(); i++)
    {
        a = (Mat_<double>(3, 1) << linePt[i].imgCoor.x, linePt[i].imgCoor.y, 1);
        y = homo * a;
        prePt = Point2f(y.at<double>(0, 0) / y.at<double>(2, 0), y.at<double>(1, 0) / y.at<double>(2, 0));
        prePt.x = prePt.x / (double)bPara.nInterSize-xAdd;
        prePt.y = prePt.y / (double)bPara.nInterSize-yAdd;
        tmpCol = cvRound(prePt.x);

        if (abs(tmpCol) - nHalfBoardW > 0)
            tmpCol = (tmpCol > 0) ? nHalfBoardW : -nHalfBoardW;
       
        int idx = ptIdx[i];
        dBoard.pts[idx].idx = Point(tmpCol, row);
        linePt[i].idx = Point(tmpCol, row);
    }

    return;
}

void GetVertiCircleLineWith2Point(DetectCircleBoard& dBoard,vector<CirclePoint>& linePt,vector<int> ptIdx,CirclePoint pt1,CirclePoint pt2)
{
    double dist = calcLength(pt1.imgCoor.x,pt1.imgCoor.y,pt2.imgCoor.x,pt2.imgCoor.y);
    double oneRowLen = dist/abs(pt1.idx.y-pt2.idx.y);

    int col = pt1.idx.x; int row = 0;
    vector<int> preRow;
    for (int i=0;i<linePt.size();i++)
    {
        if (fabs(linePt[i].imgCoor.x - pt1.imgCoor.y) - fabs(linePt[i].imgCoor.x - pt2.imgCoor.y) < 0)
        {
            dist = (linePt[i].imgCoor.y - pt1.imgCoor.y > 0) ?
                calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, pt1.imgCoor.x, pt1.imgCoor.y) :
                -calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, pt1.imgCoor.x, pt1.imgCoor.y);
           row = cvRound(dist/oneRowLen) + pt1.idx.y;
        }
        else
        {
            dist = (linePt[i].imgCoor.y - pt2.imgCoor.y > 0) ?
                calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, pt2.imgCoor.x, pt2.imgCoor.y) :
                -calcLength(linePt[i].imgCoor.x, linePt[i].imgCoor.y, pt2.imgCoor.x, pt2.imgCoor.y);

            row = cvRound(dist / oneRowLen) + pt2.idx.y;
        }

        if (i >= 1)
        {
            bool bSame = false;
            for (int j = 0; j < preRow.size(); j++)
            {
                if (preRow[j] - row == 0)
                {
                    bSame = true;
                    row = row + 1;
                    break;
                }
            }
        }
        int idx = ptIdx[i];
        dBoard.pts[idx].idx = Point(col, row);
        linePt[i].idx = Point(col, row);
        preRow.emplace_back(row);
    }
    return;
}

void GetVertiCircleLineWith2Point2(int nHalfBoardH, DetectCircleBoard& dBoard, vector<CirclePoint>& linePt, vector<int> ptIdx, CirclePoint pt1, CirclePoint pt2)
{
    double dist = calcLength(pt1.imgCoor.x, pt1.imgCoor.y, pt2.imgCoor.x, pt2.imgCoor.y);
    double oneRowLen = dist / abs(pt1.idx.y - pt2.idx.y);

    float y1 = pt1.imgCoor.y, y2 = pt2.imgCoor.y;
    float row1 = pt1.idx.y, row2 = pt2.idx.y;
    
    int col = pt1.idx.x; int row = 0,nextrow=0,prerow=0;
    vector<int> preRow;
    for (int i = 0; i < linePt.size(); i++)
    {
        float y = linePt[i].imgCoor.y;
        int tmpRow = cvRound(((y - y2) * row1 - (y - y1) * row2) / (y1 - y2));
        if (abs(tmpRow) - nHalfBoardH > 0)
            tmpRow = (tmpRow > 0) ? nHalfBoardH : -nHalfBoardH;

        if (i >= 1)
        {
            bool bSame = false;
            for (int j = 0; j < preRow.size(); j++)
            {
                if (preRow[j] - tmpRow == 0)
                {
                    bSame = true;
                    tmpRow = tmpRow + 1;
                    break;
                }
            }
        }
        int idx = ptIdx[i];
        dBoard.pts[idx].idx = Point(col, tmpRow);
        linePt[i].idx = Point(col, tmpRow);
        preRow.emplace_back(tmpRow);
    }

    return;
}

void GetVertiCircleLineWith2PointUseHomo(Mat homo, CircleBoardPara bPara, DetectCircleBoard& dBoard, vector<CirclePoint>& linePt, vector<int> ptIdx, CirclePoint pt1, CirclePoint pt2)
{
    int xAdd = (bPara.nWidth + 1) / 2;
    int yAdd = (bPara.nHeight + 1) / 2;
    int nHalfBoardH = (bPara.nHeight - 1) / 2;

    int col = pt1.idx.x;     
    Mat a, y; Point2f prePt;

    for (int i = 0; i < linePt.size(); i++)
    {
        a = (Mat_<double>(3, 1) << linePt[i].imgCoor.x, linePt[i].imgCoor.y, 1);
        y = homo * a;
        prePt = Point2f(y.at<double>(0, 0) / y.at<double>(2, 0), y.at<double>(1, 0) / y.at<double>(2, 0));
        prePt.x = prePt.x / (double)bPara.nInterSize - xAdd;
        prePt.y = prePt.y / (double)bPara.nInterSize - yAdd;
        int tmpRow = cvRound(prePt.y);

        if (abs(tmpRow) - nHalfBoardH > 0)
            tmpRow = (tmpRow > 0) ? nHalfBoardH : -nHalfBoardH;

        int idx = ptIdx[i];
        dBoard.pts[idx].idx = Point(col, tmpRow);
        linePt[i].idx = Point(col, tmpRow);
    }

    return;
}

void CheckUnsedPt(Mat homo, CircleBoardPara bPara, DetectCircleBoard& dBoard)
{
    int xAdd = (bPara.nWidth + 1) / 2;
    int yAdd = (bPara.nHeight + 1) / 2;
    int nHalfBoardH = (bPara.nHeight - 1) / 2;
    int nHalfBoardW = (bPara.nWidth - 1) / 2;

    Mat a, y; Point2f prePt;
    for (int i = 0; i < dBoard.pts.size(); i++)
    {
        if (dBoard.pts[i].idx.x == unused || dBoard.pts[i].idx.y == unused)
        {
            a = (Mat_<double>(3, 1) << dBoard.pts[i].imgCoor.x, dBoard.pts[i].imgCoor.y, 1);
            y = homo * a;
            prePt = Point2f(y.at<double>(0, 0) / y.at<double>(2, 0), y.at<double>(1, 0) / y.at<double>(2, 0));
            prePt.x = prePt.x / (double)bPara.nInterSize - xAdd;
            prePt.y = prePt.y / (double)bPara.nInterSize - yAdd;
            int tmpRow = cvRound(prePt.y);
            int tmpCol = cvRound(prePt.x);

            if (abs(tmpRow) - nHalfBoardH > 0)
                tmpRow = (tmpRow > 0) ? nHalfBoardH : -nHalfBoardH;
            if (abs(tmpCol) - nHalfBoardW > 0)
                tmpCol = (tmpCol > 0) ? nHalfBoardW : -nHalfBoardW;

            dBoard.pts[i].idx = Point(tmpCol, tmpRow);
        }
    }

    return;
}

void SortHoriLinePtWithIdx(vector<CirclePoint>& linePt, vector<int>& lineIdx)
{
    vector<pair<float, int>> vpXIdx;
    for (int i = 0; i < lineIdx.size(); i++)
    {
        vpXIdx.emplace_back(make_pair(linePt[i].imgCoor.x, lineIdx[i]));
    }
    sort(linePt.begin(), linePt.end(), sortCirclePointByImgX);
    sort(vpXIdx.begin(), vpXIdx.end(), sortPairIdX);

    lineIdx.clear();
    for (int i = 0; i < vpXIdx.size(); i++)
    {
        lineIdx.emplace_back(vpXIdx[i].second);
    }
}
void SortVertiLinePtWithIdx(vector<CirclePoint>& linePt, vector<int>& lineIdx)
{
    vector<pair<float, int>> vpYIdx;
    for (int i = 0; i < lineIdx.size(); i++)
    {
        vpYIdx.emplace_back(make_pair(linePt[i].imgCoor.y, lineIdx[i]));
    }
    sort(linePt.begin(), linePt.end(), sortCirclePointByImgY);
    sort(vpYIdx.begin(), vpYIdx.end(), sortPairIdX);

    lineIdx.clear();
    for (int i = 0; i < vpYIdx.size(); i++)
    {
        lineIdx.emplace_back(vpYIdx[i].second);
    }
}

//获取所有直线
void FindAllLines(CircleBoardPara bPara,DetectCircleBoard& dBoard)
{
    double distOA = calcLength(dBoard.origin.imgCoor.x, dBoard.origin.imgCoor.y, 
        dBoard.A.imgCoor.x, dBoard.A.imgCoor.y);
    double distOE = calcLength(dBoard.origin.imgCoor.x, dBoard.origin.imgCoor.y, 
        dBoard.E.imgCoor.x, dBoard.E.imgCoor.y);

    vector<CirclePoint> lineBE, lineCD;
    vector<int> lineBEIdx, lineCDIdx;
    double deltaCol = distOE / 3;
    double deltaRow = distOA / 2;
    int nHalfBoardW = (bPara.nWidth - 1) / 2;
    int nHalfBoardH = (bPara.nHeight - 1) / 2;

   //lineBE.emplace_back(dBoard.B); lineBE.emplace_back(dBoard.E);
   FindLinePtWith2Point(bPara.nBigCircle,dBoard, dBoard.B, dBoard.E, 0.5*deltaCol, lineBE,lineBEIdx);
   SortHoriLinePtWithIdx(lineBE, lineBEIdx);
   if(dBoard.B.imgCoor.x-dBoard.E.imgCoor.x<0)
       GetHoriCircleLineWith2Point3(nHalfBoardW,dBoard, lineBE, lineBEIdx, dBoard.B, dBoard.E);
   else
       GetHoriCircleLineWith2Point3(nHalfBoardW,dBoard, lineBE, lineBEIdx, dBoard.E, dBoard.B);
   //sort(lineBE.begin(), lineBE.end(), sortCirclePointByImgX);

   // lineCD.emplace_back(dBoard.C); lineCD.emplace_back(dBoard.D);
    FindLinePtWith2Point(bPara.nBigCircle,dBoard, dBoard.C, dBoard.D, 0.5 * deltaCol, lineCD,lineCDIdx);
    SortHoriLinePtWithIdx(lineCD, lineCDIdx);
    if (dBoard.C.imgCoor.x - dBoard.D.imgCoor.x < 0)
        GetHoriCircleLineWith2Point3(nHalfBoardW,dBoard,lineCD,lineCDIdx, dBoard.C, dBoard.D);
    else
        GetHoriCircleLineWith2Point3(nHalfBoardW,dBoard, lineCD, lineCDIdx, dBoard.D, dBoard.C);
   // sort(lineCD.begin(), lineCD.end(), sortCirclePointByImgX);

    int start = 0; int size = 0;
    int sizeBE = lineBE.size(), sizeCD = lineCD.size();
    if (lineBE[0].idx.x - lineCD[0].idx.x <= 0)
    {
        for (int i = 0; i < lineBE.size(); i++)
        {
            if (lineBE[i].idx.x - lineCD[0].idx.x == 0)
            {
                start = i;
                break;
            }
        }

        size = (sizeBE -start - sizeCD) < 0 ? (sizeBE -start) : sizeCD;
        for (int i = 0; i < size; i++)
        {
            int newi = start + i;
            if (lineBE[newi].idx.x - lineCD[i].idx.x != 0)
                continue;

            vector<CirclePoint> curLine;
            vector<int> curLineIdx;
            FindLinePtWith2Point(bPara.nBigCircle, dBoard, lineBE[newi], lineCD[i], 0.5*deltaCol, curLine, curLineIdx);
            SortVertiLinePtWithIdx(curLine, curLineIdx);
            if (lineBE[newi].imgCoor.y - lineCD[i].imgCoor.y < 0)
                GetVertiCircleLineWith2Point2(nHalfBoardH, dBoard, curLine, curLineIdx, lineBE[newi], lineCD[i]);
            else
                GetVertiCircleLineWith2Point2(nHalfBoardH, dBoard, curLine, curLineIdx, lineCD[i], lineBE[newi]);
           // sort(curLine.begin(), curLine.end(), sortCirclePointByImgY);

        }
    }
    else if (lineBE[0].idx.x - lineCD[0].idx.x > 0)
    {
        for (int i = 0; i < lineCD.size(); i++)
        {
            if (lineCD[i].idx.x - lineBE[0].idx.x == 0)
            {
                start = i;
                break;
            }
        }
        size = ((sizeCD - start) - sizeBE) < 0 ? (sizeCD - start) : sizeBE;
        for (int i = 0; i < size; i++)
        {
            int newi = start + i;
            if (lineCD[newi].idx.x - lineBE[i].idx.x != 0)
                continue;

            vector<CirclePoint> curLine;
            vector<int> curLineIdx;
            FindLinePtWith2Point(bPara.nBigCircle, dBoard, lineBE[i], lineCD[newi], 0.5*deltaCol, curLine, curLineIdx);
            SortVertiLinePtWithIdx(curLine, curLineIdx);
            if (lineBE[newi].imgCoor.y - lineCD[i].imgCoor.y < 0)
                GetVertiCircleLineWith2Point2(nHalfBoardH, dBoard, curLine, curLineIdx, lineBE[i], lineCD[newi]);
            else
                GetVertiCircleLineWith2Point2(nHalfBoardH, dBoard, curLine, curLineIdx, lineCD[i], lineBE[newi]);
          //  sort(curLine.begin(), curLine.end(), sortCirclePointByImgY);
        }
    }

    sort(dBoard.pts.begin(), dBoard.pts.end(), sortCirclePointByIdxRowCol);
}

void FindAllLinesUseHomo(CircleBoardPara bPara, DetectCircleBoard& dBoard,Mat homo)
{
    double distOA = calcLength(dBoard.origin.imgCoor.x, dBoard.origin.imgCoor.y,
        dBoard.A.imgCoor.x, dBoard.A.imgCoor.y);
    double distOE = calcLength(dBoard.origin.imgCoor.x, dBoard.origin.imgCoor.y,
        dBoard.E.imgCoor.x, dBoard.E.imgCoor.y);

    vector<CirclePoint> lineBE, lineCD;
    vector<int> lineBEIdx, lineCDIdx;
    double deltaCol = distOE / 3;
    double deltaRow = distOA / 2;
    int nHalfBoardW = (bPara.nWidth - 1) / 2;
    int nHalfBoardH = (bPara.nHeight - 1) / 2;

    //lineBE.emplace_back(dBoard.B); lineBE.emplace_back(dBoard.E);
    FindLinePtWith2Point(bPara.nBigCircle, dBoard, dBoard.B, dBoard.E, 0.5 * deltaCol, lineBE, lineBEIdx);
    SortHoriLinePtWithIdx(lineBE, lineBEIdx);
    GetHoriCircleLineWith2PointUseHomo(homo, bPara, dBoard, lineBE, lineBEIdx, dBoard.B, dBoard.E);
    //sort(lineBE.begin(), lineBE.end(), sortCirclePointByImgX);

    // lineCD.emplace_back(dBoard.C); lineCD.emplace_back(dBoard.D);
    FindLinePtWith2Point(bPara.nBigCircle, dBoard, dBoard.C, dBoard.D, 0.5 * deltaCol, lineCD, lineCDIdx);
    SortHoriLinePtWithIdx(lineCD, lineCDIdx);
    GetHoriCircleLineWith2PointUseHomo(homo, bPara, dBoard, lineCD, lineCDIdx, dBoard.C, dBoard.D);
    // sort(lineCD.begin(), lineCD.end(), sortCirclePointByImgX);

    int start = 0; int size = 0;
    int sizeBE = lineBE.size(), sizeCD = lineCD.size();
    if (lineBE[0].idx.x - lineCD[0].idx.x <= 0)
    {
        for (int i = 0; i < lineBE.size(); i++)
        {
            if (lineBE[i].idx.x - lineCD[0].idx.x == 0)
            {
                start = i;
                break;
            }
        }

        size = (sizeBE - start - sizeCD) < 0 ? (sizeBE - start) : sizeCD;
        for (int i = 0; i < size; i++)
        {
            int newi = start + i;
            if (lineBE[newi].idx.x - lineCD[i].idx.x != 0)
                continue;

            vector<CirclePoint> curLine;
            vector<int> curLineIdx;
            FindLinePtWith2Point(bPara.nBigCircle, dBoard, lineBE[newi], lineCD[i], 0.5 * deltaCol, curLine, curLineIdx);
            SortVertiLinePtWithIdx(curLine, curLineIdx);
            GetVertiCircleLineWith2PointUseHomo(homo,bPara, dBoard, curLine, curLineIdx, lineBE[newi], lineCD[i]);
            // sort(curLine.begin(), curLine.end(), sortCirclePointByImgY);

        }
    }
    else if (lineBE[0].idx.x - lineCD[0].idx.x > 0)
    {
        for (int i = 0; i < lineCD.size(); i++)
        {
            if (lineCD[i].idx.x - lineBE[0].idx.x == 0)
            {
                start = i;
                break;
            }
        }
        size = ((sizeCD - start) - sizeBE) < 0 ? (sizeCD - start) : sizeBE;
        for (int i = 0; i < size; i++)
        {
            int newi = start + i;
            if (lineCD[newi].idx.x - lineBE[i].idx.x != 0)
                continue;

            vector<CirclePoint> curLine;
            vector<int> curLineIdx;
            FindLinePtWith2Point(bPara.nBigCircle, dBoard, lineBE[i], lineCD[newi], 0.5 * deltaCol, curLine, curLineIdx);
            SortVertiLinePtWithIdx(curLine, curLineIdx);
            GetVertiCircleLineWith2PointUseHomo(homo,bPara, dBoard, curLine, curLineIdx, lineBE[i], lineCD[newi]);
            //  sort(curLine.begin(), curLine.end(), sortCirclePointByImgY);
        }
    }

    CheckUnsedPt(homo, bPara, dBoard);
    sort(dBoard.pts.begin(), dBoard.pts.end(), sortCirclePointByIdxRowCol);
}

//通过共线的两点，找到直线上其他点，距离限值delta，并确定行列号
void FindPtWithLinePoint(CircleBoardPara bPara,DetectCircleBoard& dBoard, CirclePoint pt1, CirclePoint pt2, double delta)
{
    double A = pt2.imgCoor.y - pt1.imgCoor.y;
    double B = pt1.imgCoor.x - pt2.imgCoor.x;
    double C = pt2.imgCoor.x * pt1.imgCoor.y - pt1.imgCoor.x * pt2.imgCoor.y;
    double AABB = sqrt(A * A + B * B);

    float x1 = pt1.imgCoor.x, x2 = pt2.imgCoor.x;
    float col1 = pt1.idx.x, col2 = pt2.idx.x;
    int row = pt1.idx.y;
    int nHalfBoardW = (bPara.nWidth - 1) / 2;

    for (int i = 0; i < dBoard.pts.size(); i++)
    {
        if (dBoard.pts[i].idx.x != unused)
            continue;

        double dist2Line = fabs(A * dBoard.pts[i].imgCoor.x + B * dBoard.pts[i].imgCoor.y + C) / AABB;
        if (dist2Line - delta < 0)
        {               
            float x = dBoard.pts[i].imgCoor.x;
            int tmpCol = cvRound(((x - x2) * col1 - (x - x1) * col2) / (x1 - x2));
            if (abs(tmpCol) - nHalfBoardW > 0)
                tmpCol = (tmpCol > 0) ? nHalfBoardW : -nHalfBoardW;
                
            dBoard.pts[i].idx = Point(tmpCol, row);
        }
    }

    return;
}

void CheckAllHoriLine(CircleBoardPara bPara, DetectCircleBoard& dBoard,vector<int>& delActRow)
{
    double distOA = calcLength(dBoard.origin.imgCoor.x, dBoard.origin.imgCoor.y,
        dBoard.A.imgCoor.x, dBoard.A.imgCoor.y);
    double deltaRow = distOA / 4;

    int ptCnt = dBoard.pts.size();
    int nHalfBoardH = (bPara.nHeight-1)/2;
    int lastPos = 0;
    delActRow.clear();
    for (int i = -nHalfBoardH; i < nHalfBoardH; i++)
    {
        vector<pair<float,int>> oneRow;
        float aveRow = 0.0;
        if (i == 0)
            lastPos = 0;

        for (int j = lastPos; j < ptCnt; j++)
       {
            if (dBoard.pts[j].idx.x == unused)
                continue;
            if (dBoard.pts[j].idx.y == i)
            {
                aveRow += dBoard.pts[j].imgCoor.y;
                oneRow.emplace_back(make_pair(dBoard.pts[j].imgCoor.y,j));
            }
            else if (dBoard.pts[j].idx.y > i)
            {
                lastPos = j;
                break;
            }
        }

        aveRow = aveRow / oneRow.size();
        for (int k = 0; k < oneRow.size(); k++)
        {
            if (fabs(oneRow[k].first - aveRow) > deltaRow || oneRow.size() <= 2)
            {
                dBoard.pts[oneRow[k].second].idx = Point(unused, unused);
                delActRow.emplace_back(i);
            }
        }
    }
}

void FillAllHoriLine(CircleBoardPara bPara, DetectCircleBoard& dBoard, vector<int> delActRow)
{
    double distOA = calcLength(dBoard.origin.imgCoor.x, dBoard.origin.imgCoor.y,
        dBoard.A.imgCoor.x, dBoard.A.imgCoor.y);
    double deltaRow = distOA / 4;

    int ptCnt = dBoard.pts.size();
    int nHalfBoardH = (bPara.nHeight - 1) / 2;
    int lastPos = 0;
    for (int i = 0; i < delActRow.size(); i++)
    {
        int curRow = delActRow[i];
        vector<pair<float, int>> oneRow;
        if (i == 0)
            lastPos = 0;

        for (int j = lastPos; j < ptCnt; j++)
        {
            if (dBoard.pts[j].idx.x == unused)
                continue;
            if (dBoard.pts[j].idx.y == curRow)
            {
                oneRow.emplace_back(make_pair(dBoard.pts[j].imgCoor.y, j));
            }
            else if (dBoard.pts[j].idx.y > curRow)
            {
                lastPos = j;
                break;
            }
        }

        if (oneRow.size() == 0)
            continue;

        int startIdx = oneRow[0].second;
        int endIdx = oneRow[oneRow.size() - 1].second;
        FindPtWithLinePoint(bPara,dBoard, dBoard.pts[startIdx],
            dBoard.pts[endIdx], deltaRow);        
    }
}

void CalcHomographyUseFiveCircle(CircleBoardPara bPara,DetectCircleBoard& dBoard,Mat& homo)
{
    int xAdd = (bPara.nWidth + 1) / 2;
    int yAdd = (bPara.nHeight + 1) / 2;
    vector<Point2f> srcPts(6), dstPts(6);
    srcPts[0] = dBoard.A.imgCoor;     
    srcPts[1] = dBoard.B.imgCoor;
    srcPts[2] = dBoard.C.imgCoor;
    srcPts[3] = dBoard.D.imgCoor;
    srcPts[4] = dBoard.E.imgCoor;
    srcPts[5] = dBoard.origin.imgCoor;

    dstPts[0] = Point2f((dBoard.A.idx.x + xAdd) * bPara.nInterSize, (dBoard.A.idx.y + yAdd) * bPara.nInterSize);
    dstPts[1] = Point2f((dBoard.B.idx.x + xAdd) * bPara.nInterSize, (dBoard.B.idx.y + yAdd) * bPara.nInterSize);
    dstPts[2] = Point2f((dBoard.C.idx.x + xAdd) * bPara.nInterSize, (dBoard.C.idx.y + yAdd) * bPara.nInterSize);
    dstPts[3] = Point2f((dBoard.D.idx.x + xAdd) * bPara.nInterSize, (dBoard.D.idx.y + yAdd) * bPara.nInterSize);
    dstPts[4] = Point2f((dBoard.E.idx.x + xAdd) * bPara.nInterSize, (dBoard.E.idx.y + yAdd) * bPara.nInterSize);
    dstPts[5] = Point2f((dBoard.origin.idx.x + xAdd) * bPara.nInterSize, (dBoard.origin.idx.y + yAdd) * bPara.nInterSize);

    homo = findHomography(srcPts, dstPts);
}

//确定标定点的行列号信息
bool GetCircleBoardWithFiveBigCircle(CircleBoardPara bPara,DetectCircleBoard& dBoard)
{
    FindFiveBigCircleAndOrigin(bPara, dBoard);
    Mat homo;
    CalcHomographyUseFiveCircle(bPara, dBoard, homo);
    //FindAllLines(bPara, dBoard);
    FindAllLinesUseHomo(bPara, dBoard,homo);
  /*  vector<int> delActRow;
    CheckAllHoriLine(bPara,dBoard,delActRow);
    if (delActRow.size() > 0)
        FillAllHoriLine(bPara, dBoard, delActRow);*/

    dBoard.CalcAllId(bPara);
    sort(dBoard.pts.begin(), dBoard.pts.end(),sortCirclePointByID);
    return true;
}
