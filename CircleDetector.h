#pragma once

#include "CaliStruct.h"
#include "SubPixEdge.h"


int procImgCnt=unused;

//亚像素提取边缘
void DetectSubPixEdge(string pdfPath,bool bSavePdf,const Mat& srcImg, vector<vector<Point2f>>& subpixedges)
{
    //parameters setting
    double* x;          /* x[n] y[n] coordinates of result contour point n */
    double* y;
    int* curve_limits;  /* limits of the curves in the x[] and y[] */
    int N, M;         /* result: N contour points, forming M curves */
    double S = 1.5; /* default sigma=0 */
    double H = 15; /* default th_h=0  */
    double L = 5; /* default th_l=0  */
    double W = 1.0; /* default W=1.3   */

    //char* pdf_out = const_cast<char*>((to_string(procImgCnt) + ".pdf").c_str());  /*pdf filename*/
    //char* txt_out = const_cast<char*>((to_string(procImgCnt) + ".txt").c_str());

    const int iHeight = srcImg.rows;
    const int iWidth = srcImg.cols;

    Mat grayImage, dstImage;
    if (srcImg.channels() > 1)
        cvtColor(srcImg, grayImage, COLOR_BGR2GRAY);
    else
        grayImage = srcImg.clone();

    dstImage = grayImage;

    uchar* pSrc = grayImage.data;//new uchar[iHeight*iWidth];
    uchar* pDst = dstImage.data;

    devernay(&x, &y, &N, &curve_limits, &M, pSrc, pDst, iWidth, iHeight, S, H, L);
    if (bSavePdf)
    {
        char pdf[1024];
        strcpy(pdf, pdfPath.c_str());
        write_curves_pdf(x,y, curve_limits,M, pdf,iWidth,iHeight,W);
    }
    
    for (int k = 0; k < M; k++) /* write curves */
    {
        vector<Point2f> oneEdge;
        for (int i = curve_limits[k]; i < curve_limits[k + 1]; i++)
        {
            oneEdge.push_back(Point2f(x[i], y[i]));
        }
        subpixedges.push_back(oneEdge);
    }


}

//填充标定板信息
void FillCircleBoard(const vector<KeyPoint> kpts, vector<vector<Point2f>> subpixEdges,double aveSize, DetectCircleBoard& board)
{
    //获取关键点数量，亚像素边缘数量
    int ptCnt = kpts.size(); 
    int edgeCnt = subpixEdges.size();

    //标定板点信息清空
    board.pts.clear();
    for (int i=0;i<ptCnt;i++)
    {
        //最小距离，最小距离对应的序号
        double minDist = DBL_MAX; int minIdx = -1;
        for (int j = 0; j < edgeCnt; j++)
        {
            //边缘点被清空则不处理
            if (subpixEdges[j].size() == 0)
                continue;

            float edgeLen = subpixEdges[j].size();
            float ptlen = PI * kpts[i].size;
            if (edgeLen / ptlen - 1.5 > 0 || edgeLen / ptlen - 0.5 < 0)//点到轮廓距离
                continue;
            double dist =pointPolygonTest(subpixEdges[j],kpts[i].pt,true);
            if (dist>0 && dist - minDist < 0)
            {
                minDist = dist;
                minIdx = j;
            }
        }
        //序号小于0，表示没有找到
        if (minIdx < 0)
            continue;

        RotatedRect rrc = fitEllipse(subpixEdges[minIdx]);
        float size = (rrc.size.width + rrc.size.height) * 0.5 ;
        double len = arcLength(subpixEdges[minIdx],true);
        double ratio = len / kpts[i].size;
        if (PI- ratio >0.2 || size / kpts[i].size - 1.5 > 0 || size / kpts[i].size - 0.5 < 0)
        {
            //清空对应的边缘点信息，避免重复
            subpixEdges[minIdx].clear();
            i = i- 1;
            continue;
        }
        //填充标定点信息
        CirclePoint cpt;
        cpt.cntr=subpixEdges[minIdx];
        cpt.imgCoor = kpts[i].pt;

        cpt.fitImgCoor = rrc.center;
        cpt.eccentricImgCoor = rrc.center;
        cpt.rrcEllipse = rrc;
        cpt.size = kpts[i].size;
        //加入标定板中
        board.pts.push_back(cpt);

        subpixEdges[minIdx].clear();
    }
}

//标定板是否合格，即大圆是否满足个数要求
bool IsBoardQualified(const vector<KeyPoint>& kpts,int nBigCircleCnt)
{
    bool bQualified = true;
    for (int i = 1; i < nBigCircleCnt; i++)
    {
        double ratio = kpts[0].size / kpts[i].size;
        if (ratio > 1.5)
        {
            bQualified = false;
            break;
        }
    }
    return bQualified;
}

//利用一幅影像的关键点Size统计值，过滤出过小或者过大的点
double FilterKeyPtBySize(vector<KeyPoint> srcPts, vector<KeyPoint>& result)
{
    double aveSize = 0; int ptCnt = srcPts.size();
    for (int i = 0; i < ptCnt; i++)
    {
        aveSize += srcPts[i].size;
    }
    aveSize = aveSize / (double)ptCnt;

    double upThr = aveSize * 2.5;
    double downThr = aveSize * 0.5;
    for (int i = 0; i < ptCnt; i++)
    {
        if ((srcPts[i].size - upThr) * (srcPts[i].size - downThr) < 0)
            result.push_back(srcPts[i]);
    }

    return aveSize;
}

//利用平均灰度过滤关键点
void FilterKeyPtByGray(Mat srcImg,vector<KeyPoint> srcPts, vector<KeyPoint>& result)
{
    int ptCnt = srcPts.size();
    for (int i = 0; i < ptCnt; i++)
    {
        int size = srcPts[i].size * 0.7;
        int left = srcPts[i].pt.x - size *0.5;
        int up = srcPts[i].pt.y - size * 0.5;
        Rect rc = Rect(left, up, size, size);
        double aveGray = sum(srcImg(rc))[0] / (size * size);
        if (aveGray > 100)
            continue;
        result.push_back(srcPts[i]);
    }

    return ;
}

//亚像素标定板提取
bool DetectSubPixCircleSingleImg(const Mat& srcImg,string pdfPath,bool bSavePdf,CircleBoardPara bPara,const Ptr<SimpleBlobDetector>& detector,DetectCircleBoard& oneBoard)
{  
    assert(srcImg.rows > 0 && srcImg.cols > 0);

    //过滤之后的检测结果
    int imgRows = srcImg.rows;
    int imgCols = srcImg.cols;

    if (procImgCnt == unused)
        procImgCnt = 1;
    else
        procImgCnt++;

    //斑点检测获取keypoint
    vector<KeyPoint> keyPoints,coarsePts1,coarsePts;
    detector.get()->detect(srcImg, keyPoints);
    sort(keyPoints.begin(), keyPoints.end(), sortKeyPointBySize);

    //根据斑点大小过滤掉过大或过小的点
    double aveSize = FilterKeyPtBySize(keyPoints, coarsePts1);
    FilterKeyPtByGray(srcImg,coarsePts1, coarsePts);
    //是否满足大圆个数要求，不足则不需要继续检测
    bool bQualified = IsBoardQualified(coarsePts, bPara.nBigCircle);

    if (bQualified)
    {
        //亚像素边缘
        vector<vector<Point2f>> subPixEdges;
        DetectSubPixEdge(pdfPath,bSavePdf,srcImg, subPixEdges);
        //填充标定板信息
        FillCircleBoard(coarsePts, subPixEdges, aveSize, oneBoard);
    }
    else
    {
        oneBoard.ClearData();
        //输出大圆未全部检测出来的信息
        cout <<"No\t"<<to_string(procImgCnt) << "\timage not all big circle is detected"<<endl;
    }
    return bQualified;
}
