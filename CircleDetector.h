#pragma once

#include "CaliStruct.h"
#include "SubPixEdge.h"


int procImgCnt=unused;

//��������ȡ��Ե
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

//���궨����Ϣ
void FillCircleBoard(const vector<KeyPoint> kpts, vector<vector<Point2f>> subpixEdges,double aveSize, DetectCircleBoard& board)
{
    //��ȡ�ؼ��������������ر�Ե����
    int ptCnt = kpts.size(); 
    int edgeCnt = subpixEdges.size();

    //�궨�����Ϣ���
    board.pts.clear();
    for (int i=0;i<ptCnt;i++)
    {
        //��С���룬��С�����Ӧ�����
        double minDist = DBL_MAX; int minIdx = -1;
        for (int j = 0; j < edgeCnt; j++)
        {
            //��Ե�㱻����򲻴���
            if (subpixEdges[j].size() == 0)
                continue;

            float edgeLen = subpixEdges[j].size();
            float ptlen = PI * kpts[i].size;
            if (edgeLen / ptlen - 1.5 > 0 || edgeLen / ptlen - 0.5 < 0)//�㵽��������
                continue;
            double dist =pointPolygonTest(subpixEdges[j],kpts[i].pt,true);
            if (dist>0 && dist - minDist < 0)
            {
                minDist = dist;
                minIdx = j;
            }
        }
        //���С��0����ʾû���ҵ�
        if (minIdx < 0)
            continue;

        RotatedRect rrc = fitEllipse(subpixEdges[minIdx]);
        float size = (rrc.size.width + rrc.size.height) * 0.5 ;
        double len = arcLength(subpixEdges[minIdx],true);
        double ratio = len / kpts[i].size;
        if (PI- ratio >0.2 || size / kpts[i].size - 1.5 > 0 || size / kpts[i].size - 0.5 < 0)
        {
            //��ն�Ӧ�ı�Ե����Ϣ�������ظ�
            subpixEdges[minIdx].clear();
            i = i- 1;
            continue;
        }
        //���궨����Ϣ
        CirclePoint cpt;
        cpt.cntr=subpixEdges[minIdx];
        cpt.imgCoor = kpts[i].pt;

        cpt.fitImgCoor = rrc.center;
        cpt.eccentricImgCoor = rrc.center;
        cpt.rrcEllipse = rrc;
        cpt.size = kpts[i].size;
        //����궨����
        board.pts.push_back(cpt);

        subpixEdges[minIdx].clear();
    }
}

//�궨���Ƿ�ϸ񣬼���Բ�Ƿ��������Ҫ��
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

//����һ��Ӱ��Ĺؼ���Sizeͳ��ֵ�����˳���С���߹���ĵ�
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

//����ƽ���Ҷȹ��˹ؼ���
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

//�����ر궨����ȡ
bool DetectSubPixCircleSingleImg(const Mat& srcImg,string pdfPath,bool bSavePdf,CircleBoardPara bPara,const Ptr<SimpleBlobDetector>& detector,DetectCircleBoard& oneBoard)
{  
    assert(srcImg.rows > 0 && srcImg.cols > 0);

    //����֮��ļ����
    int imgRows = srcImg.rows;
    int imgCols = srcImg.cols;

    if (procImgCnt == unused)
        procImgCnt = 1;
    else
        procImgCnt++;

    //�ߵ����ȡkeypoint
    vector<KeyPoint> keyPoints,coarsePts1,coarsePts;
    detector.get()->detect(srcImg, keyPoints);
    sort(keyPoints.begin(), keyPoints.end(), sortKeyPointBySize);

    //���ݰߵ��С���˵�������С�ĵ�
    double aveSize = FilterKeyPtBySize(keyPoints, coarsePts1);
    FilterKeyPtByGray(srcImg,coarsePts1, coarsePts);
    //�Ƿ������Բ����Ҫ�󣬲�������Ҫ�������
    bool bQualified = IsBoardQualified(coarsePts, bPara.nBigCircle);

    if (bQualified)
    {
        //�����ر�Ե
        vector<vector<Point2f>> subPixEdges;
        DetectSubPixEdge(pdfPath,bSavePdf,srcImg, subPixEdges);
        //���궨����Ϣ
        FillCircleBoard(coarsePts, subPixEdges, aveSize, oneBoard);
    }
    else
    {
        oneBoard.ClearData();
        //�����Բδȫ������������Ϣ
        cout <<"No\t"<<to_string(procImgCnt) << "\timage not all big circle is detected"<<endl;
    }
    return bQualified;
}
