// SingleImgTest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "CaliStruct.h"
#include "CircleDetector.h"
#include "CircleBoardDetector.h"
#include "CommonCalculation.h"

void singleProc(CaliProject proj)
{
    string imgName = "D550MM_37";

    string imgPath = proj.imgFolder + imgName +"."+ bmpType;
    string dstPath = proj.tmpFolder + imgName + "." + jpgType;
    string txtPath = proj.txtFolder + imgName + "." + caliBoardPtType;
    string coverPath = proj.coverFolder + imgName + "_cover." + jpgType;
    string pdfPath = proj.pdfFolder + imgName + "_sub." + subPixType;
    string unprojPath = proj.iterprojFolder + imgName + "_proj." + jpgType;

    DetectCircleBoard dBoard;
    dBoard.SetID(imgName);

    CalibImageData caImgData;
    SingleCalibCamera camera;
    camera.InitCalibration(CALIB_USE_INTRINSIC_GUESS,
        TermCriteria(
            TermCriteria::COUNT + TermCriteria::EPS,
            20000, DBL_EPSILON));

    unProjectTransData unprojData;
    Mat unprojImg,undistortImg, coverImg,reprojImg;
    Mat reprojTrans;
    CameraInitial initial = proj.caliInitials[0];;

    double rms[5] = {0,0,0,0,0};

    Mat srcImg = imread(imgPath, 0);
    if (imageSize.width == unused)
    {
        imageSize = Size(srcImg.cols, srcImg.rows);
        for (int k = 0; k < proj.caliInitials.size(); k++)
        {
            if (proj.caliInitials[k].camInfo.second.width - imageSize.width == 0)
            {
                if (proj.caliInitials[k].camInfo.second.height - imageSize.height == 0)
                {
                    initial = proj.caliInitials[k];
                }
            }
        }
    }

    std::cout << "ProcImage: " << imgName << endl;

    bool bBoardQualified = DetectSubPixCircleSingleImg(srcImg, pdfPath, true, proj.bPara, proj.detector,dBoard);
    if (bBoardQualified)
    {
        GetCircleBoardWithFiveBigCircle(proj.bPara, dBoard);
        caImgData.TransSingleBoard2CaliImage(proj.bPara, dBoard, CaliPtType::FitPt);
        double rms= camera.DoCalibration(proj.caliInitials[0],caImgData, imageSize);
        std::cout << "rms0 = " << rms << "\t";   

        dstPath = proj.tmpFolder + imgName + "_." + jpgType;
        txtPath = proj.txtFolder + imgName + "_." + caliBoardPtType;

        WriteSingleDetectCircleBoardImg(dstPath, srcImg, proj.bPara, dBoard);
        WriteSingleCircleBoardTxt(txtPath, proj.bPara, dBoard);

        for (int iter = 0; iter < proj.maxIterCnt; iter++)
        {
            unprojPath = proj.iterprojFolder + imgName + "_proj_" +to_string(iter)+"."+ jpgType;
            pdfPath = proj.pdfFolder + imgName + "_sub_" + to_string(iter) + "." + subPixType;
            dstPath = proj.tmpFolder + imgName + "_" + to_string(iter) + "." + jpgType;
            txtPath = proj.txtFolder + imgName + "_" + to_string(iter) + "." + caliBoardPtType;

            camera.InitUndistor(imageSize);
            camera.DoSingleUndistort(srcImg, undistortImg);

            //if(fabs(proj.bPara.fRes-unused) <1e-6)
             //   proj.bPara.fRes = (float)cvFloor(((float)proj.bPara.nBigD / dBoard.A.size)*100)/100.0;
           
            unprojData.TransBoard2UnprojData(proj.bPara, imageSize, dBoard);
            unprojData.DoSingleImgUnproj(undistortImg, unprojImg);
            cv::imwrite(unprojPath, unprojImg);
            dBoard.ClearData();

            bBoardQualified = DetectSubPixCircleSingleImg(unprojImg, pdfPath, true, proj.bPara, proj.detector, dBoard);
            if (!bBoardQualified)
                break;

            unprojData.GetReprojTrans(reprojTrans);
            dBoard.TransBoardPt(reprojTrans);
            GetCircleBoardWithFiveBigCircle(proj.bPara, dBoard);

            caImgData.TransSingleBoard2CaliImage(proj.bPara, dBoard, CaliPtType::FitPt);
            double rms2 = camera.DoCalibration(proj.caliInitials[0],caImgData, imageSize);
           std::cout << "iter " << iter << " rms = " << rms2 <<"\t";

            //double ratio = CalSingleImageCoverage(dBoard, srcImg.size(), coverageSize, coverImg);
            //imwrite(coverPath, coverImg);
            //std::cout << "coverage:\t" << ratio << endl;
            WriteSingleDetectCircleBoardImg(dstPath, srcImg, proj.bPara, dBoard);
            WriteSingleCircleBoardTxt(txtPath, proj.bPara, dBoard);
            srcImg = undistortImg.clone();
        }
    }
}

void batProcSingleCamOneImageIter(CaliProject proj)
{
    string imgName = "";

    vector<string> imgList;
    get_filepaths_from_dir(proj.imgFolder, imgList, bmpType);
    sortFiles(imgList);

    string imgPath = proj.imgFolder + imgName +"."+ bmpType;
    string dstPath = proj.tmpFolder + imgName + "." + jpgType;
    string txtPath = proj.txtFolder + imgName + "." + caliBoardPtType;
    string coverPath = proj.coverFolder + imgName + "_cover." + jpgType;
    string pdfPath = proj.pdfFolder + imgName + "_sub." + subPixType;
    string unprojPath = proj.iterprojFolder + imgName + "_proj." + jpgType;

    DetectCircleBoard dBoard;
    vector<vector<DetectCircleBoard>> allBoards(proj.camCnt);
    CalibImageData caImgData;
    CalibImageData allCaliImgData;

    SingleCalibCamera camera,allCamera;
    camera.InitCalibration(
        CALIB_USE_INTRINSIC_GUESS| CALIB_FIX_FOCAL_LENGTH, 
        TermCriteria(
        TermCriteria::COUNT + TermCriteria::EPS,
        100, 0.001));

    allCamera.InitCalibration(
        CALIB_USE_INTRINSIC_GUESS ,
        TermCriteria(
            TermCriteria::COUNT + TermCriteria::EPS,
            3000, 1e-6));

    unProjectTransData unprojData;
    Mat unprojImg, undistortImg, coverImg, reprojImg;
    Mat reprojTrans;

    CameraInitial initial = proj.caliInitials[0];;

    vector<pair<double, DetectCircleBoard>> iterBoards(proj.maxIterCnt+1);
    for (int i = 0; i < imgList.size(); i++)
    {
        imgPath = imgList[i];
        GetImgName(imgPath, imgName);
        std::cout << "ProcImage: " << imgName << endl;

        if(i==0)
            allBoards[i].resize(imgList.size());

        DetectCircleBoard dBoard;
        dBoard.SetID(imgName);

        InnPara para;
        Mat srcImg = imread(imgPath, 0);
        if (i==0 || imageSize.width == unused)
        {
            imageSize = Size(srcImg.cols, srcImg.rows);
            for (int k = 0; k < proj.caliInitials.size(); k++)
            {
                if (proj.caliInitials[k].camInfo.second.width - imageSize.width == 0)
                {
                    if (proj.caliInitials[k].camInfo.second.height - imageSize.height == 0)
                    {
                        initial = proj.caliInitials[k];
                    }
                }
            }
        }

        bool bBoardQualified = DetectSubPixCircleSingleImg(srcImg, pdfPath, true, proj.bPara, proj.detector, dBoard);
        if (bBoardQualified)
        {
            GetCircleBoardWithFiveBigCircle(proj.bPara, dBoard);
             proj.procImgCnt++;
             caImgData.TransSingleBoard2CaliImage(proj.bPara, dBoard, CaliPtType::FitPt);
            
            double rms = camera.DoCalibration(initial,caImgData, imageSize);
            std::cout << "iter " << 0 << " rms = " << rms << "\t";
            camera.GetCameraPara(para);
            std::cout << "\t" << para.fx << "\t" << para.fy << "\t" << para.cx << "\t" << para.cy << "\t";

            iterBoards[0] = (make_pair(rms, dBoard));
            for (int iter = 0; iter < proj.maxIterCnt; iter++)
            {
                unprojPath = proj.iterprojFolder + imgName + "_proj_" + to_string(iter) + "." + jpgType;
                pdfPath = proj.pdfFolder + imgName + "_sub_" + to_string(iter) + "." + subPixType;
                dstPath = proj.tmpFolder + imgName + "_" + to_string(iter) + "." + jpgType;
                txtPath = proj.txtFolder + imgName + "_" + to_string(iter) + "." + caliBoardPtType;

                camera.InitUndistor(imageSize);
                camera.DoSingleUndistort(srcImg, undistortImg);

                //proj.bPara.fRes = (float)cvFloor(((float)proj.bPara.nBigD / dBoard.A.size) * 100) / 100.0;
                unprojData.TransBoard2UnprojData(proj.bPara, imageSize,dBoard);
                unprojData.DoSingleImgUnproj(undistortImg, unprojImg);
                cv::imwrite(unprojPath, unprojImg);
                dBoard.ClearData();

                bBoardQualified = DetectSubPixCircleSingleImg(unprojImg, pdfPath, true, proj.bPara, proj.detector, dBoard);
                unprojData.GetReprojTrans(reprojTrans);
                dBoard.TransBoardPt(reprojTrans);
                GetCircleBoardWithFiveBigCircle(proj.bPara, dBoard);

                caImgData.TransSingleBoard2CaliImage(proj.bPara, dBoard, CaliPtType::FitPt);
                double rms2 = camera.DoCalibration(initial, caImgData, imageSize);
                std::cout << "iter " << iter+1 << " rms = " << rms2 << "\t";
                camera.GetCameraPara(para);
                std::cout << "\t" << para.fx << "\t" << para.fy << "\t" << para.cx << "\t" << para.cy << "\t";

                //double ratio = CalSingleImageCoverage(dBoard, imageSize, proj.coverageSize, coverImg);
                //std::cout << "coverage:\t" << ratio << "\t";

                WriteSingleDetectCircleBoardImg(dstPath, srcImg, proj.bPara, dBoard);
                WriteSingleCircleBoardTxt(txtPath, proj.bPara, dBoard);
                srcImg = undistortImg.clone();
                iterBoards[iter+1] = (make_pair(rms2, dBoard));
            }   

            double minRMS = rms; int minPos = 0;
            for (int k = 0; k < iterBoards.size(); k++)
            {
                if (iterBoards[k].first - minRMS < 0)
                {
                    minRMS = iterBoards[k].first;
                    minPos = k;
                }
            }
            std::cout << endl;
            std::cout << "***** min rms = " << minRMS <<" minPos = "<<minPos<< " ******" << endl;
            allBoards[0][i] = iterBoards[minPos].second;
        }

        if (i>0 &&(i-(imgList.size()-1))!=0&& i % 10==0)
        {
            allCaliImgData.TransAllBoard2CaliImage(proj.bPara, proj.camCnt - 1, allBoards, CaliPtType::FitPt);
            double rms2 = allCamera.DoCalibration(initial, allCaliImgData, imageSize);
            std::cout << "***** ***** all rms = " << rms2 <<" ******"<< endl;
            camera.GetCameraPara(para);
            std::cout << "\t" << para.fx << "\t" << para.fy << "\t" << para.cx << "\t" << para.cy << "\t";
            std::cout << "\t" << para.k1 << "\t" << para.k2 << "\t" << para.k3 << "\t" << para.p1 << "\t" << para.p2 << "\t";
            std::cout << endl;

        }
    }
    allCaliImgData.TransAllBoard2CaliImage(proj.bPara, proj.camCnt - 1, allBoards, CaliPtType::EccentriPt);
    double rms2 = allCamera.DoCalibration(initial, allCaliImgData, imageSize);
    std::cout << "***** all KeyPt rms = " << rms2 << " ******" << endl;
    std::cout << camera.cameraMatrix << endl;
    std::cout << camera.distCoeffs << endl;

    allCaliImgData.TransAllBoard2CaliImage(proj.bPara,proj.camCnt-1,allBoards, CaliPtType::FitPt);
    rms2 = allCamera.DoCalibration(initial, allCaliImgData, imageSize);
    std::cout << "***** all FitPt rms = " << rms2 << " ******" << endl;
    std::cout << camera.cameraMatrix << endl;
    std::cout << camera.distCoeffs << endl;

    double ratio = CalCoverageUseAllImage(allBoards[0], imageSize, proj.coverageSize, coverImg);
    cv::imwrite(coverPath, coverImg);
    std::cout << "All Coverage:\t" << ratio << endl;
}

void batProcSingleCamAllImageIter(CaliProject proj)
{
    string imgName = "all";

    vector<string> imgList;
    get_filepaths_from_dir(proj.imgFolder, imgList, bmpType);
    sortFiles(imgList);

    string imgPath = proj.imgFolder + imgName + "." + bmpType;
    string dstPath = proj.tmpFolder + imgName + "." + jpgType;
    string txtPath = proj.txtFolder + imgName + "." + caliBoardPtType;
    string coverPath = proj.coverFolder + imgName + "_cover." + jpgType;
    string pdfPath = proj.pdfFolder + imgName + "_sub." + subPixType;
    string unprojPath = proj.iterprojFolder + imgName + "_proj." + jpgType;

    CalibImageData caImgData;
    CalibImageData allCaliImgData;

    SingleCalibCamera allCamera;
    allCamera.InitCalibration(
        CALIB_USE_INTRINSIC_GUESS,
        TermCriteria(
            TermCriteria::COUNT + TermCriteria::EPS,
            20000, DBL_EPSILON));

    unProjectTransData unprojData;
    Mat unprojImg, undistortImg, coverImg, reprojImg;
    Mat reprojTrans;

    CameraInitial initial = proj.caliInitials[0];;

    vector<vector<DetectCircleBoard>> allBoards(proj.camCnt);
    for(int i=0;i<proj.camCnt;i++)
        allBoards[i].resize(imgList.size());

    vector<pair<double, vector<DetectCircleBoard>>> iterBoards(proj.maxIterCnt + 1);
    InnPara para;
    vector<DetectCircleBoard> vBoards(imgList.size());
    vector<Mat> vSrcImg(imgList.size());
    std::cout << "Iter 0"<<endl;
    procImgCnt = 0;
    for (int i = 0; i < imgList.size(); i++)
    {        
        imgPath = imgList[i];
        GetImgName(imgPath, imgName);
        std::cout << "ProcImage: " << imgName << "\t";

        dstPath = proj.tmpFolder + imgName + "_" + to_string(0) + "." + jpgType;
        txtPath = proj.txtFolder + imgName + "_" + to_string(0) + "." + caliBoardPtType;

        vBoards[i].SetID(imgName);
        vSrcImg[i] = imread(imgPath, 0);
        if (i==0 || imageSize.width == unused)
        {
            imageSize = Size(vSrcImg[i].cols, vSrcImg[i].rows);
            for (int k = 0; k < proj.caliInitials.size(); k++)
            {
                if (proj.caliInitials[k].camInfo.second.width - imageSize.width == 0 &&
                    proj.caliInitials[k].camInfo.second.height - imageSize.height == 0)
                {
                    initial = proj.caliInitials[k];
                }
            }
        }

        bool bBoardQualified = DetectSubPixCircleSingleImg(vSrcImg[i], pdfPath, true, proj.bPara, proj.detector, vBoards[i]);
        if (bBoardQualified)
        {
            GetCircleBoardWithFiveBigCircle(proj.bPara, vBoards[i]);
            proj.procImgCnt++;

            WriteSingleDetectCircleBoardImg(dstPath, vSrcImg[i], proj.bPara, vBoards[i]);
            WriteSingleCircleBoardTxt(txtPath, proj.bPara, vBoards[i]);
        }
    }

    allBoards[0] = vBoards;
    //allCaliImgData.TransAllBoard2CaliImage(proj.bPara, proj.camCnt - 1, allBoards, CaliPtType::EccentriPt);
    //double rms1 = allCamera.DoCalibration(initial, allCaliImgData, imageSize);
    //std::cout << "***** all FitPt rms = " << rms1 << " ******" << endl;
    //std::cout << allCamera.cameraMatrix << endl;
    //std::cout << allCamera.distCoeffs << endl;

    allCaliImgData.TransAllBoard2CaliImage(proj.bPara, proj.camCnt - 1, allBoards, CaliPtType::FitPt);
    double rms2 = allCamera.DoCalibration(initial, allCaliImgData, imageSize);
    std::cout << "***** all IterPt rms = " << rms2 << " ******" << endl;
    allCamera.GetCameraPara(para);
    std::cout << "\t" << para.fx << "\t" << para.fy << "\t" << para.cx << "\t" << para.cy << "\t";
    std::cout << "\t" << para.k1 << "\t" << para.k2 << "\t" << para.k3 << "\t" << para.p1 << "\t" << para.p2 << "\t";
    std::cout << endl;

    double ratio = CalCoverageUseAllImage(allBoards[0], imageSize, proj.coverageSize, coverImg);
    cv::imwrite(coverPath, coverImg);
    std::cout << "All Coverage:\t" << ratio << endl;
    iterBoards[0] = make_pair(rms2,vBoards);

    for (int iter = 1; iter <= proj.maxIterCnt; iter++)
    {
        std::cout << "Iter: "<<iter<<endl;
        
        allCamera.InitUndistor(imageSize);
        proj.procImgCnt = 0;
        procImgCnt = 0;
        for (int i = 0; i < imgList.size(); i++)
        {
            if (vBoards[i].pts.size() == 0)
                continue;
            
            imgPath = imgList[i];
            GetImgName(imgPath, imgName);
            std::cout << "ProcImage: " << imgName << "\t";
            
            unprojPath = proj.iterprojFolder + imgName + "_proj_" + to_string(iter) + "." + jpgType;
            pdfPath = proj.pdfFolder + imgName + "_sub_" + to_string(iter) + "." + subPixType;
            dstPath = proj.tmpFolder + imgName + "_" + to_string(iter) + "." + jpgType;
            txtPath = proj.txtFolder + imgName + "_" + to_string(iter) + "." + caliBoardPtType;
                        
            allCamera.DoSingleUndistort(vSrcImg[i], undistortImg);

            unprojData.TransBoard2UnprojData(proj.bPara, imageSize, vBoards[i]);
            unprojData.DoSingleImgUnproj(undistortImg, unprojImg);
            cv::imwrite(unprojPath, unprojImg);
            vBoards[i].ClearData();

            bool bBoardQualified = DetectSubPixCircleSingleImg(unprojImg, pdfPath, true, proj.bPara, proj.detector, vBoards[i]);
            if (!bBoardQualified)
                continue;

            proj.procImgCnt++;
            unprojData.GetReprojTrans(reprojTrans);
            vBoards[i].TransBoardPt(reprojTrans);
            GetCircleBoardWithFiveBigCircle(proj.bPara, vBoards[i]);
            WriteSingleDetectCircleBoardImg(dstPath, vSrcImg[i], proj.bPara, vBoards[i]);
            WriteSingleCircleBoardTxt(txtPath, proj.bPara, vBoards[i]);

            vSrcImg[i] = undistortImg.clone();
       }

        allBoards[0] = vBoards;
        //allCaliImgData.TransAllBoard2CaliImage(proj.bPara, proj.camCnt - 1, allBoards, CaliPtType::EccentriPt);
        //double rms1 = allCamera.DoCalibration(initial, allCaliImgData, imageSize);
        //std::cout << "***** all FitPt rms = " << rms1 << " ******" << endl;
        //std::cout << allCamera.cameraMatrix << endl;
        //std::cout << allCamera.distCoeffs << endl;

        allCaliImgData.TransAllBoard2CaliImage(proj.bPara, proj.camCnt - 1, allBoards, CaliPtType::FitPt);
        double rms2 = allCamera.DoCalibration(initial, allCaliImgData, imageSize);
        std::cout << "***** all IterPt rms = " << rms2 << " ******" << endl;
        allCamera.GetCameraPara(para);
        std::cout << "\t" << para.fx << "\t" << para.fy << "\t" << para.cx << "\t" << para.cy << "\t";
        std::cout << "\t" << para.k1 << "\t" << para.k2 << "\t" << para.k3 << "\t" << para.p1 << "\t" << para.p2 << "\t";
        std::cout << endl;
        iterBoards[iter] = make_pair(rms2, vBoards);
    }
}

void batProcAllCamAllImageIter(CaliProject proj)
{
    string imgName = "all";

    //CalibImageData caImgData;

    unProjectTransData unprojData;
    Mat unprojImg, undistortImg, coverImg, reprojImg;
    Mat reprojTrans;

    vector<vector<DetectCircleBoard>> allBoards(proj.camCnt);
    vector<SingleCalibCamera> allSingleCameras(proj.camCnt);
    vector<CalibImageData> vCaliImgData(proj.camCnt);
    vector<StereoCalibImageData> vStereoData(proj.camCnt-1);
    vector<Size> allImageSize(proj.camCnt);

    MultiCalibCameras multiCam;
    multiCam.InitStereoCalibration(CALIB_FIX_INTRINSIC,
        TermCriteria(
            TermCriteria::COUNT + TermCriteria::EPS,
            200000, DBL_EPSILON));

    for (int cam = 0; cam < proj.camCnt; cam++)
    {
        std::cout << "Cam " <<cam<< " calibration start:"<<endl;

        vector<string> imgList;
        get_filepaths_from_dir(proj.imgFolders[cam], imgList, bmpType);
        sortFiles(imgList);

        allSingleCameras[cam].InitCalibration(
            CALIB_USE_INTRINSIC_GUESS,
            TermCriteria(
                TermCriteria::COUNT + TermCriteria::EPS,
                200000, DBL_EPSILON));

        allBoards[cam].resize(imgList.size());

        string imgPath = proj.imgFolders[cam] + imgName + "." + bmpType;
        string dstPath = proj.tmpFolders[cam] + imgName + "." + jpgType;
        string txtPath = proj.txtFolders[cam] + imgName + "." + caliBoardPtType;
        string coverPath = proj.coverFolders[cam] + imgName + "_cover." + jpgType;
        string pdfPath = proj.pdfFolders[cam] + imgName + "_sub." + subPixType;
        string unprojPath = proj.iterprojFolders[cam] + imgName + "_proj." + jpgType;

        CameraInitial initial = proj.caliInitials[cam];
        CalibImageData allCaliImgData;

        vector<pair<double, vector<DetectCircleBoard>>> iterBoards(proj.maxIterCnt + 1);
        InnPara para;
        vector<DetectCircleBoard> vBoards(imgList.size());
        vector<Mat> vSrcImg(imgList.size());
        std::cout << "Iter 0" << endl;
        procImgCnt = 0;

        for (int i = 0; i < imgList.size(); i++)
        {
            imgPath = imgList[i];
            GetImgName(imgPath, imgName);
            std::cout << "ProcImage: " << imgName << "\t";

            dstPath = proj.tmpFolders[cam] + imgName + "_" + to_string(0) + "." + jpgType;
            txtPath = proj.txtFolders[cam] + imgName + "_" + to_string(0) + "." + caliBoardPtType;
            pdfPath = proj.pdfFolders[cam] + imgName + "_sub_" + to_string(0) + "." + subPixType;

            vBoards[i].SetID(imgName);
            vSrcImg[i] = imread(imgPath, 0);
            if (i==0 || imageSize.width == unused)
            {
                imageSize = Size(vSrcImg[i].cols, vSrcImg[i].rows);
                allImageSize[cam] = imageSize;
                for (int k = 0; k < proj.caliInitials.size(); k++)
                {
                    if (proj.caliInitials[k].camInfo.second.width - imageSize.width == 0 &&
                        proj.caliInitials[k].camInfo.second.height - imageSize.height == 0)
                    {
                        initial = proj.caliInitials[k];
                    }
                }
            }

            bool bBoardQualified = DetectSubPixCircleSingleImg(vSrcImg[i], pdfPath, true, proj.bPara, proj.detector, vBoards[i]);
            if (bBoardQualified)
            {
                GetCircleBoardWithFiveBigCircle(proj.bPara, vBoards[i]);
                proj.procImgCnt++;

                WriteSingleDetectCircleBoardImg(dstPath, vSrcImg[i], proj.bPara, vBoards[i]);
                WriteSingleCircleBoardTxt(txtPath, proj.bPara, vBoards[i]);
            }
        }

        allBoards[cam] = vBoards;

        allCaliImgData.TransAllBoard2CaliImage(proj.bPara, cam, allBoards, CaliPtType::FitPt);
        double rms2 = allSingleCameras[cam].DoCalibration(initial, allCaliImgData, imageSize);
        std::cout << "***** all IterPt rms = " << rms2 << " ******" << endl;
        allSingleCameras[cam].GetCameraPara(para);
        std::cout << "\t" << para.fx << "\t" << para.fy << "\t" << para.cx << "\t" << para.cy << "\t";
        std::cout << "\t" << para.k1 << "\t" << para.k2 << "\t" << para.k3 << "\t" << para.p1 << "\t" << para.p2 << "\t";
        std::cout << endl;

        double ratio = CalCoverageUseAllImage(allBoards[cam], imageSize, proj.coverageSize, coverImg);
        cv::imwrite(coverPath, coverImg);
        std::cout << "All Coverage:\t" << ratio << endl;

        if (cam > 0)
        {
            vStereoData[cam - 1].TransBoard2StereoCaliData(proj.bPara, allBoards[0], allBoards[cam], CaliPtType::FitPt);
            multiCam.InitCameraList(allSingleCameras);
            double multiRms = multiCam.DoStereoCalibration(vCaliImgData, vStereoData, allImageSize);
            std::cout << "***** multi rms = " << multiRms << " ******" << endl;
            Pos relPos;
            multiCam.GetOneRelCamPos(cam, relPos);
            std::cout << "\t" << relPos.T[0] << "\t" << relPos.T[1] << "\t" << relPos.T[2] << "\t" << relPos.Eulor[0] << "\t" << relPos.Eulor[1] << "\t" << relPos.Eulor[2] << endl;
        }
        //iterBoards[0] = make_pair(rms2, vBoards);
        for (int iter = 1; iter <= proj.maxIterCnt; iter++)
        {
            std::cout << "Iter: " << iter << endl;

            allSingleCameras[cam].InitUndistor(imageSize);
            proj.procImgCnt = 0;
            procImgCnt = 0;
            for (int i = 0; i < imgList.size(); i++)
            {
                if (vBoards[i].pts.size() == 0)
                    continue;

                imgPath = imgList[i];
                GetImgName(imgPath, imgName);
                std::cout << "ProcImage: " << imgName << "\t";

                unprojPath = proj.iterprojFolders[cam] + imgName + "_proj_" + to_string(iter) + "." + jpgType;
                pdfPath = proj.pdfFolders[cam] + imgName + "_sub_" + to_string(iter) + "." + subPixType;
                dstPath = proj.tmpFolders[cam] + imgName + "_" + to_string(iter) + "." + jpgType;
                txtPath = proj.txtFolders[cam] + imgName + "_" + to_string(iter) + "." + caliBoardPtType;

                allSingleCameras[cam].DoSingleUndistort(vSrcImg[i], undistortImg);

                unprojData.TransBoard2UnprojData(proj.bPara, imageSize, vBoards[i]);
                unprojData.DoSingleImgUnproj(undistortImg, unprojImg);
                cv::imwrite(unprojPath, unprojImg);
                vBoards[i].ClearData();

                bool bBoardQualified = DetectSubPixCircleSingleImg(unprojImg, pdfPath, true, proj.bPara, proj.detector, vBoards[i]);
                if (!bBoardQualified)
                    continue;

                proj.procImgCnt++;
                unprojData.GetReprojTrans(reprojTrans);
                vBoards[i].TransBoardPt(reprojTrans);
                GetCircleBoardWithFiveBigCircle(proj.bPara, vBoards[i]);
                WriteSingleDetectCircleBoardImg(dstPath, vSrcImg[i], proj.bPara, vBoards[i]);
                WriteSingleCircleBoardTxt(txtPath, proj.bPara, vBoards[i]);

                vSrcImg[i] = undistortImg.clone();
            }

            allBoards[cam] = vBoards;

            allCaliImgData.TransAllBoard2CaliImage(proj.bPara, cam, allBoards, CaliPtType::FitPt);
            double rms2 = allSingleCameras[cam].DoCalibration(initial, allCaliImgData, imageSize);
            std::cout << "***** all IterPt rms = " << rms2 << " ******" << endl;
            allSingleCameras[cam].GetCameraPara(para);
            std::cout << "\t" << para.fx << "\t" << para.fy << "\t" << para.cx << "\t" << para.cy << "\t";
            std::cout << "\t" << para.k1 << "\t" << para.k2 << "\t" << para.k3 << "\t" << para.p1 << "\t" << para.p2 << "\t";
            std::cout << endl;
          //  iterBoards[iter] = make_pair(rms2, vBoards);
            if (cam > 0)
            {
                vStereoData[cam - 1].TransBoard2StereoCaliData(proj.bPara, allBoards[0], allBoards[cam], CaliPtType::FitPt);
                multiCam.InitCameraList(allSingleCameras);
                double multiRms = multiCam.DoStereoCalibration(vCaliImgData, vStereoData, allImageSize);
                std::cout << "***** multi rms = " << multiRms << " ******" << endl;
                Pos relPos;
                multiCam.GetOneRelCamPos(proj.camCnt - 1, relPos);
                std::cout << "\t" << relPos.T[0] << "\t" << relPos.T[1] << "\t" << relPos.T[2] << "\t" << relPos.Eulor[0] << "\t" << relPos.Eulor[1] << "\t" << relPos.Eulor[2] << endl;
            }
        }

        vCaliImgData[cam] = allCaliImgData;
        if (cam >0)
        {
            vStereoData[cam - 1].TransBoard2StereoCaliData(proj.bPara, allBoards[0], allBoards[cam], CaliPtType::FitPt);
        }
    }

    multiCam.InitCameraList(allSingleCameras);
    double multiRms = multiCam.DoStereoCalibration(vCaliImgData, vStereoData, allImageSize);
    std::cout << "***** multi rms = " << multiRms << " ******" << endl;
    Pos relPos;
    multiCam.GetOneRelCamPos(proj.camCnt - 1, relPos);
    std::cout << "\t" << relPos.T[0]<< "\t" << relPos.T[1] << "\t" << relPos.T[2] << "\t" << relPos.Eulor[0] << "\t" << relPos.Eulor[1]<< "\t" << relPos.Eulor[2] << endl;

}

int main()
{
    CircleBoardPara smallPara;
    smallPara.nBoardH = 400; smallPara.nBoardW = 500;
    smallPara.nBigD = 8;    smallPara.nSmallD = 4;
    smallPara.nBigCircle = 5;
    smallPara.nHeight = 15;    smallPara.nWidth = 19;
    smallPara.nInterSize = 10;
    smallPara.fRes = 0.1;

    CircleBoardPara bigPara;
    bigPara.nBoardH = 400; bigPara.nBoardW = 500;
    bigPara.nBigD = 20;    bigPara.nSmallD = 10;
    bigPara.nBigCircle = 5;
    bigPara.nHeight = 15;    bigPara.nWidth = 19;
    bigPara.nInterSize = 25;
    bigPara.fRes = 0.20;

    CaliProject proj;
    proj.bPara = bigPara;

    proj.bCreateSub = true;
    proj.maxIterCnt = 0;
    
    proj.camCnt = 2;
    proj.imgFolders.resize(proj.camCnt);
    proj.imgFolder = "C:\\calibProjects\\20211026_CircleTest\\L_Cam\\";
    proj.imgFolders[0] = "C:\\calibProjects\\20211026_CircleTest\\L_Cam\\";
    proj.imgFolders[1] = "C:\\calibProjects\\20211026_CircleTest\\SR_Cam\\";
    proj.InitDetectSubDir(proj.camCnt);
    proj.InitBlobDetector();

    CameraInitial cam;
    cam.focal = 12;
    cam.camInfo = make_pair(0.0048, Size(1280, 1024));
    proj.caliInitials.push_back(cam);

    cam.focal = 12;
    cam.camInfo = make_pair(0.00345, Size(2448, 2048));
    proj.caliInitials.push_back(cam);

   // singleProc(proj);
    batProcAllCamAllImageIter(proj);
   // batProcSingleCamAllImageIter(proj);
    std::cout << "TheEnd!\n";
    cv::waitKey();
}
