//-----------------------------------------------------------------------------------
//
//  Copyright (c) 2017 Zheng Tang <zhtang@uw.edu>.  All rights reserved.
//
//  Description:
//      Implementation of tracking by detection, segmentation, calibration,
//      and constrained multiple kernels
//
//-----------------------------------------------------------------------------------

//#include <direct.h>	// in Windows
#include <sys/stat.h>	// in Linux
#include "Cfg.h"
#include "RoiSel.h"
#include "ObjDet.h"
#include "ChgDet.h"
#include "CamCal.h"
#include "ObjTrk.h"

int main(int argc, char *argv[])
{
	int nFrmCnt = 0, nProcFrmNum = 0;
	bool bPrevTstpFlg = false;
	double fTtlTmSec = 0.0;
	double fPrevTmSec, fCurrTmSec;	// in Windows
	std::time_t tCurrTstpSec, tPrevTstpSec;	// in Linux
	long nCurrTstpMsec, nPrevTstpMsec;	// in Linux
	//SYSTEMTIME tCurrSysTm;	// in Windows
	//struct _timeb stbCurrTm;	// in Windows
	struct timespec stsTstp;	// in Linux
	cv::VideoCapture oVdoCap;
	cv::VideoWriter oVdoWrt;
	cv::Mat oImgFrm, oImgRoi, oImgSegInit, oImgSegMsk;
	cv::Size oFrmSz;
	std::vector<CDetNd> voDetNd;
	CObjDet oObjDet;
	CChgDet oChgDet;
	CCamCal oCamCal;
	CObjTrk oObjTrk;
	CCfg oCfg;
	oCfg.ldCfgFl();

	// set starting frame count
	if (oCfg.getIdx1BasedFlg())
        nFrmCnt = 1;

	cv::namedWindow("current frame", CV_WINDOW_NORMAL);

	// read video source
	// from video file
	if (0 == oCfg.getInVdoTyp())
		oVdoCap = cv::VideoCapture(oCfg.getInVdoPth());
	// from image files
	else if (1 == oCfg.getInVdoTyp())
	{
        char acInImgNm[128] = { 0 };
        std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
        char acInImgPth[128] = { 0 };
        std::sprintf(acInImgPth, oCfg.getInFrmFlrPth());
        std::strcat(acInImgPth, acInImgNm);
		oImgFrm = cv::imread(acInImgPth, CV_LOAD_IMAGE_COLOR);
	}
	// from local camera
	else if (2 == oCfg.getInVdoTyp())
		oVdoCap = cv::VideoCapture(0);
	// from IP camera
	else if (3 == oCfg.getInVdoTyp())
		oVdoCap.open(oCfg.getInVdoPth());

	// handle error in reading video source
	if ((1 != oCfg.getInVdoTyp()) && (!oVdoCap.isOpened()))
	{
		std::cout << "Error: The video is not captured properly" << std::endl;
		return 0;
	}

	// test local/IP camera
	if ((2 == oCfg.getInVdoTyp()) || (3 == oCfg.getInVdoTyp()))
	{
		std::cout << "Test local/IP camera" << std::endl;
		cv::namedWindow("camera test", CV_WINDOW_NORMAL);

		// press "Esc" when the camera is ready
		while (1)
		{
			oVdoCap >> oImgFrm;
			cv::imshow("camera test", oImgFrm);
			int nKeyboardIn = cv::waitKey(0);	// read keyboard event
			if (nKeyboardIn == 27)
				break;
		}

		cv::destroyWindow("camera test");
	}

	// set frame size
	if (1 == oCfg.getInVdoTyp())
	{
		oFrmSz.width = oImgFrm.cols;
		oFrmSz.height = oImgFrm.rows;
	}
	else
	{
		oFrmSz.width = (int)oVdoCap.get(CV_CAP_PROP_FRAME_WIDTH);
		oFrmSz.height = (int)oVdoCap.get(CV_CAP_PROP_FRAME_HEIGHT);
	}

	// resize frame if necessary
	if (0 >= oCfg.getRszFrmHei())
		oCfg.setFrmSz(oFrmSz);
	else
	{
		oFrmSz = cv::Size((((float)oFrmSz.width / (float)oFrmSz.height) * oCfg.getRszFrmHei()), oCfg.getRszFrmHei());
		oCfg.setFrmSz(oFrmSz);
	}

	// set video frame rate
	if (0 == oCfg.getInVdoTyp())
		oCfg.setFrmRt(oVdoCap.get(CV_CAP_PROP_FPS));
	else
		oCfg.setFrmRt(oCfg.getOvrdFrmRt());

	// select ROI
	if (oCfg.getSelRoiFlg())
	{
		while (true)
		{
			int nKeyboardIn = cv::waitKey(0);	// read keyboard event

			if (nKeyboardIn == 'p')	// proceed to next frame
			{
				if (1 == oCfg.getInVdoTyp())
				{
                    char acInImgNm[128] = { 0 };
                    std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
                    char acInImgPth[128] = { 0 };
                    std::sprintf(acInImgPth, oCfg.getInFrmFlrPth());
                    std::strcat(acInImgPth, acInImgNm);
                    oImgFrm = cv::imread(acInImgPth, CV_LOAD_IMAGE_COLOR);
                    nFrmCnt++;
				}
				else
					oVdoCap >> oImgFrm;

				if (oImgFrm.empty())
					std::cout << "Error: Reach the end of the video." << std::endl;

				cv::imshow("current frame", oImgFrm);
			}

			if (nKeyboardIn == 27)	// press "Esc" to terminate
			{
				cv::destroyWindow("current frame");
				break;
			}

			if (nKeyboardIn == 's')			// select ROI
			{
				if (1 == oCfg.getInVdoTyp())
				{
                    char acInImgNm[128] = { 0 };
                    std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
                    char acInImgPth[128] = { 0 };
                    std::sprintf(acInImgPth, oCfg.getInFrmFlrPth());
                    std::strcat(acInImgPth, acInImgNm);
                    oImgFrm = cv::imread(acInImgPth, CV_LOAD_IMAGE_COLOR);
				}
				else
					oVdoCap >> oImgFrm;

				if (oImgFrm.empty())
					std::cout << "Error: Reach the end of the video." << std::endl;

				cv::resize(oImgFrm, oImgFrm, oFrmSz);

				oRoiSel.selRoi(oImgFrm, oCfg);
				cv::destroyWindow("current frame");
				break;
			}

			nFrmCnt++;
		}

		oImgRoi = cv::imread(oCfg.getInRoiPth(), CV_LOAD_IMAGE_GRAYSCALE);

		// reset video capture
		if (0 == oCfg.getInVdoTyp())
			oVdoCap = cv::VideoCapture(oCfg.getInVdoPth());
	}
	else
	{
		oImgRoi = cv::imread(oCfg.getInRoiPth(), CV_LOAD_IMAGE_GRAYSCALE);

		if (oImgRoi.empty())	// if ROI image does not exist, create a non-constraint ROI image
		{
			oImgRoi = cv::Mat(oFrmSz, CV_8UC1, cv::Scalar_<uchar>(255));
			cv::imwrite(oCfg.getInRoiPth(), oImgRoi);
		}
	}

	// set the ROI area
	oCfg.setRoiArea(cv::countNonZero(oImgRoi));

	// create video writer for output video
	if (oCfg.getOutVdoFlg())
	{
		if (0 == oCfg.getInVdoTyp())
			oVdoWrt = cv::VideoWriter(oCfg.getOutVdoPth(), CV_FOURCC('M', 'P', '4', '2'), (double)oVdoCap.get(CV_CAP_PROP_FPS), oFrmSz);
		else
			oVdoWrt = cv::VideoWriter(oCfg.getOutVdoPth(), CV_FOURCC('M', 'P', '4', '2'), oCfg.getFrmRt(), oFrmSz);
	}

	// create directory for output frame images
	if (oCfg.getOutFrmFlg())
		//_mkdir(oCfg.getOutFrmFlrPth());	// in Windows
		mkdir(oCfg.getOutFrmFlrPth(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux

	// create directory for output segmentation masks
	if (oCfg.getOutSegFlg())
		//_mkdir(oCfg.getOutSegFlrPth());	// in Windows
		mkdir(oCfg.getOutSegFlrPth(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux

	// create directory for output images
	if (oCfg.getOutImgFlg())
		//_mkdir(oCfg.getOutImgFlrPth());	// in Windows
		mkdir(oCfg.getOutImgFlrPth(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux

	// create directory for output images of face detection
	if (oCfg.getFcDetFlg())
		//_mkdir(oCfg.getOutFcDetFlrPth());	// in Windows
		mkdir(oCfg.getOutFcDetFlrPth(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux

	// camera calibration
	if (0 < oCfg.getInCalTyp())
	{
		// preliminary 2D tracking for self-calibration
		while (true)
		{
			// read video frame
			if (1 == oCfg.getInVdoTyp())
            {
                char acInImgNm[128] = { 0 };
                std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
                char acInImgPth[128] = { 0 };
                std::sprintf(acInImgPth, oCfg.getInFrmFlrPth());
                std::strcat(acInImgPth, acInImgNm);
                oImgFrm = cv::imread(acInImgPth, CV_LOAD_IMAGE_COLOR);
            }
            else
                oVdoCap >> oImgFrm;

			// resize video frame if necessary
			if (0 < oCfg.getRszFrmHei())
				cv::resize(oImgFrm, oImgFrm, oFrmSz);

			// run till the end of the video
			if ((oImgFrm.empty()) || ((0 < oCfg.getProcFrmNum()) && (oCfg.getProcFrmNum() < nProcFrmNum)))
				break;

			// manually end the process by pressing "Esc"
			int nKybdIn = cv::waitKey(1);	// read keyboard event
			if (nKybdIn == 27)
				break;

			// initialize at the beginning
			if (oCfg.getProcStFrmCnt() == nFrmCnt)
			{
				// initialize object detector
				oObjDet.initialize(oCfg, oImgRoi);

				// get initializing frame for segmentation
				if (oCfg.getInBgFlg())
				{
					cv::Mat oImgBg = cv::imread(oCfg.getInBgPth(), CV_LOAD_IMAGE_COLOR);

					if (0 < oCfg.getRszFrmHei())
						cv::resize(oImgBg, oImgBg, oFrmSz);

					oImgSegInit = oImgBg.clone();
				}
				else
					oImgSegInit = oImgFrm.clone();

				// initialize camera calibrator
				oCamCal.initialize(oCfg, oImgSegInit);

				// initialize change detector
				if (oCfg.getTrkSegFlg())
					oChgDet.initialize(oCfg, oImgSegInit, oImgRoi);

				// initialize tracker (for self-calibration)
				oObjTrk.initialize(oCfg, true);

				// manual calibration
				if (1 == oCfg.getInCalTyp())
				{
					oVanLnSel.initialize(oCfg, oImgSegInit);
					std::vector<cv::Point> oVanPt = oVanLnSel.process();
					oCamCal.process(oVanPt, oObjTrk.get1dNdLs(), oObjTrk.get1dNdInstNum());
					break;
				}
			}

			// for debug
			//if (-1 < nFrmCnt)
			//	cv::waitKey(0);

            if (oCfg.getProcStFrmCnt() <= nFrmCnt)
            {
                // run object detection
                oObjDet.process(oImgFrm, voDetNd, nFrmCnt);

                // run change detection
				if (oCfg.getTrkSegFlg())
					oChgDet.process(oImgFrm, oImgSegMsk, nFrmCnt);

                // run tracking
                oObjTrk.process(oImgFrm, voDetNd, oImgSegMsk, nFrmCnt);

                // run self-calibration
                if (oCamCal.process(std::vector<cv::Point>(), oObjTrk.get1dNdLs(), oObjTrk.get1dNdInstNum()))
                    break;

                nProcFrmNum++;
            }

			nFrmCnt++;
		}

		// check whether the text file of the camera parameters exists
		if (FILE *pfCamParam = fopen(oCfg.getInCamParamPth(), "r"))
			fclose(pfCamParam);
		// reach the end of the video: use all we have to perform self-calibration
		else
			oCamCal.process(std::vector<cv::Point>(), oObjTrk.get1dNdLs(), oObjTrk.get1dNdInstNum(), true);

        // selecting test points to evaluate the estimated camera parameters
        if (oCfg.getCalSelTstPtFlg())
        {
            oTstPtSel.initialize(oCfg, oImgSegInit);
            oTstPtSel.process();
        }
	}

	// reset video capture
	if (0 == oCfg.getInVdoTyp())
		oVdoCap = cv::VideoCapture(oCfg.getInVdoPth());

	// set starting frame count
	if (oCfg.getIdx1BasedFlg())
        nFrmCnt = 1;
	else
        nFrmCnt = 0;

	// regular tracking
	cv::namedWindow("current frame", CV_WINDOW_NORMAL);
	nProcFrmNum = 0;
	std::printf("\nStart regular tracking...\n");
	while (true)
	{
		// read video frame
		if (1 == oCfg.getInVdoTyp())
        {
            char acInImgNm[128] = { 0 };
            std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
            char acInImgPth[128] = { 0 };
            std::sprintf(acInImgPth, oCfg.getInFrmFlrPth());
            std::strcat(acInImgPth, acInImgNm);
            oImgFrm = cv::imread(acInImgPth, CV_LOAD_IMAGE_COLOR);
        }
        else
            oVdoCap >> oImgFrm;

		// resize video frame if necessary
		if (0 < oCfg.getRszFrmHei())
			cv::resize(oImgFrm, oImgFrm, oFrmSz);

		// run till the end of the video
		if ((oImgFrm.empty()) || ((0 < oCfg.getProcFrmNum()) && (oCfg.getProcFrmNum() < nProcFrmNum)))
			break;

		// manually end the process by pressing "Esc"
		int nKeyboardIn = cv::waitKey(1);	// read keyboard event
		if (nKeyboardIn == 27)
			break;

		// showing frame count and time stamp
		if ((0 == oCfg.getInVdoTyp()) || (1 == oCfg.getInVdoTyp()))
			std::printf("frame %06d\n", nFrmCnt);
		else
		{
			//GetSystemTime(&tCurrSysTm);	// in Windows
			//std::printf("frame %05d: %04d-%02d-%02d %02d:%02d:%02d.%03d\n", nFrmCnt, tCurrSysTm.wYear, tCurrSysTm.wMonth, tCurrSysTm.wDay,
			////	tCurrSysTm.wHour, tCurrSysTm.wMinute, tCurrSysTm.wSecond, tCurrSysTm.wMilliseconds);	// in Windows

			//_ftime(&stbCurrTm); // C4996	// in Windows
			//// Note: _ftime is deprecated; consider using _ftime_s instead
			//std::printf("frame %05d: %I64d%03d\n", nFrmCnt, stbCurrTm.time, stbCurrTm.millitm);	// in Windows

			tCurrTstpSec = (intmax_t)stsTstp.tv_sec;	// in Linux
			nCurrTstpMsec = round(stsTstp.tv_nsec / 1.0e6);	// in Linux
			clock_gettime(CLOCK_REALTIME, &stsTstp);	// in Linux
			std::printf("frame %05d: %lld.%03ld\n", nFrmCnt, (long long)tCurrTstpSec, nCurrTstpMsec);	// in Linux

			// output timestamp text file
			FILE* pfOutTstp = std::fopen(oCfg.getOutTstpPth(), "a");

			//std::fprintf(pfOutTstp, "\n%d,%04d%02d%02d,%02d%02d%02d%03d",
			//	m_nFrmCnt, tCurrSysTm.wYear, tCurrSysTm.wMonth, tCurrSysTm.wDay,
			//	tCurrSysTm.wHour, tCurrSysTm.wMinute, tCurrSysTm.wSecond, tCurrSysTm.wMilliseconds);	// in Windows

			//std::fprintf(pfOutTstp, "\n%d,%I64d%03d",
			//	nFrmCnt, stbCurrTm.time, stbCurrTm.millitm);	// in Windows

			std::fprintf(pfOutTstp, "\n%d,%lld.%03ld",
				nFrmCnt, (long long)tCurrTstpSec, nCurrTstpMsec);	// in Linux

			std::fclose(pfOutTstp);

			// compute total time in seconds that the program has been running and set the frame rate correspondingly
			if (oCfg.getProcTrkFlg())
			{
                if (oCfg.getProcStFrmCnt() < nFrmCnt)
                {
                   //fCurrTmSec = (float)stbCurrTm.time + ((float)stbCurrTm.millitm / 1000.0f);	// in Windows

                    if (bPrevTstpFlg)
                    {
                        //fTtlTmSec += (fCurrTmSec - fPrevTmSec);	// in Windows
                        double fDiffTmMsec = (double)(nCurrTstpMsec - nPrevTstpMsec) / 1.0e6;	// in Linux
                        fTtlTmSec += std::difftime(tCurrTstpSec, tPrevTstpSec) + ((0.0 < fDiffTmMsec) ? fDiffTmMsec : -fDiffTmMsec);	// in Linux
                        oObjTrk.setFrmRt((nFrmCnt - oCfg.getProcStFrmCnt() - 1.0) / fTtlTmSec);
                    }

                    //fPrevTmSec = fCurrTmSec;	// in Windows
                    tPrevTstpSec = tCurrTstpSec;	// in Linux
                    nPrevTstpMsec = nCurrTstpMsec;	// in Linux

                    if (!bPrevTstpFlg)
                        bPrevTstpFlg = true;
                }
			}
		}

		// initialize at the beginning
		if (oCfg.getProcStFrmCnt() == nFrmCnt)
		{
            // set starting time stamp
            if ((2 == oCfg.getInVdoTyp()) || (3 == oCfg.getInVdoTyp()))
            {
                //GetSystemTime(&tCurrSysTm);	// in Windows
                //oCfg.setStSysTm(tCurrSysTm);	// in Windows

                //_ftime(&stbCurrTm);	// C4996	// in Windows
                //// Note: _ftime is deprecated; consider using _ftime_s instead
                //oCfg.setStTstp(stbCurrTm);	// in Windows

                clock_gettime(CLOCK_REALTIME, &stsTstp);	// in Linux
                oCfg.setStTstp(stsTstp);	// in Linux
            }

			// initialize object detector
			oObjDet.initialize(oCfg, oImgRoi);

            if (oCfg.getTrkSegFlg())
            {
                // get initializing frame for segmentation
                if (oCfg.getInBgFlg())
                {
                    cv::Mat oImgBg = cv::imread(oCfg.getInBgPth(), CV_LOAD_IMAGE_COLOR);

                    if (0 < oCfg.getRszFrmHei())
                        cv::resize(oImgBg, oImgBg, oFrmSz);

                    oImgSegInit = oImgBg.clone();
                }
                else
                    oImgSegInit = oImgFrm.clone();

                // initialize change detector
				if (oCfg.getTrkSegFlg())
					oChgDet.initialize(oCfg, oImgSegInit, oImgRoi);
            }

			// initialize tracker (not for self-calibration)
			if (oCfg.getProcTrkFlg())
                oObjTrk.initialize(oCfg, false);
		}

		// for debug
		//if (-1 < nFrmCnt)
		//	cv::waitKey(0);

        if (oCfg.getProcStFrmCnt() <= nFrmCnt)
        {
            // save output (original) frame image
            if (oCfg.getOutFrmFlg())
            {
                char acOutFrmNm[128] = { 0 };
                std::sprintf(acOutFrmNm, "%06d.jpg", nFrmCnt);
                char acOutFrmPth[128] = { 0 };
                std::sprintf(acOutFrmPth, oCfg.getOutFrmFlrPth());
                std::strcat(acOutFrmPth, acOutFrmNm);
                cv::imwrite(acOutFrmPth, oImgFrm);
            }

            // run object detection
            oObjDet.process(oImgFrm, voDetNd, nFrmCnt);

            // run change detection
            if (oCfg.getTrkSegFlg())
                oChgDet.process(oImgFrm, oImgSegMsk, nFrmCnt);

            // run tracking
            if (oCfg.getProcTrkFlg())
                oObjTrk.process(oImgFrm, voDetNd, oImgSegMsk, nFrmCnt);

            // output segmentation results
            if (oCfg.getTrkSegFlg())
                oChgDet.output(oImgFrm, oImgSegMsk);

            // output object detection results
            oObjDet.output(oImgFrm, voDetNd);

            // output tracking results
            if (oCfg.getProcTrkFlg())
                oObjTrk.output(oImgFrm);

            cv::imshow("current frame", oImgFrm);
            //cv::waitKey(0);

            // save output segmentation masks
            if (oCfg.getTrkSegFlg() && oCfg.getOutSegFlg())
            {
                char acOutSegNm[128] = { 0 };
                std::sprintf(acOutSegNm, "%06d.jpg", nFrmCnt);
                char acOutSegPth[128] = { 0 };
                std::sprintf(acOutSegPth, oCfg.getOutSegFlrPth());
                std::strcat(acOutSegPth, acOutSegNm);
                cv::imwrite(acOutSegPth, oImgSegMsk);
            }

            // write (plotted) frame to output video
            if (oCfg.getOutVdoFlg())
                oVdoWrt.write(oImgFrm);

            // save output (plotted) frame image
            if (oCfg.getOutImgFlg())
            {
                char acOutImgNm[128] = { 0 };
                std::sprintf(acOutImgNm, "%06d.jpg", nFrmCnt);
                char acOutImgPth[128] = { 0 };
                std::sprintf(acOutImgPth, oCfg.getOutImgFlrPth());
                std::strcat(acOutImgPth, acOutImgNm);
                cv::imwrite(acOutImgPth, oImgFrm);
            }

            nProcFrmNum++;
        }

		nFrmCnt++;
	}

	cv::destroyWindow("current frame");

	return 0;
}
