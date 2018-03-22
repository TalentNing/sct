#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define IM1 2147483563
#define IM2 2147483399
#define AM (1.0/IM1)
#define IMM1 (IM1-1)
#define IA1 40014
#define IA2 40692
#define IQ1 53668
#define IQ2 52774
#define IR1 12211
#define IR2 3791
#define NTAB 32
#define NDIV (1+IMM1/NTAB)
#define EPS1 1.2e-7
#define RNMX (1.0-EPS1)

//! prints a matrix of uchar
static void prtMatUchar(cv::Mat oMat)
{
	for (int y = 0; y < oMat.rows; y++)
	{
		for (int x = 0; x < oMat.cols; x++)
		{
			std::printf("%03d ", oMat.at<uchar>(y, x));
		}
		std::printf("\n");
	}
}

//! prints a matrix of float
static void prtMatFlt(cv::Mat oMat)
{
	for (int y = 0; y < oMat.rows; y++)
	{
		for (int x = 0; x < oMat.cols; x++)
		{
			std::printf("%.3f ", oMat.at<float>(y, x));
		}
		std::printf("\n");
	}
}

//! prints a matrix of double
static void prtMatDbl(cv::Mat oMat)
{
	for (int y = 0; y < oMat.rows; y++)
	{
		for (int x = 0; x < oMat.cols; x++)
		{
			std::printf("%.3f ", oMat.at<double>(y, x));
		}
		std::printf("\n");
	}
}

//! prints a projection matrix
static void prtProjMat(float afProjMat[12])
{
	std::printf("%.3f %.3f %.3f %.3f\n", afProjMat[0], afProjMat[1], afProjMat[2], afProjMat[3]);
	std::printf("%.3f %.3f %.3f %.3f\n", afProjMat[4], afProjMat[5], afProjMat[6], afProjMat[7]);
	std::printf("%.3f %.3f %.3f %.3f\n", afProjMat[8], afProjMat[9], afProjMat[10], afProjMat[11]);
}

//! plots depth map
static void pltDepMap(cv::Mat oDepMap, int nObjNum)
{
	if (nObjNum > 0)
	{
		cv::Mat oImg = cv::Mat::zeros(oDepMap.size(), CV_8UC1);
		int nStp = 255 / nObjNum;

		for (int y = 0; y < oDepMap.rows; y++)
		{
			for (int x = 0; x < oDepMap.cols; x++)
			{
				oImg.at<uchar>(y, x) = oDepMap.at<int>(y, x) * nStp;
			}
		}

		//cv::imwrite("..\\data\\depmap.jpg", oImg);	// in Windows
		cv::imwrite("./data/depmap.jpg", oImg);	// in Linux
	}
}

//! compares the y coordinates of two points
static bool cmpPtY(cv::Point2f oPt1, cv::Point2f oPt2)	// used to sort pixel points
{
	return oPt1.y > oPt2.y;
}

//! generates expanded image
static cv::Mat genExpnImg(float fExpnRat, cv::Size oFrmSz)
{
	cv::Mat oImgExpn;

	if (1.0f >= fExpnRat)
	{
		oImgExpn = cv::Mat::zeros(cv::Size(oFrmSz.width, oFrmSz.height), CV_8UC3);
		return oImgExpn;
	}
	else
	{
		oImgExpn = cv::Mat::zeros(cv::Size((oFrmSz.width * fExpnRat), (oFrmSz.height * fExpnRat)), CV_8UC3);
		return oImgExpn;
	}
}

//! projects a point in the original image to the expanded image
static cv::Point2f projPtOrig2Expn(cv::Point2f oPtOrig, float fExpnRat, cv::Size oFrmSz)
{
	return cv::Point2f(oPtOrig.x + (oFrmSz.width * (fExpnRat - 1.0f) / 2.0f),
		oPtOrig.y + (oFrmSz.height * (fExpnRat - 1.0f) / 2.0f));
}

//! projects a point in the expanded image to the original image
static cv::Point2f projPtExpn2Orig(cv::Point2f oPtExpn, float fExpnRat, cv::Size oFrmSz)
{
	return cv::Point2f(oPtExpn.x - (oFrmSz.width * (fExpnRat - 1.0f) / 2.0f),
		oPtExpn.y - (oFrmSz.height * (fExpnRat - 1.0f) / 2.0f));
}

//! rotates a point by a given angle
static cv::Point2f rotPt(cv::Point2f oPt, float fAng)
{
	return cv::Point2f(((oPt.x * std::cos(fAng)) - (oPt.y * std::sin(fAng))),
		((oPt.x * std::sin(fAng)) + (oPt.y * std::cos(fAng))));
}

//! calculates the foot point and head point the intersections between the countour bounding box and its major axis
static std::vector<cv::Point2f> calcFtHdPt(cv::Rect2f oBBox, cv::Point2f oLnPt1, cv::Point2f oLnPt2)
{
	std::vector<cv::Point2f> voFtHdPt;

	float fLftIntxnX = oBBox.x;
	float fRgtIntxnX = oBBox.x + oBBox.width;
	float fTopIntxnY = oBBox.y;
	float fBtmIntxnY = oBBox.y + oBBox.height;
	float fLftIntxnY, fRgtIntxnY, fTopIntxnX, fBtmIntxnX;

	fLftIntxnY = (((fLftIntxnX - oLnPt2.x) / (oLnPt1.x - oLnPt2.x)) * (oLnPt1.y - oLnPt2.y)) + oLnPt2.y;
	fRgtIntxnY = (((fRgtIntxnX - oLnPt2.x) / (oLnPt1.x - oLnPt2.x)) * (oLnPt1.y - oLnPt2.y)) + oLnPt2.y;
	fTopIntxnX = (((fTopIntxnY - oLnPt2.y) / (oLnPt1.y - oLnPt2.y)) * (oLnPt1.x - oLnPt2.x)) + oLnPt2.x;
	fBtmIntxnX = (((fBtmIntxnY - oLnPt2.y) / (oLnPt1.y - oLnPt2.y)) * (oLnPt1.x - oLnPt2.x)) + oLnPt2.x;

	voFtHdPt.push_back(cv::Point2f(fLftIntxnX, fLftIntxnY));
	voFtHdPt.push_back(cv::Point2f(fRgtIntxnX, fRgtIntxnY));
	voFtHdPt.push_back(cv::Point2f(fTopIntxnX, fTopIntxnY));
	voFtHdPt.push_back(cv::Point2f(fBtmIntxnX, fBtmIntxnY));

	std::sort(voFtHdPt.begin(), voFtHdPt.end(), cmpPtY);
	// the two intersections points are always the two points in between
	voFtHdPt.erase(voFtHdPt.begin() + 3);
	voFtHdPt.erase(voFtHdPt.begin());

	// the two intersection points are both on the boxes
	if ((voFtHdPt[0].x >= oBBox.x) && (voFtHdPt[0].x <= (oBBox.x + oBBox.width)) && (voFtHdPt[0].y >= oBBox.y) && (voFtHdPt[0].y <= (oBBox.y + oBBox.height)) &&
		(voFtHdPt[1].x >= oBBox.x) && (voFtHdPt[1].x <= (oBBox.x + oBBox.width)) && (voFtHdPt[1].y >= oBBox.y) && (voFtHdPt[1].y <= (oBBox.y + oBBox.height)))
		return voFtHdPt;
	else
	{
		voFtHdPt[0] = cv::Point2f((oBBox.x + (oBBox.width / 2.0f)), (oBBox.y + oBBox.height));
		voFtHdPt[1] = cv::Point2f((oBBox.x + (oBBox.width / 2.0f)), oBBox.y);
		return voFtHdPt;
	}
}

//! projects 3D point to 2D pixel location
static cv::Point2f proj3d22d(cv::Point3f o3dPt, float afProjMat[12], int nLenUnit = 1)
{
	cv::Mat oMatP(3, 4, CV_32FC1, afProjMat);

	cv::Mat oMatM3d(4, 1, CV_32FC1);
	oMatM3d.at<float>(0, 0) = o3dPt.x * nLenUnit;
	oMatM3d.at<float>(1, 0) = o3dPt.y * nLenUnit;
	oMatM3d.at<float>(2, 0) = o3dPt.z * nLenUnit;
	oMatM3d.at<float>(3, 0) = 1.0f;

	cv::Mat oMatM2d(3, 1, CV_32FC1);
	oMatM2d = oMatP * oMatM3d;

	cv::Point2f o2dPt = cv::Point2f(oMatM2d.at<float>(0, 0) / oMatM2d.at<float>(2, 0),
		oMatM2d.at<float>(1, 0) / oMatM2d.at<float>(2, 0));

	return o2dPt;
}

//! backprojects 2D point to 3D ground position
static cv::Point3f bkproj2d23d(cv::Point2f o2dPt, float afProjMat[12], int nLenUnit = 1, int nCoordSysTyp = 1)
{
	cv::Point3f o3dPt;

	cv::Mat oMatA(3, 3, CV_64F);

	if (0 == nCoordSysTyp)
	{
		oMatA.at<double>(0, 0) = afProjMat[0];
		oMatA.at<double>(0, 1) = -o2dPt.x;
		oMatA.at<double>(0, 2) = afProjMat[2];
		oMatA.at<double>(1, 0) = afProjMat[4];
		oMatA.at<double>(1, 1) = -o2dPt.y;
		oMatA.at<double>(1, 2) = afProjMat[6];
		oMatA.at<double>(2, 0) = afProjMat[8];
		oMatA.at<double>(2, 1) = -1.0;
		oMatA.at<double>(2, 2) = afProjMat[10];
	}
	else if (1 == nCoordSysTyp)
	{
		oMatA.at<double>(0, 0) = afProjMat[0];
		oMatA.at<double>(0, 1) = afProjMat[1];
		oMatA.at<double>(0, 2) = -o2dPt.x;
		oMatA.at<double>(1, 0) = afProjMat[4];
		oMatA.at<double>(1, 1) = afProjMat[5];
		oMatA.at<double>(1, 2) = -o2dPt.y;
		oMatA.at<double>(2, 0) = afProjMat[8];
		oMatA.at<double>(2, 1) = afProjMat[9];
		oMatA.at<double>(2, 2) = -1.0;
	}

	cv::Mat oMatAInv(3, 3, CV_64F);
	cv::invert(oMatA, oMatAInv, cv::DECOMP_SVD);

	cv::Mat oMatB(3, 1, CV_64F);
	oMatB.at<double>(0, 0) = -afProjMat[3];
	oMatB.at<double>(1, 0) = -afProjMat[7];
	oMatB.at<double>(2, 0) = -afProjMat[11];

	cv::Mat oMatM(3, 1, CV_64F);
	oMatM = oMatAInv * oMatB;

	if (0 == nCoordSysTyp)
		o3dPt = cv::Point3f(oMatM.at<double>(0, 0), 0.0f, oMatM.at<double>(2, 0)) / nLenUnit;
	else if(1 == nCoordSysTyp)
		o3dPt = cv::Point3f(oMatM.at<double>(0, 0), oMatM.at<double>(1, 0), 0.0f) / nLenUnit;

	return o3dPt;
}

static float tst3dDist(cv::Mat& oImgPlt, cv::Point2f oPt1, cv::Point2f oPt2, float afProjMat[12], int nLenUnit = 1, int nCoordSysTyp = 1)
{
	cv::Point3f o3dPt1 = bkproj2d23d(oPt1, afProjMat, nLenUnit, nCoordSysTyp),
		o3dPt2 = bkproj2d23d(oPt2, afProjMat, nLenUnit, nCoordSysTyp);

	cv::circle(oImgPlt, oPt1, 5, cv::Scalar(0, 255, 0, 0), -2);
	cv::circle(oImgPlt, oPt2, 5, cv::Scalar(0, 255, 0, 0), -2);

	return cv::norm(o3dPt1 - o3dPt2);
}

//! calculates intersection over union (IOU) of two bounding boxes
static double calcBBoxIou(cv::Rect2f oBBox1, cv::Rect2f oBBox2)
{
	double fIntxnArea, fBBox1Area, fBBox2Area;

	// determine the (x, y)-coordinates of the intersection rectangle
	double fTopLftX = std::max(oBBox1.x, oBBox2.x);
	double fTopLftY = std::max(oBBox1.y, oBBox2.y);
	double fBtmRgtX = std::min((oBBox1.x + oBBox1.width), (oBBox2.x + oBBox2.width));
	double fBtmRgtY = std::min((oBBox1.y + oBBox1.height), (oBBox2.y + oBBox2.height));

	// compute the area of intersection rectangle
	if ((fBtmRgtX > fTopLftX) && (fBtmRgtY > fTopLftY))
		fIntxnArea = (fBtmRgtX - fTopLftX) * (fBtmRgtY - fTopLftY);
	// there is no intersection
	else
		return 0.0;

	// compute the area of both bounding boxes
	fBBox1Area = oBBox1.width * oBBox1.height;
	fBBox2Area = oBBox2.width * oBBox2.height;

	// compute the intersection over union by taking the intersection area and dividing it by the sum of bbox1 + bbox2 areas - the interesection area
	return (fIntxnArea / (fBBox1Area + fBBox2Area - fIntxnArea));
}

//! validates bounding box
static bool valBBox(cv::Rect2f& oBBox, cv::Size oFrmSz, cv::Mat oImgRoi)
{
	if (oBBox.x < 0.0f) { oBBox.x = 0.0f; }
	if (oBBox.y < 0.0f) { oBBox.y = 0.0f; }
	if ((oBBox.x + oBBox.width) >= oFrmSz.width) { oBBox.width = oFrmSz.width - oBBox.x - 1.0f; }
	if ((oBBox.y + oBBox.height) >= oFrmSz.height) { oBBox.height = oFrmSz.height - oBBox.y - 1.0f; }
	if ((oBBox.x >= 0.0f) && (oBBox.x < oFrmSz.width) && (oBBox.y >= 0.0f) && (oBBox.y < oFrmSz.height) && (oBBox.width > 0.0f) && (oBBox.height > 0.0f) &&
		((0 < oImgRoi.at<uchar>(cv::Point(oBBox.x, oBBox.y))) || (0 < oImgRoi.at<uchar>(cv::Point((oBBox.x + oBBox.width), oBBox.y))) ||
		(0 < oImgRoi.at<uchar>(cv::Point(oBBox.x, (oBBox.y + oBBox.height)))) || (0 < oImgRoi.at<uchar>(cv::Point((oBBox.x + oBBox.width), (oBBox.y + oBBox.height))))))
		return true;
	else
		return false;
}

// utils for randomization
/*// gaussian 3x3 pattern, based on 'floor(fspecial('gaussian', 3, 1)*256)'
static const int s_nSamplesInitPatternWidth = 3;
static const int s_nSamplesInitPatternHeight = 3;
static const int s_nSamplesInitPatternTot = 256;
static const int s_anSamplesInitPattern[s_nSamplesInitPatternHeight][s_nSamplesInitPatternWidth] = {
{19,    32,    19,},
{32,    52,    32,},
{19,    32,    19,},
};*/

// gaussian 7x7 pattern, based on 'floor(fspecial('gaussian',7,2)*512)'
static const int s_nSamplesInitPatternWidth = 7;
static const int s_nSamplesInitPatternHeight = 7;
static const int s_nSamplesInitPatternTot = 512;
static const int s_anSamplesInitPattern[s_nSamplesInitPatternHeight][s_nSamplesInitPatternWidth] = {
	{ 2,     4,     6,     7,     6,     4,     2, },
	{ 4,     8,    12,    14,    12,     8,     4, },
	{ 6,    12,    21,    25,    21,    12,     6, },
	{ 7,    14,    25,    28,    25,    14,     7, },
	{ 6,    12,    21,    25,    21,    12,     6, },
	{ 4,     8,    12,    14,    12,     8,     4, },
	{ 2,     4,     6,     7,     6,     4,     2, },
};

//! returns a random init/sampling position for the specified pixel position; also guards against out-of-bounds values via image/border size check.
static inline void getRandSamplePosition(int& x_sample, int& y_sample, const int x_orig, const int y_orig, const int border, const cv::Size& imgsize) {
	int r = 1 + rand() % s_nSamplesInitPatternTot;
	for (x_sample = 0; x_sample<s_nSamplesInitPatternWidth; ++x_sample) {
		for (y_sample = 0; y_sample<s_nSamplesInitPatternHeight; ++y_sample) {
			r -= s_anSamplesInitPattern[y_sample][x_sample];
			if (r <= 0)
				goto stop;
		}
	}
stop:
	x_sample += x_orig - s_nSamplesInitPatternWidth / 2;
	y_sample += y_orig - s_nSamplesInitPatternHeight / 2;
	if (x_sample<border)
		x_sample = border;
	else if (x_sample >= imgsize.width - border)
		x_sample = imgsize.width - border - 1;
	if (y_sample<border)
		y_sample = border;
	else if (y_sample >= imgsize.height - border)
		y_sample = imgsize.height - border - 1;
}

// simple 8-connected (3x3) neighbors pattern
static const int s_anNeighborPatternSize_3x3 = 8;
static const int s_anNeighborPattern_3x3[8][2] = {
	{ -1, 1 },{ 0, 1 },{ 1, 1 },
	{ -1, 0 },{ 1, 0 },
	{ -1,-1 },{ 0,-1 },{ 1,-1 },
};

//! returns a random neighbor position for the specified pixel position; also guards against out-of-bounds values via image/border size check.
static inline void getRandNeighborPosition_3x3(int& x_neighbor, int& y_neighbor, const int x_orig, const int y_orig, const int border, const cv::Size& imgsize) {
	int r = rand() % s_anNeighborPatternSize_3x3;
	x_neighbor = x_orig + s_anNeighborPattern_3x3[r][0];
	y_neighbor = y_orig + s_anNeighborPattern_3x3[r][1];
	if (x_neighbor<border)
		x_neighbor = border;
	else if (x_neighbor >= imgsize.width - border)
		x_neighbor = imgsize.width - border - 1;
	if (y_neighbor<border)
		y_neighbor = border;
	else if (y_neighbor >= imgsize.height - border)
		y_neighbor = imgsize.height - border - 1;
}

// 5x5 neighbors pattern
static const int s_anNeighborPatternSize_5x5 = 24;
static const int s_anNeighborPattern_5x5[24][2] = {
	{ -2, 2 },{ -1, 2 },{ 0, 2 },{ 1, 2 },{ 2, 2 },
	{ -2, 1 },{ -1, 1 },{ 0, 1 },{ 1, 1 },{ 2, 1 },
	{ -2, 0 },{ -1, 0 },{ 1, 0 },{ 2, 0 },
	{ -2,-1 },{ -1,-1 },{ 0,-1 },{ 1,-1 },{ 2,-1 },
	{ -2,-2 },{ -1,-2 },{ 0,-2 },{ 1,-2 },{ 2,-2 },
};

//! returns a random neighbor position for the specified pixel position; also guards against out-of-bounds values via image/border size check.
static inline void getRandNeighborPosition_5x5(int& x_neighbor, int& y_neighbor, const int x_orig, const int y_orig, const int border, const cv::Size& imgsize) {
	int r = rand() % s_anNeighborPatternSize_5x5;
	x_neighbor = x_orig + s_anNeighborPattern_5x5[r][0];
	y_neighbor = y_orig + s_anNeighborPattern_5x5[r][1];
	if (x_neighbor<border)
		x_neighbor = border;
	else if (x_neighbor >= imgsize.width - border)
		x_neighbor = imgsize.width - border - 1;
	if (y_neighbor<border)
		y_neighbor = border;
	else if (y_neighbor >= imgsize.height - border)
		y_neighbor = imgsize.height - border - 1;
}

//! generates a random double variable
static double rand2(long *idum)
{
	int j;
	long k;
	static long idum2 = 123456789;
	static long iy = 0;
	static long iv[NTAB];
	double temp;
	if (*idum <= 0) {
		if (-(*idum) < 1) *idum = 1;
		else *idum = -(*idum);
		idum2 = (*idum);
		for (j = NTAB + 7; j >= 0; j--) {
			k = (*idum) / IQ1;
			*idum = IA1*(*idum - k*IQ1) - k*IR1;
			if (*idum < 0) *idum += IM1;
			if (j < NTAB) iv[j] = *idum;
		}
		iy = iv[0];
	}
	k = (*idum) / IQ1;
	*idum = IA1*(*idum - k*IQ1) - k*IR1;
	if (*idum < 0) *idum += IM1;
	k = idum2 / IQ2;
	idum2 = IA2*(idum2 - k*IQ2) - k*IR2;
	if (idum2 < 0) idum2 += IM2;
	j = iy / NDIV;
	iy = iv[j] - idum2;
	iv[j] = *idum;
	if (iy < 1) iy += IMM1;
	if ((temp = AM*iy) > RNMX) return RNMX;
	else return temp;
}

//! generates a random number
static double get_rand_num(double max, double min, long seed)
{
	double rand = rand2(&seed);
	double duration = max - min;
	return min + rand*duration;
}

// utils for distance
//! popcount LUT for 8-bit vectors
static const uchar popcount_LUT8[256] = {
	0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
	1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
	1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
	1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
	3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
	4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
};

//! computes the population count of an N-byte vector using an 8-bit popcount LUT
template<typename T> static inline int popcount(T x) {
	int nBytes = sizeof(T);
	int nResult = 0;
	for (int l = 0; l<nBytes; ++l)
		nResult += popcount_LUT8[(uchar)(x >> l * 8)];
	return nResult;
}

//! computes the hamming distance between two N-byte vectors using an 8-bit popcount LUT
template<typename T> static inline int hdist(T a, T b) {
	return popcount(a^b);
}

//! computes the gradient magnitude distance between two N-byte vectors using an 8-bit popcount LUT
template<typename T> static inline int gdist(T a, T b) {
	return L1dist(popcount(a), popcount(b));
}

//! computes the population count of a (nChannels*N)-byte vector using an 8-bit popcount LUT
template<int nChannels, typename T> static inline int popcount(const T* x) {
	int nBytes = sizeof(T);
	int nResult = 0;
	for (int c = 0; c<nChannels; ++c)
		for (int l = 0; l<nBytes; ++l)
			nResult += popcount_LUT8[(uchar)(*(x + c) >> l * 8)];
	return nResult;
}

//! computes the hamming distance between two (nChannels*N)-byte vectors using an 8-bit popcount LUT
template<int nChannels, typename T> static inline int hdist(const T* a, const T* b) {
	T xor_array[nChannels];
	for (int c = 0; c<nChannels; ++c)
		xor_array[c] = a[c] ^ b[c];
	return popcount<nChannels>(xor_array);
}

//! computes the gradient magnitude distance between two (nChannels*N)-byte vectors using an 8-bit popcount LUT
template<int nChannels, typename T> static inline int gdist(const T* a, const T* b) {
	return L1dist(popcount<nChannels>(a), popcount<nChannels>(b));
}

// local binary pattern
namespace lbp
{
    template <typename _Tp>
    void OLBP_(const cv::Mat& oImgSrc, cv::Mat& oImgDst)
    {
        oImgDst = cv::Mat::zeros(oImgSrc.rows - 2, oImgSrc.cols - 2, CV_8UC1);
        for (int i = 1; i < oImgSrc.rows - 1; i++)
        {
            for (int j = 1; j < oImgSrc.cols - 1; j++)
            {
                _Tp tCent = oImgSrc.at<_Tp>(i, j);
                unsigned char ucCd = 0;
                ucCd |= (oImgSrc.at<_Tp>(i - 1, j - 1) > tCent) << 7;
                ucCd |= (oImgSrc.at<_Tp>(i - 1, j) > tCent) << 6;
                ucCd |= (oImgSrc.at<_Tp>(i - 1, j + 1) > tCent) << 5;
                ucCd |= (oImgSrc.at<_Tp>(i, j + 1) > tCent) << 4;
                ucCd |= (oImgSrc.at<_Tp>(i + 1, j + 1) > tCent) << 3;
                ucCd |= (oImgSrc.at<_Tp>(i + 1, j) > tCent) << 2;
                ucCd |= (oImgSrc.at<_Tp>(i + 1, j - 1) > tCent) << 1;
                ucCd |= (oImgSrc.at<_Tp>(i, j - 1) > tCent) << 0;
                oImgDst.at<unsigned char>(i - 1, j - 1) = ucCd;
            }
        }
    }

    template <typename _Tp>
    void ELBP_(const cv::Mat& oImgSrc, cv::Mat& oImgDst, int nRad = 1, int nNbrNum = 8)
    {
        nNbrNum = std::max(std::min(nNbrNum, 31), 1); // set bounds
        oImgDst = cv::Mat::zeros(oImgSrc.rows - 2 * nRad, oImgSrc.cols - 2 * nRad, CV_32SC1);
        for (int n = 0; n < nNbrNum; n++)
        {
            // sample points
            float x = static_cast<float>(nRad) * cos(2.0 * CV_PI * n / static_cast<float>(nNbrNum));
            float y = static_cast<float>(nRad) * -sin(2.0 * CV_PI * n / static_cast<float>(nNbrNum));
            // relative indices
            int fx = static_cast<int>(floor(x));
            int fy = static_cast<int>(floor(y));
            int cx = static_cast<int>(ceil(x));
            int cy = static_cast<int>(ceil(y));
            // fractional part
            float ty = y - fy;
            float tx = x - fx;
            // set interpolation weights
            float w1 = (1 - tx) * (1 - ty);
            float w2 = tx * (1 - ty);
            float w3 = (1 - tx) * ty;
            float w4 = tx * ty;
            // iterate through your data
            for (int i = nRad; i < oImgSrc.rows - nRad; i++)
            {
                for (int j = nRad; j < oImgSrc.cols - nRad; j++)
                {
                    float t = w1 * oImgSrc.at<_Tp>(i + fy, j + fx) + w2 * oImgSrc.at<_Tp>(i + fy, j + cx) +
                        w3 * oImgSrc.at<_Tp>(i + cy, j + fx) + w4 * oImgSrc.at<_Tp>(i + cy, j + cx);
                    // we are dealing with floating point precision, so add some little tolerance
                    oImgDst.at<unsigned char>(i - nRad, j - nRad) += ((t > oImgSrc.at<_Tp>(i, j)) && (abs(t - oImgSrc.at<_Tp>(i, j)) >
                        std::numeric_limits<float>::epsilon())) << n;
                }
            }
        }
    }

    template <typename _Tp>
    void VARLBP_(const cv::Mat& oImgSrc, cv::Mat& oImgDst, int nRad = 1, int nNbrNum = 8)
    {
        std::max(std::min(nNbrNum, 31), 1); // set bounds
        oImgDst = cv::Mat::zeros(oImgSrc.rows - 2 * nRad, oImgSrc.cols - 2 * nRad, CV_32FC1);
        cv::Mat _mean = cv::Mat::zeros(oImgSrc.rows, oImgSrc.cols, CV_32FC1);
        cv::Mat _delta = cv::Mat::zeros(oImgSrc.rows, oImgSrc.cols, CV_32FC1);
        cv::Mat _m2 = cv::Mat::zeros(oImgSrc.rows, oImgSrc.cols, CV_32FC1);
        for (int n = 0; n<nNbrNum; n++)
        {
            // sample points
            float x = static_cast<float>(nRad) * cos(2.0 * CV_PI * n / static_cast<float>(nNbrNum));
            float y = static_cast<float>(nRad) * -sin(2.0 * CV_PI * n / static_cast<float>(nNbrNum));
            // relative indices
            int fx = static_cast<int>(floor(x));
            int fy = static_cast<int>(floor(y));
            int cx = static_cast<int>(ceil(x));
            int cy = static_cast<int>(ceil(y));
            // fractional part
            float ty = y - fy;
            float tx = x - fx;
            // set interpolation weights
            float w1 = (1 - tx) * (1 - ty);
            float w2 = tx  * (1 - ty);
            float w3 = (1 - tx) *      ty;
            float w4 = tx  *      ty;
            // iterate through your data
            for (int i = nRad; i < oImgSrc.rows - nRad; i++)
            {
                for (int j = nRad; j < oImgSrc.cols - nRad; j++)
                {
                    float t = w1 * oImgSrc.at<_Tp>(i + fy, j + fx) + w2 * oImgSrc.at<_Tp>(i + fy, j + cx) +
                        w3 * oImgSrc.at<_Tp>(i + cy, j + fx) + w4 * oImgSrc.at<_Tp>(i + cy, j + cx);
                    _delta.at<float>(i, j) = t - _mean.at<float>(i, j);
                    _mean.at<float>(i, j) = (_mean.at<float>(i, j) + (_delta.at<float>(i, j) / (1.0*(n + 1))));
                    _m2.at<float>(i, j) = _m2.at<float>(i, j) + _delta.at<float>(i, j) * (t - _mean.at<float>(i, j));
                }
            }
        }
        // calculate result
        for (int i = nRad; i < oImgSrc.rows - nRad; i++)
        {
            for (int j = nRad; j < oImgSrc.cols - nRad; j++)
            {
                oImgDst.at<float>(i - nRad, j - nRad) = _m2.at<float>(i, j) / (1.0*(nNbrNum - 1));
            }
        }
    }
}
