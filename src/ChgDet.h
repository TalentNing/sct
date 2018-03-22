#pragma once

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/background_segm.hpp>
#include "utils.h"
#include "Cfg.h"

//! specifies the pixel size of the pattern used (width and height)
#define PATCH_SIZE (5)
//! specifies the number of bytes per descriptor
#define DESC_SIZE (2)

class CChgDet
{
public:
	//! full constructor
	CChgDet(void);
	//! default destructor
	~CChgDet(void);
	//! (re)initiaization method; needs to be called before starting background subtraction
	void initialize(CCfg oCfg, const cv::Mat& oInitImg, const cv::Mat& oROI);
	//! refreshes all samples based on the last analyzed frame
	void refreshModel(float fSamplesRefreshFrac, bool bForceFGUpdate = false);
	//! primary model update function; the learning param is used to override the internal learning thresholds (ignored when < 0)
	void process(cv::Mat& oInImg, cv::Mat& oCurrFGMsk, int nFrmCnt);
	//! plots foreground masks on the output frame
	void output(cv::Mat& oInImg, cv::Mat oCurrFGMsk);
	//! returns a copy of the latest reconstructed background image
	void getBackgroundImage(cv::Mat backgroundImage) const;
	//! returns a copy of the latest reconstructed background descriptors image
	void getBackgroundDescriptorsImage(cv::Mat backgroundDescImage) const;
	//! retutns background model pixel color intensity samples
	std::vector<cv::Mat> getBgColorSamples() {return m_voBGColorSamples;}
	//! retutns number of different samples per pixel/block to be taken from input frames to build the background model
	int getBgSamples() {return m_nBGSamples;}
	//! retutns input image size
	cv::Size getImgSize() {return m_oImgSize;}

	//! utility function, shortcut/lightweight/direct single-point LBSP computation function for extra flexibility (1-channel version)
	inline static void computeGrayscaleDescriptor(const cv::Mat& oInImg, const uchar _ref, const int _x, const int _y, const int _t, ushort& _res) 
	{
		//CV_DbgAssert(!oInImg.empty());
		//CV_DbgAssert(oInImg.type() == CV_8UC1);
		//CV_DbgAssert(LBSP::DESC_SIZE == 2); // @@@ also relies on a constant desc size
		//CV_DbgAssert(_x >= (int)LBSP::PATCH_SIZE / 2 && _y >= (int)LBSP::PATCH_SIZE / 2);
		//CV_DbgAssert(_x<oInImg.cols - (int)LBSP::PATCH_SIZE / 2 && _y<oInImg.rows - (int)LBSP::PATCH_SIZE / 2);
		const int _step_row = oInImg.step.p[0];
		const uchar* const _data = oInImg.data;

		// note: this is the LBSP 16 bit double-cross single channel pattern
		// 
		//  O   O   O          4 ..  3 ..  6
		//    O O O           .. 15  8 13 ..
		//  O O X O O    =>    0  9  X 11  1
		//    O O O           .. 12 10 14 ..
		//  O   O   O          7 ..  2 ..  5

		_res = ((std::abs(_data[_step_row*(_y + 1) + _x - 1] - _ref) > _t) << 15)
			+ ((std::abs(_data[_step_row*(_y - 1) + _x + 1] - _ref) > _t) << 14)
			+ ((std::abs(_data[_step_row*(_y + 1) + _x + 1] - _ref) > _t) << 13)
			+ ((std::abs(_data[_step_row*(_y - 1) + _x - 1] - _ref) > _t) << 12)
			+ ((std::abs(_data[_step_row*(_y + 0) + _x + 1] - _ref) > _t) << 11)
			+ ((std::abs(_data[_step_row*(_y - 1) + _x + 0] - _ref) > _t) << 10)
			+ ((std::abs(_data[_step_row*(_y + 0) + _x - 1] - _ref) > _t) << 9)
			+ ((std::abs(_data[_step_row*(_y + 1) + _x + 0] - _ref) > _t) << 8)
			+ ((std::abs(_data[_step_row*(_y - 2) + _x - 2] - _ref) > _t) << 7)
			+ ((std::abs(_data[_step_row*(_y + 2) + _x + 2] - _ref) > _t) << 6)
			+ ((std::abs(_data[_step_row*(_y - 2) + _x + 2] - _ref) > _t) << 5)
			+ ((std::abs(_data[_step_row*(_y + 2) + _x - 2] - _ref) > _t) << 4)
			+ ((std::abs(_data[_step_row*(_y + 2) + _x + 0] - _ref) > _t) << 3)
			+ ((std::abs(_data[_step_row*(_y - 2) + _x + 0] - _ref) > _t) << 2)
			+ ((std::abs(_data[_step_row*(_y + 0) + _x + 2] - _ref) > _t) << 1)
			+ ((std::abs(_data[_step_row*(_y + 0) + _x - 2] - _ref) > _t));

	}

	//! utility function, shortcut/lightweight/direct single-point LBSP computation function for extra flexibility (3-channels version)
	inline static void computeRGBDescriptor(const cv::Mat& oInImg, const uchar* const _ref, const int _x, const int _y, const int* const _t, ushort* _res)
	{
		//CV_DbgAssert(!oInImg.empty());
		//CV_DbgAssert(oInImg.type() == CV_8UC3);
		//CV_DbgAssert(DESC_SIZE == 2); // @@@ also relies on a constant desc size
		//CV_DbgAssert(_x >= (int)PATCH_SIZE/2 && _y >= (int)PATCH_SIZE/2);
		//CV_DbgAssert(_x < oInImg.cols-(int)PATCH_SIZE/2 && _y < oInImg.rows-(int)PATCH_SIZE/2);
		const int _step_row = oInImg.step.p[0];
		const uchar* const _data = oInImg.data;

		// note: this is the LBSP 16 bit double-cross indiv RGB pattern
		//
		//  O   O   O          4 ..  3 ..  6
		//    O O O           .. 15  8 13 ..
		//  O O X O O    =>    0  9  X 11  1
		//    O O O           .. 12 10 14 ..
		//  O   O   O          7 ..  2 ..  5
		//           3x                     3x
		//

		for(int n=0; n<3; ++n) 
		{
            _res[n] = ((std::abs(_data[_step_row*(_y+1)+3*(_x-1)+n] - _ref[n]) > _t[n]) << 15)
                + ((std::abs(_data[_step_row*(_y-1)+3*(_x+1)+n] - _ref[n]) > _t[n]) << 14)
                + ((std::abs(_data[_step_row*(_y+1)+3*(_x+1)+n] - _ref[n]) > _t[n]) << 13)
                + ((std::abs(_data[_step_row*(_y-1)+3*(_x-1)+n] - _ref[n]) > _t[n]) << 12)
                + ((std::abs(_data[_step_row*(_y+0)+3*(_x+1)+n] - _ref[n]) > _t[n]) << 11)
                + ((std::abs(_data[_step_row*(_y-1)+3*(_x+0)+n] - _ref[n]) > _t[n]) << 10)
                + ((std::abs(_data[_step_row*(_y+0)+3*(_x-1)+n] - _ref[n]) > _t[n]) << 9)
                + ((std::abs(_data[_step_row*(_y+1)+3*(_x+0)+n] - _ref[n]) > _t[n]) << 8)
                + ((std::abs(_data[_step_row*(_y-2)+3*(_x-2)+n] - _ref[n]) > _t[n]) << 7)
                + ((std::abs(_data[_step_row*(_y+2)+3*(_x+2)+n] - _ref[n]) > _t[n]) << 6)
                + ((std::abs(_data[_step_row*(_y-2)+3*(_x+2)+n] - _ref[n]) > _t[n]) << 5)
                + ((std::abs(_data[_step_row*(_y+2)+3*(_x-2)+n] - _ref[n]) > _t[n]) << 4)
                + ((std::abs(_data[_step_row*(_y+2)+3*(_x+0)+n] - _ref[n]) > _t[n]) << 3)
                + ((std::abs(_data[_step_row*(_y-2)+3*(_x+0)+n] - _ref[n]) > _t[n]) << 2)
                + ((std::abs(_data[_step_row*(_y+0)+3*(_x+2)+n] - _ref[n]) > _t[n]) << 1)
                + ((std::abs(_data[_step_row*(_y+0)+3*(_x-2)+n] - _ref[n]) > _t[n]));
        }
	}

	//! utility function, shortcut/lightweight/direct single-point LBSP computation function for extra flexibility (1-channel version)
	inline static void computeSingleRGBDescriptor(const cv::Mat& oInImg, const uchar _ref, const int _x, const int _y, const int _c, const int _t, ushort& _res)
	{
		//CV_DbgAssert(!oInImg.empty());
		//CV_DbgAssert(oInImg.type() == CV_8UC3 && _c < 3);
		//CV_DbgAssert(DESC_SIZE == 2); // also relies on a constant desc size
		//CV_DbgAssert(_x >= (int)PATCH_SIZE/2 && _y >= (int)PATCH_SIZE/2);
		//CV_DbgAssert(_x < oInImg.cols-(int)PATCH_SIZE/2 && _y < oInImg.rows-(int)PATCH_SIZE/2);
		const int _step_row = oInImg.step.p[0];
		const uchar* const _data = oInImg.data;

		// note: this is the LBSP 16 bit double-cross indiv RGB pattern as used in
		// the original article by G.-A. Bilodeau et al.
		// 
		//  O   O   O          4 ..  3 ..  6
		//    O O O           .. 15  8 13 ..
		//  O O X O O    =>    0  9  X 11  1
		//    O O O           .. 12 10 14 ..
		//  O   O   O          7 ..  2 ..  5
		//          (single/3x)            (single/3x)
		//

        _res = ((std::abs(_data[_step_row*(_y+1)+3*(_x-1)+_c] - _ref) > _t) << 15)
            + ((std::abs(_data[_step_row*(_y-1)+3*(_x+1)+_c] - _ref) > _t) << 14)
            + ((std::abs(_data[_step_row*(_y+1)+3*(_x+1)+_c] - _ref) > _t) << 13)
            + ((std::abs(_data[_step_row*(_y-1)+3*(_x-1)+_c] - _ref) > _t) << 12)
            + ((std::abs(_data[_step_row*(_y+0)+3*(_x+1)+_c] - _ref) > _t) << 11)
            + ((std::abs(_data[_step_row*(_y-1)+3*(_x+0)+_c] - _ref) > _t) << 10)
            + ((std::abs(_data[_step_row*(_y+0)+3*(_x-1)+_c] - _ref) > _t) << 9)
            + ((std::abs(_data[_step_row*(_y+1)+3*(_x+0)+_c] - _ref) > _t) << 8)
            + ((std::abs(_data[_step_row*(_y-2)+3*(_x-2)+_c] - _ref) > _t) << 7)
            + ((std::abs(_data[_step_row*(_y+2)+3*(_x+2)+_c] - _ref) > _t) << 6)
            + ((std::abs(_data[_step_row*(_y-2)+3*(_x+2)+_c] - _ref) > _t) << 5)
            + ((std::abs(_data[_step_row*(_y+2)+3*(_x-2)+_c] - _ref) > _t) << 4)
            + ((std::abs(_data[_step_row*(_y+2)+3*(_x+0)+_c] - _ref) > _t) << 3)
            + ((std::abs(_data[_step_row*(_y-2)+3*(_x+0)+_c] - _ref) > _t) << 2)
            + ((std::abs(_data[_step_row*(_y+0)+3*(_x+2)+_c] - _ref) > _t) << 1)
            + ((std::abs(_data[_step_row*(_y+0)+3*(_x-2)+_c] - _ref) > _t));
	}

	//! utility function, used to filter out bad pixels in a ROI that would trigger out of bounds error because they're too close to the image border
	static void validateROI(cv::Mat& oROI);

protected:
	struct PxInfoBase 
	{
		int nImgCoord_Y;
		int nImgCoord_X;
		int nModelIdx;
	};

	//! type of input video source: 0: video file; 1: image folder; 2: online KNN; 3: online MOG2; 4: online SuBSENSE
	int m_nInSegTyp;
	//! video results of segmentation, necessary when m_bInSegTyp = 0
	cv::VideoCapture m_oSegCap;
	//! path of input video file of segmentation, necessary when m_bInSegTyp = 0
	char m_acInSegVdoPth[256];
	//! path of input image folder of segmentation, necessary when m_bInSegTyp = 1
	char m_acInSegImgFlrPth[256];
	//! flag of plotting segmentation
	bool m_bPltSegFlg;
	//! the ratio to resize the original image for segmentation
	float m_fSegRszRat;
	//! the overridden learning rate (-1.0: default/adaptive), necessary when m_nInSegTyp > 1
	float m_fLrnRtOvrd;

	//! background model ROI used for LBSP descriptor extraction (specific to the input image size)
	cv::Mat m_oROI;
	//! input image size
	cv::Size m_oImgSize;
	//! input image channel size
	int m_nImgChannels;
	//! input image type
	int m_nImgType;
	//! LBSP internal threshold offset value, used to reduce texture noise in dark regions
	int m_nLBSPThresholdOffset;
	//! LBSP relative internal threshold (kept here since we don't keep an LBSP object)
	float m_fRelLBSPThreshold;
	//! total number of pixels (depends on the input frame size) & total number of relevant pixels
	int m_nTotPxCount, m_nTotRelevantPxCount;
	//! current frame index, frame count since last model reset & model reset cooldown counters
	int m_nFrameIndex, m_nFramesSinceLastReset, m_nModelResetCooldown;
	//! pre-allocated internal LBSP threshold values LUT for all possible 8-bit intensities
	int m_anLBSPThreshold_8bitLUT[UCHAR_MAX + 1];
	//! internal pixel index LUT for all relevant analysis regions (based on the provided ROI)
	int* m_aPxIdxLUT;
	//! internal pixel info LUT for all possible pixel indexes
	PxInfoBase* m_aPxInfoLUT;
	//! default kernel size for median blur post-proc filtering
	int m_nDefaultMedianBlurKernelSize;
	//! specifies whether the algorithm is fully initialized or not
	bool m_bInitialized;
	//! specifies whether automatic model resets are enabled or not
	bool m_bAutoModelResetEnabled;
	//! specifies whether the camera is considered moving or not
	bool m_bUsingMovingCamera;
	//! copy of latest pixel intensities (used when refreshing model)
	cv::Mat m_oLastColorFrame;
	//! copy of latest descriptors (used when refreshing model)
	cv::Mat m_oLastDescFrame;
	//! the foreground mask generated by the method at [t-1]
	cv::Mat m_oLastFGMsk;

	//! the history length for KNN and MOG2
	int m_nHstLen;
	//! threshold on the squared distance for KNN (default: 400.0) and MOG2 (default: 16.0)
	float m_fDist2Thld;
	//! absolute minimal color distance threshold ('R' or 'radius' in the original ViBe paper, used as the default/initial 'R(x)' value here)
	int m_nMinColorDistThreshold;
	//! absolute descriptor distance threshold offset
	int m_nDescDistThresholdOffset;
	//! number of different samples per pixel/block to be taken from input frames to build the background model (same as 'N' in ViBe/PBAS)
	int m_nBGSamples;
	//! number of similar samples needed to consider the current pixel/block as 'background' (same as '#_min' in ViBe/PBAS)
	int m_nRequiredBGSamples;
	//! number of samples to use to compute the learning rate of moving averages
	int m_nSamplesForMovingAvgs;
	////! size threshold for blobs (near)
	//float m_fBlobSzThresNear;
	////! size threshold for blobs (middle)
	//float m_fBlobSzThresMid;
	////! size threshold for blobs (far)
	//float m_fBlobSzThresFar;
	//! maximum length of gap between blobs in x direction
	int m_nFillGapX;
	//! maximum length of gap between blobs in y direction
	int m_nFillGapY;
	//! perform shadow removal
	bool m_bShdwRmvFlg;
	//! shadow thresholding for maximum Y ratio
	float m_fShdwYRatMax;
	//! shadow thresholding for minimum Y ratio
	float m_fShdwYRatMin;
	//! shadow thresholding for maximum Cr difference
	int m_nShdwCrDiffMax;
	//! shadow thresholding for maximum Cb difference
	int m_nShdwCbDiffMax;
	//! last calculated non-zero desc ratio
	float m_fLastNonZeroDescRatio;
	//! specifies whether Tmin/Tmax scaling is enabled or not
	bool m_bLearningRateScalingEnabled;
	//! current learning rate caps
	float m_fCurrLearningRateLowerCap, m_fCurrLearningRateUpperCap;
	//! current kernel size for median blur post-proc filtering
	int m_nMedianBlurKernelSize;
	//! specifies the px update spread range
	bool m_bUse3x3Spread;
	//! specifies the downsampled frame size used for cam motion analysis
	cv::Size m_oDownSampledFrameSize;

	//! background model pixel color intensity samples (equivalent to 'B(x)' in PBAS)
	std::vector<cv::Mat> m_voBGColorSamples;
	//! background model descriptors samples
	std::vector<cv::Mat> m_voBGDescSamples;

	//! per-pixel update rates ('T(x)' in PBAS, which contains pixel-level 'sigmas', as referred to in ViBe)
	cv::Mat m_oUpdateRateFrame;
	//! per-pixel distance thresholds (equivalent to 'R(x)' in PBAS, but used as a relative value to determine both intensity and descriptor variation thresholds)
	cv::Mat m_oDistThresholdFrame;
	//! per-pixel distance variation modulators ('v(x)', relative value used to modulate 'R(x)' and 'T(x)' variations)
	cv::Mat m_oVariationModulatorFrame;
	//! per-pixel mean distances between consecutive frames ('D_last(x)', used to detect ghosts and high variation regions in the sequence)
	cv::Mat m_oMeanLastDistFrame;
	//! per-pixel mean minimal distances from the model ('D_min(x)' in PBAS, used to control variation magnitude and direction of 'T(x)' and 'R(x)')
	cv::Mat m_oMeanMinDistFrame_LT, m_oMeanMinDistFrame_ST;
	//! per-pixel mean downsampled distances between consecutive frames (used to analyze camera movement and control max learning rates globally)
	cv::Mat m_oMeanDownSampledLastDistFrame_LT, m_oMeanDownSampledLastDistFrame_ST;
	//! per-pixel mean raw segmentation results (used to detect unstable segmentation regions)
	cv::Mat m_oMeanRawSegmResFrame_LT, m_oMeanRawSegmResFrame_ST;
	//! per-pixel mean raw segmentation results (used to detect unstable segmentation regions)
	cv::Mat m_oMeanFinalSegmResFrame_LT, m_oMeanFinalSegmResFrame_ST;
	//! a lookup map used to keep track of unstable regions (based on segm. noise & local dist. thresholds)
	cv::Mat m_oUnstableRegionMsk;
	//! per-pixel blink detection map ('Z(x)')
	cv::Mat m_oBlinksFrame;
	//! pre-allocated matrix used to downsample the input frame when needed
	cv::Mat m_oDownSampledFrame_MotionAnalysis;
	//! the foreground mask generated by the method at [t-1] (without post-proc, used for blinking px detection)
	cv::Mat m_oLastRawFGMsk;

	//! pre-allocated CV_8UC1 matrices used to speed up morph ops
	cv::Mat m_oFGMsk_PreFlood;
	cv::Mat m_oFGMsk_FloodedHoles;
	cv::Mat m_oLastFGMsk_dilated;
	cv::Mat m_oLastFGMsk_dilated_inverted;
	cv::Mat m_oCurrRawFGBlinkMsk;
	cv::Mat m_oLastRawFGBlinkMsk;

	// MOG background subtractor
	cv::Ptr<cv::BackgroundSubtractor> m_pBgSubKNN;	// KNN background subtractor
	cv::Ptr<cv::BackgroundSubtractor> m_pBgSubMOG2;	// MOG2 background subtractor

	//! removes shadow in the foreground mask
	bool detectShdw(const uchar* const anCurrColor, const uchar* const anBgColor);
	////! filters out foreground blobs whose sizes are too small
	//void filterFgBlobSz(cv::Mat oFgMsk);
	//! fills gaps in x direction and y direction (only between different blobs but not inside one blob)
	void fillBlobGap(cv::Mat oFgMsk);
};
