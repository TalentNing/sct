#pragma once

#include <opencv2/video/tracking.hpp> // for Kalman filter
//#include <opencv2/tracking.hpp> // for visual tracking
#include "utils.h"
#include "Cfg.h"
#include "ObjDet.h"
#include "ChgDet.h"

//! define the type of coordinate system for camera calibration: 0: X-Z ground plane; 1: X-Y ground plane (default: 0)
#define COORD_SYS_TYP (1)
//! define the default state of object nodes
#define ND_ST8_DFLT (-1)
//! define the normal state of object nodes: ind obj w/ det bbox and seg blob
#define ND_ST8_NORM (0)
//! define the merged state of object nodes: ind obj w/ part of seg blob visible & det bbox
#define ND_ST8_MERG (1)
//! define the connected state of object nodes: obj connected w/ other obj/seg blob but not occluded
#define ND_ST8_CONN (2)
//! define the non-detected state of object nodes: ind obj w/ seg blob (matched w/ prev trk nodes) & no det bbox
#define ND_ST8_FN_DET (3)
//! define the occluded state of object nodes (by other object(s)): obj occluded by other obj
#define ND_ST8_OCCL_OBJ (4)
//! define the occluded state of object nodes (by background): ind obj w/ part of seg blob visible & no det bbox
#define ND_ST8_OCCL_BG (5)
//! define the occluded state of object nodes (by frame/ROI edge): ind obj w/ part of seg blob visible at frame/ROI edge
#define ND_ST8_OCCL_EDG (6)
//! define the grouped state of object nodes: obj close with more than one pred node
#define ND_ST8_GRP (7)
//! define the threshold for the IOU between matching nodes (default: 0.35f)
#define MTCH_ND_IOU_THLD (0.35f)
//! define the threshold for the IOU to matching tracking ndoe / left node with entering node (default: 0.10f)
#define NTR_LFT_IOU_THLD (0.10f)
//! define the threshold for the IOU between entering node and a previous trackinging node (default: 0.25f)
#define NTR_PREV_IOU_THLD (0.25f)
//! define the threshold for the IOU between two tracking nodes that indicates full overlap (default: 0.85f)
#define FULL_OVLP_IOU_THLD (0.85f)
//! define the threshold for the IOU that indicates close distance between two nodes (default: 0.10f)
#define CLS_DIST_IOU_THLD (0.10f)
//! define the threshold for visibility to determine the occluded-by-object state (default: 0.50f)
#define VIS_OCCL_OBJ_THLD (0.50f)
//! define the step size for the weight of 3D depth on distance measurement (default: 10)
#define DIST_WGT_3D_DEP_STP_SZ (10)
//! define the step size (ratio of frame height) for the weight of 2D depth on distance measurement (default: 0.333f)
#define DIST_WGT_2D_DEP_STP_SZ (0.333f)
//! define the increment for the weight of 3D and 2D depth on distance measurement (default: 0.50f)
#define DIST_WGT_DEP_INC (0.50f)
//! define the values for foot points coordinates in the process noise covariance matrix (Q) of Kalman filter (default: 1e-1)
#define KF_PROC_NOISE_COV_FTPT (1e-1)
//! define the values for velocities in the process noise covariance matrix (Q) of Kalman filter (default: 1e-1)
#define KF_PROC_NOISE_COV_VEL (1e-1)
//! define the values for sizes of bounding boxes in the process noise covariance matrix (Q) of Kalman filter (default: 1e-1)
#define KF_PROC_NOISE_COV_BBOX_SZ (1e-1)
//! define the values in the measurement noise covariance matrix (R) of Kalman filter (default: 1e-1)
#define KF_MEAS_NOISE_COV (1e-1)
//! define the values in the priori error estimate covariance matrix (P'(k)) of Kalman filter (default: 1)
#define KF_ERR_COV_PRE (1)
//! define the LBP operator for adaptive appearance model: 0: Extended LBP; 1: Fixed Sampling LBP; 2: Variance-based LBP (default: 1)
#define APP_MDL_LBP_OP (1)
//! define the type of spatial weight added on adaptive appearance model: 0: Uniform; 1: Gaussian; 2: Roof; 3: Epanechnikov (default: 0)
#define APP_MDL_WGT_TYP (1)
//! define the ratio threshold for the minimum area of visible part of an object to be compared with an appearance model (default: 5e-4)
#define APP_MDL_AREA_RAT_THLD (5e-4)
//! define the sigma of Gaussian weight for adaptive appearance model (default: 2.0)
#define APP_MDL_WGT_GAUSS_SIGMA (2.0)
//! define the threshold for changing rate of bounding box area for re-identification (default: 1.0f)
#define REID_PRED_BBOX_SZ_RT_THLD (1.0f)
//! define the threshold for changing rate of bounding box aspect ratio for re-identification (default: 1.0f)
#define REID_PRED_BBOX_AR_RT_THLD (1.0f)
//! define the scale factor for face detection (default: 1.1)
#define FC_DET_SCL_FAC (1.1)
//! define the number of neighbors each candidate rectangle should have to retain it for face detection (default: 3)
#define FC_DET_MIN_NGBR_NUM (3)
//! define the number of iterations to dilate the visible part of the occluded object for intersection (default: 10)
#define DIL_ITER_PART_OCCL_OBJ (10)
//! define the number of iterations to dilate after drawing contours to remove noise (default: 2)
#define DIL_ITER_RMV_CTR_NOISE (2)
//! define the number of iterations to erode after drawing contours to remove noise (default: 3)
#define ERO_ITER_RMV_CTR_NOISE (3)
//! define the threshold for the number of instances of each object class to enable Gaussian modeling (default: 100)
#define CLS_INST_NUM_THLD (1000)
//! define the breadth in pixels to determine frame/ROI edge case (default: 10)
#define FRM_EDG_BDTH (10)

// matching score in a matrix
class CMtchNdScr
{
public:
	CMtchNdScr(void);
	CMtchNdScr(int iRow, int iCol, double fScr);
	~CMtchNdScr(void);

	inline int getRowIdx(void) { return m_iRow; }
	inline void setRowIdx(int iRow) { m_iRow = iRow; }
	inline int getColIdx(void) { return m_iCol; }
	inline void setColIdx(int iCol) { m_iCol = iCol; }
	inline double getScr(void) { return m_fScr; }
	inline void setScr(double fScr) { m_fScr = fScr; }

protected:
	//! index of row
	int m_iRow;
	//! index of column
	int m_iCol;
	//! matching score
	double m_fScr;
};

// matching pair of candidate node and predicted node and their scores
class CMtchCandPredPr
{
public:
	CMtchCandPredPr(void);
	CMtchCandPredPr(int iCand, int iPred, double fMtchScr);
	~CMtchCandPredPr(void);

	inline int getCandIdx(void) { return m_iCand; }
	inline void setCandIdx(int iCand) { m_iCand = iCand; }
	inline int getPredIdx(void) { return m_iPred; }
	inline void setPredIdx(int iPred) { m_iPred = iPred; }
	inline double getMtchScr(void) { return m_fMtchScr; }
	inline void setMtchScr(double fMtchScr) { m_fMtchScr = fMtchScr; }

protected:
	//! index of candidate node
	int m_iCand;
	//! index of predicted node
	int m_iPred;
	//! matching score
	double m_fMtchScr;
};

// 1-dimension node for self-calibration
class C1dNd
{
public:
	C1dNd(void);
	C1dNd(int nFrmCnt, int nId, cv::Point2f o2dFtPt, cv::Point2f o2dHdPt);
	~C1dNd(void);

	inline int getFrmCnt(void) { return m_nFrmCnt; }
	inline void setFrmCnt(int nFrmCnt) { m_nFrmCnt = nFrmCnt; }
	inline int getId(void) { return m_nId; }
	inline void setId(int nId) { m_nId = nId; }
	inline cv::Point2f get2dFtPt(void) { return m_o2dFtPt; }
	inline void set2dFtPt(cv::Point2f o2dFtPt) { m_o2dFtPt = o2dFtPt; }
	inline cv::Point2f get2dHdPt(void) { return m_o2dHdPt; }
	inline void set2dHdPt(cv::Point2f o2dHdPt) { m_o2dHdPt = o2dHdPt; }

protected:
	//! frame count associated with object detection
	int m_nFrmCnt;
	//! object identity
	int m_nId;
	//! 2D foot point
	cv::Point2f m_o2dFtPt;
	//! 2D head point
	cv::Point2f m_o2dHdPt;
};

// trajectory of tracking interms of frame counts, bounding boxes, 2D foot points and 3D foot points
class CTraj
{
public:
	CTraj(void);
	CTraj(std::vector<int> vnFrmCnt, std::vector<cv::Rect2f> voBBox, std::vector<cv::Point2f> vo2dFtPt,
		std::vector<cv::Point3f> vo3dFtPt, std::vector<cv::Size2f> vo3dBBoxSz);
	~CTraj(void);

	inline int getTrajLen(void) { return m_vnTrajFrmCnt.size(); }
	inline void addTrajTrkNd(int nFrmCnt, cv::Rect2f oBBox, cv::Point2f o2dFtPt, cv::Point3f o3dFtPt, cv::Size2f o3dBBoxSz)
	{
		addTrajFrmCnt(nFrmCnt);
		addTrajBBox(oBBox);
		addTraj2dFtPt(o2dFtPt);
		addTraj3dFtPt(o3dFtPt);
		addTraj3dBBoxSz(o3dBBoxSz);
	}
	inline void rmv1stTrajTrkNd(void)
	{
		m_vnTrajFrmCnt.erase(m_vnTrajFrmCnt.begin());
		m_voTrajBBox.erase(m_voTrajBBox.begin());
		m_voTraj2dFtPt.erase(m_voTraj2dFtPt.begin());
		m_voTraj3dFtPt.erase(m_voTraj3dFtPt.begin());
		m_voTraj3dBBoxSz.erase(m_voTraj3dBBoxSz.begin());
	}
	inline std::vector<int> getTrajFrmCnts() { return m_vnTrajFrmCnt; }
	inline int getTrajFrmCnt(int nIdx) { return m_vnTrajFrmCnt[nIdx]; }
	inline void setTrajFrmCnts(std::vector<int> vnFrmCnt) { m_vnTrajFrmCnt = vnFrmCnt; }
	inline void addTrajFrmCnt(int nFrmCnt) { m_vnTrajFrmCnt.push_back(nFrmCnt); }
	inline std::vector<cv::Rect2f> getTrajBBoxs() { return m_voTrajBBox; }
	inline cv::Rect2f getTrajBBox(int nIdx) { return m_voTrajBBox[nIdx]; }
	inline void setTrajBBoxs(std::vector<cv::Rect2f> voBBox) { m_voTrajBBox = voBBox; }
	inline void addTrajBBox(cv::Rect2f oBBox) { m_voTrajBBox.push_back(oBBox); }
	inline std::vector<cv::Point2f> getTraj2dFtPts() { return m_voTraj2dFtPt; }
	inline cv::Point2f getTraj2dFtPt(int nIdx) { return m_voTraj2dFtPt[nIdx]; }
	inline void setTraj2dFtPts(std::vector<cv::Point2f> vo2dFtPt) { m_voTraj2dFtPt = vo2dFtPt; }
	inline void addTraj2dFtPt(cv::Point2f o2dFtPt) { m_voTraj2dFtPt.push_back(o2dFtPt); }
	inline std::vector<cv::Point3f> getTraj3dFtPts() { return m_voTraj3dFtPt; }
	inline cv::Point3f getTraj3dFtPt(int nIdx) { return m_voTraj3dFtPt[nIdx]; }
	inline void setTraj3dFtPts(std::vector<cv::Point3f> vo3dFtPt) { m_voTraj3dFtPt = vo3dFtPt; }
	inline void addTraj3dFtPt(cv::Point3f o3dFtPt) { m_voTraj3dFtPt.push_back(o3dFtPt); }
	inline std::vector<cv::Size2f> getTraj3dBBoxSzs() { return m_voTraj3dBBoxSz; }
	inline cv::Size2f getTraj3dBBoxSz(int nIdx) { return m_voTraj3dBBoxSz[nIdx]; }
	inline void setTraj3dBBoxSzs(std::vector<cv::Size2f> voTraj3dBBoxSz) { m_voTraj3dBBoxSz = voTraj3dBBoxSz; }
	inline void addTraj3dBBoxSz(cv::Size2f oTraj3dBBoxSz) { m_voTraj3dBBoxSz.push_back(oTraj3dBBoxSz); }

protected:
	//! frame counts
	std::vector<int> m_vnTrajFrmCnt;
	//! object bounding boxes
	std::vector<cv::Rect2f> m_voTrajBBox;
	//! 2D foot points
	std::vector<cv::Point2f> m_voTraj2dFtPt;
	//! 3D foot points
	std::vector<cv::Point3f> m_voTraj3dFtPt;
	//! 3D bounding box sizes
	std::vector<cv::Size2f> m_voTraj3dBBoxSz;
};

// adaptive appearance model for tracking
class CAppMdl
{
public:
	CAppMdl(void);
	CAppMdl(CCfg oCfg);
	CAppMdl(cv::Mat oMdlClr, cv::Mat oMdlLbp, cv::Mat oMdlGradMag, cv::Mat oMdlGradAng, cv::Mat oMdlSmpNum);
	~CAppMdl(void);

	inline cv::Mat getMdlClr(void) { return m_oMdlClr; }
	inline void setMdlClr(cv::Mat oMdlClr) { m_oMdlClr = oMdlClr; }
	inline cv::Vec3b getMdlClrSmp(int x, int y, int i) { return m_oMdlClr.at<cv::Vec3b>(x, y, i); }
	inline void setMdlClrSmp(int x, int y, int i, cv::Vec3b ovClrSmp) { m_oMdlClr.at<cv::Vec3b>(x, y, i) = ovClrSmp; }
	inline cv::Mat getMdlLbp(void) { return m_oMdlLbp; }
	inline void setMdlLbp(cv::Mat oMdlLbp) { m_oMdlLbp = oMdlLbp; }
	inline uchar getMdlLbpSmp(int x, int y, int i) { return m_oMdlLbp.at<uchar>(x, y, i); }
	inline void setMdlLbpSmp(int x, int y, int i, uchar ovLbpSmp) { m_oMdlLbp.at<uchar>(x, y, i) = ovLbpSmp; }
	inline cv::Mat getMdlGradMag(void) { return m_oMdlGradMag; }
	inline void setMdlGradMag(cv::Mat oMdlGradMag) { m_oMdlGradMag = oMdlGradMag; }
	inline cv::Vec2f getMdlGradMagSmp(int x, int y, int i) { return m_oMdlGradMag.at<cv::Vec2f>(x, y, i); }
	inline void setMdlGradMagSmp(int x, int y, int i, cv::Vec2f ovGradMagSmp) { m_oMdlGradMag.at<cv::Vec2f>(x, y, i) = ovGradMagSmp; }
	inline cv::Mat getMdlGradAng(void) { return m_oMdlGradAng; }
	inline void setMdlGradAng(cv::Mat oMdlGradAng) { m_oMdlGradAng = oMdlGradAng; }
	inline cv::Vec2b getMdlGradAngSmp(int x, int y, int i) { return m_oMdlGradAng.at<cv::Vec2b>(x, y, i); }
	inline void setMdlGradAngSmp(int x, int y, int i, cv::Vec2b ovGradAngSmp) { m_oMdlGradAng.at<cv::Vec2b>(x, y, i) = ovGradAngSmp; }
	inline cv::Mat getMdlSmpNum(void) { return m_oMdlSmpNum; }
	inline void setMdlSmpNum(cv::Mat oMdlSmpNum) { m_oMdlSmpNum = oMdlSmpNum; }
	inline int getMdlSmpNumPix(int x, int y) { return m_oMdlSmpNum.at<int>(x, y); }
	inline void setMdlSmpNumPix(int x, int y, int nSmpNum) { m_oMdlSmpNum.at<int>(x, y) = nSmpNum; }
	inline void incMdlSmpNumPix(int x, int y) { m_oMdlSmpNum.at<int>(x, y)++; }

	//! plots average appearance model
    void pltAppMdl(CCfg oCfg, char* acTxt);

protected:
    //! adaptive appearance model of color
	cv::Mat m_oMdlClr;
	//! adaptive appearance model of color
	cv::Mat m_oMdlLbp;
	//! adaptive appearance model of gradient magnitude
	cv::Mat m_oMdlGradMag;
	//! adaptive appearance model of gradient angle
	cv::Mat m_oMdlGradAng;
	//! number of samples at each pixel location
	cv::Mat m_oMdlSmpNum;
};

// candidate node for tracker to match with
class CCandNd : public CDetNd
{
public:
	CCandNd(void);
	CCandNd(CDetNd oDetNd, std::vector<cv::Point> voCtr, int nSt8);
	CCandNd(CDetNd oDetNd, int nSt8);
	CCandNd(std::vector<cv::Point> voCtr, int nSt8);
	CCandNd(cv::Rect oBBox, int nSt8);
	~CCandNd(void);

	inline void setDetNd(CDetNd oDetNd)
	{
		setFrmCnt(oDetNd.getFrmCnt());
		setDetBBox(oDetNd.getDetBBox());
		setDetScr(oDetNd.getDetScr());
		setDet3dFtPt(oDetNd.getDet3dFtPt());
		setDetCls(oDetNd.getDetCls());
	}
	inline std::vector<cv::Point> getCtr(void) { return m_voCtr; }
	inline void setCtr(std::vector<cv::Point> voCtr) { m_voCtr = voCtr; }
	inline cv::Moments getCtrMom(void) { return m_oCtrMom; }
	inline void setCtrMom(cv::Moments oCtrMom) { m_oCtrMom = oCtrMom; }
	inline cv::Rect2f getBBox(void) { return m_oBBox; }
	inline void setBBox(cv::Rect2f oBBox) { m_oBBox = oBBox; }
	inline cv::RotatedRect getElps(void) { return m_oElps; }
	inline void setElps(cv::RotatedRect oElps) { m_oElps = oElps; }
	inline cv::Point2f getMassCent(void) { return m_oMassCent; }
	inline void setMassCent(cv::Point2f oMassCent) { m_oMassCent = oMassCent; }
	inline double getArea(void) { return m_fArea; }
	inline void setArea(double fArea) { m_fArea = fArea; }
	inline cv::Point2f get2dFtPt(void) { return m_o2dFtPt; }
	inline void set2dFtPt(cv::Point2f o2dFtPt) { m_o2dFtPt = o2dFtPt; }
	inline cv::Point2f get2dHdPt(void) { return m_o2dHdPt; }
	inline void set2dHdPt(cv::Point2f o2dHdPt) { m_o2dHdPt = o2dHdPt; }
	inline cv::Point3f get3dFtPt(void) { return m_o3dFtPt; }
	inline void set3dFtPt(cv::Point3f o3dFtPt) { m_o3dFtPt = o3dFtPt; }
	inline cv::Size2f get3dBBoxSz(void) { return m_ov3dBBoxSz; }
	inline void set3dBBoxSz(cv::Size2f ov3dBBoxSz) { m_ov3dBBoxSz = ov3dBBoxSz; }
	inline float getDep(void) { return m_fDep; }
	inline void setDep(float fDep) { m_fDep = fDep; }
	inline int getCandDepIdx(void) { return m_nCandDepIdx; }
	inline void setCandDepIdx(int nCandDepIdx) { m_nCandDepIdx = nCandDepIdx; }
	inline double getCandVis(void) { return m_fCandVis; }
	inline void setCandVis(double fCandVis) { m_fCandVis = fCandVis; }
	inline int getSt8(void) { return m_nSt8; }
	inline void setSt8(int nSt8) { m_nSt8 = nSt8; }
	inline int getMtchFrmNum(void) { return m_nMtchFrmNum; }
	inline void setMtchFrmNum(int nMtchFrmNum) { m_nMtchFrmNum = nMtchFrmNum; }
	inline void decMtchFrmNum(void) { m_nMtchFrmNum = (m_nMtchFrmNum > 0) ? (m_nMtchFrmNum - 1) : 0; }
	inline void resetMtchFrmNum(void) { m_nMtchFrmNum = 0; }
	inline bool getMtchTrkFlg(void) { return m_bMtchTrkFlg; }
	inline void setMtchTrkFlg(bool bMtchTrkFlg) { m_bMtchTrkFlg = bMtchTrkFlg; }
	inline std::vector<int> getGrpId(void) { return m_viGrpId; }
	inline int getGrpIdNum(void) { return m_viGrpId.size(); }
	inline void setGrpId(std::vector<int> viGrpId) { m_viGrpId = viGrpId; }
    inline void addGrpId(int iGrpId) { m_viGrpId.push_back(iGrpId); }
    inline void resetGrpId(void) { std::vector<int>().swap(m_viGrpId); }

protected:
	//! contour of segmented object blob
	std::vector<cv::Point> m_voCtr;
	//! moments of segmented object blob
	cv::Moments m_oCtrMom;
	//! object bounding box
	cv::Rect2f m_oBBox;
	//! object ellipse
	cv::RotatedRect m_oElps;
	//! mass center
	cv::Point2f m_oMassCent;
	//! object area
	double m_fArea;
	//! 2D foot point
	cv::Point2f m_o2dFtPt;
	//! 2D head point
	cv::Point2f m_o2dHdPt;
	//! 3D foot point (in meter)
	cv::Point3f m_o3dFtPt;
	//! 3D bounding box size - width & height (in meter)
	cv::Size2f m_ov3dBBoxSz;
	//! distance to the camera (in meter)
	float m_fDep;
	//! depth index in the candidate depth map
	int m_nCandDepIdx;
	//! the candidate visibility (0 -> 1: more area visible)
	double m_fCandVis;
	//! state of the object node
	int m_nSt8;
	//! number of matching frames
	int m_nMtchFrmNum;
	//! flag of matching with tracking node
	bool m_bMtchTrkFlg;
	//! list of IDs that are close to the candidate node
	std::vector<int> m_viGrpId;
};

// tracking node
class CTrkNd : public CCandNd, public CTraj, public CAppMdl
{
public:
	CTrkNd(void);
	CTrkNd(CCandNd oCandNd, int nSt8, int nId, int nNtrFrmCnt, cv::KalmanFilter oKF);
	~CTrkNd(void);

	inline void setCandNd(CCandNd oCandNd, int nSt8 = ND_ST8_DFLT)
	{
		setDetNd(oCandNd);
		setCtr(oCandNd.getCtr());
		setCtrMom(oCandNd.getCtrMom());
		setBBox(oCandNd.getBBox());
		setElps(oCandNd.getElps());
		setMassCent(oCandNd.getMassCent());
		setArea(oCandNd.getArea());
		set2dFtPt(oCandNd.get2dFtPt());
		set2dHdPt(oCandNd.get2dHdPt());
		set3dFtPt(oCandNd.get3dFtPt());
		set3dBBoxSz(oCandNd.get3dBBoxSz());
		setDep(oCandNd.getDep());
        setCandDepIdx(oCandNd.getCandDepIdx());
		setCandVis(oCandNd.getCandVis());
		setSt8(nSt8);
		setMtchFrmNum(oCandNd.getMtchFrmNum());
		setMtchTrkFlg(true);
		setGrpId(oCandNd.getGrpId());
	}
	inline int getId(void) { return m_nId; }
	inline void setId(int nId) { m_nId = nId; }
	inline int getNtrFrmCnt(void) { return m_nNtrFrmCnt; }
	inline void setNtrFrmCnt(int nNtrFrmCnt) { m_nNtrFrmCnt = nNtrFrmCnt; }
	inline int getMtchCandTyp(void) { return m_nMtchCandTyp; }
	inline void setMtchCandTyp(int nMtchCandTyp) { m_nMtchCandTyp = nMtchCandTyp; }
	inline int getHypFrmNum(void) { return m_nHypFrmNum; }
	inline void setHypFrmNum(int nHypFrmNum) { m_nHypFrmNum = nHypFrmNum; }
	inline void incHypFrmNum(void) { m_nHypFrmNum++; }
	inline void decHypFrmNum(void) { m_nHypFrmNum = (m_nHypFrmNum > 0) ? (m_nHypFrmNum - 1) : 0; }
	inline void resetHypFrmNum(void) { m_nHypFrmNum = 0; }
	inline double getVis(void) { return m_fVis; }
	inline void setVis(double fVis) { m_fVis = fVis; }
	inline int getDepIdx(void) { return m_nDepIdx; }
	inline void setDepIdx(int nDepthInd) { m_nDepIdx = nDepthInd; }
	inline cv::KalmanFilter getKF(void) { return m_oKF; }
	inline void setKF(cv::KalmanFilter oKF) { m_oKF = oKF; }
	inline void setKFTmDiff(double fTmDiff)
	{
		m_oKF.transitionMatrix.at<double>(0, 2) = fTmDiff;
		m_oKF.transitionMatrix.at<double>(1, 3) = fTmDiff;
	}
	inline void resetKFErrCovPre(void) { cv::setIdentity(m_oKF.errorCovPre, cv::Scalar::all(KF_ERR_COV_PRE)); }
	inline void setKFSt8Post(cv::Mat oSt8Vec) { m_oKF.statePost = oSt8Vec; }
	inline cv::Mat predKF(void) { return m_oKF.predict(); }
	inline void corrKF(cv::Mat oMeasVec) { m_oKF.correct(oMeasVec); }
	inline CAppMdl getAppMdl(void) { return CAppMdl(m_oMdlClr, m_oMdlLbp, m_oMdlGradMag, m_oMdlGradAng, m_oMdlSmpNum); }
	inline void setAppMdl(CAppMdl oAppMdl)
	{
		setMdlClr(oAppMdl.getMdlClr());
		setMdlLbp(oAppMdl.getMdlLbp());
		setMdlGradMag(oAppMdl.getMdlGradMag());
		setMdlGradAng(oAppMdl.getMdlGradAng());
		setMdlSmpNum(oAppMdl.getMdlSmpNum());
	}
	inline CTraj getTraj(void) { return CTraj(m_vnTrajFrmCnt, m_voTrajBBox, m_voTraj2dFtPt, m_voTraj3dFtPt, m_voTraj3dBBoxSz); }
	inline void setTraj(CTraj oTraj)
	{
		setTrajFrmCnts(oTraj.getTrajFrmCnts());
		setTrajBBoxs(oTraj.getTrajBBoxs());
		setTraj2dFtPts(oTraj.getTraj2dFtPts());
		setTraj3dFtPts(oTraj.getTraj3dFtPts());
	}

protected:
	//! object identity
	int m_nId;
	//! the entering frame count
	int m_nNtrFrmCnt;
	//! matching type with a candidate node: -1: unmatched; 0: matched with occluded candidate; 1: matched normally
	int m_nMtchCandTyp;
	//! the number of frames that the tracking node is tracked as hypothesis (to prevent propagation of tracking error)
	int m_nHypFrmNum;
	//! the visibility (0 -> 1: more area visible)
	double m_fVis;
	//! the depth index (larger -> closer)
	int m_nDepIdx;
	//! Kalman filter
	cv::KalmanFilter m_oKF;
};

class CLftNd : public CCandNd, public CTraj, public CAppMdl
{
public:
	CLftNd(void);
	CLftNd(CTrkNd oTrkNd, int nFrmCnt, cv::Vec3f ovPred3dMov = cv::Vec3f(0.0f, 0.0f, 0.0f), cv::Vec2f ovPred3dBBoxSzChg = cv::Vec2f(0.0f, 0.0f),
		cv::Vec2f ovPred2dMov = cv::Vec2f(0.0f, 0.0f), cv::Vec2f ovPred2dBBoxSzChg = cv::Vec2f(0.0f, 0.0f));
	~CLftNd(void);

	inline void setTrkNd(CTrkNd oTrkNd, int nFrmCnt, cv::Vec3f ovPred3dMov = cv::Vec3f(0.0f, 0.0f, 0.0f), cv::Vec2f ovPred3dBBoxSzChg = cv::Vec2f(0.0f, 0.0f),
		cv::Vec2f ovPred2dMov = cv::Vec2f(0.0f, 0.0f), cv::Vec2f ovPred2dBBoxSzChg = cv::Vec2f(0.0f, 0.0f))
	{
		setDetNd(oTrkNd);
		setCtr(oTrkNd.getCtr());
		setCtrMom(oTrkNd.getCtrMom());
		setBBox(oTrkNd.getBBox());
		setElps(oTrkNd.getElps());
		setMassCent(oTrkNd.getMassCent());
		setArea(oTrkNd.getArea());
		set2dFtPt(oTrkNd.get2dFtPt());
		set2dHdPt(oTrkNd.get2dHdPt());
		set3dFtPt(oTrkNd.get3dFtPt());
		set3dBBoxSz(oTrkNd.get3dBBoxSz());
		setDep(oTrkNd.getDep());
		setCandDepIdx(oTrkNd.getCandDepIdx());
		setCandVis(oTrkNd.getCandVis());
		setSt8(oTrkNd.getSt8());
		setMtchFrmNum(oTrkNd.getMtchFrmNum());
		setMtchTrkFlg(oTrkNd.getMtchTrkFlg());
		setGrpId(oTrkNd.getGrpId());
		setId(oTrkNd.getId());
		setLftFrmCnt(nFrmCnt);
		setPred3dMov(ovPred3dMov);
		setPred3dBBoxSzChg(ovPred3dBBoxSzChg);
		setPred2dMov(ovPred2dMov);
		setPred2dBBoxSzChg(ovPred2dBBoxSzChg);
		setAppMdl(oTrkNd.getAppMdl());
		setTraj(oTrkNd.getTraj());
	}
	inline int getId(void) { return m_nId; }
	inline void setId(int nId) { m_nId = nId; }
	inline int getLftFrmCnt(void) { return m_nLftFrmCnt; }
	inline void setLftFrmCnt(int nLftFrmCnt) { m_nLftFrmCnt = nLftFrmCnt; }
	inline cv::Vec3f getPred3dMov(void) { return m_ovPred3dMov; }
	inline void setPred3dMov(cv::Vec3f ovPred3dMov) { m_ovPred3dMov = ovPred3dMov; }
	inline cv::Vec2f getPred3dBBoxSzChg(void) { return m_ovPred3dBBoxSzChg; }
	inline void setPred3dBBoxSzChg(cv::Vec2f ovPred3dBBoxSzChg) { m_ovPred3dBBoxSzChg = ovPred3dBBoxSzChg; }
	inline void shf2Pred3d(cv::Point3f& oPred3dFtPt, cv::Size2f& oPred3dBBoxSz, int nCurrFrmCnt)
	{
		int nFrmDiff = nCurrFrmCnt - m_nLftFrmCnt;
		oPred3dFtPt = cv::Point3f((m_o3dFtPt.x + (m_ovPred3dMov[0] * nFrmDiff)), (m_o3dFtPt.y + (m_ovPred3dMov[1] * nFrmDiff)),
			(m_o3dFtPt.z + (m_ovPred3dMov[2] * nFrmDiff)));
		oPred3dBBoxSz.width = m_ov3dBBoxSz.width + (m_ovPred3dBBoxSzChg[0] * (nCurrFrmCnt - m_nLftFrmCnt));
		oPred3dBBoxSz.height = m_ov3dBBoxSz.height + (m_ovPred3dBBoxSzChg[1] * (nCurrFrmCnt - m_nLftFrmCnt));
	}
	inline cv::Vec2f getPred2dMov(void) { return m_ovPred2dMov; }
	inline void setPred2dMov(cv::Vec2f ovPred2dMov) { m_ovPred2dMov = ovPred2dMov; }
	inline cv::Vec2f getPred2dBBoxSzChg(void) { return m_ovPred2dBBoxSzChg; }
	inline void setPred2dBBoxSzChg(cv::Vec2f ovPred2dBBoxSzChg) { m_ovPred2dBBoxSzChg = ovPred2dBBoxSzChg; }
	inline void shf2Pred2d(cv::Point2f& oPred2dFtPt, cv::Size2f& oPred2dBBoxSz, int nCurrFrmCnt)
	{
		int nFrmDiff = nCurrFrmCnt - m_nLftFrmCnt;
		oPred2dFtPt = cv::Point2f((m_o2dFtPt.x + (m_ovPred2dMov[0] * nFrmDiff)), (m_o2dFtPt.y + (m_ovPred2dMov[1] * nFrmDiff)));
		oPred2dBBoxSz.width = m_oBBox.width + (m_ovPred2dBBoxSzChg[0] * (nCurrFrmCnt - m_nLftFrmCnt));
		oPred2dBBoxSz.height = m_oBBox.height + (m_ovPred2dBBoxSzChg[1] * (nCurrFrmCnt - m_nLftFrmCnt));
	}
	inline CAppMdl getAppMdl(void) { return CAppMdl(m_oMdlClr, m_oMdlLbp, m_oMdlGradMag, m_oMdlGradAng, m_oMdlSmpNum); }
	inline void setAppMdl(CAppMdl oAppMdl)
	{
		setMdlClr(oAppMdl.getMdlClr());
		setMdlLbp(oAppMdl.getMdlLbp());
		setMdlGradMag(oAppMdl.getMdlGradMag());
		setMdlGradAng(oAppMdl.getMdlGradAng());
		setMdlSmpNum(oAppMdl.getMdlSmpNum());
	}
	inline CTraj getTraj(void) { return CTraj(m_vnTrajFrmCnt, m_voTrajBBox, m_voTraj2dFtPt, m_voTraj3dFtPt, m_voTraj3dBBoxSz); }
	inline void setTraj(CTraj oTraj)
	{
		setTrajFrmCnts(oTraj.getTrajFrmCnts());
		setTrajBBoxs(oTraj.getTrajBBoxs());
		setTraj2dFtPts(oTraj.getTraj2dFtPts());
		setTraj3dFtPts(oTraj.getTraj3dFtPts());
	}

protected:
	//! object identity for tracking
	int m_nId;
	//! the left frame count
	int m_nLftFrmCnt;
	//! predicted 3D movement (m^3/frame)
	cv::Vec3f m_ovPred3dMov;
	//! predicted 3D bounding box size change ratio - width & height
	cv::Vec2f m_ovPred3dBBoxSzChg;
	//! predicted 2D movement (pix^2/frame)
	cv::Vec2f m_ovPred2dMov;
	//! predicted 2D bounding box size change ratio - width & height
	cv::Vec2f m_ovPred2dBBoxSzChg;
};

class CObjTrk
{
public:
	CObjTrk(void);
	~CObjTrk(void);

	//! initializes the tracker
	void initialize(CCfg oCfg, bool bSlfCalFlg);
	//! run tracking algorithm
	void process(cv::Mat oImgFrm, std::vector<CDetNd>& voDetNd, cv::Mat oImgSegMsk, int nFrmCnt);
	//! outputs tracking results
	void output(cv::Mat& oImgFrm);
	//! sets video frame rate
	inline void setFrmRt(double fFrmRt) { m_oCfg.setFrmRt(fFrmRt); }
	//! gets the list of 1-dimension nodes for self-calibration
	inline std::vector<C1dNd> get1dNdLs(void) { return m_vo1dNd; }
	//! get the list of numbers of instances in terms of object IDs in the list of 1-dimension nodes
	inline std::vector<int> get1dNdInstNum(void) { return m_vn1dNdInstNum; }

private:
	//! detects entering tracking nodes
	void detNtrTrkNd(void);
	//! detects temporarily left nodes
	void detLftNd(void);
	//! clears invalid temporarily left nodes
	void clrLftNd(void);
	//! insert current node to the list of 1-dimension nodes for self-calibration if applicable
	void ins1dNdLs(void);
	//! inserts each current tracking node to its trajectory
	void insTrkNdTraj(void);
	//! sets the list of predicted nodes
	void initPredNdLs(void);
	//! performs tracking by two-way matching for candidate nodes in grouped states
	void trkGrp(void);
	//! initializes Kalman filter
	cv::KalmanFilter initKF(CCandNd oCandNd);
	//! performs Kalman filter tracking
	void trkKF(CTrkNd& oTrkNd);
	//! updates Kalman filter (matching each tracking node with candidate node)
	void updKF(CTrkNd& oTrkNd);
	////! performs visual tracking (matching each tracking node with candidate node)
	//void trkVis(CTrkNd& oTrkNd);
	//! performs tracking by prediction for unmatched tracking node(s)
	void trkPred(void);
	//! performs re-identification with left nodes
	int reIdLftNd(CCandNd oCandNdNtr);
	//! performs face detection on tracking node(s)
	void detFc(void);
	//! matches a tracking node with a candidate node
	void mtchTrkCand(CTrkNd& oTrkNd, cv::Rect2f oPredBBox, cv::Point3f oPred3dFtPt, cv::Point2f oPred2dFtPt);
	//! initializes the list of candidate nodes
	void initCandNdLs(std::vector<CDetNd>& voDetNd);
	//! matches detected bounding box(es) with segmented foreground blob(s)
	bool mtchDetSeg(cv::Mat& oMatDetSegIou, std::vector<bool>& voDetMtchFlg, std::vector<bool>& voSegMtchFlg,
		std::vector<CDetNd>& voDetNd, std::vector<std::vector<cv::Point> >& vvoCtr);
	//! matches unmatched segmented foreground blob(s) with predicted tracking nodes
	void mtchSegPred(cv::Mat& oMatSegTrkScr, std::vector<std::vector<cv::Point> >& voUnmtchCtr, std::vector<cv::Rect2f>& voUnmtchCtrBox);
	//! calculates 3D Gaussian models for different classes of objects
	void calc3dGaussMdl(void);
	//! shifts the candidate node to a predicted 3D position
	bool shfCandNd(CCandNd oCandNd, CCandNd& oPredCandNd, cv::Point3f oPred3dFtPt, cv::Size2f oPred3dBBoxSz);
	//! shifts the candidate node to a predicted 2D location
	bool shfCandNd(CCandNd oCandNd, CCandNd& oPredCandNd, cv::Point2f oPred2dFtPt, cv::Size2f oPred2dBBoxSz);
	//! calculates 3D information for a candidate node
	void calc3dInfo(CCandNd& oCandNd);
	//! generates depth map from the list of tracking nodes
	cv::Mat genDepMap(std::vector<CTrkNd> voTrkNd);
	//! generates depth map from the list of candidate nodes
	cv::Mat genDepMap(std::vector<CCandNd> voCandNd);
	//! calculates the visibility of tracking node
	double calcVis(cv::Rect2f oBBox, int iDep, cv::Mat oDepMap);
	//! updates the adaptive appearance model
	void updAppMdl(CTrkNd& oTrkNd);
	//! calculates the spatial weight for each pixel point in the adaptive appearance model
	double calcAppMdlWgt(cv::Point2d oPix, cv::Point2f oMassCent);
	//! calculates the matching score with an adaptive appearance model
	double calcAppMdlMtchScr(CAppMdl oAppMdl, cv::Mat oCandFrm, cv::Mat oCandMsk);
	//! outputs tranking results in txt file
	void outTxt(void);
	//! plots tracking bounding box(es) on the frame image
	void pltTrkBBox(cv::Mat& oImgFrm);
	//! plots candidate bounding box(es) on the frame image
	void pltCandBBox(cv::Mat& oImgFrm); // for debug

	//! configuration file
	CCfg m_oCfg;
	//! vector of colors of bounding boxes for plotting 2D tracking results
	std::vector<cv::Scalar> m_voBBoxClr;
	//! current frame count
	int m_nFrmCnt;
	//! maximum object ID
	int m_nMaxId;
	//! flag of 2D tracking for self-calibration
	bool m_bSlfCalFlg;
	//! dimension of tracking: 3 or 2
	int m_nTrkDim;
	//! projective matrix
	float m_afProjMat[12];
	//! cascade classifier for face detection
	cv::CascadeClassifier m_oFcCscdClf;
	//! HOG descriptor
	cv::HOGDescriptor m_oHogDesc;
	//! current frame image
	cv::Mat m_oCurrFrm;
	//! previous frame image
	cv::Mat m_oPrevFrm;
	//! current segmentated mask
	cv::Mat m_oCurrMsk;
	//! ROI image
	cv::Mat m_oImgRoi;
	//! depth map of candidate nodes
	cv::Mat m_oCandDepMap;
	//! number of instances for different classes of objects
	std::vector<int> m_vnClsInstNum;
	//! means of 3D sizes for different classes of objects
	std::vector<cv::Size2f> m_vo3dSzMean;
	//! variances of 3D sizes for different classes of objects
	std::vector<cv::Size2f> m_vo3dSzVar;
	//! means of 3D speeds for different classes of objects
	std::vector<double> m_vo3dVelMean;
	//! variances of 3D speeds for different classes of objects
	std::vector<double> m_vo3dVelVar;
	////! visual tracker
	//cv::Ptr<cv::Tracker> m_pTrkVis;
	//! list of candidate nodes in current frame
	std::vector<CCandNd> m_voCurrCandNd;
	//! list of candidate nodes in previous frame
	std::vector<CCandNd> m_voPrevCandNd;
	//! list of tracking nodes in current frame
	std::vector<CTrkNd> m_voCurrTrkNd;
	//! list of tracking nodes by 3D prediction
	std::vector<CTrkNd> m_voPredNd;
	//! list of temporarily left nodes
	std::vector<CLftNd> m_voLftNd;
	//! list of 1-dimension nodes for self-calibration
	std::vector<C1dNd> m_vo1dNd;
	//! list of numbers of instances in terms of object IDs in the list of 1-dimension nodes
	std::vector<int> m_vn1dNdInstNum;
};
