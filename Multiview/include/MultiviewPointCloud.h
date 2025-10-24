# pragma once
#include <opencv2/core.hpp>
#include <boost/filesystem/fstream.hpp>
#include <Eigen/Dense>
#include <k4a/k4a.hpp>
#include "PointCloudViewer.h"
#include "calibration.h"
#include "frustum.h"
#include "octree.h"
#include "utils.h"
#include "k4aview.h"
#include "k4autils.h"

using namespace std;
using namespace Eigen;

class MultiviewPointCloud
{
public:
	MultiviewPointCloud(const vector<uint32_t> &indices, const vector<string> &deviceSerials);
	~MultiviewPointCloud();

	void setDeviceIndices(const vector<uint32_t> &indices);
	void setDeviceSerials(const vector<string> &deviceSerials);

	vector<xy_table_t *> getXYTables();
	void setXYTables(const vector<xy_table_t *> &xyTables);

	void setImages(vector<cv::Mat> &colorImages, vector<cv::Mat> &depthImages);
	void setViews(const vector<View *> &views);
	void setImagesFromViews();
	bool loadTransformations(const string &path);

	void updatePointCloud(const vector<Calibration *> &extCalibration);
	void updatePointCloudNoCulling(const vector<Calibration *> &extCalibration);
	void updatePointCloudPlaneCulling(const vector<Calibration *> &extCalibration);
	void updatePointCloudClipCulling(const vector<Calibration *> &extCalibration);
	void updatePointCloudClipCullingFromViews(const vector<Calibration *> &extCalibration);
	void updatePointCloudOctreeCulling(const vector<Calibration *> &extCalibration);

	void updateViews(const vector<Calibration *> &extCalibration, uint32_t frameId);
	void storeViews(const string &work_dir);

	void initViewer();
	void viewPointCloud();
	void viewPointCloudFrustum();
	void viewPointCloudFrustumClip();
	void savePointCloud(const string &filename);
	uint32_t getViewSizeInBytes();

	Frustum *getFrustum();
	FrustumClip *getFrustumClip();
	void setFrustum(const Frustum &frustum);
	void setFrustumClip(const FrustumClip &frustum);

	void readFrustum(const string &filename);	// TODO: RAJRUP - Remove later
	void writeFrustum(const string &filename);	// TODO: RAJRUP - Remove later

	vector<vector<Point3s>> m_vLastFrameVertices;
    vector<vector<RGB>> m_vLastFrameRGB;

	vector<vector<Point3f>> m_vLastFrameVertices_f;
    vector<vector<RGB>> m_vLastFrameRGB_f;

	vector<View> m_views;
	uint32_t m_viewSizeInBytes;
	
	PointCloud_t::Ptr pclCloud;

private:
	bool loadCalibrationColorDownscaled(const string &path, const uint32_t index, k4a_calibration_t &calibrationColorDownscaled);
	void updateDepthPointCloudForColorFrame(uint32_t index);
	void initXYTables(const k4a_calibration_type_t camera);

	void updateFrustum(CamInt &camInt, CamView &camView);
	void updateFrustumClip(Matrix4d &camProjViewMat);
	
	void generatePointCloud();
	uint32_t getVerticesByIndex(uint32_t index);
	uint32_t getTotalVertices();

	void calcViewSizeInBytes();
	uint32_t numValidPointsInViews();

	// void initializeMultiview();

	// bool bInitialized;
	vector<uint32_t> device_indices;
	vector<string> device_serial_numbers;

	vector<k4a::image> colorImage; // Renamed colorImageDownscaled to colorImage
    vector<k4a::image> depthImage;

	vector<int> nColorFrameHeight, nColorFrameWidth;
	vector<int> nDepthFrameHeight, nDepthFrameWidth;

	vector<k4a::transformation> device_transformationColorDownscaled;
	vector<k4a_calibration_t> device_calibrationColorDownscaled;
	// vector<k4a::image> device_xytables;
	vector<xy_table_t *> xy_tables;

	vector<k4a::image> pointCloudImage;
	vector<k4a::image> transformedDepthImage;

	vector<Point3f *> pPointCloud;
	vector<uint16_t *> pDepth;
    vector<RGB *> pColorRGBX; // To Do: RAJRUP: change this to simple arrays instead of vector?
	vector<Point3fV *> ePointCloud;

	bool m_bFilter; // true if filtering is require
    int m_nFilterNeighbors;
	float m_fFilterThreshold;
	vector<float> m_vBounds;

	// float minX, maxX, minY, maxY, minZ, maxZ;
	Vector3f minBound, maxBound;
	
	Frustum m_frustum;
	FrustumClip m_frustumClip;

	Octree *p_octree;

	string windowName;
	PointCloudViewer *pViewer;
};