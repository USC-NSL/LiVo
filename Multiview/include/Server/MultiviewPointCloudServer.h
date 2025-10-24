#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include <boost/filesystem/fstream.hpp>
#include <Eigen/Dense>
#include <k4a/k4a.hpp>

#include <draco/compression/config/compression_shared.h>
#include <draco/compression/encode.h>
#include <draco/compression/expert_encode.h>
#include <draco/core/cycle_timer.h>
#include <draco/io/file_utils.h>
#include <draco/io/point_cloud_io.h>

#include "absl/container/flat_hash_map.h"
#include "consts.h"
#include "view.h"
#include "kalman.h"
#include "frustum.h"
#include "voxel.h"
#include "utils.h"

using namespace std;

void calc_max_min_corners_frustum(const Frustum &frustum, Vector3f &min_corner, Vector3f &max_corner);

class MultiviewPointCloudServer
{

public:

	MultiviewPointCloudServer(const vector<uint32_t> &indices, const vector<string> &deviceSerials, const int datasetType=DATASET::KINECT);
	~MultiviewPointCloudServer();

	void setDeviceIndices(const vector<uint32_t> &indices);
	void setDeviceSerials(const vector<string> &deviceSerials);

	vector<xy_table_t *> getXYTables();
	bool loadTransformations(const string &path);

	void setImages(const vector<pair<uint32_t, cv::Mat>> &colorImages, const vector<pair<uint32_t, cv::Mat>> &depthImages);
	void updateViewsClipCulling(const vector<Calibration *> &extCalibration, int frameID);
	void updateViewsNormalCulling(const vector<Calibration *> &extCalibration, int frameID);
	void updateViewsExpansionNormalCulling(const vector<Calibration *> &extCalibration, int frameID, float expansionFactor=1.0f);
	void updateViewsExpansionNormalCullingUnion(const vector<Calibration *> &extCalibration, int frameID, float expansionFactor=1.0f);
	void updateViewsExpansionNormalCullingBuffer(const vector<Calibration *> &extCalibration, int frameID);
	void updateViewsFromVoxelIncremental(const vector<Calibration *> &extCalibration, int frameID);
	void storeViews(const string &work_dir);
	void storeViews_standalone(const string &work_dir);

	void addQR2Views(const string &work_dir, int frameID);

	void setUserData(const UserData &userData);
	void setFrustum(const Frustum &frustum);
	void setFrustum(const Frustum &prev_frustum, const Frustum &pred_frustum);
	void setFrustumClip(const FrustumClip &frustum);

	vector<View> m_views;
	uint32_t m_viewSizeWithFrustumInBytes;
	uint32_t m_viewSizeWithoutFrustumInBytes;

	// For pointcloud transmission only
	void updatePointCloud(const vector<Calibration *> &extCalibration);
	void updatePointCloudFromVoxel(const vector<Calibration *> &extCalibration);
	int getNumPoints();
	uint8_t *compressPointCloudZstd(int &size, int compressionLevel);
	uint8_t *compressPointCloudDraco(int &size, int cl, int qp);

private:
	bool loadCalibrationColorDownscaled(const string &path, const uint32_t index, k4a_calibration_t &calibrationColorDownscaled);
	void initXYTables(const k4a_calibration_type_t camera);

	void calcViewSizeInBytes();
	uint32_t numValidPointsInViews();

	uint32_t getVerticesByIndex(uint32_t index);
	
	int m_datasetType;

	vector<uint32_t> device_indices;
	vector<string> device_serial_numbers;

	vector<k4a_calibration_t> device_calibrationColorDownscaled;
	vector<k4a::transformation> device_transformationColorDownscaled;
	vector<xy_table_t *> xy_tables;

	vector<int> nColorFrameHeight, nColorFrameWidth;
	vector<int> nDepthFrameHeight, nDepthFrameWidth;

	vector<k4a::image> colorImage; // Renamed colorImageDownscaled to colorImage
    vector<k4a::image> depthImage;
	vector<k4a::image> transformedDepthImage;

	vector<uint16_t *> pDepth;
	vector<Point3f *> pPoint;
    vector<RGB *> pColorRGBX; // To Do: RAJRUP: change this to simple arrays instead of vector?

	Vector3f minBound, maxBound;

	Frustum m_frustum;
	Frustum m_prevFrustum, m_predFrustum;
	FrustumClip m_frustumClip;

	Voxel *pVoxel;

	vector<cv::Mat> color_images;
	vector<cv::Mat> depth_images;
	vector<cv::Mat> binary_masks;
	vector<pair<int, int>> frame_info;
	vector<UserData> user_data;
	absl::flat_hash_map<uint32_t, UserData> m_mapUserData;

	// For pointcloud transmission only
	vector<vector<int>> goodIndices;

	// Draco enocder
	draco::Encoder mDracoEncoder;
};