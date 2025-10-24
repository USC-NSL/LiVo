# pragma once
#include <vector>
#include <draco/compression/decode.h>
#include <draco/core/cycle_timer.h>
#include <draco/io/file_utils.h>
#include <draco/io/obj_encoder.h>
#include <draco/io/parser_utils.h>
#include <draco/io/ply_encoder.h>
#include <draco/io/stl_encoder.h>

#include "PointCloudViewer.h"
#include "calibration.h"
#include "consts.h"
#include "view.h"
#include "frustum.h"
#include "octree.h"
#include "utils.h"

using namespace std;

class MultiviewPointCloudClient
{
public:

	MultiviewPointCloudClient(const vector<uint32_t> &indices, const vector<string> &deviceSerials, const int datasetType=DATASET::KINECT);
	~MultiviewPointCloudClient();

	void initViewer();
	void viewPointCloud();
	void viewPointCloudFrustum();
	void viewPointCloudFrustumClip();

	void setXYTables(const vector<xy_table_t *> &xyTables);

	void setViews(const vector<View *> &views);
	void setImagesFromViews();
	void setBinaryMask(const vector<cv::Mat> &masks);

	void updatePointCloudClipCullingFromViews(const vector<Calibration *> &extCalibration);
	void updatePointCloudClipCullingFromViewsWithoutPcl(const vector<Calibration *> &extCalibration);
	void updatePointCloudNormalCullingFromViewsWithoutPcl(const vector<Calibration *> &extCalibration);

	// Implemented in MultiviewPointCloud.cpp. Integrate as required.
	void updatePointCloud(const vector<Calibration *> &extCalibration);
	void updatePointCloudNoCulling(const vector<Calibration *> &extCalibration);
	void updatePointCloudPlaneCulling(const vector<Calibration *> &extCalibration);
	void updatePointCloudClipCulling(const vector<Calibration *> &extCalibration);
	void updatePointCloudOctreeCulling(const vector<Calibration *> &extCalibration);
	
	PointCloud_t::Ptr generatePCLObject();
	void generatePCLObject(PointCloud_t::Ptr &cloud);
	void generateStreamablePtcl(uint8_t *&ptclBuffer, int32_t &bufferSize, uint32_t &nPoints);

	void getOpen3DVerticesColor(vector<Vector3d> &points, vector<Vector3d> &colors);
	
	void setFrustum(const Frustum &frustum);
	void updateFrustum(const CamInt &camInt, const CamView &camView);
	void updateFrustumClip(const Matrix4d &camProjViewMat);

	Frustum *getFrustum();
	FrustumClip *getFrustumClip();

	void savePointCloud(const string &filename);
	void savePointCloudBinary(const string &filename);

	vector<View> m_views;
	uint32_t m_viewSizeInBytes;
	uint32_t nValidPoints;

	PointCloud_t::Ptr pclCloud;

	// For pointcloud transmission only
	void decompressPointCloudZstd(const uint8_t *buf, int size, int compressionLevel);
	void decompressPointCloudDraco(const uint8_t *buf, int size, int cl, int qp);
	void updatePointCloudFromBuffer();
	void updatePointCloudFromBufferClipCulling();
	void updatePointCloudFromZstdBufferO3D(vector<Vector3d> &points, vector<Vector3d> &colors);
	void updatePointCloudFromDracoBufferO3D(vector<Vector3d> &points, vector<Vector3d> &colors);

private:

	int m_datasetType;

	vector<uint32_t> device_indices;
	vector<string> device_serial_numbers;
	vector<xy_table_t *> xy_tables;

	vector<int> nColorFrameHeight, nColorFrameWidth;
	vector<int> nDepthFrameHeight, nDepthFrameWidth;

	vector<Point3f *> pPointCloud;
	vector<uint16_t *> pDepth;
    vector<RGB *> pColorRGBX; // To Do: RAJRUP: change this to simple arrays instead of vector?
	vector<bool *> pBinaryMask;

	Frustum m_frustum;
	FrustumClip m_frustumClip;

	// float minX, maxX, minY, maxY, minZ, maxZ;
	Vector3f minBound, maxBound;

	Octree *p_octree;

	string windowName;
	PointCloudViewer *pViewer;

	// For Zstd decompression
	uint8_t *pPointCloudBuf;
	int pointCloudBufSize;

	// For Draco decompression
	draco::Decoder mDracoDecoder;
	std::unique_ptr<draco::PointCloud> pDracoPtcl;
};
