#pragma once
#include <string>
#include "Client/MultiviewPointCloudClient.h"
#include "calibration.h"
#include "frustum.h"
#include "view.h"
#include "consts.h"
#include "timer.h"

using namespace std;

namespace CLIENT_STAGES
{
	enum
	{
		UPDATE_PTCL = 0,
		GENERATE_PCL,
		NUM_STAGES
	};
}

class MultiviewClient
{

public:

	MultiviewClient(const string &path="./", const int datasetType=DATASET::KINECT);
	~MultiviewClient();
	bool init(const vector<uint32_t> &deviceIndices, const vector<std::string> &deviceSerialNumbers, const vector<Calibration *> &calibrations, bool viewPtcl = false);

	bool restart_playback();
	bool get_calibration_requirement();
	bool get_calibration_status();
	bool load_calibration(const vector<Calibration *> &calibration);
	bool load_device_details(const vector<uint32_t> &deviceIndices,const vector<string> &deviceSerialNumbers);
	void set_xy_table(const vector<xy_table_t *> &xy_table);
	// bool set_view(const vector<View *> &views);
	bool set_view(const vector<View *> &views);
	vector<View *> get_view();
	
	bool update_ptcl();
	void load_binary_mask(int frameId);
	void generate_pcl_object();
	void generate_pcl_object(PointCloud_t::Ptr &pcl_object);
	PointCloud_t::Ptr get_pcl_object();
	void get_streamable_ptcl(uint8_t *&ptclBuffer, int32_t &size);
	void get_open3d_points_colors(vector<Eigen::Vector3d> &points, vector<Eigen::Vector3d> &colors);
	uint32_t get_num_points_in_frustum();

	void set_frustum(const Frustum &frustum);
	void update_frustum(const CamInt &camInt, const CamView &camView);
	void update_frustum_clip(const Matrix4d &camProjViewMat);
	void view_ptcl();
	// void save_ptcl();
	// void save_ptcl_binary();
	void save_ptcl_from_pcl(PointCloud_t::Ptr pcl_object);
	void save_ptcl_binary_from_pcl(PointCloud_t::Ptr pcl_object);
	void set_curr_frameId(int frameID);
	int get_curr_frameId();
	Frustum get_frustum();
	FrustumClip get_frustumClip();

	uint64_t get_update_ptcl_time();
	uint64_t get_elapsed_time();
	uint64_t get_elapsed_pcl_time();

	void set_work_dir(const string &path="./");

	// For pointcloud transmission only
	bool update_ptcl_3D_decompress();
	void update_ptcl_3D_decompress_open3D(const string &compressionType, vector<Eigen::Vector3d> &points, vector<Eigen::Vector3d> &colors);
	bool decompress_ptcl(const uint8_t *buf, int size, const string &compressionType, const zstd_options &zstd_params, const draco_options &draco_params);

private:

	int m_datasetType;

	// MultiviewPointCloud *m_pPointCloud;
	MultiviewPointCloudClient *m_pPointCloud;
	vector<Calibration *> m_vpCalibration;
	
	vector<uint32_t> m_vDeviceIndices;
	vector<string> m_vDeviceSerialNumbers;

	string m_WorkDir;
	string m_OutDir;

	int m_nDevices;
	uint32_t m_nFrames;
	int m_currFrameId;

	bool m_bInitialized;
	bool m_bCalibrate; // true if calibration is required, false means calibration is done
	bool m_bCalibrated; // true if calibration is successful, false means calibration has failed

	PointCloud_t::Ptr pcl_object;

	StopWatch m_timer;
	vector<uint64_t> m_vStageTimes;
};