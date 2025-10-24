#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <k4a/k4a.hpp>

#include "MultiDeviceCapturer.h"
#include "Server/MultiviewPointCloudServer.h"
#include "calibration.h"
#include "frustum.h"
#include "view.h"
#include "consts.h"
#include "timer.h"

using namespace std;

namespace SERVER_STAGES
{
	enum
	{
		LOAD_FRAME = 0,
		UPDATE_VIEW,
		SEND_FRAME,
		NUM_STAGES
	};
}

class MultiviewServer
{
public:
	MultiviewServer(const string &path="./", const int datasetType=DATASET::KINECT);
	MultiviewServer(int maxCalibrationIterations=30, const string &path="./", const int datasetType=DATASET::KINECT);
	~MultiviewServer();
	bool init_capture();
	bool init_playback();
	bool save_device_details();
	
	bool restart_playback();
	vector<uint32_t> get_device_indices();
	vector<string> get_device_serial_numbers();
	int get_device_count(); 
	void icp_refinement();
	bool calibrate();
	bool get_calibration_requirement();
	bool get_calibration_status();
	bool save_calibration();
	bool load_calibration();
	vector<Calibration *> get_calibration();
	void set_curr_frameId(int frameID);
	int get_curr_frameId();
	vector<xy_table_t *> get_xy_table();
	bool capture_frame();
	bool store_frame();
	bool load_frame(bool preload=false);
	bool load_frame_parallel(bool preload=false);
	bool load_frame_with_colorized_depth_parallel(bool preload=false, string format="png");
	void update_view();
	uint32_t get_view_size_in_frustum();
	uint32_t get_view_size_without_frustum();

	void set_user_data(const UserData &userData);
	void set_frustum(const Frustum &frustum);
	void set_frustum(const Frustum &prev_frustum, const Frustum &pred_frustum);
	void set_frustum_clip(const FrustumClip &frustumClip);
	vector<View *> get_view();
	// vector<cv::Mat> get_binary_mask();
	void set_work_dir(const string &path="./");

	bool preload();
	uint64_t get_load_frame_time();
	uint64_t get_update_view_time();
	uint64_t get_send_frame_time();
	uint64_t get_elapsed_time();

	// For pointcloud transmission only
	void update_ptcl();
	const uint8_t *compress_ptcl(int &size, string compressionType, const zstd_options &zstd_params, const draco_options &draco_params);
	const uint8_t *get_compressed_ptcl(int &size);
	int get_compressed_ptcl_size();
	int get_num_points_ptcl();

private:

	bool load_device_details();

	int m_datasetType;

	MultiDeviceCapturer *m_pCapturer;
	MultiviewPointCloudServer *m_pPointCloud;
	vector<Calibration *> m_vpCalibration;

	uint8_t *m_pCompressedPtcl;
	int m_compressedPtclSize;

	vector<uint32_t> m_vDeviceIndices;
	vector<string> m_vDeviceSerialNumbers;
	vector<k4a::image> m_vColorImages;
	vector<k4a::image> m_vDepthImages;

	vector<cv::Mat> m_vBinaryMasks;

	vector<k4a::image> m_vColorImagesQueue;
	vector<k4a::image> m_vDepthImagesQueue;

	int m_nDevices;
	int m_maxCalibrationIterations;
	int m_currFrameId;

	string m_WorkDir;

	bool m_bCamerasInitialized;		// Used to check if the cameras are initialized during Capture
	bool m_bPlaybackInitialized;	// Used to check if the playback is initialized during Playback
	bool m_bCalibrate; 				// true if calibration is required, false means calibration is done
	bool m_bCalibrated; 			// true if calibration is successful, false means calibration has failed

	vector<vector<pair<uint32_t, cv::Mat>>> color_preload;
	vector<vector<pair<uint32_t, cv::Mat>>> depth_preload;

	vector<pair<uint32_t, cv::Mat>> color_images_cv2;
	vector<pair<uint32_t, cv::Mat>> depth_images_cv2;
	
	StopWatch m_timer;
	vector<uint64_t> m_vStageTimes;
};