#pragma once
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <gflags/gflags.h>
#include "absl/base/log_severity.h"
#include "absl/log/log.h"
#include "absl/log/globals.h"
#include "absl/log/check.h"

DECLARE_bool(capture);
DECLARE_string(seq_name);
DECLARE_int32(view_ptcl);
DECLARE_bool(save_ptcl);
DECLARE_bool(ground);
DECLARE_bool(save_views);
DECLARE_string(save_views_subfolder);
DECLARE_bool(save_binary_mask);
DECLARE_bool(load_binary_mask);
DECLARE_int32(start_frame_id);
DECLARE_int32(end_frame_id);
DECLARE_int32(ncaptures);
DECLARE_int32(server_cull);
DECLARE_int32(client_cull);
DECLARE_bool(save_frustum);
DECLARE_int32(load_frustum);
DECLARE_bool(update_frustum);
DECLARE_string(config_file);
DECLARE_string(config_folder);
DECLARE_int32(send_ptcl);
DECLARE_bool(save_frame);
DECLARE_string(output_dir);
DECLARE_string(render_image_path);
DECLARE_string(save_ptcl_path);
DECLARE_string(save_cbitrate_file);
DECLARE_string(save_dbitrate_file);
DECLARE_int32(server_fps);
DECLARE_int32(client_fps);
DECLARE_int32(cb);
DECLARE_int32(db);
DECLARE_int32(cqp);
DECLARE_int32(dqp);
DECLARE_bool(use_mm);						// Use server side rate limiting using mahimahi
DECLARE_string(mm_trace_name);				// Mahimahi trace file name
DECLARE_bool(use_server_bitrate_est);		// Use server side bitrate estimation when mahimahi is used [default]
DECLARE_bool(use_client_bitrate);			// Use client side bitrate estimation when mahimahi is used
DECLARE_bool(use_client_bitrate_split);		// Use bitrate split sent by client
DECLARE_bool(use_split_adapt);				// Use depth to color bitrate split adaptation
DECLARE_double(d2c_split);					// Depth to color bitrate split ratio
DECLARE_string(compression_type);
DECLARE_int32(zstd_cl);
DECLARE_int32(draco_cl);
DECLARE_int32(draco_qp);

#define CALIBRATION_EXT_FILENAME "extrinsics"
#define CALIBRATION_INT_FILENAME "intrinsics"
#define DEVICE_FILE_NAME "devices"

#define ANG2RAD(x) x * (M_PI/180.0f)
#define INTER_FRAME_DELAY 33.0f				// 30 fps
#define PIPELINE_DATA_MAX_WAIT 20000				// 20 sec

// General global constants. These are read from config file.
extern std::string PTCL_STREAM_ADDRESS;
extern int PTCL_STREAM_PORT;
extern uint32_t START_FRAME;
extern uint32_t END_FRAME;
extern std::string QRCODE_FOLDER;

namespace DATASET
{
	enum
	{
		PANOPTIC = 0,
		KINECT = 1
	};

	enum
	{
		KINECT_NCAM = 4,
		PANOPTIC_NCAM = 10
	};
}

static const char * dataset2str[] = { "PANOPTIC", "KINECT" };

namespace VIEWER
{
	enum
	{
		NO_VIEWER = 0,
		PCL_VIEWER = 1,
		UNITY_VIEWER = 2,
		OPEN3D_VIEWER = 3
	};
}

static const char * viewer2str[] = { "NO_VIEWER", "PCL_VIEWER", "UNITY_VIEWER", "OPEN3D_VIEWER" };

namespace CULLING
{
	enum
	{
		NO_CULLING = 0,
		CLIP_CULLING = 1,
		CLIP_VOXEL_CULLING = 2,
		NORMAL_CULLING = 3,
		NORMAL_VOXEL_CULLING = 4,
		NORMAL_EXPANSION_CULLING = 5
	};
}

static const char * culling2str[] = { "nocull", "clip_cull", "clip_voxel_cull", "cull", "voxel_cull", "kpcull" };

namespace LOAD_FRUSTUM
{
	enum
	{
		NONE = 0,
		PANOPTIC = 1,
		KINECT = 2,
		DISK = 3
	};
}

static const char * load_frustum2str[] = { "NONE", "PANOPTIC", "KINECT", "DISK" };

namespace PIXEL_SIZE
{
	enum
	{
		BGRA = 1,	// Every pixel is 1 byte
		DEPTH = 2	// Depth in 16 bit format
	};
}

static const char * pixel_size2str[] = { "BGRA", "DEPTH" };

namespace CHANNEL_DIM
{
	enum
	{
		BGRA = 4,	// BGRA
		DEPTH = 3	// YUV
	};
}

static const char * channel_dim2str[] = { "DEPTH_BGRA", "DEPTH_YUV16" };

#define FORMAT(items) \
    static_cast<std::ostringstream &>((std::ostringstream() << std::string() << items)).str()

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud_t;
typedef pcl::PointXYZRGB Point_t;


static const char *SERVER_HOST = "68.181.32.205"; 	// Unused in the cpp version, Use SERVER_HOST_NEW instead
static const int WEBRTC_CSERVER_PORT = 5252;
static const int WEBRTC_DSERVER_PORT = WEBRTC_CSERVER_PORT + 1;

// static const char* SERVER_HOST = "100.64.0.2";	// Mahimahi IP address

// static const char* SERVER_HOST = "127.0.0.1";

static const char *WEBRTC_SERVER_HOST = SERVER_HOST;

extern std::string SERVER_HOST_NEW;
extern std::string WEBRTC_SERVER_HOST_NEW;