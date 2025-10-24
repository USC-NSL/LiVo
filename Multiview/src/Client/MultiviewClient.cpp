#include "Client/MultiviewClient.h"
#include "utils.h"
#include "consts.h"
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

MultiviewClient::MultiviewClient(const string &path, const int datasetType)
{
	m_pPointCloud = NULL;

	m_datasetType = datasetType;

	m_nDevices = 0;
	m_nFrames = 0;
	m_currFrameId = -1;

	m_bInitialized = false;
	m_bCalibrate = true;
	m_bCalibrated = false;

	m_WorkDir = path;

	m_OutDir = FORMAT(m_WorkDir << "/client/");

	if(!fs::exists(m_OutDir))
		fs::create_directories(m_OutDir);
}

/**
 * RAJRUP: Destroy all objects, escpecially the capturer
 * To Do: Check other objects and release them
 **/
MultiviewClient::~MultiviewClient()
{
}

bool MultiviewClient::init(const vector<uint32_t> &deviceIndices, const vector<std::string> &deviceSerialNumbers, const vector<Calibration *> &calibrations, bool viewPtcl)
{
	load_device_details(deviceIndices, deviceSerialNumbers);
	load_calibration(calibrations);

	// m_pPointCloud = new MultiviewPointCloud(m_vDeviceIndices, m_vDeviceSerialNumbers);
	m_pPointCloud = new MultiviewPointCloudClient(m_vDeviceIndices, m_vDeviceSerialNumbers, m_datasetType);
	if(viewPtcl)
		m_pPointCloud->initViewer();

	m_vStageTimes.resize(CLIENT_STAGES::NUM_STAGES, 0);

	m_bInitialized = true;
	return true;
}

bool MultiviewClient::restart_playback()
{
	if(!m_bInitialized)
	{
		LOG(ERROR) << "MultiviewClient not initialized!";
		return false;
	}

	if(!m_bCalibrated)
	{
		LOG(ERROR) << "MultiviewClient not calibrated!";
		return false;
	}

	for(int i = CLIENT_STAGES::UPDATE_PTCL; i < CLIENT_STAGES::NUM_STAGES; i++)
		m_vStageTimes[i] = 0;
	m_timer.Restart();

	return true;
}

bool MultiviewClient::get_calibration_requirement()
{
	return m_bCalibrate;
}

bool MultiviewClient::get_calibration_status()
{
	return m_bCalibrated;
}

// Only playback can load calibration
bool MultiviewClient::load_calibration(const vector<Calibration *> &calibrations)
{
	if(m_nDevices <= 0)
	{
		LOG(ERROR) << "Playback devices not initialized!";
		return false;
	}

	if(calibrations.size() != m_nDevices)
	{
		LOG(ERROR) << "Calibrations size does not match number of devices!";
		return false;
	}

	if(m_vpCalibration.empty())
		m_vpCalibration.resize(m_nDevices);
	
	for(auto i : m_vDeviceIndices)
	{
		if(calibrations[i]->bCalibrated)
			m_vpCalibration[i] = calibrations[i];
		else
		{
			LOG(ERROR) << "Device " << i << " is not calibrated";
			return false;
		}
	}

	m_bCalibrate = false;
	m_bCalibrated = true;

	return true;
}

bool MultiviewClient::load_device_details(const vector<uint32_t> &deviceIndices, const vector<string> &deviceSerialNumbers)
{
	if(!m_vDeviceIndices.empty())
		m_vDeviceIndices.clear();
	if(!m_vDeviceSerialNumbers.empty())
		m_vDeviceSerialNumbers.clear();

	m_vDeviceIndices = deviceIndices;
	m_vDeviceSerialNumbers = deviceSerialNumbers;
	m_nDevices = m_vDeviceIndices.size();

	return true;
}

void MultiviewClient::set_curr_frameId(int frameID)
{
	m_currFrameId = frameID;
}

int MultiviewClient::get_curr_frameId()
{
	return m_currFrameId;
}

void MultiviewClient::set_xy_table(const vector<xy_table_t *> &xy_table)
{
	m_pPointCloud->setXYTables(xy_table);
}

// Load views from the server
bool MultiviewClient::set_view(const vector<View *> &views)
{
	if(!m_bInitialized)
	{
		LOG(ERROR) << "MultiviewClient not initialized!";
		return false;
	}

	if(!m_bCalibrated)
	{
		LOG(ERROR) << "MultiviewClient not calibrated!";
		return false;
	}

	if(views.size() != m_nDevices)
	{
		LOG(ERROR) << "Warning: Number of views do not match number of devices!";
		return false;
	}
	m_pPointCloud->setViews(views);
	return true;
}

/**
 * @brief Returns the current selected views. Returning vector of pointers for efficiency. 
 * Call this function after views are updated using the update_view() function.
 * @return vector<View *> Remember to clear the views vector after use. 
 * Don't delete the views (pointing obejects inside the vector) themselves.
 */
vector<View *> MultiviewClient::get_view()
{
	vector<View *> views;
	for(auto &view : m_pPointCloud->m_views)
	{
		views.push_back(&view);
	}
	return views;
}

// Load binary mask from disk
void MultiviewClient::load_binary_mask(int frameId)
{
	vector<cv::Mat> binary_masks(m_vDeviceIndices.size());
	for(auto i : m_vDeviceIndices)
	{
		string file_path = FORMAT(m_WorkDir << "binary_mask/" << frameId << "_binary_mask_" << i << ".png");
		binary_masks[i] = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
	}
	m_pPointCloud->setBinaryMask(binary_masks);
	// LOG(INFO) << "Loaded binary mask for frame " << frameId << endl;
}

// Generates point cloud
bool MultiviewClient::update_ptcl()
{
	// Load images from selected views
	m_pPointCloud->setImagesFromViews();

	/*********************** View Selection ***********************/
	// Clip space Culling on selected views

	// m_pPointCloud->updatePointCloudClipCullingFromViews(m_vpCalibration);

	if(FLAGS_client_cull == CULLING::CLIP_CULLING || 
		FLAGS_client_cull == CULLING::CLIP_VOXEL_CULLING)
	{
		m_pPointCloud->updatePointCloudClipCullingFromViewsWithoutPcl(m_vpCalibration);
	}
	else if(FLAGS_client_cull == CULLING::NORMAL_CULLING || 
			FLAGS_client_cull == CULLING::NORMAL_VOXEL_CULLING || 
			FLAGS_client_cull == CULLING::NORMAL_EXPANSION_CULLING || 
			FLAGS_client_cull == CULLING::NO_CULLING)
	{
		// LOG(INFO) << "Culling method: " << culling2str[FLAGS_client_cull];
		m_pPointCloud->updatePointCloudNormalCullingFromViewsWithoutPcl(m_vpCalibration);
	}
	else
	{
		LOG(ERROR) << "Invalid culling method!";
		return false;
	}

	m_vStageTimes[CLIENT_STAGES::UPDATE_PTCL] = m_timer.ElapsedMs();

	m_nFrames++;
	return true;
}

uint32_t MultiviewClient::get_num_points_in_frustum()
{
	return m_pPointCloud->nValidPoints;
}

void MultiviewClient::generate_pcl_object()
{
	pcl_object = m_pPointCloud->generatePCLObject();

	m_vStageTimes[CLIENT_STAGES::GENERATE_PCL] = m_timer.ElapsedMs() - m_vStageTimes[CLIENT_STAGES::UPDATE_PTCL];
}

void MultiviewClient::generate_pcl_object(PointCloud_t::Ptr &pcl_object)
{
	m_pPointCloud->generatePCLObject(pcl_object);

	m_vStageTimes[CLIENT_STAGES::GENERATE_PCL] = m_timer.ElapsedMs() - m_vStageTimes[CLIENT_STAGES::UPDATE_PTCL];
}

PointCloud_t::Ptr MultiviewClient::get_pcl_object()
{
	return pcl_object;
}

void MultiviewClient::get_open3d_points_colors(vector<Eigen::Vector3d> &points, vector<Eigen::Vector3d> &colors)
{
	m_pPointCloud->getOpen3DVerticesColor(points, colors);
}

/**
 * @brief Get the streamable ptcl object in a buffer
 * Refer to MultiviewPointCloud::generateStreamablePtcl() for details on the buffer format
 * @param ptclBuffer Buffer will be allocated in this function 
 * @param size Size of the buffer
 */
void MultiviewClient::get_streamable_ptcl(uint8_t *&ptclBuffer, int32_t &size)
{
	uint32_t nPoints = 0;
	m_pPointCloud->generateStreamablePtcl(ptclBuffer, size, nPoints);
}

void MultiviewClient::set_frustum(const Frustum &frustum)
{
	m_pPointCloud->setFrustum(frustum);
}

void MultiviewClient::update_frustum(const CamInt &camInt, const CamView &camView)
{
	m_pPointCloud->updateFrustum(camInt, camView);
}

void MultiviewClient::update_frustum_clip(const Matrix4d &camProjViewMat)
{
	m_pPointCloud->updateFrustumClip(camProjViewMat);
}

// Return a copy of frustum
Frustum MultiviewClient::get_frustum()
{
	return *(m_pPointCloud->getFrustum());
}

// Return a copy of frustum
FrustumClip MultiviewClient::get_frustumClip()
{
	return *(m_pPointCloud->getFrustumClip());
}

uint64_t MultiviewClient::get_update_ptcl_time()
{
	return m_vStageTimes[CLIENT_STAGES::UPDATE_PTCL];
}

uint64_t MultiviewClient::get_elapsed_pcl_time()
{
	return m_vStageTimes[CLIENT_STAGES::GENERATE_PCL];
}

uint64_t MultiviewClient::get_elapsed_time()
{
	return m_timer.ElapsedMs();
}

void MultiviewClient::view_ptcl()
{
	m_pPointCloud->viewPointCloudFrustumClip();
}

// void MultiviewClient::save_ptcl()
// {
// 	string file_path = FORMAT(m_OutDir << m_currFrameId << ".ply");
// 	m_pPointCloud->savePointCloud(file_path);
// }

void MultiviewClient::save_ptcl_from_pcl(PointCloud_t::Ptr pcl_object)
{
	string file_path = FORMAT(m_OutDir << m_currFrameId << ".ply");
	saveToPly(file_path, pcl_object);
}

// void MultiviewClient::save_ptcl_binary()
// {
// 	string file_path = FORMAT(m_OutDir << m_currFrameId << ".ply");
// 	m_pPointCloud->savePointCloudBinary(file_path);
// }

void MultiviewClient::save_ptcl_binary_from_pcl(PointCloud_t::Ptr pcl_object)
{
	string file_path = FORMAT(m_OutDir << m_currFrameId << ".ply");
	saveToPlyBinary(file_path, pcl_object);
}

void MultiviewClient::set_work_dir(const string &path)
{
	m_WorkDir = path;
	m_OutDir = FORMAT(m_WorkDir << "/client/");

	if(!fs::exists(m_OutDir))
		fs::create_directories(m_OutDir);
}

// Generates point cloud
bool MultiviewClient::update_ptcl_3D_decompress()
{
	time_point start, end;
	uint64_t elapsed_ms;
	start = time_now();

	// Point cloud generation from buffer
	// To Do: Rajrup: Use a flag to check that decompression is done and buffer is ready
	if(FLAGS_client_cull == CULLING::CLIP_CULLING || FLAGS_client_cull == CULLING::NORMAL_VOXEL_CULLING)
		m_pPointCloud->updatePointCloudFromBufferClipCulling();
	else
	{
		m_pPointCloud->updatePointCloudFromBuffer(); 		// PCL viewer version
	}
	
	end = time_now();
	elapsed_ms = duration_ms(start, end);
	cout << "Updated point cloud in " << elapsed_ms << " ms" << endl;

	m_nFrames++;
	return true;
}

void MultiviewClient::update_ptcl_3D_decompress_open3D(const string &compressionType, vector<Eigen::Vector3d> &points, vector<Eigen::Vector3d> &colors)
{
	time_point start, end;
	uint64_t elapsed_ms;
	start = time_now();

	// Point cloud generation from buffer
	// To Do: Rajrup: Use a flag to check that decompression is done and buffer is ready

	if(compressionType == "zstd")
		m_pPointCloud->updatePointCloudFromZstdBufferO3D(points, colors); 		// Open3D viewer version
	else if(compressionType == "draco")
		m_pPointCloud->updatePointCloudFromDracoBufferO3D(points, colors); 		// Open3D viewer version
	else
		LOG(FATAL) << "Invalid compression type!";
	
	end = time_now();
	elapsed_ms = duration_ms(start, end);
	cout << "Updated point cloud in " << elapsed_ms << " ms" << endl;

	m_nFrames++;
}

bool MultiviewClient::decompress_ptcl(const uint8_t *buf, int size, const string &compressionType, const zstd_options &zstd_params, const draco_options &draco_params)
{
	StopWatch sw;

	if(compressionType == "zstd")
		m_pPointCloud->decompressPointCloudZstd(buf, size, zstd_params.compression_level);
	else if(compressionType == "draco")
		m_pPointCloud->decompressPointCloudDraco(buf, size, draco_params.cl, draco_params.qp);
	else
	{
		LOG(ERROR) << "Invalid compression type!";
		return false;
	}

	// LOG(INFO) << "Decompressed ptcl in " << sw.ElapsedMs() << " ms";
	return true;
}