#include "Server/MultiviewServer.h"
#include "colorized_depth.h"
#include <boost/filesystem.hpp>
#include <omp.h>
#include <algorithm>

#include "utils.h"
#include "k4aconsts.h"
#include "k4autils.h"
#include "consts.h"

namespace fs = boost::filesystem;

MultiviewServer::MultiviewServer(const string &path, const int datasetType) : MultiviewServer(0, path, datasetType) {}

MultiviewServer::MultiviewServer(int maxCalibrationIterations, const string &path, const int datasetType)
{
	m_pCapturer = NULL;
	m_pPointCloud = NULL;
	m_pCompressedPtcl = NULL;
	m_compressedPtclSize = 0;
	
	m_datasetType = datasetType;

	m_nDevices = 0;
	m_maxCalibrationIterations = maxCalibrationIterations;
	m_currFrameId = -1;

	m_WorkDir = path;

	m_bCamerasInitialized = false;
	m_bPlaybackInitialized = false;

	m_bCalibrate = true;
	m_bCalibrated = false;	
}

bool MultiviewServer::init_capture()
{
	// Camera Parameters
	int exposureStep = 1;
	//Formula copied from here: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/7cd8683a1a71b8baebef4a3537e6edd8639d1e95/examples/k4arecorder/main.cpp#L333
	float absExposure = (exp2f((float)exposureStep) * 1000000.0f);
	//We add 0.5 because C++ always truncates when converting to an integer. 
	//This ensures that values will always be rounded correctly
	float absExposureRoundingMargin = absExposure + 0.5;
	int color_exposure_usec = (int)absExposureRoundingMargin;

	// color_exposure_usec = 8000;
	int powerline_freq = 1;

	m_pCapturer = new MultiDeviceCapturer(color_exposure_usec, powerline_freq);
	m_pCapturer->print_devices();

	m_nDevices = m_pCapturer->device_count;
	
	m_vDeviceIndices = m_pCapturer->get_device_indices();
	m_vDeviceSerialNumbers = m_pCapturer->get_device_serial_numbers();

	if(!save_device_details())
	{
		LOG(ERROR) << "Failed to save device details";
		return false;
	}

	if(m_vColorImages.empty())
		m_vColorImages.resize(m_nDevices);
	if(m_vDepthImages.empty())
		m_vDepthImages.resize(m_nDevices);

	/**
	 * RAJRUP: Start with a blank capture Acquire Frame, this is required for intialization of each camera
	 * To Do: Should go in the constructor of MultiDeviceCapturer?
	 **/
	
	LOG(INFO) << "Start with some captures";
	m_pCapturer->start_devices();

	int dummyCapture = DUMMY_CAPTURES;
	while(dummyCapture-- > 0)
	{
		if(!m_pCapturer->get_synchronized_captures())
		{
			LOG(ERROR) << "Failed to get synchronized captures";
			m_bCamerasInitialized = false;
			return false;
		}        
	}

	m_bCamerasInitialized = true;
	m_bPlaybackInitialized = false;
	return true;
}

// To Do: RAJRUP: Add call to load_calibration() in init_playback() function, since playback cannot work with calibration 
bool MultiviewServer::init_playback()
{
	m_bCalibrate = true;
	m_bCalibrated = false;

	if(m_bPlaybackInitialized)
	{
		LOG(WARNING) << "Playback already initialized";
		return true;
	}

	if(m_pCapturer)
	{
		m_pCapturer->stop_devices();
		delete m_pCapturer;
		m_pCapturer = NULL;
	}

	if(!load_device_details()) // Initializes device indices, device serial numbers and number of devices
	{
		LOG(ERROR) << "Failed to load device details";
		return false;
	}

	assert(m_nDevices > 0);

	if(m_vColorImages.empty())
		m_vColorImages.resize(m_nDevices);
	if(m_vDepthImages.empty())
		m_vDepthImages.resize(m_nDevices);
	color_images_cv2.resize(m_nDevices);
	depth_images_cv2.resize(m_nDevices);

	if(m_vBinaryMasks.empty())
		m_vBinaryMasks.resize(m_nDevices);
	
	// m_pPointCloud = new MultiviewPointCloud(m_vDeviceIndices, m_vDeviceSerialNumbers);
	m_pPointCloud = new MultiviewPointCloudServer(m_vDeviceIndices, m_vDeviceSerialNumbers, m_datasetType);

	m_pPointCloud->loadTransformations(m_WorkDir);

	m_vStageTimes.resize(SERVER_STAGES::NUM_STAGES, 0);

	m_bCamerasInitialized = false;
	m_bPlaybackInitialized = true;

	return true;
}

/**
 * RAJRUP: Destroy all objects, escpecially the capturer
 * To Do: Check other objects and release them
 **/
MultiviewServer::~MultiviewServer()
{
	if(m_pCapturer)
	{
		LOG(INFO) << "Stopping all cameras";
		m_pCapturer->stop_devices();
		delete m_pCapturer;
		m_pCapturer = NULL;
	}

	if(m_bPlaybackInitialized)
	{
		LOG(INFO) << "Stopping playback";
		
		m_vDeviceIndices.clear();
		m_vDeviceSerialNumbers.clear();

		for(int i = 0; i < m_nDevices; i++)
		{
			m_vColorImages[i].reset();
			m_vDepthImages[i].reset();
		}
		m_vColorImages.clear();
		m_vDepthImages.clear();

		if(m_pPointCloud)
		{
			delete m_pPointCloud;
			m_pPointCloud = NULL;
		}
	}

	for(auto elem : m_vpCalibration)
		delete elem;
	m_vpCalibration.clear();

	m_vStageTimes.clear();
}

bool MultiviewServer::calibrate()
{
	if(!m_bCamerasInitialized)
	{
		LOG(ERROR) << "Cameras not initialized";
		return false;
	}

	if(m_vpCalibration.empty())
		m_vpCalibration.resize(m_nDevices);
	
	for (uint32_t i : m_vDeviceIndices)
		m_vpCalibration[i] = new Calibration(m_maxCalibrationIterations);

	for (int iter = 0; iter <= m_maxCalibrationIterations && m_bCalibrate; iter++)
	{

		// Acquire Frame
		bool capture_flag = m_pCapturer->get_synchronized_captures();

#if (LOG)
		if (capture_flag)
			LOG(INFO) << "Synchronized Capture Successful!";
		else
			LOG(INFO) << "Synchronized Capture Failed!";
#endif

		vector<Point3f *> pCameraCoordinates(m_nDevices, NULL);
		for (uint32_t i : m_vDeviceIndices)
			pCameraCoordinates[i] = new Point3f[m_pCapturer->nColorFrameWidth[i] * m_pCapturer->nColorFrameHeight[i]];

		m_pCapturer->MapColorFrameToCameraSpace(pCameraCoordinates);

		bool res = true;
		for (uint32_t i : m_vDeviceIndices)
			res &= m_vpCalibration[i]->calibrate(m_pCapturer->pColorRGBX[i], pCameraCoordinates[i], m_pCapturer->nColorFrameWidth[i], m_pCapturer->nColorFrameHeight[i]);
		
		/**
		 * RAJRUP
		 * To Do: Create template function to delete any vector
		 **/ 
		for (auto p : pCameraCoordinates) 
			delete p;

		pCameraCoordinates.clear();
		if (res)
			m_bCalibrate = false;

#if (LOG)
		if (res)
			LOG(INFO) << "Iter " << iter << ", Calibration Successful!";
#endif
	}

	m_bCalibrated = true;
	for (uint32_t i : m_vDeviceIndices)
	{
		m_bCalibrated &= m_vpCalibration[i]->bCalibrated;

#if (LOG)
		LOG(INFO) << "Camera Index: " << i << ", Serial number: " << m_vDeviceSerialNumbers[i];

		if (m_vpCalibration[i]->bCalibrated)
			LOG(INFO) << " is Calibrated";
		else 
			LOG(INFO) << " is not Calibrated";
#endif
	}

	return m_bCalibrated;
}

/**
 * @brief Save both intrincs and extrincs
 * 
 * @return true Save is successful
 * @return false Save failed since cameras extrinsics are not calibrated
 */
bool MultiviewServer::save_calibration()
{
	if(m_bCalibrate)
	{
		LOG(ERROR) << "Calibration hasn't been perfomed!";
		return false;
	}
	
	if (!m_bCalibrated)
	{
		LOG(ERROR) << "Cameras are not calibrated!";
		return false;
	}
	
	// Save intrincs
	m_pCapturer->saveCalibration(m_WorkDir);
	
	// Save extrincs
	for (uint32_t i : m_vDeviceIndices)
		m_vpCalibration[i]->saveCalibration(m_vDeviceSerialNumbers[i], m_WorkDir);

	return true;
}

// Only playback can load calibration
bool MultiviewServer::load_calibration()
{
	if(!m_bPlaybackInitialized)
	{
		LOG(ERROR) << "Playback not initialized";
		return false;
	}

	if(m_bCalibrated)
	{
		LOG(WARNING) << "Calibrations already loaded!";
		LOG(INFO) << "Overwriting previous calibration";
	}

	if(m_vpCalibration.empty())
		m_vpCalibration.resize(m_nDevices);
	
	for (uint32_t i : m_vDeviceIndices)
		m_vpCalibration[i] = new Calibration(m_maxCalibrationIterations);

	for (uint32_t i : m_vDeviceIndices)
	{
		if(m_datasetType == DATASET::KINECT && !m_vpCalibration[i]->loadCalibration(m_vDeviceSerialNumbers[i], m_WorkDir))
		{
			LOG(ERROR) << "Failed to load calibration for camera ID " << i << " serial ID: " << m_vDeviceSerialNumbers[i];
			return false;
		}

		if(m_datasetType == DATASET::PANOPTIC && !m_vpCalibration[i]->loadCalibration(m_vDeviceSerialNumbers[i], m_WorkDir, true))
		{
			LOG(ERROR) << "Failed to load calibration for camera ID " << i << " serial ID: " << m_vDeviceSerialNumbers[i];
			return false;
		}
	}
	
	m_bCalibrate = false;
	m_bCalibrated = true;

	return true;
}

vector<Calibration *> MultiviewServer::get_calibration()
{
	return m_vpCalibration;
}

bool MultiviewServer::save_device_details()
{   
	fs::ofstream file;
	fs::path filepath{FORMAT(m_WorkDir << DEVICE_FILE_NAME << ".txt")};
	file.open(filepath, ios::out | ios::trunc);
	if (!file.is_open())
	{	
		LOG(ERROR) << "Failed to open file: " << filepath;
		return false;
	}

	for (uint32_t i : m_vDeviceIndices)
		file << i << " " << m_vDeviceSerialNumbers[i] << endl;

	file.close();
	LOG(INFO) << "Device details saved to: " << filepath;
	return true;
}

bool MultiviewServer::load_device_details()
{
	fs::ifstream file;
	fs::path filepath{FORMAT(m_WorkDir << DEVICE_FILE_NAME << ".txt")};
	file.open(filepath, ios::in);
	if(!file.is_open())
	{
		LOG(ERROR) << "Failed to open file: " << filepath;
		return false;
	}

	if(!m_vDeviceIndices.empty())
		m_vDeviceIndices.clear();
	if(!m_vDeviceSerialNumbers.empty())
		m_vDeviceSerialNumbers.clear();

	m_nDevices = 0;
	uint32_t device_index;
	string device_serial_number;

	while(file >> device_index >> device_serial_number)
	{
		m_vDeviceIndices.push_back(device_index);
		m_vDeviceSerialNumbers.push_back(device_serial_number);
		m_nDevices++;
	}
	
	file.close();
	LOG(INFO) << "Device details loaded from: " << filepath;
	return true;
}

vector<uint32_t> MultiviewServer::get_device_indices()
{
	return m_vDeviceIndices;
}

vector<std::string> MultiviewServer::get_device_serial_numbers()
{
	return m_vDeviceSerialNumbers;
}

int MultiviewServer::get_device_count()
{
	return m_nDevices;
}

/**
 * RAJRUP
 * To Do: Implement ICP. Contact: @Weiwu for this
 **/ 
void MultiviewServer::icp_refinement()
{
	// Pass transformation matrix

	// Revise transformation matrix using ICP
}

bool MultiviewServer::get_calibration_requirement()
{
	return m_bCalibrate;
}

bool MultiviewServer::get_calibration_status()
{
	return m_bCalibrated;
}

void MultiviewServer::set_curr_frameId(int frameID)
{
	m_currFrameId = frameID;
}

int MultiviewServer::get_curr_frameId()
{
	return m_currFrameId;
}

vector<xy_table_t *> MultiviewServer::get_xy_table()
{
	return m_pPointCloud->getXYTables();
}

bool MultiviewServer::restart_playback()
{
	if(!m_bPlaybackInitialized)
	{
		LOG(ERROR) << "Playback not initialized";
		return false;
	}

	if(!m_bCalibrated)
	{
		LOG(ERROR) << "Calibration not performed";
		LOG(INFO) << "Calibrate first!";
		return false;
	}

	for(int i = SERVER_STAGES::LOAD_FRAME; i < SERVER_STAGES::NUM_STAGES; i++)
		m_vStageTimes[i] = 0;
	m_timer.Restart();
	
	return true;
}

// Update latest capture
bool MultiviewServer::capture_frame()
{
	if(!m_bCamerasInitialized)
	{
		LOG(INFO) << "Cameras not initialized";
		return false;
	}

	if (!m_bCalibrated)
	{
		LOG(ERROR) << "Calibration not performed";
		LOG(INFO) << "Calibrate first!";
		return false;
	}

	// Acquire Frame
	if(!m_pCapturer->get_synchronized_captures())
	{
		LOG(WARNING) << "Synchronized Capture Failed!";
		return false;
	}

	for(uint32_t i : m_vDeviceIndices)
	{
		m_vColorImages[i] = m_pCapturer->GetColorImage(i);
		m_vDepthImages[i] = m_pCapturer->GetDepthImage(i);

		m_vColorImagesQueue.push_back(m_vColorImages[i]);
		m_vDepthImagesQueue.push_back(m_vDepthImages[i]);
	}
	
	return true;
}

// Stores latest captured images
bool MultiviewServer::store_frame()
{
	for(uint32_t i : m_vDeviceIndices)
	{
		cv::Mat color_image_cv2 = color_to_opencv(m_vColorImages[i]);
		cv::Mat depth_image_cv2 = depth_to_opencv(m_vDepthImages[i]);

		string color_image_path = FORMAT(m_WorkDir << m_currFrameId << "_color_" << i << ".png");
		string depth_image_path = FORMAT(m_WorkDir << m_currFrameId << "_depth_" << i << ".png");
		
		if(!cv::imwrite(color_image_path, color_image_cv2, compression_params))
		{
			LOG(ERROR) << "Failed to write color image: " << color_image_path;
			return false;
		}
		if(!cv::imwrite(depth_image_path, depth_image_cv2, compression_params))
		{
			LOG(ERROR) << "Failed to write depth image: " << depth_image_path;
			return false;
		}
	}
	return true;
}

// Load images from m_WorkDir
bool MultiviewServer::load_frame(bool preload)
{
	if(!m_bPlaybackInitialized)
	{
		LOG(ERROR) << "Playback not initialized";
		return false;
	}

	if(!m_bCalibrated)
	{
		LOG(ERROR) << "Calibration not performed";
		LOG(INFO) << "Calibrate first!";
		return false;
	}

	if(m_currFrameId < 0)
	{
		LOG(ERROR) << "Invalid Frame Id!";
		return false;
	}

	// preload
	if (preload) 
	{
		color_images_cv2 = color_preload[m_currFrameId];
		depth_images_cv2 = depth_preload[m_currFrameId];
	} 
	// runtime load
	else 
	{
		for(uint32_t i : m_vDeviceIndices)
		{
			string color_image_path = FORMAT(m_WorkDir << m_currFrameId << "_color_" << i << ".png");
			string depth_image_path = FORMAT(m_WorkDir << m_currFrameId << "_depth_" << i << ".png");

			if(!fs::exists(color_image_path))
			{
				LOG(WARNING) << "Color image from " << color_image_path << " doesn't exist, so skipping ...";
				return false;
			}

			if(!fs::exists(depth_image_path))
			{
				LOG(WARNING) << "Depth image from " << depth_image_path << " doesn't exist, so skipping ...";
				return false;
			}

			cv::Mat color_image_cv2 = cv::imread(color_image_path, cv::IMREAD_UNCHANGED); // IMREAD_UNCHANGED reads with Alpha channel
			
			if(color_image_cv2.empty())
			{
				LOG(WARNING) << "Failed to read color image from " << color_image_path << ", so skipping ...";
				return false;
			}

			cv::Mat depth_image_cv2 = cv::imread(depth_image_path, cv::IMREAD_ANYDEPTH);

			if(depth_image_cv2.empty())
			{
				LOG(WARNING) << "Failed to read depth image from " << depth_image_path << ", so skipping ...";
				return false;
			}

			color_images_cv2.push_back(make_pair(i, color_image_cv2));
			depth_images_cv2.push_back(make_pair(i, depth_image_cv2));

			// show_color_image(color_image_cv2);
			// show_depth_image(depth_image_cv2);
		}
	}

	if(color_images_cv2.size() == 0 || depth_images_cv2.size() == 0)
	{
		LOG(ERROR) << "Empty color or depth images";
		return false;
	}

	// m_pPointCloud->setImages(color_images_cv2, depth_images_cv2);
	m_pPointCloud->setImages(color_images_cv2, depth_images_cv2);

	m_vStageTimes[SERVER_STAGES::LOAD_FRAME] = m_timer.ElapsedMs();
	return true;
}

// Load images from m_WorkDir
/**
 * @brief Loads images from m_WorkDir. Uses openmp to load images in parallel.
 * TODO: Rajrup: Remove load_frame() and rename this to load_frame()
 * @param preload
 */
bool MultiviewServer::load_frame_parallel(bool preload)
{
	if(!m_bPlaybackInitialized)
	{
		LOG(ERROR) << "Playback not initialized";
		return false;
	}

	if(!m_bCalibrated)
	{
		LOG(ERROR) << "Calibration not performed";
		LOG(INFO) << "Calibrate first!";
		return false;
	}

	if(m_currFrameId < 0)
	{
		LOG(ERROR) << "Invalid Frame Id!";
		return false;
	}

	// preload
	if (preload) 
	{
		color_images_cv2 = color_preload[m_currFrameId];
		depth_images_cv2 = depth_preload[m_currFrameId];
	} 
	// runtime load
	else 
	{
		#pragma omp parallel for num_threads(4)
		for (int i = 0; i < m_nDevices; i++)
		{
			uint32_t device_id = m_vDeviceIndices[i];

			string color_image_path = FORMAT(m_WorkDir << "color/" << m_currFrameId << "_color_" << device_id << ".png");
			string depth_image_path = FORMAT(m_WorkDir << "depth/" << m_currFrameId << "_depth_" << device_id << ".png");

			if(!fs::exists(color_image_path))
			{
				LOG(WARNING) << "Color image from " << color_image_path << " doesn't exist, so skipping ...";
				continue;
			}

			if(!fs::exists(depth_image_path))
			{
				LOG(WARNING) << "Depth image from " << depth_image_path << " doesn't exist, so skipping ...";
				continue;
			}

			cv::Mat color_image_cv2 = cv::imread(color_image_path, cv::IMREAD_UNCHANGED); // IMREAD_UNCHANGED reads with Alpha channel
				
			if(color_image_cv2.empty())
			{
				LOG(WARNING) << "Failed to read color image from " << color_image_path << ", so skipping ...";
				continue;
			}
			
			// Debug
			// if(i == 0)
			// {
			// 	// Adding text to 1st view
			// 	int text_offset_x, text_offset_y;
			// 	text_offset_x = 20 + 82 + 300;
			// 	text_offset_y = 20 + 82;
			// 	cv::putText(color_image_cv2, FORMAT(m_currFrameId), cv::Point(text_offset_x, text_offset_y), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(255, 0, 0, 255), 8);
			// }

			cv::Mat depth_image_cv2 = cv::imread(depth_image_path, cv::IMREAD_ANYDEPTH);

			if(depth_image_cv2.empty())
			{
				LOG(WARNING) << "Failed to read depth image from " << depth_image_path << ", so skipping ...";
				continue;
			}

			// Debug
			// if(i == 0)
			// {
			// 	// Adding text to 1st view
			// 	int text_offset_x, text_offset_y;
			// 	text_offset_x = 20 + 82 + 300;
			// 	text_offset_y =  20 + 82;

			// 	for(int i=0; i<82 + 20; i++)
			// 	{
			// 		for(int j=0; j<82 + 100; j++)
			// 		{
			// 			depth_image_cv2.at<uint16_t>(i + 20, j + 20 + 350) = 65535;
			// 		}
			// 	}

			// 	// cv::Mat temp;
			// 	// cv::merge(vector<cv::Mat>{depth_yuv16_cv[0], depth_yuv16_cv[1], depth_yuv16_cv[2]}, temp);
			// 	cv::putText(depth_image_cv2, FORMAT(m_currFrameId), cv::Point(text_offset_x, text_offset_y), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(255), 8);
			// 	// cv::split(temp, vector<cv::Mat>{depth_yuv16_cv[0], depth_yuv16_cv[1], depth_yuv16_cv[2]});
			// }

			// // Debug: Colorize the depth image
			// uint16_t *depth = reinterpret_cast<uint16_t *>(depth_image_cv2.data);
			// RGB *colorized_depth = new RGB[depth_image_cv2.rows * depth_image_cv2.cols];
			// depth_to_color(depth, depth_image_cv2.rows, depth_image_cv2.cols, colorized_depth, false);
			// cv::Mat depth2color_cv2 = color_to_opencv(colorized_depth, depth_image_cv2.cols, depth_image_cv2.rows, false);

			// string out_dir = FORMAT(m_WorkDir << "colorized_depth/");
			// if(!fs::exists(out_dir))
			// 	fs::create_directory(out_dir);
			// cv::imwrite(FORMAT(out_dir << m_currFrameId << "_depth_" << device_id << ".png"), depth2color_cv2);
			// delete[] colorized_depth;

			color_images_cv2[device_id] = make_pair(device_id, color_image_cv2);
			depth_images_cv2[device_id] = make_pair(device_id, depth_image_cv2);
		}
	}

	if(color_images_cv2.size() == 0 || depth_images_cv2.size() == 0)
	{
		LOG(ERROR) << "Empty color or depth images";
		return false;
	}

	m_pPointCloud->setImages(color_images_cv2, depth_images_cv2);
	m_vStageTimes[SERVER_STAGES::LOAD_FRAME] = m_timer.ElapsedMs();
	return true;
}

bool MultiviewServer::load_frame_with_colorized_depth_parallel(bool preload, string format)
{
	if(!m_bPlaybackInitialized)
	{
		LOG(ERROR) << "Playback not initialized";
		return false;
	}

	if(!m_bCalibrated)
	{
		LOG(ERROR) << "Calibration not performed";
		LOG(INFO) << "Calibrate first!";
		return false;
	}

	if(m_currFrameId < 0)
	{
		LOG(ERROR) << "Invalid Frame Id!";
		return false;
	}

	// preload
	if (preload) 
	{
		color_images_cv2 = color_preload[m_currFrameId];
		depth_images_cv2 = depth_preload[m_currFrameId];
	} 
	// runtime load
	else 
	{
		vector<uint16_t *> depth(m_nDevices, NULL);

		// #pragma omp parallel for num_threads(4)
		for (int i = 0; i < m_nDevices; i++)
		{
			uint32_t device_id = m_vDeviceIndices[i];

			string color_image_path = FORMAT(m_WorkDir << m_currFrameId << "_color_" << device_id << ".png");
			
			string depth_image_path = FORMAT(m_WorkDir << m_currFrameId << "_depth_" << device_id << ".png");
			if(format == "jpeg" || format == "jpg")
				depth_image_path = FORMAT(m_WorkDir << m_currFrameId << "_depth_" << device_id << ".jpg");

			if(!fs::exists(color_image_path))
			{
				LOG(WARNING) << "Color image from " << color_image_path << " doesn't exist, so skipping ...";
				continue;
			}

			if(!fs::exists(depth_image_path))
			{
				LOG(WARNING) << "Depth image from " << depth_image_path << " doesn't exist, so skipping ...";
				continue;
			}

			cv::Mat color_image_cv2 = cv::imread(color_image_path, cv::IMREAD_UNCHANGED); // IMREAD_UNCHANGED reads with Alpha channel
				
			if(color_image_cv2.empty())
			{
				LOG(WARNING) << "Failed to read color image from " << color_image_path << ", so skipping ...";
				continue;
			}

			cv::Mat depth_image_cv2 = cv::imread(depth_image_path, cv::IMREAD_UNCHANGED);
			if(depth_image_cv2.channels() == 3)
			{
				// Add alpha channel. Jpeg has 3 channels, png can support 4
				cv::cvtColor(depth_image_cv2, depth_image_cv2, cv::COLOR_BGR2BGRA);
			}

			RGB *colorized_depth = reinterpret_cast<RGB *>(depth_image_cv2.data);
			
			if(depth[device_id] == NULL)
				depth[device_id] = new uint16_t[depth_image_cv2.rows * depth_image_cv2.cols];

			color_to_depth(colorized_depth, depth_image_cv2.rows, depth_image_cv2.cols, depth[device_id], false);
			depth_image_cv2 = depth_to_opencv(depth[device_id], depth_image_cv2.cols, depth_image_cv2.rows);

			// cv::imshow("View", depth_image_cv2);
			// cv::waitKey(0);

			if(depth_image_cv2.empty())
			{
				LOG(WARNING) << "Failed to read depth image from " << depth_image_path << ", so skipping ...";
				continue;
			}

			color_images_cv2[device_id] = make_pair(device_id, color_image_cv2);
			depth_images_cv2[device_id] = make_pair(device_id, depth_image_cv2);
		}

		for (int i = 0; i < m_nDevices; i++)
			delete[] depth[i];
		depth.clear();
	}

	if(color_images_cv2.size() == 0 || depth_images_cv2.size() == 0)
	{
		LOG(ERROR) << "Empty color or depth images";
		return false;
	}

	// cv::imshow("View"+to_string(depth_images_cv2[0].first), depth_images_cv2[0].second);
	// cv::waitKey(0);

	m_pPointCloud->setImages(color_images_cv2, depth_images_cv2);
	m_vStageTimes[SERVER_STAGES::LOAD_FRAME] = m_timer.ElapsedMs();
	return true;
}

void MultiviewServer::update_view()
{	
	/*********************** View Selection ***********************/
	// Select views
	if(FLAGS_server_cull == CULLING::CLIP_CULLING || 
		FLAGS_server_cull == CULLING::CLIP_VOXEL_CULLING)
	{
		m_pPointCloud->updateViewsClipCulling(m_vpCalibration, m_currFrameId);
	}
	else if(FLAGS_server_cull == CULLING::NO_CULLING ||
		FLAGS_server_cull == CULLING::NORMAL_CULLING ||
		FLAGS_server_cull == CULLING::NORMAL_VOXEL_CULLING)
	{
		m_pPointCloud->updateViewsNormalCulling(m_vpCalibration, m_currFrameId);
		// m_pPointCloud->updateViewsExpansionNormalCulling(m_vpCalibration, m_currFrameId);
	}
	else if(FLAGS_server_cull == CULLING::NORMAL_EXPANSION_CULLING)
	{
		// m_pPointCloud->updateViewsExpansionNormalCulling(m_vpCalibration, m_currFrameId);
		// m_pPointCloud->updateViewsExpansionNormalCullingUnion(m_vpCalibration, m_currFrameId);
		m_pPointCloud->updateViewsExpansionNormalCullingBuffer(m_vpCalibration, m_currFrameId);
	}
	else
		LOG(FATAL) << "Invalid culling method";

	// m_pPointCloud->addQR2Views(QRCODE_FOLDER, m_currFrameId);

	// Save views
	if(FLAGS_save_views)
	{
		m_pPointCloud->storeViews(m_WorkDir);

		// Saving culled GT views for calculating RMSE
		// m_pPointCloud->storeViews_standalone(m_WorkDir);
	}
	
	if(FLAGS_save_binary_mask)
	{
		string out_path = FORMAT(m_WorkDir << "binary_mask/");
		if(!fs::exists(out_path))
			fs::create_directory(out_path);
		
		for(auto i : m_vDeviceIndices)
		{
			string binary_mask_path = FORMAT(out_path << m_currFrameId << "_binary_mask_" << i << ".png");
			cv::Mat binary_mask = binary_mask_to_opencv(m_pPointCloud->m_views[i].binary_mask, m_pPointCloud->m_views[i].depthWidth, m_pPointCloud->m_views[i].depthHeight);
			cv::imwrite(binary_mask_path, binary_mask);
		}
	}

	m_vStageTimes[SERVER_STAGES::UPDATE_VIEW] = m_timer.ElapsedMs() - m_vStageTimes[SERVER_STAGES::LOAD_FRAME];
}

uint32_t MultiviewServer::get_view_size_in_frustum()
{
	return m_pPointCloud->m_viewSizeWithFrustumInBytes;
}

uint32_t MultiviewServer::get_view_size_without_frustum()
{
	return m_pPointCloud->m_viewSizeWithoutFrustumInBytes;
}

// void MultiviewServer::update_binary_mask()
// {
// 	m_pPointCloud->calculateBinaryMask(m_vBinaryMasks);

// 	if(FLAGS_save_binary_mask)
// 	{
// 		for(uint32_t i : m_vDeviceIndices)
// 		{
// 			string binary_mask_path = FORMAT(m_WorkDir << m_nFrames << "_binary_mask_" << i << ".png");
// 			cv::imwrite(binary_mask_path, m_vBinaryMasks[i], compression_params);
// 			cout << "Saved binary mask to " << binary_mask_path << endl;
// 		}
// 	}
// }

/**
 * @brief Returns the current selected views. Returning vector of pointers for efficiency. 
 * Call this function after views are updated using the update_view() function.
 * @return vector<View *> Remember to clear the views vector after use. 
 * Don't delete the views (pointing obejects inside the vector) themselves.
 */
vector<View *> MultiviewServer::get_view()
{
	vector<View *> views;
	for(auto &view : m_pPointCloud->m_views)
	{
		views.push_back(&view);
	}
	m_vStageTimes[SERVER_STAGES::UPDATE_VIEW] = m_timer.ElapsedMs() - m_vStageTimes[SERVER_STAGES::LOAD_FRAME];
	return views;
}

// vector<cv::Mat> MultiviewServer::get_binary_mask()
// {
// 	return m_vBinaryMasks;
// }

void MultiviewServer::set_user_data(const UserData &user_data)
{
	m_pPointCloud->setUserData(user_data);
}

void MultiviewServer::set_frustum(const Frustum &frustum)
{
	m_pPointCloud->setFrustum(frustum);
}

void MultiviewServer::set_frustum(const Frustum &prev_frustum, const Frustum &pred_frustum)
{
	m_pPointCloud->setFrustum(prev_frustum, pred_frustum);
}

void MultiviewServer::set_frustum_clip(const FrustumClip &frustum)
{
	m_pPointCloud->setFrustumClip(frustum);
}

void MultiviewServer::set_work_dir(const string &path)
{
	m_WorkDir = path;
}

uint64_t MultiviewServer::get_load_frame_time()
{
	return m_vStageTimes[SERVER_STAGES::LOAD_FRAME];
}

uint64_t MultiviewServer::get_update_view_time()
{
	return m_vStageTimes[SERVER_STAGES::UPDATE_VIEW];
}

/**
 * @brief TODO: Rajrup: This might be incorrect. Check it.
 * 
 * @return uint64_t 
 */
uint64_t MultiviewServer::get_send_frame_time()
{
	m_vStageTimes[SERVER_STAGES::SEND_FRAME] = m_timer.ElapsedMs() - (m_vStageTimes[SERVER_STAGES::UPDATE_VIEW] + m_vStageTimes[SERVER_STAGES::LOAD_FRAME]);
	return m_vStageTimes[SERVER_STAGES::SEND_FRAME];
}

uint64_t MultiviewServer::get_elapsed_time()
{
	return m_timer.ElapsedMs();
}

bool MultiviewServer::preload() 
{
	if(!m_bPlaybackInitialized)
	{
		LOG(ERROR) << "Playback not initialized";
		return false;
	}

	if(!m_bCalibrated)
	{
		LOG(ERROR) << "Calibration not performed";
		LOG(INFO) << "Calibrate first!";
		return false;
	}

	time_point start, end;
	uint64_t elapsed_ms;

	start = time_now();

	for(uint32_t preload_id = START_FRAME; preload_id <= END_FRAME; preload_id++) 
	{
		LOG(INFO) << "Preloading frame " << preload_id;
		// <camera device id, image>
		vector<pair<uint32_t, cv::Mat>> color_images_cv2;
		vector<pair<uint32_t, cv::Mat>> depth_images_cv2;

		for(uint32_t i : m_vDeviceIndices)
		{
			string color_image_path = FORMAT(m_WorkDir << preload_id << "_color_" << i << ".png");
			string depth_image_path = FORMAT(m_WorkDir << preload_id << "_depth_" << i << ".png");

			if(!fs::exists(color_image_path))
			{
				LOG(WARNING) << "Color image from " << color_image_path << " doesn't exist, so skipping ...";
				return false;
			}

			if(!fs::exists(depth_image_path))
			{
				LOG(WARNING) << "Depth image from " << depth_image_path << " doesn't exist, so skipping ...";
				return false;
			}

			cv::Mat color_image_cv2 = cv::imread(color_image_path, cv::IMREAD_UNCHANGED); // IMREAD_UNCHANGED reads with Alpha channel
			
			if(color_image_cv2.empty())
			{
				LOG(WARNING) << "Failed to read color image from " << color_image_path << ", so skipping ...";
				return false;
			}
			
			cv::Mat depth_image_cv2 = cv::imread(depth_image_path, cv::IMREAD_ANYDEPTH);

			if(depth_image_cv2.empty())
			{
				LOG(WARNING) << "Failed to read depth image from " << depth_image_path << ", so skipping ...";
				return false;
			}

			color_images_cv2.push_back(make_pair(i, color_image_cv2));
			depth_images_cv2.push_back(make_pair(i, depth_image_cv2));
		}

		color_preload.push_back(color_images_cv2);
		depth_preload.push_back(depth_images_cv2);
	}

	end = time_now();
	elapsed_ms = duration_ms(start, end);
	LOG(INFO) << "Preloaded frame in " << elapsed_ms << " ms";

	return true;
}

// For pointcloud transmission only
void MultiviewServer::update_ptcl()
{
	time_point start, end;
	uint64_t elapsed_ms;
	start = time_now();

	/*********************** PtCl Generation ***********************/
	end = time_now();
	elapsed_ms = duration_ms(start, end);
	cout << "Updated views in " << elapsed_ms << " ms" << endl;

	start = time_now();
	// Generate point cloud
	if(FLAGS_server_cull == CULLING::NO_CULLING || 
		FLAGS_server_cull == CULLING::CLIP_CULLING ||
		FLAGS_server_cull == CULLING::NORMAL_CULLING)
		m_pPointCloud->updatePointCloud(m_vpCalibration);
	else if(FLAGS_server_cull == CULLING::NORMAL_VOXEL_CULLING)
		m_pPointCloud->updatePointCloudFromVoxel(m_vpCalibration);
	else
	{
		cout << "Invalid culling method" << endl;
		exit(EXIT_FAILURE);
	}

	end = time_now();
	elapsed_ms = duration_ms(start, end);
	cout << "Updated ptcl in " << elapsed_ms << " ms" << endl;
}

int MultiviewServer::get_num_points_ptcl()
{
	return m_pPointCloud->getNumPoints();
}

/**
 * @brief Returns compressed point cloud. Also stores the compressed point cloud, which can be accessed using get_compressed_ptcl()
 * 
 * @param size compressed ptcl size in bytes
 * @param compressionType 
 * @param zstd_params 
 * @param draco_params 
 * @return const uint8_t* Returns a pointer to the compressed point cloud
 */
const uint8_t *MultiviewServer::compress_ptcl(int &size, string compressionType, const zstd_options &zstd_params, const draco_options &draco_params)
{
	StopWatch sw;

	if(m_pCompressedPtcl != NULL)
	{
		delete[] m_pCompressedPtcl;
		m_pCompressedPtcl = NULL;
	}

	if(compressionType == "zstd")
	{
		m_pCompressedPtcl = m_pPointCloud->compressPointCloudZstd(size, zstd_params.compression_level);
		m_compressedPtclSize = size;
	}
	else if(compressionType == "draco")
	{
		m_pCompressedPtcl = m_pPointCloud->compressPointCloudDraco(size, draco_params.cl, draco_params.qp);
		m_compressedPtclSize = size;
	}
	else
	{
		LOG(FATAL) << "Invalid compression type";
		m_compressedPtclSize = 0;
	}
	
	// LOG(INFO) << "Compressed ptcl in " << sw.ElapsedMs() << " ms";
	return m_pCompressedPtcl;
}

const uint8_t *MultiviewServer::get_compressed_ptcl(int &size)
{
	size = m_compressedPtclSize;
	return m_pCompressedPtcl;
}

int MultiviewServer::get_compressed_ptcl_size()
{
	return m_compressedPtclSize;
}