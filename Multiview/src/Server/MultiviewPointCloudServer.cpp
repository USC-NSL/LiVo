#include "Server/MultiviewPointCloudServer.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <zstd.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include "consts.h"
#include "k4aconsts.h"

#define VOXEL_SIZE 0.05F		// To Do: Rajrup: Not working for 0.01F
#define VOXEL_MIN_BOUND -4.0F
#define VOXEL_MAX_BOUND 4.0F

#define GROUND_THRESHOLD 0.06F
#define WALL_THRESHOLD 2.4F

namespace fs = boost::filesystem;

void calc_max_min_corners_frustum(const Frustum &frustum, Vector3f &min_corner, Vector3f &max_corner)
{
	Vector3f corners[8] = {frustum.fbl, frustum.fbr, frustum.ftl, frustum.ftr, frustum.nbl, frustum.nbr, frustum.ntl, frustum.ntr};

	for(int i = 0; i < 8; i++)
	{
		if(i == 0)
		{
			min_corner = corners[i];
			max_corner = corners[i];
		}
		else
		{
			min_corner = min_corner.cwiseMin(corners[i]);
			max_corner = max_corner.cwiseMax(corners[i]);
		}
	}
}

MultiviewPointCloudServer::MultiviewPointCloudServer(const vector<uint32_t> &indices, const vector<string> &deviceSerials, const int datasetType)
{
	m_datasetType = datasetType;

	device_indices = indices;
	device_serial_numbers = deviceSerials;

	nColorFrameHeight.resize(device_indices.size()); // To Do: RAJRUP: Change to vector<int> to vector<pair<int, int>> for height and width
	nColorFrameWidth.resize(device_indices.size());

	nDepthFrameHeight.resize(device_indices.size());
	nDepthFrameWidth.resize(device_indices.size());

	colorImage.resize(device_indices.size());
	depthImage.resize(device_indices.size());

	device_transformationColorDownscaled.resize(device_indices.size());
	device_calibrationColorDownscaled.resize(device_indices.size());
	xy_tables.resize(device_indices.size(), NULL);

	pDepth.resize(device_indices.size(), NULL);
	pPoint.resize(device_indices.size(), NULL);
	pColorRGBX.resize(device_indices.size(), NULL);

	transformedDepthImage.resize(device_indices.size());

	m_views.resize(device_indices.size());

	minBound[0] = minBound[1] = minBound[2] = VOXEL_MIN_BOUND;
	maxBound[0] = maxBound[1] = maxBound[2] = VOXEL_MAX_BOUND;

	pVoxel = new Voxel(minBound, maxBound, VOXEL_SIZE);

	goodIndices.resize(device_indices.size());
}

MultiviewPointCloudServer::~MultiviewPointCloudServer()
{
	nColorFrameHeight.clear();
	nColorFrameWidth.clear();

	nDepthFrameHeight.clear();
	nDepthFrameWidth.clear();

	for (auto i : device_indices)
	{
		device_transformationColorDownscaled[i].destroy();
		
		if(m_datasetType == DATASET::KINECT)
		{
			delete [] xy_tables[i]->x_table;
			delete [] xy_tables[i]->y_table;
			delete xy_tables[i];
		}

		if(pPoint[i] != NULL)
			delete[] pPoint[i];

		// No need to clear them, since they point to the memory of the k4a::image
		// if(pColorRGBX[i] != NULL)
		// 	delete [] pColorRGBX[i];
		// if(pDepth[i] != NULL)
		// 	delete[] pDepth[i];

		colorImage[i].reset();
		depthImage[i].reset();
		transformedDepthImage[i].reset();
	}
	colorImage.clear();
	depthImage.clear();
	device_transformationColorDownscaled.clear();
	device_calibrationColorDownscaled.clear();
	xy_tables.clear();
	transformedDepthImage.clear();

	pDepth.clear();
	pColorRGBX.clear();

	delete pVoxel;
	pVoxel = NULL;

	m_views.clear();

	device_indices.clear();
	device_serial_numbers.clear();
}

void MultiviewPointCloudServer::setDeviceIndices(const vector<uint32_t> &indices)
{
	device_indices = indices;
}

void MultiviewPointCloudServer::setDeviceSerials(const vector<string> &deviceSerials)
{
	device_serial_numbers = deviceSerials;
}

bool MultiviewPointCloudServer::loadCalibrationColorDownscaled(const string &path, const uint32_t index, k4a_calibration_t &calibrationColorDownscaled)
{
	fs::path bin_filepath{FORMAT(path << CALIBRATION_INT_FILENAME << "_raw_" << device_serial_numbers[index] << ".bin")};
	fs::ifstream bin_file;
	bin_file.open(bin_filepath, ios::binary);

	if (!bin_file.is_open())
	{	
		LOG(ERROR) << "Failed to open file: " << bin_filepath;
		return false;
	}

	vector<uint8_t> blob;
	boost::archive::binary_iarchive iar(bin_file);
	iar >> blob;

	bin_file.close();
	LOG(INFO) << "Raw Intrinsic Calibration for device " << device_serial_numbers[index] << " loaded from " << bin_filepath;

	k4a_result_t result = k4a_calibration_get_from_raw(reinterpret_cast<char *>(&blob[0]), blob.size(), DEFAULT_DEPTH_MODE, DEFAULT_COLOR_MODE, &calibrationColorDownscaled);

	if (K4A_RESULT_SUCCEEDED != result)
	{
		LOG(ERROR) << "Failed to load calibration from raw calibration blob for index: " << index;
		return false;
	}

	fs::path filepath{FORMAT(path << CALIBRATION_INT_FILENAME << "_ColorDownscaled_" << device_serial_numbers[index] << ".txt")};
	fs::ifstream file;
	file.open(filepath, ios::in);
	if (!file.is_open())
	{
		LOG(ERROR) << "Failed to load Calibration file: " << filepath;
		return false;
	}

	uint32_t calib_param;
	float calib_value;
	while(!file.eof())	
	{
		file >> calib_param >> calib_value;
		switch(calib_param)
		{
			case RW:
				calibrationColorDownscaled.color_camera_calibration.resolution_width = int(calib_value);
				break;
			case RH:
				calibrationColorDownscaled.color_camera_calibration.resolution_height = int(calib_value);
				break;
			case FX:
				calibrationColorDownscaled.color_camera_calibration.intrinsics.parameters.param.fx = calib_value;
				break;
			case FY:
				calibrationColorDownscaled.color_camera_calibration.intrinsics.parameters.param.fy = calib_value;
				break;
			case CX:
				calibrationColorDownscaled.color_camera_calibration.intrinsics.parameters.param.cx = calib_value;
				break;
			case CY:
				calibrationColorDownscaled.color_camera_calibration.intrinsics.parameters.param.cy = calib_value;
				break;
			default:
				LOG(ERROR) << "Unknown calibration parameter: " << calib_param;
				return false;
		}
	}

	file.close();
	LOG(INFO) << "Intrinsic Calibration for device " << device_serial_numbers[index] << " loaded from " << filepath;

	return true;
}

/**
 * Rajrup: Read intrinsic camera parameters file
 * To Do: Only supports transformation colored downscaled
 */
bool MultiviewPointCloudServer::loadTransformations(const string &path)
{
	if(m_datasetType == DATASET::KINECT)
	{
		for (uint32_t index : device_indices)
		{
			k4a_calibration_t calibration;
			if(!loadCalibrationColorDownscaled(path, index, calibration))
			{
				LOG(ERROR) << "Failed to load Calibration file for device index: " << index;
				return false;
			}
			device_calibrationColorDownscaled[index] = calibration;
			device_transformationColorDownscaled[index] = k4a::transformation(calibration);
		}
		initXYTables(K4A_CALIBRATION_TYPE_COLOR); // Populate the lookup tables for x,y,z when calibration is loaded
	}
	
	return true;
}

/**
 * RAJRUP: Inspiration from func transformation_init_xy_tables() in transformation.c
 * Also check fastpointcloud example in Kinect SDK
 */
void MultiviewPointCloudServer::initXYTables(const k4a_calibration_type_t camera)
{
	for (uint32_t index : device_indices)
	{
		k4a_calibration_t calibration = device_calibrationColorDownscaled[index];

		int width = 0;
    	int height = 0;

		switch (camera)
		{
			case K4A_CALIBRATION_TYPE_DEPTH:
				width = calibration.depth_camera_calibration.resolution_width;
				height = calibration.depth_camera_calibration.resolution_height;
				break;
			case K4A_CALIBRATION_TYPE_COLOR:
				width = calibration.color_camera_calibration.resolution_width;
				height = calibration.color_camera_calibration.resolution_height;
				break;
			default:
				LOG(ERROR) << "Unknown Camera Type: " << camera;
				break;
		}

		if(xy_tables[index] == NULL)
			xy_tables[index] = new xy_table_t(width, height);
		else
		{
			delete [] xy_tables[index]->x_table;
			delete [] xy_tables[index]->y_table;
			delete xy_tables[index];
			xy_tables[index] = new xy_table_t(width, height);
		}

		k4a_float2_t p;
		k4a_float3_t ray;
		int valid = 1;

		for (int y = 0, idx = 0; y < height; y++)
		{
			p.xy.y = (float)y;
			for (int x = 0; x < width; x++, idx++)
			{
				p.xy.x = (float)x;

				if(k4a_calibration_2d_to_3d(&calibration, &p, 1.f, camera, camera, &ray, &valid) == K4A_RESULT_SUCCEEDED)
				{
					if (valid)
					{
						xy_tables[index]->x_table[idx] = ray.xyz.x;
						xy_tables[index]->y_table[idx] = ray.xyz.y;
					}
					else
					{
						xy_tables[index]->x_table[idx] = nanf("");
						xy_tables[index]->y_table[idx] = nanf("");
					}
				}
				else
				{
					xy_tables[index]->x_table[idx] = nanf("");
					xy_tables[index]->y_table[idx] = nanf("");
				}
			}
		}
	}
}

vector<xy_table_t *> MultiviewPointCloudServer::getXYTables()
{
	return xy_tables;
}

/**
 * @brief Images are sorted according to device indices
 * 
 * @param cImgs <Camera Device Id, Color image>
 * @param dImgs <Camera Device Id, Depth image>
 */
void MultiviewPointCloudServer::setImages(const vector<pair<uint32_t, cv::Mat>> &cImgs, const vector<pair<uint32_t, cv::Mat>> &dImgs)
{
	assert(cImgs.size() == dImgs.size());
	assert(cImgs.size() == device_indices.size());

	for(int i = 0; i < cImgs.size(); i++)
	{
		uint32_t index = cImgs[i].first;
		assert(index == dImgs[i].first);

		cv::Mat cImg = cImgs[i].second;
		cv::Mat dImg = dImgs[i].second;

		// LOG(INFO) << "Color Channels: " << cImg.channels();
		// LOG(INFO) << "Depth Channels: " << dImg.channels();

		// // check if any of the depth value is > 0
		// LOG(INFO) << "No. of non-zero elements: " << cv::countNonZero(dImg);

		nColorFrameWidth[index] = cImg.cols;
		nColorFrameHeight[index] = cImg.rows;

		nDepthFrameWidth[index] = dImg.cols;
		nDepthFrameHeight[index] = dImg.rows;

		if(!colorImage[index].is_valid())
			colorImage[index] = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, cImg.cols, cImg.rows, cImg.cols * 4 * (int)sizeof(uint8_t));

		if(!depthImage[index].is_valid())
			depthImage[index] = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, dImg.cols, dImg.rows, dImg.cols * (int)sizeof(uint16_t));

		if (!transformedDepthImage[index].is_valid())
			transformedDepthImage[index] = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, 
																nColorFrameWidth[index], 
																nColorFrameHeight[index], 
																nColorFrameWidth[index] * 
																(int)sizeof(uint16_t));

		memcpy(colorImage[index].get_buffer(), &cImg.ptr<cv::Vec4b>(0)[0], cImg.rows * cImg.cols * sizeof(cv::Vec4b));
		memcpy(depthImage[index].get_buffer(), &dImg.ptr<uint16_t>(0)[0], dImg.rows * dImg.cols * sizeof(uint16_t));

		if(m_datasetType == DATASET::KINECT)
		{
			device_transformationColorDownscaled[index].depth_image_to_color_camera(depthImage[index], &transformedDepthImage[index]);
			
			nDepthFrameWidth[index] = transformedDepthImage[index].get_width_pixels();
			nDepthFrameHeight[index] = transformedDepthImage[index].get_height_pixels();
		}

		// After tranformation of depth to color image, color and depth image dimensions are same.
		assert(nDepthFrameWidth[index] == nColorFrameWidth[index]);
		assert(nDepthFrameHeight[index] == nColorFrameHeight[index]);

		if(pPoint[index] == NULL)
			pPoint[index] = new Point3f[nColorFrameWidth[index] * nColorFrameHeight[index]];

		// if(pColorRGBX[index] == NULL)
		// 	pColorRGBX[index] = new RGB[nColorFrameWidth[index] * nColorFrameHeight[index]];
		
		// if(pDepth[index] == NULL)
		// 	pDepth[index] = new uint16_t[nDepthFrameWidth[index] * nDepthFrameHeight[index]];
		
		// TODO: Rajrup: Reinterpret cast causes pColorRGBX, pDepth to point to the memory of the k4a::image
		// So there is memory to original allocation of memory. CHECK THIS!!
		pColorRGBX[index] = reinterpret_cast<RGB *>(colorImage[index].get_buffer());

		if(m_datasetType == DATASET::KINECT)
			pDepth[index] = reinterpret_cast<uint16_t *>(transformedDepthImage[index].get_buffer());
		else
			pDepth[index] = reinterpret_cast<uint16_t *>(depthImage[index].get_buffer());
		// pDepth[index] = reinterpret_cast<uint16_t *>(transformedDepthImage[index].get_buffer());
	}

	// // DEBUG: Print the size of the color and depth images
	// if(cImgs.size() > 0 && dImgs.size() > 0)
	// {
	// 	LOG(INFO) << "Resolution of Color Image: (" << nColorFrameHeight[0] << ", " << nColorFrameWidth[0] << ")";
	// 	LOG(INFO) << "Resolution of Depth Image: (" << nDepthFrameHeight[0] << ", " << nDepthFrameWidth[0] << ")";
	// }
}

/**
 * @brief Update views using on the server side.
 * This function performs frustum culling, if FLAGS_load_frustum || FLAGS_update_frustum  || FLAGS_server_cull is true. 
 * Else, it just updates the views without culling.
 * @param extCalibration 
 * @param frameID 
 */
void MultiviewPointCloudServer::updateViewsClipCulling(const vector<Calibration *> &extCalibration, int frameID)
{
	/**
	 * @brief Bounding box of 8 meteres around the camera. 
	 * This is used to avoid finding tight bounding box.
	 * We can use incremental octree generation.
	 */

	minBound[0] = minBound[1] = minBound[2] = -4.0f;
	maxBound[0] = maxBound[1] = maxBound[2] = 4.0f;

	pVoxel->clearVoxels2();

	m_viewSizeWithFrustumInBytes = 0;
	m_viewSizeWithoutFrustumInBytes = 0;

	for (uint32_t index : device_indices)
	{
		if(!m_views[index].isInitialized())
			m_views[index].initView(nDepthFrameWidth[index], nDepthFrameHeight[index], nColorFrameWidth[index], nColorFrameHeight[index]);

		m_views[index].frameID = frameID;
		m_views[index].viewID = index;
		m_views[index].valid = false;		// Invalidate the view at the beginning

		bool validView = false;

		time_point start, end;
		uint64_t elapsed_ms;
		
		xy_table_t *xy_table_data_ = NULL;
		if(m_datasetType == DATASET::KINECT)
			xy_table_data_ = xy_tables[index];
		
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		Calibration *calibration = extCalibration[index];
		if(!calibration->bCalibrated)
			LOG(FATAL) << "Calibration not available for device " << index;

		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;

		RGB *color_data = pColorRGBX[index];
		uint16_t *depth_data = pDepth[index];

		Point3f point, temp;
		uint32_t numValidPoints = 0;
		uint32_t totalPoints = 0;
		RGB no_color;

		for (uint32_t i = 0; i < nVertices; i++)
		{
			// RGB *color = &color_data[i];
			if (depth_data[i] > 0)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Change these operations to matrix multiplication using Eigen library.
				 */

				if(m_datasetType == DATASET::KINECT)
				{
					if(isnan(xy_table_data_->x_table[i]) || isnan(xy_table_data_->y_table[i]))
						continue;
					
					calibration->kinect2point(xy_table_data_, depth_data[i], i, point);
				}
				else if(m_datasetType == DATASET::PANOPTIC)
				{
					calibration->panoptic2point(depth_data[i], i, point);
				}
				else
					LOG(FATAL) << "Invalid dataset type";

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.
				// Find min-max tight bounding box around the point cloud in updatePointCloudPlaneCulling() function.
				
				totalPoints++;

				// clip culling
				if (FLAGS_server_cull == CULLING::CLIP_CULLING)
				{ 
					Eigen::Vector4f f_point(point.X, point.Y, point.Z, 1.0);
					if(m_frustumClip.pointInFrustum(f_point) == INSIDE)
					{
						// Adding point to the server view and select the view;
						m_views[index].depth_image[i] = depth_data[i];
						m_views[index].color_image[i] = color_data[i];
						m_views[index].binary_mask[i] = 1;
						validView = true;
						numValidPoints++;
					}
					else	
					{
						m_views[index].depth_image[i] = 0;
						m_views[index].color_image[i] = no_color;
						m_views[index].binary_mask[i] = 0;
					}
				}
				// voxel culling
				else if(FLAGS_server_cull == CULLING::CLIP_VOXEL_CULLING)
				{
					Vector3f v_point(point.X, point.Y, point.Z);
					int frustum_pos = pVoxel->pointInFrustum(v_point, &m_frustumClip);
					if(frustum_pos == INSIDE || frustum_pos == INTERSECT)
					{
						// Adding point to the server view and select the view;
						m_views[index].depth_image[i] = depth_data[i];
						m_views[index].color_image[i] = color_data[i];
						m_views[index].binary_mask[i] = 1;
						validView = true;
						numValidPoints++;
					}
					else
					{
						m_views[index].depth_image[i] = 0;
						m_views[index].color_image[i] = no_color;
						m_views[index].binary_mask[i] = 0;
					}
				}
				else 
				{
					// NO CULLING
					// Adding point to the server view and select the view;
					m_views[index].depth_image[i] = depth_data[i];
					m_views[index].color_image[i] = color_data[i];
					m_views[index].binary_mask[i] = 1;
					validView = true;
					numValidPoints++;
				}
			}
			else
			{
				m_views[index].depth_image[i] = 0;
				m_views[index].color_image[i] = no_color;
				m_views[index].binary_mask[i] = 0;
			}
		}
		
		m_viewSizeWithFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * numValidPoints;
		m_viewSizeWithoutFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * totalPoints;
		m_views[index].valid = validView;
	}
}

void MultiviewPointCloudServer::updateViewsNormalCulling(const vector<Calibration *> &extCalibration, int frameID)
{
	/**
	 * @brief Bounding box of 8 meteres around the camera. 
	 * This is used to avoid finding tight bounding box.
	 * We can use incremental octree generation.
	 */

	minBound[0] = minBound[1] = minBound[2] = -4.0f;
	maxBound[0] = maxBound[1] = maxBound[2] = 4.0f;

	pVoxel->clearVoxels2();

	m_viewSizeWithFrustumInBytes = 0;
	m_viewSizeWithoutFrustumInBytes = 0;

	for (uint32_t index : device_indices)
	{
		if(!m_views[index].isInitialized())
			m_views[index].initView(nDepthFrameWidth[index], nDepthFrameHeight[index], nColorFrameWidth[index], nColorFrameHeight[index]);

		m_views[index].frameID = frameID;
		m_views[index].viewID = index;
		m_views[index].valid = false;		// Invalidate the view at the beginning

		bool validView = false;

		time_point start, end;
		uint64_t elapsed_ms;
		
		xy_table_t *xy_table_data_ = NULL;
		if(m_datasetType == DATASET::KINECT)
			xy_table_data_ = xy_tables[index];
		
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		Calibration *calibration = extCalibration[index];
		if(!calibration->bCalibrated)
			LOG(FATAL) << "Calibration not available for device " << index;

		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;

		RGB *color_data = pColorRGBX[index];
		uint16_t *depth_data = pDepth[index];

		Point3f point, temp;
		uint32_t numValidPoints = 0;
		uint32_t totalPoints = 0;
		RGB no_color;

		for (uint32_t i = 0; i < nVertices; i++)
		{
			// RGB *color = &color_data[i];
			if (depth_data[i] > 0)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Change these operations to matrix multiplication using Eigen library.
				 */

				if(m_datasetType == DATASET::KINECT)
				{
					if(isnan(xy_table_data_->x_table[i]) || isnan(xy_table_data_->y_table[i]))
						continue;
					
					calibration->kinect2point(xy_table_data_, depth_data[i], i, point);
				}
				else if(m_datasetType == DATASET::PANOPTIC)
				{
					calibration->panoptic2point(depth_data[i], i, point);
				}
				else
					LOG(FATAL) << "Invalid dataset type";

				// Ignore floor points
				if(FLAGS_ground == false)
				{
					if(point.Y > GROUND_THRESHOLD)
						point.Invalid = true;

					// Remove floor lights
					float distFromCenter = sqrt((point.X * point.X) + (point.Z * point.Z));
					if(distFromCenter > WALL_THRESHOLD)
						point.Invalid = true;
				}

				if(point.Invalid)
				{
					m_views[index].depth_image[i] = 0;
					m_views[index].color_image[i] = no_color;
					m_views[index].binary_mask[i] = 0;
					continue;
				}

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.
				// Find min-max tight bounding box around the point cloud in updatePointCloudPlaneCulling() function.
				
				totalPoints++;

				// normal culling - also named plane culling in the code
				if (FLAGS_server_cull == CULLING::NORMAL_CULLING)
				{ 
					Vector3f f_point(point.X, point.Y, point.Z);

					if(m_frustum.pointInFrustum2(f_point) == INSIDE)
					{
						// Adding point to the server view and select the view;
						m_views[index].depth_image[i] = depth_data[i];
						m_views[index].color_image[i] = color_data[i];
						m_views[index].binary_mask[i] = 1;
						validView = true;
						numValidPoints++;
					}
					else	
					{
						m_views[index].depth_image[i] = 0;
						m_views[index].color_image[i] = no_color;
						m_views[index].binary_mask[i] = 0;
					}
				}
				// voxel culling
				else if(FLAGS_server_cull == CULLING::NORMAL_VOXEL_CULLING)
				{
					// To DO: Rajrup - Add voxel culling here
					Vector3f f_point(point.X, point.Y, point.Z);

					if(m_frustum.pointInFrustum2(f_point) == INSIDE)
					{
						// Adding point to the server view and select the view;
						m_views[index].depth_image[i] = depth_data[i];
						m_views[index].color_image[i] = color_data[i];
						m_views[index].binary_mask[i] = 1;
						validView = true;
						numValidPoints++;
					}
					else	
					{
						m_views[index].depth_image[i] = 0;
						m_views[index].color_image[i] = no_color;
						m_views[index].binary_mask[i] = 0;
					}
				}
				else 
				{
					// NO CULLING
					// Adding point to the server view and select the view;
					m_views[index].depth_image[i] = depth_data[i];
					m_views[index].color_image[i] = color_data[i];
					m_views[index].binary_mask[i] = 1;
					validView = true;
					numValidPoints++;
				}
			}
			else
			{
				m_views[index].depth_image[i] = 0;
				m_views[index].color_image[i] = no_color;
				m_views[index].binary_mask[i] = 0;
			}
		}
		
		m_viewSizeWithFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * numValidPoints;
		m_viewSizeWithoutFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * totalPoints;
		m_views[index].valid = validView;

		// All views are alwyas valid.
		m_views[index].valid = true;
	}
}

/**
 * @brief Expand the space between last received real user frustum and predicted user frustum.
 * Algorithm: Fit a tight cube that enxloses 2 frustums. Expand the cube by expansionFactor.
 * Cull points that lie ourside this expanded space.
 * @param extCalibration 
 * @param frameID 
 * @param expansionFactor
 */
void MultiviewPointCloudServer::updateViewsExpansionNormalCulling(const vector<Calibration *> &extCalibration, int frameID, float expansionFactor)
{
	pVoxel->clearVoxels2();

	// Cube enclosing the 2 frustums.
	Vector3f min_corner1, max_corner1, min_corner2, max_corner2;
	m_prevFrustum.getFrustumCubeEnclosedCorners(min_corner1, max_corner1);
	m_predFrustum.getFrustumCubeEnclosedCorners(min_corner2, max_corner2);

	calc_max_min_corners_frustum(m_prevFrustum, min_corner1, max_corner1);
	calc_max_min_corners_frustum(m_predFrustum, min_corner2, max_corner2);

	Vector3f cube_min_corner = min_corner1.cwiseMin(min_corner2);
	Vector3f cube_max_corner = max_corner1.cwiseMax(max_corner2);

	m_viewSizeWithFrustumInBytes = 0;
	m_viewSizeWithoutFrustumInBytes = 0;

	// Now we will check if points lie in the cube or not.
	for (uint32_t index : device_indices)
	{
		if(!m_views[index].isInitialized())
			m_views[index].initView(nDepthFrameWidth[index], nDepthFrameHeight[index], nColorFrameWidth[index], nColorFrameHeight[index]);

		m_views[index].frameID = frameID;
		m_views[index].viewID = index;
		m_views[index].valid = false;		// Invalidate the view at the beginning

		bool validView = false;

		time_point start, end;
		uint64_t elapsed_ms;
		
		xy_table_t *xy_table_data_ = NULL;
		if(m_datasetType == DATASET::KINECT)
			xy_table_data_ = xy_tables[index];
		
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		Calibration *calibration = extCalibration[index];
		if(!calibration->bCalibrated)
			LOG(FATAL) << "Calibration not available for device " << index;

		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;

		RGB *color_data = pColorRGBX[index];
		uint16_t *depth_data = pDepth[index];

		Point3f point, temp;
		uint32_t numValidPoints = 0;
		uint32_t totalPoints = 0;
		RGB no_color;

		for (uint32_t i = 0; i < nVertices; i++)
		{
			// RGB *color = &color_data[i];
			if (depth_data[i] > 0)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Change these operations to matrix multiplication using Eigen library.
				 */

				if(m_datasetType == DATASET::KINECT)
				{
					if(isnan(xy_table_data_->x_table[i]) || isnan(xy_table_data_->y_table[i]))
						continue;
					
					calibration->kinect2point(xy_table_data_, depth_data[i], i, point);
				}
				else if(m_datasetType == DATASET::PANOPTIC)
				{
					calibration->panoptic2point(depth_data[i], i, point);
				}
				else
					LOG(FATAL) << "Invalid dataset type";

				// Ignore floor points
				if(FLAGS_ground == false)
				{
					if(point.Y > -1.0f * GROUND_THRESHOLD)
						continue;

					// Remove floor lights
					float distFromCenter = sqrt(point.X * point.X + point.Z * point.Z);
					if(distFromCenter > WALL_THRESHOLD)
						continue;
				}

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.
				// Find min-max tight bounding box around the point cloud in updatePointCloudPlaneCulling() function.
				
				totalPoints++;

				// normal culling - also named plane culling in the code
				if (FLAGS_server_cull == CULLING::NORMAL_EXPANSION_CULLING)
				{
					if(point.X >= cube_min_corner(0) && point.X <= cube_max_corner(0) &&
						point.Y >= cube_min_corner(1) && point.Y <= cube_max_corner(1) &&
						point.Z >= cube_min_corner(2) && point.Z <= cube_max_corner(2))
					{
						// Adding point to the server view and select the view;
						m_views[index].depth_image[i] = depth_data[i];
						m_views[index].color_image[i] = color_data[i];
						m_views[index].binary_mask[i] = 1;
						validView = true;
						numValidPoints++;
					}
					else	
					{
						m_views[index].depth_image[i] = 0;
						m_views[index].color_image[i] = no_color;
						m_views[index].binary_mask[i] = 0;
					}
				}
				else 
				{
					// NO CULLING
					// Adding point to the server view and select the view;
					m_views[index].depth_image[i] = depth_data[i];
					m_views[index].color_image[i] = color_data[i];
					m_views[index].binary_mask[i] = 1;
					validView = true;
					numValidPoints++;
				}
			}
			else
			{
				m_views[index].depth_image[i] = 0;
				m_views[index].color_image[i] = no_color;
				m_views[index].binary_mask[i] = 0;
			}
		}
		
		m_viewSizeWithFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * numValidPoints;
		m_viewSizeWithoutFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * totalPoints;
		m_views[index].valid = validView;

		// cout << "View ID: " << m_views[index].viewID << ", Valid Points: " << numValidPoints << endl;
	}

}

void MultiviewPointCloudServer::updateViewsExpansionNormalCullingBuffer(const vector<Calibration *> &extCalibration, int frameID)
{
	/**
	 * @brief Bounding box of 8 meteres around the camera. 
	 * This is used to avoid finding tight bounding box.
	 * We can use incremental octree generation.
	 */

	minBound[0] = minBound[1] = minBound[2] = -4.0f;
	maxBound[0] = maxBound[1] = maxBound[2] = 4.0f;

	pVoxel->clearVoxels2();

	m_viewSizeWithFrustumInBytes = 0;
	m_viewSizeWithoutFrustumInBytes = 0;

	for (uint32_t index : device_indices)
	{
		if(!m_views[index].isInitialized())
			m_views[index].initView(nDepthFrameWidth[index], nDepthFrameHeight[index], nColorFrameWidth[index], nColorFrameHeight[index]);

		m_views[index].frameID = frameID;
		m_views[index].viewID = index;
		m_views[index].valid = false;		// Invalidate the view at the beginning

		bool validView = false;

		time_point start, end;
		uint64_t elapsed_ms;
		
		xy_table_t *xy_table_data_ = NULL;
		if(m_datasetType == DATASET::KINECT)
			xy_table_data_ = xy_tables[index];
		
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		Calibration *calibration = extCalibration[index];
		if(!calibration->bCalibrated)
			LOG(FATAL) << "Calibration not available for device " << index;

		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;

		RGB *color_data = pColorRGBX[index];
		uint16_t *depth_data = pDepth[index];

		Point3f point, temp;
		uint32_t numValidPoints = 0;
		uint32_t totalPoints = 0;
		RGB no_color;

		for (uint32_t i = 0; i < nVertices; i++)
		{
			// RGB *color = &color_data[i];
			if (depth_data[i] > 0)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Change these operations to matrix multiplication using Eigen library.
				 */

				if(m_datasetType == DATASET::KINECT)
				{
					if(isnan(xy_table_data_->x_table[i]) || isnan(xy_table_data_->y_table[i]))
						continue;
					
					calibration->kinect2point(xy_table_data_, depth_data[i], i, point);
				}
				else if(m_datasetType == DATASET::PANOPTIC)
				{
					calibration->panoptic2point(depth_data[i], i, point);
				}
				else
					LOG(FATAL) << "Invalid dataset type";

				// Ignore floor points
				if(FLAGS_ground == false)
				{
					if(point.Y > GROUND_THRESHOLD)
						point.Invalid = true;

					// Remove floor lights
					float distFromCenter = sqrt((point.X * point.X) + (point.Z * point.Z));
					if(distFromCenter > WALL_THRESHOLD)
						point.Invalid = true;
				}

				if(point.Invalid)
				{
					m_views[index].depth_image[i] = 0;
					m_views[index].color_image[i] = no_color;
					m_views[index].binary_mask[i] = 0;
					continue;
				}

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.
				// Find min-max tight bounding box around the point cloud in updatePointCloudPlaneCulling() function.
				
				totalPoints++;

				// normal culling - also named plane culling in the code
				if (FLAGS_server_cull == CULLING::NORMAL_EXPANSION_CULLING)
				{ 
					Vector3f f_point(point.X, point.Y, point.Z);

					if(m_predFrustum.pointInFrustum2(f_point) == INSIDE)
					{
						// Adding point to the server view and select the view;
						m_views[index].depth_image[i] = depth_data[i];
						m_views[index].color_image[i] = color_data[i];
						m_views[index].binary_mask[i] = 1;
						validView = true;
						numValidPoints++;
					}
					else	
					{
						m_views[index].depth_image[i] = 0;
						m_views[index].color_image[i] = no_color;
						m_views[index].binary_mask[i] = 0;
					}
				}
				else 
				{
					// NO CULLING
					// Adding point to the server view and select the view;
					m_views[index].depth_image[i] = depth_data[i];
					m_views[index].color_image[i] = color_data[i];
					m_views[index].binary_mask[i] = 1;
					validView = true;
					numValidPoints++;
				}
			}
			else
			{
				m_views[index].depth_image[i] = 0;
				m_views[index].color_image[i] = no_color;
				m_views[index].binary_mask[i] = 0;
			}
		}
		
		m_viewSizeWithFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * numValidPoints;
		m_viewSizeWithoutFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * totalPoints;
		m_views[index].valid = validView;

		// All views are alwyas valid.
		m_views[index].valid = true;
	}
}

void MultiviewPointCloudServer::updateViewsExpansionNormalCullingUnion(const vector<Calibration *> &extCalibration, int frameID, float expansionFactor)
{
	/**
	 * @brief Bounding box of 8 meteres around the camera. 
	 * This is used to avoid finding tight bounding box.
	 * We can use incremental octree generation.
	 */

	minBound[0] = minBound[1] = minBound[2] = -4.0f;
	maxBound[0] = maxBound[1] = maxBound[2] = 4.0f;

	pVoxel->clearVoxels2();

	m_viewSizeWithFrustumInBytes = 0;
	m_viewSizeWithoutFrustumInBytes = 0;

	for (uint32_t index : device_indices)
	{
		if(!m_views[index].isInitialized())
			m_views[index].initView(nDepthFrameWidth[index], nDepthFrameHeight[index], nColorFrameWidth[index], nColorFrameHeight[index]);

		m_views[index].frameID = frameID;
		m_views[index].viewID = index;
		m_views[index].valid = false;		// Invalidate the view at the beginning

		bool validView = false;

		time_point start, end;
		uint64_t elapsed_ms;
		
		xy_table_t *xy_table_data_ = NULL;
		if(m_datasetType == DATASET::KINECT)
			xy_table_data_ = xy_tables[index];
		
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		Calibration *calibration = extCalibration[index];
		if(!calibration->bCalibrated)
			LOG(FATAL) << "Calibration not available for device " << index;

		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;

		RGB *color_data = pColorRGBX[index];
		uint16_t *depth_data = pDepth[index];

		Point3f point, temp;
		uint32_t numValidPoints = 0;
		uint32_t totalPoints = 0;
		RGB no_color;

		for (uint32_t i = 0; i < nVertices; i++)
		{
			// RGB *color = &color_data[i];
			if (depth_data[i] > 0)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Change these operations to matrix multiplication using Eigen library.
				 */

				if(m_datasetType == DATASET::KINECT)
				{
					if(isnan(xy_table_data_->x_table[i]) || isnan(xy_table_data_->y_table[i]))
						continue;
					
					calibration->kinect2point(xy_table_data_, depth_data[i], i, point);
				}
				else if(m_datasetType == DATASET::PANOPTIC)
				{
					calibration->panoptic2point(depth_data[i], i, point);
				}
				else
					LOG(FATAL) << "Invalid dataset type";

				// Ignore floor points
				if(FLAGS_ground == false)
				{
					if(point.Y > GROUND_THRESHOLD)
						point.Invalid = true;

					// Remove floor lights
					float distFromCenter = sqrt((point.X * point.X) + (point.Z * point.Z));
					if(distFromCenter > WALL_THRESHOLD)
						point.Invalid = true;
				}

				if(point.Invalid)
				{
					m_views[index].depth_image[i] = 0;
					m_views[index].color_image[i] = no_color;
					m_views[index].binary_mask[i] = 0;
					continue;
				}

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.
				// Find min-max tight bounding box around the point cloud in updatePointCloudPlaneCulling() function.
				
				totalPoints++;

				// normal culling - also named plane culling in the code
				if (FLAGS_server_cull == CULLING::NORMAL_EXPANSION_CULLING)
				{ 
					Vector3f f_point(point.X, point.Y, point.Z);

					if(m_prevFrustum.pointInFrustum2(f_point) == INSIDE || m_predFrustum.pointInFrustum2(f_point) == INSIDE)
					{
						// Adding point to the server view and select the view;
						m_views[index].depth_image[i] = depth_data[i];
						m_views[index].color_image[i] = color_data[i];
						m_views[index].binary_mask[i] = 1;
						validView = true;
						numValidPoints++;
					}
					else	
					{
						m_views[index].depth_image[i] = 0;
						m_views[index].color_image[i] = no_color;
						m_views[index].binary_mask[i] = 0;
					}
				}
				else 
				{
					// NO CULLING
					// Adding point to the server view and select the view;
					m_views[index].depth_image[i] = depth_data[i];
					m_views[index].color_image[i] = color_data[i];
					m_views[index].binary_mask[i] = 1;
					validView = true;
					numValidPoints++;
				}
			}
			else
			{
				m_views[index].depth_image[i] = 0;
				m_views[index].color_image[i] = no_color;
				m_views[index].binary_mask[i] = 0;
			}
		}
		
		m_viewSizeWithFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * numValidPoints;
		m_viewSizeWithoutFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * totalPoints;
		m_views[index].valid = validView;
	}
}

/**
 * @brief Increment the frustum by expanding to voxels around the frustum.
 * @deprecated This function will be replaced by Kalman filter based frustum selection.
 * @param extCalibration 
 * @param frameID 
 */
void MultiviewPointCloudServer::updateViewsFromVoxelIncremental(const vector<Calibration *> &extCalibration, int frameID)
{
	LOG(INFO) << "Updating views from voxel";

	/**
	 * @brief Bounding box of 8 meteres around the camera. 
	 * This is used to avoid finding tight bounding box.
	 * We can use incremental octree generation.
	 */

	minBound[0] = minBound[1] = minBound[2] = -4.0f;
	maxBound[0] = maxBound[1] = maxBound[2] = 4.0f;

	pVoxel->clearVoxels2();

	vector<vector<uint32_t>> valid_points_idx(device_indices.size());
	uint32_t num_valid_points = 0;
	for (uint32_t index : device_indices)
	{
		if(!m_views[index].isInitialized())
			m_views[index].initView(nDepthFrameWidth[index], nDepthFrameHeight[index], nColorFrameWidth[index], nColorFrameHeight[index]);

		m_views[index].frameID = frameID;
		m_views[index].viewID = index;
		m_views[index].valid = false;		// Invalidate the view at the beginning

		xy_table_t *xy_table_data_ = xy_tables[index];
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];
		Calibration *calibration = extCalibration[index];
		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;

		Point3f *point_data = pPoint[index];
		RGB *color_data = pColorRGBX[index];
		uint16_t *depth_data = pDepth[index];

		Point3f point, temp;
		RGB invalid_color;
		Point3f invalid_point(0.0f, 0.0f, 0.0f, true);

		for (uint32_t i = 0; i < nVertices; i++)
		{
			RGB *color = &color_data[i];
			if (depth_data[i] > 0 && !isnan(xy_table_data_->x_table[i]) && !isnan(xy_table_data_->y_table[i]) && calibration->bCalibrated)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Change these operations to matrix multiplication using Eigen library.
				 */
				
				// First translate then rotate
				temp.X = xy_table_data_->x_table[i] * (float)depth_data[i]/1000.0f + T[0];
				temp.Y = xy_table_data_->y_table[i] * (float)depth_data[i]/1000.0f + T[1];
				temp.Z = (float)depth_data[i]/1000.0f + T[2];

				point_data[i].X = temp.X * R[0][0] + temp.Y * R[0][1] + temp.Z * R[0][2];
				point_data[i].Y = temp.X * R[1][0] + temp.Y * R[1][1] + temp.Z * R[1][2];
				point_data[i].Z = temp.X * R[2][0] + temp.Y * R[2][1] + temp.Z * R[2][2];
				point_data[i].Invalid = false;

				valid_points_idx[index].push_back(i);
			}
			else
				point_data[i] = invalid_point;
			
			// Clear previous views
			m_views[index].depth_image[i] = 0;
			m_views[index].color_image[i] = invalid_color;
		}
		num_valid_points += valid_points_idx[index].size();
	}

	m_viewSizeWithFrustumInBytes = 0;

	vector<vector<uint32_t>> points_out_frustum_idx(device_indices.size());
	vector<vector<uint32_t>> points_in_frustum_idx(device_indices.size());
	uint32_t num_points_in_frustrum = 0;
	uint32_t num_points_out_frustrum = 0;

	for (uint32_t index : device_indices)
	{
		bool validView = false;

		Point3f *point_data = pPoint[index];
		RGB *color_data = pColorRGBX[index];
		uint16_t *depth_data = pDepth[index];

		RGB invalid_color;

		for(auto idx : valid_points_idx[index])
		{
			Point3f *point = &point_data[idx];
			RGB *color = &color_data[idx];

			Vector3f temp_point(point->X, point->Y, point->Z);
			int frustum_pos = pVoxel->pointInFrustum(temp_point, &m_frustumClip);
			if(frustum_pos == INSIDE || frustum_pos == INTERSECT)
			{
				// // Adding point to the server view and select the view;
				// m_views[index].depth_image[idx] = depth_data[idx];
				// m_views[index].color_image[idx] = color_data[idx];
				// validView = true;

				points_in_frustum_idx[index].push_back(idx);
			}
			else
			{
				if(frustum_pos == OUTSIDE)
					points_out_frustum_idx[index].push_back(idx);
				
				// m_views[index].depth_image[idx] = 0;
				// m_views[index].color_image[idx] = invalid_color;
			}
		}

		num_points_in_frustrum += points_in_frustum_idx[index].size();
		num_points_out_frustrum += points_out_frustum_idx[index].size();

		// m_viewSizeWithFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * num_points_in_frustrum;
		// m_views[index].valid = validView;
	}

	uint32_t target_points_in_frustrum = (uint32_t)(num_points_in_frustrum * 1.1f);		// 10% extra points around the frustrum

	vector<vector<uint32_t>> points_out_frustum_idx_tmp(device_indices.size());
	
	/**
	 * @brief RAJRUP
	 * Problem: Slow increments in the step of one voxel in each direction may cause num_points_out_frustrum to the same in each direction.
	 * This causes the loop to run for infinity. Why is this happening? Perhaps band of uninitialized voxels around the frustum? Causing islands, so the expansion stops?
	 * How to fix this? 
	 * Also, stray points faraway from the frustum are not being removed. How to fix this? This takes longer time for the algo to converge.
	 */
	while(num_points_in_frustrum < target_points_in_frustrum && num_points_out_frustrum > 500) 
	{
		pVoxel->expandVoxelInFrustum(&m_frustumClip);

		LOG(INFO) << "Total number of points: " << num_valid_points;
		LOG(INFO) << "Number of points in frustrum: " << num_points_in_frustrum;
		LOG(INFO) << "Number of points out frustrum: " << num_points_out_frustrum;

		num_points_in_frustrum = 0;
		num_points_out_frustrum = 0;

		for (uint32_t index : device_indices)
		{
			Point3f *point_data = pPoint[index];
			for(auto idx : points_out_frustum_idx[index])
			{
				Point3f *point = &point_data[idx];

				Vector3f temp_point(point->X, point->Y, point->Z);
				int frustum_pos = pVoxel->pointInFrustum(temp_point, &m_frustumClip);

				// Points outside min and max of the voxel are not considered
				if(frustum_pos == INSIDE || frustum_pos == INTERSECT)
				{
					points_in_frustum_idx[index].push_back(idx);
				}
				else if(frustum_pos == OUTSIDE)
				{
					points_out_frustum_idx_tmp[index].push_back(idx);
				}
			}
			
			// points_out_frustum_idx[index].clear();
			points_out_frustum_idx[index] = points_out_frustum_idx_tmp[index];
			points_out_frustum_idx_tmp[index].clear();

			num_points_in_frustrum += points_in_frustum_idx[index].size();
			num_points_out_frustrum += points_out_frustum_idx[index].size();
		}
	}

	for (uint32_t index : device_indices)
	{
		RGB *color_data = pColorRGBX[index];
		uint16_t *depth_data = pDepth[index];

		bool validView = false;

		for(auto idx : points_in_frustum_idx[index])
		{
			// Adding point to the server view and select the view;
			m_views[index].depth_image[idx] = depth_data[idx];
			m_views[index].color_image[idx] = color_data[idx];
			validView = true;
		}

		m_viewSizeWithFrustumInBytes += (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * num_points_in_frustrum;
		m_views[index].valid = validView;
	}
}

// /**
//  * @brief Calculate binary mask from depth images in the views
//  * 
//  * @param binaryMasks 
//  */
// void MultiviewPointCloudServer::calculateBinaryMask(vector<cv::Mat> &binaryMasks)
// {
// 	for (uint32_t index : device_indices)
// 	{
// 		View *view = &m_views[index];
// 		binaryMasks[index] = cv::Mat::zeros(view->depthHeight, view->depthWidth, CV_8UC1);
		
// 		if(!view->valid)
// 			continue;
		
// 		for(int i = 0; i < view->depthHeight; i++)
// 		{
// 			for(int j = 0; j < view->depthWidth; j++)
// 			{
// 				if(view->depth_image[i * view->depthWidth + j] > 0)
// 				{
// 					binaryMasks[index].at<uint8_t>(i, j) = 255;
// 				}
// 			}
// 		}
// 	}
// }

void MultiviewPointCloudServer::storeViews_standalone(const string &work_dir)
{
	LOG(INFO) << "Storing views to " << work_dir;
	LOG(INFO) << "Number of views: " << m_views.size();
	string color_dir = FORMAT(work_dir << "/" << FLAGS_save_views_subfolder << "/color/");
	string depth_dir = FORMAT(work_dir << "/" << FLAGS_save_views_subfolder << "/depth/");
	create_folder(color_dir);
	create_folder(depth_dir);

	for(int i = 0; i < m_views.size(); i++)
	{
		View *view = &m_views[i];
		CHECK(view->isValid()) << "For frame id " << view->frameID << ", the view id " << view->viewID << " is not valid";

		cv::Mat color_image_cv2 = color_to_opencv(view->color_image, view->colorWidth, view->colorHeight);
		cv::Mat depth_image_cv2 = depth_to_opencv(view->depth_image, view->depthWidth, view->depthHeight);

		int view_id = view->viewID;
		int frame_id = view->frameID;

		CHECK(frame_id >=0) << "Frame is not valid for the view id " << view_id;

		string color_image_path = FORMAT(color_dir << frame_id << "_color_" << view_id << ".png");
		string depth_image_path = FORMAT(depth_dir << frame_id << "_depth_" << view_id << ".png");

		CHECK(cv::imwrite(color_image_path, color_image_cv2)) << "Failed to write color image: " << color_image_path;
		CHECK(cv::imwrite(depth_image_path, depth_image_cv2)) << "Failed to write depth image: " << depth_image_path;
	}
}

void MultiviewPointCloudServer::storeViews(const string &work_dir)
{
	LOG(INFO) << "Storing views to " << work_dir;
	LOG(INFO) << "Number of views: " << m_views.size();
	for(int i = 0; i < m_views.size(); i++)
	{
		View *view = &m_views[i];
		if (!view->isValid())
		{
			LOG(WARNING) << "WARNING: For frame id " << view->frameID << ", the view id " << view->viewID << " is not valid";
			continue;
		}
		
		cv::Mat color_image_cv2 = color_to_opencv(view->color_image, view->colorWidth, view->colorHeight);
		cv::Mat depth_image_cv2 = depth_to_opencv(view->depth_image, view->depthWidth, view->depthHeight);
		cv::Mat binary_mask_cv2 = binary_mask_to_opencv(view->binary_mask, view->depthWidth, view->depthHeight);

		color_images.push_back(color_image_cv2);
		depth_images.push_back(depth_image_cv2);
		binary_masks.push_back(binary_mask_cv2);

		int view_id = view->viewID;
		int frame_id = view->frameID;

		frame_info.push_back(make_pair(frame_id, view_id));

		if (frame_id < 0)
		{
			LOG(WARNING) << "WARNING: Frame is not valid for the view id " << view_id;
			continue;
		}

		const int queue_size = m_views.size() * 1;
		if(color_images.size() >= queue_size)
		{
			for (int j = 0; j < queue_size; j++)
			{
				int frame_id = frame_info[j].first;
				int view_id = frame_info[j].second;
				string out_dir = FORMAT(work_dir << "test_views/" << frame_id << "/");

				// Creating a directory
				if(!fs::exists(out_dir))
				{
					if(fs::create_directories(out_dir))
						LOG(INFO) << "Directory created";
					else
					{
						LOG(ERROR) << "ERROR: " << "Directory creation failed";
						return;
					}
				}

				string color_image_path = FORMAT(out_dir << "color_" << view_id << ".jpg");
				string depth_image_path = FORMAT(out_dir << "depth_" << view_id << ".jpg");
				string binary_mask_path = FORMAT(out_dir << "binary_mask_" << view_id << ".jpg");

				if(!cv::imwrite(color_image_path, color_images[j], compression_params))
				{
					LOG(ERROR) << "Failed to write color image: " << color_image_path;
					continue;
				}
				if(!cv::imwrite(depth_image_path, depth_images[j], compression_params))
				{
					LOG(ERROR) << "Failed to write depth image: " << depth_image_path;
					continue;
				}
				if(!cv::imwrite(binary_mask_path, binary_masks[j], compression_params))
				{
					cout << "Failed to write binary mask: " << binary_mask_path << endl;
					continue;
				}
			}
			color_images.clear();
			depth_images.clear();
			binary_masks.clear();
			frame_info.clear();
		}
	}
}

void MultiviewPointCloudServer::addQR2Views(const string &qr_folder, int frameID)
{
	// Adding QR code to 1st view
	cv::Mat qr_code = cv::imread(FORMAT(qr_folder << frameID << "_qrcode.png"), cv::IMREAD_GRAYSCALE);
	cv::Mat img_view0 = color_to_opencv(m_views[0].color_image, m_views[0].colorWidth, m_views[0].colorHeight);

	int offset_x, offset_y;
	offset_x = 20;
	offset_y = 20;
	for(int i = 0; i < qr_code.rows; i++)
	{
		for(int j = 0; j < qr_code.cols; j++)
		{
			assert(i+offset_y < img_view0.rows && j+offset_x < img_view0.cols);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[0] = qr_code.at<uchar>(i, j);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[1] = qr_code.at<uchar>(i, j);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[2] = qr_code.at<uchar>(i, j);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[3] = 255;
		}
	}
	memcpy(m_views[0].color_image, &img_view0.ptr<cv::Vec4b>(0)[0], img_view0.rows * img_view0.cols * sizeof(cv::Vec4b));

	// For debugging, remove it later
	// if(frameID % 40 == 0)
	// {
	// 	img_view0 = color_to_opencv(m_views[0].color_image, m_views[0].colorWidth, m_views[0].colorHeight);
	// 	cv::imwrite(FORMAT("/home/lei/data/pipeline/server_tiled/" << frameID << "_color_withqr_0.png"), img_view0);

	// 	cv::Mat qrcode_extract = img_view0(cv::Range(offset_x, offset_x+82), cv::Range(offset_y, offset_y+82));

	// 	vector<cv::Mat> channels(img_view0.channels());
	// 	cv::split(qrcode_extract, channels);

	// 	for(int i=0; i<channels.size(); i++)
	// 	{
	// 		string channel_path = FORMAT("/home/lei/data/pipeline/server_tiled/" << frameID << "_qrextract" << i << ".png");
	// 		if(!cv::imwrite(channel_path, channels[i]))
	// 			LOG(ERROR) << "Failed to write color image: " << channel_path;
	// 		else 
	// 			LOG(INFO) << "Color image saved to " << channel_path;
	// 	}
	// }
}

/**
 * @brief Calculates size of current point cloud frame.
 *
 */
void MultiviewPointCloudServer::calcViewSizeInBytes()
{
	m_viewSizeWithFrustumInBytes = (uint32_t)(sizeof(uint16_t) + sizeof(RGB)) * numValidPointsInViews();
}

/**
 * @brief Number of valid points in the selected views.
 * Depth value > 0 is considered as valid point.
 * @return uint32_t 
 */
uint32_t MultiviewPointCloudServer::numValidPointsInViews()
{
	uint32_t numValidPoints = 0;
	for (uint32_t i = 0; i < m_views.size(); i++)
	{
		if (!m_views[i].isValid())
			continue;

		int viewID = m_views[i].viewID;
		uint16_t *depth_data = m_views[i].depth_image;
		uint32_t nVertices = getVerticesByIndex(viewID);
		for (uint32_t j = 0; j < nVertices; j++)
		{
			if (depth_data[j] > 0)
				numValidPoints++;
		}
	}
	return numValidPoints;
}

/**
 * @brief Number of vertices in the Point Cloud frame for a given device.
 * Assuming nColorFrameWidth = nDepthFrameWidth and nColorFrameHeight = nDepthFrameHeight.
 * @param index of the device
 * @return uint32_t 
 */
uint32_t MultiviewPointCloudServer::getVerticesByIndex(uint32_t index)
{
	// Rajrup: To Do: Remove asserts later
	assert(nColorFrameHeight[index] == nDepthFrameHeight[index]);
	assert(nColorFrameWidth[index] == nDepthFrameWidth[index]);
	return nColorFrameHeight[index] * nColorFrameWidth[index];
}

void MultiviewPointCloudServer::setUserData(const UserData &userData)
{
	if(m_mapUserData.find(userData.frameID) == m_mapUserData.end())
		m_mapUserData[userData.frameID] = userData;
	else
		LOG(ERROR) << "Frame ID " << userData.frameID << " already exists in the map";
}

void MultiviewPointCloudServer::setFrustum(const Frustum &frustum)
{
	m_frustum = frustum;
}

void MultiviewPointCloudServer::setFrustum(const Frustum &prev_frustum, const Frustum &pred_frustum)
{
	m_prevFrustum = prev_frustum;
	m_predFrustum = pred_frustum;
}

void MultiviewPointCloudServer::setFrustumClip(const FrustumClip &frustumClip)
{
	m_frustumClip = frustumClip;
}

void MultiviewPointCloudServer::updatePointCloud(const vector<Calibration *> &extCalibration)
{
	PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;

	for (uint32_t index : device_indices)
	{
		goodIndices[index].clear();

		Calibration *calibration = extCalibration[index];
		if(!calibration->bCalibrated)
			LOG(FATAL) << "Calibration not available for device " << index;
		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		xy_table_t *xy_table_data_ = NULL;
		if(m_datasetType == DATASET::KINECT)
			xy_table_data_ = xy_tables[index];
		
		uint16_t *depth_data = pDepth[index];
		RGB *color_data = pColorRGBX[index];
		Point3f *vertices = pPoint[index];

		for (uint32_t i = 0; i < nVertices; i++)
		{
			// All previous vertices are invalid. Only invalidate the vertex and not the coordinates.
			Point3f &point = vertices[i];
			RGB &color = color_data[i];
			point.Invalid = true;

			if (depth_data[i] > 0)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Change these operations to matrix multiplication using Eigen library.
				 */

				if(m_datasetType == DATASET::KINECT)
				{
					if(isnan(xy_table_data_->x_table[i]) || isnan(xy_table_data_->y_table[i]))
						continue;
					
					calibration->kinect2point(xy_table_data_, depth_data[i], i, point);
				}
				else if(m_datasetType == DATASET::PANOPTIC)
				{
					calibration->panoptic2point(depth_data[i], i, point);
				}
				else
					LOG(FATAL) << "Invalid dataset type";

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.
				// Find min-max tight bounding box around the point cloud in updatePointCloudPlaneCulling() function.

				if(FLAGS_server_cull == CULLING::CLIP_CULLING)
				{
					Eigen::Vector4f f_point(point.X, point.Y, point.Z, 1.0);
					if(m_frustumClip.pointInFrustum(f_point) == INSIDE)
					{
						// good_points.push_back(point);
						// good_colors.push_back(*color);
						point.Invalid = false;
						goodIndices[index].push_back(i);
					}
				}
				else if(FLAGS_server_cull == CULLING::NORMAL_CULLING)
				{
					Vector3f f_point(point.X, point.Y, point.Z);
					if(m_frustum.pointInFrustum2(f_point) == INSIDE)
					{
						// good_points.push_back(point);
						// good_colors.push_back(*color);
						point.Invalid = false;
						goodIndices[index].push_back(i);
					}
				}
				else if(FLAGS_server_cull == CULLING::NO_CULLING)
				{
					// good_points.push_back(point);
					// good_colors.push_back(*color);
					point.Invalid = false;
					goodIndices[index].push_back(i);
				}
				else
					LOG(FATAL) << "Invalid culling method";
			}
		}
	}
}

void MultiviewPointCloudServer::updatePointCloudFromVoxel(const vector<Calibration *> &extCalibration)
{
	PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;

	/**
	 * @brief Bounding box of 8 meteres around the camera. 
	 * This is used to avoid finding tight bounding box.
	 * We can use incremental octree generation.
	 */

	minBound[0] = minBound[1] = minBound[2] = -4.0f;
	maxBound[0] = maxBound[1] = maxBound[2] = 4.0f;

	pVoxel->clearVoxels2();

	vector<RGB> good_colors;
	vector<Point3f> good_points;
	
	for (uint32_t index : device_indices)
	{
		goodIndices[index].clear();

		Calibration *calibration = extCalibration[index];
		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;

		xy_table_t *xy_table_data_ = xy_tables[index];
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		uint16_t *depth_data = pDepth[index];
		RGB *color_data = pColorRGBX[index];
		Point3f *vertices = pPoint[index];
		Point3f temp;

		for (uint32_t i = 0; i < nVertices; i++)
		{
			// All previous vertices are invalid. Only onvalidate the vertex and not the coordinates.
			Point3f *point = &vertices[i];
			RGB *color = &color_data[i];

			if (depth_data[i] > 0 && !isnan(xy_table_data_->x_table[i]) && !isnan(xy_table_data_->y_table[i]) && calibration->bCalibrated)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Change these operations to matrix multiplication using Eigen library.
				 */
				
				// First translate then rotate
				temp.X = xy_table_data_->x_table[i] * (float)depth_data[i]/1000.0f + T[0];
				temp.Y = xy_table_data_->y_table[i] * (float)depth_data[i]/1000.0f + T[1];
				temp.Z = (float)depth_data[i]/1000.0f + T[2];

				point->X = static_cast<short>((temp.X * R[0][0] + temp.Y * R[0][1] + temp.Z * R[0][2])*1000.0f);
				point->Y = static_cast<short>((temp.X * R[1][0] + temp.Y * R[1][1] + temp.Z * R[1][2])*1000.0f);
				point->Z = static_cast<short>((temp.X * R[2][0] + temp.Y * R[2][1] + temp.Z * R[2][2])*1000.0f);

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.
				// Find min-max tight bounding box around the point cloud in updatePointCloudPlaneCulling() function.

				Vector3f temp_point(point->X/1000.0f, point->Y/1000.0f, point->Z/1000.0f);
				int frustum_pos = pVoxel->pointInFrustum(temp_point, &m_frustumClip);
				if(frustum_pos == INSIDE || frustum_pos == INTERSECT)
				{
					// good_points.push_back(point);
					// good_colors.push_back(*color);
					goodIndices[index].push_back(i);
				}
			}
		}
	}
}

int MultiviewPointCloudServer::getNumPoints()
{
	int numPoints = 0;
	for(int index : device_indices)
		numPoints += goodIndices[index].size();
	return numPoints;
}

/**
 * @brief Serialize the point cloud, if required compress it.
 * 
 * @param size size of the serialized point cloud in bytes
 * @param compressionLevel compression level for the point cloud. -1 for no compression.
 * @return const uint8_t * Serialized or compressed point cloud. Release this buffer after use.
 */
uint8_t *MultiviewPointCloudServer::compressPointCloudZstd(int &size, int compressionLevel)
{
	int nPoints = getNumPoints();

	// Intital sizeof(int) holds nPoints value.
	// R, G, B are chars - 1 byte each and X, Y, Z are floats - 4 bytes each.
	int bSize = sizeof(int) + nPoints * (3 * sizeof(float) + 3 * sizeof(uint8_t));
	uint8_t *buffer = new uint8_t[bSize];
	int pos = 0;

	memcpy(buffer + pos, &nPoints, sizeof(int));
	pos += sizeof(int);

	for(uint32_t index : device_indices)
	{
		Point3f *points = pPoint[index];
		RGB *colors = pColorRGBX[index];

		// struct RGB has rgbReserved field which is not sent.
		for(int i : goodIndices[index])
		{
			memcpy(buffer + pos, &points[i].X, sizeof(float));
			pos += sizeof(float);
			memcpy(buffer + pos, &points[i].Y, sizeof(float));
			pos += sizeof(float);
			memcpy(buffer + pos, &points[i].Z, sizeof(float));
			pos += sizeof(float);

			buffer[pos] = (uint8_t)colors[i].rgbRed;
			pos += 1;
			buffer[pos] = (uint8_t)colors[i].rgbGreen;
			pos += 1;
			buffer[pos] = (uint8_t)colors[i].rgbBlue;
			pos += 1;
		}
	}

	size = bSize;
	if(compressionLevel >= 0)
	{
		// *2, because according to zstd documentation, increasing the size of the output buffer above a 
		// bound should speed up the compression.

		int cBuffSize = ZSTD_compressBound(size) * 2;	
		uint8_t *compressedBuffer = new uint8_t[cBuffSize];
		
		/** Check compression levels
		 * @brief typedef enum { ZSTD_fast=1,
               ZSTD_dfast=2,
               ZSTD_greedy=3,
               ZSTD_lazy=4,
               ZSTD_lazy2=5,
               ZSTD_btlazy2=6,
               ZSTD_btopt=7,
               ZSTD_btultra=8,
               ZSTD_btultra2=9
               note : new strategies _might_ be added in the future.
                         Only the order (from fast to strong) is guaranteed
				} ZSTD_strategy;
		 */
		StopWatch sw;
		int cSize = ZSTD_compress(compressedBuffer, cBuffSize, buffer, bSize, compressionLevel);
		size = cSize;
		LOG(INFO) << "ZSTD - Encoded point cloud size " << size << " bytes";
		LOG(INFO) << "ZSTD - Encoded point cloud in " << sw.ElapsedMs() << " ms";

		delete[] buffer;
		buffer = compressedBuffer;
	}
	return buffer;
}

/**
 * @brief Compress point cloud using Draco.
 * 
 * @param size size of the compressed point cloud in bytes
 * @param cl compression level
 * @param qp quantization geometric position
 * @param qt quantization geometric texture
 * @return const char* compressed point cloud. Release this buffer after use.
 */
uint8_t *MultiviewPointCloudServer::compressPointCloudDraco(int &size, int cl, int qp)
{

	if(cl < 0 || cl > 10)
		LOG(FATAL) << "Invalid compression level " << cl;
	if(qp < 0 || qp > 30)
		LOG(FATAL) << "Invalid quantization position " << qp;

	// Convert compression level to speed (that 0 = slowest, 10 = fastest).
	const int speed = 10 - cl;

	mDracoEncoder.Reset();

	// Setup encoder options.
	if (qp > 0)
		mDracoEncoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, qp);
	// if (qt > 0)
	// 	mDracoEncoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD, qt);
	mDracoEncoder.SetSpeedOptions(speed, speed);

	// Convert Point3f and RGB to draco::PointCloud.
	int nPoints = getNumPoints();

	//Create Point Cloud
	draco::PointCloudBuilder pc_builder;
	pc_builder.Start(nPoints);

	// Add position and color attribute.
	const int pos_att_id = pc_builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
	const int color_att_id = pc_builder.AddAttribute(draco::GeometryAttribute::COLOR, 3, draco::DT_UINT8);

	// Add points to the point cloud.
	int j = 0;
	for(uint32_t index : device_indices)
	{
		Point3f *points = pPoint[index];
		RGB *colors = pColorRGBX[index];

		for(int i : goodIndices[index])
		{
			vector<float> point = {points[i].X, points[i].Y, points[i].Z};
			vector<uint8_t> color = {colors[i].rgbRed, colors[i].rgbGreen, colors[i].rgbBlue};
			pc_builder.SetAttributeValueForPoint(pos_att_id, draco::PointIndex(j), point.data());
			pc_builder.SetAttributeValueForPoint(color_att_id, draco::PointIndex(j), color.data());
			j++;
		}
	}

	// Finish the point cloud.
	std::unique_ptr<draco::PointCloud> draco_ptcl = pc_builder.Finalize(false);

	draco::CycleTimer timer;
	// Encode the geometry.
	draco::EncoderBuffer buffer;
	timer.Start();
	const draco::Status status = mDracoEncoder.EncodePointCloudToBuffer(*draco_ptcl, &buffer);
	if (!status.ok())
		LOG(FATAL) << "Failed to encode the point cloud using Draco.\n" << status.error_msg();

	timer.Stop();

	size = buffer.size();	// Compressed size in bytes
	uint8_t *compressedBuffer = new uint8_t[size];
	memcpy(compressedBuffer, buffer.data(), size);

	// LOG(INFO) << "DRACO - Encoded point cloud size " << buffer.size() << " bytes";
	// LOG(INFO) << "DRACO - Encoded point cloud in " << timer.GetInMs() << " ms";
	return compressedBuffer;
}