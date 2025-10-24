#include "MultiviewPointCloud.h"
#include <algorithm>
#include <boost/filesystem/fstream.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud.h>
// #include <pcl/octree/octree_search.h>
#include "k4autils.h"
#include "utils.h"
#include "consts.h"
#include "filter.h"
#include "k4aconsts.h"

namespace fs = boost::filesystem;

MultiviewPointCloud::MultiviewPointCloud(const vector<uint32_t> &indices, const vector<string> &deviceSerials)
{
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

	pPointCloud.resize(device_indices.size(), NULL);
	pDepth.resize(device_indices.size(), NULL);
	pColorRGBX.resize(device_indices.size(), NULL);
	ePointCloud.resize(device_indices.size(), NULL);

	pointCloudImage.resize(device_indices.size());
	transformedDepthImage.resize(device_indices.size());

	m_vLastFrameVertices.resize(device_indices.size());
	m_vLastFrameRGB.resize(device_indices.size());

	m_vLastFrameVertices_f.resize(device_indices.size());
	m_vLastFrameRGB_f.resize(device_indices.size());

	m_views.resize(device_indices.size());

	m_bFilter = false;
	m_nFilterNeighbors = 10;
	m_fFilterThreshold = 0.01f;

	// Rajrup: Default values taken from LiveScan3D
	m_vBounds.push_back(-0.5);
	m_vBounds.push_back(-0.5);
	m_vBounds.push_back(-0.5);
	m_vBounds.push_back(0.5);
	m_vBounds.push_back(0.5);
	m_vBounds.push_back(0.5);

	minBound[0] = minBound[1] = minBound[2] = 0.0;
	maxBound[0] = maxBound[1] = maxBound[2] = 0.0;

	pclCloud = NULL;
	p_octree = NULL;

	windowName = "";
	pViewer = NULL;
}

MultiviewPointCloud::~MultiviewPointCloud()
{
	nColorFrameHeight.clear();
	nColorFrameWidth.clear();

	nDepthFrameHeight.clear();
	nDepthFrameWidth.clear();

	for (auto i : device_indices)
	{
		colorImage[i].reset();
		depthImage[i].reset();

		device_transformationColorDownscaled[i].destroy();
		
		delete [] xy_tables[i]->x_table;
		delete [] xy_tables[i]->y_table;
		delete xy_tables[i];

		delete [] pPointCloud[i];
		delete [] pDepth[i];
		delete [] pColorRGBX[i];
		delete [] ePointCloud[i];

		pointCloudImage[i].reset();
		transformedDepthImage[i].reset();
	}
	colorImage.clear();
	depthImage.clear();
	device_transformationColorDownscaled.clear();
	device_calibrationColorDownscaled.clear();
	xy_tables.clear();
	pointCloudImage.clear();
	transformedDepthImage.clear();

	m_vLastFrameVertices.clear();
	m_vLastFrameRGB.clear();
	m_vLastFrameVertices_f.clear();
	m_vLastFrameRGB_f.clear();
	
	m_views.clear();
	m_vBounds.clear();

	pPointCloud.clear();
	pDepth.clear();
	pColorRGBX.clear();
	ePointCloud.clear();

	device_indices.clear();
	device_serial_numbers.clear();

	if (pViewer != NULL)
	{
		delete pViewer;
		pViewer = NULL;
	}

	if(p_octree != NULL)
	{
		delete p_octree;
		p_octree = NULL;
	}

}

void MultiviewPointCloud::initViewer()
{
	windowName = "Point Cloud Viewer";
	pViewer = new PointCloudViewer(windowName);

	// Initialize the frustum when the viewer is created
	updateFrustumClip(pViewer->m_camProjViewMat);

}

void MultiviewPointCloud::setDeviceIndices(const vector<uint32_t> &indices)
{
	device_indices = indices;
}

void MultiviewPointCloud::setDeviceSerials(const vector<string> &deviceSerials)
{
	device_serial_numbers = deviceSerials;
} 

void MultiviewPointCloud::updateFrustum(CamInt &camInt, CamView &camView)
{
	cout << "**** Updating frustum ****" << endl;
	m_frustum.setCamInternals(camInt);
	m_frustum.setCamPlanes(camView);
}

void MultiviewPointCloud::updateFrustumClip(Matrix4d &camProjViewMat)
{
	cout << "**** Updating frustum clip ****" << endl;
	m_frustumClip.setClipPlanes(camProjViewMat);
}

Frustum *MultiviewPointCloud::getFrustum()
{
	return &m_frustum;
}

FrustumClip *MultiviewPointCloud::getFrustumClip()
{
	return &m_frustumClip;
}

void MultiviewPointCloud::setFrustum(const Frustum &frustum)
{
	m_frustum = frustum;
}

void MultiviewPointCloud::setFrustumClip(const FrustumClip &frustumClip)
{
	m_frustumClip = frustumClip;
}

bool MultiviewPointCloud::loadCalibrationColorDownscaled(const string &path, const uint32_t index, k4a_calibration_t &calibrationColorDownscaled)
{
	fs::path bin_filepath{FORMAT(path << CALIBRATION_INT_FILENAME << "_raw_" << device_serial_numbers[index] << ".bin")};
	fs::ifstream bin_file;
	bin_file.open(bin_filepath, ios::binary);

	if (!bin_file.is_open())
	{	
		cout << "Failed to open file: " << bin_filepath << endl;
		return false;
	}

	vector<uint8_t> blob;
	boost::archive::binary_iarchive iar(bin_file);
	iar >> blob;

	bin_file.close();
	cout << "Raw Intrinsic Calibration for device " << device_serial_numbers[index] << " loaded from " << bin_filepath << endl;

	k4a_result_t result = k4a_calibration_get_from_raw(reinterpret_cast<char *>(&blob[0]), blob.size(), DEFAULT_DEPTH_MODE, DEFAULT_COLOR_MODE, &calibrationColorDownscaled);

	if (K4A_RESULT_SUCCEEDED != result)
	{
		cout << "Failed to load calibration from raw calibration blob for index: " << index << endl;
		return false;
	}

	fs::path filepath{FORMAT(path << CALIBRATION_INT_FILENAME << "_ColorDownscaled_" << device_serial_numbers[index] << ".txt")};
	fs::ifstream file;
	file.open(filepath, ios::in);
	if (!file.is_open())
	{
		cout << "Failed to load Calibration file: " << filepath << endl;
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
				cout << "Unknown calibration parameter: " << calib_param << endl;
				return false;
		}
	}

	file.close();
	cout << "Intrinsic Calibration for device " << device_serial_numbers[index] << " loaded from " << filepath << endl;

	return true;
}
/**
 * Rajrup: Read intrinsic camera parameters file
 * To Do: Only supports transformation colored downscaled
 */
bool MultiviewPointCloud::loadTransformations(const string &path)
{
	for (uint32_t index : device_indices)
	{
		k4a_calibration_t calibration;
		if(!loadCalibrationColorDownscaled(path, index, calibration))
		{
			cout << "Failed to load Calibration file for device index: " << index << endl;
			return false;
		}
		device_calibrationColorDownscaled[index] = calibration;
		device_transformationColorDownscaled[index] = k4a::transformation(calibration);
	}
	initXYTables(K4A_CALIBRATION_TYPE_COLOR); // Populate the lookup tables for x,y,z when calibration is loaded
	return true;
}

/**
 * RAJRUP: Inspiration from func transformation_init_xy_tables() in transformation.c
 * Also check fastpointcloud example in Kinect SDK
 */
void MultiviewPointCloud::initXYTables(const k4a_calibration_type_t camera)
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
				cout << "Unknown Camera Type: " << camera << endl;
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

vector<xy_table_t *> MultiviewPointCloud::getXYTables()
{
	return xy_tables;
}

void MultiviewPointCloud::setXYTables(const vector<xy_table_t *> &tables)
{
	assert(tables.size() == device_indices.size());

	int w, h;
	for (uint32_t index : device_indices)
	{
		xy_tables[index] = tables[index];
	}
}

void MultiviewPointCloud::setImages(vector<cv::Mat> &cImgs, vector<cv::Mat> &dImgs)
{
	assert(cImgs.size() == dImgs.size());
	assert(cImgs.size() == device_indices.size());

	for(uint32_t i = 0; i < cImgs.size(); i++)
	{
		nColorFrameHeight[i] = cImgs[i].rows;
		nColorFrameWidth[i] = cImgs[i].cols;
		if(!colorImage[i].is_valid())
		{
			pColorRGBX[i] = new RGB[nColorFrameWidth[i] * nColorFrameHeight[i]];
			pPointCloud[i] = new Point3f[nColorFrameWidth[i] * nColorFrameHeight[i]];
			// colorImage[i] = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, cImgs[i].cols, cImgs[i].rows, cImgs[i].cols * 4 * (int)sizeof(uint8_t));
		}
		colorImage[i] = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, cImgs[i].cols, cImgs[i].rows, cImgs[i].cols * 4 * (int)sizeof(uint8_t));

		nDepthFrameHeight[i] = dImgs[i].rows;
		nDepthFrameWidth[i] = dImgs[i].cols;
		if(!depthImage[i].is_valid())
		{
			// depthImage[i] = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, dImgs[i].cols, dImgs[i].rows, dImgs[i].cols * (int)sizeof(uint16_t));
		}
		depthImage[i] = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, dImgs[i].cols, dImgs[i].rows, dImgs[i].cols * (int)sizeof(uint16_t));

		memcpy(colorImage[i].get_buffer(), &cImgs[i].ptr<cv::Vec4b>(0)[0], cImgs[i].rows * cImgs[i].cols * sizeof(cv::Vec4b));
		memcpy(depthImage[i].get_buffer(), &dImgs[i].ptr<uint16_t>(0)[0], dImgs[i].rows * dImgs[i].cols * sizeof(uint16_t));

		memcpy(pColorRGBX[i], colorImage[i].get_buffer(), nColorFrameWidth[i] * nColorFrameHeight[i] * sizeof(RGB));
	}

	// DEBUG: Print the size of the color and depth images
	// if(cImgs.size() > 0 && dImgs.size() > 0)
	// {
	// 	cout << "Resolution of Color Image: (" << nColorFrameHeight[0] << ", " << nColorFrameWidth[0] << ")" << endl;
	// 	cout << "Resolution of Depth Image: (" << nDepthFrameHeight[0] << ", " << nDepthFrameWidth[0] << ")" << endl;
	// }
}

void MultiviewPointCloud::setViews(const vector<View *> &views)
{
	// Invalidate the previous views
	for(auto index : device_indices)
		m_views[index].valid = false;

	// Set the new views.
	for(auto view : views)
	{
		if(view->viewID < 0 || !view->isValid())
		{
			cout << "Skip invalid view with ID: " << view->viewID << endl;
			continue;
		}

		uint32_t viewID = view->viewID;

		// Small vector, so linear search is fine.
		if (find(device_indices.begin(), device_indices.end(), viewID) == device_indices.end())
		{
			cout << "Skipping... View " << viewID << " is not a valid device viewID." << endl;
			continue;
		}
		
		m_views[viewID] = *view;
	}
}

void MultiviewPointCloud::setImagesFromViews()
{
	for(uint32_t i = 0; i < m_views.size(); i++)
	{
		int viewID = m_views[i].viewID;
		// Only set RGB+Depth for valid views, other data structure (for invalid views) will have stale point cloud.
		if(viewID < 0 || !m_views[i].isValid())
		{
			cout << "Skip invalid view with ID: " << viewID << endl;
			continue;
		}

		nColorFrameHeight[viewID] = m_views[i].colorHeight;
		nColorFrameWidth[viewID] = m_views[i].colorWidth;
		colorImage[viewID] = m_views[i].color_image;
		if(pColorRGBX[viewID] == NULL)
		{
			pColorRGBX[viewID] = new RGB[nColorFrameWidth[viewID] * nColorFrameHeight[viewID]];
			pPointCloud[viewID] = new Point3f[nColorFrameWidth[viewID] * nColorFrameHeight[viewID]];
		}

		nDepthFrameHeight[viewID] = m_views[i].depthHeight;
		nDepthFrameWidth[viewID] = m_views[i].depthWidth;
		depthImage[viewID] = m_views[i].depth_image;
		if(pDepth[viewID] == NULL)
		{
			pDepth[viewID] = new uint16_t[nDepthFrameHeight[viewID] * nDepthFrameWidth[viewID]];
		}

		memcpy(pColorRGBX[viewID], colorImage[viewID].get_buffer(), nColorFrameWidth[viewID] * nColorFrameHeight[viewID] * sizeof(RGB));
		memcpy(pDepth[viewID], depthImage[viewID].get_buffer(), nDepthFrameHeight[viewID] * nDepthFrameWidth[viewID] * sizeof(uint16_t));
	}

	// DEBUG: Print the size of the color and depth images
	if(m_views.size() > 0)
	{
		cout << "Resolution of Color Image: (" << nColorFrameHeight[0] << ", " << nColorFrameWidth[0] << ")" << endl;
		cout << "Resolution of Depth Image: (" << nDepthFrameHeight[0] << ", " << nDepthFrameWidth[0] << ")" << endl;
	}
}

void MultiviewPointCloud::storeViews(const string &work_dir)
{
	cout << "Storing views to " << work_dir << endl;
	cout << "Number of views: " << m_views.size() << endl;
	for(int i = 0; i < m_views.size(); i++)
	{
		View *view = &m_views[i];
		if (!view->isValid())
		{
			cout << "WARNING: For frame id " << view->frameID << ", the view id " << view->viewID << " is not valid" << endl;
			continue;
		}
		
		cv::Mat color_image_cv2 = color_to_opencv(view->color_image);
		cv::Mat depth_image_cv2 = depth_to_opencv(view->depth_image);

		int view_id = view->viewID;
		int frame_id = view->frameID;

		if (frame_id < 0)
		{
			cout << "WARNING: Frame is not valid for the view id " << view_id << endl;
			continue;
		}

		string out_dir = FORMAT(work_dir << "views/" << frame_id << "/");

		// Creating a directory
		if(!fs::exists(out_dir))
		{
			if(fs::create_directories(out_dir))
				cout << "Directory created" << endl;
			else
			{
				cout << "ERROR: " << "Directory creation failed" << endl;
				return;
			}
		}

		string color_image_path = FORMAT(out_dir << "color_" << view_id << ".png");
		string depth_image_path = FORMAT(out_dir << "depth_" << view_id << ".png");
		
		if(!cv::imwrite(color_image_path, color_image_cv2, compression_params))
		{
			cout << "Failed to write color image: " << color_image_path << endl;
			continue;
		}
		if(!cv::imwrite(depth_image_path, depth_image_cv2, compression_params))
		{
			cout << "Failed to write depth image: " << depth_image_path << endl;
			continue;
		}
	}
}

void MultiviewPointCloud::updateDepthPointCloudForColorFrame(uint32_t index)
{
	if (!transformedDepthImage[index].is_valid())
	{
		transformedDepthImage[index] = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, nColorFrameWidth[index], nColorFrameHeight[index], nColorFrameWidth[index] * (int)sizeof(uint16_t));
	}

	if (!pointCloudImage[index].is_valid())
	{
		pointCloudImage[index] = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, nColorFrameWidth[index], nColorFrameHeight[index], nColorFrameWidth[index] * 3 * (int)sizeof(int16_t));
	}

	time_point start, end;
	uint64_t elapsed_ms;

	start = time_now();
	device_transformationColorDownscaled[index].depth_image_to_color_camera(depthImage[index], &transformedDepthImage[index]);
	end = time_now();
	elapsed_ms = duration_ms(start, end);
	cout << "Transform Depth image to color took " << elapsed_ms << " ms" << endl;

	start = time_now();
	device_transformationColorDownscaled[index].depth_image_to_point_cloud(transformedDepthImage[index], K4A_CALIBRATION_TYPE_COLOR, &pointCloudImage[index]);
	end = time_now();
	elapsed_ms = duration_ms(start, end);
	cout << "Transform Depth Image to Point Cloud took " << elapsed_ms << " ms" << endl;
}

void MultiviewPointCloud::updateViews(const vector<Calibration *> &extCalibration, uint32_t frameID)
{
	PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;

	m_viewSizeInBytes = 0;

	for (uint32_t index : device_indices)
	{
		if (!transformedDepthImage[index].is_valid())
			transformedDepthImage[index] = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, 
																nColorFrameWidth[index], 
																nColorFrameHeight[index], 
																nColorFrameWidth[index] * 
																(int)sizeof(uint16_t));

		device_transformationColorDownscaled[index].depth_image_to_color_camera(depthImage[index], &transformedDepthImage[index]);

		// After tranformation of depth to color image, color and depth image dimensions are same.
		nDepthFrameWidth[index] = nColorFrameWidth[index];
		nDepthFrameHeight[index] = nColorFrameHeight[index];
		
		if (!m_views[index].isInitialized())
			m_views[index].initView(nDepthFrameWidth[index], nDepthFrameHeight[index], nColorFrameWidth[index], nColorFrameHeight[index]);

		/**
		 * Color Image is in BGRA format. RGB struct is aligned in BGRA format. 
		 * Depth image is transformed to color image, so the dimension should be the one for color image.
		 */
		RGB *view_color = new RGB[nColorFrameWidth[index] * nColorFrameHeight[index]];
		uint16_t *view_depth = new uint16_t[nDepthFrameWidth[index] * nDepthFrameHeight[index]]();  

		m_views[index].frameID = frameID;
		m_views[index].viewID = index;
		m_views[index].extCalibration = *extCalibration[index];
		bool validView = false;

		time_point start, end;
		uint64_t elapsed_ms;

		xy_table_t *xy_table_data_ = xy_tables[index];
		uint16_t *depth_data = (uint16_t *)(void *)transformedDepthImage[index].get_buffer();
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];
		Calibration *calibration = extCalibration[index];
		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;

		RGB *colorInDepth = pColorRGBX[index];
		Point3f *vertices = pPointCloud[index];

		Point3f point, temp;
		uint32_t numValidPoints = 0;
		for (uint32_t i = 0; i < nVertices; i++)
		{
			RGB *color = &colorInDepth[i];
			if (depth_data[i] > 0 && !isnan(xy_table_data_->x_table[i]) && !isnan(xy_table_data_->y_table[i]) && calibration->bCalibrated)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Chnage these operations to matrix multiplication using Eigen library.
				 */
				
				temp.X = xy_table_data_->x_table[i] * (float)depth_data[i]/1000.0f + T[0];
				temp.Y = xy_table_data_->y_table[i] * (float)depth_data[i]/1000.0f + T[1];
				temp.Z = (float)depth_data[i]/1000.0f + T[2];

				point.X = temp.X * R[0][0] + temp.Y * R[0][1] + temp.Z * R[0][2];
				point.Y = temp.X * R[1][0] + temp.Y * R[1][1] + temp.Z * R[1][2];
				point.Z = temp.X * R[2][0] + temp.Y * R[2][1] + temp.Z * R[2][2];

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.
				// Find min-max tight bounding box around the point cloud in updatePointCloudPlaneCulling() function.
				
				Eigen::Vector4f f_point(point.X, point.Y, point.Z, 1.0);
				if(m_frustumClip.pointInFrustum(f_point) == INSIDE)
				{
					// Adding point to the server view and select the view;
					view_depth[i] = depth_data[i];
					view_color[i] = colorInDepth[i];
					validView = true;
					numValidPoints++;
				}
			}
		}
		m_viewSizeInBytes += (uint32_t)(sizeof(uint32_t) + sizeof(RGB)) * numValidPoints;
		memcpy(m_views[index].depth_image.get_buffer(), view_depth, nDepthFrameWidth[index] * nDepthFrameHeight[index] * sizeof(uint16_t));
		memcpy(m_views[index].color_image.get_buffer(), view_color, nColorFrameWidth[index] * nColorFrameHeight[index] * sizeof(RGB));
		m_views[index].valid = validView;
	}
}

/**
 * @brief Calculates size of current point cloud frame.
 *
 */
void MultiviewPointCloud::calcViewSizeInBytes()
{
	m_viewSizeInBytes = (uint32_t)(sizeof(uint32_t) + sizeof(RGB)) * numValidPointsInViews();
}

/**
 * @brief Number of valid points in the selected views.
 * Depth value > 0 is considered as valid point.
 * @return uint32_t 
 */
uint32_t MultiviewPointCloud::numValidPointsInViews()
{
	uint32_t numValidPoints = 0;
	for (uint32_t i = 0; i < m_views.size(); i++)
	{
		if (!m_views[i].isValid())
			continue;

		int viewID = m_views[i].viewID;
		uint16_t *depth_data = (uint16_t *)(void *)m_views[i].depth_image.get_buffer();
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
uint32_t MultiviewPointCloud::getVerticesByIndex(uint32_t index)
{
	// Rajrup: To Do: Remove asserts later
	assert(nColorFrameHeight[index] == nDepthFrameHeight[index]);
	assert(nColorFrameWidth[index] == nDepthFrameWidth[index]);
	return nColorFrameHeight[index] * nColorFrameWidth[index];
}

/**
 * @brief Number of total vertices in the Point Cloud frame.
 * 
 * @param index of the device
 * @return uint32_t 
 */
uint32_t MultiviewPointCloud::getTotalVertices()
{
	uint32_t totalVertices = 0;
	for (uint32_t index : device_indices)
	{
		totalVertices += getVerticesByIndex(index);
	}
	return totalVertices;
}

/**
 * @brief Convert 3D depth image to point cloud in PCL format. It optimizes updatePointCloud2() function. 
 * Ideas from transformation/rgbz.c file in Azure Kinect SDK.
 * RAJRUP: Later I will use this function and remove updatePointCloudNoCulling() function. 
 * @param extCalibration 
 */
void MultiviewPointCloud::updatePointCloudClipCullingFromViews(const vector<Calibration *> &extCalibration)
{
	PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;
	
	int countViews = 0;
	for (uint32_t index : device_indices)
	{
		Calibration *calibration = extCalibration[index];
		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		xy_table_t *xy_table_data_ = xy_tables[index];

		uint16_t *depth_data = pDepth[index];
		RGB *colorInDepth = pColorRGBX[index];
		Point3f *vertices = pPointCloud[index];
		Point3f temp;

		/**
		 * If this view is not selected, skip it.
		 * If not calibrated, skip this view
		 * Invalidate all the vertices in our point cloud data structure
		 */
		if(m_views[index].viewID < 0 || !m_views[index].isValid() || !calibration->bCalibrated)
		{
			for (uint32_t i = 0; i < nVertices; i++)
				vertices[i].Invalid = true;
			continue;
		}
		
		countViews++;
		for (uint32_t i = 0; i < nVertices; i++)
		{	
			// All previous vertices are invalid. Only onvalidate the vertex and not the coordinates.
			Point3f *point = &vertices[i];
			RGB *color = &colorInDepth[i];
			point->Invalid = true;
			
			// No need to check if xy_table_data[i].xy.x or xy_table_data[i].xy.y is NaN. This done on server side.
			if (depth_data[i] > 0)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Change these operations to matrix multiplication using Eigen library.
				 */
				
				temp.X = xy_table_data_->x_table[i] * (float)depth_data[i]/1000.0f + T[0];
				temp.Y = xy_table_data_->y_table[i] * (float)depth_data[i]/1000.0f + T[1];
				temp.Z = (float)depth_data[i]/1000.0f + T[2];

				point->X = temp.X * R[0][0] + temp.Y * R[0][1] + temp.Z * R[0][2];
				point->Y = temp.X * R[1][0] + temp.Y * R[1][1] + temp.Z * R[1][2];
				point->Z = temp.X * R[2][0] + temp.Y * R[2][1] + temp.Z * R[2][2];

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.
				// Find min-max tight bounding box around the point cloud in updatePointCloudPlaneCulling() function.
				    
				point->Invalid = false;
				
				Eigen::Vector4f f_point(point->X, point->Y, point->Z, 1.0);
				if(m_frustumClip.pointInFrustum(f_point) == INSIDE)
				{
					Point_t pclPoint(point->X, point->Y, point->Z, color->rgbRed, color->rgbGreen, color->rgbBlue);
					cloud->points.push_back(pclPoint);
				}
			}
		}
	}
	pclCloud = cloud;

	// DEBUG: Print the size of point cloud
	// if(device_indices.size() > 0)
	// 	cout << "Resolution of Point Cloud : (" << nColorFrameHeight[0] << ", " << nColorFrameWidth[0] << ")" << endl;
	// cout << "Point cloud size: " << pclCloud->points.size() << endl;
	// cout << "Number of views: " << countViews << endl;
}

/**
 * @brief Convert 3D depth image to point cloud in PCL format. It optimizes updatePointCloud2() function. 
 * Ideas from transformation/rgbz.c file in Azure Kinect SDK.
 * RAJRUP: Later I will use this function and remove updatePointCloudNoCulling() function. 
 * @param extCalibration 
 */
void MultiviewPointCloud::updatePointCloudClipCulling(const vector<Calibration *> &extCalibration)
{
	PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;
	
	for (uint32_t index : device_indices)
	{
		if (!transformedDepthImage[index].is_valid())
			transformedDepthImage[index] = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, 
																nColorFrameWidth[index], 
																nColorFrameHeight[index], 
																nColorFrameWidth[index] * 
																(int)sizeof(uint16_t));

		/**
		 * RAJRUP: Transformation from depth image to color image
		 * TODO: Load transformed depth image from file. Need to change on the capture side.
		 */
		device_transformationColorDownscaled[index].depth_image_to_color_camera(depthImage[index], &transformedDepthImage[index]);

		xy_table_t *xy_table_data_ = xy_tables[index];
		uint16_t *depth_data = (uint16_t *)(void *)transformedDepthImage[index].get_buffer();
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];
		Calibration *calibration = extCalibration[index];
		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;

		RGB *colorInDepth = pColorRGBX[index];
		Point3f *vertices = pPointCloud[index];
		Point3f temp;

		for (uint32_t i = 0; i < nVertices; i++)
		{	
			// All previous vertices are invalid. Only onvalidate the vertex and not the coordinates.
			Point3f *point = &vertices[i];
			RGB *color = &colorInDepth[i];
			point->Invalid = true;

			if (depth_data[i] > 0 && !isnan(xy_table_data_->x_table[i]) && !isnan(xy_table_data_->y_table[i]) && calibration->bCalibrated)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Chnage these operations to matrix multiplication using Eigen library.
				 */
				
				temp.X = xy_table_data_->x_table[i] * (float)depth_data[i]/1000.0f + T[0];
				temp.Y = xy_table_data_->y_table[i] * (float)depth_data[i]/1000.0f + T[1];
				temp.Z = (float)depth_data[i]/1000.0f + T[2];

				point->X = temp.X * R[0][0] + temp.Y * R[0][1] + temp.Z * R[0][2];
				point->Y = temp.X * R[1][0] + temp.Y * R[1][1] + temp.Z * R[1][2];
				point->Z = temp.X * R[2][0] + temp.Y * R[2][1] + temp.Z * R[2][2];

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.
				// Find min-max tight bounding box around the point cloud in updatePointCloudPlaneCulling() function.
				    
				point->Invalid = false;
				
				Eigen::Vector4f f_point(point->X, point->Y, point->Z, 1.0);
				if(m_frustumClip.pointInFrustum(f_point) == INSIDE)
				{
					Point_t pclPoint(point->X, point->Y, point->Z, color->rgbRed, color->rgbGreen, color->rgbBlue);
					cloud->points.push_back(pclPoint);
				}
			}
		}
		
	}
	pclCloud = cloud;

	// DEBUG: Print the size of point cloud
	// if(device_indices.size() > 0)
	// 	cout << "Resolution of Point Cloud : (" << nColorFrameHeight[0] << ", " << nColorFrameWidth[0] << ")" << endl;
	// cout << "Point cloud size: " << pclCloud->points.size() << endl;
}

/**
 * @brief Convert 3D depth image to point cloud in PCL format. It optimizes updatePointCloud2() function. 
 * Ideas from transformation/rgbz.c file in Azure Kinect SDK.
 * RAJRUP: Later I will use this function and remove updatePointCloudNoCulling() function. 
 * @param extCalibration 
 */
void MultiviewPointCloud::updatePointCloudPlaneCulling(const vector<Calibration *> &extCalibration)
{
	PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;
	
	minBound[0] = minBound[1] = minBound[2] = 1e10f;
	maxBound[0] = maxBound[1] = maxBound[2] = -1e10f;
	for (uint32_t index : device_indices)
	{
		if (!transformedDepthImage[index].is_valid())
			transformedDepthImage[index] = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, 
																nColorFrameWidth[index], 
																nColorFrameHeight[index], 
																nColorFrameWidth[index] * 
																(int)sizeof(uint16_t));

		/**
		 * RAJRUP: Transformation from depth image to color image
		 * TODO: Load transformed depth image from file. Need to change on the capture side.
		 */
		device_transformationColorDownscaled[index].depth_image_to_color_camera(depthImage[index], &transformedDepthImage[index]);

		xy_table_t *xy_table_data_ = xy_tables[index];
		uint16_t *depth_data = (uint16_t *)(void *)transformedDepthImage[index].get_buffer();
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];
		Calibration *calibration = extCalibration[index];
		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;

		RGB *colorInDepth = pColorRGBX[index];
		Point3f *vertices = pPointCloud[index];
		Point3f temp;

		for (uint32_t i = 0; i < nVertices; i++)
		{	
			// All previous vertices are invalid. Only onvalidate the vertex and not the coordinates.
			Point3f *point = &vertices[i];
			RGB *color = &colorInDepth[i];
			point->Invalid = true;

			if (depth_data[i] > 0 && !isnan(xy_table_data_->x_table[i]) && !isnan(xy_table_data_->y_table[i]) && calibration->bCalibrated)
			{
				point->X = xy_table_data_->x_table[i] * (float)depth_data[i]/1000.0f + T[0];
				point->Y = xy_table_data_->y_table[i] * (float)depth_data[i]/1000.0f + T[1];
				point->Z = (float)depth_data[i]/1000.0f + T[2];

				temp.X = point->X * R[0][0] + point->Y * R[0][1] + point->Z * R[0][2];
				temp.Y = point->X * R[1][0] + point->Y * R[1][1] + point->Z * R[1][2];
				temp.Z = point->X * R[2][0] + point->Y * R[2][1] + point->Z * R[2][2];
				memcpy(point, &temp, sizeof(Point3f));

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.

				if(point->X < minBound[0])
					minBound[0] = point->X;
				else if(point->X > maxBound[0])
					maxBound[0] = point->X;
				if(point->Y < minBound[1])
					minBound[1] = point->Y;
				else if(point->Y > maxBound[1])
					maxBound[1] = point->Y;
				if(point->Z < minBound[2])
					minBound[2] = point->Z;
				else if(point->Z > maxBound[2])
					maxBound[2] = point->Z;
				    
				point->Invalid = false;
				
				Eigen::Vector3f f_point(point->X, point->Y, point->Z);
				if(m_frustum.pointInFrustum(f_point) == INSIDE)
				{
					Point_t pclPoint(point->X, point->Y, point->Z, color->rgbRed, color->rgbGreen, color->rgbBlue);
					cloud->points.push_back(pclPoint);
				}
				// Point_t pclPoint(point->X, point->Y, point->Z, color->rgbRed, color->rgbGreen, color->rgbBlue);
				// cloud->points.push_back(pclPoint);
			}
		}
	}
	pclCloud = cloud;
	cout << "Point cloud size: " << pclCloud->points.size() << endl;
	cout << "minX: " << minBound[0] << " maxX: " << maxBound[0] << endl;
	cout << "minY: " << minBound[1] << " maxY: " << maxBound[1] << endl;
	cout << "minZ: " << minBound[2] << " maxZ: " << maxBound[2] << endl;
}

void MultiviewPointCloud::updatePointCloudOctreeCulling(const vector<Calibration *> &extCalibration)
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

	Vector4f minBound_ = Vector4f(minBound[0], minBound[1], minBound[2], 1.0f);
	Vector4f maxBound_ = Vector4f(maxBound[0], maxBound[1], maxBound[2], 1.0f);
	OctreeAABBCull *octreeCull = new OctreeAABBCull(minBound_, maxBound_, m_frustumClip);

	for (uint32_t index : device_indices)
	{
		if (!transformedDepthImage[index].is_valid())
			transformedDepthImage[index] = k4a::image::create(
																K4A_IMAGE_FORMAT_DEPTH16, 
																nColorFrameWidth[index], 
																nColorFrameHeight[index], 
																nColorFrameWidth[index] * 
																(int)sizeof(uint16_t)
															);

		if(ePointCloud[index] == NULL)
		{
			// Create the point cloud using Eigen. Vector4f: X = index 0, Y = index 1, Z = index 2, Valid = index 3 (1.0f = valid, 0.0f = invalid)
			ePointCloud[index] = new Point3fV[nColorFrameWidth[index] * nColorFrameHeight[index]];
		}
		
		// time_point start, end;
		// uint64_t elapsed_ms;

		/**
		 * RAJRUP: Transformation from depth image to color image
		 * TODO: Load transformed depth image from file. Need to change on the capture side.
		 */
		device_transformationColorDownscaled[index].depth_image_to_color_camera(depthImage[index], &transformedDepthImage[index]);

		xy_table_t *xy_table_data_ = xy_tables[index];
		uint16_t *depth_data = (uint16_t *)(void *)transformedDepthImage[index].get_buffer();
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];
		Calibration *calibration = extCalibration[index];
		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;

		RGB *colorInDepth = pColorRGBX[index];
		Point3f *vertices = pPointCloud[index];
		Point3f temp;

		for (uint32_t i = 0; i < nVertices; i++)
		{	
			// All previous vertices are invalid. Only onvalidate the vertex and not the coordinates.
			Point3f *point = &vertices[i];
			RGB *color = &colorInDepth[i];
			Point3fV *ePoint = &ePointCloud[index][i];
			point->Invalid = true;
			ePoint->valid = false;

			if (depth_data[i] > 0 && !isnan(xy_table_data_->x_table[i]) && !isnan(xy_table_data_->y_table[i]) && calibration->bCalibrated)
			{
				point->X = xy_table_data_->x_table[i] * (float)depth_data[i]/1000.0f + T[0];
				point->Y = xy_table_data_->y_table[i] * (float)depth_data[i]/1000.0f + T[1];
				point->Z = (float)depth_data[i]/1000.0f + T[2];

				temp.X = point->X * R[0][0] + point->Y * R[0][1] + point->Z * R[0][2];
				temp.Y = point->X * R[1][0] + point->Y * R[1][1] + point->Z * R[1][2];
				temp.Z = point->X * R[2][0] + point->Y * R[2][1] + point->Z * R[2][2];
				memcpy(point, &temp, sizeof(Point3f));

				// We can use v_bounds here to bound the point cloud. Refer to updatePointCloudNoCulling() function.

				point->Invalid = false;

				// Copy point to Point Cloud in eigen format
				*ePoint = Point3fV(point->X, point->Y, point->Z, !(point->Invalid)); // ePoint Cloud contains valid bool

				if (ePoint->valid)
				{
					int aabb_pos = octreeCull->culling(&(ePoint->xyz), m_frustumClip);
					if(aabb_pos == INSIDE || aabb_pos == INTERSECT)
						cloud->points.push_back(Point_t(point->X, point->Y, point->Z, color->rgbRed, color->rgbGreen, color->rgbBlue));
					// else if(aabb_pos == UNASSIGNED) 			// Fail safe check
					// 	cout << "Unassigned point" << endl;
				}
			}
		}
	}
	pclCloud = cloud;
	cout << "Point cloud size: " << pclCloud->points.size() << endl;

	if(octreeCull != nullptr)
		cout << "Height of Eigen Octree: " << octreeCull->calcHeight() << endl;

	time_point start, end;
	uint64_t elapsed_ms;

	// --------------------------------------- Time taken for PCL Octree Generation ----------------------------------------------
	/* start = time_now();
	pcl::octree::OctreePointCloud<Point_t> octreePCL(0.0001f);
	octreePCL.defineBoundingBox(minBound[0], minBound[1], minBound[2], maxBound[0], maxBound[1], maxBound[2]);

	octreePCL.setInputCloud(pclCloud);
	octreePCL.addPointsFromInputCloud();

	end = time_now();
	elapsed_ms = duration_ms(start, end);
	cout << "PCL Octree construction time: " << elapsed_ms << " ms" << endl;
	cout << "PCL Octree depth: " << octreePCL.getTreeDepth() << endl; */

	// ----------------------------------------- Time taken for our Octree Generation --------------------------------------------
	/* start = time_now();
	OctreeEigen *octree = new OctreeEigen(minBound, maxBound);
	for (uint32_t index : device_indices)
	{
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		for (uint32_t i = 0; i < nVertices; i++)
		{
			Point3fV *epoint = &ePointCloud[index][i];
			if (epoint->valid)
				octree->insert(&(epoint->xyz));
		}
	}
	// *** Deletion has overhead ***
	// delete octree;
	// octree = nullptr;

	end = time_now();
	elapsed_ms = duration_ms(start, end);
	cout << "Eigen Octree construction time: " << elapsed_ms << " ms" << endl;

	if(octree != nullptr)
		cout << "Height of Eigen Octree: " << octree->calcHeight() << endl; */

	// ---------------------------------------- Time taken for our Octree Culling (Non Incremental) ---------------------------------------------
	/* start = time_now();
	Vector4f minBound_ = Vector4f(minBound[0], minBound[1], minBound[2], 1.0f);
	Vector4f maxBound_ = Vector4f(maxBound[0], maxBound[1], maxBound[2], 1.0f);
	OctreeAABBCull *octreeCull = new OctreeAABBCull(minBound_, maxBound_, m_frustumClip);
	for (uint32_t index : device_indices)
	{
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];
		RGB *colorInDepth = pColorRGBX[index];
		Point3f *vertices = pPointCloud[index];

		for (uint32_t i = 0; i < nVertices; i++)
		{
			Point3f *point = &vertices[i];
			RGB *color = &colorInDepth[i];
			Point3fV *epoint = &ePointCloud[index][i];
			if (epoint->valid)
			{
				int aabb_pos = octreeCull->culling(&(epoint->xyz), m_frustumClip);
				if(aabb_pos == INSIDE || aabb_pos == INTERSECT)
					cloud->points.push_back(Point_t(point->X, point->Y, point->Z, color->rgbRed, color->rgbGreen, color->rgbBlue));
				else if(aabb_pos == UNASSIGNED)
					cout << "Unassigned point" << endl;
			}
		}
	}
	pclCloud = cloud;
	cout << "Point cloud size: " << pclCloud->points.size() << endl;
	// *** Deletion has overhead ***
	// delete octree;
	// octree = nullptr;

	end = time_now();
	elapsed_ms = duration_ms(start, end);
	cout << "Eigen Octree construction time: " << elapsed_ms << " ms" << endl;

	if(octreeCull != nullptr)
		cout << "Height of Eigen Octree: " << octreeCull->calcHeight() << endl; */
}

/**
 * @brief Convert 3D depth image to point cloud. It converts the point cloud to a PCL format.
 * Rajrup: Do I really need convertion to PCL for downstream processing? 
 * @param extCalibration 
 */
void MultiviewPointCloud::updatePointCloudNoCulling(const vector<Calibration *> &extCalibration)
{
	PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;

	for (uint32_t index : device_indices)
	{
		updateDepthPointCloudForColorFrame(index);

		int16_t *pointCloudData = (int16_t *)pointCloudImage[index].get_buffer();
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		RGB *colorInDepth = pColorRGBX[index];
		Point3f *vertices = pPointCloud[index];

		for (int i = 0; i < nColorFrameHeight[index]; i++)
		{
			for (int j = 0; j < nColorFrameWidth[index]; j++)
			{
				uint32_t vertexIndex = j + i * nColorFrameWidth[index];

				vertices[vertexIndex].X = pointCloudData[3 * vertexIndex + 0] / 1000.0f;
				vertices[vertexIndex].Y = pointCloudData[3 * vertexIndex + 1] / 1000.0f;
				vertices[vertexIndex].Z = pointCloudData[3 * vertexIndex + 2] / 1000.0f;

				if (vertices[vertexIndex].Z >= 0.0001)
				{
					Point3f tempPoint = vertices[vertexIndex];
					RGB tempColor = colorInDepth[vertexIndex];
					if (extCalibration[index]->bCalibrated)
					{
						tempPoint.X += extCalibration[index]->worldT[0];
						tempPoint.Y += extCalibration[index]->worldT[1];
						tempPoint.Z += extCalibration[index]->worldT[2];
						tempPoint = RotatePoint(tempPoint, extCalibration[index]->worldR);

						// if (tempPoint.X < m_vBounds[0] || tempPoint.X > m_vBounds[3]
						//     || tempPoint.Y < m_vBounds[1] || tempPoint.Y > m_vBounds[4]
						//     || tempPoint.Z < m_vBounds[2] || tempPoint.Z > m_vBounds[5]) 
						//     continue;
							
					}
					Point_t point;
					point.x = tempPoint.X;
					point.y = tempPoint.Y;
					point.z = tempPoint.Z;
					point.r = tempColor.rgbRed;
					point.g = tempColor.rgbGreen;
					point.b = tempColor.rgbBlue;
					cloud->points.push_back(point);
				}
			}
		}
	}
	pclCloud = cloud;
}

void MultiviewPointCloud::updatePointCloud(const vector<Calibration *> &extCalibration)
{
	vector<Point3s> lFrameVerts;
	vector<RGB> lFrameRGB;

	time_point start, end;
	uint64_t elapsed_ms;
	for (uint32_t index : device_indices)
	{
		
        start = time_now();
		updateDepthPointCloudForColorFrame(index);
		end = time_now();
        elapsed_ms = duration_ms(start, end);
		cout << "Time taken by updateDepthPointCloudForColorFrame: " << elapsed_ms << " ms" << endl;

		start = time_now();
		int16_t *pointCloudData = (int16_t *)pointCloudImage[index].get_buffer();
		for (int i = 0; i < nColorFrameHeight[index]; i++)
		{
			for (int j = 0; j < nColorFrameWidth[index]; j++)
			{
				pPointCloud[index][j + i * nColorFrameWidth[index]].X = pointCloudData[3 * (j + i * nColorFrameWidth[index]) + 0] / 1000.0f;
				pPointCloud[index][j + i * nColorFrameWidth[index]].Y = pointCloudData[3 * (j + i * nColorFrameWidth[index]) + 1] / 1000.0f;
				pPointCloud[index][j + i * nColorFrameWidth[index]].Z = pointCloudData[3 * (j + i * nColorFrameWidth[index]) + 2] / 1000.0f;
			}
		}
		end = time_now();
        elapsed_ms = duration_ms(start, end);
		cout << "Time taken by PointCloud copy: " << elapsed_ms << " ms" << endl;

		start = time_now();
		// Generate clean point cloud - remove points with invalid depth, filter points, generate points within a bounding box
		Point3f *vertices = pPointCloud[index];
		RGB *colorInDepth = pColorRGBX[index];

		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		//To save some processing cost, we allocate a full frame size (nVertices) of a Point3f Vector beforehand
		//instead of using push_back for each vertice. Even though we have to copy the vertices into a clean array
		//later and it uses a little bit more RAM, this gives us a nice speed increase for this function, around 25-50%

		Point3f invalidPoint = Point3f(0, 0, 0, true);
		vector<Point3f> AllVertices(nVertices);
		uint32_t goodVerticesCount = 0;

		for (uint32_t vertexIndex = 0; vertexIndex < nVertices; vertexIndex++)
		{   
			// Rajrup: No need to deal with bodies now
			// if (m_bStreamOnlyBodies && bodyIndex[vertexIndex] >= bodies.size())
			//     continue;

			//As the resizing function doesn't return a valid RGB-Reserved value which indicates that this pixel is invalid,
			//we cut all vertices under a distance of 0.0001mm, as the invalid vertices always have a Z-Value of 0
			// Rajrup: Note this rgbReserved is reserved for alpha channel. I don't filter on alpha channel now.
			// if (vertices[vertexIndex].Z >= 0.0001 && colorInDepth[vertexIndex].rgbReserved == 255)
			if (vertices[vertexIndex].Z >= 0.0001)
			{
				Point3f temp = vertices[vertexIndex];
				RGB tempColor = colorInDepth[vertexIndex];
				if (extCalibration[index]->bCalibrated)
				{
					temp.X += extCalibration[index]->worldT[0];
					temp.Y += extCalibration[index]->worldT[1];
					temp.Z += extCalibration[index]->worldT[2];
					temp = RotatePoint(temp, extCalibration[index]->worldR);

					// if (temp.X < m_vBounds[0] || temp.X > m_vBounds[3]
					//     || temp.Y < m_vBounds[1] || temp.Y > m_vBounds[4]
					//     || temp.Z < m_vBounds[2] || temp.Z > m_vBounds[5]) 
					// {
					//     AllVertices[vertexIndex] = invalidPoint;
					//     continue;
					// }
						
				}

				AllVertices[vertexIndex] = temp;
				goodVerticesCount++;
			}
			else 
			{
				AllVertices[vertexIndex] = invalidPoint;
			}
		}
		
		end = time_now();
		elapsed_ms = duration_ms(start, end);
		cout << "Time taken by PointCloud filtering: " << elapsed_ms << " ms" << endl;

		start = time_now();
		vector<Point3f> goodVertices(goodVerticesCount);
		vector<RGB> goodColorPoints(goodVerticesCount);
		int goodVerticesShortCounter = 0;

		//Copy all valid vertices into a clean vector 
		for (unsigned int i = 0; i < AllVertices.size(); i++)
		{
			if (!AllVertices[i].Invalid) 
			{
				goodVertices[goodVerticesShortCounter] = AllVertices[i];
				goodColorPoints[goodVerticesShortCounter] = colorInDepth[i];
				goodVerticesShortCounter++;
			}
		}

		if (m_bFilter)
			filter(goodVertices, goodColorPoints, m_nFilterNeighbors, m_fFilterThreshold);


		vector<Point3s> goodVerticesShort(goodVertices.size());
		
		for (size_t i = 0; i < goodVertices.size(); i++)
		{
			goodVerticesShort[i] = goodVertices[i];
		}

		m_vLastFrameVertices[index] = goodVerticesShort;
		m_vLastFrameRGB[index] = goodColorPoints;

		end = time_now();
		elapsed_ms = duration_ms(start, end);
		cout << "Time taken by good PointCloud generation: " << elapsed_ms << " ms" << endl;

		start = time_now();
		// Merge all the points into one
		lFrameVerts.insert(lFrameVerts.end(), m_vLastFrameVertices[index].begin(), m_vLastFrameVertices[index].end());
		lFrameRGB.insert(lFrameRGB.end(), m_vLastFrameRGB[index].begin(), m_vLastFrameRGB[index].end());
		end = time_now();
		cout << "Time taken by PointCloud merge: " << duration_ms(start, end) << " ms" << endl;
	}

	start = time_now();
	// Creates a PCL point cloud
	pclCloud = convertToPCL(lFrameVerts, lFrameRGB);
	end = time_now();
	elapsed_ms = duration_ms(start, end);
	cout << "Time taken by PCL conversion: " << elapsed_ms << " ms" << endl;
}

void MultiviewPointCloud::viewPointCloud()
{
	if(pViewer != NULL)
		pViewer->display(pclCloud);
}

void MultiviewPointCloud::viewPointCloudFrustum()
{
	if(pViewer != NULL)
	{
		pViewer->displayCullPlane(pclCloud, minBound, maxBound);
		if(pViewer->b_camChanged)
		{
			pViewer->b_camChanged = false;
			updateFrustum(pViewer->m_camInt, pViewer->m_camView);
		}
	}
}

void MultiviewPointCloud::viewPointCloudFrustumClip()
{
	if(pViewer != NULL)
	{
		pViewer->displayCullClip(pclCloud);
		if(pViewer->b_camChanged)
		{
			pViewer->b_camChanged = false;
			updateFrustumClip(pViewer->m_camProjViewMat);
		}
	}
}

void MultiviewPointCloud::savePointCloud(const string &filename)
{
	if(pcl::io::savePLYFile(filename, *pclCloud) == 0)
		cout << "Saved " << filename << endl;
	else
		cout << "Failed to save " << filename << endl;
}