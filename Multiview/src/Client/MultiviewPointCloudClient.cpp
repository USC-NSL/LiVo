#include "Client/MultiviewPointCloudClient.h"
#include <omp.h>
#include <zstd.h>

MultiviewPointCloudClient::MultiviewPointCloudClient(const vector<uint32_t> &indices, const vector<string> &deviceSerials, const int datasetType)
{	
	m_datasetType = datasetType;

	device_indices = indices;
	device_serial_numbers = deviceSerials;
	xy_tables.resize(device_indices.size(), NULL);

	nColorFrameHeight.resize(device_indices.size()); // To Do: RAJRUP: Change to vector<int> to vector<pair<int, int>> for height and width
	nColorFrameWidth.resize(device_indices.size());

	nDepthFrameHeight.resize(device_indices.size());
	nDepthFrameWidth.resize(device_indices.size());

	pPointCloud.resize(device_indices.size(), NULL);
	pDepth.resize(device_indices.size(), NULL);
	pColorRGBX.resize(device_indices.size(), NULL);
	pBinaryMask.resize(device_indices.size(), NULL);

	m_views.resize(device_indices.size());

	nValidPoints = 0;

	minBound[0] = minBound[1] = minBound[2] = 0.0;
	maxBound[0] = maxBound[1] = maxBound[2] = 0.0;

	pclCloud = NULL;
	p_octree = NULL;

	windowName = "";
	pViewer = NULL;

	pPointCloudBuf = NULL;
	pointCloudBufSize = 0;
}

MultiviewPointCloudClient::~MultiviewPointCloudClient()
{

}

void MultiviewPointCloudClient::initViewer()
{
	windowName = "Point Cloud Viewer";
	pViewer = new PointCloudViewer(windowName);

	// Initialize the frustum when the viewer is created
	updateFrustumClip(pViewer->m_camProjViewMat);
}

void MultiviewPointCloudClient::viewPointCloud()
{
	if(pViewer != NULL)
		pViewer->display(pclCloud);
}

void MultiviewPointCloudClient::viewPointCloudFrustum()
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

void MultiviewPointCloudClient::viewPointCloudFrustumClip()
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

void MultiviewPointCloudClient::savePointCloud(const string &filename)
{
	saveToPly(filename, pclCloud);
}

void MultiviewPointCloudClient::savePointCloudBinary(const string &filename)
{
	saveToPlyBinary(filename, pclCloud);
}

void MultiviewPointCloudClient::setXYTables(const vector<xy_table_t *> &tables)
{
	assert(tables.size() == device_indices.size());

	int w, h;
	for (uint32_t index : device_indices)
	{
		xy_tables[index] = tables[index];
	}
}

void MultiviewPointCloudClient::setViews(const vector<View *> &views)
{
	// Invalidate the previous views
	for(auto index : device_indices)
		m_views[index].valid = false;

	// Set the new views.
	#pragma omp parallel for
	for(int i = 0; i < views.size(); i++)
	{
		auto view = views[i];
		if(view->viewID < 0 || !view->isValid())
		{
			LOG(WARNING) << "Skip invalid view with ID: " << view->viewID;
			continue;
		}

		uint32_t viewID = view->viewID;

		// Small vector, so linear search is fine.
		if (find(device_indices.begin(), device_indices.end(), viewID) == device_indices.end())
		{
			LOG(WARNING) << "Skipping... View " << viewID << " is not a valid device viewID.";
			continue;
		}
		
		if(!m_views[viewID].isInitialized())
			m_views[viewID].initView(view->depthWidth, view->depthHeight, view->colorWidth, view->colorHeight);

		// Deep copy images
		memcpy(m_views[viewID].depth_image, view->depth_image, view->depthWidth * view->depthHeight * (int)sizeof(uint16_t));
		memcpy(m_views[viewID].color_image, view->color_image, view->colorWidth * view->colorHeight * (int)sizeof(RGB));
		m_views[viewID].valid = true;
		m_views[viewID].viewID = viewID;
		m_views[viewID].frameID = view->frameID;
		m_views[viewID].depthWidth = view->depthWidth;
		m_views[viewID].depthHeight = view->depthHeight;
		m_views[viewID].colorWidth = view->colorWidth;
		m_views[viewID].colorHeight = view->colorHeight;
	}
}

void MultiviewPointCloudClient::setBinaryMask(const vector<cv::Mat> &masks)
{
	assert(masks.size() == device_indices.size());

	for (uint32_t index : device_indices)
	{
		assert(masks[index].rows == m_views[index].depthHeight && masks[index].cols == m_views[index].depthWidth);
		for(int i = 0; i < m_views[index].depthHeight; i++)
			for(int j = 0; j < m_views[index].depthWidth; j++)
			{
				m_views[index].binary_mask[i * m_views[index].depthWidth + j] = masks[index].at<uchar>(i, j) > 0 ? true : false;
				assert(masks[index].at<uchar>(i, j) == 0 || masks[index].at<uchar>(i, j) == 255);
			}
	}
}

void MultiviewPointCloudClient::setImagesFromViews()
{
	for(uint32_t i = 0; i < m_views.size(); i++)
	{
		int viewID = m_views[i].viewID;
		// Only set RGB+Depth for valid views, other data structure (for invalid views) will have stale point cloud.
		if(viewID < 0 || !m_views[i].isValid())
		{
			LOG(WARNING) << "Skip invalid view with ID: " << viewID;
			continue;
		}

		nColorFrameHeight[viewID] = m_views[i].colorHeight;
		nColorFrameWidth[viewID] = m_views[i].colorWidth;

		if(pColorRGBX[viewID] == NULL)
		{
			pColorRGBX[viewID] = new RGB[nColorFrameWidth[viewID] * nColorFrameHeight[viewID]];
			pPointCloud[viewID] = new Point3f[nColorFrameWidth[viewID] * nColorFrameHeight[viewID]];
		}

		nDepthFrameHeight[viewID] = m_views[i].depthHeight;
		nDepthFrameWidth[viewID] = m_views[i].depthWidth;

		assert(nDepthFrameHeight[viewID] == nColorFrameHeight[viewID]);
		assert(nDepthFrameWidth[viewID] == nColorFrameWidth[viewID]);
		
		if(pDepth[viewID] == NULL)
		{
			pDepth[viewID] = new uint16_t[nDepthFrameHeight[viewID] * nDepthFrameWidth[viewID]];
		}

		pColorRGBX[viewID] = m_views[i].color_image;
		pDepth[viewID] = m_views[i].depth_image;

		if(FLAGS_load_binary_mask)
		{
			if(pBinaryMask[viewID] == NULL)
				pBinaryMask[viewID] = new bool[nDepthFrameHeight[viewID] * nDepthFrameWidth[viewID]];

			pBinaryMask[viewID] = m_views[i].binary_mask;
		}
		else
		{
			if(pBinaryMask[viewID] != NULL)
				delete[] pBinaryMask[viewID];
			pBinaryMask[viewID] = NULL;
		}
	}

	// // DEBUG: Print the size of the color and depth images
	// if(m_views.size() > 0)
	// {
	// 	LOG(INFO) << "Resolution of Color Image: (" << nColorFrameHeight[0] << ", " << nColorFrameWidth[0] << ")";
	// 	LOG(INFO) << "Resolution of Depth Image: (" << nDepthFrameHeight[0] << ", " << nDepthFrameWidth[0] << ")";
	// }
}

/**
 * @brief Convert 3D depth image to point cloud in PCL format. It optimizes updatePointCloud2() function. 
 * Ideas from transformation/rgbz.c file in Azure Kinect SDK.
 * RAJRUP: Later I will use this function and remove updatePointCloudNoCulling() function. 
 * Only this function uses binary mask.
 * @param extCalibration 
 */
void MultiviewPointCloudClient::updatePointCloudClipCullingFromViewsWithoutPcl(const vector<Calibration *> &extCalibration)
{
	int countViews = 0;
	nValidPoints = 0;
	for (uint32_t index : device_indices)
	{
		Calibration *calibration = extCalibration[index];
		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		xy_table_t *xy_table_data_ = NULL;
		if(m_datasetType == DATASET::KINECT)
			xy_table_data_ = xy_tables[index];

		uint16_t *depth_data = pDepth[index];
		RGB *color_data = pColorRGBX[index];
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
			Point3f &point = vertices[i];
			RGB &color = color_data[i];
			point.Invalid = true;
			
			// No need to check if xy_table_data[i].xy.x or xy_table_data[i].xy.y is NaN. This done on server side.
			if (depth_data[i] > 0)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Change these operations to matrix multiplication using Eigen library.
				 */
				
				// temp.X = xy_table_data_->x_table[i] * (float)depth_data[i]/1000.0f + T[0];
				// temp.Y = xy_table_data_->y_table[i] * (float)depth_data[i]/1000.0f + T[1];
				// temp.Z = (float)depth_data[i]/1000.0f + T[2];

				// point->X = temp.X * R[0][0] + temp.Y * R[0][1] + temp.Z * R[0][2];
				// point->Y = temp.X * R[1][0] + temp.Y * R[1][1] + temp.Z * R[1][2];
				// point->Z = temp.X * R[2][0] + temp.Y * R[2][1] + temp.Z * R[2][2];

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
				
				/**
				 * @brief TODO: Rajrup: Voxel Culling is not implemented on the client side yet.
				 * Similarly, Normal culling and normal voxel culling aren't defined yet.
				 */
				if(FLAGS_client_cull == CULLING::CLIP_CULLING || FLAGS_client_cull == CULLING::CLIP_VOXEL_CULLING)
				{
					Eigen::Vector4f f_point(point.X, point.Y, point.Z, 1.0);
					if(m_frustumClip.pointInFrustum(f_point) == INSIDE)
					{
						point.Invalid = false;
						nValidPoints++;
					}
				}
				else
				{
					// No culling
					point.Invalid = false;
					nValidPoints++;
				}
			}
		}
	}
}

/**
 * @brief Convert 3D depth image to point cloud in PCL format.
 * Ideas from transformation/rgbz.c file in Azure Kinect SDK.
 * @param extCalibration 
 */
void MultiviewPointCloudClient::updatePointCloudNormalCullingFromViewsWithoutPcl(const vector<Calibration *> &extCalibration)
{
	// // Save the point cloud for each view to disk
	// string path = "/home/lei/data/pipeline/client_tiled/test/";
	// PointCloud_t::Ptr full_cloud(new PointCloud_t);
	// full_cloud->is_dense = false;

	// static int countFrames = 0;
	// countFrames++;
	int countViews = 0;
	nValidPoints = 0;

	// #pragma omp parallel for
	for (uint32_t index : device_indices)
	{
		// PointCloud_t::Ptr cloud(new PointCloud_t);
		// cloud->is_dense = false;

		Calibration *calibration = extCalibration[index];
		vector<float> T = calibration->worldT;
		vector<vector<float>> R = calibration->worldR;
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];

		xy_table_t *xy_table_data_ = NULL;
		if(m_datasetType == DATASET::KINECT)
			xy_table_data_ = xy_tables[index];

		uint16_t *depth_data = pDepth[index];
		RGB *color_data = pColorRGBX[index];
		Point3f *vertices = pPointCloud[index];
		bool *binary_mask = pBinaryMask[index];
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
			Point3f &point = vertices[i];
			RGB &color = color_data[i];
			point.Invalid = true;
			
			// No need to check if xy_table_data[i].xy.x or xy_table_data[i].xy.y is NaN. This done on server side.
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
				
				if(FLAGS_load_binary_mask && binary_mask[i] == false)
					continue;

				/**
				 * @brief TODO: Rajrup: Normal Voxel Culling is not implemented on the client side yet.
				 */
				if(FLAGS_client_cull == CULLING::NORMAL_CULLING || 
					FLAGS_client_cull == CULLING::NORMAL_VOXEL_CULLING || 
					FLAGS_client_cull == CULLING::NORMAL_EXPANSION_CULLING)
				{
					Vector3f f_point(point.X, point.Y, point.Z);
					if(m_frustum.pointInFrustum2(f_point) == INSIDE)
					{
						// LOG(INFO) << "culling happening!";
						point.Invalid = false;
						nValidPoints++;
						// cloud->points.push_back(Point_t(point.X, -point.Y, point.Z, color.rgbRed, color.rgbGreen, color.rgbBlue));
					}
				}
				else
				{
					// No culling
					point.Invalid = false;
					nValidPoints++;
					// cloud->points.push_back(Point_t(point.X, -point.Y, point.Z, color.rgbRed, color.rgbGreen, color.rgbBlue));
				}
			}
		}
		// // Save the point cloud for each view to disk
		// saveToPly(FORMAT(path << "ptcl_" << countFrames << "_" << index << ".ply"), cloud);
		// full_cloud->points.insert(full_cloud->points.end(), cloud->points.begin(), cloud->points.end());
	}
	// saveToPly(FORMAT(path << "ptcl_" << countFrames << ".ply"), full_cloud);
}

/**
 * @brief Convert 3D depth image to point cloud in PCL format. It optimizes updatePointCloud2() function. 
 * Ideas from transformation/rgbz.c file in Azure Kinect SDK.
 * RAJRUP: Later I will use this function and remove updatePointCloudNoCulling() function. 
 * @param extCalibration 
 */
void MultiviewPointCloudClient::updatePointCloudClipCullingFromViews(const vector<Calibration *> &extCalibration)
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

		xy_table_t *xy_table_data_ = NULL;
		if(m_datasetType == DATASET::KINECT)
			xy_table_data_ = xy_tables[index];

		uint16_t *depth_data = pDepth[index];
		RGB *color_data = pColorRGBX[index];
		bool *binary_mask = pBinaryMask[index];
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
			Point3f &point = vertices[i];
			RGB &color = color_data[i];
			point.Invalid = true;
			
			// TODO: Rajrup: Binary mask needs to be integrated on the client side.
			if(binary_mask && !binary_mask[i])
				continue;

			// No need to check if xy_table_data[i].xy.x or xy_table_data[i].xy.y is NaN. This done on server side.
			if (depth_data[i] > 0)
			{
				/**
				 * @brief RAJRUP
				 * TODO: Change these operations to matrix multiplication using Eigen library.
				 */
				
				// temp.X = xy_table_data_->x_table[i] * (float)depth_data[i]/1000.0f + T[0];
				// temp.Y = xy_table_data_->y_table[i] * (float)depth_data[i]/1000.0f + T[1];
				// temp.Z = (float)depth_data[i]/1000.0f + T[2];

				// point->X = temp.X * R[0][0] + temp.Y * R[0][1] + temp.Z * R[0][2];
				// point->Y = temp.X * R[1][0] + temp.Y * R[1][1] + temp.Z * R[1][2];
				// point->Z = temp.X * R[2][0] + temp.Y * R[2][1] + temp.Z * R[2][2];

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
				    
				point.Invalid = false;
				
				Eigen::Vector4f f_point(point.X, point.Y, point.Z, 1.0);
				if(m_frustumClip.pointInFrustum(f_point) == INSIDE)
				{
					Point_t pclPoint(point.X, point.Y, point.Z, color.rgbRed, color.rgbGreen, color.rgbBlue);
					cloud->points.push_back(pclPoint);
				}
			}
		}
	}
	pclCloud = cloud;

	// DEBUG: Print the size of point cloud
	// if(device_indices.size() > 0)
	// 	LOG(INFO) << "Resolution of Point Cloud : (" << nColorFrameHeight[0] << ", " << nColorFrameWidth[0] << ")";
	// LOG(INFO) << "Point cloud size: " << pclCloud->points.size();
	// LOG(INFO) << "Number of views: " << countViews;
}

/**
 * @brief This function generates a streamable point cloud from the point cloud data structure.
 * It is assumed that the each point in the point cloud has 15 bytes of data - 12 bytes for the vertex and 3 bytes for the color.
 * @param ptclBuffer First 12 * nPoints bytes are the vertices and the next 3 * nPoints bytes are the colors. 
 * @param bufferSize 
 * @param nPoints
 */
void MultiviewPointCloudClient::generateStreamablePtcl(uint8_t *&ptclBuffer, int32_t &bufferSize, uint32_t &nPoints)
{
	int32_t verticesSize = nValidPoints * sizeof(Point3f_S);
	int32_t colorsSize = nValidPoints * sizeof(RGB_S);
	bufferSize = verticesSize + colorsSize; // This should be 15 bytes per point.
	ptclBuffer = new uint8_t[bufferSize];
	nPoints = nValidPoints;

	// First copy the vertices and then the colors.
	uint32_t v_j = 0;
	uint32_t c_j = verticesSize;
	for (uint32_t index : device_indices)
	{
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];
		Point3f *vertices = pPointCloud[index];
		RGB *color_data = pColorRGBX[index];
		// LOG(INFO) << "nVertices: " << nVertices;

		for (uint32_t i = 0; i < nVertices; i++)
		{
			if(vertices[i].Invalid)
				continue;

			memcpy(ptclBuffer + v_j, &vertices[i], sizeof(Point3f_S));
			v_j += sizeof(Point3f_S);

			memcpy(ptclBuffer + c_j, &color_data[i], sizeof(RGB_S));
			c_j += sizeof(RGB_S);
		}
	}
}

PointCloud_t::Ptr MultiviewPointCloudClient::generatePCLObject()
{
	// LOG(INFO) << "generatePCLObject(): nValidPoints: " << nValidPoints;

	PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;

	uint32_t j = 0;
	for (uint32_t index : device_indices)
	{
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];
		Point3f *vertices = pPointCloud[index];
		RGB *color_data = pColorRGBX[index];
		// LOG(INFO) << "nVertices: " << nVertices;

		for (uint32_t i = 0; i < nVertices; i++)
		{
			Point3f &point = vertices[i];
			RGB &color = color_data[i];
			if(!point.Invalid)
			{
				cloud->points.push_back(Point_t(point.X, point.Y, point.Z, color.rgbRed, color.rgbGreen, color.rgbBlue));
			}
		}
	}
	pclCloud = cloud;
	return pclCloud;
}

void MultiviewPointCloudClient::generatePCLObject(PointCloud_t::Ptr &cloud)
{
	// LOG(INFO) << "generatePCLObject(): nValidPoints: " << nValidPoints;
	cloud->is_dense = false;
	cloud->points.resize(nValidPoints);

	uint32_t j = 0;
	for (uint32_t index : device_indices)
	{
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];
		Point3f *vertices = pPointCloud[index];
		RGB *color_data = pColorRGBX[index];
		// LOG(INFO) << "nVertices: " << nVertices;

		for (uint32_t i = 0; i < nVertices; i++)
		{
			Point3f &point = vertices[i];
			RGB &color = color_data[i];
			if(!point.Invalid)
			{
				cloud->points[j] = Point_t(point.X, point.Y, point.Z, color.rgbRed, color.rgbGreen, color.rgbBlue);
				j++;
			}
		}
	}
}

void MultiviewPointCloudClient::getOpen3DVerticesColor(vector<Vector3d> &points, vector<Vector3d> &colors)
{
	points.resize(nValidPoints);
	colors.resize(nValidPoints);
	int j = 0;
	for (uint32_t index : device_indices)
	{
		uint32_t nVertices = nColorFrameHeight[index] * nColorFrameWidth[index];
		Point3f *vertices = pPointCloud[index];
		RGB *color_data = pColorRGBX[index];
		
		// LOG(INFO) << "nVertices: " << nVertices;

		for (uint32_t i = 0; i < nVertices; i++)
		{
			Point3f &point = vertices[i];
			RGB &color = color_data[i];
			if(!point.Invalid)
			{
				points[j] = Vector3d(point.X, point.Y, point.Z);
				colors[j] = Vector3d(color.rgbRed, color.rgbGreen, color.rgbBlue)/255.0;
				j++;
			}
		}
	}
}

void MultiviewPointCloudClient::setFrustum(const Frustum &frustum)
{
	m_frustum = frustum;
}

void MultiviewPointCloudClient::updateFrustum(const CamInt &camInt, const CamView &camView)
{
	LOG(INFO) << "**** Updating frustum ****";
	m_frustum.setCamInternals(camInt);
	m_frustum.setCamPlanes(camView);
}

void MultiviewPointCloudClient::updateFrustumClip(const Matrix4d &camProjViewMat)
{
	LOG(INFO) << "**** Updating frustum clip ****";
	m_frustumClip.setClipPlanes(camProjViewMat);
}

Frustum *MultiviewPointCloudClient::getFrustum()
{
	return &m_frustum;
}

FrustumClip *MultiviewPointCloudClient::getFrustumClip()
{
	return &m_frustumClip;
}

/**
 * @brief Decompress the point cloud data using ZSTD library if compressionLevel >= 0.
 * 
 * @param buf Buffer containing the compressed point cloud data. It isn't freed in this function.
 * @param size 
 * @param compressionLevel Compression level used to compress the point cloud data. Compression level is set to -1 if no compression is used.
 */
void MultiviewPointCloudClient::decompressPointCloudZstd(const uint8_t *buf, int size, int compressionLevel)
{
	delete []pPointCloudBuf;
	pPointCloudBuf = NULL;

	StopWatch sw;
	if(compressionLevel >= 0)
	{
		int outSize = (int)ZSTD_getDecompressedSize(buf, (size_t)size);
		uint8_t *outBuf = new uint8_t[outSize];
		ZSTD_decompress(outBuf, (size_t)outSize, buf, (size_t)size);
		pPointCloudBuf = outBuf;
		pointCloudBufSize = outSize;
	}
	else
	{
		pPointCloudBuf = new uint8_t[size];
		memcpy(pPointCloudBuf, buf, size);
		pointCloudBufSize = size;
	}

	LOG(INFO) << "ZSTD - Decoded point cloud size: " << pointCloudBufSize << " bytes";
	LOG(INFO) << "ZSTD - Decoded point cloud in " << sw.ElapsedMs() << " ms";
}

/**
 * @brief Decompress the point cloud data using DRACO library.
 * 
 * @param buf Buffer containing the compressed point cloud data. It isn't freed in this function.
 * @param size 
 * @param cl compression level
 * @param qp quantization parameter
 */
void MultiviewPointCloudClient::decompressPointCloudDraco(const uint8_t *buf, int size, int cl, int qp)
{
	// Create a draco decoding buffer. Note that no data is copied in this step.
	draco::DecoderBuffer buffer;
	const char *data = reinterpret_cast<const char *>(buf);
	buffer.Init(data, size);

	draco::CycleTimer timer;

	// Decode the input data into a geometry.

	auto type_statusor = draco::Decoder::GetEncodedGeometryType(&buffer);
	if (!type_statusor.ok())
		LOG(FATAL) << "Failed getting geometry type: " << type_statusor.status().error_msg();

	const draco::EncodedGeometryType geom_type = type_statusor.value();
	assert(geom_type == draco::POINT_CLOUD);

	// Failed to decode it as mesh, so let's try to decode it as a point cloud.
	timer.Start();
	auto statusor = mDracoDecoder.DecodePointCloudFromBuffer(&buffer);
	if (!statusor.ok())
		LOG(FATAL) << "Failed decoding point cloud: " << statusor.status().error_msg();
	pDracoPtcl = std::move(statusor).value();
	timer.Stop();
	// LOG(INFO) << "DRACO - Decoded point cloud size " << buffer.decoded_size() << " bytes";
	// LOG(INFO) << "DRACO - Decoded point cloud in " << timer.GetInMs() << " ms";
}

void MultiviewPointCloudClient::updatePointCloudFromBuffer()
{
	int pos = 0;
	int nPoints = 0;
	memcpy(&nPoints, pPointCloudBuf + pos, sizeof(int));
	pos += sizeof(int);

	PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;

	Point3s point_s;
	Point3f point_f;
	RGB color;
	for(int i = 0; i < nPoints; i++)
	{
		memcpy(&point_s, pPointCloudBuf + pos, sizeof(Point3s));
		pos += sizeof(Point3s);

		color.rgbRed = pPointCloudBuf[pos++];
		color.rgbGreen = pPointCloudBuf[pos++];
		color.rgbBlue = pPointCloudBuf[pos++];

		point_f.X = static_cast<float>(point_s.X / 1000.0f);
		point_f.Y = static_cast<float>(point_s.Y / 1000.0f);
		point_f.Z = static_cast<float>(point_s.Z / 1000.0f);

		Point_t pclPoint(point_f.X, point_f.Y, point_f.Z, color.rgbRed, color.rgbGreen, color.rgbBlue);
		cloud->points.push_back(pclPoint);
	}
	pclCloud = cloud;
}

void MultiviewPointCloudClient::updatePointCloudFromZstdBufferO3D(vector<Vector3d> &points, vector<Vector3d> &colors)
{
	int pos = 0;
	int nPoints = 0;
	memcpy(&nPoints, pPointCloudBuf + pos, sizeof(int));
	pos += sizeof(int);

	Point3f point_f;
	RGB color;

	for(int i = 0; i < nPoints; i++)
	{
		memcpy(&(point_f.X), pPointCloudBuf + pos, sizeof(float));
		pos += sizeof(float);
		memcpy(&(point_f.Y), pPointCloudBuf + pos, sizeof(float));
		pos += sizeof(float);
		memcpy(&(point_f.Z), pPointCloudBuf + pos, sizeof(float));
		pos += sizeof(float);

		color.rgbRed = (uint8_t)pPointCloudBuf[pos];
		pos += 1;
		color.rgbGreen = (uint8_t)pPointCloudBuf[pos];
		pos += 1;
		color.rgbBlue = (uint8_t)pPointCloudBuf[pos];
		pos += 1;

		if(FLAGS_client_cull == CULLING::NORMAL_CULLING)
		{
			if(m_frustum.pointInFrustum2(Vector3f(point_f.X, point_f.Y, point_f.Z)) == INSIDE)
			{
				points.push_back(Vector3d(point_f.X, point_f.Y, point_f.Z));
				colors.push_back(Vector3d(color.rgbRed, color.rgbGreen, color.rgbBlue)/255.0);
			}
		}
		else if(FLAGS_client_cull == CULLING::NO_CULLING)
		{
			points.push_back(Vector3d(point_f.X, point_f.Y, point_f.Z));
			colors.push_back(Vector3d(color.rgbRed, color.rgbGreen, color.rgbBlue)/255.0);
		}
		else
			LOG(FATAL) << "Invalid culling type";
	}
}

void MultiviewPointCloudClient::updatePointCloudFromDracoBufferO3D(vector<Vector3d> &points, vector<Vector3d> &colors)
{
	// get vertices and color from draco point cloud
	int nPoints = pDracoPtcl->num_points();
	// points.resize(pDracoPtcl->num_points());
	// colors.resize(pDracoPtcl->num_points());

	float pos[3];
	uint8_t color[3];
	for (draco::PointIndex i(0); i < nPoints; ++i)
	{
		const draco::PointAttribute *const att = pDracoPtcl->GetNamedAttribute(draco::GeometryAttribute::POSITION, 0);
		const draco::PointAttribute *const att_color = pDracoPtcl->GetNamedAttribute(draco::GeometryAttribute::COLOR, 0);

		if (!att || !att->ConvertValue<float, 3>(att->mapped_index(i), pos))
			LOG(FATAL) << "Failed to get point attribute.";

		if (!att_color || !att_color->ConvertValue<uint8_t, 3>(att_color->mapped_index(i), color))
			LOG(FATAL) << "Failed to get color attribute.";

		if(FLAGS_client_cull == CULLING::NORMAL_CULLING)
		{
			if(m_frustum.pointInFrustum2(Vector3f(pos[0], pos[1], pos[2])) == INSIDE)
			{
				points.push_back(Vector3d(pos[0], pos[1], pos[2]));
				colors.push_back(Vector3d(color[0], color[1], color[2])/255.0);
			}
		}
		else if(FLAGS_client_cull == CULLING::NO_CULLING)
		{
			points.push_back(Vector3d(pos[0], pos[1], pos[2]));
			colors.push_back(Vector3d(color[0], color[1], color[2])/255.0);
		}
		else
			LOG(FATAL) << "Invalid culling type";

		// points[i.value()] << pos[0], pos[1], pos[2];
		// colors[i.value()] = Vector3d(color[0], color[1], color[2])/255.0;
	}
}

void MultiviewPointCloudClient::updatePointCloudFromBufferClipCulling()
{
	int pos = 0;
	int nPoints = 0;
	memcpy(&nPoints, pPointCloudBuf + pos, sizeof(int));
	pos += sizeof(int);

	PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;

	Point3s point_s;
	Point3f point_f;
	RGB color;
	for(int i = 0; i < nPoints; i++)
	{
		memcpy(&point_s, pPointCloudBuf + pos, sizeof(Point3s));
		pos += sizeof(Point3s);

		color.rgbRed = pPointCloudBuf[pos++];
		color.rgbGreen = pPointCloudBuf[pos++];
		color.rgbBlue = pPointCloudBuf[pos++];

		point_f.X = static_cast<float>(point_s.X / 1000.0f);
		point_f.Y = static_cast<float>(point_s.Y / 1000.0f);
		point_f.Z = static_cast<float>(point_s.Z / 1000.0f);
		point_f.Invalid = false;

		Eigen::Vector4f f_point(point_f.X, point_f.Y, point_f.Z, 1.0);
		if(m_frustumClip.pointInFrustum(f_point) == INSIDE)
		{
			Point_t pclPoint(point_f.X, point_f.Y, point_f.Z, color.rgbRed, color.rgbGreen, color.rgbBlue);
			cloud->points.push_back(pclPoint);
		}
	}
	pclCloud = cloud;
}