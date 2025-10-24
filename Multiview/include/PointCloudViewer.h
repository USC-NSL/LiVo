#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "frustum.h"
#include "consts.h"

using namespace std;

class PointCloudViewer
{
public:
	PointCloudViewer(const string &wName);
	~PointCloudViewer();

	void display(const PointCloud_t::Ptr &cloud);
	void displayCullPlane(const PointCloud_t::Ptr &cloud);
	void displayCullPlane(const PointCloud_t::Ptr &cloud, const Vector3f &minBound, const Vector3f &maxBound);
	void displayCullClip(const PointCloud_t::Ptr &cloud);
	void displayCullClipThread(const PointCloud_t::Ptr &cloud);

	void camUpdate();
	void camUpdateClip();

	void drawText(const string &text, const pcl::visualization::MouseEvent &event);

	CamInt m_camInt;
	CamView m_camView;

	Matrix4d m_camProjMat;
	Matrix4d m_camViewMat;
	Matrix4d m_camProjViewMat;
	bool b_camChanged;

private:
	pcl::visualization::CloudViewer *viewer;
	pcl::visualization::PCLVisualizer *viewer2;

	// void drawCube(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
	
	// void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void* viewer_void);

	boost::mutex updateModelMutex;
	
};