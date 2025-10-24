#include <iostream>
#include <memory>
#include <thread>
#include <csignal>
#include <atomic>
#include <vector>
#include <Eigen/Dense>
#include <boost/filesystem/fstream.hpp>
#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"
#include "consts.h"
#include "timer.h"

using namespace open3d;
using namespace Eigen;
using namespace std;
namespace fs = boost::filesystem;

std::atomic_bool running(true);

void sigint_handler(int signum)
{
	LOG(INFO) << "Received Ctrl+C signal. Stopping program...";
	running = false; // set the global flag to true to signal all threads to stop
}

void read_ptcl(const string &dir, int frameID, vector<uint8_t> &data)
{
	fs::path filepath{FORMAT(dir << frameID << ".bytes")};
	fs::ifstream file;
	file.open(filepath, ios::in | ios::binary);
	if (!file.is_open())
		LOG(FATAL) << "Failed to load ptcl file: " << filepath;
	
	file.seekg(0, std::ios::end);
	data.resize(file.tellg());
	file.seekg(0, std::ios::beg);
	file.read(reinterpret_cast<char *>(data.data()), data.size());
	file.close();
}

void parse_ptcl(const vector<uint8_t> &data, vector<Vector3d> &points, vector<Vector3d> &colors)
{
	int nPoints = data.size() / 15;
	points.resize(nPoints);
	colors.resize(nPoints);
	for(int i = 0; i < nPoints; i++)
	{
		float x = 0, y = 0, z = 0;
		memcpy(&x, &data[i * 15 + 0], sizeof(float));
		memcpy(&y, &data[i * 15 + 4], sizeof(float));
		memcpy(&z, &data[i * 15 + 8], sizeof(float));
		points[i] = Vector3d(x, -y, z);
		colors[i] = Vector3d(data[i * 15 + 12], data[i * 15 + 13], data[i * 15 + 14]);
		colors[i] /= 255.0;
	}
}

int main()
{
	std::signal(SIGINT, sigint_handler);

	LOG(INFO) << "Hello Open3D!";

	string path = "/datassd/KinectStream/panoptic_captures/kinoptic_ptclouds/";

	int start = 3000;
	int end = 5000;

	StopWatch sw;

	// Read from disk
	vector<vector<uint8_t>> ptcl_buf;
	for(int i = start; i <= end; i++)
	{
		sw.Restart();
		vector<uint8_t> temp;
		read_ptcl(path, i, temp);
		ptcl_buf.push_back(temp);
		LOG(INFO) << "Read ptcl " << i << " in " << sw.ElapsedMs() << " ms";
	}

	vector<vector<Vector3d>> points;
	vector<vector<Vector3d>> colors;

	// Parse
	for(int i = 0; i < ptcl_buf.size(); i++)
	{
		sw.Restart();
		vector<Vector3d> temp_points;
		vector<Vector3d> temp_colors;
		parse_ptcl(ptcl_buf[i], temp_points, temp_colors);
		points.push_back(temp_points);
		colors.push_back(temp_colors);
		LOG(INFO) << "Parsed ptcl " << i << " num points " << temp_points.size() << " in " << sw.ElapsedMs() << " ms";

		ptcl_buf[i].clear();
	}
	ptcl_buf.clear();

	// Visualize
	visualization::Visualizer visualizer;
	visualizer.CreateVisualizerWindow("Open3D", 1600, 900, 50, 50);
	auto pcd_ptr = std::make_shared<geometry::PointCloud>();

	int i = 0;
	int nframes = 0;

	
	sw.Restart();
	while(running)
	{
		pcd_ptr->points_ = points[i];
		pcd_ptr->colors_ = colors[i];
		if(i == 0)
			visualizer.AddGeometry(pcd_ptr);
		else
			visualizer.UpdateGeometry(pcd_ptr);
		visualizer.PollEvents();
		visualizer.UpdateRender();

		nframes++;
		i = (i + 1) % (end - start + 1);

		float fps = (float)nframes / sw.ElapsedMs() * 1000.0f;

		if(nframes % 100 == 0)
			LOG(INFO) << "Render FPS: " << fps;
	}

	return 0;
}