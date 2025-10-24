#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <chrono>
#include <thread>
#include <cmath>
#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"

#include "kalman.h"
#include "voxel.h"
#include "timer.h"
#include "test/utils.h"
#include "pconsts.h"
#include "DataPlayback.h"

#define READ_PLY 1
#define FRUSTUM_EXPANSION_CULL 0
#define FRUSTUM_ACTUAL_CULL 1
#define LOOKAHEAD_EXP_FRAME_NUM 3

using namespace std;
using namespace open3d;

void simluate_lookahead(std::shared_ptr<DataPlayback> m_dataPlayback, uint32_t s_currFrameId, uint32_t m_start_frame_id, std::shared_ptr<KalmanPredictor2> m_kalmanPredictor2, int lookAhead, Frustum &s_frustum, Frustum &s_pred_frustum, Frustum &s_actual_frustum)
{
	// Predict frustum from trace
	if(m_kalmanPredictor2 != nullptr)
	{
		if(s_currFrameId - lookAhead < m_start_frame_id)
		{
			UserData data = m_dataPlayback->getUserData(s_currFrameId);
			s_frustum = data.frustum;
			s_pred_frustum = data.frustum;
            s_actual_frustum = data.frustum;
		}
		else
		{
			UserData data = m_dataPlayback->getUserData(s_currFrameId - lookAhead);
			s_frustum = data.frustum;
            data = m_dataPlayback->getUserData(s_currFrameId);
            s_actual_frustum = data.frustum;
			// if(!m_kalmanPredictor2->predictDataUsingQuat(s_currFrameId, lookAhead, s_frustum, s_pred_frustum))
			// 	LOG(FATAL) << "Failed to predict frustum for Frame ID: " << s_currFrameId;

            if(!m_kalmanPredictor2->predictDataUsingQuat_test(s_currFrameId, lookAhead, s_frustum, s_pred_frustum, s_actual_frustum))
				LOG(FATAL) << "Failed to predict frustum for Frame ID: " << s_currFrameId;

			// LOG(INFO) << "Curr FID: " << s_currFrameId << ", Lookahead: " << s_currFrameId - s_frustum.frameID;
			// LOG(INFO) << "Pred FID: " << s_pred_frustum.frameID << ", Recv FID: " << s_frustum.frameID;
		}
	}
}

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

void expansionAABBFrustum(Frustum &m_prevFrustum, Frustum &m_predFrustum, Vector3f &cube_min_corner, Vector3f &cube_max_corner)
{
    // Cube enclosing the 2 frustums.
	Vector3f min_corner1, max_corner1, min_corner2, max_corner2;
	m_prevFrustum.getFrustumCubeEnclosedCorners(min_corner1, max_corner1);
	m_predFrustum.getFrustumCubeEnclosedCorners(min_corner2, max_corner2);

	calc_max_min_corners_frustum(m_prevFrustum, min_corner1, max_corner1);
	calc_max_min_corners_frustum(m_predFrustum, min_corner2, max_corner2);

	cube_min_corner = min_corner1.cwiseMin(min_corner2);
	cube_max_corner = max_corner1.cwiseMax(max_corner2);

    // LOG(INFO) << "Prev Frustum corners: ";
    // LOG(INFO) << "ntl: " << m_prevFrustum.ntl.transpose();
    // LOG(INFO) << "ntr: " << m_prevFrustum.ntr.transpose();
    // LOG(INFO) << "nbl: " << m_prevFrustum.nbl.transpose();
    // LOG(INFO) << "nbr: " << m_prevFrustum.nbr.transpose();
    // LOG(INFO) << "ftl: " << m_prevFrustum.ftl.transpose();
    // LOG(INFO) << "ftr: " << m_prevFrustum.ftr.transpose();
    // LOG(INFO) << "fbl: " << m_prevFrustum.fbl.transpose();
    // LOG(INFO) << "fbr: " << m_prevFrustum.fbr.transpose();

    // LOG(INFO) << "Pred Frustum corners: ";
    // LOG(INFO) << "ntl: " << m_predFrustum.ntl.transpose();
    // LOG(INFO) << "ntr: " << m_predFrustum.ntr.transpose();
    // LOG(INFO) << "nbl: " << m_predFrustum.nbl.transpose();
    // LOG(INFO) << "nbr: " << m_predFrustum.nbr.transpose();
    // LOG(INFO) << "ftl: " << m_predFrustum.ftl.transpose();
    // LOG(INFO) << "ftr: " << m_predFrustum.ftr.transpose();
    // LOG(INFO) << "fbl: " << m_predFrustum.fbl.transpose();
    // LOG(INFO) << "fbr: " << m_predFrustum.fbr.transpose();

    // LOG(INFO) << "Cube corners: ";
    // LOG(INFO) << "min_corner: " << cube_min_corner.transpose();
    // LOG(INFO) << "max_corner: " << cube_max_corner.transpose();
}

open3d::geometry::LineSet create_exapansion_frustum(const Vector3f &min_corner, const Vector3f &max_corner, const Eigen::Vector3d &color = Eigen::Vector3d(0, 0, 1))
{   
    auto frustum_points = geometry::PointCloud();
    frustum_points.points_.push_back(Vector3d(min_corner(0), max_corner(1), max_corner(2))); //ntl
    frustum_points.points_.push_back(Vector3d(max_corner(0), max_corner(1), max_corner(2))); //ntr
    frustum_points.points_.push_back(Vector3d(min_corner(0), min_corner(1), max_corner(2))); //nbl
    frustum_points.points_.push_back(Vector3d(max_corner(0), min_corner(1), max_corner(2))); //nbr
    frustum_points.points_.push_back(Vector3d(min_corner(0), max_corner(1), min_corner(2))); //ftl
    frustum_points.points_.push_back(Vector3d(max_corner(0), max_corner(1), min_corner(2))); //ftr
    frustum_points.points_.push_back(Vector3d(min_corner(0), min_corner(1), min_corner(2))); //fbl
    frustum_points.points_.push_back(Vector3d(max_corner(0), min_corner(1), min_corner(2))); //fbr
    auto lines = std::vector<Eigen::Vector2i>{{0, 1}, {0, 2}, {1, 3}, {2, 3},
                                              {4, 5}, {4, 6}, {5, 7}, {6, 7},
                                              {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    auto colors = std::vector<Eigen::Vector3d>(lines.size(), color);
    auto frustum_lines = open3d::geometry::LineSet(frustum_points.points_, lines);
    frustum_lines.colors_ = colors;
    return frustum_lines;
}

open3d::geometry::LineSet create_frustum_o3d(const Frustum &frustum, const Eigen::Vector3d &color = Eigen::Vector3d(1, 0, 0))
{
    auto frustum_points = geometry::PointCloud();
    frustum_points.points_.push_back(Vector3d(frustum.ntl(0), frustum.ntl(1), frustum.ntl(2)));
    frustum_points.points_.push_back(Vector3d(frustum.ntr(0), frustum.ntr(1), frustum.ntr(2)));
    frustum_points.points_.push_back(Vector3d(frustum.nbl(0), frustum.nbl(1), frustum.nbl(2)));
    frustum_points.points_.push_back(Vector3d(frustum.nbr(0), frustum.nbr(1), frustum.nbr(2)));
    frustum_points.points_.push_back(Vector3d(frustum.ftl(0), frustum.ftl(1), frustum.ftl(2)));
    frustum_points.points_.push_back(Vector3d(frustum.ftr(0), frustum.ftr(1), frustum.ftr(2)));
    frustum_points.points_.push_back(Vector3d(frustum.fbl(0), frustum.fbl(1), frustum.fbl(2)));
    frustum_points.points_.push_back(Vector3d(frustum.fbr(0), frustum.fbr(1), frustum.fbr(2)));
    auto lines = std::vector<Eigen::Vector2i>{{0, 1}, {0, 2}, {1, 3}, {2, 3},
                                              {4, 5}, {4, 6}, {5, 7}, {6, 7},
                                              {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    auto colors = std::vector<Eigen::Vector3d>(lines.size(), color);
    auto frustum_lines = geometry::LineSet(frustum_points.points_, lines);
    frustum_lines.colors_ = colors;
    return frustum_lines;
}

float avg_frustum_size = 0.0f;
int frustum_count = 1;
void visualize_frustum(visualization::Visualizer &vis, Frustum &s_actual_frustum, Frustum &s_frustum, Frustum &s_pred_frustum, open3d::geometry::PointCloud &ptcl)
{
    auto actual_frustum_lines = create_frustum_o3d(s_actual_frustum, Eigen::Vector3d(1, 0, 0));
    auto pred_frustum_lines = create_frustum_o3d(s_pred_frustum, Eigen::Vector3d(0, 1, 0));
    auto prev_frustum_lines = create_frustum_o3d(s_frustum, Eigen::Vector3d(0, 0, 1));
    
    vis.AddGeometry(std::make_shared<open3d::geometry::LineSet>(actual_frustum_lines));
    // vis.AddGeometry(std::make_shared<open3d::geometry::LineSet>(pred_frustum_lines));
    // vis.AddGeometry(std::make_shared<open3d::geometry::LineSet>(prev_frustum_lines));

    Vector3f min_corner, max_corner;
    expansionAABBFrustum(s_frustum, s_pred_frustum, min_corner, max_corner);
    auto expansion_frustum_lines = create_exapansion_frustum(min_corner, max_corner, Eigen::Vector3d(0, 0, 0));
    // vis.AddGeometry(std::make_shared<open3d::geometry::LineSet>(expansion_frustum_lines));

    if(ptcl.points_.size() > 0)
    {
        if(FRUSTUM_EXPANSION_CULL)
        {
            open3d::geometry::PointCloud ptcl_culled;
            for(int i = 0; i < ptcl.points_.size(); i++)
            {
                if(ptcl.points_[i](0) >= min_corner(0) && ptcl.points_[i](0) <= max_corner(0) &&
                    ptcl.points_[i](1) >= min_corner(1) && ptcl.points_[i](1) <= max_corner(1) &&
                    ptcl.points_[i](2) >= min_corner(2) && ptcl.points_[i](2) <= max_corner(2))
                {
                    ptcl_culled.points_.push_back(ptcl.points_[i]);
                    ptcl_culled.colors_.push_back(ptcl.colors_[i]);
                }
            }
            vis.AddGeometry(std::make_shared<open3d::geometry::PointCloud>(ptcl_culled));
            float frustum_size = ((float)ptcl_culled.points_.size()/(float)ptcl.points_.size());
            avg_frustum_size = (avg_frustum_size * (frustum_count - 1.0f) + frustum_size) / (float)frustum_count;
            LOG(INFO) << "FrameID: " << s_pred_frustum.frameID << ", Frustum size: " << frustum_size << ", Avg Frustum size: " << avg_frustum_size;
            frustum_count++;
        }
        else if(FRUSTUM_ACTUAL_CULL)
        {
            open3d::geometry::PointCloud ptcl_culled;
            for(int i = 0; i < ptcl.points_.size(); i++)
            {
                if(s_actual_frustum.pointInFrustum2(Vector3f(ptcl.points_[i](0), ptcl.points_[i](1), ptcl.points_[i](2))))
                {
                    ptcl_culled.points_.push_back(ptcl.points_[i]);
                    ptcl_culled.colors_.push_back(ptcl.colors_[i]);
                }
            }
            vis.AddGeometry(std::make_shared<open3d::geometry::PointCloud>(ptcl_culled));
            float frustum_size = ((float)ptcl_culled.points_.size()/(float)ptcl.points_.size());
            avg_frustum_size = (avg_frustum_size * (frustum_count - 1.0f) + frustum_size) / (float)frustum_count;
            LOG(INFO) << "FrameID: " << s_pred_frustum.frameID << ", Frustum size: " << frustum_size << ", Avg Frustum size: " << avg_frustum_size;
            frustum_count++;
        }
        else
            vis.AddGeometry(std::make_shared<open3d::geometry::PointCloud>(ptcl));
    }
}

void read_ptcl(const uint32_t frame_id, open3d::geometry::PointCloud &ptcl)
{
    string ptcl_file = FORMAT("/datassd/pipeline_ptcl/client_ptcl/160906_band2_with_ground/o3d_gt_with_ground_s_nocull_c_nocull/" << frame_id << ".ply");
    if(!fs::exists(ptcl_file))
        LOG(FATAL) << "File " << ptcl_file << " doesn't exist";

    open3d::io::ReadPointCloud(ptcl_file, ptcl);
    Eigen::Matrix4d flip_transform;
    flip_transform << 1, 0, 0, 0,
                    0, -1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
    ptcl.Transform(flip_transform);
}

int main(int argc, char** argv)
{
    gflags::SetUsageMessage("Usage: kalman_test_new [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    absl::SetMinLogLevel((absl::LogSeverityAtLeast)0);

    string config_file = "/home/lei/rajrup/KinectStream/Multiview/config/panoptic_160906_band2.json";
    parse_config(config_file);
    
    LOG(INFO) << "Frustrum KF prediction";

    string path = FORMAT(PANOPTIC_DATA_PATH << SEQ_NAME << "/");
    LOG(INFO) << "Path: " << path;
    string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);

    if(!fs::exists(user_data_path))
        LOG(FATAL) << "Directory " << user_data_path << " doesn't exist";

    std::shared_ptr<DataPlayback> data_playback = std::make_shared<DataPlayback>(user_data_path, SEQ_NAME, LOG_ID);
    data_playback->loadData(START_FRAME, END_FRAME);

    // visualization::Visualizer vis_standalone;
    // vis_standalone.CreateVisualizerWindow("Open3D");
    // auto opt = vis_standalone.GetRenderOption();
    // opt.show_coordinate_frame_ = true;
    open3d::visualization::VisualizerWithKeyCallback vis_standalone;
    vis_standalone.CreateVisualizerWindow("Open3D");
    auto opt = vis_standalone.GetRenderOption();
    // opt.ChangeLineWidth(10.0f);
    opt.line_width_ = 100.0f;
    opt.show_coordinate_frame_ = true;

    std::shared_ptr<KalmanPredictor2> m_kalmanPredictor2 = std::make_shared<KalmanPredictor2>(14, 7, 1.0e-2f, 1.0f, 33.0f/1000.0f);

    Frustum s_frustum, s_pred_frustum, s_actual_frustum;
    // for(uint32_t s_currFrameId = START_FRAME; s_currFrameId <= END_FRAME; s_currFrameId++)
    // {
    //     LOG(INFO) << "Frame ID: " << s_currFrameId;
    //     simluate_lookahead(data_playback, s_currFrameId, START_FRAME, m_kalmanPredictor2, LOOKAHEAD_EXP_FRAME_NUM, s_frustum, s_pred_frustum, s_actual_frustum);

        

    //     // visualize_frustum(vis_standalone, s_actual_frustum, s_pred_frustum);

    //     // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // }
    uint32_t start_frame_id = 35;
    uint32_t end_frame_id = start_frame_id + 4000;
    uint32_t s_currFrameId = start_frame_id;
    open3d::geometry::PointCloud ptcl;
    // Register right click callback to visualize frustum
    vis_standalone.RegisterKeyCallback(GLFW_KEY_N, [&](open3d::visualization::Visualizer *vis) -> bool 
    {
        vis->ClearGeometries();
        for(int i = 0; i < 1; i++)
        {
            LOG(INFO) << "Frame ID: " << s_currFrameId;
            simluate_lookahead(data_playback, s_currFrameId, start_frame_id, m_kalmanPredictor2, LOOKAHEAD_EXP_FRAME_NUM, s_frustum, s_pred_frustum, s_actual_frustum);
            if(s_currFrameId % 1 == 0)
            {
                if(READ_PLY)
                    read_ptcl(s_currFrameId, ptcl);
                visualize_frustum(*vis, s_actual_frustum, s_frustum, s_pred_frustum, ptcl);
                ptcl.Clear();
            }
                
            s_currFrameId += 1;
            if(s_currFrameId > end_frame_id)
                return false;
        }
        return true;
    });

    vis_standalone.RegisterKeyCallback(GLFW_KEY_C, [&](open3d::visualization::Visualizer *vis) -> bool 
    {
        vis->ClearGeometries();
        return true;
    });

    vis_standalone.RegisterKeyCallback(GLFW_KEY_ESCAPE, [&](open3d::visualization::Visualizer *vis) -> bool 
    {
        vis->Close();
        return true;
    });

    vis_standalone.PollEvents();
    vis_standalone.UpdateRender();
    vis_standalone.Run();

    return 0;
}