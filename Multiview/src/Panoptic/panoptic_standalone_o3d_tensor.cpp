#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <chrono>
#include <thread>

#include "Server/MultiviewServer.h"
#include "Client/MultiviewClient.h"
#include "Client/PointCloudStreamer.h"
#include "DataPlayback.h"
#include "pconsts.h"
#include "consts.h"
#include "timer.h"
#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/io/PointCloudIO.h"

DEFINE_int32(log_level, 0, "0 - INFO, 1 - WARNING, 2 - ERROR, 3 - FATAL.");
DEFINE_int32(view_ptcl, VIEWER::PCL_VIEWER, "View point cloud streams using PCL or Unity Viewer. 0 - No view. 1 - PCL. 2 - Unity. 3 - Open3D.");
DEFINE_string(seq_name, "", "Sequence name.");
DEFINE_int32(start_frame_id, -1, "Start frame ID.");
DEFINE_int32(ncaptures, 6000, "Number of captures to be made or loaded from disk.");
DEFINE_bool(save_ptcl, false, "Save point cloud streams to disk as .ply(s).");
DEFINE_bool(save_views, false, "Save views to disk as .png(s).");
DEFINE_int32(server_cull, CULLING::NORMAL_VOXEL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling. 5 - Expansion Culling");
DEFINE_int32(client_cull, CULLING::NORMAL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling.");
DEFINE_int32(load_frustum, LOAD_FRUSTUM::PANOPTIC, "0 - Use deafult frustum (PCL), 1 - User trace on Panoptic, 2 - User trace on Kinect (Not implemented), 3 - Load frustum from disk as .txt.");
DEFINE_bool(save_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_bool(load_binary_mask, false, "Load binary mask from disk.");
DEFINE_string(config_file, "/home/lei/rajrup/KinectStream/Multiview/config/panoptic.json", "Path to config file.");
DEFINE_string(render_image_path, "", "Path to save render image using Open3D. Default is don't save");

using namespace std;
namespace fs = boost::filesystem;
namespace o3d = open3d;

volatile sig_atomic_t flag;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_handler(int signal_num)
{
    flag = signal_num;
}

int main(int argc, char** argv)
{
    gflags::SetUsageMessage("Usage: PanopticO3D_GT [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    absl::SetMinLogLevel((absl::LogSeverityAtLeast)FLAGS_log_level);

    string config_file = FLAGS_config_file;
    parse_config(config_file);

    // Pre-defined flags since we need to extract ground truth frames
    FLAGS_server_cull = CULLING::NO_CULLING;
    FLAGS_client_cull = CULLING::NO_CULLING;
    FLAGS_view_ptcl = VIEWER::OPEN3D_VIEWER;

    string seqName = "160317_moonbaby1";
    // int startFrame = 59;
    // int endFrame = 6193;

    int startFrame = 200;
    int endFrame = 1000;

    string path = FORMAT(PANOPTIC_DATA_PATH << seqName << "_clean/"); // RAJRUP: Save to SSD
    // string path = "/home/lei/data/pipeline/server_standalone/nvenc_10k_infinite/";
    // string path = "/home/lei/data/pipeline/server_standalone/nvenc_10k_20k/";
    // string path = "/home/lei/data/pipeline/server_standalone/nvenc_10k_jpeg_1/";

	LOG(INFO) << "Data Path: " << path;
    LOG(INFO) << "Sequence Name: " << seqName;

    // Creating a directory
    if(!fs::exists(path))
        LOG(FATAL) << "Directory " << path << " doesn't exist";

    string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);
    if(!fs::exists(user_data_path))
        LOG(FATAL) << "Directory " << user_data_path << " doesn't exist";

    if(FLAGS_view_ptcl != VIEWER::OPEN3D_VIEWER && FLAGS_render_image_path != "")
        LOG(FATAL) << "Please set --view_ptcl=3 for Open3D viewer to render image";

    string output_path = FLAGS_render_image_path;
    LOG(INFO) << "Output Path: " << output_path;
    
    if(FLAGS_render_image_path != "" && !fs::exists(output_path))
    {
        if(fs::create_directories(output_path))
            LOG(INFO) << "Directory " << output_path << " created";
        else
            LOG(FATAL) << "Directory " << output_path << " creation failed";
    }

	// Playback mode for Panoptic Dataset
    // ---------------------------------------INITIALIZATION------------------------------------------------------
    
    // Check: Load panoptic user data
    QCHECK(FLAGS_load_frustum == LOAD_FRUSTUM::PANOPTIC) << "Please set --load_frustum=1 for Panoptic Dataset";

    unique_ptr<MultiviewServer> server = make_unique<MultiviewServer>(path, DATASET::PANOPTIC);
    unique_ptr<MultiviewClient> client = make_unique<MultiviewClient>(path, DATASET::PANOPTIC);

    unique_ptr<DataPlayback> data_playback = make_unique<DataPlayback>(user_data_path, SEQ_NAME, LOG_ID);
    data_playback->loadData(START_FRAME, END_FRAME);
    UserData orig_data, pred_data;
    bool first_frame = true;

    // unique_ptr<KalmanPredictor> kalman_predictor = make_unique<KalmanPredictor>();

	LOG(INFO) << "DATASET: " << SEQ_NAME;
    LOG(INFO) << "Start Frame: " << START_FRAME;
    LOG(INFO) << "End Frame: " << END_FRAME;

    o3d::core::Device device("CUDA:0");
    auto dtype = o3d::core::Dtype::Float32;
    o3d::visualization::Visualizer visualizer;
    std::shared_ptr<o3d::t::geometry::PointCloud> pcd_ptr = nullptr;
    if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
    {
        // Visualizer initialization
        visualizer.CreateVisualizerWindow("Open3D", 1600, 900, 0, 0);
        pcd_ptr = std::make_shared<o3d::t::geometry::PointCloud>(device);
    }

    if(!server->init_playback())
        LOG(FATAL) << "Server: Failed to initialize playback!";

    if(server->get_calibration_requirement())
    {
        if(!server->load_calibration())
            LOG(FATAL) << "Server: Failed to load calibration!";
    }

	vector<uint32_t> s_deviceIndices = server->get_device_indices();
    vector<string> s_deviceSerialNumbers = server->get_device_serial_numbers();
    int s_numDevices = server->get_device_count();
    if(s_numDevices <= 0)
        LOG(FATAL) << "Server: No devices found during playback!";

    if(s_numDevices != s_deviceIndices.size() || s_numDevices != s_deviceSerialNumbers.size())
        LOG(FATAL) << "Server: Device count mismatch during playback!";
    
    // Don't free the calibration data, it is owned by the server
    vector<Calibration *> s_calibrations = server->get_calibration();

    client->init(s_deviceIndices, s_deviceSerialNumbers, s_calibrations);

    if(!server->get_calibration_status())
        LOG(FATAL) << "Server: Failed to load Calibration!";

    if(!client->get_calibration_status())
        LOG(FATAL) << "Client: Failed to load Calibration!";

	// ---------------------------------------PLAYBACK------------------------------------------------------
    // Register Async Signal Handler
	try
	{
		struct sigaction sa;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        sa.sa_handler = signal_handler;
        sigaction(SIGINT, &sa, 0);

        StopWatch sw_total;
        StopWatch sw_stage;

        for(uint32_t capture_id = startFrame; capture_id <= endFrame; capture_id++)
        {
			sw_total.Restart();   
            if (flag) 
                throw (int)flag;
            
            LOG(INFO) << "-----------------------------------------------------------------------";

            sw_stage.Restart();
            server->set_curr_frameId(capture_id);
            LOG(INFO) << "Server: Starting with frame: " << capture_id;
            if(!server->load_frame_parallel())
            {
                LOG(INFO) << "Server: Skipping frame id " << capture_id;
                continue;
            }

            // if(!server->load_frame_with_colorized_depth_parallel(false, "png"))
            // {
            //     LOG(INFO) << "Server: Skipping frame id " << capture_id;
            //     continue;
            // }

            // if(!server->load_frame_with_colorized_depth_parallel(false, "jpeg"))
            // {
            //     LOG(INFO) << "Server: Skipping frame id " << capture_id;
            //     continue;
            // }

            LOG(INFO) << "Server: Load frame time: " << sw_stage.ElapsedMs() << " ms";

			sw_stage.Restart();

            orig_data = data_playback->getUserData(server->get_curr_frameId());
            server->set_frustum(orig_data.frustum);
            server->update_view();

            // Don't free the views, it is owned by the server
            vector<View *> s_views = server->get_view();
            LOG(INFO) << "Server: Update view time: " << sw_stage.ElapsedMs() << " ms";

            sw_stage.Restart();
            int s_currFrameId = server->get_curr_frameId();
            client->set_curr_frameId(s_currFrameId);
			client->set_frustum(orig_data.frustum);
            int c_currFrameId = client->get_curr_frameId();
            
            if(!client->set_view(s_views))
                LOG(FATAL) << "Client: Failed to set view!";

            if(FLAGS_load_binary_mask)
		        client->load_binary_mask(c_currFrameId);

            LOG(INFO) << "Client: Set view time: " << sw_stage.ElapsedMs() << " ms";
            
            sw_stage.Restart();
			if(!client->update_ptcl())
                LOG(FATAL) << "Client: Failed to update ptcl!";

            LOG(INFO) << "Client: Update ptcl time: " << sw_stage.ElapsedMs() << " ms";

            if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
            {
                sw_stage.Restart();

                O3DData *o3d_data = new O3DData();
                o3d_data->frame_id = c_currFrameId;
                client->get_open3d_points_colors(o3d_data->points, o3d_data->colors);

                // Get user trace
                assert(server->get_curr_frameId() == client->get_curr_frameId());
                assert(o3d_data->frame_id == client->get_curr_frameId());
                client->set_frustum(orig_data.frustum);

                // cout << o3d_data->points[0][0] << ", " << o3d_data->points[0][1] << ", " << o3d_data->points[0][2] << endl;
                // LOG(INFO) << "Client: Generate open3d time: " << sw_stage.ElapsedMs() << " ms";
                
                // sw_stage.Restart();
                // pcd_ptr->Clear();

                o3d::visualization::ViewControl& ctr = visualizer.GetViewControl();
                ctr.ChangeFieldOfView(60);

                o3d::camera::PinholeCameraParameters camera_params;
                ctr.ConvertToPinholeCameraParameters(camera_params);

                Eigen::Vector4d q_rotation{orig_data.quat[3], orig_data.quat[0], orig_data.quat[1], orig_data.quat[2]};
                Eigen::Matrix3d R = o3d::geometry::Geometry3D::GetRotationMatrixFromQuaternion(q_rotation);
                Eigen::Matrix4d temp;
                temp << R.coeff(0, 0), R.coeff(0, 1), R.coeff(0, 2), orig_data.pos[0],
                        R.coeff(1, 0), R.coeff(1, 1), R.coeff(1, 2), orig_data.pos[1],
                        R.coeff(2, 0), R.coeff(2, 1), R.coeff(2, 2), orig_data.pos[2],
                        0, 0, 0, 1;
                camera_params.extrinsic_ << temp.inverse();
                ctr.ConvertFromPinholeCameraParameters(camera_params);

                // pcd_ptr->points_ = o3d_data->points;
                // pcd_ptr->colors_ = o3d_data->colors;

                vector<Vector3f> temp_p(o3d_data->points.size());

                for(int i = 0; i < o3d_data->points.size(); i++)
                {
                    temp_p[i][0] = o3d_data->points[i][0];
                    temp_p[i][1] = o3d_data->points[i][1];
                    temp_p[i][2] = o3d_data->points[i][2];
                }
                

                // auto size = o3d::core::SizeVector({(int32_t)temp_p.size()});
                auto size = o3d::core::SizeVector(temp_p.size());
                auto tensor = o3d::core::Tensor(temp_p, size, dtype, device);
                pcd_ptr->SetPointPositions(tensor);

                size = o3d::core::SizeVector({(int32_t)o3d_data->colors.size()});
                pcd_ptr->SetPointColors(o3d::core::Tensor(o3d_data->colors, size, dtype, device));


                // Experimental
                // int angle = 90;
                // Eigen::Matrix4d pcd_flip ;
                // pcd_flip << 1, 0, 0, 0,
                //             0, 1, 0, 0,
                //             0, 0, 1, 0,
                //             0, 0, 0, 1;
                // auto rotation = pcd_ptr->GetRotationMatrixFromXYZ(Eigen::Vector3d(ANG2RAD(0), ANG2RAD(0), ANG2RAD(90)));
                // pcd_ptr->Transform(pcd_flip);
                // pcd_ptr->Translate(Eigen::Vector3d(1, 0, 0));
                // pcd_ptr->Rotate(rotation, Eigen::Vector3d(0, 0, 0));

                auto pcd_cpu = pcd_ptr->ToLegacy();
                if(first_frame)
                {
                    visualizer.AddGeometry(std::make_shared<o3d::geometry::PointCloud>(pcd_cpu));
                    first_frame = false;
                }
                else
                    visualizer.UpdateGeometry(std::make_shared<o3d::geometry::PointCloud>(pcd_cpu));
                visualizer.PollEvents();
                visualizer.UpdateRender();
                
                if(FLAGS_render_image_path != "")
                {
                    string output_file = FORMAT(output_path << client->get_curr_frameId() << ".png");
                    visualizer.CaptureScreenImage(output_file);
                }
                // o3d::io::WritePointCloud(FORMAT(output_path << client->get_curr_frameId() << ".ply"), pcd_cpu);

                delete o3d_data;

                LOG(INFO) << "Client: View open3d ptcl time: " << sw_stage.ElapsedMs() << " ms";
            }
            else
                LOG(FATAL) << "Invalid viewer! Only use Open3D for generating GT";

            LOG(INFO) << "Playback: End-to-end latency: " << sw_total.ElapsedMs() << " ms";
		}
        visualizer.DestroyVisualizerWindow();
	}
	catch(int ex)
    {
        LOG(INFO) << "Caught: Ctrl+C";
        LOG(INFO) << "Exiting...";
        return EXIT_FAILURE;
    }

    sleep(2);
	return EXIT_SUCCESS;

	return 0;
}