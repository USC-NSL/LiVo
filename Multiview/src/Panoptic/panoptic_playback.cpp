#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <boost/filesystem.hpp>

#include "Server/MultiviewServer.h"
#include "Client/MultiviewClient.h"
#include "Client/PointCloudStreamer.h"
#include "DataPlayback.h"
#include "pconsts.h"
#include "consts.h"
#include "timer.h"
#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"

DEFINE_int32(log_level, 0, "0 - INFO, 1 - WARNING, 2 - ERROR, 3 - FATAL.");
DEFINE_int32(view_ptcl, VIEWER::PCL_VIEWER, "View point cloud streams using PCL or Unity Viewer. 0 - No view. 1 - PCL. 2 - Unity. 3 - Open3D.");
DEFINE_bool(save_ptcl, false, "Save point cloud streams to disk as .ply(s).");
DEFINE_bool(save_views, false, "Save views to disk as .png(s).");
DEFINE_int32(server_cull, CULLING::NORMAL_VOXEL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling. 5 - Expansion Culling");
DEFINE_int32(client_cull, CULLING::NORMAL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling.");
DEFINE_int32(load_frustum, LOAD_FRUSTUM::PANOPTIC, "0 - Use deafult frustum (PCL), 1 - User trace on Panoptic, 2 - User trace on Kinect (Not implemented), 3 - Load frustum from disk as .txt.");
DEFINE_bool(save_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_bool(load_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_string(config_file, "/home/lei/rajrup/KinectStream/Multiview/config/panoptic.json", "Path to config file.");
DEFINE_bool(ground, false, "Ground removal.");

using namespace std;
namespace fs = boost::filesystem;
using namespace open3d;

volatile sig_atomic_t flag;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_handler(int signal_num)
{
    flag = signal_num;
}

int main(int argc, char** argv)
{
    gflags::SetUsageMessage("Usage: PanopticPlaybackServerClient [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    absl::SetMinLogLevel((absl::LogSeverityAtLeast)FLAGS_log_level);

    string config_file = FLAGS_config_file;
    parse_config(config_file);

    string path = FORMAT(PANOPTIC_DATA_PATH << SEQ_NAME << "/"); // RAJRUP: Save to SSD

	LOG(INFO) << "Data Path: " << path;
    LOG(INFO) << "Sequence Name: " << SEQ_NAME;

    // Creating a directory
    if(!fs::exists(path))
        LOG(FATAL) << "Directory " << path << " doesn't exist";

    string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);

	// Playback mode for Panoptic Dataset
    // ---------------------------------------INITIALIZATION------------------------------------------------------
    
    // Check: Load panoptic user data
    QCHECK(FLAGS_load_frustum == LOAD_FRUSTUM::PANOPTIC) << "Please set --load_frustum=1 for Panoptic Dataset";

    unique_ptr<MultiviewServer> server = make_unique<MultiviewServer>(path, DATASET::PANOPTIC);
    unique_ptr<MultiviewClient> client = make_unique<MultiviewClient>(path, DATASET::PANOPTIC);

    unique_ptr<PointCloudViewer> pcl_viewer(nullptr);
    unique_ptr<PointCloudStreamer> ptcl_streamer(nullptr);

    unique_ptr<DataPlayback> data_playback = make_unique<DataPlayback>(user_data_path, SEQ_NAME, LOG_ID);
    data_playback->loadData(START_FRAME, END_FRAME);
    UserData orig_data, pred_data;

    unique_ptr<KalmanPredictor> kalman_predictor = make_unique<KalmanPredictor>();

	LOG(INFO) << "DATASET: " << SEQ_NAME;
    LOG(INFO) << "Start Frame: " << START_FRAME;
    LOG(INFO) << "End Frame: " << END_FRAME;
    
    if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
        pcl_viewer = make_unique<PointCloudViewer>("viewer");
    
    if(FLAGS_view_ptcl == VIEWER::UNITY_VIEWER)
        ptcl_streamer = make_unique<PointCloudStreamer>(PTCL_STREAM_PORT);

    visualization::Visualizer visualizer;
    std::shared_ptr<geometry::PointCloud> pcd_ptr = nullptr;
    if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
    {
        // Visualizer initialization
        visualizer.CreateVisualizerWindow("Open3D", 1600, 900, 50, 50);
        pcd_ptr = std::make_shared<geometry::PointCloud>();
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

        for(uint32_t capture_id = START_FRAME; capture_id <= END_FRAME; capture_id++)
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
            LOG(INFO) << "Server: Load frame time: " << sw_stage.ElapsedMs() << " ms";

			sw_stage.Restart();
            
            // Predict frustum from trace
            kalman_predictor->predictData(server->get_curr_frameId(), 1, orig_data, pred_data);

            // server->set_frustum(pred_data.frustum);
            server->set_frustum(orig_data.frustum, pred_data.frustum);
            server->update_view();

            // Don't free the views, it is owned by the server
            vector<View *> s_views = server->get_view();
            LOG(INFO) << "Server: Update view time: " << sw_stage.ElapsedMs() << " ms";

            sw_stage.Restart();
            int s_currFrameId = server->get_curr_frameId();
            client->set_curr_frameId(s_currFrameId);
			client->set_frustum(orig_data.frustum);
            
            if(!client->set_view(s_views))
                LOG(FATAL) << "Client: Failed to set view!";
            
			if(!client->update_ptcl())
                LOG(FATAL) << "Client: Failed to update ptcl!";

            LOG(INFO) << "Client: Update ptcl time: " << sw_stage.ElapsedMs() << " ms";

            if(FLAGS_view_ptcl == VIEWER::UNITY_VIEWER)
            {
                sw_stage.Restart();
                
                uint8_t *ptclBuffer;
                int32_t bufferSize = 0;
                client->get_streamable_ptcl(ptclBuffer, bufferSize);
                LOG(INFO) << "Client: Generate ptcl time: " << sw_stage.ElapsedMs() << " ms";

                sw_stage.Restart();
                if(!ptcl_streamer->sendPointCloud(ptclBuffer, bufferSize))
                    LOG(FATAL) << "Client: Failed to send ptcl to Unity!";

                LOG(INFO) << "Client: Send ptcl to Unity time: " << sw_stage.ElapsedMs() << " ms";

                delete[] ptclBuffer;
            }
            else if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
            {
				sw_stage.Restart();
                client->generate_pcl_object();
                LOG(INFO) << "Client: Generate pcl time: " << sw_stage.ElapsedMs() << " ms";
                
                sw_stage.Restart();
                PointCloud_t::Ptr pcl_object = client->get_pcl_object();
				pcl_viewer->displayCullPlane(pcl_object);
				LOG(INFO) << "Client: View ptcl time: " << sw_stage.ElapsedMs() << " ms";
			}
            else if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER && capture_id >= START_FRAME + 2)
            {
                sw_stage.Restart();

                vector<Vector3d> points, colors;
                client->get_open3d_points_colors(points, colors);
                cout << points[0][0] << ", " << points[0][1] << ", " << points[0][2] << endl;
                // LOG(INFO) << "Client: Generate open3d time: " << sw_stage.ElapsedMs() << " ms";
                
                // sw_stage.Restart();
                // pcd_ptr->Clear();
                pcd_ptr->points_ = points;
                pcd_ptr->colors_ = colors;

                if(capture_id == START_FRAME + 2)
                    visualizer.AddGeometry(pcd_ptr);
                else
                    visualizer.UpdateGeometry(pcd_ptr);
                visualizer.PollEvents();
                visualizer.UpdateRender();
                LOG(INFO) << "Client: View open3d ptcl time: " << sw_stage.ElapsedMs() << " ms";
            }

            // Get next user trace
            orig_data = data_playback->getUserData(client->get_curr_frameId());
            LOG(INFO) << "Playback: End-to-end latency: " << sw_total.ElapsedMs() << " ms";
		}
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