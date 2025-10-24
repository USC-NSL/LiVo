#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include "Server/MultiviewServer.h"
#include "Client/MultiviewClient.h"
#include "Client/PointCloudStreamer.h"
#include "DataPlayback.h"
#include "pconsts.h"
#include "consts.h"
#include "timer.h"

DEFINE_int32(log_level, 0, "0 - INFO, 1 - WARNING, 2 - ERROR, 3 - FATAL.");
DEFINE_int32(view_ptcl, VIEWER::PCL_VIEWER, "View point cloud streams using PCL or Unity Viewer. 0 - No view. 1 - PCL. 2 - Unity.");
DEFINE_bool(save_ptcl, false, "Save point cloud streams to disk as .ply(s).");
DEFINE_bool(save_views, false, "Save views to disk as .png(s).");
DEFINE_bool(save_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_bool(load_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_int32(server_cull, CULLING::NO_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling.");
DEFINE_int32(client_cull, CULLING::NO_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling.");
DEFINE_bool(save_frustum, false, "Save frustum to disk as .txt.");
DEFINE_int32(load_frustum, LOAD_FRUSTUM::NONE, "0 - Use default frustum (PCL), 1 - User trace on Panoptic, 2 - User trace on Kinect (Not implemented), 3 - Load frustum from disk as .txt.");
DEFINE_bool(update_frustum, false, "Placeholder.");
DEFINE_string(config_file, "/home/lei/rajrup/KinectStream/Multiview/config/panoptic.json", "Path to config file.");

using namespace std;
namespace fs = boost::filesystem;
using json = nlohmann::json;

volatile sig_atomic_t flag;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_handler(int signal_num)
{
    flag = signal_num;
}

int main(int argc, char** argv)
{
    gflags::SetUsageMessage("Usage: PanopticServerClient [OPTION] ...");

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
        LOG(FATAL) << "ERROR: Directory " << path << " doesn't exist";

    string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);

    // Playback mode for Panoptic Dataset
    // ---------------------------------------INITIALIZATION------------------------------------------------------
    unique_ptr<MultiviewServer> server = make_unique<MultiviewServer>(path, DATASET::PANOPTIC);
    unique_ptr<MultiviewClient> client = make_unique<MultiviewClient>(path, DATASET::PANOPTIC);

    unique_ptr<PointCloudViewer> pcl_viewer(nullptr);
    unique_ptr<PointCloudStreamer> ptcl_streamer(nullptr);

    LOG(INFO) << "DATASET: " << SEQ_NAME;
    LOG(INFO) << "Start Frame: " << START_FRAME;
    LOG(INFO) << "End Frame: " << END_FRAME;
    
    if(FLAGS_server_cull > CULLING::NORMAL_VOXEL_CULLING || FLAGS_client_cull > CULLING::NORMAL_VOXEL_CULLING)
        LOG(FATAL) << "Server: Invalid culling mode!";

    if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
        pcl_viewer = make_unique<PointCloudViewer>("viewer");
    
    if(FLAGS_view_ptcl == VIEWER::UNITY_VIEWER)
        ptcl_streamer = make_unique<PointCloudStreamer>(PTCL_STREAM_PORT);
    
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
                LOG(WARNING) << "Skipping frame id " << capture_id;
                continue;
            }
            LOG(INFO) << "Server: Load frame time: " << sw_stage.ElapsedMs() << " ms";
                
            sw_stage.Restart();
            server->update_view();
            LOG(INFO) << "Server: Update view time: " << sw_stage.ElapsedMs() << " ms";

            sw_stage.Restart();
            int s_currFrameId = server->get_curr_frameId();
            client->set_curr_frameId(s_currFrameId);

            // Don't free the views, it is owned by the server
            vector<View *> s_views = server->get_view();
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

            if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
            {
                sw_stage.Restart();
                client->generate_pcl_object();
                LOG(INFO) << "Client: Generate pcl time: " << sw_stage.ElapsedMs() << " ms";
                
                sw_stage.Restart();
                PointCloud_t::Ptr pcl_object = client->get_pcl_object();
                
                if(FLAGS_server_cull == CULLING::NO_CULLING || 
                    FLAGS_server_cull == CULLING::CLIP_CULLING || 
                    FLAGS_server_cull == CULLING::CLIP_VOXEL_CULLING)
                {
                    FrustumClip c_frustumClip;
                    if(FLAGS_load_frustum == LOAD_FRUSTUM::DISK)
                    {
                        // Load PCL viewer's frustum from disk
                        string frustum_fname = FORMAT(capture_id << ".txt");
                        FrustumClip c_frustumClip_new;
                        if(!c_frustumClip_new.loadFromFile(path, frustum_fname))
                            LOG(FATAL) << "Failed to load frustum!";
                        
                        LOG(INFO) << "Loaded client's frustum";
                        c_frustumClip_new.print();

                        pcl_viewer->displayCullClip(pcl_object);

                        server->set_frustum_clip(c_frustumClip_new);
                    }
                    else
                    {   
                        // Use PCL viewer to get the frustum
                        pcl_viewer->displayCullClip(pcl_object);
                        if(pcl_viewer->b_camChanged)
                        {
                            // LOG(INFO) << "Client: Updating frustum";
                            pcl_viewer->b_camChanged = false;
                            client->update_frustum_clip(pcl_viewer->m_camProjViewMat);
                        }
                        c_frustumClip = client->get_frustumClip();
                        server->set_frustum_clip(c_frustumClip);
                    }

                    if(FLAGS_save_frustum)
                    {
                        string frustum_fname = FORMAT(capture_id << ".txt");
                        if(!c_frustumClip.saveToFile(path, frustum_fname))
                            LOG(FATAL) << "Failed to save frustum!";
                        
                        LOG(INFO) << "Saved client's frustum";
                        c_frustumClip.print();
                    }  
                }
                else if(FLAGS_server_cull == CULLING::NORMAL_CULLING ||
                        FLAGS_server_cull == CULLING::NORMAL_VOXEL_CULLING)
                {
                    pcl_viewer->displayCullPlane(pcl_object);
                    if(pcl_viewer->b_camChanged)
                    {
                        LOG(INFO) << "Client: Updating frustum";
                        pcl_viewer->b_camChanged = false;
                        client->update_frustum(pcl_viewer->m_camInt, pcl_viewer->m_camView);
                    }
                    Frustum c_frustum = client->get_frustum();
                    server->set_frustum(c_frustum);
                }
                else
                {
                    pcl_viewer->display(pcl_object);
                }
                LOG(INFO) << "Client: View ptcl time: " << sw_stage.ElapsedMs() << " ms";
            }

            if(FLAGS_save_ptcl)
            {
                sw_stage.Restart();
                PointCloud_t::Ptr pcl_object = client->get_pcl_object();
                client->save_ptcl_from_pcl(pcl_object);
                LOG(INFO) << "Client: Save ptcl time: " << sw_stage.ElapsedMs() << " ms";    
            }
            LOG(INFO) << "End-to-end latency: " << sw_total.ElapsedMs() << " ms";
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
}