#include <iostream>
#include <boost/filesystem.hpp>
#include <atomic>
#include <signal.h>
#include <gst/gst.h>
#include "gflags/gflags.h"

// #include "Server/MultiviewServer.h"
#include "Client/MultiviewClient.h"
// #include "Server/WebRTCServer.h"
#include "Client/NetworkReceiver.h"
#include "k4aconsts.h"
#include "pconsts.h"
#include "consts.h"
#include "timer.h"
#include "Client/MultiviewClientPoolNew.h"

// // websocket
// #include <boost/beast/core.hpp>
// #include <boost/beast/websocket.hpp>
// #include <boost/asio/strand.hpp>
// #include <cstdlib>
// #include <functional>
// #include <iostream>
// #include <memory>
// #include <string>

// namespace beast = boost::beast;         // from <boost/beast.hpp>
// namespace http = beast::http;           // from <boost/beast/http.hpp>
// namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
// namespace net = boost::asio;            // from <boost/asio.hpp>
// using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>
// // websocket end

DEFINE_int32(log_level, 0, "0 - INFO, 1 - WARNING, 2 - ERROR, 3 - FATAL.");
DEFINE_int32(view_ptcl, VIEWER::PCL_VIEWER, "View point cloud streams using PCL or Unity Viewer. 0 - No view. 1 - PCL. 2 - Unity. 3 - Open3D");
DEFINE_string(seq_name, "", "Sequence name.");
DEFINE_bool(save_ptcl, false, "Save point cloud streams to disk as .ply(s).");
// DEFINE_bool(save_views, false, "Save views to disk as .png(s).");
DEFINE_bool(load_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_int32(start_frame_id, -1, "Start frame ID.");
DEFINE_int32(end_frame_id, -1, "End frame ID.");
DEFINE_int32(ncaptures, 6000, "Number of captures to be made or loaded from disk.");
DEFINE_int32(load_frustum, LOAD_FRUSTUM::PANOPTIC, "0 - Use deafult frustum (PCL), 1 - User trace on Panoptic, 2 - User trace on Kinect (Not implemented), 3 - Load frustum from disk as .txt.");
DEFINE_int32(server_cull, CULLING::NORMAL_VOXEL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling. 5 - Expansion Culling");
DEFINE_int32(client_cull, CULLING::NORMAL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling.");
DEFINE_string(config_file, "/home/lei/rajrup/KinectStream/Multiview/config/panoptic.json", "Path to config file.");
DEFINE_int32(client_fps, 30, "Client FPS.");
DEFINE_bool(ground, false, "Ground removal.");
DEFINE_bool(tile, true, "Receiving tiled images");
DEFINE_int32(send_ptcl, 0, "0 - Normal, 1 - Ptcl only");
DEFINE_bool(save_frame, false, "Save frames to disk as .png(s).");
DEFINE_int32(cb, 20000, "Color bitrate in kbps.");
DEFINE_int32(db, 120000, "depth bitrate in kbps.");
DEFINE_string(output_dir, "", "Output directory to save frames, ptcl, views, binary mask, bitrate, etc.");
DEFINE_string(render_image_path, "", "Relative path from output_dir. Path to save render image using Open3D. Default is don't save");
DEFINE_string(save_ptcl_path, "", "Relative path from output_dir. Path to save ptcl to disk as .ply(s). Default is don't save");
DEFINE_string(save_cbitrate_file, "", "Relative path from output_dir. Path to save color bitrate to disk as .csv. Default is don't save");
DEFINE_string(save_dbitrate_file, "", "Relative path from output_dir. Path to save depth bitrate to disk as .csv. Default is don't save");
DEFINE_string(compression_type, "zstd", "Compression type. Options: zstd, draco, gpcc, None (will be supported later)");
DEFINE_int32(zstd_cl, 2, "Compression level for zstd");                     // Default: 2 in LiveScan3D
DEFINE_int32(draco_cl, 7, "Compression level for draco");                   // Default: 7 in Draco encoder demo
DEFINE_int32(draco_qp, 11, "Quantization geometric parameter for draco");   // Default: 11 in Draco encoder demo
DEFINE_bool(use_mm, false, "Use rate limiting using mahimahi. This will be used only if use_mm=true. This must be used with use_client_bitrate, so that client will send bitrate to server.");      // Deprecated. Remove later.
DEFINE_double(d2c_split, 7.0f/8.0f, "Depth to color bitrate split ratio. Default is 7:1. If use_split_adapt is false, this will be used as the fixed split ratio.");
DEFINE_bool(use_split_adapt, false, "Use depth to colot bitrate split adaptation. Use static split ratio if false.");
DEFINE_string(mm_trace_name, "", "Mahimahi trace file name.");

using namespace std;
namespace fs = boost::filesystem;

volatile sig_atomic_t flag;

std::string SERVER_HOST_NEW;
std::string WEBRTC_SERVER_HOST_NEW;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_handler(int signal_num)
{
    flag = signal_num;
}

// TODO: remove all server dependencies
int main_worker() 
{
    LOG(INFO) << "Entered Client";
    LOG(INFO) << "Press Ctrl-C to exit";

    /**
    // Kinect
    std::string date = "Feb_3_2022";
    std::string path = FORMAT(DATA_PATH << date << "/"); // RAJRUP: Save to SSD

    LOG(INFO) << "Data Path: " << path;
    LOG(INFO) << "Date: " << date;
    */

    // Panoptic

    // Playback
    // ---------------------------------------INITIALIZATION------------------------------------------------------
    while(bool x = main_worker_ready_to_init2.load(memory_order_relaxed) == false)
        sleep(0.01);

    if(FLAGS_seq_name == "")
        LOG(FATAL) << "Sequence name not provided!" << endl;

    if(FLAGS_start_frame_id < 0)
        LOG(FATAL) << "Start frame ID not provided!" << endl;

    std::string seqName = FLAGS_seq_name;
    std::string path = FORMAT(PANOPTIC_DATA_PATH << seqName << "/"); // RAJRUP: Save to SSD

    LOG(INFO) << "Data Path: " << path;
    LOG(INFO) << "Sequence Name: " << seqName;
    LOG(INFO) << "Culling Type: " << culling2str[FLAGS_client_cull];
    
    if(FLAGS_render_image_path != "")
    {
        FLAGS_render_image_path = FORMAT(FLAGS_output_dir << "/" << FLAGS_render_image_path << "log" << LOG_ID << "/");
        if(fs::exists(FLAGS_render_image_path))
        {
            // remove directory
            fs::remove_all(FLAGS_render_image_path);
            LOG(INFO) << "Directory: " << FLAGS_render_image_path << " removed";
        }
        fs::create_directories(FLAGS_render_image_path);


        // if(!fs::exists(FLAGS_render_image_path))
        // {
        //     if(fs::create_directories(FLAGS_render_image_path))
        //         LOG(INFO) << "Directory: " << FLAGS_render_image_path << " created";
        //     else
        //         LOG(FATAL) << "Directory: " << FLAGS_render_image_path << " creation failed";
        // }
    }

    if(FLAGS_save_ptcl_path != "")
    {
        FLAGS_save_ptcl_path = FORMAT(FLAGS_output_dir << "/" << FLAGS_save_ptcl_path << "log" << LOG_ID << "/");
        if(fs::exists(FLAGS_save_ptcl_path))
        {
            // remove directory
            fs::remove_all(FLAGS_save_ptcl_path);
            LOG(INFO) << "Directory: " << FLAGS_save_ptcl_path << " removed";
        }
        fs::create_directories(FLAGS_save_ptcl_path);

        // if(!fs::exists(FLAGS_save_ptcl_path))
        // {
        //     if(fs::create_directories(FLAGS_save_ptcl_path))
        //         LOG(INFO) << "Directory: " << FLAGS_save_ptcl_path << " created";
        //     else
        //         LOG(FATAL) << "Directory: " << FLAGS_save_ptcl_path << " creation failed";
        // }
    }

    if(FLAGS_save_cbitrate_file != "")
    {
        std::string path = FORMAT(FLAGS_output_dir << "/bitrate/" << "log" << LOG_ID << "/");
        if(!fs::exists(path))
        {
            if(fs::create_directories(path))
                LOG(INFO) << "Directory: " << path << " created";
            else
                LOG(FATAL) << "Directory: " << path << " creation failed";
        }
        FLAGS_save_cbitrate_file = FORMAT(path << FLAGS_save_cbitrate_file);
    }

    if(FLAGS_save_dbitrate_file != "")
    {
        std::string path = FORMAT(FLAGS_output_dir << "/bitrate/" << "log" << LOG_ID << "/");
        if(!fs::exists(path))
        {
            if(fs::create_directories(path))
                LOG(INFO) << "Directory: " << path << " created";
            else
                LOG(FATAL) << "Directory: " << path << " creation failed";
        }
        FLAGS_save_dbitrate_file = FORMAT(path << FLAGS_save_dbitrate_file);
    }
    
    std::string compression_type = FLAGS_compression_type;
    zstd_options zstd_params;
    draco_options draco_params;
    if(compression_type == "zstd")
    {
        zstd_params.compression_level = FLAGS_zstd_cl;
    }
    else if(compression_type == "draco")
    {
        draco_params.cl = FLAGS_draco_cl;
        draco_params.qp = FLAGS_draco_qp;
    }
    else
        LOG(FATAL) << "Only zstd and draco are supported. Invalid compression type!";

    int nCaptures = FLAGS_ncaptures;
    
    int start_frame_id = FLAGS_start_frame_id;
    int end_frame_id = start_frame_id + FLAGS_ncaptures - 1;
    if(FLAGS_end_frame_id > 0)
        end_frame_id = FLAGS_end_frame_id;
        
    try 
    {
        struct sigaction sa;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        sa.sa_handler = signal_handler;
        sigaction(SIGINT, &sa, 0);

        if (FLAGS_send_ptcl == 0)
        {
            LOG(INFO) << "Run RGBD pipeline...";
            MultiviewClientPoolNew clientPool(path, DATASET::PANOPTIC, start_frame_id, end_frame_id, 5);
            clientPool.run();
            // clientPool.dummy_run();
        }
        else
        {
            LOG(ERROR) << "Ptcl pipeline not implemented yet...";
        }
    }
    catch(int ex)
    {
        LOG(ERROR) << "Caught: Ctrl+C";
        LOG(ERROR) << "Exiting...";
        return EXIT_FAILURE;
    }

    sleep(2);
	return EXIT_SUCCESS;
}

// Websocket connecting Multiview Server and Client
void websocket_worker()
{
    unique_ptr<NetworkSocketClient> client = std::make_unique<NetworkSocketClient>(SERVER_HOST_NEW.c_str(), 8080, FLAGS_send_ptcl);
    LOG(INFO) << "WebSocket Client connected to Server";
    try {
        struct sigaction sa;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        sa.sa_handler = signal_handler;
        sigaction(SIGINT, &sa, 0);

        client->run();
    }
    catch(int ex)
    {
        LOG(ERROR) << "Caught: Ctrl+C";
        LOG(ERROR) << "Exiting...";
        return;
    }

    LOG(INFO) << "Websocket Client Initialized" << endl;
    // client->get_calibration();
    // client->sendFrustum();
}

int main(int argc, char** argv)
{
    gflags::SetUsageMessage("Usage: MultiviewClient [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    absl::SetMinLogLevel((absl::LogSeverityAtLeast)FLAGS_log_level);

    // Initialize Gstreamer
    gst_init(&argc, &argv);
    
    std::string config_file = FLAGS_config_file;
    parse_config(config_file);

    if(FLAGS_view_ptcl != VIEWER::OPEN3D_VIEWER && FLAGS_render_image_path != "")
        LOG(FATAL) << "Please set --view_ptcl=3 for Open3D viewer to render image";
    
    SERVER_HOST_NEW = "68.181.32.205";
    WEBRTC_SERVER_HOST_NEW = SERVER_HOST_NEW;

	thread t1(main_worker);
    thread t3(websocket_worker);

    // if (FLAGS_send_ptcl == 0) 
    // {
    //     thread t2(webrtc_client_worker);
    //     t2.join();
    // }

    t1.join();
    t3.join();


    /**
     * @brief DEBUG
     * Motivation example or
     * Dump RGBD frames to disk in .mp4 format
     * Also remember to comment sending mask to client in WebRTCClientPool::recv() function in WebRTCServer.cpp
     */

    // thread t1(main_worker);
    // if (!FLAGS_send_ptcl) 
    // {
    //     thread t2(webrtc_client_worker);
    //     t2.join();
    // }

    // t1.join();
}
