#include <iostream>
#include <boost/filesystem.hpp>
#include <atomic>
#include <gst/gst.h>
#include "gflags/gflags.h"

#include "Server/MultiviewServer.h"
#include "Server/NetworkSender.h"
#include "k4aconsts.h"
#include "pconsts.h"
#include "consts.h"
#include "timer.h"
#include "Server/MultiviewServerPoolNew.h"

// // websocket
// #include <boost/beast/core.hpp>
// #include <boost/beast/websocket.hpp>
// #include <boost/asio/dispatch.hpp>
// #include <boost/asio/strand.hpp>

// namespace beast = boost::beast;         // from <boost/beast.hpp>
// namespace http = beast::http;           // from <boost/beast/http.hpp>
// namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
// namespace net = boost::asio;            // from <boost/asio.hpp>
// using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

// TODO: Change these global definitions to sys args
DEFINE_int32(log_level, 0, "0 - INFO, 1 - WARNING, 2 - ERROR, 3 - FATAL.");
DEFINE_string(seq_name, "", "Sequence name.");
DEFINE_bool(capture, false, "True = Live mode. Capture from camera. False = Playback mode. Load point cloud sequence from disk.");
DEFINE_bool(save_ptcl, false, "Save point cloud streams to disk as .ply(s).");
DEFINE_bool(save_views, false, "Save views to disk as .png(s).");
DEFINE_string(save_views_subfolder, "test", "Subfolder to save views.");
DEFINE_int32(start_frame_id, -1, "Start frame ID.");
DEFINE_int32(end_frame_id, -1, "End frame ID.");
DEFINE_int32(ncaptures, 6000, "Number of captures to be made or loaded from disk.");
DEFINE_int32(server_cull, CULLING::NORMAL_VOXEL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling. 5 - Expansion Culling");
DEFINE_bool(preload, false, "Preload frames into memory.");
DEFINE_int32(load_frustum, LOAD_FRUSTUM::PANOPTIC, "0 - Use deafult frustum (PCL), 1 - User trace on Panoptic, 2 - User trace on Kinect (Not implemented), 3 - Load frustum from disk as .txt.");
DEFINE_bool(save_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_bool(tile, true, "Sending tiled images");
DEFINE_string(config_file, "/home/lei/rajrup/KinectStream/Multiview/config/panoptic.json", "Path to config file.");
DEFINE_bool(ground, false, "Ground removal.");
DEFINE_int32(send_ptcl, 0, "0 - Normal, 1 - Ptcl only");
DEFINE_bool(save_frame, false, "Save frames to disk as .png(s).");
DEFINE_int32(cb, 20000, "Color bitrate in kbps.");
DEFINE_int32(db, 120000, "depth bitrate in kbps.");
DEFINE_int32(cqp, 0, "Color QP.");
DEFINE_int32(dqp, 0, "Depth QP.");
DEFINE_int32(server_fps, 30, "Rate limited fps on server side");
DEFINE_string(compression_type, "zstd", "Compression type. Options: zstd, draco, gpcc, None (will be supported later)");
DEFINE_int32(zstd_cl, 2, "Compression level for zstd");                     // Default: 2 in LiveScan3D
DEFINE_int32(draco_cl, 7, "Compression level for draco");                   // Default: 7 in Draco encoder demo
DEFINE_int32(draco_qp, 11, "Quantization geometric parameter for draco");   // Default: 11 in Draco encoder demo
DEFINE_bool(use_mm, false, "Use mahimahi for server side rate limiting.");
DEFINE_bool(use_server_bitrate_est, false, "Use server side bitrate estimation when mahimahi is used. This will be used only if use_mm=true.");     // Default: Use server side bitrate estimation
// DEFINE_bool(use_client_bitrate, false, "Use client side bitrate estimation when mahimahi is used. This will be used only if use_mm=true.");
DEFINE_bool(use_client_bitrate_split, false, "Use bitrate split sent by client.");
DEFINE_string(output_dir, "", "Output directory to save frames, ptcl, views, binary mask, bitrate, etc.");
DEFINE_string(mm_trace_name, "", "Mahimahi trace file name.");

using namespace std;
namespace fs = boost::filesystem;

volatile sig_atomic_t flag;

std::string SERVER_HOST_NEW;
std::string WEBRTC_SERVER_HOST_NEW;

/**
 * @brief 
 * ZSTD: 9 compression levels from 1 (fast) - 9 (slow and strong). Compression config: ./config/zstd_params.json
 * ./build/Multiview/PanopticPtcl --view_ptcl=3 --save_binary_mask=false --load_binary_mask=false --start_frame_id=59 --ncaptures=2000 --seq_name=160317_moonbaby1_with_ground --client_cull=0 --compression_type=zstd --render_image_path=/home/lei/data/pipeline/client_standalone/pipeline_new/160317_moonbaby1_with_ground/o3d_zstd_cl2/ --save_ptcl=true
 * 
 * DRACO: compression level (cl) from 0 (lossless, fastest) - 10 (slowest), quantization geometric (qp) and texture (qt) from 0 (lossless) - 30 (worst). Compression config: ./config/draco_params.json
 * ./build/Multiview/PanopticPtcl --view_ptcl=3 --save_binary_mask=false --load_binary_mask=false --start_frame_id=59 --ncaptures=2000 --seq_name=160317_moonbaby1_with_ground --client_cull=0 --compression_type=draco --render_image_path=/home/lei/data/pipeline/client_standalone/pipeline_new/160317_moonbaby1_with_ground/test/ --save_ptcl=true
 * 
 * GPCC: 
 */

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_handler(int signal_num)
{
    flag = signal_num;
}

int main_worker() 
{
    LOG(INFO) << "Entered Server";
    LOG(INFO) << "Press Ctrl-C to exit";

    /**
    // Kinect
    std::string date = "Feb_3_2022";
    std::string path = FORMAT(DATA_PATH << date << "/"); // RAJRUP: Save to SSD

    LOG(INFO) << "Data Path: " << path;
    LOG(INFO) << "Date: " << date;
    */

    // Panoptic
    std::string seqName = FLAGS_seq_name;
    std::string path = FORMAT(PANOPTIC_DATA_PATH << seqName << "/"); // RAJRUP: Save to SSD
    
    LOG(INFO) << "Data Path: " << path;
    LOG(INFO) << "Sequence Name: " << seqName;

    // Creating a directory
    if(fs::exists(path))
        LOG(WARNING) << "Directory: " << path << " already exists";
    else
    {
        if(fs::create_directories(path))
            LOG(INFO) << "Directory: " << path << " created";
        else
            LOG(FATAL) << "Directory: " << path << " creation failed";
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

    // int startFrame = 59;
    // int endFrame = 6193;
    int nCaptures = FLAGS_ncaptures;

    if(FLAGS_capture) 
    {
        unique_ptr<MultiviewServer> server = std::make_unique<MultiviewServer>(path, DATASET::PANOPTIC);

        server->set_work_dir(path);
        if(!server->init_capture())
            LOG(FATAL) << "Failed to initialize capture!";

        if(server->get_calibration_requirement())
        {
            if(!server->calibrate())
                LOG(FATAL) << "Failed to calibrate!";

            if(!server->save_calibration())
                LOG(FATAL) << "Failed to save calibration!";

        }

        // Register Async Signal Handler
        try 
        {
            // struct sigaction sa;
            // sigemptyset(&sa.sa_mask);
            // sa.sa_flags = 0;
            // sa.sa_handler = signal_handler;
            // sigaction(SIGINT, &sa, 0);

            for(uint32_t i = 0; i < nCaptures; i++)
            {
                // if (flag) 
                //     throw (int)flag;

                if(i % 10 == 0)
                    LOG(INFO) << "Capturing frame " << i;

                if(server->capture_frame())
                {
                    server->set_curr_frameId(i);
                    if(!server->store_frame())
                    {
                        LOG(FATAL) << "Failed to store frame!";
                        return -1;
                    }
                }
                else
                {
                    LOG(FATAL) << "Failed to capture frame!";
                    return -1;
                }
            }
        }
        catch(int ex)
        {
            LOG(ERROR) << "Caught: Ctrl+C";
            LOG(ERROR) << "Exiting...";
            return EXIT_FAILURE;
        }
    }
    else 
    {
        try 
        {
            struct sigaction sa;
            sigemptyset(&sa.sa_mask);
            sa.sa_flags = 0;
            sa.sa_handler = signal_handler;
            sigaction(SIGINT, &sa, 0);

            int start_frame_id = FLAGS_start_frame_id;
            int end_frame_id = start_frame_id + FLAGS_ncaptures - 1;
            if(FLAGS_end_frame_id > 0)
                end_frame_id = FLAGS_end_frame_id;
            
            if (FLAGS_send_ptcl == 0) 
            {
                LOG(INFO) << "Run RGBD pipeline...";
                MultiviewServerPoolNew serverPool(path, DATASET::PANOPTIC, start_frame_id, end_frame_id, 3);
                serverPool.run();
            }
            else 
            {
                LOG(ERROR) << "Ptcl pipeline not implemented...";
            }
        }
        catch(int ex)
        {
            LOG(ERROR) << "Caught: Ctrl+C";
            LOG(ERROR) << "Exiting...";
            return EXIT_FAILURE;
        }
    }
    sleep(2);
	return EXIT_SUCCESS;
}

// Websocket connecting Multiview Server and Client
void websocket_worker() 
{
    unique_ptr<NetworkSocketServer> server = std::make_unique<NetworkSocketServer>(SERVER_HOST_NEW.c_str(), 8080, FLAGS_send_ptcl);

    // Wait until calibration info is loaded
    while(bool x = wsserver_ready_to_init2.load(memory_order_relaxed) == false)
        sleep(0.01);

    LOG(INFO) << "websocket server";

    try 
    {
        struct sigaction sa;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        sa.sa_handler = signal_handler;
        sigaction(SIGINT, &sa, 0);

        server->run();
    }
    catch(int ex)
    {
        LOG(ERROR) << "Caught: Ctrl+C";
        LOG(ERROR) << "Exiting...";
        return;
    }
}

int main(int argc, char** argv)
{
    gflags::SetUsageMessage("Usage: MultiviewServer [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    absl::SetMinLogLevel((absl::LogSeverityAtLeast)FLAGS_log_level);

    // Initialize Gstreamer
    gst_init(&argc, &argv);

    std::string config_file = FLAGS_config_file;
    parse_config(config_file);

    if(FLAGS_seq_name == "")
        LOG(FATAL) << "Sequence name not provided!";

    if(FLAGS_start_frame_id < 0)
        LOG(FATAL) << "Start frame ID not provided!";

    SERVER_HOST_NEW = "68.181.32.205";  // Default server IP
    if(FLAGS_use_mm)
    {
        LOG(INFO) << "Using mahimahi for server side rate limiting";
        SERVER_HOST_NEW = "100.64.0.2";
        
        // if(!FLAGS_use_server_bitrate_est)
        //     LOG(FATAL) << "use_server_bitrate_est should be true when use_mm is true!";   
    }
    
    WEBRTC_SERVER_HOST_NEW = SERVER_HOST_NEW;

    thread t1(main_worker);
    thread t2(websocket_worker);
    
    t1.join();
    t2.join();
    
    /**
     * @brief DEBUG
     * Motivation example or
     * Dump RGBD frames to disk in .mp4 format
     * Also remember to comment sending mask to client in WebRTCServerPool::send() function in WebRTCServer.cpp
     */
    
    // main_worker();
}
