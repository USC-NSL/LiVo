#include <iostream>
#include <boost/filesystem.hpp>
#include <atomic>
#include "gflags/gflags.h"

#include "Server/MultiviewServer.h"
#include "Client/MultiviewClient.h"
#include "Server/WebRTCServer.h"
#include "Client/WebRTCClient.h"
#include "k4aconsts.h"
#include "pconsts.h"
#include "consts.h"
#include "timer.h"

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
DEFINE_bool(capture, false, "True = Live mode. Capture from camera. False = Playback mode. Load point cloud sequence from disk.");
DEFINE_bool(save_ptcl, false, "Save point cloud streams to disk as .ply(s).");
DEFINE_bool(save_views, false, "Save views to disk as .png(s).");
DEFINE_int32(ncaptures, 1000, "Number of captures to be made or loaded from disk.");
DEFINE_int32(server_cull, 2, "0 - Clip culling. 1 - Voxel culling. 2 - No culling.");
DEFINE_bool(preload, false, "Preload frames into memory.");

// Frustum. Note that only one of the mode should be enabled
// Update frustum from Client. Note
DEFINE_bool(update_frustum, false, "Update frustum from client.");
// Motivation example: load frustum and generate frames
DEFINE_bool(load_frustum, false, "Load frustum from disk as .txt.");

DEFINE_bool(tile, true, "Sending tiled images");

using namespace std;
namespace fs = boost::filesystem;

volatile sig_atomic_t flag;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_handler(int signal_num)
{
    flag = signal_num;
}

int main_worker() {
    cout << "Entered Client" << endl;
    cout << "Press Ctrl-C to exit" << endl;

    // // Kinect
    // string date = "Feb_3_2022";
    // string path = FORMAT(DATA_PATH << date << "/"); // RAJRUP: Save to SSD

    // cout << "Data Path: " << path << endl;
    // cout << "Date: " << date << endl;

    // Panoptic
    string seqName = "160317_moonbaby1";
    string path = FORMAT(PANOPTIC_DATA_PATH << seqName << "/"); // RAJRUP: Save to SSD

    cout << "Data Path: " << path << endl;
    cout << "Sequence Name: " << seqName << endl;


    // Creating a directory
    if(fs::exists(path))
        cout << "WARNING :  " << "Directory already exists" << endl;
    else
    {
        if(fs::create_directories(path))
            cout << "Directory created" << endl;
        else
        {
            cout << "ERROR: " << "Directory creation failed" << endl;
            return -1;
        }
    }

    int startFrame = 59;
    int endFrame = 6193;
    int nCaptures = FLAGS_ncaptures;

    if(FLAGS_capture) 
    {
        unique_ptr<MultiviewServer> server = std::make_unique<MultiviewServer>(path, PANOPTIC);

        server->set_work_dir(path);
        if(!server->init_capture())
        {
            cout << "Failed to initialize capture!" << endl;
            return -1;
        }

        if(server->get_calibration_requirement())
        {
            if(!server->calibrate())
            {
                cout << "Failed to calibrate!" << endl;
                return -1;
            }

            if(!server->save_calibration())
            {
                cout << "Failed to save calibration!" << endl;
                return -1;
            }
        }

        // Register Async Signal Handler
        try 
        {
            struct sigaction sa;
            sigemptyset(&sa.sa_mask);
            sa.sa_flags = 0;
            sa.sa_handler = signal_handler;
            sigaction(SIGINT, &sa, 0);

            for(uint32_t i = 0; i < nCaptures; i++)
            {
                if (flag) 
                    throw (int)flag;

                if(i%10 == 0)
                    cout << "Capturing frame " << i << endl;

                if(server->capture_frame())
                {
                    server->set_curr_frameId(i);
                    if(!server->store_frame())
                    {
                        cout << "Failed to store frame!" << endl;
                        return -1;
                    }
                }
                else
                {
                    cout << "Failed to capture frame!" << endl;
                    return -1;
                }
            }
        }
        catch(int ex)
        {
            cout << "Caught: Ctrl+C" << endl;
            cout << "Exiting..." << endl;
            return EXIT_FAILURE;
        }
    }
    else 
    {
        // playback

        // ---------------------------------------INITIALIZATION------------------------------------------------------
        unique_ptr<MultiviewServer> server = make_unique<MultiviewServer>(path, PANOPTIC);

        if(!server->init_playback())
        {
            cout << "Server: Failed to initialize playback!" << endl;
            return -1;
        }

        if(server->get_calibration_requirement())
        {
            if(!server->load_calibration())
            {
                cout << "Server: Failed to load calibration!" << endl;
                return -1;
            }
        }

        vector<uint32_t> s_deviceIndices = server->get_device_indices();
        vector<string> s_deviceSerialNumbers = server->get_device_serial_numbers();
        int s_numDevices = server->get_device_count();
        if(s_numDevices <= 0)
        {
            cout << "Server: No devices found during playback!" << endl;
            return -1;
        }

        if(s_numDevices != s_deviceIndices.size() || s_numDevices != s_deviceSerialNumbers.size())
        {
            cout << "Server: Device count mismatch during playback!" << endl;
            return -1;
        }
        
        // Don't free the calibration data, it is owned by the server
        vector<Calibration *> s_calibrations = server->get_calibration();

        if(!server->get_calibration_status())
        {
            cout << "Server: Failed to load Calibration!" << endl;
            return -1;
        }

        vector<xy_table_t *> s_xy_tables = server->get_xy_table();

        // copy s_calibrations and s_xy_tables to send to client through websocket
        calibrations_init = s_calibrations;
        xy_tables_init = s_xy_tables;
        deviceIndices_init = s_deviceIndices;
        deviceSerialNumbers_init = s_deviceSerialNumbers;

        // Send signal to websocket server to send calibration info to client
        wsserver_ready_to_init.store(true);

        // Preload
        if (FLAGS_preload) {
            if (!server->preload()) {
                cout << "Server: Failed to preload frames into memory!" << endl;
                return -1;
            }
        }
        
        // Initialize WebRTC Server
        unique_ptr<WebRTCServerPool> wserver = make_unique<WebRTCServerPool>(server->get_device_count(), FLAGS_tile);

        cout << "WebRTC server init done" << endl;
        
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

            // server->set_curr_frameId(startFrame - 1); // Start with 0, since we increment it in the loop
            for(uint32_t capture_id = startFrame; capture_id <= endFrame; capture_id++)
            {   
                if (flag) 
                    throw (int)flag;

                cout << "-----------------------------------------------------------------------" << endl;
                cout << "Current Unix Timestamp: " << sw_total.Curr() << " ms for frame " << capture_id << "\n";
                sw_total.Restart();
                sw_stage.Restart();

                server->set_curr_frameId(capture_id);
                cout << "Server: Starting with frame: " << capture_id << endl;

                if(!server->load_frame_parallel())
                {
                    cout << "Skipping frame id " << capture_id << endl;
                    continue;
                }
                cout << "Server: Load frame time: " << sw_stage.ElapsedMs() << " ms" << endl;

                sw_stage.Restart();
                server->update_view();
                cout << "Server: Update view time: " << sw_stage.ElapsedMs() << " ms" << endl;

                // Debug: generate frames with offline frustums
                if(FLAGS_load_frustum) 
                {
                    string frustum_fname = FORMAT(capture_id << ".txt");
                    FrustumClip c_frustumClip_new;
                    if(!c_frustumClip_new.loadFromFile(path, frustum_fname))
                    {
                        cout << "Failed to load frustum!" << endl;
                        return -1;
                    }
                    cout << "Loaded frustum from local" << endl;
                    c_frustumClip_new.print();
                    cout << endl;

                    server->set_frustum_clip(c_frustumClip_new);
                }
;
                
                sw_stage.Restart();
                int s_currFrameId = server->get_curr_frameId();
            
                vector<View *> s_views = server->get_view();

                // cout << "Server view generated" << endl;

                // Webrtc Server send views
                wserver->send(capture_id, s_views);
                cout << "Server: Send views time: " << sw_stage.ElapsedMs() << " ms" << endl;

                if(FLAGS_update_frustum) 
                {
                    sw_stage.Restart();
                    // Set new frustum received from Client
                    if(bool x = new_frustumClip.load(memory_order_relaxed) == true) {
                        cout << "Server setting up new frustums" << endl;
                        server->set_frustum_clip(frustumClip_recv);
                        new_frustumClip.store(false);
                    } 
                    cout << "Server: Update frustum time: " << sw_stage.ElapsedMs() << " ms" << endl;   
                }
                cout << "Server side latency: " << sw_total.ElapsedMs() << " ms" << endl;
            }
        }
        catch(int ex)
        {
            cout << "Caught: Ctrl+C" << endl;
            cout << "Exiting..." << endl;
            return EXIT_FAILURE;
        }
    }

    sleep(2);
	return EXIT_SUCCESS;

}


// Websocket connecting Multiview Server and Client
void websocket_worker() {
    unique_ptr<WebSocketServer> server = std::make_unique<WebSocketServer>("127.0.0.1", 8080);

    // Wait until calibration info is loaded
    while(bool x = wsserver_ready_to_init.load(memory_order_relaxed) == false) {
        sleep(0.01);
    }

    server->run();
}

int main(int argc, char** argv)
{
    gflags::SetUsageMessage("Usage: MultiviewServer [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    thread t1(main_worker);
    thread t2(websocket_worker);
    
    t1.join();
    t2.join();
    
    // DEBUG: motivation example
    // main_worker();
}
