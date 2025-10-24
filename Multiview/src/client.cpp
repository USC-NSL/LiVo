#include <iostream>
#include <boost/filesystem.hpp>
#include <atomic>
#include <signal.h>
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

DEFINE_bool(view_ptcl, true, "View point cloud streams using PCL Viewer.");
DEFINE_bool(save_ptcl, false, "Save point cloud streams to disk as .ply(s).");
// DEFINE_bool(save_views, false, "Save views to disk as .png(s).");
DEFINE_bool(load_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_int32(ncaptures, 1000, "Number of captures to be made or loaded from disk.");
DEFINE_bool(save_frustum, false, "Save frustum to disk as .txt.");
DEFINE_bool(load_frustum, false, "Load frustum from disk as .txt.");
DEFINE_bool(update_frustum, false, "Use frustum culling in client.");
DEFINE_int32(client_cull, 2, "0 - Clip culling. 1 - Voxel culling. 2 - No culling.");

DEFINE_bool(tile, true, "Receiving tiled images");

using namespace std;
namespace fs = boost::filesystem;

volatile sig_atomic_t flag;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_handler(int signal_num)
{
    flag = signal_num;
}

// TODO: remove all server dependencies
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
    // playback
    
    // ---------------------------------------INITIALIZATION------------------------------------------------------
    // unique_ptr<MultiviewServer> server = make_unique<MultiviewServer>(path);
    unique_ptr<MultiviewClient> client = make_unique<MultiviewClient>(path, PANOPTIC);

    unique_ptr<PointCloudViewer> pcl_viewer(nullptr);
        
    if(FLAGS_view_ptcl)
        pcl_viewer = make_unique<PointCloudViewer>("viewer");

    while(bool x = main_worker_ready_to_init.load(memory_order_relaxed) == false) {
        sleep(0.01);
    }

    client->init(deviceIndices_recv, deviceSerialNumbers_recv, calibrations_recv);

    if(!client->get_calibration_status())
    {
        cout << "Client: Failed to load Calibration!" << endl;
        return -1;
    }

    // vector<xy_table_t *> s_xy_tables = server->get_xy_table();
    // client->set_xy_table(xy_tables_recv);

    // Initialize WebRTC Client
    // wdevice_num = server->get_device_count();
    wclient_ready_to_init.store(true);
    client->set_curr_frameId(startFrame - 1);

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
            cout << "-----------------------------------------------------------------------" << endl;   
            sw_total.Restart();
            if (flag) 
                throw (int)flag;

            sw_stage.Restart();
            // The running WebRTC client should send signal here for client->set_view()
            while(bool x = next_frame_ready.load(memory_order_relaxed) == false) {
                sleep(0.001);
            }
            cout << "Client: Waiting for next frame: " << sw_stage.ElapsedMs() << " ms" << endl;

            sw_stage.Restart();
            // if(!client->set_view(next_views[next_frame % POOL_LENGTH]))
            if(!client->set_view(next_views))
            {
                cout << "Client: Failed to set view!" << endl;
                return -1;
            }
            cout << "Client: Set view time: " << sw_stage.ElapsedMs() << " ms" << endl;

            if(FLAGS_load_binary_mask)
                client->load_binary_mask(capture_id);

            sw_stage.Restart();
            next_frame_ready.store(false);

            // cout << "Generating Point Cloud for frame: " << capture_id << endl;
            if(!client->update_ptcl())
            {
                cout << "Client: Failed to update ptcl!" << endl;
                return -1;
            }
            cout << "Client: Update ptcl time: " << sw_stage.ElapsedMs() << " ms" << endl;

            sw_stage.Restart();
            client->generate_pcl_object();
            cout << "Client: Generate pcl object time: " << sw_stage.ElapsedMs() << " ms" << endl;

            if(FLAGS_view_ptcl)
            {
                sw_stage.Restart();
                PointCloud_t::Ptr pcl_object = client->get_pcl_object();
                pcl_viewer->displayCullClip(pcl_object);
                if(pcl_viewer->b_camChanged)
                {
                    pcl_viewer->b_camChanged = false;
                    client->update_frustum(pcl_viewer->m_camInt, pcl_viewer->m_camView);
                }
                FrustumClip c_frustumClip = client->get_frustumClip();
                
                if(FLAGS_save_frustum)
                {
                    string frustum_fname = FORMAT(capture_id << ".txt");
                    if(!c_frustumClip.saveToFile(path, frustum_fname))
                    {
                        cout << "Failed to save frustum!" << endl;
                        return -1;
                    }
                    cout << "Saved client's frustum" << endl;
                    c_frustumClip.print();
                    cout << endl;
                }

                if(FLAGS_load_frustum)
                {
                    string frustum_fname = FORMAT(capture_id << ".txt");
                    FrustumClip c_frustumClip_new;
                    if(!c_frustumClip_new.loadFromFile(path, frustum_fname))
                    {
                        cout << "Failed to load frustum!" << endl;
                        return -1;
                    }
                    cout << "Loaded client's frustum" << endl;
                    c_frustumClip_new.print();
                    cout << endl;
                    // TODO: send to server
                }
                else
                cout << "Client: View ptcl time: " << sw_stage.ElapsedMs() << " ms" << endl;
            }

            if(FLAGS_save_ptcl)
            {
                sw_stage.Restart();
                client->save_ptcl();
                cout << "Client: Save ptcl time: " << sw_stage.ElapsedMs() << " ms" << endl;    
            }
            cout << "End-to-end latency: " << sw_total.ElapsedMs() << " ms" << endl;



            // if (FLAGS_view_ptcl) 
            // {
            //     sw_stage.Restart();
            //     client->view_ptcl();
            //     FrustumClip c_frustumClip = client->get_frustumClip();
            //     cout << "Client: View ptcl time: " << sw_stage.ElapsedMs() << " ms" << endl;
            //     // // Currently only send new frustum if the last one is already sent. TODO: push new frustums to a queue.
            //     // if(bool x = new_frustumClip.load(memory_order_relaxed) == false) {
            //     //     frustumClip_send = c_frustumClip;
            //     //     new_frustumClip.store(true);
            //     // } else {
            //     //     cout << "Warning: last frustum is not finished, bypassing new one." << endl;
            //     // }


            //     // // Debug: load/save frustum at local
            //     // string frustum_fname = FORMAT(capture_id << ".txt");
            //     // if(FLAGS_save_frustum)
            //     // {
            //     //     if(!c_frustumClip.saveToFile(path, frustum_fname))
            //     //     {
            //     //         cout << "Failed to save frustum!" << endl;
            //     //         return -1;
            //     //     } else {
            //     //         cout << "Warning: last frustum is not finished, bypassing new one." << endl;
            //     //     }
            //     //     cout << "Saved client's frustum" << endl;
            //     //     c_frustumClip.print();
            //     //     cout << endl;
            //     // }

            //     // FrustumClip c_frustumClip_new;
            //     // if(FLAGS_load_frustum)
            //     // {
            //     //     if(!c_frustumClip_new.loadFromFile(path, frustum_fname))
            //     //     {
            //     //         cout << "Failed to load frustum!" << endl;
            //     //         return -1;
            //     //     }
            //     //     cout << "Loaded client's frustum" << endl;
            //     //     c_frustumClip_new.print();
            //     //     cout << endl;

            //     //     // Currently only send new frustum if the last one is already sent. TODO: push new frustums to a queue.
            //     //     if(bool x = new_frustumClip.load(memory_order_relaxed) == false) {
            //     //         frustumClip_send = c_frustumClip;
            //     //         new_frustumClip.store(true);
            //     //     } else {
            //     //         cout << "Warning: last frustum is not finished, bypassing new one." << endl;
            //     //     }
                    
            //     //     // server->set_frustum_clip(c_frustumClip_new);
            //     // }
            //     // else
            //         // server->set_frustum_clip(c_frustumClip);
            // }


            // if(FLAGS_save_ptcl)
            // {
            //     sw_stage.Restart();
            //     client->save_ptcl();
            //     cout << "Client: Save ptcl time: " << sw_stage.ElapsedMs() << " ms" << endl; 
            // }
            // cout << "Client side latency: " << sw_total.ElapsedMs() << " ms" << endl;  
        }
    }
    catch(int ex)
    {
        cout << "Caught: Ctrl+C" << endl;
        cout << "Exiting..." << endl;
        return EXIT_FAILURE;
    }
    
    sleep(2);
	return EXIT_SUCCESS;

}

// WebRTC Client
void webrtc_client_worker() {
    // Initialize WebRTC Client
    while(bool x = wclient_ready_to_init.load(memory_order_relaxed) == false) {
        sleep(0.01);
    }

    unique_ptr<WebRTCClientPool> wclient = make_unique<WebRTCClientPool>(wdevice_num, FLAGS_tile);
    cout << "WebRTCClientPool Initialized" << endl;

    wclient->run();
}

// Websocket connecting Multiview Server and Client
void websocket_worker() {
    unique_ptr<WebSocketClient> client = std::make_unique<WebSocketClient>("127.0.0.1", 8080);
    cout << "Websocker Client Connected" << endl;
    client->run();
    cout << "Websocket Client Initialized" << endl;
    // client->get_calibration();
    // client->sendFrustum();
}

int main(int argc, char** argv)
{
    gflags::SetUsageMessage("Usage: MultiviewServer [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);

	thread t1(main_worker);
    thread t2(webrtc_client_worker);
    thread t3(websocket_worker);

    t1.join();
    t2.join();
    t3.join();
}
