#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <boost/filesystem.hpp>

#include "Server/MultiviewServer.h"
#include "Client/MultiviewClient.h"
#include "Client/PointCloudStreamer.h"
#include "k4aconsts.h"
#include "consts.h"
#include "timer.h"

DEFINE_bool(capture, false, "True = Live mode. Capture from camera. False = Playback mode. Load point cloud sequence from disk.");
DEFINE_int32(view_ptcl, VIEWER::NO_VIEWER, "View point cloud streams using PCL or Unity Viewer. 0 - No view. 1 - PCL. 2 - Unity.");
DEFINE_bool(save_ptcl, false, "Save point cloud streams to disk as .ply(s).");
DEFINE_bool(save_views, false, "Save views to disk as .png(s).");
DEFINE_int32(ncaptures, 100, "Number of captures to be made or loaded from disk.");
DEFINE_bool(save_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_bool(load_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_int32(server_cull, CULLING::NO_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling.");
DEFINE_int32(client_cull, CULLING::NO_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling.");
DEFINE_bool(save_frustum, false, "Save frustum to disk as .txt.");
DEFINE_int32(load_frustum, LOAD_FRUSTUM::NONE, "0 - Use deafult frustum (PCL), 1 - User trace on Panoptic, 2 - User trace on Kinect (Not implemented), 3 - Load frustum from disk as .txt.");
DEFINE_bool(update_frustum, false, "Placeholder.");

using namespace std;
namespace fs = boost::filesystem;

volatile sig_atomic_t flag;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_handler(int signal_num)
{
    flag = signal_num;
}

int main(int argc, char** argv)
{
	cout << "Entered Client" << endl;
    cout << "Press Ctrl-C to exit" << endl;

    gflags::SetUsageMessage("Usage: MultiviewServerClient [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // string date = get_date();
    string date = "Feb_3_2022";
    string path = FORMAT(DATA_PATH << date << "/"); // RAJRUP: Save to SSD

    cout << "Data Path: " << path << endl;
    cout << "Date: " << date << endl;

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

    int nCaptures = FLAGS_ncaptures;

    if(FLAGS_capture)
    {   
        // Capture from camera

        unique_ptr<MultiviewServer> server = std::make_unique<MultiviewServer>(path);
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
        // Playback mode

        // ---------------------------------------INITIALIZATION------------------------------------------------------
        unique_ptr<MultiviewServer> server = make_unique<MultiviewServer>(path);
        unique_ptr<MultiviewClient> client = make_unique<MultiviewClient>(path);
        
        unique_ptr<PointCloudViewer> pcl_viewer(nullptr);
        
        if(FLAGS_view_ptcl)
            pcl_viewer = make_unique<PointCloudViewer>("viewer");

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

        client->init(s_deviceIndices, s_deviceSerialNumbers, s_calibrations);

        if(!server->get_calibration_status())
        {
            cout << "Server: Failed to load Calibration!" << endl;
            return -1;
        }

        if(!client->get_calibration_status())
        {
            cout << "Client: Failed to load Calibration!" << endl;
            return -1;
        }

        vector<xy_table_t *> s_xy_tables = server->get_xy_table();
        client->set_xy_table(s_xy_tables);

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

            for(uint32_t capture_id = 1; capture_id <= nCaptures; capture_id++)
            {   
                sw_total.Restart();
                if (flag) 
                    throw (int)flag;

                cout << "-----------------------------------------------------------------------" << endl;
                
                sw_stage.Restart();
                server->set_curr_frameId(capture_id);
                cout << "Server: Starting with frame: " << capture_id << endl;
                if(!server->load_frame_parallel())
                {
                    cout << "Server: Failed to load frame!" << endl;
                    return -1;
                }
                cout << "Server: Load frame time: " << sw_stage.ElapsedMs() << " ms" << endl;
                
                sw_stage.Restart();
                server->update_view();
                cout << "Server: Update view time: " << sw_stage.ElapsedMs() << " ms" << endl;

                sw_stage.Restart();
                int s_currFrameId = server->get_curr_frameId();
                client->set_curr_frameId(s_currFrameId);

                // Don't free the views, it is owned by the server
                vector<View *> s_views = server->get_view();
                if(!client->set_view(s_views))
                {
                    cout << "Client: Failed to set view!" << endl;
                    return -1;
                }
                cout << "Client: Set view time: " << sw_stage.ElapsedMs() << " ms" << endl;

                sw_stage.Restart();
                // cout << "Generating Point Cloud for frame: " << capture_id << endl;
                if(!client->update_ptcl())
                {
                    cout << "Client: Failed to update ptcl!" << endl;
                    return -1;
                }
                cout << "Client: Update ptcl time: " << sw_stage.ElapsedMs() << " ms" << endl;

                if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
                {
                    sw_stage.Restart();
                    client->generate_pcl_object();
                    cout << "Client: Generate pcl time: " << sw_stage.ElapsedMs() << " ms" << endl;

                    sw_stage.Restart();
                    PointCloud_t::Ptr pcl_object = client->get_pcl_object();

                    if(FLAGS_server_cull == CULLING::NO_CULLING || 
                    FLAGS_server_cull == CULLING::CLIP_CULLING || 
                    FLAGS_server_cull == CULLING::CLIP_VOXEL_CULLING)
                    {
                        pcl_viewer->displayCullClip(pcl_object);
                        if(pcl_viewer->b_camChanged)
                        {
                            // cout << "Client: Updating frustum" << endl;
                            pcl_viewer->b_camChanged = false;
                            client->update_frustum_clip(pcl_viewer->m_camProjViewMat);
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

                        if(FLAGS_load_frustum == LOAD_FRUSTUM::DISK)
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

                            server->set_frustum_clip(c_frustumClip_new);
                        }
                        else
                            server->set_frustum_clip(c_frustumClip);
                    }
                    else if(FLAGS_server_cull == CULLING::NORMAL_CULLING ||
                        FLAGS_server_cull == CULLING::NORMAL_VOXEL_CULLING)
                    {
                        pcl_viewer->displayCullPlane(pcl_object);
                        if(pcl_viewer->b_camChanged)
                        {
                            cout << "Client: Updating frustum" << endl;
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
                    cout << "Client: View ptcl time: " << sw_stage.ElapsedMs() << " ms" << endl;
                }

                if(FLAGS_save_ptcl)
                {
                    sw_stage.Restart();
                    PointCloud_t::Ptr pcl_object = client->get_pcl_object();
                    client->save_ptcl_from_pcl(pcl_object);
                    cout << "Client: Save ptcl time: " << sw_stage.ElapsedMs() << " ms" << endl;
                }
                cout << "End-to-end latency: " << sw_total.ElapsedMs() << " ms" << endl;
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