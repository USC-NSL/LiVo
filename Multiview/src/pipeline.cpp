#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <boost/filesystem.hpp>

#include "Server/MultiviewServerPoolTest.h"
#include "pconsts.h"
#include "k4aconsts.h"
#include "consts.h"
#include "timer.h"

DEFINE_int32(log_level, 0, "0 - INFO, 1 - WARNING, 2 - ERROR, 3 - FATAL.");
DEFINE_int32(view_ptcl, VIEWER::NO_VIEWER, "View point cloud streams using PCL or Unity Viewer. 0 - No view. 1 - PCL. 2 - Unity. 3 - Open3D.");
DEFINE_bool(save_ptcl, false, "Save point cloud streams to disk as .ply(s).");
DEFINE_bool(save_views, false, "Save views to disk as .png(s).");
DEFINE_int32(ncaptures, 1000, "Number of captures to be made or loaded from disk.");
DEFINE_int32(server_cull, CULLING::NORMAL_VOXEL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling. 5 - Expansion Culling");
DEFINE_int32(client_cull, CULLING::NORMAL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling.");
DEFINE_bool(save_frustum, false, "Save frustum to disk as .txt.");
DEFINE_int32(load_frustum, LOAD_FRUSTUM::PANOPTIC, "0 - Use default frustum (PCL), 1 - User trace on Panoptic, 2 - User trace on Kinect (Not implemented), 3 - Load frustum from disk as .txt.");
DEFINE_bool(save_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_bool(load_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_string(config_file, "/home/lei/rajrup/KinectStream/Multiview/config/panoptic.json", "Path to config file.");
DEFINE_bool(ground, false, "Ground removal.");

using namespace std;
namespace fs = boost::filesystem;

volatile sig_atomic_t flag;

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_handler(int signal_num)
{
    flag = signal_num;
}

void kinect()
{
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
            return;
        }
    }

    int start_frame_id = 1;
    int end_frame_id = FLAGS_ncaptures;

	MultiviewServerPoolTest serverPool(path, DATASET::KINECT, start_frame_id, end_frame_id, 4);
    serverPool.start_conditional();

    // MultiviewServerPoolTest serverPool(path, DATASET::KINECT, start_frame_id, end_frame_id, 6);
    // serverPool.start_sleep();
}

void panoptic()
{
    string path = FORMAT(PANOPTIC_DATA_PATH << SEQ_NAME << "/"); // RAJRUP: Save to SSD

    LOG(INFO) << "Data Path: " << path;
    LOG(INFO) << "Sequence Name: " << SEQ_NAME;

    // Creating a directory
    if(!fs::exists(path))
        LOG(FATAL) << "Directory " << path << " doesn't exist";

    END_FRAME = START_FRAME + 1000;

	// MultiviewServerPoolTest serverPool(path, DATASET::PANOPTIC, START_FRAME, END_FRAME, 4);
	// serverPool.start_conditional();

    // MultiviewServerPoolTest serverPool(path, DATASET::PANOPTIC, START_FRAME, END_FRAME, 5);
    // serverPool.start_sleep();

    MultiviewServerPoolTest2 serverPool(path, DATASET::PANOPTIC, START_FRAME, END_FRAME, 5);
    serverPool.start_sleep();
}

int main(int argc, char** argv)
{
    gflags::SetUsageMessage("Usage: MultiviewPipeline [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    absl::SetMinLogLevel((absl::LogSeverityAtLeast)FLAGS_log_level);

    string config_file = FLAGS_config_file;
    parse_config(config_file);

    LOG(INFO) << "Log Level: " << FLAGS_log_level;
    LOG(INFO) << "View Point Cloud: " << FLAGS_view_ptcl;
    LOG(INFO) << "Server Culling: " << FLAGS_server_cull;
    LOG(INFO) << "Client Culling: " << FLAGS_client_cull;
    LOG(INFO) << "Load Frustum: " << FLAGS_load_frustum;
    LOG(INFO) << "Config File: " << config_file;

    try 
    {
        struct sigaction sa;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        sa.sa_handler = signal_handler;
        sigaction(SIGINT, &sa, 0);

		// kinect();
		panoptic();
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