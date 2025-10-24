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
DEFINE_string(compression_type, "zstd", "Compression type. Options: zstd, draco, gpcc, None (will be supported later)");
DEFINE_int32(view_ptcl, VIEWER::OPEN3D_VIEWER, "View point cloud streams using PCL or Unity Viewer. 0 - No view. 1 - PCL. 2 - Unity. 3 - Open3D.");
DEFINE_string(seq_name, "", "Sequence name.");
DEFINE_int32(start_frame_id, -1, "Start frame ID.");
DEFINE_int32(ncaptures, 6000, "Number of captures to be made or loaded from disk.");
DEFINE_bool(save_views, false, "Save views to disk as .png(s).");           // Ignore
DEFINE_string(save_views_subfolder, "test", "Subfolder to save views.");
DEFINE_int32(server_cull, CULLING::NORMAL_VOXEL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling. 5 - Expansion Culling");          // Ignore, default to NO_CULLING
DEFINE_int32(client_cull, CULLING::NORMAL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling.");
DEFINE_int32(load_frustum, LOAD_FRUSTUM::PANOPTIC, "0 - Use deafult frustum (PCL), 1 - User trace on Panoptic, 2 - User trace on Kinect (Not implemented), 3 - Load frustum from disk as .txt.");
DEFINE_bool(save_binary_mask, false, "Save binary mask to disk as .png."); // Ignore
DEFINE_bool(load_binary_mask, false, "Load binary mask from disk.");       // Ignore
DEFINE_string(config_folder, "/home/lei/rajrup/KinectStream/Multiview/config/", "Path to config files.");
DEFINE_string(render_image_path, "", "Path to save render image using Open3D. Default is don't save");
DEFINE_bool(save_ptcl, false, "Save point cloud streams to disk as .ply(s).");
DEFINE_bool(ground, false, "Ground removal.");

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
    gflags::SetUsageMessage("Usage: PanopticPtcl [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    absl::SetMinLogLevel((absl::LogSeverityAtLeast)FLAGS_log_level);

    string config_file = FORMAT(FLAGS_config_folder << "panoptic.json");
    parse_config(config_file);

    if(FLAGS_seq_name == "")
        LOG(FATAL) << "Sequence name not provided!" << endl;

    if(FLAGS_start_frame_id < 0)
        LOG(FATAL) << "Start frame ID not provided!" << endl;


    // Pre-defined flags since we need to extract ground truth frames
    FLAGS_server_cull = CULLING::NO_CULLING;
    LOG(INFO) << "Server culling set to NO_CULLING, ignoring server_cull flag";

    FLAGS_view_ptcl = VIEWER::OPEN3D_VIEWER;
    LOG(INFO) << "View point cloud streams using Open3D, ignoring view_ptcl flag";

    string seqName = FLAGS_seq_name;
    int nCaptures = FLAGS_ncaptures;
    int startFrame = FLAGS_start_frame_id;
    int endFrame = FLAGS_start_frame_id + FLAGS_ncaptures;

    string path = FORMAT(PANOPTIC_DATA_PATH << seqName << "/");
    // string path = "/home/lei/data/pipeline/server_standalone/nvenc_10k_20k/";

	LOG(INFO) << "Data Path: " << path;
    LOG(INFO) << "Sequence Name: " << seqName;

    // Creating a directory
    if(!fs::exists(path))
        LOG(FATAL) << "Directory " << path << " doesn't exist";

    // Keeping it. Might be useful for perfect prediction
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

    string compression_type = FLAGS_compression_type;
    zstd_options zstd_params;
    draco_options draco_params;
    if(compression_type == "zstd")
    {
        string config_file = FORMAT(FLAGS_config_folder << "zstd_params.json");
        parse_zstd_config(config_file, zstd_params);
    }
    else if(compression_type == "draco")
    {
        string config_file = FORMAT(FLAGS_config_folder << "draco_params.json");
        parse_draco_config(config_file, draco_params);
    }
    else
        LOG(FATAL) << "Only zstd and draco are supported. Invalid compression type!";

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

    LOG(INFO) << "User trace DATASET: " << SEQ_NAME;
    LOG(INFO) << "User trace start frame id: " << START_FRAME;
    LOG(INFO) << "User trace end frame id: " << END_FRAME;

    o3d::visualization::Visualizer visualizer;
    std::shared_ptr<o3d::geometry::PointCloud> pcd_ptr = nullptr;
    if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
    {
        // Visualizer initialization
        visualizer.CreateVisualizerWindow("Open3D", 1600, 900, 0, 0);
        // visualizer.GetRenderOption().SetPointSize(2.0);                 // Default is 5.0
        pcd_ptr = std::make_shared<o3d::geometry::PointCloud>();
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

    // Statistics
    Accumulator server_load_time_acc;
    Accumulator server_update_time_acc;
    Accumulator server_compression_time_acc;
    Accumulator server_raw_points_acc;
    Accumulator server_raw_size_acc;
    Accumulator server_compressed_size_acc;
    
    Accumulator client_decompression_time_acc;
    Accumulator client_view_time_acc;
    Accumulator client_compressed_size_acc;

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

        for(uint32_t capture_id = startFrame; capture_id < endFrame; capture_id++)
        {
			sw_total.Restart();   
            if (flag) 
                throw (int)flag;
            
            LOG(INFO) << "-----------------------------------------------------------------------";

            /*********************************** SERVER: LOAD IMAGES **********************************/
            sw_stage.Restart();
            server->set_curr_frameId(capture_id);
            LOG(INFO) << "Server: Starting with frame: " << capture_id;

            if(!server->load_frame_parallel())
            {
                LOG(INFO) << "Server: Skipping frame id " << capture_id;
                continue;
            }

            server_load_time_acc.add(sw_stage.ElapsedMs());
            LOG(INFO) << "Server: Load frame time: " << sw_stage.ElapsedMs() << " ms";

            /********************************* SERVER: GENERATE PTCL **********************************/
			sw_stage.Restart();
            orig_data = data_playback->getUserData(server->get_curr_frameId());
            server->set_frustum(orig_data.frustum);

            server->update_ptcl();

            server_raw_points_acc.add(server->get_num_points_ptcl());
            double raw_size_MB = server->get_num_points_ptcl() * (3 * sizeof(float) + 3 * sizeof(uint8_t)) / (1024.0 * 1024.0); 
            server_raw_size_acc.add(raw_size_MB);

            server_update_time_acc.add(sw_stage.ElapsedMs());
            LOG(INFO) << "Server: Update point cloud time: " << sw_stage.ElapsedMs() << " ms";

            /********************************* SERVER: COMPRESSION ***********************************/
            sw_stage.Restart();
            int size = 0;
            const uint8_t *compressed_ptcl_buf = NULL;

            compressed_ptcl_buf = server->compress_ptcl(size, compression_type, zstd_params, draco_params);

            server_compression_time_acc.add(sw_stage.ElapsedMs());
            double compressed_size_MB = size / (1024.0 * 1024.0);
            server_compressed_size_acc.add(compressed_size_MB);
            LOG(INFO) << "Server: Compress ptcl time: " << sw_stage.ElapsedMs() << " ms";
            
            /********************************* CLIENT: DECOMPRESSION *********************************/
            sw_stage.Restart();
            int s_currFrameId = server->get_curr_frameId();
            client->set_curr_frameId(s_currFrameId);
			client->set_frustum(orig_data.frustum);
            int c_currFrameId = client->get_curr_frameId();

            client->decompress_ptcl(compressed_ptcl_buf, size, compression_type, zstd_params, draco_params);

            client_decompression_time_acc.add(sw_stage.ElapsedMs());
            LOG(INFO) << "Client: Decompress ptcl time: " << sw_stage.ElapsedMs() << " ms";

            /*********************************** CLIENT: VIEWER **************************************/
            if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
            {
                sw_stage.Restart();

                O3DData *o3d_data = new O3DData();
                o3d_data->frame_id = c_currFrameId;
                client->update_ptcl_3D_decompress_open3D(compression_type, o3d_data->points, o3d_data->colors);

                LOG(INFO) << "Client: Update ptcl time: " << sw_stage.ElapsedMs() << " ms";

                // Get user trace
                assert(server->get_curr_frameId() == client->get_curr_frameId());
                assert(o3d_data->frame_id == client->get_curr_frameId());
                client->set_frustum(orig_data.frustum);

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

                pcd_ptr->points_ = o3d_data->points;
                pcd_ptr->colors_ = o3d_data->colors;

                if(first_frame)
                {
                    visualizer.AddGeometry(pcd_ptr);
                    first_frame = false;
                }
                else
                    visualizer.UpdateGeometry(pcd_ptr);
                visualizer.PollEvents();
                visualizer.UpdateRender();
                
                if(FLAGS_render_image_path != "")
                {
                    string output_file = FORMAT(output_path << client->get_curr_frameId() << ".png");
                    visualizer.CaptureScreenImage(output_file);
                    LOG(INFO) << "Client: Saved rendered image to " << output_file;
                }
                if(FLAGS_save_ptcl)
                {
                    string output_file = FORMAT(output_path << client->get_curr_frameId() << ".ply");
                    o3d::io::WritePointCloud(output_file, *pcd_ptr);
                    LOG(INFO) << "Client: Saved rendered point cloud to " << output_file;
                }

                delete o3d_data;
                client_view_time_acc.add(sw_stage.ElapsedMs());
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
        cout << "-------------------------------- STATITSTICS ---------------------------------------" << endl;
        cout << "STAGE: SERVER - LOAD TIME" << endl;
        cout << "Mean: " << server_load_time_acc.mean() << " ms, Median: " << server_load_time_acc.median() << " ms, std: " << server_load_time_acc.std() << " ms" << endl;
        cout << "STAGE: SERVER - UPDATE TIME" << endl;
        cout << "Mean: " << server_update_time_acc.mean() << " ms, Median: " << server_update_time_acc.median() << " ms, std: " << server_update_time_acc.std() << " ms" << endl;
        cout << "STAGE: SERVER - COMPRESSION TIME" << endl;
        cout << "Mean: " << server_compression_time_acc.mean() << " ms, Median: " << server_compression_time_acc.median() << " ms, std: " << server_compression_time_acc.std() << " ms" << endl;
        cout << "STAGE: CLIENT - DECOMPRESSION TIME" << endl;
        cout << "Mean: " << client_decompression_time_acc.mean() << " ms, Median: " << client_decompression_time_acc.median() << " ms, std: " << client_decompression_time_acc.std() << " ms" << endl;
        cout << "STAGE: CLIENT - VIEW TIME" << endl;
        cout << "Mean: " << client_view_time_acc.mean() << " ms, Median: " << client_view_time_acc.median() << " ms, std: " << client_view_time_acc.std() << " ms" << endl << endl;
        cout << "------------------------------------------------------------------------------------" << endl;
        cout << "RAW POINTS" << endl;
        cout << "Mean: " << server_raw_points_acc.mean() << " points, Median: " << server_raw_points_acc.median() << " points, std: " << server_raw_points_acc.std() << " points" << endl;
        cout << "RAW SIZE" << endl;
        cout << "Mean: " << server_raw_size_acc.mean() << " MB, Median: " << server_raw_size_acc.median() << " MB, std: " << server_raw_size_acc.std() << " MB" << endl;
        cout << "COMPRESSED SIZE" << endl;
        cout << "Mean: " << server_compressed_size_acc.mean() << " MB, Median: " << server_compressed_size_acc.median() << " MB, std: " << server_compressed_size_acc.std() << " MB" << endl;
        return EXIT_FAILURE;
    }

    cout << "-------------------------------- STATITSTICS ---------------------------------------" << endl;
    cout << "STAGE: SERVER - LOAD TIME" << endl;
    cout << "Mean: " << server_load_time_acc.mean() << " ms, Median: " << server_load_time_acc.median() << " ms, std: " << server_load_time_acc.std() << " ms" << endl;
    cout << "STAGE: SERVER - UPDATE TIME" << endl;
    cout << "Mean: " << server_update_time_acc.mean() << " ms, Median: " << server_update_time_acc.median() << " ms, std: " << server_update_time_acc.std() << " ms" << endl;
    cout << "STAGE: SERVER - COMPRESSION TIME" << endl;
    cout << "Mean: " << server_compression_time_acc.mean() << " ms, Median: " << server_compression_time_acc.median() << " ms, std: " << server_compression_time_acc.std() << " ms" << endl;
    cout << "STAGE: CLIENT - DECOMPRESSION TIME" << endl;
    cout << "Mean: " << client_decompression_time_acc.mean() << " ms, Median: " << client_decompression_time_acc.median() << " ms, std: " << client_decompression_time_acc.std() << " ms" << endl;
    cout << "STAGE: CLIENT - VIEW TIME" << endl;
    cout << "Mean: " << client_view_time_acc.mean() << " ms, Median: " << client_view_time_acc.median() << " ms, std: " << client_view_time_acc.std() << " ms" << endl << endl;
    cout << "------------------------------------------------------------------------------------" << endl;
    cout << "RAW POINTS" << endl;
    cout << "Mean: " << server_raw_points_acc.mean() << " points, Median: " << server_raw_points_acc.median() << " points, std: " << server_raw_points_acc.std() << " points" << endl;
    cout << "RAW SIZE" << endl;
    cout << "Mean: " << server_raw_size_acc.mean() << " MB, Median: " << server_raw_size_acc.median() << " MB, std: " << server_raw_size_acc.std() << " MB" << endl;
    cout << "COMPRESSED SIZE" << endl;
    cout << "Mean: " << server_compressed_size_acc.mean() << " MB, Median: " << server_compressed_size_acc.median() << " MB, std: " << server_compressed_size_acc.std() << " MB" << endl;

    sleep(2);
	return EXIT_SUCCESS;
}