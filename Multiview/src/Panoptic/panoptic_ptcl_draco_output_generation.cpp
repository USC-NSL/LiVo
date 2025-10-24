#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
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

#define FPS 15
#define PARALLEL "1.0"
#define LATENCY "70ms"

using namespace std;
using namespace Eigen;
using namespace boost;
namespace fs = boost::filesystem;

DEFINE_int32(log_level, 0, "0 - INFO, 1 - WARNING, 2 - ERROR, 3 - FATAL.");
DEFINE_int32(view_ptcl, VIEWER::OPEN3D_VIEWER, "View point cloud streams using PCL or Unity Viewer. 0 - No view. 1 - PCL. 2 - Unity. 3 - Open3D.");
DEFINE_string(seq_name, "", "Sequence name.");
DEFINE_int32(start_frame_id, -1, "Start frame ID.");
DEFINE_int32(end_frame_id, -1, "End frame ID.");
DEFINE_int32(ncaptures, 6000, "Number of captures to be made or loaded from disk.");
DEFINE_bool(save_views, false, "Save views to disk as .png(s).");           // Ignore
DEFINE_string(save_views_subfolder, "test", "Subfolder to save views.");
DEFINE_int32(server_cull, CULLING::NORMAL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling. 5 - Expansion Culling");          // Default is 3
DEFINE_int32(client_cull, CULLING::NORMAL_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling.");                                 
DEFINE_int32(load_frustum, LOAD_FRUSTUM::PANOPTIC, "0 - Use deafult frustum (PCL), 1 - User trace on Panoptic, 2 - User trace on Kinect (Not implemented), 3 - Load frustum from disk as .txt.");
DEFINE_bool(save_binary_mask, false, "Save binary mask to disk as .png."); // Ignore
DEFINE_bool(load_binary_mask, false, "Load binary mask from disk.");       // Ignore
DEFINE_string(config_file, "/home/lei/rajrup/KinectStream/Multiview/config/panoptic.json", "Path to config file.");
DEFINE_bool(ground, false, "Ground removal.");
DEFINE_string(render_image_path, "", "Path to save render image using Open3D. Default is don't save");
DEFINE_string(save_ptcl_path, "", "Path to save ptcl to disk as .ply(s). Default is don't save");
DEFINE_string(mm_trace_name, "", "Mahimahi trace file name.");

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

/**
 * @brief Reads draco parameters from the abr file
 * Trace file format: FrameID,CL,QP,Compression Time (in ms),Decompression Time (in ms),Num Points,Raw Size (in bytes),Compressed Size (in bytes),Target Size (in bytes)
 * @param abr_file_path 
 * @param out vector of draco_abr_params 
 */
bool read_abr_data(const string &abr_file_path, absl::flat_hash_map<uint32_t, draco_options> &out)
{
    fs::path in_file{abr_file_path};
	fs::ifstream ifs;
	ifs.open(in_file, ios::in);
	if (!ifs.is_open())
	{
		LOG(ERROR) << "Failed to load Draco ABR file: " << in_file;
		return false;
	}

    string line;
    int row_id = 0;
    while(getline(ifs, line))
    {
        if(row_id == 0)
        {
            row_id++;
            continue;
        }

        uint32_t frame_id;
        draco_options temp;
        tokenizer<escaped_list_separator<char> > tk(line, escaped_list_separator<char>("", ",:", ""));

		int j = 0;
        for(tokenizer<escaped_list_separator<char> >::iterator i(tk.begin()); i!=tk.end(); ++i)
		{
            if(j == 0)
                frame_id = static_cast<uint32_t>(stoul(*i));
            else if(j == 1)
                temp.cl = stoi(*i);
            else if(j == 2)
                temp.qp = stoi(*i);
            j++;
		}

        assert(out.find(frame_id) == out.end());
        out[frame_id] = temp;
        row_id++;
    }
    return true;
}

void ptcl_abr_draco_generation(const string &abr_table_file_path)
{
    string compression_type = "draco";
    draco_options draco_params;
    zstd_options zstd_params;

    if(FLAGS_seq_name == "")
        LOG(FATAL) << "Sequence name not provided!" << endl;

    if(FLAGS_start_frame_id < 0)
        LOG(FATAL) << "Start frame ID not provided!" << endl;

    if(FLAGS_mm_trace_name == "")
        LOG(FATAL) << "Mahimahi trace name not provided!" << endl;

    LOG(INFO) << "Server culling: " << culling2str[FLAGS_server_cull];
    LOG(INFO) << "Client culling: " << culling2str[FLAGS_client_cull];
    LOG(INFO) << "Compression type: " << compression_type;
    LOG(INFO) << "View point cloud using: " << viewer2str[FLAGS_view_ptcl];

    string seqName = FLAGS_seq_name;
    int nCaptures = FLAGS_ncaptures;
    int startFrame = FLAGS_start_frame_id;
    int endFrame = FLAGS_start_frame_id + FLAGS_ncaptures - 1;
    if(FLAGS_end_frame_id > 0)
        endFrame = FLAGS_end_frame_id;

    string path = FORMAT(PANOPTIC_DATA_PATH << seqName << "/");

	LOG(INFO) << "Data Path: " << path;
    LOG(INFO) << "Sequence Name: " << seqName;

    // Creating a directory
    if(!fs::exists(path))
        LOG(FATAL) << "Directory " << path << " doesn't exist";

    // Keeping it. Might be useful for perfect prediction
    string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);
    if(!fs::exists(user_data_path))
        LOG(FATAL) << "Directory " << user_data_path << " doesn't exist";
    
    if(FLAGS_view_ptcl!=VIEWER::NO_VIEWER && FLAGS_view_ptcl != VIEWER::OPEN3D_VIEWER)
        LOG(FATAL) << "Please set --view_ptcl=3 for Open3D viewer or --view_ptcl=0 for no viewer!";

    if(FLAGS_render_image_path != "")
    {
        FLAGS_render_image_path = FORMAT(FLAGS_render_image_path << FLAGS_mm_trace_name << "/" << "log" << LOG_ID << "/");
        if(!fs::exists(FLAGS_render_image_path))
        {
            if(fs::create_directories(FLAGS_render_image_path))
                LOG(INFO) << "Render image directory " << FLAGS_render_image_path << " created";
            else
                LOG(FATAL) << "Render image directory  " << FLAGS_render_image_path << " creation failed";
            LOG(INFO) << "Render image path: " << FLAGS_render_image_path;
        }
    }

    if(FLAGS_save_ptcl_path != "")
    {
        FLAGS_save_ptcl_path = FORMAT(FLAGS_save_ptcl_path <<  FLAGS_mm_trace_name << "/" << "log" << LOG_ID << "/");
        if(!fs::exists(FLAGS_save_ptcl_path))
        {
            if(fs::create_directories(FLAGS_save_ptcl_path))
                LOG(INFO) << "Render ptcl directory  " << FLAGS_save_ptcl_path << " created";
            else
                LOG(FATAL) << "Render ptcl directory  " << FLAGS_save_ptcl_path << " creation failed";
            LOG(INFO) << "Render ptcl path: " << FLAGS_save_ptcl_path;
        }
    }

    if(compression_type != "draco")
        LOG(FATAL) << "Only draco is supported. Invalid compression type!";

    // Read Draco ABR data
    absl::flat_hash_map<uint32_t, draco_options> draco_abr_data;
    if(!read_abr_data(abr_table_file_path, draco_abr_data))
        LOG(FATAL) << "Failed to read Draco ABR data from " << abr_table_file_path;
    
	// Playback mode for Panoptic Dataset
    // ---------------------------------------INITIALIZATION------------------------------------------------------
    
    // Check: Load panoptic user data
    QCHECK(FLAGS_load_frustum == LOAD_FRUSTUM::PANOPTIC) << "Please set --load_frustum=1 for Panoptic Dataset";

    std::unique_ptr<MultiviewServer> server = std::make_unique<MultiviewServer>(path, DATASET::PANOPTIC);
    std::unique_ptr<MultiviewClient> client = std::make_unique<MultiviewClient>(path, DATASET::PANOPTIC);

    std::unique_ptr<DataPlayback> data_playback = std::make_unique<DataPlayback>(user_data_path, SEQ_NAME, LOG_ID);
    data_playback->loadData(START_FRAME, END_FRAME);
    UserData orig_data, pred_data;
    bool first_frame = true;

    // Set viewer camera instrincs based on the intrinsic parameters from one of the frames in the user trace
    // If intrinsics change across frames, then we need to set it for each frame in the playback loop.
    CamInt viewer_intrinsics;
    orig_data = data_playback->getUserData(START_FRAME+1);
    viewer_intrinsics.angle = orig_data.frustum.angle;
    viewer_intrinsics.ratio = orig_data.frustum.ratio;
    viewer_intrinsics.nearD = orig_data.frustum.nearD;
    viewer_intrinsics.farD = orig_data.frustum.farD;

    LOG(INFO) << "User trace DATASET: " << SEQ_NAME;
    LOG(INFO) << "User trace start frame id: " << START_FRAME;
    LOG(INFO) << "User trace end frame id: " << END_FRAME;

    o3d::visualization::Visualizer visualizer;
    std::shared_ptr<o3d::geometry::PointCloud> pcd_ptr = nullptr;
    if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
    {
        float ratio = viewer_intrinsics.ratio;
        float nearD = viewer_intrinsics.nearD;
        float farD = viewer_intrinsics.farD;

        int width = 1000;
        int height = int((float)width / ratio);

        // Visualizer initialization
        // visualizer.CreateVisualizerWindow("Open3D", 1600, 900, 0, 0);
        visualizer.CreateVisualizerWindow("Open3D", width, height, 0, 0);

        visualizer.GetRenderOption().SetPointSize(4.0);                 // Default is 5.0
        o3d::visualization::ViewControl& view_ctl = visualizer.GetViewControl();

        // TODO: Rajrup - Set camera parameters from user trace. 
        view_ctl.SetConstantZNear(viewer_intrinsics.nearD);
        view_ctl.SetConstantZFar(viewer_intrinsics.farD);
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

            /*********************************** SERVER: LOAD IMAGES **********************************/
            sw_stage.Restart();
            server->set_curr_frameId(capture_id);

            if(!server->load_frame_parallel())
            {
                LOG(INFO) << "Server: Skipping frame id " << capture_id;
                continue;
            }

            /********************************* SERVER: GENERATE PTCL **********************************/
			sw_stage.Restart();
            orig_data = data_playback->getUserData(server->get_curr_frameId());
            server->set_frustum(orig_data.frustum);

            server->update_ptcl();
            uint32_t num_points = server->get_num_points_ptcl();
            uint64_t raw_size_bytes = num_points * (3 * sizeof(float) + 3 * sizeof(uint8_t));
            double raw_size_MB = (double)raw_size_bytes / (1024.0 * 1024.0);

            int s_currFrameId = server->get_curr_frameId();
            client->set_curr_frameId(s_currFrameId);
            client->set_frustum(orig_data.frustum);
            int c_currFrameId = client->get_curr_frameId();

            /********************************* SERVER: COMPRESSION & DECOMPRESSION ***********************************/
            if(num_points > 0)
            {
                /********************************* SERVER: COMPRESSION *********************************/
                if(draco_abr_data.find(s_currFrameId) == draco_abr_data.end())
                    LOG(FATAL) << "Server: Draco ABR data not found for frame id " << s_currFrameId;
                
                draco_params.cl = draco_abr_data[s_currFrameId].cl;
                draco_params.qp = draco_abr_data[s_currFrameId].qp;

                LOG(INFO) << "Frame ID - " << s_currFrameId << ", cl: " << draco_params.cl << ", qp: " << draco_params.qp << ", Num Points: " << num_points << ", Raw Size: " << raw_size_MB << " MB";

                if(draco_params.cl < 0 || draco_params.qp < 0)
                    continue;


                sw_stage.Restart();
                int size = 0;
                const uint8_t *compressed_ptcl_buf = NULL;
                compressed_ptcl_buf = server->compress_ptcl(size, compression_type, zstd_params, draco_params);
                uint32_t compression_time = sw_stage.ElapsedMs();

                uint64_t compressed_size_bytes = size;
                double compressed_size_MB = (double)size / (1024.0 * 1024.0);

                // LOG(INFO) << "Server: Draco <cl: " << cl << ", qp: " << qp << "> Compress ptcl time: " << compression_time << " ms";

                /********************************* CLIENT: DECOMPRESSION *********************************/
                sw_stage.Restart();
                client->decompress_ptcl(compressed_ptcl_buf, size, compression_type, zstd_params, draco_params);

                uint32_t decompression_time = sw_stage.ElapsedMs();
                // LOG(INFO) << "Client: Decompress ptcl time: " << decompression_time << " ms";
            }
            else
                LOG(INFO) << "Frame ID - " << s_currFrameId << " No points to compress!";
            
            /*********************************** CLIENT: VIEWER **************************************/
            if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
            {
                sw_stage.Restart();

                O3DData *o3d_data = new O3DData();
                o3d_data->frame_id = c_currFrameId;

                if(num_points > 0)
                    client->update_ptcl_3D_decompress_open3D(compression_type, o3d_data->points, o3d_data->colors);
                else
                {
                    o3d_data->points.clear();
                    o3d_data->colors.clear();
                }

                // Get user trace
                assert(server->get_curr_frameId() == client->get_curr_frameId());
                assert(o3d_data->frame_id == client->get_curr_frameId());
                // client->set_frustum(orig_data.frustum);

                o3d::visualization::ViewControl& ctr = visualizer.GetViewControl();
                // ctr.ChangeFieldOfView(60);

                o3d::camera::PinholeCameraParameters camera_params;
                ctr.ConvertToPinholeCameraParameters(camera_params);

                Eigen::Vector4d q_rotation{orig_data.quat[3], -1.0 * orig_data.quat[0], orig_data.quat[1], -1.0 * orig_data.quat[2]};
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

                // Note: Num points after decoding may be zero if qp is too low. 
                LOG(INFO) << "Frame ID - " << c_currFrameId << ", Render Num Points: " << o3d_data->points.size();

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
                    if(client->get_curr_frameId() % 10 == 0)
                    {
                        string output_file = FORMAT(FLAGS_render_image_path << client->get_curr_frameId() << ".png");
                        visualizer.CaptureScreenImage(output_file);
                        LOG(INFO) << "Client: Saved rendered image to " << output_file;
                    }
                }
                if(FLAGS_save_ptcl_path != "")
                {
                    if(client->get_curr_frameId() % 10 == 0)
                    {
                        string output_file = FORMAT(FLAGS_save_ptcl_path << client->get_curr_frameId() << ".ply");
                        // Correct point cloud from upside down to right side up
                        o3d::geometry::PointCloud temp_pcd;
                        temp_pcd.points_ = o3d_data->points;
                        temp_pcd.colors_ = o3d_data->colors;

                        Eigen::Matrix4d flip_transform;
                        flip_transform << 1, 0, 0, 0,
                                        0, -1, 0, 0,
                                        0, 0, 1, 0,
                                        0, 0, 0, 1;
                        temp_pcd.Transform(flip_transform);
                        o3d::io::WritePointCloud(output_file, temp_pcd);
                        LOG(INFO) << "Client: Saved rendered ptcl to " << output_file;
                    }
                }

                delete o3d_data;
                LOG(INFO) << "Client: View open3d ptcl time: " << sw_stage.ElapsedMs() << " ms";
            }
            else
                LOG(INFO) << "No viewer selected!";

            LOG(INFO) << "Playback: End-to-end latency: " << sw_total.ElapsedMs() << " ms";
        }
        if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
            visualizer.DestroyVisualizerWindow();
    }
    catch(int ex)
    {
        LOG(INFO) << "Caught: Ctrl+C";
        LOG(INFO) << "Exiting...";
        return;
    }
    sleep(2);
	return;
}

int main(int argc, char **argv)
{
    gflags::SetUsageMessage("Usage: PanopticPtclDracoOutputGeneration[OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    absl::SetMinLogLevel((absl::LogSeverityAtLeast)FLAGS_log_level);

    string config_file = FLAGS_config_file;
    parse_config(config_file);

    string in_dir = FORMAT("/datassd/pipeline_ptcl/ptcl_compression_table/" << FLAGS_seq_name << "/" << USER_TRACE_FOLDER << "/" << FLAGS_mm_trace_name << "/");

    if(!fs::exists(in_dir))
        LOG(FATAL) << "ABR input directory " << in_dir << " doesn't exist!";

    string table_file_path = FORMAT(in_dir << "draco_parallel" << PARALLEL << "_abr_log" << LOG_ID << "_lat" << LATENCY << "_fps" << FPS << ".csv");
    ptcl_abr_draco_generation(table_file_path);
}