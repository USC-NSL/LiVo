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
DEFINE_int32(view_ptcl, VIEWER::OPEN3D_VIEWER, "View point cloud streams using PCL or Unity Viewer. 0 - No view. 1 - PCL. 2 - Unity. 3 - Open3D.");
DEFINE_string(seq_name, "", "Sequence name.");
DEFINE_int32(start_frame_id, -1, "Start frame ID.");
DEFINE_int32(end_frame_id, -1, "End frame ID.");
DEFINE_int32(ncaptures, 6000, "Number of captures to be made or loaded from disk.");
DEFINE_bool(save_views, false, "Save views to disk as .png(s).");
DEFINE_string(save_views_subfolder, "test", "Subfolder to save views.");
DEFINE_int32(server_cull, CULLING::NO_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling. 5 - Expansion Culling");
DEFINE_int32(client_cull, CULLING::NO_CULLING, "0 - No Culling, 1 - Clip culling. 2 - Voxel Clip culling. 3 - Normal culling. 4 - Normal Voxel culling.");
DEFINE_int32(load_frustum, LOAD_FRUSTUM::PANOPTIC, "0 - Use default frustum (PCL), 1 - User trace on Panoptic, 2 - User trace on Kinect (Not implemented), 3 - Load frustum from disk as .txt.");
DEFINE_bool(save_binary_mask, false, "Save binary mask to disk as .png.");
DEFINE_bool(load_binary_mask, false, "Load binary mask from disk.");
DEFINE_string(config_file, "/home/lei/rajrup/KinectStream/Multiview/config/panoptic.json", "Path to config file.");
DEFINE_string(render_image_path, "", "Path to save render image using Open3D. Default is don't save");
DEFINE_string(save_ptcl_path, "", "Path to save ptcl to disk as .ply(s). Default is don't save");
DEFINE_string(collect_stats, "", "File to collect stats for each frame");
DEFINE_bool(ground, false, "Ground removal.");

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
    gflags::SetUsageMessage("Usage: PanopticGT [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    absl::SetMinLogLevel((absl::LogSeverityAtLeast)FLAGS_log_level);

    string config_file = FLAGS_config_file;
    parse_config(config_file);

    if(FLAGS_seq_name == "")
        LOG(FATAL) << "Sequence name not provided!" << endl;

    if(FLAGS_start_frame_id < 0)
        LOG(FATAL) << "Start frame ID not provided!" << endl;

    LOG(INFO) << "Server culling: " << culling2str[FLAGS_server_cull];
    LOG(INFO) << "Client culling: " << culling2str[FLAGS_client_cull];

    FLAGS_view_ptcl = VIEWER::OPEN3D_VIEWER;
    LOG(INFO) << "View point cloud streams using Open3D, ignoring view_ptcl flag";

    string seqName = FLAGS_seq_name;
     int nCaptures = FLAGS_ncaptures;
    int startFrame = FLAGS_start_frame_id;
    int endFrame = FLAGS_start_frame_id + FLAGS_ncaptures - 1;
    if(FLAGS_end_frame_id > 0)
        endFrame = FLAGS_end_frame_id;

    string path = FORMAT(PANOPTIC_DATA_PATH << seqName << "/");
    // string path = "/home/lei/data/pipeline/server_standalone/nvenc_10k_20k/";
    // string path = "/home/lei/data/pipeline/server_standalone/pipeline_new/160317_moonbaby1_no_ground/nvenc_c_100000k_d_yuv16_200000k/";

	LOG(INFO) << "Data Path: " << path;
    LOG(INFO) << "Sequence Name: " << seqName;

    // Creating a directory
    if(!fs::exists(path))
        LOG(FATAL) << "Directory " << path << " doesn't exist";

    string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);
    if(!fs::exists(user_data_path))
        LOG(FATAL) << "Directory " << user_data_path << " doesn't exist";
    
    if(FLAGS_view_ptcl != VIEWER::OPEN3D_VIEWER && FLAGS_render_image_path != "")
        LOG(FATAL) << "Please set --view_ptcl=3 for Open3D viewer to render image";
    
    if(FLAGS_render_image_path != "")
    {
        // FLAGS_render_image_path = FORMAT(FLAGS_render_image_path << "log" << LOG_ID << "/");

        if(fs::exists(FLAGS_render_image_path))
        {
            // remove directory
            fs::remove_all(FLAGS_render_image_path);
            LOG(INFO) << "Directory: " << FLAGS_render_image_path << " removed";
        }
        fs::create_directories(FLAGS_render_image_path);
        LOG(INFO) << "Render image path: " << FLAGS_render_image_path;
    }

    if(FLAGS_save_ptcl_path != "")
    {
        // FLAGS_save_ptcl_path = FORMAT(FLAGS_save_ptcl_path << "log" << LOG_ID << "/");

        if(fs::exists(FLAGS_save_ptcl_path))
        {
            // remove directory
            fs::remove_all(FLAGS_save_ptcl_path);
            LOG(INFO) << "Directory: " << FLAGS_save_ptcl_path << " removed";
        }
        fs::create_directories(FLAGS_save_ptcl_path);
        LOG(INFO) << "Render ptcl path: " << FLAGS_save_ptcl_path;
    }

    fs::path frustum_size_path = FORMAT("/datassd/pipeline/client_tiled/pipeline_new/" << seqName << "/comp_o3d_pointssim/frustum_size.csv");
    fs::ofstream frustum_size_file;
    frustum_size_file.open(frustum_size_path, ios::out | ios::trunc);
    frustum_size_file << "Frame,FrustumPoints,Frustum_Size(in bytes)" << endl;

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
        UserData data_first_frame = data_playback->getUserData(startFrame+1);
        float ratio = data_first_frame.frustum.ratio;
        float nearD = data_first_frame.frustum.nearD;
        float farD = data_first_frame.frustum.farD;

        int width = 1000;
        int height = int((float)width / ratio);

        // Visualizer initialization
        // visualizer.CreateVisualizerWindow("Open3D", 1600, 900, 0, 0);
        visualizer.CreateVisualizerWindow("Open3D", width, height, 0, 0);
        visualizer.GetRenderOption().SetPointSize(3.0);                 // Default is 5.0
        o3d::visualization::ViewControl& view_ctl = visualizer.GetViewControl();

        view_ctl.SetConstantZNear(nearD);
        view_ctl.SetConstantZFar(farD);
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
    Accumulator server_raw_points_acc;
    Accumulator server_raw_size_acc;
    
    Accumulator client_gen_ptcl_time_acc;
    Accumulator client_view_ptcl_time_acc;
    Accumulator client_save_rgb_time_acc;
    Accumulator client_save_ptcl_time_acc;

    string out_path = "/home/lei/rajrup/KinectStream/data/processed_data/";
    string out_file = FORMAT(out_path << "frustum_corners_negative_y.csv");

    ///// DEBUG
    // fs::ofstream frustum_file;
    // fs::path frustum_file_path{FORMAT(out_path << "frustum_corners_negative_y.csv")};
    // frustum_file.open(frustum_file_path, ios::out | ios::trunc);

    // frustum_file << "frame_id,ntlx,ntly,ntlz,ntrx,ntry,ntrz,nblx,nbly,nblz,nbrx,nbry,nbrz,ftlx,ftly,ftlz,ftrx,ftry,ftrz,fblx,fbly,fblz,fbrx,fbry,fbrz,nearD,farD,ratio,angle,tang,nw,nh,fw,fh" << endl;

    // for(uint32_t frame_id = startFrame; frame_id < endFrame; frame_id++)
    // {
    //     UserData user_data = data_playback->getUserData(frame_id);

    //     frustum_file << frame_id << ","
    //                     << user_data.frustum.ntl(0) << "," << user_data.frustum.ntl(1) << "," << user_data.frustum.ntl(2) << ","
    //                     << user_data.frustum.ntr(0) << "," << user_data.frustum.ntr(1) << "," << user_data.frustum.ntr(2) << ","
    //                     << user_data.frustum.nbl(0) << "," << user_data.frustum.nbl(1) << "," << user_data.frustum.nbl(2) << ","
    //                     << user_data.frustum.nbr(0) << "," << user_data.frustum.nbr(1) << "," << user_data.frustum.nbr(2) << ","
    //                     << user_data.frustum.ftl(0) << "," << user_data.frustum.ftl(1) << "," << user_data.frustum.ftl(2) << ","
    //                     << user_data.frustum.ftr(0) << "," << user_data.frustum.ftr(1) << "," << user_data.frustum.ftr(2) << ","
    //                     << user_data.frustum.fbl(0) << "," << user_data.frustum.fbl(1) << "," << user_data.frustum.fbl(2) << ","
    //                     << user_data.frustum.fbr(0) << "," << user_data.frustum.fbr(1) << "," << user_data.frustum.fbr(2) << ","
    //                     << user_data.frustum.nearD << "," << user_data.frustum.farD << ","
    //                     << user_data.frustum.ratio << "," << user_data.frustum.angle << "," << user_data.frustum.tang << ","
    //                     << user_data.frustum.nw << "," << user_data.frustum.nh << ","
    //                     << user_data.frustum.fw << "," << user_data.frustum.fh << endl;
    // }
    // frustum_file.close();
    ///// DEBUG

    // return 0;

    /** Log user trace file for Christina
    // fs::ofstream user_trace_file;
    // string user_trace_file_path = FORMAT("/home/lei/rajrup/standalone/KinectStream/data/" << seqName << ".csv");
    // user_trace_file.open(user_trace_file_path, ios::out);
    // if(!user_trace_file.is_open())
    //     LOG(FATAL) << "Failed to open user trace file: " << user_trace_file_path;
    // user_trace_file << "frameID,x,y,z,rx,ry,rz,qx,qy,qz,qw" << endl;
    */

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
            LOG(INFO) << "Server: Starting with frame: " << capture_id;

            if(!server->load_frame_parallel())
            {
                LOG(INFO) << "Server: Skipping frame id " << capture_id;
                continue;
            }

            // if(!server->load_frame_with_colorized_depth_parallel(false, "png"))
            // {
            //     LOG(INFO) << "Server: Skipping frame id " << capture_id;
            //     continue;
            // }

            // if(!server->load_frame_with_colorized_depth_parallel(false, "jpeg"))
            // {
            //     LOG(INFO) << "Server: Skipping frame id " << capture_id;
            //     continue;
            // }
            server_load_time_acc.add(sw_stage.ElapsedMs());
            LOG(INFO) << "Server: Load frame time: " << sw_stage.ElapsedMs() << " ms";

            /********************************* SERVER: UPDATE VIEWS **********************************/
			sw_stage.Restart();
            orig_data = data_playback->getUserData(server->get_curr_frameId());
            server->set_frustum(orig_data.frustum);

            // orig_data.frustum.saveToFile("/home/lei/data/pipeline/client_standalone/pipeline_new/", FORMAT(server->get_curr_frameId() << ".txt"));

            server->update_view();
            uint64_t view_size = server->get_view_size_in_frustum();
            uint64_t num_points = view_size / (sizeof(uint16_t) + sizeof(RGB));
            frustum_size_file << server->get_curr_frameId() << "," << num_points << "," << view_size << endl;

            // Don't free the views, it is owned by the server
            vector<View *> s_views = server->get_view();
            server_update_time_acc.add(sw_stage.ElapsedMs());
            LOG(INFO) << "Server: Update views time: " << sw_stage.ElapsedMs() << " ms";

            /********************************* CLIENT: GENERATE PTCL *********************************/
            sw_stage.Restart();
            int s_currFrameId = server->get_curr_frameId();
            client->set_curr_frameId(s_currFrameId);
			client->set_frustum(orig_data.frustum);
            int c_currFrameId = client->get_curr_frameId();
            
            if(!client->set_view(s_views))
                LOG(FATAL) << "Client: Failed to set view!";

            if(FLAGS_load_binary_mask)
		        client->load_binary_mask(c_currFrameId);

            LOG(INFO) << "Client: Set view time: " << sw_stage.ElapsedMs() << " ms";
            
            sw_stage.Restart();
			if(!client->update_ptcl())
                LOG(FATAL) << "Client: Failed to update ptcl!";

            client_gen_ptcl_time_acc.add(sw_stage.ElapsedMs());
            LOG(INFO) << "Client: Update ptcl time: " << sw_stage.ElapsedMs() << " ms";

            /*********************************** CLIENT: VIEWER **************************************/
            if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
            {
                sw_stage.Restart();

                O3DData *o3d_data = new O3DData();
                o3d_data->frame_id = c_currFrameId;
                client->get_open3d_points_colors(o3d_data->points, o3d_data->colors);

                // Get user trace
                assert(server->get_curr_frameId() == client->get_curr_frameId());
                assert(o3d_data->frame_id == client->get_curr_frameId());
                client->set_frustum(orig_data.frustum);

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

                /** Write trace file (for Christina)
                // user_trace_file << o3d_data->frame_id << "," << orig_data.pos[0] << "," << orig_data.pos[1] << "," << orig_data.pos[2] << "," << orig_data.rot[0] << "," << orig_data.rot[1] << "," << orig_data.rot[2] << "," <<  orig_data.quat[0] << "," << orig_data.quat[1] << "," << orig_data.quat[2] << "," << orig_data.quat[3] << endl;
                */

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

                client_view_ptcl_time_acc.add(sw_stage.ElapsedMs());
                LOG(INFO) << "Client: View open3d ptcl time: " << sw_stage.ElapsedMs() << " ms";

                // cv::Mat img = cv::Mat(595, 1000, CV_32FC3);
                // cv::Mat temp_img;
                
                if(FLAGS_render_image_path != "")
                {
                    if(client->get_curr_frameId() % 100 == 0)
                    {
                        sw_stage.Restart();
                        string output_file = FORMAT(FLAGS_render_image_path << client->get_curr_frameId() << ".png");
                        visualizer.CaptureScreenImage(output_file);

                        // Alternative way to capture screen
                        // auto test = visualizer.CaptureScreenFloatBuffer();
                        // memcpy(img.data, test->data_.data(), test->data_.size());
                        
                        // img.convertTo(temp_img, CV_8UC3, 255.0);    // Convert to 8-bit from 0.0-1.0 to 0-255
                        // cv::cvtColor(temp_img, temp_img, cv::COLOR_RGB2BGR);
                        // cv::imwrite(output_file, temp_img);
                        // // cv::imshow("Open3D", temp);
                        // // cv::waitKey(1);

                        LOG(INFO) << "Client: Saved rendered image to " << output_file;
                        LOG(INFO) << "Client: Save rendered image time: " << sw_stage.ElapsedMs() << " ms";
                        client_save_rgb_time_acc.add(sw_stage.ElapsedMs());
                    }
                }
                if(FLAGS_save_ptcl_path != "")
                {
                    if(client->get_curr_frameId() % 10 == 0)
                    {
                        sw_stage.Restart();
                        string output_file = FORMAT(FLAGS_save_ptcl_path << client->get_curr_frameId() << ".ply");

                        // Correct point cloud from upside down to right side up
                        o3d::geometry::PointCloud temp_pcd;
                        temp_pcd.points_ = o3d_data->points;
                        temp_pcd.colors_ = o3d_data->colors;

                        // TODO: Rajrup: Comment to save point cloud upside. This is required for meshreduce GT. 
                        Eigen::Matrix4d flip_transform;
                        flip_transform << 1, 0, 0, 0,
                                        0, -1, 0, 0,
                                        0, 0, 1, 0,
                                        0, 0, 0, 1;
                        temp_pcd.Transform(flip_transform);
                        o3d::io::WritePointCloud(output_file, temp_pcd);
                        LOG(INFO) << "Client: Saved rendered ptcl to " << output_file;
                        client_save_ptcl_time_acc.add(sw_stage.ElapsedMs());
                        LOG(INFO) << "Client: Save rendered ptcl time: " << sw_stage.ElapsedMs() << " ms";
                    }
                }

                delete o3d_data;
            }
            else
                LOG(FATAL) << "Invalid viewer! Only use Open3D for generating GT";

            LOG(INFO) << "Playback: End-to-end latency: " << sw_total.ElapsedMs() << " ms";
		}
        visualizer.DestroyVisualizerWindow();
        // user_trace_file.close();
	}
	catch(int ex)
    {
        frustum_size_file.flush();
        frustum_size_file.close();
        LOG(INFO) << "Caught: Ctrl+C";
        LOG(INFO) << "Exiting...";
        cout << "-------------------------------- STATITSTICS ---------------------------------------" << endl;
        cout << "STAGE: SERVER - LOAD TIME" << endl;
        cout << "Mean: " << server_load_time_acc.mean() << " ms, Median: " << server_load_time_acc.median() << " ms, std: " << server_load_time_acc.std() << " ms" << endl;
        cout << "STAGE: SERVER - UPDATE VIEWS TIME" << endl;
        cout << "Mean: " << server_update_time_acc.mean() << " ms, Median: " << server_update_time_acc.median() << " ms, std: " << server_update_time_acc.std() << " ms" << endl;
        cout << "STAGE: CLIENT - GENERATE PTCL TIME" << endl;
        cout << "Mean: " << client_gen_ptcl_time_acc.mean() << " ms, Median: " << client_gen_ptcl_time_acc.median() << " ms, std: " << client_gen_ptcl_time_acc.std() << " ms" << endl;
        cout << "STAGE: CLIENT - VIEW PTCL TIME" << endl;
        cout << "Mean: " << client_view_ptcl_time_acc.mean() << " ms, Median: " << client_view_ptcl_time_acc.median() << " ms, std: " << client_view_ptcl_time_acc.std() << " ms" << endl << endl;
        cout << "STAGE: CLIENT - SAVE RGB TIME" << endl;
        cout << "Mean: " << client_save_rgb_time_acc.mean() << " ms, Median: " << client_save_rgb_time_acc.median() << " ms, std: " << client_save_rgb_time_acc.std() << " ms" << endl;
        cout << "STAGE: CLIENT - SAVE PTCL TIME" << endl;
        cout << "Mean: " << client_save_ptcl_time_acc.mean() << " ms, Median: " << client_save_ptcl_time_acc.median() << " ms, std: " << client_save_ptcl_time_acc.std() << " ms" << endl;
        return EXIT_FAILURE;
    }

    frustum_size_file.flush();
    frustum_size_file.close();
    cout << "-------------------------------- STATITSTICS ---------------------------------------" << endl;
    cout << "STAGE: SERVER - LOAD TIME" << endl;
    cout << "Mean: " << server_load_time_acc.mean() << " ms, Median: " << server_load_time_acc.median() << " ms, std: " << server_load_time_acc.std() << " ms" << endl;
    cout << "STAGE: SERVER - UPDATE VIEWS TIME" << endl;
    cout << "Mean: " << server_update_time_acc.mean() << " ms, Median: " << server_update_time_acc.median() << " ms, std: " << server_update_time_acc.std() << " ms" << endl;
    cout << "STAGE: CLIENT - GENERATE PTCL TIME" << endl;
    cout << "Mean: " << client_gen_ptcl_time_acc.mean() << " ms, Median: " << client_gen_ptcl_time_acc.median() << " ms, std: " << client_gen_ptcl_time_acc.std() << " ms" << endl;
    cout << "STAGE: CLIENT - VIEW PTCL TIME" << endl;
    cout << "Mean: " << client_view_ptcl_time_acc.mean() << " ms, Median: " << client_view_ptcl_time_acc.median() << " ms, std: " << client_view_ptcl_time_acc.std() << " ms" << endl << endl;
    cout << "STAGE: CLIENT - SAVE RGB TIME" << endl;
    cout << "Mean: " << client_save_rgb_time_acc.mean() << " ms, Median: " << client_save_rgb_time_acc.median() << " ms, std: " << client_save_rgb_time_acc.std() << " ms" << endl;
    cout << "STAGE: CLIENT - SAVE PTCL TIME" << endl;
    cout << "Mean: " << client_save_ptcl_time_acc.mean() << " ms, Median: " << client_save_ptcl_time_acc.median() << " ms, std: " << client_save_ptcl_time_acc.std() << " ms" << endl;
    sleep(2);
	return EXIT_SUCCESS;
}