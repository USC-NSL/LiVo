#include "Client/MultiviewClientPool.h"
#include <thread>
#include <csignal>
#include <algorithm>
#include "gflags/gflags.h"
#include "Client/WebRTCClient.h"
#include "Client/PointCloudStreamer.h"
#include "timer.h"
#include "consts.h"

namespace fs = boost::filesystem;

std::atomic_bool running(true);

void sigint_handler(int signum)
{
	LOG(INFO) << "Received Ctrl+C signal. Stopping program...";
	running = false; // set the global flag to true to signal all threads to stop
}

MultiviewClientPool::MultiviewClientPool(const string &path, const int datasetType, const int start_frame_id, const int end_frame_id, uint32_t nThreads) : m_nThreads(nThreads), m_frame_counter(0)
{	
	m_start_frame_id = start_frame_id;
	m_end_frame_id = end_frame_id;
	m_capture_id = start_frame_id;

	if(datasetType == DATASET::PANOPTIC)
	{
		string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);
		m_dataPlayback = make_unique<DataPlayback>(user_data_path, SEQ_NAME, LOG_ID);
    	m_dataPlayback->loadData(START_FRAME, END_FRAME);
	}
	else
		m_dataPlayback = nullptr;

	if(FLAGS_view_ptcl != VIEWER::OPEN3D_VIEWER && FLAGS_render_image_path != "")
		LOG(FATAL) << "Render image only works with Open3D viewer!";

	m_outputPath = FLAGS_render_image_path;
    
    if(FLAGS_render_image_path != "" && !fs::exists(m_outputPath))
    {
        if(fs::create_directories(m_outputPath))
            LOG(INFO) << "Directory " << m_outputPath << " created";
        else
            LOG(FATAL) << "Directory " << m_outputPath << " creation failed";
    }

	m_vStageTimes.resize(PIPELINE_STAGES::NUM_STAGES);

	for (uint32_t i = 0; i < IQ_CAPACITY; i++)
	{
		m_pClients.push_back(new MultiviewClient(path, datasetType));
    	m_pClients[i]->init(deviceIndices_recv, deviceSerialNumbers_recv, calibrations_recv);

        if(!m_pClients[i]->get_calibration_status())
            LOG(FATAL) << "Failed to load Calibration!";
		
		if(datasetType == DATASET::KINECT)
			m_pClients[i]->set_xy_table(xy_tables_recv);
	}

	assert(m_pClients.size() == IQ_CAPACITY);

	m_qrDecoder = cv::QRCodeDetector();

	up = new Mahimahi();
	down = new Mahimahi();
}

MultiviewClientPool::~MultiviewClientPool()
{
	m_qUpdatePtcl.reset();
	m_qGeneratePtcl.reset();
	m_qOpen3D.reset();
	m_qPcl.reset();

	for (uint32_t i = 0; i < IQ_CAPACITY; i++)
	{
		delete m_pClients[i];
	}
	m_pClients.clear();
}

int32_t MultiviewClientPool::detect_qrcode(View *view, int border)
{
	cv::Mat img = color_to_opencv(view->color_image, view->colorWidth, view->colorHeight);

	int offset_x = 20;
	int offset_y = 20;

	cv::Mat tmp;
	cv::extractChannel(img, tmp, 0);
	cv::Mat qrcode_img = tmp(cv::Range(offset_x, offset_x+82), cv::Range(offset_y, offset_y+82));

	LOG(INFO) << "-----------------------------------------------";
	// Detect qr code in each channel

	vector<cv::Point> corners;
	corners.push_back(cv::Point(border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, qrcode_img.rows - border));
	corners.push_back(cv::Point(border, qrcode_img.rows - border));

	// string decode_info = m_qrDecoder.detectAndDecode(qrcode_img, corners);
	string decode_info = m_qrDecoder.decode(qrcode_img, corners);
	LOG(INFO) << "Frame_number: " << decode_info;
	if(decode_info.length() > 0)
	{
		int32_t frame_id = stoi(decode_info);
		return frame_id;
	}
	return -1;
}

void MultiviewClientPool::dummy_run()
{
	std::signal(SIGINT, sigint_handler);

	load_trace(up, down);
	wclient_ready_to_init.store(true);

	vector<thread> threads;
	int nInputObjects = IQ_CAPACITY - 1;
	for (uint32_t i = 0; i < nInputObjects; i++)
    {
        m_qUpdatePtcl.push(m_pClients[i]);
    }

	StopWatch sw_pipeline;
	const int sleep_time = 2;

	while(running)
	{	
		while(bool x = next_frame_ready.load(memory_order_relaxed) == false && running) {
			sleep_ms(sleep_time);
		}

		int temp = next_frame;
		m_capture_id = temp;

		bool valid_views = true;
		for(int i = 0; i < next_views.size(); i++)
		{
			if(!next_views[i]->valid)
			{
				valid_views = false;
				break;
			}
		}

		// TODO: Rajrup: Some frames have ID 0, check why? This is a temporary fix.
		if(valid_views)
			LOG(INFO) << "Frame: " << next_views[0]->frameID;
		else
			LOG(INFO) << "Skipping Frame ...";

		// For WebRTC
		next_frame_ready.store(false);

		// fps.counter(m_capture_id);					// Old FPS counter
		float fps_achieved = fps_counter.fps_counter();	// New FPS counter
		if(m_frame_counter % 10 == 0)
			LOG(INFO) << "Client FPS: " << fps_achieved;
	}
}

void MultiviewClientPool::run() 
{
	std::signal(SIGINT, sigint_handler);

	load_trace(up, down);
	wclient_ready_to_init.store(true);

    vector<thread> threads;
	int nInputObjects = IQ_CAPACITY - 1;		// Viewer thread is not included
	for (uint32_t i = 0; i < nInputObjects; i++)
    {
        m_qUpdatePtcl.push(m_pClients[i]);
    }

	vector<vector<pair<int, uint64_t>>> stageOperationTimes(PIPELINE_STAGES::NUM_STAGES);

	StopWatch sw_pipeline;
	const int sleep_time = 2;
	for (uint32_t i = 0; i < m_nThreads; i++)
	{
		threads.push_back(thread([this, i, sleep_time, &sw_pipeline, &stageOperationTimes, &nInputObjects] 
		{
			switch (i)
			{
				// Client Input From WebRTC
				case PIPELINE_STAGES::UPDATE_PTCL:
				{
					MultiviewClient *pClient = NULL;

					fs::path filepath_update_ptcl{FORMAT("/datassd/pipeline/client_tiled/pipeline_new/output/nocull/0_update_ptcl_receiver_latency.txt")};
					fs::path filepath_update_ptcl_proc{FORMAT("/datassd/pipeline/client_tiled/pipeline_new/output/nocull/0_update_ptcl_receiver_processing_time.txt")};
					fs::path filepath_webrtc_recv{FORMAT("/datassd/pipeline/client_tiled/pipeline_new/output/nocull/0_webrtc_c++_receiver_timestamp.txt")};
					fs::path filepath_fps{FORMAT("/datassd/pipeline/client_tiled/pipeline_new/output/" << SEQ_NAME << "_wifi25_livo_fps.txt")};

					fs::ofstream file_update_ptcl, file_update_ptcl_proc, file_webrtc_recv, file_fps;
					file_update_ptcl.open(filepath_update_ptcl, ios::out | ios::trunc);
					file_update_ptcl_proc.open(filepath_update_ptcl_proc, ios::out | ios::trunc);
					file_webrtc_recv.open(filepath_webrtc_recv, ios::out | ios::trunc);
					file_fps.open(filepath_fps, ios::out | ios::trunc);

					file_update_ptcl << "FrameID,Time" << endl;
					file_update_ptcl_proc << "FrameID,Time" << endl;
					file_webrtc_recv << "FrameID,Time" << endl;
					file_fps << "FrameID,FPS" << endl;

					StopWatch sw_update_ptcl, sw_webrtc, sw_local;
					int cur_frame_id = m_start_frame_id;
					float fps_achieved = 0.0;
					while(running)
					{
						sw_update_ptcl.Restart();
						pClient = NULL;
						while(running && !m_qUpdatePtcl.pop(pClient))
							sleep_ms(sleep_time);				// waiting to pop from the queue
						
						sw_local.Restart();
						pClient->restart_playback();
						if(m_capture_id == m_start_frame_id)
							sw_pipeline.Restart();							// Start the timer for the first frame

						while(bool x = next_frame_ready.load(memory_order_relaxed) == false && running) {
							sleep_ms(sleep_time);
						}
						// LOG(INFO) << "Next frame " << next_frame << " at " << sw.Curr();
						// int temp = next_frame;
						// m_capture_id = temp;
						m_frame_counter++;

						/**
						// DEBUG: Check if the frame ID is consistent
						int test_frame_id = next_views[0]->frameID;
						for(int i = 0; i < next_views.size(); i++)
						{
							if(next_views[i]->frameID != test_frame_id)
							{
								LOG(FATAL) << "Frame ID mismatch, view: " << next_views[i]->viewID << " has frame ID " << next_views[i]->frameID << ", view 0 has frame ID " << test_frame_id;
							}
						}
						*/
						
						pClient->set_curr_frameId(next_views[0]->frameID);
						cur_frame_id = pClient->get_curr_frameId();	

						file_webrtc_recv << cur_frame_id << "," << sw_webrtc.Curr() << endl;

						// // TODO: Rajrup: Some frames have ID 0, check why? This is a temporary fix.
						// if(cur_frame_id == 0)
						// {
						// 	LOG(WARNING) << "Frame ID is 0, skipping...";
						// 	while(running && !m_qUpdatePtcl.push(pClient))
						// 		sleep_ms(sleep_time);
						// 	next_frame_ready.store(false);
						// 	continue;
						// }

						sw_local.Restart();
						bool valid_views = true;
						for(int i = 0; i < next_views.size(); i++)
						{
							if(!next_views[i]->valid)
							{
								valid_views = false;
								break;
							}
						}
						
						if(!valid_views)
						{
							LOG(INFO) << "Skipping Frame ...";
							while(running && !m_qUpdatePtcl.push(pClient))
								sleep_ms(sleep_time);
							next_frame_ready.store(false);
							continue;
						}

						// fps.counter(m_capture_id);					// Old FPS counter
						fps_achieved = fps_counter.fps_counter();	// New FPS counter

						if(m_frame_counter % 10 == 0)
						{
							LOG(INFO) << "Client FPS: " << fps_achieved;
							file_fps << cur_frame_id << "," << fps_achieved << endl;
						}

						// Set the view for the client, if the frame is valid
						if(!pClient->set_view(next_views))
						{
							running = false;
							LOG(FATAL) << "Failed to set view for frame ID " << cur_frame_id;
						}

						// TODO: Rajrup: Send origData to the server
						UserData origData = m_dataPlayback->getUserData(cur_frame_id);
						if(origData.frustum.frameID != cur_frame_id)
						{
							LOG(FATAL) << "Frame ID mismatch, user trace has frame ID " << origData.frustum.frameID << ", but reuired frame id is " << cur_frame_id;
						}
						pClient->set_frustum(origData.frustum);

						// For WebRTC
						next_frame_ready.store(false);
						// detect_qrcode(next_views[0]);

						
						if(!pClient->update_ptcl())
						{
							running = false;
							LOG(FATAL) << "Failed to update ptcl for frame ID " << cur_frame_id;
						}

						stageOperationTimes[PIPELINE_STAGES::UPDATE_PTCL].push_back(make_pair(cur_frame_id, sw_local.ElapsedMs()));
						file_update_ptcl_proc << cur_frame_id << "," << sw_local.ElapsedMs() << endl;

						while(running && !m_qGeneratePtcl.push(pClient))
							sleep_ms(sleep_time);

						bwe.store(down->get_throughput());
						LOG(INFO) << "Frame: " << cur_frame_id << ", BWE: " << bwe;
						m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL].push_back(make_pair(cur_frame_id, sw_update_ptcl.ElapsedMs()));

						file_update_ptcl << cur_frame_id << "," << sw_update_ptcl.ElapsedMs() << endl;
						
						// exit if all the frames are processed
						if(cur_frame_id >= m_end_frame_id)
						{
							LOG(INFO) << "Exiting UPDATE_PTCL stage";
							sleep_ms(sleep_time);
							break;
						}
					}
					break;
				}
				
				// Client Update Ptcl
				case PIPELINE_STAGES::GENERATE_PTCL:
				{
					MultiviewClient *pClient = NULL;

					fs::path filepath_end{FORMAT("/datassd/pipeline/client_tiled/pipeline_new/output/nocull/0_end_frame_recveiver_timestamp.txt")};
					fs::path filepath_render_ptcl{FORMAT("/datassd/pipeline/client_tiled/pipeline_new/output/nocull/0_render_ptcl_receiver_latency.txt")};
					fs::path filepath_render_ptcl_proc{FORMAT("/datassd/pipeline/client_tiled/pipeline_new/output/nocull/0_render_ptcl_receiver_processing_time.txt")};
					fs::ofstream file_end, file_render_ptcl_proc, file_render_ptcl;
					file_end.open(filepath_end, ios::out | ios::trunc);
					file_render_ptcl.open(filepath_render_ptcl, ios::out | ios::trunc);
					file_render_ptcl_proc.open(filepath_render_ptcl_proc, ios::out | ios::trunc);

					file_end << "FrameID,Time" << endl;
					file_render_ptcl << "FrameID,Time" << endl;
					file_render_ptcl_proc << "FrameID,Time" << endl;

					StopWatch sw_render_ptcl, sw_end, sw_local;

					int cur_frame_id = m_start_frame_id;
					while(running)
					{
						sw_render_ptcl.Restart();
						pClient = NULL;
						while(running && !m_qGeneratePtcl.pop(pClient))
							sleep_ms(sleep_time);
						
						sw_local.Restart();
						cur_frame_id = pClient->get_curr_frameId();
						if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
						{
							PclData *pcl_data = new PclData();
							pcl_data->frame_id = cur_frame_id;
							pClient->generate_pcl_object(pcl_data->pcl_object);
							
							while(running && !m_qPcl.push(pcl_data))
								sleep_ms(sleep_time);
						}
						else if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
						{
							O3DData *o3d_data = new O3DData();
							o3d_data->frame_id = cur_frame_id;
							pClient->get_open3d_points_colors(o3d_data->points, o3d_data->colors);

							while(running && !m_qOpen3D.push(o3d_data))
								sleep_ms(sleep_time);
						}
						else
						{
							// No viewer
							while(running && !m_qDummy.push(cur_frame_id))
								sleep_ms(sleep_time);
						}

						file_render_ptcl << cur_frame_id << "," << sw_render_ptcl.ElapsedMs() << endl;
						file_end << cur_frame_id << "," << sw_end.Curr() << endl;

						stageOperationTimes[PIPELINE_STAGES::GENERATE_PTCL].push_back(make_pair(cur_frame_id, sw_local.ElapsedMs()));
						file_render_ptcl_proc << cur_frame_id << "," << sw_local.ElapsedMs() << endl;
						
						// pClient->restart_playback();
						// while(bool x = next_frame_ready.load(memory_order_relaxed) == false && running) {
						// 	sleep_ms(sleep_time);
						// }
						// LOG(INFO) << "Next frame " << next_frame << " at " << sw.Curr();

						// pClient->set_curr_frameId(next_frame);
						// if(!pClient->set_view(next_views))
						// {
						// 	LOG(INFO) << "Failed to set view for frame ID " << cur_frame_id;
						// 	exit(EXIT_FAILURE);
						// }
						// // For WebRTC
						// next_frame_ready.store(false);
						
						// if(!pClient->update_ptcl())
						// {
						// 	running = false;
						// 	LOG(FATAL) << "Failed to update ptcl for frame ID " << cur_frame_id;
						// }

						// LOG(INFO) << "Update ptcl " << m_capture_id << " at " << sw.Curr();
						// fps.counter(m_capture_id);

						// LOG(INFO) << "update ptcl " << cur_frame_id;
						
						
						// -------------------------- With Viewer --------------------------
						// while(running && !m_qViewPtcl.push(pClient))
						// 	sleep_ms(sleep_time);

						// // ----------------------- Without Viewer -----------------------
						// // If we only update PTCL						
						// m_vClientLatency.push_back(make_pair(cur_frame_id, pClient->get_elapsed_time()));
						// m_vFps.push_back(make_pair(cur_frame_id, sw.ElapsedMs()));

						// // without viewer
						// if(cur_frame_id <= (m_end_frame_id - SQ_CAPACITY))
						// {
						// 	// Insert to the input queue of the server
						// 	while(running && !m_qUpdatePtcl.push(pClient)) {
						// 		sleep_ms(sleep_time);
						// 	}
						// }

						if(cur_frame_id <= (m_end_frame_id - min(nInputObjects, SQ_CAPACITY)))
						{
							// LOG(INFO) << "Thread " << i << " push to InputClient queue for frame id: " << cur_frame_id << " atomic id: " << m_capture_id;

							// Insert to the input queue of the client
							while(running && !m_qUpdatePtcl.push(pClient))
								sleep_ms(sleep_time);
						}

						m_vStageTimes[PIPELINE_STAGES::GENERATE_PTCL].push_back(make_pair(cur_frame_id, sw.ElapsedMs()));


						// exit if all the frames are processed
						if(cur_frame_id >= m_end_frame_id)
						{
							LOG(INFO) << "Exiting GENERATE_PTCL stage";
							sleep_ms(sleep_time);
							break;
						}
					}
					break;
				}
				
				// Client generate and view point cloud (If required move the ptcl generation to previous stage)
				case PIPELINE_STAGES::VIEW_PTCL:
				{
					StopWatch sw, sw_local;
					int cur_frame_id = m_start_frame_id;
					int num_frames_disp = 0;

					unique_ptr<PointCloudViewer> pcl_viewer(nullptr);
					if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
						pcl_viewer = make_unique<PointCloudViewer>("viewer");
					
					visualization::Visualizer visualizer;
					std::shared_ptr<geometry::PointCloud> pcd_ptr = nullptr;
					if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
					{
						// Visualizer initialization
						// visualizer.CreateVisualizerWindow("Open3D", 1600, 900, 0, 0);

						// Read camera instrinsics to set the view control for viewer's camera
						// Asuming the camera internals are same across frames
						UserData origData = m_dataPlayback->getUserData(m_start_frame_id + 1);
						float ratio = origData.frustum.ratio;
						float nearD = origData.frustum.nearD;
						float farD = origData.frustum.farD;

						int width = 1000;
						int height = int((float)width / ratio);

						// visualizer.CreateVisualizerWindow("Display", 1000, 595, 0, 0);
						visualizer.CreateVisualizerWindow("Display", width, height, 0, 0);
						visualizer.GetRenderOption().SetPointSize(4.0);                 // Default is 5.0
						visualization::ViewControl& view_ctl = visualizer.GetViewControl();
						
						view_ctl.SetConstantZNear(nearD);
        				view_ctl.SetConstantZFar(farD);
						pcd_ptr = std::make_shared<geometry::PointCloud>();
					}

					bool first_frame = true;
					while(running)
					{
						sw.Restart();
						if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
						{
							PclData *pcl_data = NULL;
							while(running && !m_qPcl.pop(pcl_data))
								sleep_ms(sleep_time);
							
							sw_local.Restart();
							cur_frame_id = pcl_data->frame_id;
							pcl_viewer->displayCullClipThread(pcl_data->pcl_object);

							delete pcl_data;
						}
						else if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
						{
							O3DData *o3d_data = NULL;
							while(running && !m_qOpen3D.pop(o3d_data))
								sleep_ms(sleep_time);
							
							sw_local.Restart();
							cur_frame_id = o3d_data->frame_id;
							UserData origData = m_dataPlayback->getUserData(cur_frame_id);
							if(origData.frustum.frameID != cur_frame_id)
							{
								LOG(FATAL) << "Frame ID mismatch, user trace has frame ID " << origData.frustum.frameID << ", but reuired frame id is " << cur_frame_id;
							}

							while(bool x = new_frustum.load(memory_order_relaxed) == true && running)
            					sleep_ms(sleep_time);

							// Send the frustum to the server
							frustum_send = origData.frustum;
							// Change furstum position to Unity coordinates - Negative y-axis
							frustum_send.pos(1) *= -1.0f;

							new_frustum.store(true);

							visualization::ViewControl& ctr = visualizer.GetViewControl();
							// ctr.ChangeFieldOfView(60);

							camera::PinholeCameraParameters camera_params;
							ctr.ConvertToPinholeCameraParameters(camera_params);

							// Tested by me and Christina that this works.
							Eigen::Vector4d q_rotation{origData.quat[3], -1.0 * origData.quat[0], origData.quat[1], -1.0 * origData.quat[2]};
							Eigen::Matrix3d R = geometry::Geometry3D::GetRotationMatrixFromQuaternion(q_rotation);
							Eigen::Matrix4d transform;
							transform << R.coeff(0, 0), R.coeff(0, 1), R.coeff(0, 2), origData.pos[0],
									R.coeff(1, 0), R.coeff(1, 1), R.coeff(1, 2), origData.pos[1],
									R.coeff(2, 0), R.coeff(2, 1), R.coeff(2, 2), origData.pos[2],
									0, 0, 0, 1;
							camera_params.extrinsic_ << transform.inverse();
							ctr.ConvertFromPinholeCameraParameters(camera_params);

							pcd_ptr->Clear();
							pcd_ptr->points_ = o3d_data->points;
							pcd_ptr->colors_ = o3d_data->colors;
							// Matrix4d transform;
							// transform << 	1, 0, 0, 0,
							// 				0, -1, 0, 0,
							// 				0, 0, -1, 0,
							// 				0, 0, 0, 1; 
							// pcd_ptr->Transform(transform);

							if(first_frame)
							{
								visualizer.AddGeometry(pcd_ptr);
								first_frame = false;
							}
							else
								visualizer.UpdateGeometry(pcd_ptr);
							visualizer.PollEvents();
							visualizer.UpdateRender();

							int initial_offset = 300;
							num_frames_disp++;
							uint32_t total_frames = cur_frame_id - m_start_frame_id;
							float rendered_frames = (num_frames_disp - initial_offset)/(float)(total_frames - initial_offset);

							if(num_frames_disp > initial_offset && total_frames % 10 == 0)
								LOG(INFO) << "Rendered frames: " << rendered_frames << ", Lost frames: " << (1.0 - rendered_frames) << endl;

							if(FLAGS_render_image_path != "" && cur_frame_id % 100 == 0)
							{
								string output_file = FORMAT(FLAGS_render_image_path << cur_frame_id << ".png");
								visualizer.CaptureScreenImage(output_file);
								LOG(INFO) << "Saved rendered image to " << output_file;
							}

							// delete o3d_data;
							while(running && !m_qSavePtcl.push(o3d_data))
								sleep_ms(sleep_time);
						}
						else
						{
							// No viewer
							uint32_t dummy = 0;
							while(running && !m_qDummy.pop(dummy))
								sleep_ms(sleep_time);
							
							sw_local.Restart();
							cur_frame_id = dummy;
						}

						stageOperationTimes[PIPELINE_STAGES::VIEW_PTCL].push_back(make_pair(cur_frame_id, sw_local.ElapsedMs()));

						m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL].push_back(make_pair(cur_frame_id, sw.ElapsedMs()));
						m_vFps.push_back(make_pair(cur_frame_id, sw_pipeline.ElapsedMs()));

						// exit if all the frames are processed
						if(cur_frame_id >= m_end_frame_id)
						{
							LOG(INFO) << "Exiting VIEW_PTCL stage";
							visualizer.DestroyVisualizerWindow();
							sleep_ms(sleep_time);
							break;
						}
					}
					break;
				}
				case PIPELINE_STAGES::SAVE_RENDER:
				{
					StopWatch sw, sw_local;
					int cur_frame_id = m_start_frame_id;
					geometry::PointCloud o3d_pcd;
					while(running)
					{
						if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
						{
							O3DData *o3d_data = NULL;
							while(running && !m_qSavePtcl.pop(o3d_data))
								sleep_ms(sleep_time);
							
							// LOG(INFO) << "Saving thread for frame ID: " << o3d_data->frame_id;
							cur_frame_id = o3d_data->frame_id;

							if(FLAGS_save_ptcl_path != "" && cur_frame_id % 100 == 0)
							{
								string output_file = FORMAT(FLAGS_save_ptcl_path << cur_frame_id << ".ply");

								// Correct point cloud from upside down to right side up
								o3d_pcd.points_ = o3d_data->points;
								o3d_pcd.colors_ = o3d_data->colors;

								Eigen::Matrix4d flip_transform;
								flip_transform << 1, 0, 0, 0,
												0, -1, 0, 0,
												0, 0, 1, 0,
												0, 0, 0, 1;
								o3d_pcd.Transform(flip_transform);
								io::WritePointCloud(output_file, o3d_pcd);
								LOG(INFO) << "Client: Saved rendered ptcl to " << output_file;
							}

							delete o3d_data;

							// exit if all the frames are processed
							if(cur_frame_id >= m_end_frame_id)
							{
								LOG(INFO) << "Exiting SAVE_RENDER stage";
								sleep_ms(sleep_time);
								break;
							}
						}
					}
					break;
				}
				default:
					break;
			}
		}));
	}

	LOG(INFO) << "Waiting for threads to join...";
	for (auto &t : threads)
	{
		t.join();
	}
	LOG(INFO) << "Threads joined!";

	// Examine latency
	assert(m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL].size() == m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL].size());
	
	vector<float> avg_stage_latency(PIPELINE_STAGES::NUM_STAGES, 0.0f);
	vector<float> avg_stage_operation_latency(PIPELINE_STAGES::NUM_STAGES, 0.0f);
	float avg_fps = 0.0f;
	double avg_client_latency = 0.0;
	
	int start_frame = 5;
	int end_frame = m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL].size() - 1;
	// skip the first 5 frames during calculation

	int num_frames = 0;
	for(uint32_t i = start_frame; i < m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL].size(); i++)
	{
		assert(m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL][i].first == m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL][i].first);

		for(int j = 0; j < PIPELINE_STAGES::NUM_STAGES; j++)
		{
			avg_stage_latency[j] += m_vStageTimes[j][i].second;
			avg_stage_operation_latency[j] += stageOperationTimes[j][i].second;
			avg_client_latency += m_vStageTimes[j][i].second;
		}
		num_frames++;
	}
	
	for(int j = 0; j < PIPELINE_STAGES::NUM_STAGES; j++)
	{
		avg_stage_latency[j] /= num_frames;
		avg_stage_operation_latency[j] /= num_frames;
	}
	avg_client_latency /= num_frames;
	avg_fps = float(num_frames) * 1000.0f / (m_vFps[end_frame].second - m_vFps[start_frame].second);

	LOG(INFO) << "------------------------------------------------------------";
	LOG(INFO) << "Frames: " << num_frames;

	LOG(INFO) << "Update ptcl: " << avg_stage_latency[PIPELINE_STAGES::UPDATE_PTCL] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::UPDATE_PTCL] << " ms";
	LOG(INFO) << "Generate ptcl: " << avg_stage_latency[PIPELINE_STAGES::GENERATE_PTCL] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::GENERATE_PTCL] << " ms";
	LOG(INFO) << "View ptcl: " << avg_stage_latency[PIPELINE_STAGES::VIEW_PTCL] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::VIEW_PTCL] << " ms";
	LOG(INFO) << "Client latency: " << avg_client_latency << " ms";
	// LOG(INFO) << "End-to-end latency: " << avg_total_latency << " ms";
	LOG(INFO) << "FPS: " << avg_fps;
}

MultiviewClientPoolPtcl::MultiviewClientPoolPtcl(const string &path, const int datasetType, const int start_frame_id, const int end_frame_id, string compression_type, zstd_options zstd_params, draco_options draco_params, uint32_t nThreads) : 
												m_nThreads(nThreads), m_frame_counter(0), m_compression_type(compression_type), m_zstd_params(zstd_params), m_draco_params(draco_params)
{	
	m_start_frame_id = start_frame_id;
	m_end_frame_id = end_frame_id;
	m_capture_id = start_frame_id;

	if(datasetType == DATASET::PANOPTIC)
	{
		string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);
		m_dataPlayback = make_unique<DataPlayback>(user_data_path, SEQ_NAME, LOG_ID);
    	m_dataPlayback->loadData(START_FRAME, END_FRAME);
	}
	else
		m_dataPlayback = nullptr;

	if(FLAGS_view_ptcl != VIEWER::OPEN3D_VIEWER && FLAGS_render_image_path != "")
		LOG(FATAL) << "Render image only works with Open3D viewer!";

	m_outputPath = FLAGS_render_image_path;
    
    if(FLAGS_render_image_path != "" && !fs::exists(m_outputPath))
    {
        if(fs::create_directories(m_outputPath))
            LOG(INFO) << "Directory " << m_outputPath << " created";
        else
            LOG(FATAL) << "Directory " << m_outputPath << " creation failed";
    }

	m_vStageTimes.resize(PIPELINE_PTCL_STAGES::NUM_STAGES);

	for (uint32_t i = 0; i < IQ_PTCL_CAPACITY; i++)
	{
		m_pClients.push_back(new MultiviewClient(path, datasetType));
    	m_pClients[i]->init(deviceIndices_recv, deviceSerialNumbers_recv, calibrations_recv);

        if(!m_pClients[i]->get_calibration_status())
            LOG(FATAL) << "Failed to load Calibration!";
		
		if(datasetType == DATASET::KINECT)
			m_pClients[i]->set_xy_table(xy_tables_recv);
	}

	assert(m_pClients.size() == IQ_PTCL_CAPACITY);

	up = new Mahimahi();
	down = new Mahimahi();
}

MultiviewClientPoolPtcl::~MultiviewClientPoolPtcl()
{
	m_qDecompressPtcl.reset();
	m_qGeneratePtcl.reset();
	m_qOpen3D.reset();
	m_qPcl.reset();

	for (uint32_t i = 0; i < IQ_PTCL_CAPACITY; i++)
	{
		delete m_pClients[i];
	}
	m_pClients.clear();
}

void MultiviewClientPoolPtcl::run_ptcl() 
{
	std::signal(SIGINT, sigint_handler);

	load_trace(up, down);
	wclient_ready_to_init.store(true);

    vector<thread> threads;
	int nInputObjects = IQ_PTCL_CAPACITY - 1;	// Viewer thread is not included
	for (uint32_t i = 0; i < nInputObjects; i++)
    {
        m_qDecompressPtcl.push(m_pClients[i]);
    }

	StopWatch sw_pipeline;
	const int sleep_time = 2;
	for (uint32_t i = 0; i < m_nThreads; i++)
	{
		threads.push_back(thread([this, i, sleep_time, &sw_pipeline, &nInputObjects] 
		{
			switch (i)
			{
				// Client Ptcl Input From WebSocket
				case PIPELINE_PTCL_STAGES::DECOMPRESS_PTCL:
				{
					MultiviewClient *pClient = NULL;
					StopWatch sw_stage;
					int cur_frame_id = m_start_frame_id;
					float fps_achieved = 0.0;
					while(running)
					{
						sw_stage.Restart();
						pClient = NULL;
						while(running && !m_qDecompressPtcl.pop(pClient))
							sleep_ms(sleep_time);				// waiting to pop from the queue

						pClient->restart_playback();
						if(m_capture_id == m_start_frame_id)
							sw_pipeline.Restart();							// Start the timer for the first frame
						// pClient->set_curr_frameId(m_capture_id);
						// m_capture_id++;

						while(bool x = next_frame_ready.load(memory_order_relaxed) == false && running) {
							sleep_ms(sleep_time);
						}
						// LOG(INFO) << "Next frame " << next_frame << " at " << sw.Curr();
						m_capture_id = next_frame.load();
						LOG(INFO) << "Frame ID: " << m_capture_id;

						pClient->set_curr_frameId(m_capture_id);
						cur_frame_id = pClient->get_curr_frameId();

						// TODO: Rajrup: Send origData to the server
						UserData origData = m_dataPlayback->getUserData(cur_frame_id);
						pClient->set_frustum(origData.frustum);

						if(!pClient->decompress_ptcl(compressed_ptcl_buf, compressed_ptcl_buf_size, m_compression_type, m_zstd_params, m_draco_params))
						{
							running = false;
							LOG(FATAL) << "Failed to decompress ptcl for frame ID " << cur_frame_id;
						}

						// For signalling the receiver to start the next frame
						next_frame_ready.store(false);

						while(running && !m_qGeneratePtcl.push(pClient))
							sleep_ms(sleep_time);

						fps_achieved = fps_counter.fps_counter();	// New FPS counter
						LOG(INFO) << "Client FPS: " << fps_achieved;
						m_vStageTimes[PIPELINE_PTCL_STAGES::DECOMPRESS_PTCL].push_back(make_pair(cur_frame_id, sw.ElapsedMs()));

						// exit if all the frames are processed
						if(cur_frame_id >= m_end_frame_id)
						{
							LOG(INFO) << "Exiting DECOMPRESS_PTCL stage";
							sleep_ms(sleep_time);
							break;
						}
						LOG(INFO) << "Decompression Stage Time: " << sw_stage.ElapsedMs() << " ms";
					}
					break;
				}
				
				// Client Update Ptcl
				case PIPELINE_PTCL_STAGES::GENERATE_PTCL:
				{
					MultiviewClient *pClient = NULL;
					StopWatch sw;
					int cur_frame_id = m_start_frame_id;
					while(running)
					{
						sw.Restart();
						pClient = NULL;
						while(running && !m_qGeneratePtcl.pop(pClient))
							sleep_ms(sleep_time);
						
						cur_frame_id = pClient->get_curr_frameId();
						if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
						{
							LOG(ERROR) << "PCL viewer not supported for ptcl pipeline!";
							PclData *pcl_data = new PclData();
							pcl_data->frame_id = cur_frame_id;
							// pClient->generate_pcl_object(pcl_data->pcl_object);
							
							while(running && !m_qPcl.push(pcl_data))
								sleep_ms(sleep_time);
						}
						else if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
						{
							O3DData *o3d_data = new O3DData();
							o3d_data->frame_id = cur_frame_id;
							pClient->update_ptcl_3D_decompress_open3D(m_compression_type, o3d_data->points, o3d_data->colors);

							while(running && !m_qOpen3D.push(o3d_data))
								sleep_ms(sleep_time);
						}

						if(cur_frame_id <= (m_end_frame_id - min(nInputObjects, SQ_PTCL_CAPACITY)))
						{
							// LOG(INFO) << "Thread " << i << " push to InputClient queue for frame id: " << cur_frame_id << " atomic id: " << m_capture_id;

							// Insert to the input queue of the client
							while(running && !m_qDecompressPtcl.push(pClient))
								sleep_ms(sleep_time);
						}

						m_vStageTimes[PIPELINE_PTCL_STAGES::GENERATE_PTCL].push_back(make_pair(cur_frame_id, sw.ElapsedMs()));


						// exit if all the frames are processed
						if(cur_frame_id >= m_end_frame_id)
						{
							LOG(INFO) << "Exiting GENERATE_PTCL stage";
							sleep_ms(sleep_time);
							break;
						}
					}
					break;
				}
				
				// Client generate and view point cloud (If required move the ptcl generation to previous stage)
				case PIPELINE_STAGES::VIEW_PTCL:
				{
					StopWatch sw, sw_local;
					int cur_frame_id = m_start_frame_id;

					unique_ptr<PointCloudViewer> pcl_viewer(nullptr);
					if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
						pcl_viewer = make_unique<PointCloudViewer>("viewer");
					
					visualization::Visualizer visualizer;
					std::shared_ptr<geometry::PointCloud> pcd_ptr = nullptr;
					if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
					{
						// Visualizer initialization
						visualizer.CreateVisualizerWindow("Open3D", 1600, 900, 0, 0);
						pcd_ptr = std::make_shared<geometry::PointCloud>();
					}

					bool first_frame = true;
					while(running)
					{
						sw.Restart();
						if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
						{
							PclData *pcl_data = NULL;
							while(running && !m_qPcl.pop(pcl_data))
								sleep_ms(sleep_time);
							
							sw_local.Restart();
							cur_frame_id = pcl_data->frame_id;
							pcl_viewer->displayCullClipThread(pcl_data->pcl_object);

							delete pcl_data;
						}
						else if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
						{
							O3DData *o3d_data = NULL;
							while(running && !m_qOpen3D.pop(o3d_data))
								sleep_ms(sleep_time);
							
							sw_local.Restart();
							cur_frame_id = o3d_data->frame_id;
							LOG(INFO) << "Viewing Frame: " << cur_frame_id;
							UserData origData = m_dataPlayback->getUserData(cur_frame_id);

							visualization::ViewControl& ctr = visualizer.GetViewControl();
							ctr.ChangeFieldOfView(60);

							camera::PinholeCameraParameters camera_params;
							ctr.ConvertToPinholeCameraParameters(camera_params);

							Eigen::Vector4d q_rotation{origData.quat[3], origData.quat[0], origData.quat[1], origData.quat[2]};
							Eigen::Matrix3d R = geometry::Geometry3D::GetRotationMatrixFromQuaternion(q_rotation);
							Eigen::Matrix4d temp;
							temp << R.coeff(0, 0), R.coeff(0, 1), R.coeff(0, 2), origData.pos[0],
									R.coeff(1, 0), R.coeff(1, 1), R.coeff(1, 2), origData.pos[1],
									R.coeff(2, 0), R.coeff(2, 1), R.coeff(2, 2), origData.pos[2],
									0, 0, 0, 1;
							camera_params.extrinsic_ << temp.inverse();
							ctr.ConvertFromPinholeCameraParameters(camera_params);

							pcd_ptr->Clear();
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

							if(m_outputPath != "")
							{
								string output_file = FORMAT(m_outputPath << cur_frame_id << ".png");
								visualizer.CaptureScreenImage(output_file);
								LOG(INFO) << "Saved rendered image to " << output_file;
							}
							
							delete o3d_data;
						}
						else
						{
							// No viewer
							uint32_t dummy = 0;
							while(running && !m_qDummy.pop(dummy))
								sleep_ms(sleep_time);
							
							sw_local.Restart();
							cur_frame_id = dummy;
						}

						m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL].push_back(make_pair(cur_frame_id, sw.ElapsedMs()));

						// exit if all the frames are processed
						if(cur_frame_id >= m_end_frame_id)
						{
							LOG(INFO) << "Exiting VIEW_PTCL stage";
							visualizer.DestroyVisualizerWindow();
							sleep_ms(sleep_time);
							break;
						}
					}
					break;
				}
				default:
					break;
			}
		}));
	}

	LOG(INFO) << "Waiting for threads to join...";
	for (auto &t : threads)
	{
		t.join();
	}
	LOG(INFO) << "Threads joined!";
}