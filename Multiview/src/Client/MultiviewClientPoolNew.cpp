#include "Client/MultiviewClientPoolNew.h"
#include <thread>
#include <csignal>
#include <algorithm>
#include "gflags/gflags.h"
#include "Client/NetworkReceiver.h"
#include "Client/PointCloudStreamer.h"
#include "timer.h"
#include "consts.h"

namespace fs = boost::filesystem;

#define SKIP_FIRST_FRAMES 500

std::atomic_bool running(true);

void sigint_handler(int signum)
{
	LOG(INFO) << "Received Ctrl+C signal. Stopping program...";
	running = false; // set the global flag to true to signal all threads to stop
}

MultiviewClientPoolNew::MultiviewClientPoolNew(const std::string &path, const int datasetType, const int start_frame_id, const int end_frame_id, uint32_t nThreads) : m_nThreads(nThreads), m_frame_counter(0), fps_limiter(FLAGS_client_fps), fps_counter()
{	
	m_start_frame_id = start_frame_id;
	m_end_frame_id = end_frame_id;
	m_capture_id = start_frame_id;

	if(datasetType == DATASET::PANOPTIC)
	{
		std::string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);
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
    	m_pClients[i]->init(deviceIndices_recv2, deviceSerialNumbers_recv2, calibrations_recv2);

        if(!m_pClients[i]->get_calibration_status())
            LOG(FATAL) << "Failed to load Calibration!";
		
		if(datasetType == DATASET::KINECT)
			m_pClients[i]->set_xy_table(xy_tables_recv2);
	}

	CHECK(m_pClients.size() == IQ_CAPACITY) << "Input queue capacity mismatch!";
	CHECK(cimg_h2 == dimg_h2 && cimg_w2 == dimg_w2) << "Color and Depth image dimensions mismatch!";

	for (int i = 0; i < deviceIndices_recv2.size(); i++) 
    {
        View* v = new View();
        v->initView(dimg_w2, dimg_h2, cimg_w2, cimg_h2);
        c_views.push_back(v);
    }

	m_qrDecoder = cv::QRCodeDetector();

	up = new Mahimahi();
	down = new Mahimahi();

	bs = new BitrateSplitterClient(cimg_h2, cimg_w2, deviceIndices_recv2.size(), FLAGS_d2c_split);

	if(FLAGS_use_mm)
		load_trace(up, down);
}

MultiviewClientPoolNew::~MultiviewClientPoolNew()
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

int32_t MultiviewClientPoolNew::detect_qrcode(View *view, int border)
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

	// std::string decode_info = m_qrDecoder.detectAndDecode(qrcode_img, corners);
	std::string decode_info = m_qrDecoder.decode(qrcode_img, corners);
	LOG(INFO) << "Frame_number: " << decode_info;
	if(decode_info.length() > 0)
	{
		int32_t frame_id = stoi(decode_info);
		return frame_id;
	}
	return -1;
}


void MultiviewClientPoolNew::dummy_run()
{
	std::signal(SIGINT, sigint_handler);

	load_trace(up, down);
	// wclient_ready_to_init.store(true);

	// Initialize WebRTC Server
	unique_ptr<NetworkReceiverPool> wclient = make_unique<NetworkReceiverPool>(FLAGS_save_frame, "/home/lei/data/pipeline/client_tiled/pipeline_new/test/"); // TODO, CHANGE, MODIFY: Output directory for test dumps
	// vector<thread> threads;
	// int nInputObjects = IQ_CAPACITY - 1;
	// for (uint32_t i = 0; i < nInputObjects; i++)
    // {
    //     m_qUpdatePtcl.push(m_pClients[i]);
    // }

	// fs::path filepath_webrtc_recv{FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/statistics/webrtc_receiver_timestamp.txt")};
	// fs::path filepath_fps{FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/statistics/fps.txt")};
	// fs::ofstream file_webrtc_recv, file_fps;
	// file_webrtc_recv.open(filepath_webrtc_recv, ios::out | ios::trunc);
	// file_fps.open(filepath_fps, ios::out | ios::trunc);
	// file_webrtc_recv << "FrameID,Time" << endl;
	// file_fps << "FrameID,FPS" << endl;

	StopWatch sw_webrtc;
	const int sleep_time = 2;

	uint32_t frame_id = m_start_frame_id;
	uint32_t lost_frames = 0;

	while(running)
	{
		if(frame_id == m_end_frame_id)
			break;
		
		// fps_limiter.rate_limit();
		RECV_STATUS status = wclient->poll_frame(c_views, frame_id);

		if(status == RECV_STATUS::FRAME_SKIP)
		{
			LOG(WARNING) << "Lost Frame ID: " << frame_id;
			lost_frames++;
			frame_id++;
			continue;
		}
		else if(status == RECV_STATUS::FRAME_WAIT)
			LOG(FATAL) << "Triggered frame wait!";

		if(c_views[0]->frameID != frame_id)
			LOG(FATAL) << "Frame ID mismatch: Received " << c_views[0]->frameID << ", Expected " << frame_id;

		float fps_achieved = fps_counter.fps_counter();	// New FPS counter
		LOG(INFO) << "Client - Frame ID: " << frame_id << ", RECV Time: " << sw_webrtc.Curr() << " ms";
		// file_fps << frame_id << "," << fps_achieved << endl;
		// file_webrtc_recv << frame_id << "," << sw_webrtc.Curr() << endl;

		bool valid_views = true;
		for(int i = 0; i < c_views.size(); i++)
		{
			if(!c_views[i]->valid && c_views[i]->frameID != frame_id)
			{
				valid_views = false;
				break;
			}
		}
		
		if(valid_views)
			LOG(INFO) << "Frame: " << c_views[0]->frameID;
		else
			LOG(ERROR) << "Invalid Frame ID: " << c_views[0]->frameID;
		
		if(m_frame_counter % 10 == 0)
		{
			LOG(INFO) << "Client FPS: " << fps_achieved;
			int total_frames = frame_id - m_start_frame_id + 1;
			LOG(INFO) << "#Actual Frames: " << total_frames << ", #Received Frames: " << m_frame_counter << ", #Lost Frames: " << lost_frames << ", Loss Rate: " << (float)lost_frames / total_frames;
		}
		
		frame_id++;
		m_frame_counter++;
	}
}

void MultiviewClientPoolNew::run() 
{
	std::signal(SIGINT, sigint_handler);
	// wclient_ready_to_init.store(true);

	// Initialize WebRTC Server
	std::string save_recv_path = "/home/lei/data/pipeline/client_tiled/pipeline_new/test/";										// TODO, CHANGE, MODIFY: Output directory for test dumps
	if(FLAGS_render_image_path != "")
		save_recv_path = FLAGS_render_image_path;
	unique_ptr<NetworkReceiverPool> wclient = make_unique<NetworkReceiverPool>(FLAGS_save_frame, FLAGS_render_image_path);

    vector<thread> threads;
	int nInputObjects = IQ_CAPACITY - 1;		// Viewer thread is not included
	for (uint32_t i = 0; i < nInputObjects; i++)
    {
        m_loadFrame.push(m_pClients[i]);
    }

	vector<vector<pair<int, uint64_t>>> stageOperationTimes(PIPELINE_STAGES::NUM_STAGES);

	StopWatch sw_pipeline;
	const int sleep_time = 2;
	// std::string method = "livo_nocull";
	std::string method = "livo";		// TODO: Rajrup: Changee this to livo. Also Change path in ReceiverFrameBuffer.cpp in ReceiverFrameBuffer() constructor

	std::string path = FORMAT("/datassd/pipeline_cpp/server_tiled/e2e_latency/" << method << "/"); 
	create_folder(path);

	std::string ablation_dir = FORMAT("/datassd/pipeline_cpp/server_tiled/ablation/" << method << "/");
	create_folder(ablation_dir);

	std::string out_bitrate_dir = FORMAT(FLAGS_output_dir << "/bitrate_split_s_" << FLAGS_server_cull << "/log" << LOG_ID << "/");
	create_folder(out_bitrate_dir);

	// Threads for each stage of the pipeline
	for (uint32_t i = 0; i < m_nThreads; i++)
	{
		threads.push_back(thread([this, i, sleep_time, &wclient, &sw_pipeline, &stageOperationTimes, &nInputObjects, &path, &ablation_dir, &out_bitrate_dir] 
		{
			switch (i)
			{
				// Client Input From WebRTC
				case PIPELINE_STAGES::LOAD_FRAME:
				{
					MultiviewClient *pClient = NULL;
					StopWatch sw_load_frame, sw_webrtc, sw_local;

					fs::path filepath_recv_frame{FORMAT(path << "0_recv_frame_receiver_latency.txt")};
					fs::path filepath_recv_frame_proc{FORMAT(path << "0_recv_frame_receiver_processing_time.txt")};
					fs::path filepath_fps{FORMAT(ablation_dir << SEQ_NAME << "_wifi25_livo_fps.txt")};
					// fs::path filepath_bwe_adapt{FORMAT(out_bitrate_dir << "bwe" << bwe2.load() << "_adapt.txt")};

					fs::ofstream file_recv_frame, file_recv_frame_proc, file_fps;
					file_recv_frame.open(filepath_recv_frame, ios::out | ios::trunc);
					file_recv_frame_proc.open(filepath_recv_frame_proc, ios::out | ios::trunc);
					file_fps.open(filepath_fps, ios::out | ios::trunc);
					// file_bwe_adapt.open(filepath_bwe_adapt, ios::out | ios::trunc);

					file_recv_frame << "FrameID,Time" << endl;
					file_recv_frame_proc << "FrameID,Time" << endl;
					file_fps << "FrameID,FPS" << endl;
					// file_bwe_adapt << "FrameID,cBWE,dBWE" << endl;

					int frame_id = m_start_frame_id;
					int lost_frames = 0;
					bool lost_frame_reset = false;
					float fps_achieved = 0.0;
					while(running)
					{	
						sw_load_frame.Restart();
						pClient = NULL;
						while(running && !m_loadFrame.pop(pClient))
							sleep_ms(sleep_time);				// waiting to pop from the queue
						
						// fps_limiter.rate_limit();
						sw_local.Restart();
						pClient->restart_playback();
						if(frame_id == m_start_frame_id)
							sw_pipeline.Restart();							// Start the timer for the first frame

						RECV_STATUS status = RECV_STATUS::FRAME_SKIP;
						while(status == RECV_STATUS::FRAME_SKIP)
						{
							status = wclient->poll_frame(c_views, frame_id);
							if(status == RECV_STATUS::FRAME_READY)
								break;
							else if(status == RECV_STATUS::FRAME_WAIT)
								LOG(FATAL) << "Triggered frame wait!";
							else if(status == RECV_STATUS::FRAME_SKIP)
							{
								LOG(WARNING) << "Lost Frame ID: " << frame_id;
								lost_frames++;
								frame_id++;
							}
						}

						if(c_views[0]->frameID != frame_id)
							LOG(FATAL) << "Frame ID mismatch: Received " << c_views[0]->frameID << ", Expected " << frame_id;

						// DEBUG: Check if the frame ID is consistent
						bool valid_views = true;
						for(int i = 0; i < c_views.size(); i++)
						{
							if(!c_views[i]->valid && c_views[i]->frameID != frame_id)
							{
								valid_views = false;
								break;
							}
						}
						if(valid_views)
							LOG(INFO) << "Frame: " << c_views[0]->frameID;
						else
							LOG(FATAL) << "Invalid Frame ID: " << c_views[0]->frameID;

						m_frame_counter++;
						
						pClient->set_curr_frameId(frame_id);
						fps_achieved = fps_counter.fps_counter();	// New FPS counter
						file_fps << frame_id << "," << fps_achieved << endl;

						// Skip the first 500 frames for loss calculation
						if(!lost_frame_reset && (frame_id - m_start_frame_id + 1) > SKIP_FIRST_FRAMES)
						{
							lost_frame_reset = true;
							m_frame_counter = 1;
							lost_frames = 0;
						}

						if(m_frame_counter % 10 == 0)
						{
							LOG(INFO) << "Client FPS: " << fps_achieved;
							int total_frames = frame_id - (m_start_frame_id + SKIP_FIRST_FRAMES) + 1;
							LOG(INFO) << "#Actual Frames: " << total_frames << ", #Received Frames: " << m_frame_counter << ", #Lost Frames: " << lost_frames << ", Loss Rate: " << (float)lost_frames / total_frames;
						}

						// Set the view for the client, if the frame is valid
						if(!pClient->set_view(c_views))
						{
							running = false;
							LOG(FATAL) << "Failed to set view for frame ID " << frame_id;
						}

						// if(FLAGS_use_client_bitrate)
						// {
						// 	int bwe, cbwe, dwe;
						// 	bs->split_bitrate_simple(bwe, cbwe, dwe);
						// 	if(bs->set_distorted_view(c_views, frame_id))
						// 		LOG(INFO) << "Distorted frame set for frame ID " << frame_id;
						// 	file_bwe_adapt << frame_id << "," << cbwe << "," << dwe << endl;
						// }
						
						if(bs->set_distorted_view(c_views, frame_id))
							LOG(INFO) << "Distorted frame set for frame ID " << frame_id;

						// Debug
						// if(frame_id % 10 == 0)
						// {
						// 	std::string save_dir = "/home/lei/data/pipeline/client_tiled/pipeline_new/test/";
						// 	if(!fs::exists(save_dir))
						// 		fs::create_directories(save_dir);
						// 	for(int i = 0; i < c_views.size(); i++)
						// 	{
						// 		save_color(reinterpret_cast<uint8_t *>(c_views[i]->color_image), c_views[i]->colorHeight, c_views[i]->colorWidth, FORMAT(save_dir << frame_id << "_color_bgra_lf_" << i << ".png"));
						// 		cv::Mat depth_image_cv2(c_views[i]->depthHeight, c_views[i]->depthWidth, CV_16UC1, reinterpret_cast<uint8_t *>(c_views[i]->depth_image));
						// 		std::string path = FORMAT(save_dir << frame_id << "_depth_yuv16_lf_" << i << ".png");
						// 		if(!cv::imwrite(path, depth_image_cv2))
						// 			LOG(ERROR) << "Failed to write depth image: " << path;
						// 	}
						// }

						file_recv_frame_proc << frame_id << "," << sw_local.ElapsedMs() << endl;
						while(running && !m_qUpdatePtcl.push(pClient))
							sleep_ms(sleep_time);

						file_recv_frame << frame_id << "," << sw_load_frame.ElapsedMs() << endl;

						// exit if all the frames are processed
						if(frame_id >= m_end_frame_id || !running)
						{
							file_recv_frame.close();
							file_recv_frame_proc.close();
							running = false;
							LOG(INFO) << "Exiting LOAD_FRAME stage";
							sleep_ms(sleep_time);
							break;
						}
						frame_id++;
					}
				}

				// Client update point cloud
				case PIPELINE_STAGES::UPDATE_PTCL:
				{
					MultiviewClient *pClient = NULL;

					fs::path filepath_update_ptcl{FORMAT(path << "0_update_ptcl_receiver_latency.txt")};
					fs::path filepath_update_ptcl_proc{FORMAT(path << "0_update_ptcl_receiver_processing_time.txt")};

					fs::ofstream file_update_ptcl, file_update_ptcl_proc;
					file_update_ptcl.open(filepath_update_ptcl, ios::out | ios::trunc);
					file_update_ptcl_proc.open(filepath_update_ptcl_proc, ios::out | ios::trunc);

					file_update_ptcl << "FrameID,Time" << endl;
					file_update_ptcl_proc << "FrameID,Time" << endl;

					StopWatch sw_update_ptcl, sw_webrtc, sw_local;
					int cur_frame_id = m_start_frame_id;
					while(running)
					{
						sw_update_ptcl.Restart();
						pClient = NULL;
						while(running && !m_qUpdatePtcl.pop(pClient))
							sleep_ms(sleep_time);				// waiting to pop from the queue
						
						sw_local.Restart();
						cur_frame_id = pClient->get_curr_frameId();

						// TODO: Rajrup: Send origData to the server
						UserData origData = m_dataPlayback->getUserData(cur_frame_id);
						if(origData.frustum.frameID != cur_frame_id)
							LOG(FATAL) << "Frame ID mismatch, user trace has frame ID " << origData.frustum.frameID << ", but reuired frame id is " << cur_frame_id;
						pClient->set_frustum(origData.frustum);

						vector<View *> temp_views = pClient->get_view();

						// Debug
						// if(cur_frame_id % 10 == 0)
						// {
						// 	std::string save_dir = "/home/lei/data/pipeline/client_tiled/pipeline_new/test/";
						// 	if(!fs::exists(save_dir))
						// 		fs::create_directories(save_dir);
						// 	for(int i = 0; i < temp_views.size(); i++)
						// 	{
						// 		save_color(reinterpret_cast<uint8_t *>(temp_views[i]->color_image), temp_views[i]->colorHeight, temp_views[i]->colorWidth, FORMAT(save_dir << cur_frame_id << "_color_bgra_up_" << i << ".png"));
						// 		cv::Mat depth_image_cv2(temp_views[i]->depthHeight, temp_views[i]->depthWidth, CV_16UC1, reinterpret_cast<uint8_t *>(temp_views[i]->depth_image));
						// 		std::string path = FORMAT(save_dir << cur_frame_id << "_depth_yuv16_up_" << i << ".png");
						// 		if(!cv::imwrite(path, depth_image_cv2))
						// 			LOG(ERROR) << "Failed to write depth image: " << path;
						// 	}
						// }

						if(!pClient->update_ptcl())
						{
							running = false;
							LOG(FATAL) << "Failed to update ptcl for frame ID " << cur_frame_id;
						}
						num_frustum_points2.store(pClient->get_num_points_in_frustum());
						LOG(INFO) << "Frustum Points: " << num_frustum_points2.load();
						
						file_update_ptcl_proc << cur_frame_id << "," << sw_local.ElapsedMs() << endl;
						stageOperationTimes[PIPELINE_STAGES::UPDATE_PTCL].push_back(make_pair(cur_frame_id, sw_local.ElapsedMs()));

						// if(cur_frame_id <= (m_end_frame_id - min(nInputObjects, SQ_CAPACITY)))
						{
							while(running && !m_qGeneratePtcl.push(pClient))
								sleep_ms(sleep_time);
						}

						m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL].push_back(make_pair(cur_frame_id, sw_update_ptcl.ElapsedMs()));

						file_update_ptcl << cur_frame_id << "," << sw_update_ptcl.ElapsedMs() << endl;
						
						// exit if all the frames are processed
						if(cur_frame_id >= m_end_frame_id || !running)
						{
							file_update_ptcl.close();
							file_update_ptcl_proc.close();
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

					fs::path filepath_end{FORMAT(path << "0_end_frame_recveiver_timestamp.txt")};
					fs::path filepath_render_ptcl{FORMAT(path << "0_render_ptcl_receiver_latency.txt")};
					fs::path filepath_render_ptcl_proc{FORMAT(path << "0_render_ptcl_receiver_processing_time.txt")};
					fs::ofstream file_end, file_render_ptcl_proc, file_render_ptcl;
					file_end.open(filepath_end, ios::out | ios::trunc);
					file_render_ptcl.open(filepath_render_ptcl, ios::out | ios::trunc);
					file_render_ptcl_proc.open(filepath_render_ptcl_proc, ios::out | ios::trunc);

					file_end << "FrameID,Time" << endl;
					file_render_ptcl << "FrameID,Time" << endl;
					file_render_ptcl_proc << "FrameID,Time" << endl;

					StopWatch sw_render_ptcl, sw_end, sw_local, sw_webrtc;

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
						LOG(INFO) << "Client - Frame ID: " << cur_frame_id << ", RECV Time: " << sw_end.Curr() << " ms";

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

						// if(cur_frame_id < m_end_frame_id)
						{
							// LOG(INFO) << "Thread " << i << " push to InputClient queue for frame id: " << cur_frame_id << " atomic id: " << m_capture_id;

							// Insert to the input queue of the client
							while(running && !m_loadFrame.push(pClient))
								sleep_ms(sleep_time);
						}

						m_vStageTimes[PIPELINE_STAGES::GENERATE_PTCL].push_back(make_pair(cur_frame_id, sw.ElapsedMs()));

						// exit if all the frames are processed
						if(cur_frame_id >= m_end_frame_id || !running)
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
						visualizer.GetRenderOption().SetPointSize(3.0);                 // Default is 5.0
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

							while(bool x = new_frustum2.load(memory_order_relaxed) == true && running)
            					sleep_ms(sleep_time);

							// Send the frustum to the server
							frustum_send2 = origData.frustum;
							// Change furstum position to Unity coordinates - Negative y-axis
							frustum_send2.pos(1) *= -1.0f;

							new_frustum2.store(true);

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
								LOG(INFO) << "Rendered frames: " << rendered_frames << ", Lost frames: " << (1.0 - rendered_frames);

							if(FLAGS_render_image_path != "" && cur_frame_id % 100 == 0)
							{
								std::string output_file = FORMAT(FLAGS_render_image_path << cur_frame_id << ".png");
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
						if(cur_frame_id >= m_end_frame_id || !running)
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

							if(FLAGS_save_ptcl_path != "" && cur_frame_id % 10 == 0)
							{
								std::string output_file = FORMAT(FLAGS_save_ptcl_path << cur_frame_id << ".ply");

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
							if(cur_frame_id >= m_end_frame_id || !running)
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

	// Thread for bitrate adaptation
	// if(FLAGS_use_client_bitrate)
	{
		threads.push_back(thread([this, sleep_time, &path, &out_bitrate_dir]
		{
			StopWatch sw;
			std::string gt_path = "";
			if(FLAGS_server_cull == CULLING::NO_CULLING)
				gt_path = FORMAT(PANOPTIC_DATA_PATH << FLAGS_seq_name << "/");
			else if(FLAGS_server_cull == CULLING::NORMAL_CULLING)
				gt_path = FORMAT(PANOPTIC_DATA_PATH << FLAGS_seq_name << "/culled_views_logID_" << LOG_ID << "/");
			else if(FLAGS_server_cull == CULLING::NORMAL_EXPANSION_CULLING)
				gt_path = FORMAT(PANOPTIC_DATA_PATH << FLAGS_seq_name << "/kpculled_views_logID_" << LOG_ID << "/");
			else
				LOG(FATAL) << "Invalid culling mode!";
			
			// fs::path filepath_rmse{FORMAT(path << "rmse_bwe" << bwe2.load() << "_cbwe" << cbwe2.load() << "_dbwe" << dbwe2.load() << ".txt")};
			// fs::ofstream file_rmse;
			// file_rmse.open(filepath_rmse, ios::out | ios::trunc);
			// file_rmse << "FrameID,cRMSE,dRMSE,cPSNR,dPSNR,cSSIM,dSSIM,Time" << endl;
			
			// TODO, CHANGE, MODIFY: Output directory for static bitrate vs mahimahi trace
			// fs::path filepath_split{FORMAT(out_bitrate_dir << "rmse_split_" << FLAGS_cb + FLAGS_db << "_adapt.txt")};
			fs::path filepath_split{FORMAT(out_bitrate_dir << "rmse_split_" << FLAGS_mm_trace_name << "_adapt.txt")};
			fs::ofstream file_split;
			file_split.open(filepath_split, ios::out | ios::trunc);
			file_split << "FrameID,cRMSE,dRMSE,d2c_split,dRMSE-cRMSE,step_size,Time" << endl;

			StopWatch sw_prev_split_update;
			float prev_d2c_split_update = 0.0;
			while(running)
			{
				// if(FLAGS_use_mm)
				// {
				// 	bwe2.store(down->get_throughput());
				// 	LOG(INFO) << "Frame: " << frame_id << ", BWE: " << bwe2;
				// }

				if(bs->has_distorted_view() && FLAGS_use_split_adapt)
				// if(bs->has_distorted_view())
				{
					sw.Restart();
					// Distorted frame is available, load gt frame
					bs->set_gt_view_from_disk(gt_path);

					// float crmse = bs->calc_color_rmse();
					// float drmse = bs->calc_depth_rmse();
					// float cpsnr = bs->calc_color_psnr();
					// float dpsnr = bs->calc_depth_psnr();
					// float cssim = bs->calc_color_ssim();
					// float dssim = bs->calc_depth_ssim();
				
					// int frame_id = bs->get_frame_id();

					// LOG(INFO) << "Frame: " << frame_id << ", cRMSE: " << crmse << ", cRMSE: " << drmse << ", cPSNR: " << cpsnr << ", dPSNR: " << dpsnr << ", cSSIM: " << cssim << ", dSSIM: " << dssim << ", Time: " << sw.ElapsedMs() << " ms";
					// file_rmse << frame_id << "," << crmse << "," << drmse << "," << cpsnr << "," << dpsnr << "," << cssim << "," << dssim << "," << sw.ElapsedMs() << endl;
					float crmse=0, drmse=0, d2c_split=0;
					int frame_id = bs->get_frame_id();
					
					// Skip the first 200 frames for adaptation
					if(frame_id < 100)
					{
						bs->allow_distorted_view();
						continue;
					} 

					if(FLAGS_use_split_adapt)
					{
						// Perform bitrate adaptation
						FinderStats stat;
						d2c_split = bs->split_bitrate_adapt(stat);

						// Initial update
						if(prev_d2c_split_update == 0)
						{
							d2c_split2.store(d2c_split);
							prev_d2c_split_update = d2c_split;
							sw_prev_split_update.Restart();
						}

						// Update split if split changed and time elapsed is greater than 1 second
						if(d2c_split != prev_d2c_split_update && sw_prev_split_update.ElapsedMs() > 1000)
						{
							d2c_split2.store(d2c_split);
							prev_d2c_split_update = d2c_split;
							sw_prev_split_update.Restart();
						}
						
						bs->allow_distorted_view();
						LOG(INFO) << "Frame: " << frame_id << ", cRMSE: " << stat.f1_x << ", dRMSE: " << stat.f2_x << ", d2c_split: " << d2c_split << ", Time: " << sw.ElapsedMs() << " ms";
						file_split << frame_id << "," << stat.f1_x << "," << stat.f2_x << "," << d2c_split << "," << stat.delta_f << "," << stat.step_size << "," << sw.ElapsedMs() << endl;
					}
					else
					{
						crmse = bs->calc_color_rmse();
						drmse = bs->calc_depth_rmse();
						d2c_split = bs->get_depth2color_split();
						d2c_split2.store(d2c_split);
						bs->allow_distorted_view();
						LOG(INFO) << "Frame: " << frame_id << ", cRMSE: " << crmse << ", dRMSE: " << drmse << ", d2c_split: " << d2c_split << ", Time: " << sw.ElapsedMs() << " ms";
						file_split << frame_id << "," << crmse << "," << drmse << "," << d2c_split << "," << 0.0 << "," << drmse - crmse << "," << sw.ElapsedMs() << endl;
					}

					if(frame_id >= m_end_frame_id || !running)
					{
						LOG(INFO) << "Exiting BITRATE_ADAPT stage";
						sleep_ms(sleep_time);
						break;
					}
				}
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