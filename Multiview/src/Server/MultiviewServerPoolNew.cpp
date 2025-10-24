#include "Server/MultiviewServerPoolNew.h"
#include <thread>
#include <csignal>
#include <atomic>
#include "gflags/gflags.h"
#include "timer.h"
#include "consts.h"
#include "mahimahi.h"

#define LOOKAHEAD_EXP
#define LOOKAHEAD_EXP_FRAME_NUM 10
#define DEPTH_BITRATE_PER_POINT 0.14213F 	// 142.13 bps per point
#define FRUSTUM_BUFFER -0.2F				// 20 cm buffer around the frustum
// #define DEPTH_BITRATE_PER_POINT 1.0F 	// 142.13 bps per point

namespace fs = boost::filesystem;
using namespace std;
// #define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))

std::atomic_bool running(true);

void sigint_handler(int signum)
{
	LOG(INFO) << "Received Ctrl+C signal. Stopping program...";
	running = false; // set the global flag to true to signal all threads to stop
}

MultiviewServerPoolNew::MultiviewServerPoolNew(const std::string &path, const int datasetType, const int start_frame_id, const int end_frame_id, uint32_t nThreads) : 
										m_nThreads(nThreads), m_frame_counter(0), fps_limiter2(FLAGS_server_fps)
{	
	m_start_frame_id = start_frame_id;
	m_end_frame_id = end_frame_id;
	m_capture_id = start_frame_id;
	
	if(datasetType == DATASET::PANOPTIC)
	{
		// m_kalmanPredictor = make_unique<KalmanPredictor>();
		m_kalmanPredictor2 = make_unique<KalmanPredictor2>(14, 7, 1.0e-2f, 1.0f, 33.0f/1000.0f);
		std::string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);
		m_dataPlayback = make_unique<DataPlayback>(user_data_path, SEQ_NAME, LOG_ID);
    	m_dataPlayback->loadData(START_FRAME, END_FRAME);
	}
	else
	{
		// m_kalmanPredictor = nullptr;
		m_kalmanPredictor2 = nullptr;
		m_dataPlayback = nullptr;
	}	
	
	m_vStageTimes.resize(PIPELINE_STAGES::NUM_STAGES);

	for (uint32_t i = 0; i < IQ_CAPACITY; i++)
	{
		m_pServers.push_back(new MultiviewServer(path, datasetType));
		if(!m_pServers[i]->init_playback())
			LOG(FATAL) << "Failed to initialize playback!";

        if(m_pServers[i]->get_calibration_requirement())
        {
            if(!m_pServers[i]->load_calibration())
                LOG(FATAL) << "Failed to load calibration!";
        }
	}

	for (uint32_t i = 0; i < IQ_CAPACITY; i++)
	{
		vector<uint32_t> s_deviceIndices = m_pServers[i]->get_device_indices();
        vector<std::string> s_deviceSerialNumbers = m_pServers[i]->get_device_serial_numbers();
        int s_numDevices = m_pServers[i]->get_device_count();
        if(s_numDevices <= 0)
			LOG(FATAL) << "No devices found during playback!";
            
        if(s_numDevices != s_deviceIndices.size() || s_numDevices != s_deviceSerialNumbers.size())
            LOG(FATAL) << "Device count mismatch during playback!";
        
        // Don't free the calibration data, it is owned by the server
        vector<Calibration *> s_calibrations = m_pServers[i]->get_calibration();

		if(!m_pServers[i]->get_calibration_status())
            LOG(FATAL) << "Failed to load Calibration!";
		
		// Copy calibration data and send to client
        if (i == 0) 
		{
            calibrations_init2 = s_calibrations;
            deviceIndices_init2 = s_deviceIndices;
            deviceSerialNumbers_init2 = s_deviceSerialNumbers;
        }

		if(datasetType == DATASET::KINECT)
		{
			vector<xy_table_t *> s_xy_tables = m_pServers[i]->get_xy_table();
            xy_tables_init2 = s_xy_tables;
		}
	}

	assert(m_pServers.size() == IQ_CAPACITY);

	up = new Mahimahi();
	down = new Mahimahi();

	if(FLAGS_use_mm)
		load_trace(up, down);
}

MultiviewServerPoolNew::~MultiviewServerPoolNew()
{
	// m_qInputServer.reset();
	m_qLoadFrame.reset();
	m_qUpdateView.reset();
	m_qServer2Client.reset();

	for (uint32_t i = 0; i < IQ_CAPACITY; i++)
	{
		delete m_pServers[i];
	}
	m_pServers.clear();
}

void MultiviewServerPoolNew::simluate_lookahead(MultiviewServer *pServer, int lookAhead, Frustum &s_frustum, Frustum &s_pred_frustum)
{
	uint32_t s_currFrameId = pServer->get_curr_frameId();
	// Predict frustum from trace
	if(m_kalmanPredictor2 != nullptr)
	{
		if(s_currFrameId < lookAhead || s_currFrameId - lookAhead < m_start_frame_id)
		{
			UserData origData = m_dataPlayback->getUserData(s_currFrameId);
			s_frustum = origData.frustum;
			s_pred_frustum = origData.frustum;
		}
		else
		{
			UserData origData = m_dataPlayback->getUserData(s_currFrameId - lookAhead);
			s_frustum = origData.frustum;
			if(!m_kalmanPredictor2->predictDataUsingQuat(s_currFrameId, lookAhead, s_frustum, s_pred_frustum))
				LOG(FATAL) << "Failed to predict frustum for Frame ID: " << s_currFrameId;

			LOG(INFO) << "Curr FID: " << s_currFrameId << ", Lookahead: " << s_currFrameId - s_frustum.frameID;
			LOG(INFO) << "Pred FID: " << s_pred_frustum.frameID << ", Recv FID: " << s_frustum.frameID;
		}
	}
}

void bitrate_splitter(uint32_t frustum_points)
{
	int target_bwe = bwe2.load();
	int cbwe_now = 0;
	int dbwe_now = int(DEPTH_BITRATE_PER_POINT * (float)frustum_points);   // In kbps
	LOG(INFO) << "Frustum Points: " << frustum_points << ", Depth BWE: " << dbwe_now;
	int total_bwe = int(dbwe_now * 8.0F/7.0F);
	
	if(total_bwe < target_bwe)
	{
		cbwe_now = target_bwe - total_bwe;

		if(dbwe_now < cbwe_now)
		{
			dbwe_now = int(6.0f/7.0f * (float)target_bwe);
			cbwe_now = target_bwe - dbwe_now;
		}

		cbwe2.store(cbwe_now);
		dbwe2.store(dbwe_now);
	}
	else
	{
		dbwe_now = int(6.0f/7.0f * (float)target_bwe);
		cbwe_now = target_bwe - dbwe_now;
		cbwe2.store(cbwe_now);
		dbwe2.store(dbwe_now);
	}
}

void split_bitrate_simple(int &out_bwe, int &out_cbwe, int &out_dbwe, float &d2c_split)
{
    int target_bwe = bwe2.load();

	// TODO: Rajrup: Remove later
	// Reduce target_bwe by 10% to avoid overloading the network
	if(FLAGS_server_cull == CULLING::NORMAL_EXPANSION_CULLING) // kp culling
		target_bwe = int(0.90f * (float)target_bwe);

	d2c_split = d2c_split2.load();
    int dbwe_now = int(d2c_split * (float)target_bwe);
    int cbwe_now = target_bwe - dbwe_now;
    cbwe2.store(cbwe_now);
    dbwe2.store(dbwe_now);
    out_bwe = target_bwe;
    out_cbwe = cbwe_now;
    out_dbwe = dbwe_now;
}

void MultiviewServerPoolNew::run() 
{
	std::signal(SIGINT, sigint_handler);

	// Send signal to websocket server to send calibration info to client
	wsserver_ready_to_init2.store(true);
	
	// Initialize WebRTC Server
	unique_ptr<NetworkSenderPool> wserver = make_unique<NetworkSenderPool>(FLAGS_save_frame, "/home/lei/data/pipeline/server_tiled/pipeline_new/test/");

	LOG(INFO) << "Server initialization done";
	
	vector<thread> threads;
	for (uint32_t i = 0; i < IQ_CAPACITY; i++)
		m_qLoadFrame.push(m_pServers[i]);
	
	const int sleep_time = 2;
	// std::string method = "livo_nocull";
	std::string method = "livo";		// TODO: Rajrup: Chane this to livo. Also Change path in SenderFrameBuffer.cpp in SenderFrameBuffer() constructor

	std::string path = FORMAT("/datassd/pipeline_cpp/server_tiled/e2e_latency/" << method << "/"); 
	create_folder(path);

	std::string ablation_dir = FORMAT("/datassd/pipeline_cpp/server_tiled/ablation/" << method << "/");
	create_folder(ablation_dir);
	
	std::string out_bitrate_dir = FORMAT(FLAGS_output_dir << "/bitrate_split_s_" << FLAGS_server_cull << "/log" << LOG_ID << "/");
	create_folder(out_bitrate_dir);

	for (uint32_t i = 0; i < m_nThreads; i++)
	{
		threads.push_back(thread([this, i, sleep_time, &wserver, &path, &ablation_dir, &out_bitrate_dir]
		{
			switch(i)
			{
				// Server Load Frame
				case PIPELINE_STAGES::LOAD_FRAME:
				{
					MultiviewServer *pServer = NULL;
					float fps_limited = 0.0;

					fs::path filepath_start{FORMAT(path << "0_start_frame_sender_timestamp.txt")};
					fs::path filepath_load{FORMAT(path << "0_load_frame_sender_latency.txt")};
					fs::path filepath_load_proc{FORMAT(path << "0_load_frame_sender_processing_time.txt")};

					// TODO, CHANGE, MODIFY: Output directory for static bitrate vs mahimahi trace
					// fs::path filepath_bwe_adapt{FORMAT(out_bitrate_dir << "bwe_" << FLAGS_cb + FLAGS_db << "_adapt.txt")};
					fs::path filepath_bwe_adapt{FORMAT(out_bitrate_dir << "bwe_" << FLAGS_mm_trace_name << "_adapt.txt")};

					fs::ofstream file_load, file_load_proc, file_start, file_end, file_bwe_adapt;
					file_start.open(filepath_start, ios::out | ios::trunc);
					file_load.open(filepath_load, ios::out | ios::trunc);
					file_load_proc.open(filepath_load_proc, ios::out | ios::trunc);
					file_bwe_adapt.open(filepath_bwe_adapt, ios::out | ios::trunc);

					file_start << "FrameID,Time" << endl;
					file_load << "FrameID,Time" << endl;
					file_load_proc << "FrameID,Time" << endl;
					file_bwe_adapt << "FrameID,cBWE,dBWE,d2c_split" << endl;

					StopWatch sw_load, sw_local, sw_start, sw_webrtc;
					while(running)
					{
						pServer = NULL;
						sw_load.Restart();
						while(running && !m_qLoadFrame.pop(pServer))	
							sleep_ms(sleep_time);			// waiting to pop from the queue

						sw_local.Restart();
						pServer->restart_playback();
						if(m_capture_id == m_start_frame_id)
							sw.Restart();							// Start the timer for the first frame
						pServer->set_curr_frameId(m_capture_id);
						int s_currFrameId = pServer->get_curr_frameId();
						file_start << s_currFrameId << "," << sw_start.Curr() << endl;
						LOG(INFO) << "Server - Frame ID: " << s_currFrameId << ", SEND Time: " << sw_webrtc.Curr() << " ms";

						if(FLAGS_use_server_bitrate_est)
						{
							bwe2.store(up->get_throughput());
							LOG(INFO) << "Frame: " << s_currFrameId << ", MahiMahi BWE: " << bwe2;
						}

						// Bitrat split is delayed. So perform the split before the frame is loaded.
						if(FLAGS_use_client_bitrate_split)
						{
							int bwe, cbwe, dwe;
							float d2c_split;
							split_bitrate_simple(bwe, cbwe, dwe, d2c_split);
							file_bwe_adapt << s_currFrameId << "," << cbwe << "," << dwe << "," << d2c_split << endl;
							LOG(INFO) << "Frame: " << s_currFrameId << ", BWE: " << bwe << ", cBWE: " << cbwe << ", dBWE: " << dwe << ", Split: " << d2c_split;
						}

						// fps.counter(m_capture_id);
						m_frame_counter++;
						// fps.rate_limit(m_frame_counter, FLAGS_server_fps); // Old rate limiter
						fps_limited = fps_limiter2.rate_limit();		// New rate limiter
						if(m_frame_counter % 10 == 0)
							LOG(INFO) << "Server FPS: " << fps_limited;
						
						m_capture_id++;

						if(!pServer->load_frame_parallel())
						{
							running = false;
							LOG(FATAL) << "Failed to load frame " << pServer->get_curr_frameId();
						}

						m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].push_back(make_pair(pServer->get_curr_frameId(), pServer->get_load_frame_time()));
						file_load_proc << s_currFrameId << "," << sw_local.ElapsedMs() << endl;

						while(running && !m_qUpdateView.push(pServer))
							sleep_ms(sleep_time);

						file_load << s_currFrameId << "," << sw_load.ElapsedMs() << endl;

						// exit if all the frames are processed
						if(pServer->get_curr_frameId() == m_end_frame_id)
						{
							LOG(INFO) << "Exiting LOAD_FRAME stage";
							sleep_ms(sleep_time);
							break;
						}
					}
					break;
				}

				// Server Update View
				case PIPELINE_STAGES::UPDATE_VIEW:
				{
					MultiviewServer *pServer  = NULL;
					
					fs::path filepath_uv{FORMAT(path << "0_update_view_sender_latency.txt")};
					fs::path filepath_uv_proc{FORMAT(path << "0_update_view_sender_processing_time.txt")};
					// fs::path filepath_frustum_size{FORMAT(path << "frustum_size_kpcull"<< LOOKAHEAD_EXP_FRAME_NUM <<".txt")};
					fs::path filepath_frustum_size{FORMAT(ablation_dir << "frustum_size_pcull_" << SEQ_NAME << ".txt")};
					
					std::string trace_type = "tracep1-scaled10.0_user_test";
					// std::string trace_type = "wifi-25-scaled15.0";
					std::string bw_path = FORMAT("/datassd/pipeline/mm_bitrate_estimate/" << trace_type << "/" << FLAGS_seq_name << "/" << USER_TRACE_FOLDER << "/");
					fs::path filepath_bw{FORMAT(bw_path << "bw_server_estimate_log" << LOG_ID << ".csv")};

					fs::ofstream file_uv, file_uv_proc, file_bw, file_frustum_size;
					file_uv.open(filepath_uv, ios::out | ios::trunc);
					file_uv_proc.open(filepath_uv_proc, ios::out | ios::trunc);
					file_bw.open(filepath_bw, ios::out | ios::trunc);
					file_frustum_size.open(filepath_frustum_size, ios::out | ios::trunc);

					file_uv << "FrameID,Time" << endl;
					file_uv_proc << "FrameID,Time" << endl;
					file_frustum_size << "FrameID,FrustumSize,WithoutFrustumSize,Ratio" << endl;
					file_bw << "FrameID,BWE(in kbps),CBWE(in kbps),DBWE(in kbps)" << endl;

					StopWatch sw_uv, sw_local, sw_webrtc;

					uint32_t frustum_points = 0;
					uint32_t total_points = 0;
					float frustum_ratio = 0.0f;

					Frustum s_frustum;
					Frustum s_pred_frustum;
					StopWatch sw_kalman;
					while(running)
					{
						pServer  = NULL;
						sw_uv.Restart();
						while(running && !m_qUpdateView.pop(pServer))
							sleep_ms(sleep_time);
						
						sw_local.Restart();
						int s_currFrameId = pServer->get_curr_frameId();
						// LOG(INFO) << "Server - Frame ID: " << s_currFrameId << ", SEND Time: " << sw_webrtc.Curr() << " ms";

						UserData origData = m_dataPlayback->getUserData(s_currFrameId);
						
						if(FLAGS_server_cull == CULLING::NORMAL_CULLING)
							pServer->set_frustum(origData.frustum);

#ifdef LOOKAHEAD_EXP
						/*************************** KALMAN FILTER lookahead experiment **********************/
						if(bool x = new_frustum2.load(memory_order_relaxed) == true)
						{
							s_frustum = frustum_recv2;
							new_frustum2.store(false);
						}
						if(FLAGS_server_cull == CULLING::NORMAL_EXPANSION_CULLING)
						{
							simluate_lookahead(pServer, LOOKAHEAD_EXP_FRAME_NUM, s_frustum, s_pred_frustum);
							s_pred_frustum.add_buffer(FRUSTUM_BUFFER);
							pServer->set_frustum(s_frustum, s_pred_frustum);
						}
#else
						/**************************** KALMAN FILTER for pipeline *****************************/
						s_frustum = origData.frustum;
						s_pred_frustum = origData.frustum;
						pServer->set_frustum(s_frustum, s_pred_frustum);
						if(bool x = new_frustum.load(memory_order_relaxed) == true)
						{
							s_frustum = frustum_recv;
							// cout << "++++++++++++++++++++++++++++++++++++++++++" << endl;
							// LOG(INFO) << "Frame ID: " << s_currFrameId << ", Trace Frustum ID: " << origData.frustum.frameID << ", Received Frustum ID: " << s_frustum.frameID;
							LOG(INFO) << "Curr FID: " << s_currFrameId << ", Lookahead: " << s_currFrameId - s_frustum.frameID;
							// s_frustum.print();
							new_frustum.store(false);

							// Predict frustum from received user frustrum from client
							if(FLAGS_server_cull == CULLING::NORMAL_EXPANSION_CULLING)
							{
								if(m_kalmanPredictor != nullptr)
								{
									uint32_t lookAhead = s_currFrameId - s_frustum.frameID;
									sw_kalman.Restart();
									if(!m_kalmanPredictor2->predictDataUsingQuat(s_currFrameId, lookAhead, s_frustum, s_pred_frustum))
										LOG(FATAL) << "Failed to predict frustum for Frame ID: " << s_currFrameId;

									LOG(INFO) << "Pred FID: " << s_pred_frustum.frameID << ", Recv FID: " << s_frustum.frameID;
									// LOG(INFO) << "Kalman Prediction Time: " << sw_kalman.ElapsedMs() << " ms";
									// s_pred_frustum.print();
									pServer->set_frustum(s_frustum, s_pred_frustum);
								}
								else
									LOG(FATAL) << "Kalman predictor is not initialized!";
							}
						}
#endif
						pServer->update_view();
						frustum_points = pServer->get_view_size_in_frustum() / (sizeof(uint16_t) + sizeof(RGB));
						total_points = pServer->get_view_size_without_frustum() / (sizeof(uint16_t) + sizeof(RGB));
						frustum_ratio = (float)frustum_points / (float)total_points;
						file_frustum_size << s_currFrameId << "," << frustum_points << "," << total_points << "," << frustum_ratio << endl;

						// Split the bitrate between color and depth. For client bitrate, the client will send the splitted bitrate
						// if(FLAGS_use_server_bitrate_est)
						// {
						// 	bwe2.store(up->get_throughput());
						// 	LOG(INFO) << "Frame: " << s_currFrameId << ", BWE: " << bwe2;
						// 	bitrate_splitter(frustum_points);
						// 	file_bw << s_currFrameId << "," << bwe2.load() << "," << cbwe2.load() << "," << dbwe2.load() << endl;
						// }
						// else if(FLAGS_use_client_bitrate)
						// {
						// 	bitrate_splitter(frustum_points);
						// }

						// s_views = pServer->get_view();

						m_vStageTimes[PIPELINE_STAGES::UPDATE_VIEW].push_back(make_pair(pServer->get_curr_frameId(), pServer->get_update_view_time()));

						file_uv_proc << s_currFrameId << "," << sw_local.ElapsedMs() << endl;

						while(running && !m_qServer2Client.push(pServer))
							sleep_ms(sleep_time);

						file_uv << s_currFrameId << "," << sw_uv.ElapsedMs() << endl;
						
						// exit if all the frames are processed
						if(pServer->get_curr_frameId() == m_end_frame_id)
						{
							file_frustum_size.close();
							LOG(INFO) << "Exiting UPDATE_VIEW stage";
							sleep_ms(sleep_time);
							break;
						}
					}
					break;
				}

				// Server Send Frame
				case PIPELINE_STAGES::SEND_FRAME:
				{
					MultiviewServer *pServer = NULL;

					LOG(INFO) << "Server - SEND_FRAME: Thread ID: " << i;
					fs::path filepath_send{FORMAT(path << "0_send_frame_sender_latency.txt")};
					fs::path filepath_send_proc{FORMAT(path << "0_send_frame_sender_processing_time.txt")};

					fs::ofstream file_send, file_send_proc;
					file_send.open(filepath_send, ios::out | ios::trunc);
					file_send_proc.open(filepath_send_proc, ios::out | ios::trunc);

					file_send << "FrameID,Time" << endl;
					file_send_proc << "FrameID,Time" << endl;

					StopWatch sw_send, sw_local;

					while(running)
					{
						pServer = NULL;
						sw_send.Restart();
						while(!m_qServer2Client.pop(pServer))
							sleep_ms(sleep_time);

						sw_local.Restart();
						int s_currFrameId = pServer->get_curr_frameId();
						s_views = pServer->get_view();

						// Debug
						// if(s_currFrameId % 10 == 0)
						// {
						// 	std::string save_dir = "/home/lei/data/pipeline/server_tiled/pipeline_new/test/";
						// 	if(!fs::exists(save_dir))
						// 		fs::create_directories(save_dir);
						// 	for(int i = 0; i < s_views.size(); i++)
						// 	{
						// 		save_color(reinterpret_cast<uint8_t *>(s_views[i]->color_image), s_views[i]->colorHeight, s_views[i]->colorWidth, FORMAT(save_dir << s_currFrameId << "_color_bgra_sf_" << i << ".png"));
						// 		cv::Mat depth_image_cv2(s_views[i]->depthHeight, s_views[i]->depthWidth, CV_16UC1, reinterpret_cast<uint8_t *>(s_views[i]->depth_image));
						// 		std::string path = FORMAT(save_dir << s_currFrameId << "_depth_yuv16_sf_" << i << ".png");
						// 		if(!cv::imwrite(path, depth_image_cv2))
						// 			LOG(ERROR) << "Failed to write depth image: " << path;
						// 	}
						// }

						wserver->send(s_currFrameId, s_views);

						m_vStageTimes[PIPELINE_STAGES::SEND_FRAME].push_back(make_pair(pServer->get_curr_frameId(), sw_send.ElapsedMs()));
						m_vServerLatency.push_back(make_pair(pServer->get_curr_frameId(), pServer->get_elapsed_time()));
						file_send_proc << s_currFrameId << "," << sw_local.ElapsedMs() << endl;

						/**
						 * @brief Pushing the pServer to the input queue of the server for reuse
						 * Don't check any variable of pServer, since it will reset by the input thread.
						 * Exception: Only for last frame, we can check the frame id since it's not inserted to the queue
						 * If there are 20 frames and stage queue of size = 3, then quit when the frame id is 17
						 * We don't want to reutilise the server object for the last 3 frames, since input frames are finished
						 */
						if(pServer->get_curr_frameId() <= (m_end_frame_id - SQ_CAPACITY))
						{
							// Insert to the input queue of the server
							while(!m_qLoadFrame.push(pServer))
								sleep_ms(sleep_time);
						}

						file_send << s_currFrameId << "," << sw_send.ElapsedMs() << endl;

						// LOG(INFO) << "SEND_FRAME: Done for frame id: " << pServer->get_curr_frameId();
						// exit if all the frames are processed
						if(pServer->get_curr_frameId() == m_end_frame_id)
						{
							LOG(INFO) << "Exiting SEND_FRAME stage";
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

	// Examine latency
	// assert(m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].size() == m_vStageTimes[PIPELINE_STAGES::UPDATE_VIEW].size());
	// assert(m_vStageTimes[PIPELINE_STAGES::SEND_FRAME].size() == m_vStageTimes[PIPELINE_STAGES::UPDATE_VIEW].size());
	
	int num_frames = 5;

	// skip the first 5 frames during calculation
	for(uint32_t i = 5; i < m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].size(); i++)
	{
		// assert(m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME][i].first == m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL][i].first);
		// assert(m_vStageTimes[PIPELINE_STAGES::UPDATE_VIEW][i].first == m_vServerLatency[i].first);

		LOG(INFO) << "------------------------------------------------------------";
		LOG(INFO) << "Frame: " << m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME][i].first;
		LOG(INFO) << "Load frame: " << m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME][i].second << " ms";
		LOG(INFO) << "Update view: " << m_vStageTimes[PIPELINE_STAGES::UPDATE_VIEW][i].second << " ms";
		LOG(INFO) << "Send frame: " << m_vStageTimes[PIPELINE_STAGES::SEND_FRAME][i].second << " ms";
		LOG(INFO) << "Server latency: " << m_vServerLatency[i].second << " ms";
	}
    
}