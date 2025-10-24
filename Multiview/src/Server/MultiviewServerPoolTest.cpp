#include "Server/MultiviewServerPoolTest.h"
#include <thread>
#include <csignal>
#include <algorithm>
#include "gflags/gflags.h"
#include "Client/PointCloudStreamer.h"
#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"

using namespace open3d;

#define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))

std::atomic_bool running(true);

void sigint_handler(int signum)
{
	LOG(INFO) << "Received Ctrl+C signal. Stopping program...";
	running = false; // set the global flag to true to signal all threads to stop
}

MultiviewServerPoolTest::MultiviewServerPoolTest(const string &path, const int datasetType, const int start_frame_id, const int end_frame_id, uint32_t nThreads) : m_nThreads(nThreads)
{	
	m_start_frame_id = start_frame_id;
	m_end_frame_id = end_frame_id;
	m_capture_id = start_frame_id;
	
	if(datasetType == DATASET::PANOPTIC)
	{
		string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);
		m_dataPlayback = make_unique<DataPlayback>(user_data_path, SEQ_NAME, LOG_ID);
    	m_dataPlayback->loadData(START_FRAME, END_FRAME);
		m_kalmanPredictor = make_unique<KalmanPredictor>();
	}
	else
	{
		m_dataPlayback = nullptr;
		m_kalmanPredictor = nullptr;
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
		m_pClients.push_back(new MultiviewClient(path, datasetType));

		vector<uint32_t> s_deviceIndices = m_pServers[i]->get_device_indices();
        vector<string> s_deviceSerialNumbers = m_pServers[i]->get_device_serial_numbers();
        int s_numDevices = m_pServers[i]->get_device_count();
        if(s_numDevices <= 0)
			LOG(FATAL) << "No devices found during playback!";
            
        if(s_numDevices != s_deviceIndices.size() || s_numDevices != s_deviceSerialNumbers.size())
            LOG(FATAL) << "Device count mismatch during playback!";
        
        // Don't free the calibration data, it is owned by the server
        vector<Calibration *> s_calibrations = m_pServers[i]->get_calibration();

        m_pClients[i]->init(s_deviceIndices, s_deviceSerialNumbers, s_calibrations);

		if(!m_pServers[i]->get_calibration_status())
            LOG(FATAL) << "Failed to load Calibration!";

        if(!m_pClients[i]->get_calibration_status())
            LOG(FATAL) << "Failed to load Calibration!";
		
		if(datasetType == DATASET::KINECT)
		{
			vector<xy_table_t *> s_xy_tables = m_pServers[i]->get_xy_table();
			m_pClients[i]->set_xy_table(s_xy_tables);
		}
	}

	assert(m_pServers.size() == IQ_CAPACITY);
	assert(m_pClients.size() == IQ_CAPACITY);
}

MultiviewServerPoolTest::~MultiviewServerPoolTest()
{
	m_qLoadFrame.reset();
	m_qUpdateView.reset();
	m_qServer2Client.reset();
	m_qInputClient.reset();

	for (uint32_t i = 0; i < IQ_CAPACITY; i++)
	{
		delete m_pServers[i];
	}
	m_pServers.clear();

	for (uint32_t i = 0; i < IQ_CAPACITY; i++)
	{
		delete m_pClients[i];
	}
	m_pClients.clear();
}

void MultiviewServerPoolTest::start_conditional()
{
	std::signal(SIGINT, sigint_handler);

    vector<thread> threads;
	for (uint32_t i = 0; i < IQ_CAPACITY; i++)
    {
		m_qLoadFrame.push(m_pServers[i]);
        m_qInputClient.push(m_pClients[i]);
    }
	
	// Generate Ptcl stage takes 2 mutexes
	vector<mutex> mutexes(PIPELINE_STAGES::NUM_STAGES + 1);
	vector<condition_variable> cvs(PIPELINE_STAGES::NUM_STAGES + 1);
	mutex data_read_m;

	StopWatch sw_pipeline;
	const int sleep_time = 5;
    for (uint32_t i = 0; i < m_nThreads; i++)
    {
        threads.push_back(thread([this, i, sleep_time, &mutexes, &cvs, &data_read_m, &sw_pipeline]
		{
			switch(i)
			{
				// Server Load Frame
				case PIPELINE_STAGES::LOAD_FRAME:
				{
					MultiviewServer *pServer = NULL;
					while(running)
					{
						pServer = NULL;
						unique_lock<mutex> lock0(mutexes[0]);
						while(running && !m_qLoadFrame.pop(pServer))
							cvs[0].wait(lock0);
						lock0.unlock();
						cvs[0].notify_one();
						
						pServer->restart_playback();
						if(m_capture_id == m_start_frame_id)
							sw_pipeline.Restart();		// Start the timer for the first frame
						pServer->set_curr_frameId(m_capture_id);
						m_capture_id++;

						if(!pServer->load_frame_parallel())
						{
							running = false;
							LOG(FATAL) << "Thread " << i << ": Failed to load frame " << pServer->get_curr_frameId();
						}

						m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].push_back(make_pair(pServer->get_curr_frameId(), pServer->get_load_frame_time()));
						
						// After an pSever is pushed to the queue, it will be used by the next thread
						unique_lock<mutex> lock1(mutexes[1]);
						while(running && !m_qUpdateView.push(pServer))
							cvs[1].wait(lock1);
						lock1.unlock();
						cvs[1].notify_one();

						// exit if all the frames are processed
						if(pServer->get_curr_frameId() == m_end_frame_id)
						{
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
					while (running)
					{
						pServer  = NULL;
						unique_lock<mutex> lock1(mutexes[1]);
						while(running && !m_qUpdateView.pop(pServer))
							cvs[1].wait(lock1);
						lock1.unlock();
						cvs[1].notify_one();

						// Predict frustum from trace
						if(m_kalmanPredictor != nullptr)
						{
            				// Data race with UPDATE_PTCL
							unique_lock<mutex> lock(data_read_m);
							m_kalmanPredictor->predictData(pServer->get_curr_frameId(), 1, m_origData, m_predData);
							pServer->set_frustum(m_origData.frustum, m_predData.frustum);
							lock.unlock();
						}

						pServer->update_view();

						m_vStageTimes[PIPELINE_STAGES::UPDATE_VIEW].push_back(make_pair(pServer->get_curr_frameId(), pServer->get_update_view_time()));

						unique_lock<mutex> lock2(mutexes[2]);
						while(running && !m_qServer2Client.push(pServer))
							cvs[2].wait(lock2);
						lock2.unlock();
						cvs[2].notify_one();

						// exit if all the frames are processed
						if(pServer->get_curr_frameId() == m_end_frame_id)
						{
							sleep_ms(sleep_time);
							break;
						}
					}
					break;
				}
				
				// Client Update Ptcl
				case PIPELINE_STAGES::UPDATE_PTCL:
				{
					MultiviewServer *pServer = NULL;
					MultiviewClient *pClient = NULL;
					while(running)
					{
						pServer = NULL;
						pClient = NULL;
						unique_lock<mutex> lock2(mutexes[2]);
						while(running && !m_qServer2Client.pop(pServer))
							cvs[2].wait(lock2);
						lock2.unlock();
						cvs[2].notify_one();

						unique_lock<mutex> lock3(mutexes[3]);
						while(running && !m_qInputClient.pop(pClient))
							cvs[3].wait(lock3);
						lock3.unlock();
						cvs[3].notify_one();
						
						m_vServerLatency.push_back(make_pair(pServer->get_curr_frameId(), pServer->get_elapsed_time()));
						
						pClient->restart_playback();
						int s_currFrameId = pServer->get_curr_frameId();
						pClient->set_curr_frameId(s_currFrameId);
						vector<View *> s_views = pServer->get_view();

						unique_lock<mutex> lock(data_read_m);
						pClient->set_frustum(m_origData.frustum);
						lock.unlock();

						if(!pClient->set_view(s_views))
						{
							running = false;
							LOG(FATAL) << "Thread " << i << ": Failed to set view " << pClient->get_curr_frameId();
						}
						
						/**
						 * @brief Pushing the pServer to the input queue of the server for reuse
						 * Don't check any variable of pServer, since it will reset by the input thread.
						 * Exception: Only for last frame, we can check the frame id since it's not inserted to the queue
						 * If there are 20 frames and stage queue of size = 3, then quit when the frame id is 17
						 * We don't want to reutilise the server object for the last 3 frames, since input frames are finished
						 */
						if(pServer->get_curr_frameId() <= (m_end_frame_id - SQ_CAPACITY))
						{
							// LOG(INFO) << "Thread " << i << " push to InputServer queue for server frame id: " << pServer->get_curr_frameId() << " client frame id: " << pClient->get_curr_frameId();

							// Insert to the input queue of the server
							unique_lock<mutex> lock0(mutexes[0]);
							while(running && !m_qLoadFrame.push(pServer))
								cvs[0].wait(lock0);
							lock0.unlock();
							cvs[0].notify_one();
						}

						if(!pClient->update_ptcl())
						{
							running = false;
							LOG(FATAL) << "Thread " << i << ": Failed to update ptcl " << pClient->get_curr_frameId();
						}
						
						// Get next user trace
						lock.lock();
            			m_origData = m_dataPlayback->getUserData(pClient->get_curr_frameId());
						lock.unlock();

						// For benchmarking
						m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL].push_back(make_pair(pClient->get_curr_frameId(), pClient->get_update_ptcl_time()));
						m_vClientLatency.push_back(make_pair(pClient->get_curr_frameId(), pClient->get_elapsed_time()));
						m_vFps.push_back(make_pair(pClient->get_curr_frameId(), sw_pipeline.ElapsedMs()));

						// exit if all the frames are processed
						if(pClient->get_curr_frameId() == m_end_frame_id)
						{
							LOG(INFO) << "Thread " << i << ": Done for frame id: " << pClient->get_curr_frameId();
							sleep_ms(sleep_time);
							break;
						}

						LOG(INFO) << "Thread " << i << ": Done with frame id: " << pClient->get_curr_frameId();
						if(pClient->get_curr_frameId() <= (m_end_frame_id - SQ_CAPACITY))
						{
							// LOG(INFO) << "Thread " << i << " push to InputClient queue for frame id: " << pClient->get_curr_frameId() << " atomic id: " << m_capture_id;

							// Insert to the input queue of the client
							unique_lock<mutex> lock3(mutexes[3]);
							while(running && !m_qInputClient.push(pClient))
								cvs[3].wait(lock3);
							lock3.unlock();
							cvs[3].notify_one();
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
	assert(m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].size() == m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL].size());
	
	vector<float> avg_stage_latency(PIPELINE_STAGES::NUM_STAGES, 0.0f);
	float avg_server_latency = 0.0f;
	float avg_client_latency = 0.0f;
	float avg_fps = 0.0f;
	float avg_total_latency = 0.0f;
	
	int skip_frames = 5;
	int num_frames = skip_frames;
	// skip the first 5 frames during calculation
	for(uint32_t i = skip_frames; i < m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].size(); i++)
	{
		assert(m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME][i].first == m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL][i].first);
		assert(m_vStageTimes[PIPELINE_STAGES::UPDATE_VIEW][i].first == m_vServerLatency[i].first);
		assert(m_vStageTimes[PIPELINE_STAGES::UPDATE_VIEW][i].first == m_vClientLatency[i].first);

		avg_stage_latency[PIPELINE_STAGES::LOAD_FRAME] += m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME][i].second;

		avg_stage_latency[PIPELINE_STAGES::UPDATE_VIEW] += m_vStageTimes[PIPELINE_STAGES::UPDATE_VIEW][i].second;

		avg_stage_latency[PIPELINE_STAGES::UPDATE_PTCL] += m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL][i].second;

		avg_server_latency += m_vServerLatency[i].second;
		avg_client_latency += m_vClientLatency[i].second;
		avg_total_latency += m_vServerLatency[i].second + m_vClientLatency[i].second;
		avg_fps = m_vFps[i].second - m_vFps[skip_frames].second;
	}

	num_frames = m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].size() - skip_frames;
	avg_stage_latency[PIPELINE_STAGES::LOAD_FRAME] /= num_frames;
	avg_stage_latency[PIPELINE_STAGES::UPDATE_VIEW] /= num_frames;
	avg_stage_latency[PIPELINE_STAGES::UPDATE_PTCL] /= num_frames;
	avg_server_latency /= num_frames;
	avg_client_latency /= num_frames;
	avg_total_latency /= num_frames;
	avg_fps = float(num_frames) * 1000.0f / avg_fps;

	LOG(INFO) << "------------------------------------------------------------";
	LOG(INFO) << "Frames: " << num_frames;
	LOG(INFO) << "Load frame: " << avg_stage_latency[PIPELINE_STAGES::LOAD_FRAME] << " ms";
	LOG(INFO) << "Update view: " << avg_stage_latency[PIPELINE_STAGES::UPDATE_VIEW] << " ms";
	LOG(INFO) << "Server latency: " << avg_server_latency << " ms";

	LOG(INFO) << "Update ptcl: " << avg_stage_latency[PIPELINE_STAGES::UPDATE_PTCL] << " ms";
	LOG(INFO) << "Client latency: " << avg_client_latency << " ms";
	LOG(INFO) << "End-to-end latency: " << avg_total_latency << " ms";
	LOG(INFO) << "FPS: " << avg_fps;
}

void MultiviewServerPoolTest::start_sleep()
{
	std::signal(SIGINT, sigint_handler);

    vector<thread> threads;
	int nInputObjects = IQ_CAPACITY - 1;
    for (uint32_t i = 0; i < nInputObjects; i++)
	{
		m_qLoadFrame.push(m_pServers[i]);
	}

	for (uint32_t i = 0; i < nInputObjects; i++)
	{
		m_qInputClient.push(m_pClients[i]);
	}   

	vector<vector<pair<int, uint64_t>>> stageOperationTimes(PIPELINE_STAGES::NUM_STAGES);
	mutex data_read_m;

	StopWatch sw_pipeline;
	const int sleep_time = 1;

    for (uint32_t i = 0; i < m_nThreads; i++)
    {
        threads.push_back(thread([this, i, sleep_time, &data_read_m, &sw_pipeline, &stageOperationTimes, &nInputObjects] 
        {
            switch (i)
            {
				// Server Load Frame
				case PIPELINE_STAGES::LOAD_FRAME:
				{
					MultiviewServer *pServer = NULL;
					StopWatch sw, sw_local;
					int frame_id = 0;
					while(running)
					{
						sw.Restart();
						pServer = NULL;
						while(running && !m_qLoadFrame.pop(pServer))	
							sleep_ms(sleep_time);			// waiting to pop from the queue
						
						sw_local.Restart();
						pServer->restart_playback();
						if(m_capture_id == m_start_frame_id)
							sw_pipeline.Restart();							// Start the timer for the first frame
						pServer->set_curr_frameId(m_capture_id);
						m_capture_id++;

						frame_id = pServer->get_curr_frameId();
						if(!pServer->load_frame_parallel())
						{
							running = false;
							LOG(FATAL) << "Thread " << i << ": Failed to load frame " << frame_id;
						}
						stageOperationTimes[PIPELINE_STAGES::LOAD_FRAME].push_back(make_pair(frame_id, sw_local.ElapsedMs()));
						while(running && !m_qUpdateView.push(pServer))
							sleep_ms(sleep_time);

						// exit if all the frames are processed
						if(pServer->get_curr_frameId() == m_end_frame_id)
						{
							sleep_ms(sleep_time);
							break;
						}
						m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].push_back(make_pair(frame_id, sw.ElapsedMs()));
					}
					break;
				}

				// Server Update View
            	case PIPELINE_STAGES::UPDATE_VIEW:
				{
					MultiviewServer *pServer  = NULL;
					StopWatch sw, sw_local;
					int frame_id = 0;
					while(running)
					{
						sw.Restart();
						pServer  = NULL;
						while(running && !m_qUpdateView.pop(pServer))
							sleep_ms(sleep_time);

						sw_local.Restart();
						frame_id = pServer->get_curr_frameId();
						// Predict frustum from trace
						if(m_kalmanPredictor != nullptr)
						{
            				// Data race with UPDATE_PTCL
							unique_lock<mutex> lock(data_read_m);
							m_kalmanPredictor->predictData(frame_id, 1, m_origData, m_predData);
							pServer->set_frustum(m_origData.frustum, m_predData.frustum);
							lock.unlock();
						}

						pServer->update_view();

						stageOperationTimes[PIPELINE_STAGES::UPDATE_VIEW].push_back(make_pair(frame_id, sw_local.ElapsedMs()));

						while(running && !m_qServer2Client.push(pServer))
							sleep_ms(sleep_time);

						// exit if all the frames are processed
						if(frame_id == m_end_frame_id)
						{
							sleep_ms(sleep_time);
							break;
						}
						m_vStageTimes[PIPELINE_STAGES::UPDATE_VIEW].push_back(make_pair(frame_id, sw.ElapsedMs()));
					}
					break;
				}

				// Client Update Ptcl
				case PIPELINE_STAGES::UPDATE_PTCL:
				{
					MultiviewServer *pServer = NULL;
					MultiviewClient *pClient = NULL;
					StopWatch sw, sw_local;
					int frame_id = 0;
					while(running)
					{
						sw.Restart();
						pServer = NULL;
						pClient = NULL;
						while(running && !m_qServer2Client.pop(pServer))
							sleep_ms(sleep_time);

						while(running && !m_qInputClient.pop(pClient))
							sleep_ms(sleep_time);
						
						sw_local.Restart();
						frame_id = pServer->get_curr_frameId();

						pClient->restart_playback();
						pClient->set_curr_frameId(frame_id);
						vector<View *> s_views = pServer->get_view();

						unique_lock<mutex> lock(data_read_m);
						pClient->set_frustum(m_origData.frustum);
						lock.unlock();

						if(!pClient->set_view(s_views))
						{
							running = false;
							LOG(FATAL) << "Thread " << i << ": Failed to set view " << pClient->get_curr_frameId();
						}

						/**
						 * @brief Pushing the pServer to the input queue of the server for reuse
						 * Don't check any variable of pServer, since it will reset by the input thread.
						 * Exception: Only for last frame, we can check the frame id since it's not inserted to the queue
						 * If there are 20 frames and stage queue of size = 3, then quit when the frame id is 17
						 * We don't want to reutilise the server object for the last 3 frames, since input frames are finished
						 */
						if(frame_id <= (m_end_frame_id - min(nInputObjects, SQ_CAPACITY)))
						{
							// Insert to the input queue of the server
							while(running && !m_qLoadFrame.push(pServer))
								sleep_ms(sleep_time);
						}
						
						if(!pClient->update_ptcl())
						{
							running = false;
							LOG(FATAL) << "Thread " << i << ": Failed to update ptcl " << frame_id;
						}

						stageOperationTimes[PIPELINE_STAGES::UPDATE_PTCL].push_back(make_pair(frame_id, sw_local.ElapsedMs()));

						while(running && !m_qGeneratePtcl.push(pClient))
							sleep_ms(sleep_time);

						// exit if all the frames are processed
						if(frame_id == m_end_frame_id)
						{
							sleep_ms(sleep_time);
							break;
						}
						m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL].push_back(make_pair(frame_id, sw.ElapsedMs()));
					}
					break;
				}

				case PIPELINE_STAGES::GENERATE_PTCL:
				{
					MultiviewClient *pClient = NULL;
					StopWatch sw, sw_local;
					int frame_id = 0;
					while(running)
					{
						sw.Restart();
						pClient = NULL;
						while(running && !m_qGeneratePtcl.pop(pClient))
							sleep_ms(sleep_time);

						sw_local.Restart();
						frame_id = pClient->get_curr_frameId();
						if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
							pClient->generate_pcl_object();
						
						stageOperationTimes[PIPELINE_STAGES::GENERATE_PTCL].push_back(make_pair(frame_id, sw_local.ElapsedMs()));
						while(running && !m_qViewPtcl.push(pClient))
							sleep_ms(sleep_time);

						// exit if all the frames are processed
						if(frame_id == m_end_frame_id)
						{
							sleep_ms(sleep_time);
							break;
						}
						m_vStageTimes[PIPELINE_STAGES::GENERATE_PTCL].push_back(make_pair(frame_id, sw.ElapsedMs()));
					}
					break;
				}
				
				// Client generate and view point cloud
				case PIPELINE_STAGES::VIEW_PTCL:
				{
					MultiviewClient *pClient = NULL;
					StopWatch sw, sw_local, sw_open3d;
					int frame_id = 0;

					unique_ptr<PointCloudViewer> pcl_viewer(nullptr);
					if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
						pcl_viewer = make_unique<PointCloudViewer>("viewer");
					
					unique_ptr<PointCloudStreamer> ptcl_streamer(nullptr);
					if(FLAGS_view_ptcl == VIEWER::UNITY_VIEWER)
        				ptcl_streamer = make_unique<PointCloudStreamer>(PTCL_STREAM_PORT);

					visualization::Visualizer visualizer;
					std::shared_ptr<geometry::PointCloud> pcd_ptr = nullptr;
					if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
					{
						// Visualizer initialization
						visualizer.CreateVisualizerWindow("Open3D", 1600, 900, 50, 50);
						pcd_ptr = std::make_shared<geometry::PointCloud>();
					}
					
					while(running)
					{
						sw.Restart();
						pClient = NULL;
						while(running && !m_qViewPtcl.pop(pClient))
							sleep_ms(sleep_time);
						
						sw_local.Restart();
						frame_id = pClient->get_curr_frameId();
						if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
						{
							// pClient->generate_pcl_object();
							PointCloud_t::Ptr pcl_object = pClient->get_pcl_object();

							if(FLAGS_save_ptcl)
								pClient->save_ptcl_binary_from_pcl(pcl_object);
							pcl_viewer->displayCullClipThread(pcl_object);
						}
						else if(FLAGS_view_ptcl == VIEWER::UNITY_VIEWER)
						{
							uint8_t *ptclBuffer;
							int32_t bufferSize = 0;
							pClient->get_streamable_ptcl(ptclBuffer, bufferSize);
							if(!ptcl_streamer->sendPointCloud(ptclBuffer, bufferSize))
							{
								running = false;
								LOG(FATAL) << "Client: Failed to send ptcl to Unity " << frame_id;
							}
							delete[] ptclBuffer;
						}
						else if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER && frame_id >= m_start_frame_id + 2)
						{
							sw_open3d.Restart();
							pcd_ptr->points_.clear();
							pcd_ptr->colors_.clear();
							pClient->get_open3d_points_colors(pcd_ptr->points_, pcd_ptr->colors_);

							// LOG(INFO) << "Generate Open3d for " << frame_id << ": " << sw_open3d.ElapsedMs() << " ms";

							sw_open3d.Restart();
							if(frame_id == m_start_frame_id + 2)
								visualizer.AddGeometry(pcd_ptr);
							else
								visualizer.UpdateGeometry(pcd_ptr);
							visualizer.PollEvents();
							visualizer.UpdateRender();
							// LOG(INFO) << "View Open3d for " << frame_id << ": " << sw_open3d.ElapsedMs() << " ms";
						}

						// Get next user trace
						unique_lock<mutex> lock(data_read_m);
            			m_origData = m_dataPlayback->getUserData(frame_id);
						lock.unlock();

						stageOperationTimes[PIPELINE_STAGES::VIEW_PTCL].push_back(make_pair(frame_id, sw_local.ElapsedMs()));
						// exit if all the frames are processed
						if(frame_id == m_end_frame_id)
						{
							LOG(INFO) << "Thread " << i << ": All Done for frame id: " << frame_id;
							sleep_ms(sleep_time);
							break;
						}

						/**
						 * @brief Pushing the pClient to the input queue of the server for reuse
						 * Don't check any variable of pClient, since it will reset by the input thread.
						 * Exception: Only for last frame, we can check the frame id since it's not inserted to the queue
						 * If there are 20 frames and stage queue of size = 3, then quit when the frame id is 17
						 * We don't want to reutilise the server object for the last 3 frames, since input frames are finished
						 */

						LOG(INFO) << "Thread " << i << ": Done with frame id: " << frame_id;
						if(frame_id <= (m_end_frame_id - min(nInputObjects, SQ_CAPACITY)))
						{
							// LOG(INFO) << "Thread " << i << " push to InputClient queue for frame id: " << frame_id << " atomic id: " << m_capture_id;

							// Insert to the input queue of the client
							while(running && !m_qInputClient.push(pClient))
								sleep_ms(sleep_time);
						}

						m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL].push_back(make_pair(frame_id, sw.ElapsedMs()));
						m_vFps.push_back(make_pair(frame_id, sw_pipeline.ElapsedMs()));
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
	assert(m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].size() == m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL].size());
	
	vector<float> avg_stage_latency(PIPELINE_STAGES::NUM_STAGES, 0.0f);
	vector<float> avg_stage_operation_latency(PIPELINE_STAGES::NUM_STAGES, 0.0f);
	// float avg_server_latency = 0.0f;
	// float avg_client_latency = 0.0f;
	float avg_fps = 0.0f;
	double avg_total_latency = 0.0;
	
	int skip_frames = 5;
	// skip the first 5 frames during calculation
	for(uint32_t i = skip_frames; i < m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].size(); i++)
	{
		assert(m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME][i].first == m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL][i].first);
		assert(m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL][i].first == m_vServerLatency[i].first);
		assert(m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL][i].first == m_vClientLatency[i].first);

		for(int j = 0; j < PIPELINE_STAGES::NUM_STAGES; j++)
		{
			avg_stage_latency[j] += m_vStageTimes[j][i].second;
			avg_stage_operation_latency[j] += stageOperationTimes[j][i].second;
			avg_total_latency += m_vStageTimes[j][i].second;
		}

		// avg_server_latency += m_vServerLatency[i].second;
		// avg_client_latency += m_vClientLatency[i].second;
		avg_fps = m_vFps[i].second - m_vFps[skip_frames].second;
	}

	int num_frames = m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].size() - skip_frames;
	for(int j = 0; j < PIPELINE_STAGES::NUM_STAGES; j++)
	{
		avg_stage_latency[j] /= num_frames;
		avg_stage_operation_latency[j] /= num_frames;
	}
	// avg_server_latency /= num_frames;
	// avg_client_latency /= num_frames;
	avg_total_latency /= num_frames;
	avg_fps = float(num_frames) * 1000.0f / avg_fps;

	LOG(INFO) << "------------------------------------------------------------";
	LOG(INFO) << "Frames: " << num_frames;
	LOG(INFO) << "Load frame: " << avg_stage_latency[PIPELINE_STAGES::LOAD_FRAME] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::LOAD_FRAME] << " ms";
	LOG(INFO) << "Update view: " << avg_stage_latency[PIPELINE_STAGES::UPDATE_VIEW] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::UPDATE_VIEW] << " ms";
	// LOG(INFO) << "Server latency: " << avg_server_latency << " ms";

	LOG(INFO) << "Update ptcl: " << avg_stage_latency[PIPELINE_STAGES::UPDATE_PTCL] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::UPDATE_PTCL] << " ms";
	LOG(INFO) << "Generate ptcl: " << avg_stage_latency[PIPELINE_STAGES::GENERATE_PTCL] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::GENERATE_PTCL] << " ms";
	LOG(INFO) << "View ptcl: " << avg_stage_latency[PIPELINE_STAGES::VIEW_PTCL] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::VIEW_PTCL] << " ms";
	// LOG(INFO) << "Client latency: " << avg_client_latency << " ms";
	LOG(INFO) << "End-to-end latency: " << avg_total_latency << " ms";
	LOG(INFO) << "FPS: " << avg_fps;
}

MultiviewServerPoolTest2::MultiviewServerPoolTest2(const string &path, const int datasetType, const int start_frame_id, const int end_frame_id, uint32_t nThreads) : m_nThreads(nThreads)
{	
	m_start_frame_id = start_frame_id;
	m_end_frame_id = end_frame_id;
	m_capture_id = start_frame_id;
	
	if(datasetType == DATASET::PANOPTIC)
	{
		string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);
		m_dataPlayback = make_unique<DataPlayback>(user_data_path, SEQ_NAME, LOG_ID);
    	m_dataPlayback->loadData(START_FRAME, END_FRAME);
		m_kalmanPredictor = make_unique<KalmanPredictor>();
	}
	else
	{
		m_dataPlayback = nullptr;
		m_kalmanPredictor = nullptr;
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
		m_pClients.push_back(new MultiviewClient(path, datasetType));

		vector<uint32_t> s_deviceIndices = m_pServers[i]->get_device_indices();
        vector<string> s_deviceSerialNumbers = m_pServers[i]->get_device_serial_numbers();
        int s_numDevices = m_pServers[i]->get_device_count();
        if(s_numDevices <= 0)
			LOG(FATAL) << "No devices found during playback!";
            
        if(s_numDevices != s_deviceIndices.size() || s_numDevices != s_deviceSerialNumbers.size())
            LOG(FATAL) << "Device count mismatch during playback!";
        
        // Don't free the calibration data, it is owned by the server
        vector<Calibration *> s_calibrations = m_pServers[i]->get_calibration();

        m_pClients[i]->init(s_deviceIndices, s_deviceSerialNumbers, s_calibrations);

		if(!m_pServers[i]->get_calibration_status())
            LOG(FATAL) << "Failed to load Calibration!";

        if(!m_pClients[i]->get_calibration_status())
            LOG(FATAL) << "Failed to load Calibration!";
		
		if(datasetType == DATASET::KINECT)
		{
			vector<xy_table_t *> s_xy_tables = m_pServers[i]->get_xy_table();
			m_pClients[i]->set_xy_table(s_xy_tables);
		}
	}

	assert(m_pServers.size() == IQ_CAPACITY);
	assert(m_pClients.size() == IQ_CAPACITY);
}

MultiviewServerPoolTest2::~MultiviewServerPoolTest2()
{
	m_qLoadFrame.reset();
	m_qUpdateView.reset();
	m_qServer2Client.reset();
	m_qInputClient.reset();

	for (uint32_t i = 0; i < IQ_CAPACITY; i++)
		delete m_pServers[i];
	m_pServers.clear();

	for (uint32_t i = 0; i < IQ_CAPACITY; i++)
		delete m_pClients[i];
	m_pClients.clear();
}

void MultiviewServerPoolTest2::start_sleep()
{
	std::signal(SIGINT, sigint_handler);

    vector<thread> threads;
	int nInputObjects = IQ_CAPACITY - 1;
    for (uint32_t i = 0; i < nInputObjects; i++)
	{
		m_qLoadFrame.push(m_pServers[i]);
	}

	for (uint32_t i = 0; i < nInputObjects; i++)
	{
		m_qInputClient.push(m_pClients[i]);
	}   

	vector<vector<pair<int, uint64_t>>> stageOperationTimes(PIPELINE_STAGES::NUM_STAGES);
	mutex data_read_m;

	StopWatch sw_pipeline;
	const int sleep_time = 1;

    for (uint32_t i = 0; i < m_nThreads; i++)
    {
        threads.push_back(thread([this, i, sleep_time, &data_read_m, &sw_pipeline, &stageOperationTimes, &nInputObjects] 
        {
            switch (i)
            {
				// Server Load Frame
				case PIPELINE_STAGES::LOAD_FRAME:
				{
					MultiviewServer *pServer = NULL;
					StopWatch sw, sw_local;
					int frame_id = 0;
					while(running)
					{
						sw.Restart();
						pServer = NULL;
						while(running && !m_qLoadFrame.pop(pServer))	
							sleep_ms(sleep_time);			// waiting to pop from the queue
						
						sw_local.Restart();
						pServer->restart_playback();
						if(m_capture_id == m_start_frame_id)
							sw_pipeline.Restart();							// Start the timer for the first frame
						pServer->set_curr_frameId(m_capture_id);
						m_capture_id++;

						frame_id = pServer->get_curr_frameId();
						if(!pServer->load_frame_parallel())
						{
							running = false;
							LOG(FATAL) << "Thread " << i << ": Failed to load frame " << frame_id;
						}
						stageOperationTimes[PIPELINE_STAGES::LOAD_FRAME].push_back(make_pair(frame_id, sw_local.ElapsedMs()));
						while(running && !m_qUpdateView.push(pServer))
							sleep_ms(sleep_time);

						// exit if all the frames are processed
						if(pServer->get_curr_frameId() == m_end_frame_id)
						{
							sleep_ms(sleep_time);
							break;
						}
						m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].push_back(make_pair(frame_id, sw.ElapsedMs()));
					}
					break;
				}

				// Server Update View
            	case PIPELINE_STAGES::UPDATE_VIEW:
				{
					MultiviewServer *pServer  = NULL;
					StopWatch sw, sw_local;
					int frame_id = 0;
					while(running)
					{
						sw.Restart();
						pServer  = NULL;
						while(running && !m_qUpdateView.pop(pServer))
							sleep_ms(sleep_time);

						sw_local.Restart();
						frame_id = pServer->get_curr_frameId();
						// Predict frustum from trace
						if(m_kalmanPredictor != nullptr)
						{
            				// Data race with UPDATE_PTCL
							unique_lock<mutex> lock(data_read_m);
							m_kalmanPredictor->predictData(frame_id, 1, m_origData, m_predData);
							pServer->set_frustum(m_origData.frustum, m_predData.frustum);
							lock.unlock();
						}

						pServer->update_view();

						stageOperationTimes[PIPELINE_STAGES::UPDATE_VIEW].push_back(make_pair(frame_id, sw_local.ElapsedMs()));

						while(running && !m_qServer2Client.push(pServer))
							sleep_ms(sleep_time);

						// exit if all the frames are processed
						if(frame_id == m_end_frame_id)
						{
							sleep_ms(sleep_time);
							break;
						}
						m_vStageTimes[PIPELINE_STAGES::UPDATE_VIEW].push_back(make_pair(frame_id, sw.ElapsedMs()));
					}
					break;
				}

				// Client Update Ptcl
				case PIPELINE_STAGES::UPDATE_PTCL:
				{
					MultiviewServer *pServer = NULL;
					MultiviewClient *pClient = NULL;
					StopWatch sw, sw_local;
					int frame_id = 0;
					while(running)
					{
						sw.Restart();
						pServer = NULL;
						pClient = NULL;
						while(running && !m_qServer2Client.pop(pServer))
							sleep_ms(sleep_time);

						while(running && !m_qInputClient.pop(pClient))
							sleep_ms(sleep_time);
						
						sw_local.Restart();
						frame_id = pServer->get_curr_frameId();

						pClient->restart_playback();
						pClient->set_curr_frameId(frame_id);
						vector<View *> s_views = pServer->get_view();

						unique_lock<mutex> lock(data_read_m);
						m_origData = m_dataPlayback->getUserData(frame_id);
						pClient->set_frustum(m_origData.frustum);
						lock.unlock();

						if(!pClient->set_view(s_views))
						{
							running = false;
							LOG(FATAL) << "Thread " << i << ": Failed to set view " << frame_id;
						}

						/**
						 * @brief Pushing the pServer to the input queue of the server for reuse
						 * Don't check any variable of pServer, since it will reset by the input thread.
						 * Exception: Only for last frame, we can check the frame id since it's not inserted to the queue
						 * If there are 20 frames and stage queue of size = 3, then quit when the frame id is 17
						 * We don't want to reutilise the server object for the last 3 frames, since input frames are finished
						 */
						if(frame_id <= (m_end_frame_id - min(nInputObjects, SQ_CAPACITY)))
						{
							// Insert to the input queue of the server
							while(running && !m_qLoadFrame.push(pServer))
								sleep_ms(sleep_time);
						}
						
						if(!pClient->update_ptcl())
						{
							running = false;
							LOG(FATAL) << "Thread " << i << ": Failed to update ptcl " << frame_id;
						}

						stageOperationTimes[PIPELINE_STAGES::UPDATE_PTCL].push_back(make_pair(frame_id, sw_local.ElapsedMs()));

						while(running && !m_qGeneratePtcl.push(pClient))
							sleep_ms(sleep_time);

						// exit if all the frames are processed
						if(frame_id == m_end_frame_id)
						{
							sleep_ms(sleep_time);
							break;
						}
						m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL].push_back(make_pair(frame_id, sw.ElapsedMs()));
					}
					break;
				}

				case PIPELINE_STAGES::GENERATE_PTCL:
				{
					MultiviewClient *pClient = NULL;
					StopWatch sw, sw_local;
					int frame_id = 0;
					while(running)
					{
						sw.Restart();
						pClient = NULL;
						while(running && !m_qGeneratePtcl.pop(pClient))
							sleep_ms(sleep_time);
						
						sw_local.Restart();
						frame_id = pClient->get_curr_frameId();
						if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
						{
							PclData *pcl_data = new PclData();
							pcl_data->frame_id = frame_id;
							pClient->generate_pcl_object(pcl_data->pcl_object);
							
							while(running && !m_qPcl.push(pcl_data))
								sleep_ms(sleep_time);
						}
						else if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
						{
							O3DData *o3d_data = new O3DData();
							o3d_data->frame_id = frame_id;
							pClient->get_open3d_points_colors(o3d_data->points, o3d_data->colors);

							while(running && !m_qOpen3D.push(o3d_data))
								sleep_ms(sleep_time);
						}
						
						stageOperationTimes[PIPELINE_STAGES::GENERATE_PTCL].push_back(make_pair(frame_id, sw_local.ElapsedMs()));

						// exit if all the frames are processed
						if(frame_id == m_end_frame_id)
						{
							sleep_ms(sleep_time);
							break;
						}
						m_vStageTimes[PIPELINE_STAGES::GENERATE_PTCL].push_back(make_pair(frame_id, sw.ElapsedMs()));

						/**
						 * @brief Pushing the pClient to the input queue of the server for reuse
						 * Don't check any variable of pClient, since it will reset by the input thread.
						 * Exception: Only for last frame, we can check the frame id since it's not inserted to the queue
						 * If there are 20 frames and stage queue of size = 3, then quit when the frame id is 17
						 * We don't want to reutilise the server object for the last 3 frames, since input frames are finished
						 */

						LOG(INFO) << "Thread " << i << ": Done with frame id: " << frame_id;
						if(frame_id <= (m_end_frame_id - min(nInputObjects, SQ_CAPACITY)))
						{
							// LOG(INFO) << "Thread " << i << " push to InputClient queue for frame id: " << frame_id << " atomic id: " << m_capture_id;

							// Insert to the input queue of the client
							while(running && !m_qInputClient.push(pClient))
								sleep_ms(sleep_time);
						}
					}
					break;
				}
				
				// Client generate and view point cloud
				case PIPELINE_STAGES::VIEW_PTCL:
				{
					StopWatch sw, sw_local;
					int frame_id = 0;

					unique_ptr<PointCloudViewer> pcl_viewer(nullptr);
					if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
						pcl_viewer = make_unique<PointCloudViewer>("viewer");
					
					unique_ptr<PointCloudStreamer> ptcl_streamer(nullptr);
					if(FLAGS_view_ptcl == VIEWER::UNITY_VIEWER)
        				ptcl_streamer = make_unique<PointCloudStreamer>(PTCL_STREAM_PORT);

					visualization::Visualizer visualizer;
					std::shared_ptr<geometry::PointCloud> pcd_ptr = nullptr;
					if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
					{
						// Visualizer initialization
						visualizer.CreateVisualizerWindow("Open3D", 1600, 900, 50, 50);
						pcd_ptr = std::make_shared<geometry::PointCloud>();
					}
					
					while(running)
					{
						sw.Restart();
						if(FLAGS_view_ptcl == VIEWER::PCL_VIEWER)
						{
							PclData *pcl_data = NULL;
							while(running && !m_qPcl.pop(pcl_data))
								sleep_ms(sleep_time);
							
							frame_id = pcl_data->frame_id;
							sw_local.Restart();
							pcl_viewer->displayCullClipThread(pcl_data->pcl_object);

							delete pcl_data;
						}
						else if(FLAGS_view_ptcl == VIEWER::OPEN3D_VIEWER)
						{
							O3DData *o3d_data = NULL;
							while(running && !m_qOpen3D.pop(o3d_data))
								sleep_ms(sleep_time);
							
							frame_id = o3d_data->frame_id;
							sw_local.Restart();
							if(frame_id >= m_start_frame_id + 2)
							{
								pcd_ptr->points_ = o3d_data->points;
								pcd_ptr->colors_ = o3d_data->colors;

								if(frame_id == m_start_frame_id + 2)
									visualizer.AddGeometry(pcd_ptr);
								else
									visualizer.UpdateGeometry(pcd_ptr);
								visualizer.PollEvents();
								visualizer.UpdateRender();
							}
							delete o3d_data;
						}

						stageOperationTimes[PIPELINE_STAGES::VIEW_PTCL].push_back(make_pair(frame_id, sw_local.ElapsedMs()));
						// exit if all the frames are processed
						if(frame_id == m_end_frame_id)
						{
							LOG(INFO) << "Thread " << i << ": All Done for frame id: " << frame_id;
							sleep_ms(sleep_time);
							break;
						}

						m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL].push_back(make_pair(frame_id, sw.ElapsedMs()));
						m_vFps.push_back(make_pair(frame_id, sw_pipeline.ElapsedMs()));
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
	assert(m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].size() == m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL].size());
	
	vector<float> avg_stage_latency(PIPELINE_STAGES::NUM_STAGES, 0.0f);
	vector<float> avg_stage_operation_latency(PIPELINE_STAGES::NUM_STAGES, 0.0f);
	// float avg_server_latency = 0.0f;
	// float avg_client_latency = 0.0f;
	float avg_fps = 0.0f;
	double avg_total_latency = 0.0;
	
	int skip_frames = 5;
	// skip the first 5 frames during calculation
	for(uint32_t i = skip_frames; i < m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].size(); i++)
	{
		assert(m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME][i].first == m_vStageTimes[PIPELINE_STAGES::UPDATE_PTCL][i].first);
		assert(m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL][i].first == m_vServerLatency[i].first);
		assert(m_vStageTimes[PIPELINE_STAGES::VIEW_PTCL][i].first == m_vClientLatency[i].first);

		for(int j = 0; j < PIPELINE_STAGES::NUM_STAGES; j++)
		{
			avg_stage_latency[j] += m_vStageTimes[j][i].second;
			avg_stage_operation_latency[j] += stageOperationTimes[j][i].second;
			avg_total_latency += m_vStageTimes[j][i].second;
		}

		// avg_server_latency += m_vServerLatency[i].second;
		// avg_client_latency += m_vClientLatency[i].second;
		avg_fps = m_vFps[i].second - m_vFps[skip_frames].second;
	}

	int num_frames = m_vStageTimes[PIPELINE_STAGES::LOAD_FRAME].size() - skip_frames;
	for(int j = 0; j < PIPELINE_STAGES::NUM_STAGES; j++)
	{
		avg_stage_latency[j] /= num_frames;
		avg_stage_operation_latency[j] /= num_frames;
	}
	// avg_server_latency /= num_frames;
	// avg_client_latency /= num_frames;
	avg_total_latency /= num_frames;
	avg_fps = float(num_frames) * 1000.0f / avg_fps;

	LOG(INFO) << "------------------------------------------------------------";
	LOG(INFO) << "Frames: " << num_frames;
	LOG(INFO) << "Load frame: " << avg_stage_latency[PIPELINE_STAGES::LOAD_FRAME] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::LOAD_FRAME] << " ms";
	LOG(INFO) << "Update view: " << avg_stage_latency[PIPELINE_STAGES::UPDATE_VIEW] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::UPDATE_VIEW] << " ms";
	// LOG(INFO) << "Server latency: " << avg_server_latency << " ms";

	LOG(INFO) << "Update ptcl: " << avg_stage_latency[PIPELINE_STAGES::UPDATE_PTCL] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::UPDATE_PTCL] << " ms";
	LOG(INFO) << "Generate ptcl: " << avg_stage_latency[PIPELINE_STAGES::GENERATE_PTCL] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::GENERATE_PTCL] << " ms";
	LOG(INFO) << "View ptcl: " << avg_stage_latency[PIPELINE_STAGES::VIEW_PTCL] << " ms, local operation: " << avg_stage_operation_latency[PIPELINE_STAGES::VIEW_PTCL] << " ms";
	// LOG(INFO) << "Client latency: " << avg_client_latency << " ms";
	LOG(INFO) << "End-to-end latency: " << avg_total_latency << " ms";
	LOG(INFO) << "FPS: " << avg_fps;
}