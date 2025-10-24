#pragma once
#include "Server/MultiviewServer.h"
#include "Server/WebRTCServer.h"
#include <boost/lockfree/spsc_queue.hpp>
// #include <boost/lockfree/policies.hpp>
#include <iostream>
#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include "DataPlayback.h"
#include "pconsts.h"
#include "consts.h"
#include "timer.h"
#include "mahimahi.h"

#define IQ_CAPACITY 3 	// Input queue capacity
#define SQ_CAPACITY 1	// Inter-stage queue capacity

using namespace std;
namespace bf = boost::lockfree;

namespace PIPELINE_STAGES
{
	enum
	{
		LOAD_FRAME = 0,
		UPDATE_VIEW,
		SEND_FRAME,
		NUM_STAGES
	};
}

class MultiviewServerPool
{

public:
	MultiviewServerPool(const string &path, const int datasetType, const int start_frame_id, const int end_frame_id, uint32_t nThreads);
	~MultiviewServerPool();
	void run();
	void run_ptcl();

private:
	void simluate_lookahead(MultiviewServer *pServer, int lookAhead, Frustum &s_frustum, Frustum &s_pred_frustum);

	uint32_t m_nThreads;
	vector<MultiviewServer *> m_pServers;
	
    UserData m_origData, m_predData;
	unique_ptr<DataPlayback> m_dataPlayback;
    unique_ptr<KalmanPredictor> m_kalmanPredictor;
	unique_ptr<KalmanPredictor2> m_kalmanPredictor2;

	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_CAPACITY>> m_qLoadFrame;
	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_CAPACITY>> m_qUpdateView;
	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_CAPACITY>> m_qServer2Client;

	// std::mutex m;
	// std::condition_variable cv;

	std::atomic_int m_start_frame_id;
	std::atomic_int m_end_frame_id;
	std::atomic_int m_capture_id;

	vector<vector<pair<int, uint64_t>>> m_vStageTimes;
	vector<pair<int, uint64_t>> m_vServerLatency;
	vector<pair<int, uint64_t>> m_vFps;
	StopWatch sw;

	vector<View *> s_views;

	uint32_t m_frame_counter;
	FPSCounter fps;
	FPSCounter2 fps_limiter2;

	string mahi_metafile;
    Mahimahi *up;
    Mahimahi *down;
};

#define IQ_PTCL_CAPACITY 4 	// Input queue capacity
#define SQ_PTCL_CAPACITY 1	// Inter-stage queue capacity

namespace PIPELINE_PTCL_STAGES
{
	enum
	{
		LOAD_FRAME = 0,
		UPDATE_PTCL,
		COMPRESS_PTCL,
		SEND_FRAME,
		NUM_STAGES
	};
}

class MultiviewServerPoolPtcl
{

public:
	MultiviewServerPoolPtcl(const string &path, const int datasetType, const int start_frame_id, const int end_frame_id, string compression_type, zstd_options zstd_params, draco_options draco_params, uint32_t nThreads);
	~MultiviewServerPoolPtcl();
	void run_ptcl();

private:
	uint32_t m_nThreads;
	vector<MultiviewServer *> m_pServers;
	
    UserData m_origData, m_predData;
	unique_ptr<DataPlayback> m_dataPlayback;
    unique_ptr<KalmanPredictor> m_kalmanPredictor;

	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_PTCL_CAPACITY>> m_qLoadFrame;
	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_PTCL_CAPACITY>> m_qUpdatePtcl;
	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_PTCL_CAPACITY>> m_qCompressPtcl;
	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_PTCL_CAPACITY>> m_qServer2Client;

	// std::mutex m;
	// std::condition_variable cv;

	std::atomic_int m_start_frame_id;
	std::atomic_int m_end_frame_id;
	std::atomic_int m_capture_id;

	vector<vector<pair<int, uint64_t>>> m_vStageTimes;

	// Ptcl compression settings
	string m_compression_type;
	zstd_options m_zstd_params;
	draco_options m_draco_params;

	StopWatch sw;

	vector<View *> s_views;

	uint32_t m_frame_counter;
	FPSCounter fps;
	FPSCounter2 fps_limiter2;

	// Maintain Statistics
	Accumulator server_load_time_acc;
	Accumulator server_update_time_acc;
	Accumulator server_compression_time_acc;
	Accumulator server_raw_points_acc;
	Accumulator server_raw_size_acc;
	Accumulator server_compressed_size_acc;
};