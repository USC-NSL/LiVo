#pragma once
#include <boost/lockfree/spsc_queue.hpp>
// #include <boost/lockfree/policies.hpp>
#include <iostream>
#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include "Server/MultiviewServer.h"
#include "Server/NetworkSender.h"
#include "DataPlayback.h"
#include "pconsts.h"
#include "consts.h"
#include "timer.h"
#include "mahimahi.h"

#define IQ_CAPACITY 3 	// Input queue capacity
#define SQ_CAPACITY 3	// Inter-stage queue capacity

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

class MultiviewServerPoolNew
{

public:
	MultiviewServerPoolNew(const std::string &path, const int datasetType, const int start_frame_id, const int end_frame_id, uint32_t nThreads);
	~MultiviewServerPoolNew();
	void run();
	void run_ptcl();

private:
	void simluate_lookahead(MultiviewServer *pServer, int lookAhead, Frustum &s_frustum, Frustum &s_pred_frustum);

	uint32_t m_nThreads;
	vector<MultiviewServer *> m_pServers;
	
    UserData m_origData, m_predData;
	unique_ptr<DataPlayback> m_dataPlayback;
    // unique_ptr<KalmanPredictor> m_kalmanPredictor;
	unique_ptr<KalmanPredictor2> m_kalmanPredictor2;

	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_CAPACITY>> m_qLoadFrame;
	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_CAPACITY>> m_qUpdateView;
	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_CAPACITY>> m_qServer2Client;

	// std::mutex m;
	// std::condition_variable cv;

	std::atomic_int m_start_frame_id;
	std::atomic_int m_end_frame_id;
	std::atomic_int m_capture_id;
	vector<View *> s_views;

	vector<vector<pair<int, uint64_t>>> m_vStageTimes;
	vector<pair<int, uint64_t>> m_vServerLatency;
	vector<pair<int, uint64_t>> m_vFps;
	StopWatch sw;

	uint32_t m_frame_counter;
	FPSCounter fps;
	FPSCounter2 fps_limiter2;

	std::string mahi_metafile;
    Mahimahi *up;
    Mahimahi *down;
};