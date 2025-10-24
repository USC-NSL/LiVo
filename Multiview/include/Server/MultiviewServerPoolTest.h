#pragma once
#include "Server/MultiviewServer.h"
#include "Client/MultiviewClient.h"
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

#define IQ_CAPACITY 8 	// Input queue capacity
#define SQ_CAPACITY 5	// Inter-stage queue capacity

using namespace std;
namespace bf = boost::lockfree;

namespace PIPELINE_STAGES
{
	enum
	{
		LOAD_FRAME = 0,
		UPDATE_VIEW,
		UPDATE_PTCL,
		GENERATE_PTCL,
		VIEW_PTCL,
		NUM_STAGES
	};
}

class MultiviewServerPoolTest
{

public:
	MultiviewServerPoolTest(const string &path, const int datasetType, const int start_frame_id, const int end_frame_id, uint32_t nThreads);
	~MultiviewServerPoolTest();
	void start_sleep();
	void start_conditional();

private:
	uint32_t m_nThreads;
	vector<MultiviewServer *> m_pServers;
	vector<MultiviewClient *> m_pClients;

	unique_ptr<DataPlayback> m_dataPlayback;
    UserData m_origData, m_predData;
    unique_ptr<KalmanPredictor> m_kalmanPredictor;

	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_CAPACITY>> m_qLoadFrame;
	bf::spsc_queue<MultiviewServer *, bf::capacity<SQ_CAPACITY>> m_qUpdateView;
	bf::spsc_queue<MultiviewServer *, bf::capacity<SQ_CAPACITY>> m_qServer2Client;

	bf::spsc_queue<MultiviewClient *, bf::capacity<IQ_CAPACITY>> m_qInputClient;
	bf::spsc_queue<MultiviewClient *, bf::capacity<SQ_CAPACITY>> m_qGeneratePtcl;
	bf::spsc_queue<MultiviewClient *, bf::capacity<SQ_CAPACITY>> m_qViewPtcl;

	// std::mutex m;
	// std::condition_variable cv;

	std::atomic_int m_start_frame_id;
	std::atomic_int m_end_frame_id;
	std::atomic_int m_capture_id;

	vector<vector<pair<int, uint64_t>>> m_vStageTimes;
	vector<pair<int, uint64_t>> m_vServerLatency;
	vector<pair<int, uint64_t>> m_vClientLatency;
	vector<pair<int, uint64_t>> m_vFps;
};

class MultiviewServerPoolTest2
{

public:
	MultiviewServerPoolTest2(const string &path, const int datasetType, const int start_frame_id, const int end_frame_id, uint32_t nThreads);
	~MultiviewServerPoolTest2();
	void start_sleep();

private:
	
	uint32_t m_nThreads;
	vector<MultiviewServer *> m_pServers;
	vector<MultiviewClient *> m_pClients;

	unique_ptr<DataPlayback> m_dataPlayback;
    UserData m_origData, m_predData;
    unique_ptr<KalmanPredictor> m_kalmanPredictor;

	bf::spsc_queue<MultiviewServer *, bf::capacity<IQ_CAPACITY>> m_qLoadFrame;
	bf::spsc_queue<MultiviewServer *, bf::capacity<SQ_CAPACITY>> m_qUpdateView;
	bf::spsc_queue<MultiviewServer *, bf::capacity<SQ_CAPACITY>> m_qServer2Client;

	bf::spsc_queue<MultiviewClient *, bf::capacity<IQ_CAPACITY>> m_qInputClient;
	bf::spsc_queue<MultiviewClient *, bf::capacity<SQ_CAPACITY>> m_qGeneratePtcl;
	bf::spsc_queue<O3DData *, bf::capacity<1000>> m_qOpen3D;
	bf::spsc_queue<PclData *, bf::capacity<1000>> m_qPcl;

	// std::mutex m;
	// std::condition_variable cv;

	std::atomic_int m_start_frame_id;
	std::atomic_int m_end_frame_id;
	std::atomic_int m_capture_id;

	vector<vector<pair<int, uint64_t>>> m_vStageTimes;
	vector<pair<int, uint64_t>> m_vServerLatency;
	vector<pair<int, uint64_t>> m_vClientLatency;
	vector<pair<int, uint64_t>> m_vFps;

};