#pragma once
#include <boost/lockfree/spsc_queue.hpp>
// #include <boost/lockfree/policies.hpp>
#include <iostream>
#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"
#include <opencv2/objdetect.hpp>
#include "Client/MultiviewClient.h"
#include "Client/BitrateSplitterClient.h"
#include "DataPlayback.h"
#include "pconsts.h"
#include "consts.h"
#include "timer.h"
#include "mahimahi.h"

#define IQ_CAPACITY 3 	// Input queue capacity
#define SQ_CAPACITY 3	// Inter-stage queue capacity
// #define VQ_CAPACITY 1000	// Viewer queue capacity
#define VQ_CAPACITY 100	// Viewer queue capacity

using namespace std;
namespace bf = boost::lockfree;
using namespace open3d;

namespace PIPELINE_STAGES
{
	enum
	{
		LOAD_FRAME = 0,
		UPDATE_PTCL,
		GENERATE_PTCL,
		VIEW_PTCL,
		SAVE_RENDER,
		NUM_STAGES
	};
}

typedef struct O3DVizData
{
	uint32_t frame_id;
	visualization::Visualizer *viz;
	O3DVizData() : frame_id(0), viz(nullptr) {}
	O3DVizData(uint32_t frame_id, visualization::Visualizer *viz) : frame_id(frame_id), viz(viz) {}
} O3DVizData;

class MultiviewClientPoolNew
{

public:
	MultiviewClientPoolNew(const std::string &path, const int datasetType, const int start_frame_id, const int end_frame_id, uint32_t nThreads);
	~MultiviewClientPoolNew();
	void run();
	void dummy_run();
	// void run_ptcl();


private:
	uint32_t m_nThreads;
	vector<MultiviewClient *> m_pClients;

	unique_ptr<DataPlayback> m_dataPlayback;
	std::string m_outputPath;
    // UserData m_origData;


	bf::spsc_queue<MultiviewClient *, bf::capacity<IQ_CAPACITY>> m_loadFrame;
	bf::spsc_queue<MultiviewClient *, bf::capacity<SQ_CAPACITY>> m_qUpdatePtcl;
	bf::spsc_queue<MultiviewClient *, bf::capacity<SQ_CAPACITY>> m_qGeneratePtcl;
	bf::spsc_queue<O3DData *, bf::capacity<VQ_CAPACITY>> m_qOpen3D;
	bf::spsc_queue<PclData *, bf::capacity<VQ_CAPACITY>> m_qPcl;
	bf::spsc_queue<O3DVizData *, bf::capacity<VQ_CAPACITY>> m_qSaveRender;
	bf::spsc_queue<O3DData *, bf::capacity<VQ_CAPACITY>> m_qSavePtcl;
	bf::spsc_queue<uint32_t, bf::capacity<VQ_CAPACITY>> m_qDummy;

	// std::mutex m;
	// std::condition_variable cv;

	std::atomic_int m_start_frame_id;
	std::atomic_int m_end_frame_id;
	std::atomic_int m_capture_id;
	vector<View *> c_views;

	cv::QRCodeDetector m_qrDecoder;

	vector<vector<pair<int, uint64_t>>> m_vStageTimes;
	vector<pair<int, uint64_t>> m_vClientLatency;
	vector<pair<int, uint64_t>> m_vFps;
	StopWatch sw;

	uint32_t m_frame_counter;
	FPSCounter fps;
	FPSCounter2 fps_limiter, fps_counter;

	// Control Bitrate
	std::string mahi_metafile;
    Mahimahi *up;
    Mahimahi *down;
	BitrateSplitterClient *bs;

	// bool load_trace();
    // int get_throughput();

	int32_t detect_qrcode(View *view, int border=4*2);
};