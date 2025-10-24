#include <nlohmann/json.hpp>
#include <iostream>
#include "DataPlayback.h"
#include "utils.h"
#include "pconsts.h"
#include "consts.h"
#include "kalman.h"

using namespace std;
using json = nlohmann::json;

// Defined in the main file like - main.cpp, panoptic.cpp, panoptic_playback.cpp
extern std::string PANOPTIC_DATA_PATH;
extern std::string USER_DATA_PATH;
// extern std::string SEQ_NAME;
// extern int LOG_ID;
// extern std::string USER_TRACE_FOLDER;
// extern std::string IP_ADDRESS;
// extern int PORT;
// extern uint32_t START_FRAME;
// extern uint32_t END_FRAME;

int main()
{
	absl::SetMinLogLevel(absl::LogSeverityAtLeast::kInfo);
	PANOPTIC_DATA_PATH = "/datassd/KinectStream/panoptic_captures/";
	USER_DATA_PATH = "/data/Dropbox/Project/VolumetricVideo/KinectStream/data/";
	string seqName = "170307_dance5";
	string data_dir = FORMAT(USER_DATA_PATH << "user_log/2023-03-09/");
	
	int logID = 0;

	json j;
    loadjson(FORMAT(PANOPTIC_DATA_PATH << "FrameIndices.json"), j);

    uint32_t startFrame = j[seqName]["start_idx"].get<uint32_t>();
    uint32_t endFrame = j[seqName]["end_idx"].get<uint32_t>();

	DataPlayback dataPlayback(data_dir, seqName, logID);
	dataPlayback.loadData(startFrame, endFrame);

	UserData userData = dataPlayback.getUserData(startFrame);

	LOG(INFO) << "Frame ID: " << userData.frameID;
	LOG(INFO) << "Timestamp: " << userData.ts_orig;
	LOG(INFO) << "Camera Pose: " << userData.pos;
	LOG(INFO) << "Camera Rotation: " << userData.rot;
	LOG(INFO) << "Frustum: ";
	userData.frustum.print();

	LOG(INFO) << "------------------------------------------------------------------";
	KalmanPredictor kp;

	UserData orig, pred;
	for(uint32_t i = dataPlayback.getStartFrameID(); i < dataPlayback.getEndFrameID(); i++)
	{
		kp.predictData(i, 1, orig, pred);
		LOG(INFO) << "O Frame ID: " << orig.frameID;
		LOG(INFO) << "O Timestamp: " << orig.ts_orig;
		LOG(INFO) << "O Camera Pose: " << orig.pos;
		LOG(INFO) << "O Camera Rotation: " << orig.rot << endl;

		LOG(INFO) << "P Frame ID: " << pred.frameID;
		LOG(INFO) << "P Timestamp: " << pred.ts_orig;
		LOG(INFO) << "P Camera Pose: " << pred.pos;
		LOG(INFO) << "P Camera Rotation: " << pred.rot << endl;
		LOG(INFO) << "------------------------------------------------------------------";

		orig = dataPlayback.getUserData(i);
	}

	return 0;
}