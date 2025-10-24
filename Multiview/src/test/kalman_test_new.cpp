#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <chrono>
#include <thread>
#include <cmath>

#include "kalman.h"
#include "voxel.h"
#include "timer.h"
#include "test/utils.h"
#include "pconsts.h"
#include "DataPlayback.h"

typedef vector<vector<float>> Data;

#define VOXEL_SIZE 0.05F		// To Do: Rajrup: Not working for 0.01F
#define VOXEL_MIN_BOUND -4.0F
#define VOXEL_MAX_BOUND 4.0F
#define LOOK_AHEAD_TIME 300.0f       // Kalman window size 300 ms
#define LOG_ID 0

using namespace std;
namespace fs = boost::filesystem;

void calc_max_min_corners_frustum(const Frustum &frustum, Vector3f &min_corner, Vector3f &max_corner)
{
	Vector3f corners[8] = {frustum.fbl, frustum.fbr, frustum.ftl, frustum.ftr, frustum.nbl, frustum.nbr, frustum.ntl, frustum.ntr};

	for(int i = 0; i < 8; i++)
	{
		if(i == 0)
		{
			min_corner = corners[i];
			max_corner = corners[i];
		}
		else
		{
			min_corner = min_corner.cwiseMin(corners[i]);
			max_corner = max_corner.cwiseMax(corners[i]);
		}
	}
}

void frustumPrediction()
{
	LOG(INFO) << "Frustrum KF prediction";

    string path = FORMAT(PANOPTIC_DATA_PATH << SEQ_NAME << "/");
    LOG(INFO) << "Path: " << path;
    string user_data_path = FORMAT(USER_DATA_PATH << USER_TRACE_FOLDER);

    if(!fs::exists(user_data_path))
        LOG(FATAL) << "Directory " << user_data_path << " doesn't exist";

    std::unique_ptr<DataPlayback> data_playback = make_unique<DataPlayback>(user_data_path, SEQ_NAME, LOG_ID);
    data_playback->loadData(START_FRAME, END_FRAME);

    string project_dir = "/home/lei/rajrup/KinectStream/";
    string pred_data_folder = FORMAT(project_dir << "data/predicted_data_new/" << SEQ_NAME << "/");
    if(!fs::exists(pred_data_folder))
    {
        LOG(WARNING) << "Predicted data folder does not exist!";
        LOG(INFO) << "Creating folder: " << pred_data_folder;
        fs::create_directory(pred_data_folder);
    }

    fs::path out_file{FORMAT(pred_data_folder << "camera_log" << LOG_ID << "_original.csv")};
	fs::ofstream ofs;
	ofs.open(out_file, ios::out | ios::trunc);
	if (!ofs.is_open())
	{
		LOG(INFO) << "Failed to open Kalman Output file: " << out_file;
		return;
	}

    ofs << "Time(ms),x,y,z,qx,qy,qz,qw" << endl;

    CamInt camInt;
    Data data;
    for(int i=START_FRAME; i <= END_FRAME; i++)
    {
        UserData orig_data = data_playback->getUserData(i);
        data.push_back(vector<float>{float(orig_data.ts_orig)/1000.0f, orig_data.pos(0), orig_data.pos(1), orig_data.pos(2), orig_data.quat(0), orig_data.quat(1), orig_data.quat(2), orig_data.quat(3)});
        ofs << orig_data.ts_orig << "," << std::fixed << setprecision(7) << orig_data.pos(0) << "," << orig_data.pos(1) << "," << orig_data.pos(2) << "," << orig_data.quat(0) << "," << orig_data.quat(1) << "," << orig_data.quat(2) << "," << orig_data.quat(3) << endl;
        if(i == START_FRAME)
        {
            camInt.angle = orig_data.frustum.angle;
            camInt.ratio = orig_data.frustum.ratio;
            camInt.nearD = orig_data.frustum.nearD;
            camInt.farD = orig_data.frustum.farD;
        }
    }
    ofs.close();

    out_file = FORMAT(pred_data_folder << "camera_log" << LOG_ID << "_kalman.csv");
	ofs.open(out_file, ios::out | ios::trunc);
	if (!ofs.is_open())
	{
		LOG(INFO) << "Failed to open Kalman Output file: " << out_file;
		return;
	}

    ofs << "Time(ms),x,y,z,qx,qy,qz,qw" << endl;

    double avg_delta_t = 0.0f;
    for(int i = 10; i < data.size(); i++)
    {
        avg_delta_t += data[i][0] - data[i-1][0];
    }
    avg_delta_t /= double(data.size() - 10);
    avg_delta_t *= 1000.0; // in ms
    assert(avg_delta_t > 10);

    //////////////////////////////
    /* Step 1: Kalman filtering */
    //////////////////////////////
    LOG(INFO) << "Starting Kalman Filter";

	VVR_Kalman *kalman = new VVR_Kalman(14, 7);
    float kalman_lookahead_t = LOOK_AHEAD_TIME;  // in ms for lookahead window
    int lookAhead = std::ceil(kalman_lookahead_t/avg_delta_t);
    LOG(INFO) << "Average Delta T: " << avg_delta_t;
    LOG(INFO) << "Lookahead Time: " << lookAhead;
    assert(lookAhead > 0);

	/**
     * @brief data[0] is dummy.
     * @brief data[1] is the first measurement. Initializes the model.
     * @brief Similary, predictions[0] is dummy.
     * @brief predictions[1] is the first measurement.
     * @brief predictions[2] is the first prediction.
     */

    Data predictions;
    for(int i = 0; i < data.size(); i++)
    {
        if(i == 0)
        {
            ofs << int(data[i][0]*1000.0f) << "," << std::fixed << setprecision(7) << data[i][1] << "," << data[i][2] << "," << data[i][3] << "," << data[i][4] << "," << data[i][5] << "," << data[i][6] << "," << data[i][7] << endl;
            predictions.push_back(vector<float>{data[i][0]*1000.0f, data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6], data[i][7]});
        }
        else if(i == 1)
        {
            // Rajrup: Since some precision is lost in the conversion to seconds, we need to fix this later.
            kalman->Init_Model(data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6], data[i][7], data[i][0]);
            ofs << int(data[i][0]*1000.0f) << "," << std::fixed << setprecision(7) << data[i][1] << "," << data[i][2] << "," << data[i][3] << "," << data[i][4] << "," << data[i][5] << "," << data[i][6] << "," << data[i][7] << endl;
            predictions.push_back(vector<float>{data[i][0]*1000.0f, data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6], data[i][7]});
        }
        else if(i >= 2)
        {
            vector<float> prediction = kalman->Predict_Update(data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6], data[i][7], data[i][0] - data[i-1][0], lookAhead);
            ofs << int(data[i][0]*1000.0f) << "," << std::fixed << setprecision(7) << prediction[0] << "," << prediction[1] << "," << prediction[2] << "," << prediction[3] << "," << prediction[4] << "," << prediction[5] << "," << prediction[6];
            predictions.push_back(vector<float>{data[i][0]*1000.0f, prediction[0], prediction[1], prediction[2], prediction[3], prediction[4], prediction[5], prediction[6]});

            if(i < data.size() - 1)
                ofs << endl;
        }
        else
            LOG(FATAL) << "Error: Invalid index";
    }
	ofs.close();

    //////////////////////////////
    /* Step 2: Voxel Prediction */
    //////////////////////////////
    Vector3f minBound, maxBound;
    minBound[0] = minBound[1] = minBound[2] = VOXEL_MIN_BOUND;
	maxBound[0] = maxBound[1] = maxBound[2] = VOXEL_MAX_BOUND;
    Voxel *pVoxel = new Voxel(minBound, maxBound, VOXEL_SIZE);

    assert(predictions.size() == data.size());

	StopWatch sw;

    uint64_t correct_predicitions = 0, incorrect_predictions = 0, num_predictions = 0;
    for(int i = 0; i < predictions.size(); i++)
    {
        if(i < 2)
        {
            // Skip the first two entries of predictions.
            continue;
        }
        else if(i >= 2)
        {
            Vector3f prev_pos = Vector3f(data[i][1], data[i][2], data[i][3]);
            Vector3f prev_rot = Vector3f(data[i][4], data[i][5], data[i][6]);
            Vector3f pred_pos = Vector3f(predictions[i][1], predictions[i][2], predictions[i][3]);
            Vector3f pred_rot = Vector3f(predictions[i][4], predictions[i][5], predictions[i][6]);

            // check point is within bounds
            if (prev_pos[0] < VOXEL_MIN_BOUND || prev_pos[0] > VOXEL_MAX_BOUND ||
                prev_pos[1] < VOXEL_MIN_BOUND || prev_pos[1] > VOXEL_MAX_BOUND ||
                prev_pos[2] < VOXEL_MIN_BOUND || prev_pos[2] > VOXEL_MAX_BOUND)
            {
                LOG(FATAL) << "Error: Previous position (" << prev_pos << ") is out of bounds";
            }
            num_predictions += 1;
            
            pVoxel->expandVoxelBetweenPoints(prev_pos, prev_rot, pred_pos, pred_rot);
        }
        else
            LOG(FATAL) << "Error: Invalid index";
        
        // Except the last prediction
        int orig_frustum_pos = 0;
        if(i < predictions.size() - lookAhead)
        {
            orig_frustum_pos = pVoxel->pointInFrustumVoxels(Vector3f(data[i+lookAhead][1], data[i+lookAhead][2], data[i+lookAhead][3]));
            if(orig_frustum_pos == INSIDE || orig_frustum_pos == INTERSECT)
                correct_predicitions += 1;
            else
                incorrect_predictions += 1;
        }

        // LOG(INFO) << "Frustum Position: " << orig_frustum_pos;
        // LOG(INFO) << "Correct: " << correct_predicitions << ", Incorrect: " << incorrect_predictions << ", Total: " << num_predictions;

        pVoxel->clearVoxels2();
    }

    float accuracy = (float)correct_predicitions / (float)num_predictions;
	LOG(INFO) << "Correct: " << correct_predicitions << ", Incorrect: " << incorrect_predictions << ", Total: " << num_predictions;
    LOG(INFO) << "Frustum Center - Prediction Accuracy: " << accuracy;

	delete pVoxel;
	pVoxel = NULL;

    /////////////////////////////////////
	//// Frustrum Prediction Accuracy ////
	/////////////////////////////////////

	minBound[0] = minBound[1] = minBound[2] = VOXEL_MIN_BOUND;
	maxBound[0] = maxBound[1] = maxBound[2] = VOXEL_MAX_BOUND;
    pVoxel = new Voxel(minBound, maxBound, VOXEL_SIZE);

	assert(predictions.size() == data.size());

    correct_predicitions = 0, incorrect_predictions = 0, num_predictions = 0;
    for(int i = 0; i < predictions.size(); i++)
    {
        sw.Restart();
        LOG(INFO) << "Processing frame: " << i;
        int expandedVoxelsInFrustum = 0;
        int totalVoxelsInFrustum = 0;;

        if(i < 2)
        {
            // Skip the first two entries of predictions.
            continue;
        }
        else if(i >= 2 && i < predictions.size() - lookAhead)
        {
            // Except the last prediction
			Vector3f prev_pos = Vector3f(data[i][1], data[i][2], data[i][3]);
            Vector4f prev_quat = Vector4f(data[i][4], data[i][5], data[i][6], data[i][7]);
            Vector3f pred_pos = Vector3f(predictions[i][1], predictions[i][2], predictions[i][3]);
            Vector4f pred_quat = Vector4f(predictions[i][4], predictions[i][5], predictions[i][6], predictions[i][7]);
            Vector3f orig_pos = Vector3f(data[i+lookAhead][1], data[i+lookAhead][2], data[i+lookAhead][3]);
            Vector4f orig_quat = Vector4f(data[i+lookAhead][4], data[i+lookAhead][5], data[i+lookAhead][6], data[i+lookAhead][7]);

            // check point is within bounds
            if (prev_pos[0] < VOXEL_MIN_BOUND || prev_pos[0] > VOXEL_MAX_BOUND ||
                prev_pos[1] < VOXEL_MIN_BOUND || prev_pos[1] > VOXEL_MAX_BOUND ||
                prev_pos[2] < VOXEL_MIN_BOUND || prev_pos[2] > VOXEL_MAX_BOUND)
            {
                LOG(FATAL) << "Error: Previous Frustum position (" << prev_pos << ") is out of bounds";
            }

			Frustum prev_frustum, pred_frustum, orig_frustum;
            calc_frustum_from_pos_rot2(prev_pos, Vector3f(0, 0, 0), prev_quat, camInt, prev_frustum);
			calc_frustum_from_pos_rot2(pred_pos, Vector3f(0, 0, 0), pred_quat, camInt, pred_frustum);
            calc_frustum_from_pos_rot2(orig_pos, Vector3f(0, 0, 0), orig_quat, camInt, orig_frustum);
			
			Vector3f prev_frustum_min_corner, prev_frustum_max_corner, pred_frustum_min_corner, pred_frustum_max_corner;

            Vector3f aa, bb;
            prev_frustum.getFrustumCubeEnclosedCorners(prev_frustum_min_corner, prev_frustum_max_corner);
            pred_frustum.getFrustumCubeEnclosedCorners(pred_frustum_min_corner, pred_frustum_max_corner);
            aa = prev_frustum_min_corner;
            bb = prev_frustum_max_corner;

			calc_max_min_corners_frustum(prev_frustum, prev_frustum_min_corner, prev_frustum_max_corner);
			calc_max_min_corners_frustum(pred_frustum, pred_frustum_min_corner, pred_frustum_max_corner);

			Vector3f cube_min_corner = prev_frustum_min_corner.cwiseMin(pred_frustum_min_corner);
			Vector3f cube_max_corner = prev_frustum_max_corner.cwiseMax(pred_frustum_max_corner);

			pVoxel->expandVoxelInFrustum2(cube_min_corner, cube_max_corner, &orig_frustum, expandedVoxelsInFrustum, totalVoxelsInFrustum);
		}
		
		// LOG(INFO) << "Prediction Time 1 (in ms): " << sw.ElapsedMs();

		// sw.Restart();
		
        correct_predicitions += expandedVoxelsInFrustum;
        incorrect_predictions += totalVoxelsInFrustum - expandedVoxelsInFrustum;
        num_predictions += totalVoxelsInFrustum;

		// LOG(INFO) << "Frustum Position: " << orig_frustum_pos;
		LOG(INFO) << "Correct: " << correct_predicitions << ", Incorrect: " << incorrect_predictions << ", Total: " << num_predictions << ", Acc: " << (float)correct_predicitions / (float)num_predictions;
		
		pVoxel->clearVoxels2();
		LOG(INFO) << "Prediction Time (in ms): " << sw.ElapsedMs();
	}
	LOG(INFO) << "Correct: " << correct_predicitions << ", Incorrect: " << incorrect_predictions << ", Total: " << num_predictions;
	accuracy = (float)correct_predicitions / (float)num_predictions;
	LOG(INFO) << "Frustum - Prediction Accuracy: " << accuracy;
}

int main(int argc, char** argv)
{
    gflags::SetUsageMessage("Usage: kalman_test_new [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    absl::SetMinLogLevel((absl::LogSeverityAtLeast)0);

    string config_file = "/home/lei/rajrup/KinectStream/Multiview/config/panoptic.json";
    parse_config(config_file);
    frustumPrediction();
    return 0;
}