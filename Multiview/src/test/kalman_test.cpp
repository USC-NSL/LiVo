#include "kalman.h"
#include "voxel.h"
#include "timer.h"
#include "test/utils.h"
#include <cmath>
#include <boost/filesystem/fstream.hpp>

typedef vector<vector<float>> Data;

#define VOXEL_SIZE 0.05F		// To Do: Rajrup: Not working for 0.01F
#define VOXEL_MIN_BOUND -4.0F
#define VOXEL_MAX_BOUND 4.0F
#define LOOH_AHEAD_TIME 300.0f       // Kalman window size 200 ms

namespace fs = boost::filesystem;

bool read_frustum_data(const string &user_data_folder, const int logID, Data &data_frustum_pos, CamInt &camInt, vector<Frustum> &frustum_from_pos)
{
	Data data_frustum_planes, data_frustum_corners;

	// Read camera position (position, rotation, quaternion, forward vector, up vector)
    string file_name = FORMAT(user_data_folder << "camera_log" << logID << ".txt");
    if(!read_user_data(file_name, data_frustum_pos))
    {
        LOG(ERROR) << "Error: Could not read file: " << file_name;
        return false;
    }

    // // Read equations of frustum planes (left, right, down (bottom), up (top), near, far)
    // file_name = FORMAT(user_data_folder << "frustum_planes" << logID << ".txt");
    // read_user_data(file_name, data_frustum_planes);

    // // Read frustum corners (near corners, far corners)
    // file_name = FORMAT(user_data_folder << "frustum_corners" << logID << ".txt");
    // read_user_data(file_name, data_frustum_corners);

    // Read camera intrinsics (angle, ratio, nearD, farD)
    file_name = FORMAT(user_data_folder << "frustum_details" << logID << ".txt");
    if(!read_frustum_details(file_name, camInt))
    {
        LOG(ERROR) << "Error: Could not read file: " << file_name;
        return false;
    }

    // vector<CamView> cam_view;
    // parse_camera_pos(data_frustum_pos, camInt, cam_view, frustum_from_pos);
    
	// // Frustum planes from Unity data
    // vector<Frustum> frustum_from_planes;
    // parse_frustum_planes(data_frustum_planes, camInt, frustum_from_planes);

	// // Frustum corners from Unity data 
    // vector<vector<Vector3f>> frustum_corners;
    // parse_frustum_corners(data_frustum_corners, frustum_corners);

    // //---------- Test frustum
    // vector<CamView> my_cam_view;
    // vector<Frustum> my_frustum_from_pos;
    // calc_frustum_vec(frustum_from_pos, camInt, cam_view, my_cam_view, my_frustum_from_pos);

    return true;
}

/**
 * @brief Calculates a frustum from position and rotation
 * 
 * @param pos 
 * @param rot in angles
 * @param camInt 
 * @param frustum 
 */
void calc_frustum_from_pos_rot(const Vector3f &pos, const Vector3f &rot, const CamInt &camInt, Frustum &frustum)
{
	frustum.setCamInternals(camInt);
	CamView camView;

	Matrix4f R, T;

	Vector4f X = Vector4f::UnitX(); // right
	Vector4f Y = Vector4f::UnitY(); // up
	Vector4f Z = Vector4f::UnitZ(); // forward
	
	camView.p = pos;
	camView.r = rot;
	
	calc_TR(camView.p, camView.r, T, R);
	camView.l = (T * (R * Z)).head(3);
	camView.u = (T * (R * Y)).head(3);

	frustum.setCamPlanes(camView);
}

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

int frustumPrediction()
{
	LOG(INFO) << "Frustrum KF prediction";

	//---------- Construct kalman filtering for sensor data
	Data data; // Timestamp, psotion, rotation, quaternion, forward vector, up vector
	vector<Frustum> frustum_from_pos;
	CamInt camInt;

    Data predictions;
    // string project_dir = "/data/Dropbox/Project/VolumetricVideo/KinectStream/";
    // string project_dir = "/home/rajrup/Dropbox/Project/VolumetricVideo/KinectStream/";
    string project_dir = "/home/lei/rajrup/KinectStream/";

    // string kinoptic_data = "160422_haggling1";
    // string kinoptic_data = "170915_office1";
    // string kinoptic_data = "170915_toddler4";
    // string kinoptic_data = "160906_ian3";
    // string kinoptic_data = "160906_band2";
    string kinoptic_data = "170307_dance5";
    int logID = 0;
    string user_data_folder = FORMAT(project_dir << "data/user_log/2023-03-09/" << kinoptic_data << "/");

    if(!boost::filesystem::exists(user_data_folder))
        LOG(FATAL) << "User data folder does not exist: " << user_data_folder;

	if(!read_frustum_data(user_data_folder, logID, data, camInt, frustum_from_pos))
        LOG(FATAL) << "Error: Could not user data!!";

	LOG(INFO) << "Reading Done";
	
	// ----------------Prediction-------------------------
	string pred_data_folder = FORMAT(project_dir << "data/predicted_data/" << kinoptic_data << "/");

    if(!fs::exists(pred_data_folder))
    {
        LOG(WARNING) << "Predicted data folder does not exist!";
        LOG(INFO) << "Creating folder: " << pred_data_folder;
        fs::create_directory(pred_data_folder);
    }

    fs::path out_file{FORMAT(pred_data_folder << "camera_log" << logID << "_kalman.txt")};
	fs::ofstream ofs;
	ofs.open(out_file, ios::out | ios::trunc);
	if (!ofs.is_open())
	{
		LOG(INFO) << "Failed to open Kalman Output file: " << out_file;
		return 0;
	}

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

	VVR_Kalman *kalman = new VVR_Kalman();
    float kalman_lookahead_t = LOOH_AHEAD_TIME;  // in ms for lookahead window
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

    for(int i = 0; i < data.size(); i++)
    {
        if(i == 0)
        {
            ofs << int(data[i][0]*1000.0f) << ": " << std::fixed << setprecision(7) << data[i][1] << "," << data[i][2] << "," << data[i][3] << "," << data[i][4] << "," << data[i][5] << "," << data[i][6] << endl;
            predictions.push_back(vector<float>{data[i][0]*1000.0f, data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6]});
        }
        else if(i == 1)
        {
            // Rajrup: Since some precision is lost in the conversion to seconds, we need to fix this later.
            kalman->Init_Model(data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6], data[i][0]);
            ofs << int(data[i][0]*1000.0f) << ": " << std::fixed << setprecision(7) << data[i][1] << "," << data[i][2] << "," << data[i][3] << "," << data[i][4] << "," << data[i][5] << "," << data[i][6] << endl;
            predictions.push_back(vector<float>{data[i][0]*1000.0f, data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6]});
        }
        else if(i >= 2)
        {
            vector<float> prediction = kalman->Predict_Update(data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6], data[i][0] - data[i-1][0], lookAhead);
            ofs << int(data[i][0]*1000.0f) << ": " << std::fixed << setprecision(7) << prediction[0] << "," << prediction[1] << "," << prediction[2] << "," << prediction[3] << "," << prediction[4] << "," << prediction[5];
            predictions.push_back(vector<float>{data[i][0]*1000.0f, prediction[0], prediction[1], prediction[2], prediction[3], prediction[4], prediction[5]});

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
            Vector3f prev_rot = Vector3f(data[i][4], data[i][5], data[i][6]);
            Vector3f pred_pos = Vector3f(predictions[i][1], predictions[i][2], predictions[i][3]);
            Vector3f pred_rot = Vector3f(predictions[i][4], predictions[i][5], predictions[i][6]);
            Vector3f orig_pos = Vector3f(data[i+lookAhead][1], data[i+lookAhead][2], data[i+lookAhead][3]);
            Vector3f orig_rot = Vector3f(data[i+lookAhead][4], data[i+lookAhead][5], data[i+lookAhead][6]);

            // check point is within bounds
            if (prev_pos[0] < VOXEL_MIN_BOUND || prev_pos[0] > VOXEL_MAX_BOUND ||
                prev_pos[1] < VOXEL_MIN_BOUND || prev_pos[1] > VOXEL_MAX_BOUND ||
                prev_pos[2] < VOXEL_MIN_BOUND || prev_pos[2] > VOXEL_MAX_BOUND)
            {
                LOG(FATAL) << "Error: Previous Frustum position (" << prev_pos << ") is out of bounds";
            }

			Frustum prev_frustum, pred_frustum, orig_frustum;
			calc_frustum_from_pos_rot(prev_pos, prev_rot, camInt, prev_frustum);
			calc_frustum_from_pos_rot(pred_pos, pred_rot, camInt, pred_frustum);
            calc_frustum_from_pos_rot(orig_pos, orig_rot, camInt, orig_frustum);
			
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
	LOG(INFO) << "Frustum center - Prediction Accuracy: " << accuracy;
    return 1;
}

int main()
{
    frustumPrediction();
    return 0;
}