#include "kalman.h"
#include "voxel.h"

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <boost/tokenizer.hpp>
#include <boost/filesystem.hpp>

#define VOXEL_SIZE 0.05F		// To Do: Rajrup: Not working for 0.01F
#define VOXEL_MIN_BOUND -4.0F
#define VOXEL_MAX_BOUND 4.0F

#define FORMAT(items) \
    static_cast<std::ostringstream &>((std::ostringstream() << std::string() << items)).str()

using namespace std;
using namespace Eigen;
using namespace boost;
namespace fs = boost::filesystem;

VVR_Kalman::VVR_Kalman()
{
    num_states = 12;
    num_measurements = 6;

    float delta_t = 0.1f;
    float sigmaQ = 1.0e-2;
    float sigmaR = 1.0;

    cv::KalmanFilter kf(num_states, num_measurements, 0, CV_32F);
    kfs.push_back(kf);

    SetTransitionMatrix(delta_t);
    SetMeasurementMatrix();
    setIdentity(kfs[0].processNoiseCov, cv::Scalar::all(sigmaQ));
    setIdentity(kfs[0].measurementNoiseCov, cv::Scalar::all(sigmaR));
    cv::setIdentity(kfs[0].errorCovPost, cv::Scalar::all(1));

    bInitialized = false;

    return;
}

/**
 * @brief Construct Kalman Filter with the given parameters.
 * 
 * @param num_states : Number of states. 
 * If the state is (x, y, z, roll, pitch, yaw), then the number of states is 12 (x, x', y, y', z, z', roll, roll', pitch, pitch', yaw, yaw').
 * If the state is (x, y, z, q_x, q_y, q_z, q_w), then the number of states is 14 (x, x', y, y', z, z', q_x, q_x', q_y, q_y', q_z, q_z', q_w, q_w').
 * @param num_measurements : Number of measurements.
 * If the measurement is (x, y, z, roll, pitch, yaw), then the number of measurements is 6.
 * If the measurement is (x, y, z, q_x, q_y, q_z, q_w), then the number of measurements is 7.
 * @param sigmaQ : Process noise covariance. Default value is 1.0e-2
 * @param sigmaR : Measurement noise covariance. Default value is 1.0
 * @param delta_t : Time step. Default value is 0.1 seconds (100 ms)
 */
VVR_Kalman::VVR_Kalman(int num_states, int num_measurements, float sigmaQ, float sigmaR, float delta_t) : 
                        num_states(num_states), num_measurements(num_measurements)

{
    if(num_states != 12 && num_states != 14)
        LOG(FATAL) << "Invalid number of states: " << num_states;

    if(num_measurements != 6 && num_measurements != 7)
        LOG(FATAL) << "Invalid number of measurements: " << num_measurements;

    cv::KalmanFilter kf(num_states, num_measurements, 0, CV_32F);
    kfs.push_back(kf);

    SetTransitionMatrix(delta_t);
    SetMeasurementMatrix();
    setIdentity(kfs[0].processNoiseCov, cv::Scalar::all(sigmaQ));
    setIdentity(kfs[0].measurementNoiseCov, cv::Scalar::all(sigmaR));
    cv::setIdentity(kfs[0].errorCovPost, cv::Scalar::all(1));

    bInitialized = false;

    return;
}

void VVR_Kalman::SetMeasurementMatrix()
{
    kfs[0].measurementMatrix = cv::Mat::zeros(num_measurements, num_states, CV_32F);
    for (int i = 0; i < num_measurements; i++)
        kfs[0].measurementMatrix.at<float>(i, i*2) = 1.0f;
    // LOG(INFO) << "Measurement Matrix: " << kfs[0].measurementMatrix;
}

void VVR_Kalman::SetTransitionMatrix(float delta_t)
{
    kfs[0].transitionMatrix = cv::Mat::zeros(num_states, num_states, CV_32F);
    for (int i = 0; i < num_states; i++)
    {
        kfs[0].transitionMatrix.at<float>(i, i) = 1.0f;
        if (i % 2 == 0)
            kfs[0].transitionMatrix.at<float>(i, i + 1) = delta_t;
    }
}

void VVR_Kalman::Init_Model(float x, float y, float z, float roll, float pitch, float yaw, float delta_t)
{
    if(num_states != 12)
        LOG(FATAL) << "Invalid number of states: " << num_states;

    cv::Mat state(num_states, 1, CV_32F);
    state.at<float>(0) = x;
    state.at<float>(1) = 0;
    state.at<float>(2) = y;
    state.at<float>(3) = 0;
    state.at<float>(4) = z;
    state.at<float>(5) = 0;
    state.at<float>(6) = roll;
    state.at<float>(7) = 0;
    state.at<float>(8) = pitch;
    state.at<float>(9) = 0;
    state.at<float>(10) = yaw;
    state.at<float>(11) = 0;
    SetTransitionMatrix(delta_t);
    kfs[0].statePost = state;

    bInitialized = true;

    return;
}

void VVR_Kalman::Init_Model(float x, float y, float z, float q_x, float q_y, float q_z, float q_w, float delta_t)
{
    if(num_states != 14)
        LOG(FATAL) << "Invalid number of states: " << num_states;

    cv::Mat state(num_states, 1, CV_32F);
    state.at<float>(0) = x;
    state.at<float>(1) = 0;
    state.at<float>(2) = y;
    state.at<float>(3) = 0;
    state.at<float>(4) = z;
    state.at<float>(5) = 0;
    state.at<float>(6) = q_x;
    state.at<float>(7) = 0;
    state.at<float>(8) = q_y;
    state.at<float>(9) = 0;
    state.at<float>(10) = q_z;
    state.at<float>(11) = 0;
    state.at<float>(12) = q_w;
    state.at<float>(13) = 0;
    SetTransitionMatrix(delta_t);
    kfs[0].statePost = state;

    bInitialized = true;

    return;
}

void VVR_Kalman::Predict(vector<vector<float>> &state, const int lookAhead)
{
    auto& kf = kfs[0];
    cv::Mat kf_pred;
    for (int i = 0; i < lookAhead; i++)
    {
        kf_pred = kf.predict();
        for (int j = 0; j < num_states; j += 2)
            state[i].push_back(kf_pred.at<float>(j));
    }
}

/**
 * @brief Predicts postion and Euler rotation angles using Kalman Filter.
 * @deprecated Use this function if prediction is done on Euler rotation, rather than quarternion.
 * @param x 
 * @param y 
 * @param z 
 * @param roll 
 * @param pitch 
 * @param yaw 
 * @param delta_t 
 * @param lookAhead 
 * @return std::vector<float> Predicted position and Euler rotation angles.
 */
std::vector<float> VVR_Kalman::Predict_Update(float x, float y, float z, float roll, float pitch, float yaw, float delta_t, const int lookAhead)
{
    if(!bInitialized)
        LOG(FATAL) << "Kalman filter not initialized";

    if(num_states != 12)
        LOG(FATAL) << "Invalid number of states: " << num_states;

    vector<vector<float>> prediction(lookAhead);

    // auto& kf = kfs[0];
    // auto& kf_pred = kf.predict();
    
    // LOG(INFO) << "Predicted state size: " << kf_pred.rows << ", " << kf_pred.cols;
    // LOG(INFO) << "Predicted state: " << kf_pred;
    // for (int i = 0; i < num_states; i += 2)
    //     prediction.push_back(kf_pred.at<float>(i));

    Predict(prediction, lookAhead);

    // LOG(INFO) << "Prediction:------------------------";
    // for(int i = 0; i < prediction.size(); i++)
    // {
    //     for(int j = 0; j < prediction[i].size(); j++)
    //         LOG(INFO) << prediction[i][j] << ", ";
    //     LOG(INFO);
    // }

    cv::Mat update(num_measurements, 1, CV_32F);
    update.at<float>(0) = x;
    update.at<float>(1) = y;
    update.at<float>(2) = z;
    update.at<float>(3) = roll;
    update.at<float>(4) = pitch;
    update.at<float>(5) = yaw;
    kfs[0].correct(update);

    // To Do: Set transition matrix after or before the correction step?
    SetTransitionMatrix(delta_t);

    return prediction[prediction.size() - 1];
}

/**
 * @brief Predicts postion and quarternion using Kalman Filter.
 * Our pipeline uses quarternion for rotation.
 * @param x 
 * @param y 
 * @param z 
 * @param q_x 
 * @param q_y 
 * @param q_z 
 * @param q_w 
 * @param delta_t 
 * @param lookAhead 
 * @return vector<float> Predicted position and quarternion.
 */
vector<float> VVR_Kalman::Predict_Update(float x, float y, float z, float q_x, float q_y, float q_z, float q_w, float delta_t, const int lookAhead)
{
    if(!bInitialized)
        LOG(FATAL) << "Kalman filter not initialized";

    if(num_states != 14)
        LOG(FATAL) << "Invalid number of states: " << num_states;

    vector<vector<float>> prediction(lookAhead);

    // auto& kf = kfs[0];
    // auto& kf_pred = kf.predict();
    
    // LOG(INFO) << "Predicted state size: " << kf_pred.rows << ", " << kf_pred.cols;
    // LOG(INFO) << "Predicted state: " << kf_pred;
    // for (int i = 0; i < num_states; i += 2)
    //     prediction.push_back(kf_pred.at<float>(i));

    Predict(prediction, lookAhead);

    // LOG(INFO) << "Prediction:------------------------";
    // for(int i = 0; i < prediction.size(); i++)
    // {
    //     for(int j = 0; j < prediction[i].size(); j++)
    //         LOG(INFO) << prediction[i][j] << ", ";
    //     LOG(INFO);
    // }

    cv::Mat update(num_measurements, 1, CV_32F);
    update.at<float>(0) = x;
    update.at<float>(1) = y;
    update.at<float>(2) = z;
    update.at<float>(3) = q_x;
    update.at<float>(4) = q_y;
    update.at<float>(5) = q_z;
    update.at<float>(6) = q_w;
    kfs[0].correct(update);

    // To Do: Set transition matrix after or before the correction step?
    SetTransitionMatrix(delta_t);

    return prediction[prediction.size() - 1];

}


KalmanPredictor::KalmanPredictor() : m_kalman()
{
    m_bInitialized = false;
    m_currFrameID = 0;
    m_prevFrameID = 0;
    m_predictCount = 0;
    m_avgDeltaTime = 0;
}

/**
 * @brief This constructor is generic. Use it for any number of states and measurements.
 * We use this for predicting position and quarternion.
 * @param num_states : Number of states. 
 * If the state is (x, y, z, roll, pitch, yaw), then the number of states is 12 (x, x', y, y', z, z', roll, roll', pitch, pitch', yaw, yaw').
 * If the state is (x, y, z, q_x, q_y, q_z, q_w), then the number of states is 14 (x, x', y, y', z, z', q_x, q_x', q_y, q_y', q_z, q_z', q_w, q_w').
 * @param num_measurements : Number of measurements.
 * If the measurement is (x, y, z, roll, pitch, yaw), then the number of measurements is 6.
 * If the measurement is (x, y, z, q_x, q_y, q_z, q_w), then the number of measurements is 7.
* @param sigmaQ : Process noise covariance. Default value is 1.0e-2
 * @param sigmaR : Measurement noise covariance. Default value is 1.0
 * @param delta_t : Time step. Default value is 0.1 seconds (100 ms)
 */
KalmanPredictor::KalmanPredictor(int num_states, int num_measurements, float sigmaQ, float sigmaR, float delta_t) : 
                                    m_kalman(num_states, num_measurements, sigmaQ, sigmaR, delta_t)
{
    m_bInitialized = false;
    m_currFrameID = 0;
    m_prevFrameID = 0;
    m_predictCount = 0;
    m_avgDeltaTime = 0;
}

/**
 * @brief This constructor is generic. Use it for any number of states and measurements.
 * We use this for predicting position and quarternion.
 * @param num_states : Number of states. 
 * If the state is (x, y, z, roll, pitch, yaw), then the number of states is 12 (x, x', y, y', z, z', roll, roll', pitch, pitch', yaw, yaw').
 * If the state is (x, y, z, q_x, q_y, q_z, q_w), then the number of states is 14 (x, x', y, y', z, z', q_x, q_x', q_y, q_y', q_z, q_z', q_w, q_w').
 * @param num_measurements : Number of measurements.
 * If the measurement is (x, y, z, roll, pitch, yaw), then the number of measurements is 6.
 * If the measurement is (x, y, z, q_x, q_y, q_z, q_w), then the number of measurements is 7.
* @param sigmaQ : Process noise covariance. Default value is 1.0e-2
 * @param sigmaR : Measurement noise covariance. Default value is 1.0
 * @param delta_t : Time step i.e. time interval between 2 frames. E.g. 30 fps means delta_t = 0.033s. Default value is 0.1 seconds (100 ms). 
 */
KalmanPredictor2::KalmanPredictor2(int num_states, int num_measurements, float sigmaQ, float sigmaR, float delta_t):
                                    m_kalman(num_states, num_measurements, sigmaQ, sigmaR, delta_t)
{
    m_bInitialized = false;
    m_delta_t = delta_t;
}

KalmanPredictor::~KalmanPredictor()
{
    m_mapPredData.clear();
}

KalmanPredictor2::~KalmanPredictor2()
{
}

/**
 * @brief Predicts the user pos, rot and frustum for the given frameID, based on a previous real user data.
 * @deprecated Use this function if prediction is done on Euler rotation, rather than quarternion. 
 * @param frameID 
 * @param lookAheadTime in ms. 0 ms means run Kalman Filter for next frame.
 * @param orig Original user trace
 * @param pred Predicted user trace
 */
bool KalmanPredictor::predictData(uint32_t frameID, uint32_t lookAheadTime, const UserData &orig, UserData &pred)
{
    // Data playaback hasn't started yet
    if(orig.frameID <= 0)
    {
        pred = orig;
        pred.frameID = frameID;
        return true;
    }

    if(m_mapPredData.find(frameID) != m_mapPredData.end())
        LOG(FATAL) << "Kalman Filter called for duplicate frame ID: " << frameID;

    if(m_predictCount > 0 && m_prevFrameID != frameID - 1 && m_mapPredData.find(m_prevFrameID) == m_mapPredData.end()) // Check from second frame onwards
        LOG(FATAL) << "Kalman Filter called for non-sequential frame ID: " << frameID;

    // Ignore the timestamp in original data
    Vector3f orig_pos = orig.pos;
    Vector3f orig_rot = orig.rot;
    uint32_t orig_frameID = orig.frameID;
    
    uint32_t ts;
    if(!m_bInitialized)
    {
        // Timer starts
        m_timer.Restart();

        // Initialize Kalman Filter at 30 fps. This is the frame rate at which the user trace is recorded.
        m_kalman.Init_Model(orig_pos[0], orig_pos[1], orig_pos[2], orig_rot[0], orig_rot[1], orig_rot[2], INTER_FRAME_DELAY/1000.0f);
        pred = orig;
        pred.frameID = frameID;
        pred.ts_pred = INTER_FRAME_DELAY;
        
        m_prevFrameID = 0;
        m_predictCount = 0;
        m_bInitialized = true;

        return true;
    }

    ts = m_timer.ElapsedMs();

    double dt = ts;
    if(m_predictCount > 0)
    {
        dt = ts - m_mapPredData[m_prevFrameID].ts_pred;
        m_avgDeltaTime = ceil((m_avgDeltaTime * m_predictCount + dt) / (m_predictCount + 1)); // Round off to ms
    }
    else
    {
        // First prediction
        m_avgDeltaTime = ceil(dt);
    }
    
    int lookAhead = m_avgDeltaTime < 1 ? 1 : ceil(lookAheadTime / m_avgDeltaTime);
    QCHECK(lookAhead > 0) << "Look ahead time is too small";

    vector<float> prediction = m_kalman.Predict_Update(orig_pos[0], orig_pos[1], orig_pos[2], orig_rot[0], orig_rot[1], orig_rot[2], dt/1000.0f, lookAhead);

    pred.frameID = frameID;
    pred.ts_orig = orig.ts_orig;
    pred.ts_pred = ts;
    pred.pos = Vector3f(prediction[0], prediction[1], prediction[2]);
    pred.rot = Vector3f(prediction[3], prediction[4], prediction[5]);
    calc_frustum_from_pos_rot2(pred.pos, pred.rot, m_camInt, pred.frustum);

    m_prevFrameID = frameID;
    m_mapPredData[frameID] = pred;
    m_predictCount++;

    return true;
}

/**
 * @brief Predict position and quaternion using Kalman Filter.
 * This function is used now.
 * @param frameID 
 * @param lookAheadTime 
 * @param orig 
 * @param pred 
 */
bool KalmanPredictor::predictDataUsingQuat(uint32_t frameID, uint32_t lookAheadTime, const UserData &orig, UserData &pred)
{
    // Data playaback hasn't started yet
    if(orig.frameID <= 0)
    {
        pred = orig;
        pred.frameID = frameID;
        return true;
    }

    if(m_mapPredData.find(frameID) != m_mapPredData.end())
        LOG(FATAL) << "Kalman Filter called for duplicate frame ID: " << frameID;

    if(m_predictCount > 0 && m_prevFrameID != frameID - 1 && m_mapPredData.find(m_prevFrameID) == m_mapPredData.end()) // Check from second frame onwards
        LOG(FATAL) << "Kalman Filter called for non-sequential frame ID: " << frameID;

    // Ignore the timestamp in original data
    Vector3f orig_pos = orig.pos;
    Vector4f orig_quat = orig.quat;
    uint32_t orig_frameID = orig.frameID;
    
    uint32_t ts;
    if(!m_bInitialized)
    {
        // Timer starts
        m_timer.Restart();

        // Initialize Kalman Filter at 30 fps. This is the frame rate at which the user trace is recorded.
        m_kalman.Init_Model(orig_pos[0], orig_pos[1], orig_pos[2], orig_quat[0], orig_quat[1], orig_quat[2], orig_quat[3], INTER_FRAME_DELAY/1000.0f);
        pred = orig;
        pred.frameID = frameID;
        pred.ts_pred = INTER_FRAME_DELAY;
        
        m_prevFrameID = 0;
        m_predictCount = 0;
        m_bInitialized = true;

        return true;
    }

    ts = m_timer.ElapsedMs();

    double dt = ts;
    if(m_predictCount > 0)
    {
        dt = ts - m_mapPredData[m_prevFrameID].ts_pred;
        m_avgDeltaTime = ceil((m_avgDeltaTime * m_predictCount + dt) / (m_predictCount + 1)); // Round off to ms
    }
    else
    {
        // First prediction
        m_avgDeltaTime = ceil(dt);
    }
    
    int lookAhead = m_avgDeltaTime < 1 ? 1 : ceil(lookAheadTime / m_avgDeltaTime);
    QCHECK(lookAhead > 0) << "Look ahead time is too small";

    vector<float> prediction = m_kalman.Predict_Update(orig_pos[0], orig_pos[1], orig_pos[2], orig_quat[0], orig_quat[1], orig_quat[2], orig_quat[3], dt/1000.0f, lookAhead);

    pred.frameID = frameID;
    pred.ts_orig = orig.ts_orig;
    pred.ts_pred = ts;
    pred.pos = Vector3f(prediction[0], prediction[1], prediction[2]);
    pred.quat = Vector4f(prediction[3], prediction[4], prediction[5], prediction[6]);
    Vector3f dummy_rot = Vector3f(0.0, 0.0, 0.0);
    calc_frustum_from_pos_rot2(pred.pos, dummy_rot, pred.quat, m_camInt, pred.frustum);

    m_prevFrameID = frameID;
    m_mapPredData[frameID] = pred;
    m_predictCount++;

    return true;
}

/**
 * @brief Predict position and quaternion using Kalman Filter.
 * This function is used now.
 * @param frameID 
 * @param lookAheadTime 
 * @param orig 
 * @param pred 
 */
bool KalmanPredictor2::predictDataUsingQuat(uint32_t frameID, uint32_t lookAhead, const Frustum &orig_f, Frustum &pred_f)
{
    if(!m_bInitialized)
    {
        // Initialize Kalman Filter at 30 fps. This is the frame rate at which the user trace is recorded.
        m_kalman.Init_Model(orig_f.pos(0), orig_f.pos(1), orig_f.pos(2), orig_f.quat(0), orig_f.quat(1), orig_f.quat(2), orig_f.quat(3), m_delta_t);
        m_bInitialized = true;
    }

    // vector<float> prediction = m_kalman.Predict_Update(orig_f.pos(0), orig_f.pos(1), orig_f.pos(2), orig_f.quat(0), orig_f.quat(1), orig_f.quat(2), orig_f.quat(3), m_delta_t, lookAhead);

    Vector3f orig_pos = orig_f.pos;
    Matrix3f rotMat;
	rotMat << 1, 0, 0,
			  0, -1, 0,
			  0, 0, 1;
    orig_pos = rotMat * orig_pos;
    Vector4f orig_quat = orig_f.quat;

    vector<float> prediction = m_kalman.Predict_Update(orig_pos(0), orig_pos(1), orig_pos(2), orig_quat(0), orig_quat(1), orig_quat(2), orig_quat(3), m_delta_t, lookAhead);

    pred_f.frameID = frameID;
    CamInt camInt;
    camInt.angle = orig_f.angle;
    camInt.ratio = orig_f.ratio;
    camInt.nearD = orig_f.nearD;
    camInt.farD = orig_f.farD;

    Vector3f pred_pos = Vector3f(prediction[0], prediction[1], prediction[2]);
    Vector4f pred_quat = Vector4f(prediction[3], prediction[4], prediction[5], prediction[6]);
    Vector3f dummy_rot = Vector3f(0.0, 0.0, 0.0);
    calc_frustum_from_pos_rot2(pred_pos, dummy_rot, pred_quat, camInt, pred_f);
    return true;
}

/**
 * @brief Predict position and quaternion using Kalman Filter.
 * This function is used now.
 * @param frameID 
 * @param lookAheadTime 
 * @param orig 
 * @param pred 
 */
bool KalmanPredictor2::predictDataUsingQuat_test(uint32_t frameID, uint32_t lookAhead, const Frustum &orig_f, Frustum &pred_f, const Frustum &actual_f)
{
    if(!m_bInitialized)
    {
        // Initialize Kalman Filter at 30 fps. This is the frame rate at which the user trace is recorded.
        m_kalman.Init_Model(orig_f.pos(0), orig_f.pos(1), orig_f.pos(2), orig_f.quat(0), orig_f.quat(1), orig_f.quat(2), orig_f.quat(3), m_delta_t);
        m_bInitialized = true;
    }
    Vector3f orig_pos = orig_f.pos;
    Matrix3f rotMat;
	rotMat << 1, 0, 0,
			  0, -1, 0,
			  0, 0, 1;
    orig_pos = rotMat * orig_pos;
    Vector4f orig_quat = orig_f.quat;

    vector<float> prediction = m_kalman.Predict_Update(orig_pos(0), orig_pos(1), orig_pos(2), orig_quat(0), orig_quat(1), orig_quat(2), orig_quat(3), m_delta_t, lookAhead);

    pred_f.frameID = frameID;
    CamInt camInt;
    camInt.angle = orig_f.angle;
    camInt.ratio = orig_f.ratio;
    camInt.nearD = orig_f.nearD;
    camInt.farD = orig_f.farD;

    Vector3f pred_pos = Vector3f(prediction[0], prediction[1], prediction[2]);
    Vector4f pred_quat = Vector4f(prediction[3], prediction[4], prediction[5], prediction[6]);
    Vector3f dummy_rot = Vector3f(0.0, 0.0, 0.0);
    calc_frustum_from_pos_rot2(pred_pos, dummy_rot, pred_quat, camInt, pred_f);

    // LOG(INFO) << "Actu Frustum - pos: " << actual_f.pos.transpose() << ", quat: " << actual_f.quat.transpose(); 
    // LOG(INFO) << "Pred Frustum - pos: " << pred_f.pos.transpose() << ", quat: " << pred_f.quat.transpose();
    // LOG(INFO) << "Recv Frustum - pos: " << orig_f.pos.transpose() << ", quat: " << orig_f.quat.transpose();
    return true;
}

// int main(int argc, char** argv)
// {
//     //---------- Construct kalman filtering for sensor data

//     vector<string> str_data;
//     vector<vector<float>> data;
//     vector<vector<float>> predictions;
//     // string project_dir = "/data/Dropbox/Project/VolumetricVideo/KinectStream/";
//     string project_dir = "/home/rajrup/Dropbox/Project/VolumetricVideo/KinectStream/";

//     // string kinoptic_data = "160422_haggling1";
//     // string kinoptic_data = "170915_office1";
//     string kinoptic_data = "170915_toddler4";
//     int logID = 2;
//     string user_data_folder = FORMAT(project_dir << "data/user_data/" << kinoptic_data << "/");

//     if(!fs::exists(user_data_folder))
//     {
//         LOG(FATAL) << "User data folder does not exist: " << user_data_folder;
//     }

//     const int NUM_COLS = 7;
//     const int SKIP_ROWS = 1107;

//     fs::path in_file{FORMAT(user_data_folder << "user_log" << logID << ".txt")};
// 	fs::ifstream ifs;
// 	ifs.open(in_file, ios::in);
// 	if (!ifs.is_open())
// 	{
// 		LOG(ERROR) << "Failed to load HMD Camera Log file: " << in_file;
// 		return 0;
// 	}

//     string line;
//     int counter = 0;
//     while(getline(ifs, line))
//     {
//         tokenizer<escaped_list_separator<char> > tk(
//         line, escaped_list_separator<char>("", ",:", ""));
//         for(tokenizer<escaped_list_separator<char> >::iterator i(tk.begin()); i!=tk.end(); ++i) 
//         {
//             str_data.push_back(*i);
//         }
//     }

//     assert (str_data.size() % NUM_COLS == 0);
//     for(int i = 0; i < str_data.size() / NUM_COLS; i++)
//     {
//         vector<float> row;
//         cout << i << " ";
//         for(int j = 0; j < NUM_COLS; j++)
//             row.push_back(stof(str_data[i * NUM_COLS + j]));
        
//         data.push_back(row);
//         data[i][0] = data[i][0] / 1000.0f; // convert to seconds. Note: Rajrup: Some precision is lost here.

//         for(int j = 0; j < NUM_COLS; j++)
//             cout << data[i][j] << " ";
        
//         cout;
//     }
    
//     string pred_data_folder = FORMAT(project_dir << "data/predicted_data/" << kinoptic_data << "/");

//     if(!fs::exists(pred_data_folder))
//     {
//         LOG(INFO) << "Predicted data folder does not exist!";
//         LOG(INFO) << "Creating folder: " << pred_data_folder;
//         fs::create_directory(pred_data_folder);
//     }

//     fs::path out_file{FORMAT(pred_data_folder << "camera_log" << logID << "_kalman.txt")};
// 	fs::ofstream ofs;
// 	ofs.open(out_file, ios::out | ios::trunc);
// 	if (!ofs.is_open())
// 	{
// 		LOG(ERROR) << "Failed to open Kalman Output file: " << out_file;
// 		return 0;
// 	}

//     //////////////////////////////
//     /* Step 1: Kalman filtering */
//     //////////////////////////////
//     VVR_Kalman *kalman = new VVR_Kalman();

//     /**
//      * @brief data[0] is dummy.
//      * @brief data[1] is the first measurement. Initializes the model.
//      * @brief Similary, predictions[0] is dummy.
//      * @brief predictions[1] is the first measurement.
//      * @brief predictions[2] is the first prediction.
//      */
//     for(int i = 0; i < data.size(); i++)
//     {
//         if(i == 0)
//         {
//             ofs << int(data[i][0]*1000.0f) << ": " << std::fixed << setprecision(7) << data[i][1] << "," << data[i][2] << "," << data[i][3] << "," << data[i][4] << "," << data[i][5] << "," << data[i][6];
//             predictions.push_back(vector<float>{data[i][0]*1000.0f, data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6]});
//         }
//         else if(i == 1)
//         {
//             // Rajrup: Since some precision is lost in the conversion to seconds, we need to fix this later.
//             kalman->Init_Model(data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6], data[i][0]);
//             ofs << int(data[i][0]*1000.0f) << ": " << std::fixed << setprecision(7) << data[i][1] << "," << data[i][2] << "," << data[i][3] << "," << data[i][4] << "," << data[i][5] << "," << data[i][6];
//             predictions.push_back(vector<float>{data[i][0]*1000.0f, data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6]});
//         }
//         else if(i >= 2)
//         {
//             vector<float> prediction = kalman->Predict_Update(data[i][1], data[i][2], data[i][3], data[i][4], data[i][5], data[i][6], data[i][0] - data[i-1][0]);
//             ofs << int(data[i][0]*1000.0f) << ": " << std::fixed << setprecision(7) << prediction[0] << "," << prediction[1] << "," << prediction[2] << "," << prediction[3] << "," << prediction[4] << "," << prediction[5];
//             predictions.push_back(vector<float>{data[i][0]*1000.0f, prediction[0], prediction[1], prediction[2], prediction[3], prediction[4], prediction[5]});

//             if(i < data.size() - 1)
//                 ofs;
//         }
//         else
//         {
//             LOG(ERROR) << "Error: Invalid index";
//             exit(0);
//         }
//     }

//     //////////////////////////////
//     /* Step 2: Voxel Prediction */
//     //////////////////////////////
//     Vector3f minBound, maxBound;
//     minBound[0] = minBound[1] = minBound[2] = VOXEL_MIN_BOUND;
// 	maxBound[0] = maxBound[1] = maxBound[2] = VOXEL_MAX_BOUND;
//     Voxel *pVoxel = new Voxel(minBound, maxBound, VOXEL_SIZE);

//     assert(predictions.size() == data.size());

//     int correct_predicitions = 0, incorrect_predictions = 0, num_predictions = 0;
//     for(int i = 0; i < predictions.size(); i++)
//     {
//         if(i < 2)
//         {
//             // Skip the first two entries of predictions.
//             continue;
//         }
//         else if(i >= 2)
//         {
//             Vector3f orig_pos = Vector3f(data[i][1], data[i][2], data[i][3]);
//             Vector3f orig_rot = Vector3f(data[i][4], data[i][5], data[i][6]);
//             Vector3f pred_pos = Vector3f(predictions[i][1], predictions[i][2], predictions[i][3]);
//             Vector3f pred_rot = Vector3f(predictions[i][4], predictions[i][5], predictions[i][6]);

//             // check point is within bounds
//             if (orig_pos[0] < VOXEL_MIN_BOUND || orig_pos[0] > VOXEL_MAX_BOUND ||
//                 orig_pos[1] < VOXEL_MIN_BOUND || orig_pos[1] > VOXEL_MAX_BOUND ||
//                 orig_pos[2] < VOXEL_MIN_BOUND || orig_pos[2] > VOXEL_MAX_BOUND)
//             {
//                 LOG(ERROR) << "Error: Original position (" << orig_pos << ") is out of bounds";
//                 exit(0);
//             }
//             num_predictions += 1;
            
//             pVoxel->expandVoxelBetweenPoints(orig_pos, orig_rot, pred_pos, pred_rot);
//         }
//         else
//         {
//             LOG(ERROR) << "Error: Invalid index";
//             exit(0);
//         }
        
//         // Except the last prediction
//         int frustum_pos = 0;
//         if(i < predictions.size() - 1)
//         {
//             frustum_pos = pVoxel->pointInFrustumVoxels(Vector3f(data[i+1][1], data[i+1][2], data[i+1][3]));
//             if(frustum_pos == INSIDE || frustum_pos == INTERSECT)
//                 correct_predicitions += 1;
//             else
//                 incorrect_predictions += 1;
//         }

//         LOG(INFO) << "Frustum Position: " << frustum_pos;
//         LOG(INFO) << "Correct: " << correct_predicitions << ", Incorrect: " << incorrect_predictions << ", Total: " << num_predictions;

//         pVoxel->clearVoxels2();
//     }
//     float accuracy = (float)correct_predicitions / (float)num_predictions;
//     LOG(INFO) << "Accuracy: " << accuracy;
// }