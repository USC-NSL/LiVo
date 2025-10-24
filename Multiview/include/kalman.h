#pragma once
#include <chrono>
#include <string>
#include <math.h>
#include <iostream>
#include <unordered_set>
#include <vector>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/core.hpp>
#include "absl/container/flat_hash_map.h"
#include "DataPlayback.h"
#include "consts.h"
#include "frustum.h"

using namespace std;

class VVR_Kalman 
{

public:
    

    VVR_Kalman();
    VVR_Kalman(int num_states, int num_measurements, float sigmaQ = 1.0e-2f, float sigmaR = 1.0f, float delta_t = 0.1f);
    void Init_Model(float x, float y, float z, float roll, float pitch, float yaw, float delta_t);
    void Init_Model(float x, float y, float z, float q_x, float q_y, float q_z, float q_w, float delta_t);
    vector<float> Predict_Update(float x, float y, float z, float roll, float pitch, float yaw, float delta_t, const int lookAhead = 1);
    vector<float> Predict_Update(float x, float y, float z, float q_x, float q_y, float q_z, float q_w, float delta_t, const int lookAhead = 1);

private:
    bool bInitialized;
    vector<cv::KalmanFilter> kfs;
    int num_states;
    int num_measurements;

    void SetTransitionMatrix(float delta_t);
    void SetMeasurementMatrix();
    void Predict(vector<vector<float>> &state, const int lookAhead);
};

class KalmanPredictor
{

public:
    KalmanPredictor();
    KalmanPredictor(int num_states, int num_measurements, float sigmaQ = 1.0e-2f, float sigmaR = 1.0f, float delta_t = 0.1f);
    ~KalmanPredictor();
    bool predictData(uint32_t frameID, uint32_t lookAheadTime, const UserData &orig, UserData &pred);
    bool predictDataUsingQuat(uint32_t frameID, uint32_t lookAheadTime, const UserData &orig, UserData &pred);

private:
    CamInt m_camInt;
    uint32_t m_currFrameID, m_prevFrameID;
    double m_avgDeltaTime;
	double m_predictCount;
	bool m_bInitialized;
	VVR_Kalman m_kalman;
    
    /**
     * @brief Key: frameID of the prediction
     * Value: Predicted data
     * Predicted data format: 
     * frameID = original frameID
     * ts_orig = original timestamp
     * ts_pred = predicted timestamp
     * pos = predicted position
     * rot = predicted rotation
     */
    absl::flat_hash_map<uint32_t, UserData> m_mapPredData;

	StopWatch m_timer;
};

class KalmanPredictor2
{
public:
    KalmanPredictor2(int num_states, int num_measurements, float sigmaQ = 1.0e-2f, float sigmaR = 1.0f, float delta_t = 0.1f);
    ~KalmanPredictor2();
    bool predictDataUsingQuat(uint32_t frameID, uint32_t lookAhead, const Frustum &orig, Frustum &pred);
    bool predictDataUsingQuat_test(uint32_t frameID, uint32_t lookAhead, const Frustum &orig_f, Frustum &pred_f, const Frustum &actual_f);

private:
	bool m_bInitialized;
    float m_delta_t;
	VVR_Kalman m_kalman;
};
