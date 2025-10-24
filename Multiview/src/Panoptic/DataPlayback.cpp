#include "DataPlayback.h"
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <iostream>
#include "consts.h"

using namespace std;
using namespace Eigen;
using namespace boost;
namespace fs = boost::filesystem;

typedef vector<vector<float>> Data;

/**
 * @brief Reads frustum position, rotation, quaternion, forward vector, up vector from file
 * Trace file format: frame_id, x,y,z, rx,ry,rz, qx,qy,qz,qw, lx,ly,lz, ux,uy,uz
 * Position is in meters, rotation is in Euler degrees
 * @param file_name 
 * @param out  Index 0 - timestamp (in ms), Index 1-3 - position, Index 4-6 - rotation, Index 7-10 - quaternion, Index 11-13 - forward vector, Index 14-16 - up vector
 */
bool read_user_data2(const string &file_name, vector<vector<float>> &out)
{
    fs::path in_file{file_name};
	fs::ifstream ifs;
	ifs.open(in_file, ios::in);
	if (!ifs.is_open())
	{
		LOG(ERROR) << "Failed to load HMD Camera Log file: " << in_file;
		return false;
	}

    string line;
    int row_id = 0;
    while(getline(ifs, line))
    {
        vector<float> temp;
        tokenizer<escaped_list_separator<char> > tk(
        line, escaped_list_separator<char>("", ",:", ""));

		int j = 0;
        for(tokenizer<escaped_list_separator<char> >::iterator i(tk.begin()); i!=tk.end(); ++i)
		{
			temp.push_back(stof(*i));
		}
            
        out.push_back(temp);
        
        // Check if it's a matrix
        if(row_id > 0)
            assert(out[row_id].size() == out[row_id-1].size());
        out[row_id][0] = ceil(out[row_id][0]);					// Round to nearest ms
        row_id++;
    }
    return true;
}

/**
 * @brief Construct Frustum from camera position, rotation, and intrinsics
 * Also construct CamView (lookAt and Up vectors) from camera position and rotation
 * @param data_camera_pos 
 * @param camInt
 * @param cam_view 
 * @param frustum_from_pos 
 */
void parse_camera_pos2(const vector<vector<float>> &data_camera_pos, const CamInt &camInt, vector<CamView> &cam_view, vector<Frustum> &frustum_from_pos)
{
    Frustum frustum;
    frustum.setCamInternals(camInt);
    CamView camView;
    for(auto vec : data_camera_pos)
    {
        camView.p << vec[1], vec[2], vec[3];
        camView.r << ANG2RAD(vec[4]), ANG2RAD(vec[5]), ANG2RAD(vec[6]);
        camView.l << vec[11], vec[12], vec[13];
        camView.u << vec[14], vec[15], vec[16];

        frustum.setCamPlanes(camView);
        cam_view.push_back(camView);
        frustum_from_pos.push_back(frustum);
    }
}

/**
 * @brief Construct translation and rotation matrices from camera position and rotation
 * @deprecated We use quat2rot() function to convert quaternion to rotation matrix and use it. Translation is used from camera position.
 * @param pos 
 * @param angles in radians
 * @param T 
 * @param R 
 */
void calc_TR2(const Vector3f &pos, const Vector3f angles, Matrix4f &T, Matrix4f &R)
{
    T << 1, 0, 0, pos(0),
         0, 1, 0, pos(1),
         0, 0, 1, pos(2),
         0, 0, 0, 1;

    Matrix4f rotMatX;
    rotMatX << 1, 0, 0, 0,
               0, cos(angles(0)), -sin(angles(0)), 0,
               0, sin(angles(0)), cos(angles(0)), 0,
               0, 0, 0, 1;
    Matrix4f rotMatY;
    rotMatY << cos(angles(1)), 0, sin(angles(1)), 0,
               0, 1, 0, 0,
               -sin(angles(1)), 0, cos(angles(1)), 0,
               0, 0, 0, 1;
    Matrix4f rotMatZ;
    rotMatZ << cos(angles(2)), -sin(angles(2)), 0, 0,
               sin(angles(2)), cos(angles(2)), 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;
    R = rotMatY * (rotMatX * rotMatZ);
}

/**
 * @brief Get Camera Instrinsic parameters from Unity data
 * @param data_cam_view 
 * @param cam_view 
 */
bool read_frustum_details2(const string &file_name, CamInt &camInt)
{
    fs::path in_file{file_name};
	fs::ifstream ifs;
	ifs.open(in_file, ios::in);
	if (!ifs.is_open())
	{
		LOG(ERROR) << "Failed to load HMD Frustum Details file: " << in_file;
		return false;
	}

    string line;
    int row_id = 0;
    for(int i=0; i<4; i++)
    {
        string attribute;
        float value;
        ifs >> attribute >> value;
        
        if(attribute == "Angle")
            camInt.angle = value;
        else if(attribute == "Ratio")
            camInt.ratio = value;
        else if(attribute == "NearD")
            camInt.nearD = value;
        else if(attribute == "FarD")
            camInt.farD = value;
        else
            assert(false);
    }

    return true;
}

// Convert quaternion to rotation matrix
Matrix3f quat2rot(Vector4f quat)
{
    float x = quat(0);
    float y = quat(1);
    float z = quat(2);
    float w = quat(3);

    // 1st row
    float r00 = 1 - 2*y*y - 2*z*z;
    float r01 = 2*x*y - 2*z*w;
    float r02 = 2*x*z + 2*y*w;

    // 2nd row
    float r10 = 2*x*y + 2*z*w;
    float r11 = 1 - 2*x*x - 2*z*z;
    float r12 = 2*y*z - 2*x*w;

    // 3rd row
    float r20 = 2*x*z - 2*y*w;
    float r21 = 2*y*z + 2*x*w;
    float r22 = 1 - 2*x*x - 2*y*y;

    Matrix3f R;
    R << r00, r01, r02,
         r10, r11, r12,
         r20, r21, r22;

    return R;
}

/**
 * @brief Calculates a frustum from position and rotation
 * @deprecated Use calc_frustum_from_pos_rot2(const Vector3f &pos, const Vector3f &rot, const Vector4f &quat, const CamInt &camInt, Frustum &frustum) instead
 * @param pos 
 * @param rot in angles
 * @param camInt 
 * @param frustum 
 */
void calc_frustum_from_pos_rot2(const Vector3f &pos, const Vector3f &rot, const CamInt &camInt, Frustum &frustum)
{
	frustum.setCamInternals(camInt);
	CamView camView;

	Matrix4f R, T;

	Vector4f X = Vector4f::UnitX(); // right
	Vector4f Y = Vector4f::UnitY(); // up
	Vector4f Z = Vector4f::UnitZ(); // forward
	
	camView.p = pos;
	camView.r = rot;
	
	calc_TR2(camView.p, camView.r, T, R);
	camView.l = (T * (R * Z)).head(3);
	camView.u = (T * (R * Y)).head(3);

	frustum.setCamPlanes(camView);
}

/**
 * @brief Calculates a frustum from position, rotation and quaternion.
 * Preferred function to calculate frustum, since it is useful for Kalman Prediction. This is used in the pipeline now.
 * @param pos 
 * @param rot in angles
 * @param camInt 
 * @param frustum 
 */
void calc_frustum_from_pos_rot2(const Vector3f &pos, const Vector3f &rot, const Vector4f &quat, const CamInt &camInt, Frustum &frustum)
{
	frustum.setCamInternals(camInt);
	CamView camView;

    Vector3f X, Y, Z;
    X << 1, 0, 0;       // right
    Y << 0, 1, 0;       // up
    Z << 0, 0, 1;       // forward
	
    Matrix3f R = quat2rot(quat);

    // Manually calculate lookAt and up vectors by rotating Z and Y axis vectors
    Vector3f lookAt = R * Z;
    Vector3f up = R * Y;

	camView.p = pos;
	camView.r = rot;
    camView.q = quat;
    camView.l = lookAt;
    camView.l.normalize();
    camView.u = up;
    camView.u.normalize();

	frustum.setCamPlanes(camView);
}

/**
 * @brief Calculates a frustum from position, lookat and up vectors.
 * Not a preferred function to calculate frustum, since it is not useful for Kalman Prediction.
 * @param pos 
 * @param rot 
 * @param quat 
 * @param lookat 
 * @param up 
 * @param camInt 
 * @param frustum 
 */
void calc_frustum_from_pos_rot2(const Vector3f &pos, const Vector3f &rot, const Vector4f &quat, const Vector3f &lookat, const Vector3f &up, const CamInt &camInt, Frustum &frustum)
{
	frustum.setCamInternals(camInt);
	CamView camView;

	camView.p = pos;
	camView.r = rot;
    camView.l = lookat;
    camView.u = up;
    camView.q = quat;
	
	frustum.setCamPlanes(camView);
}

DataPlayback::DataPlayback(const string &path, const string &seqName, int logID = 0) : m_path(path), m_seqName(seqName),  m_logID(logID)
{
	m_userDataFolder = FORMAT(m_path << "/" << m_seqName << "/");
    LOG(INFO) << "USER TRACE FOLDER: " << m_userDataFolder;
	m_index = 0;
	m_startFrameID = m_lastFrameID = -1;
    m_currFrameID = 0;
    m_avgDeltaTime = 0;
    m_predictCount = 0;
    m_bInitialized = false;

    // m_pKalman = new VVR_Kalman();
}

DataPlayback::~DataPlayback()
{
}

/**
 * @brief Reads user's frustum trace. 
 * 
 * @param startFrameID This might be modified based on the trace given by the start and end frame in the user trace 
 * @param lastFrameID This might be modified based on the trace given by the start and end frame in the user trace
 */
bool DataPlayback::read_frustum_data(uint32_t &startFrameID, uint32_t &lastFrameID)
{
    /**
     * @brief data_frustum_pos_raw: timestamp, position, rotation, quaternion, forward vector, up vector
     * This is the data recorded when we hit play in Unity. The actual playback of the point clouds
     * starts when warm up ends. 
     */
    /**
     * @brief data_frame_id_raw: timestamp, frame id
     * Data starts from frame_id 1, when playback starts after warm up.
     * For every sequence, valid point clouds don't start from frame_id 1.
     */
	Data data_frustum_pos_raw, data_frame_id_raw;

	// Read camera position (position, rotation, quaternion, forward vector, up vector)
    string file_name = FORMAT(m_userDataFolder << "camera_log" << m_logID << ".txt");
    if(!read_user_data2(file_name, data_frustum_pos_raw))
    {
        LOG(ERROR) << "Could not read file: " << file_name;
        return false;
    }

	// Read frame id which were played back
	file_name = FORMAT(m_userDataFolder << "visualize_times" << m_logID << ".txt");
	if(!read_user_data2(file_name, data_frame_id_raw))
	{
		LOG(ERROR) << "Could not read file: " << file_name;
		return false;
	}

	// Read camera intrinsics
    file_name = FORMAT(m_userDataFolder << "frustum_details" << m_logID << ".txt");
    if(!read_frustum_details2(file_name, m_camInt))
    {
        LOG(ERROR) << "Could not read file: " << file_name;
        return false;
	}

    /**
     * @brief We need to the timestamp of the first and last valid point cloud.
     * We need to strip data from data_frustum_pos_raw that corresponds to data_frame_id_raw.
     * We need to strip data from remaining data_frustum_pos_raw and data_frame_id_raw that corresponds to the first and last valid point cloud.
	 * Also change the start and end frame id based on the collected trace.
	 * I assume that our playback starts from a frameId <= startFrameID, but the trace might end at a frameID <= lastFrameID.
     */

    int start_i = 0, end_i = data_frame_id_raw.size() - 1;
    while(start_i <= end_i && data_frame_id_raw[start_i][1] < startFrameID)
        start_i++;

	startFrameID = m_startFrameID = data_frame_id_raw[start_i][1];

    while(end_i >=start_i && data_frame_id_raw[end_i][1] > lastFrameID)
        end_i--;
	
	lastFrameID = m_lastFrameID = data_frame_id_raw[end_i][1];

    if(start_i >= data_frame_id_raw.size() || start_i > end_i)
    {
        LOG(ERROR) << "Could not find start time in the visualize times data" << m_startFrameID;
        return false;
    }

    if(end_i < 0 || end_i < start_i)
    {
        LOG(ERROR) << "Could not find last time in the visualize times data" << m_lastFrameID;
        return false;
    }

    // Extract data that corresponds to valid point cloud given by m_startFrameID and m_lastFrameID
    Data data_frustum_pos, data_frame_id;
    for(int i = start_i; i <= end_i; i++)
        data_frame_id.push_back(data_frame_id_raw[i]);

	start_i = 0, end_i = data_frustum_pos_raw.size() - 1;
	// find the the time where the frustum data starts
	while(data_frustum_pos_raw[start_i][0] != data_frame_id[0][0])
		start_i++;
	while(data_frustum_pos_raw[end_i][0] != data_frame_id[data_frame_id.size() - 1][0])
		end_i--;
	
	if(start_i >= data_frustum_pos_raw.size())
	{
		LOG(ERROR) << "Could not find the start time in the camera log data";
		return false;
	}

	if(end_i < 0)
	{
		LOG(ERROR) << "Could not find the end time in the camera log data";
		return false;
	}

	// Copy the frustum data from the start time to the end time
	for(int i = start_i; i <= end_i; i++)
		data_frustum_pos.push_back(data_frustum_pos_raw[i]);

	assert(data_frustum_pos.size() == data_frame_id.size());

    file_name = FORMAT(m_userDataFolder << "processed_trace_log" << m_logID << ".txt");
    fs::path out_file{file_name};
    fs::ofstream ofs;
    ofs.open(out_file, ios::out | ios::trunc);
    if (!ofs.is_open())
    {
        LOG(ERROR) << "Failed to open file: " << out_file;
        return false;
    }
    ofs << "frame_id,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,quat_x,quat_y,quat_z,quat_w,lookat_x,lookat_y,lookat_z,up_x,up_y,up_z\n";

	// Calculate original frustum from position and rotation
	for(int i = 0; i < data_frustum_pos.size(); i++)
	{
		assert(data_frustum_pos[i][0] == data_frame_id[i][0]); 

		UserData data;
		data.frameID = data_frame_id[i][1];
		data.ts_orig = data_frustum_pos[i][0];
        data.ts_pred = 0;
        Vector3f pos = Vector3f(data_frustum_pos[i][1], data_frustum_pos[i][2], data_frustum_pos[i][3]);
        Vector3f rot = Vector3f(data_frustum_pos[i][4], data_frustum_pos[i][5], data_frustum_pos[i][6]);
        Vector4f quat = Vector4f(data_frustum_pos[i][7], data_frustum_pos[i][8], data_frustum_pos[i][9], data_frustum_pos[i][10]);
		Vector3f lookat = Vector3f(data_frustum_pos[i][11], data_frustum_pos[i][12], data_frustum_pos[i][13]);
        Vector3f up = Vector3f(data_frustum_pos[i][14], data_frustum_pos[i][15], data_frustum_pos[i][16]);
        
        // ofs << data.frameID << "," << pos(0) << "," << pos(1) << "," << pos(2) << "," << rot(0) << "," << rot(1) << "," << rot(2) << "," << quat(0) << "," << quat(1) << "," << quat(2) << "," << quat(3) << "," << lookat(0) << "," << lookat(1) << "," << lookat(2) << "," << up(0) << "," << up(1) << "," << up(2) << "\n";

        // calc_frustum_from_pos_rot2(pos, rot, m_camInt, data.frustum);

        // Use this function if you want to calculate frustum from lookat and up vectors
        // Frustum frustum;
        // calc_frustum_from_pos_rot2(pos, rot, quat, lookat, up, m_camInt, frustum);

        // This function uses pos, rot, and quat to calculate frustum (Preferred function)
        calc_frustum_from_pos_rot2(pos, rot, quat, m_camInt, data.frustum);

        // // Sanity Check: if the frustum calculated from position and rotation is same as the frustum calculated from frustum details
        // float alpha = 1e-5;
        // assert((frustum.pos - data.frustum.pos).norm() < alpha);
        // assert((frustum.rot - data.frustum.rot).norm() < alpha);
        // assert((frustum.quat - data.frustum.quat).norm() < alpha);
        // assert((frustum.lookAt - data.frustum.lookAt).norm() < alpha);
        // assert((frustum.up - data.frustum.up).norm() < alpha);
        // assert((frustum.ntl - data.frustum.ntl).norm() < alpha);
        // assert((frustum.ntr - data.frustum.ntr).norm() < alpha);
        // assert((frustum.nbl - data.frustum.nbl).norm() < alpha);
        // assert((frustum.nbr - data.frustum.nbr).norm() < alpha);
        // assert((frustum.ftl - data.frustum.ftl).norm() < alpha);
        // assert((frustum.ftr - data.frustum.ftr).norm() < alpha);
        // assert((frustum.fbl - data.frustum.fbl).norm() < alpha);
        // assert((frustum.fbr - data.frustum.fbr).norm() < alpha);
        // assert(abs(frustum.nearD - data.frustum.nearD) < alpha);
        // assert(abs(frustum.farD - data.frustum.farD) < alpha);
        // assert(abs(frustum.ratio - data.frustum.ratio) < alpha);
        // assert(abs(frustum.angle - data.frustum.angle) < alpha);
        // assert(abs(frustum.tang - data.frustum.tang) < alpha);
        // assert(abs(frustum.nw - data.frustum.nw) < alpha);
        // assert(abs(frustum.nh - data.frustum.nh) < alpha);
        // assert(abs(frustum.fw - data.frustum.fw) < alpha);
        // assert(abs(frustum.fh - data.frustum.fh) < alpha);

        data.frustum.frameID = data.frameID;
        data.pos = data.frustum.pos;
        data.rot = data.frustum.rot;
        data.quat = data.frustum.quat;
        data.lookat = data.frustum.lookAt;
        data.up = data.frustum.up;

        assert(m_mapUserData.find(data.frameID) == m_mapUserData.end());
        m_mapUserData[data.frameID] = data;
	}
    ofs.close();
    m_currFrameID = m_startFrameID;
    return true;
}

/**
 * @brief Loads user trace from disk.
 * 
 * @param startFrameID Modifies the start frame id based on the collected trace.
 * @param lastFrameID Modifies the last frame id based on the collected trace.
 */
bool DataPlayback::loadData(uint32_t &startFrameID, uint32_t &lastFrameID)
{
	if(!fs::exists(m_path))
    {
        LOG(ERROR) << "User data folder does not exist: " << m_path << ". Exiting ...";
        return false;
    }

	if(!read_frustum_data(startFrameID, lastFrameID))
	{
		LOG(ERROR) << "Could not read user's frustum data";
		return false;
	}

	return true;
}

/**
 * @brief TODO: This remove this function later
 * 
 * @param frameID 
 * @return UserData* 
 */
UserData DataPlayback::getUserData(uint32_t frameID)
{
    if(m_mapUserData.find(frameID) == m_mapUserData.end())
        LOG(FATAL) << "Could not find frameID: " << frameID << " in the user trace";
    return m_mapUserData[frameID];
}

uint32_t DataPlayback::getStartFrameID()
{
    return m_startFrameID;
}

uint32_t DataPlayback::getEndFrameID()
{
    return m_lastFrameID;
}

/**
 * @brief Predicts the user pos, rot and frustum for the given frameID.
 * 
 * @param frameID 
 * @param lookAheadTime in ms. 0 ms means run Kalman Filter for next frame.
 * @param orig Original user trace
 * @param pred Predicted user trace
 */
// bool DataPlayback::predictData(uint32_t frameID, uint32_t lookAheadTime, UserData &orig, UserData &pred)
// {
//     if(m_mapUserData.find(frameID) == m_mapUserData.end())
//         return false;

//     orig = m_mapUserData[frameID];
//     Vector3f orig_pos = orig.pos;
//     Vector3f orig_rot = orig.rot;
//     uint32_t orig_frameID = orig.frameID;
    
//     uint32_t ts;
//     if(!m_bInitialized)
//     {
//         // Timer starts
//         m_timer.Restart();
//         ts = m_timer.ElapsedMs();
//         m_pKalman->Init_Model(orig_pos[0], orig_pos[1], orig_pos[2], orig_rot[0], orig_rot[1], orig_rot[2], ts/1000.0f);
//         pred = orig;
//         pred.ts_pred = ts;
        
//         m_prevFrameID = 0;
//         m_predictCount = 0;
//         m_bInitialized = true;

//         return true;
//     }

//     ts = m_timer.ElapsedMs();

//     double dt = ts;
//     if(m_predictCount > 0)
//     {
//         dt = ts - m_mapUserData[m_prevFrameID].ts_pred;
//         m_avgDeltaTime = ceil((m_avgDeltaTime * m_predictCount + dt) / (m_predictCount + 1)); // Round off to ms
//     }
//     else
//         m_avgDeltaTime = ceil(dt);
    
//     int lookAhead = m_avgDeltaTime < 1 ? 1 : ceil(lookAheadTime / m_avgDeltaTime);
//     QCHECK(lookAhead > 0) << "Look ahead time is too small";

//     vector<float> prediction = m_pKalman->Predict_Update(orig_pos[0], orig_pos[1], orig_pos[2], orig_rot[0], orig_rot[1], orig_rot[2], dt/1000.0f, lookAhead);

//     pred.frameID = orig_frameID;
//     pred.ts_orig = orig.ts_orig;
//     pred.ts_pred = ts;
//     pred.pos = Vector3f(prediction[0], prediction[1], prediction[2]);
//     pred.rot = Vector3f(prediction[3], prediction[4], prediction[5]);
//     calc_frustum_from_pos_rot(pred.pos, pred.rot, m_camInt, pred.frustum);

//     m_prevFrameID = frameID;
//     m_mapUserData[m_prevFrameID].ts_pred = ts;
//     m_predictCount++;

//     return true;
// }