#pragma once
#include <boost/filesystem/fstream.hpp>
#include "absl/container/flat_hash_map.h"
#include "frustum.h"
#include "consts.h"
#include "utils.h"
#include "timer.h"

using namespace std;

/**
 * @brief TODO: Rajrup: Names clash with test/utils.h
 * 
 * @param data_camera_pos 
 * @param camInt 
 * @param cam_view 
 * @param frustum_from_pos 
 */
void parse_camera_pos2(const vector<vector<float>> &data_camera_pos, const CamInt &camInt, vector<CamView> &cam_view, vector<Frustum> &frustum_from_pos);
void calc_TR2(const Vector3f &pos, const Vector3f angles, Matrix4f &T, Matrix4f &R);
void calc_frustum_vec2(const vector<Frustum> &frustum_from_pos, const CamInt &camInt, const vector<CamView> &cam_view, vector<CamView> &my_cam_view, vector<Frustum> &my_frustum_from_pos);
void parse_frustum_planes2(const vector<vector<float>> &data_frustum_planes, const CamInt &camInt, vector<Frustum> &frustum_from_planes);
void parse_frustum_corners2(const vector<vector<float>> &data_frustum_corners, vector<vector<Vector3f>> &frustum_corners);
bool read_user_data2(const std::string &file_name, vector<vector<float>> &out);
bool read_frustum_details2(const std::string &file_name, CamInt &camInt);

void calc_frustum_from_pos_rot2(const Vector3f &pos, const Vector3f &rot, const CamInt &camInt, Frustum &frustum);
void calc_frustum_from_pos_rot2(const Vector3f &pos, const Vector3f &rot, const Vector4f &quat, const CamInt &camInt, Frustum &frustum);
void calc_frustum_from_pos_rot2(const Vector3f &pos, const Vector3f &rot, const Vector4f &quat, const Vector3f &lookat, const Vector3f &up, const CamInt &camInt, Frustum &frustum);

class DataPlayback
{

public:
	DataPlayback(const std::string &path, const std::string &seqName, int logID);
	~DataPlayback();

	bool loadData(uint32_t &startFrameID, uint32_t &lastFrameID);
	uint32_t getStartFrameID();
	uint32_t getEndFrameID();
	UserData getUserData(uint32_t frameID);

private:
	bool read_frustum_data(uint32_t &startFrameID, uint32_t &lastFrameID);
	
	std::string m_path;
	std::string m_seqName;
	std::string m_userDataFolder;
	int m_logID;

	uint32_t m_startFrameID;
	uint32_t m_lastFrameID;
	CamInt m_camInt;
	
	absl::flat_hash_map<uint32_t, UserData> m_mapUserData;

	int m_index;
	uint32_t m_currFrameID, m_prevFrameID;
	double m_avgDeltaTime;
	double m_predictCount;
	bool m_bInitialized;
};