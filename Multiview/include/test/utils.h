#include "frustum.h"
#include <Eigen/Dense>
#include <iostream>
#include <string>
// #include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <boost/tokenizer.hpp>
// #include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>

#define ANG2RAD(x) x * (M_PI/180.0f)

#define FORMAT(items) \
    static_cast<std::ostringstream &>((std::ostringstream() << std::string() << items)).str()

using namespace std;
using namespace Eigen;
using namespace boost;
namespace fs = boost::filesystem;

void parse_camera_pos(const vector<vector<float>> &data_camera_pos, const CamInt &camInt, vector<CamView> &cam_view, vector<Frustum> &frustum_from_pos);
void calc_TR(const Vector3f &pos, const Vector3f angles, Matrix4f &T, Matrix4f &R);
void calc_frustum_vec(const vector<Frustum> &frustum_from_pos, const CamInt &camInt, const vector<CamView> &cam_view, vector<CamView> &my_cam_view, vector<Frustum> &my_frustum_from_pos);
void parse_frustum_planes(const vector<vector<float>> &data_frustum_planes, const CamInt &camInt, vector<Frustum> &frustum_from_planes);
void parse_frustum_corners(const vector<vector<float>> &data_frustum_corners, vector<vector<Vector3f>> &frustum_corners);
bool read_user_data(const string &file_name, vector<vector<float>> &out);
bool read_frustum_details(const string &file_name, CamInt &camInt);
