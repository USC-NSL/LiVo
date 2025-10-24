#include "test/utils.h"
#include <boost/filesystem/fstream.hpp>

/**
 * @brief Reads frustum position, rotation, quaternion, forward vector, up vector from file
 * 
 * @param file_name 
 * @param out  Index 0 - timestamp, Index 1-3 - position, Index 4-6 - rotation, Index 7-10 - quaternion, Index 11-13 - forward vector, Index 14-16 - up vector
 */
bool read_user_data(const string &file_name, vector<vector<float>> &out)
{
    fs::path in_file{file_name};
	fs::ifstream ifs;
	ifs.open(in_file, ios::in);
	if (!ifs.is_open())
	{
		cout << "Failed to load HMD Camera Log file: " << in_file << endl;
		return false;
	}

    string line;
    int row_id = 0;
    while(getline(ifs, line))
    {
        vector<float> temp;
        tokenizer<escaped_list_separator<char> > tk(
        line, escaped_list_separator<char>("", ",:", ""));
        for(tokenizer<escaped_list_separator<char> >::iterator i(tk.begin()); i!=tk.end(); ++i) 
            temp.push_back(stof(*i));
        out.push_back(temp);
        
        // Check if it's a matrix
        if(row_id > 0)
            assert(out[row_id].size() == out[row_id-1].size());
        out[row_id][0] = out[row_id][0] / 1000.0f;
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
void parse_camera_pos(const vector<vector<float>> &data_camera_pos, const CamInt &camInt, vector<CamView> &cam_view, vector<Frustum> &frustum_from_pos)
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
 * 
 * @param pos 
 * @param angles in radians
 * @param T 
 * @param R 
 */
void calc_TR(const Vector3f &pos, const Vector3f angles, Matrix4f &T, Matrix4f &R)
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

void calc_frustum_vec(const vector<Frustum> &frustum_from_pos, const CamInt &camInt, const vector<CamView> &cam_view, vector<CamView> &my_cam_view, vector<Frustum> &my_frustum_from_pos)
{
    Frustum myFrustum;
    myFrustum.setCamInternals(camInt);
    CamView myCamView;

    Matrix4f R, T;

    Vector4f X = Vector4f::UnitX(); // right
    Vector4f Y = Vector4f::UnitY(); // up
    Vector4f Z = Vector4f::UnitZ(); // forward
    for(auto camView : cam_view)
    {
        myCamView.p = camView.p;
        myCamView.r = camView.r;
        
        calc_TR(camView.p, camView.r, T, R);
        myCamView.l = (T * (R * Z)).head(3);
        myCamView.u = (T * (R * Y)).head(3);

        myFrustum.setCamPlanes(myCamView);
        my_frustum_from_pos.push_back(myFrustum);
    }
}

/**
 * @brief Get camera plane equations from Unity data.
 * Also construct Frustum from the equations of the planes
 * @param data_frustum_planes 
 * @param camInt 
 * @param frustum_from_planes 
 */
void parse_frustum_planes(const vector<vector<float>> &data_frustum_planes, const CamInt &camInt, vector<Frustum> &frustum_from_planes)
{
    Frustum frustum;
    frustum.setCamInternals(camInt);
    Plane planes[6];
    for(auto vec : data_frustum_planes)
    {
        planes[LEFT].setCoefficients2(vec[1], vec[2], vec[3], vec[4]);
        planes[RIGHT].setCoefficients2(vec[5], vec[6], vec[7], vec[8]);
        planes[BOTTOM].setCoefficients2(vec[9], vec[10], vec[11], vec[12]);
        planes[TOP].setCoefficients2(vec[13], vec[14], vec[15], vec[16]);
        planes[NEAR].setCoefficients2(vec[17], vec[18], vec[19], vec[20]);
        planes[FAR].setCoefficients2(vec[21], vec[22], vec[23], vec[24]);
        frustum.setCamPlanes(planes[LEFT], planes[RIGHT], planes[BOTTOM], planes[TOP], planes[NEAR], planes[FAR]);
        frustum_from_planes.push_back(frustum);
    }
}

/**
 * @brief Get frustum corners from Unity data
 * Note: I don't know the order of the frustum corners in Unity data
 * @param data_frustum_corners 
 * @param frustum_corners 
 */
void parse_frustum_corners(const vector<vector<float>> &data_frustum_corners, vector<vector<Vector3f>> &frustum_corners)
{
    for(auto vec : data_frustum_corners)
    {
        vector<Vector3f> corners;
        Vector3f corner;
        for(int i=0; i<8; i++)
        {
            corner << vec[i*3+1], vec[i*3+2], vec[i*3+3];
            corners.push_back(corner);
        }
        frustum_corners.push_back(corners);
    }
}

/**
 * @brief Get Camera Instrinsic parameters from Unity data
 * @param data_cam_view 
 * @param cam_view 
 */
bool read_frustum_details(const string &file_name, CamInt &camInt)
{
    fs::path in_file{file_name};
	fs::ifstream ifs;
	ifs.open(in_file, ios::in);
	if (!ifs.is_open())
	{
		cout << "Failed to load HMD Frustum Details file: " << in_file << endl;
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