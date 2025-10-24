#include "frustum.h"
#include "test/utils.h"

int main()
{
	cout << "Hello Frustum Test!" << endl;

	//---------- Construct kalman filtering for sensor data

    vector<vector<float>> data_camera_pos, data_frustum_planes, data_frustum_corners;
    vector<vector<float>> predictions;
    // string project_dir = "/data/Dropbox/Project/VolumetricVideo/KinectStream/";
    string project_dir = "/home/rajrup/Dropbox/Project/VolumetricVideo/KinectStream/";

    // string kinoptic_data = "160422_haggling1";
    // string kinoptic_data = "170915_office1";
    string kinoptic_data = "170915_toddler4";
    int logID = 0;
    string user_data_folder = FORMAT(project_dir << "data/user_log/" << kinoptic_data << "/");

    if(!boost::filesystem::exists(user_data_folder))
    {
        cout << "User data folder does not exist: " << user_data_folder << endl;
        cout << "Exiting..." << endl;
        return 0;
    }

    const int NUM_COLS = 7;
    const int SKIP_ROWS = 1107;

    // Read camera position (position, rotation, quaternion, forward vector, up vector)
    string file_name = FORMAT(user_data_folder << "camera_log" << logID << ".txt");
    read_user_data(file_name, data_camera_pos);

    // Read equations of frustum planes (left, right, down (bottom), up (top), near, far)
    file_name = FORMAT(user_data_folder << "frustum_planes" << logID << ".txt");
    read_user_data(file_name, data_frustum_planes);

    // Read frustum corners (near corners, far corners)
    file_name = FORMAT(user_data_folder << "frustum_corners" << logID << ".txt");
    read_user_data(file_name, data_frustum_corners);

    // Read camera intrinsics (angle, ratio, nearD, farD)
    CamInt camInt;
    file_name = FORMAT(user_data_folder << "frustum_details" << logID << ".txt");
    read_frustum_details(file_name, camInt);

    vector<CamView> cam_view;
    vector<Frustum> frustum_from_pos;
    parse_camera_pos(data_camera_pos, camInt, cam_view, frustum_from_pos);
    
    vector<Frustum> frustum_from_planes;
    parse_frustum_planes(data_frustum_planes, camInt, frustum_from_planes);

    vector<vector<Vector3f>> frustum_corners;
    parse_frustum_corners(data_frustum_corners, frustum_corners);

    //---------- Test frustum
    vector<CamView> my_cam_view;
    vector<Frustum> my_frustum_from_pos;
    calc_frustum_vec(frustum_from_pos, camInt, cam_view, my_cam_view, my_frustum_from_pos);

    frustum_from_pos[1000].print();
    my_frustum_from_pos[1000].print();
    frustum_from_planes[1000].print();

	return 0;
}