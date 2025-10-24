#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <boost/filesystem/fstream.hpp>
#include <nlohmann/json.hpp>
#include "calibration.h"
#include "utils.h"
#include "consts.h"

using namespace std;
namespace fs = boost::filesystem;
using json = nlohmann::json;

const int NUM_CAMS = 10;

struct RGBD
{
    cv::Mat rgb;
    cv::Mat depth;
    int frameID;
    int camID;
    bool valid;

    RGBD() : frameID(-1), camID(-1) ,valid(false) {}
};

unordered_map<int, unordered_map<int, double>> ksync_extract_univ_time(json &jKsync, const string &cam, unordered_map<int, string> &knames)
{
    unordered_map<int, unordered_map<int, double>> univ_time;

    if(cam != "color" && cam != "depth")
        return univ_time;

    json &jcam = jKsync["kinect"][cam];
    
    for(int idk = 1; idk <= NUM_CAMS; idk++)
    {
        vector<int> index;
        vector<double> time;
        for(auto& elm : jcam[knames[idk]]["index"].items())
            index.push_back(elm.value().get<int>());

        for(auto& elm : jcam[knames[idk]]["univ_time"].items())
            time.push_back(elm.value().get<double>());

        univ_time[idk] = unordered_map<int, double>();
        assert(index.size() == time.size());
        for(int i = 0; i < index.size(); i++)
            univ_time[idk][index[i]] = time[i];
    }
    return univ_time;
}

unordered_map<int, double> psync_extract_univ_time(json &j, const string &cam)
{
    unordered_map<int, double> univ_time;

    if(cam != "vga" && cam != "hd")
        return univ_time;
    
    vector<int> index;
    vector<double> time;
    for(auto& elm : j[cam]["index"].items())
        index.push_back(elm.value().get<int>());

    for(auto& elm : j[cam]["univ_time"].items())
        time.push_back(elm.value().get<double>());

    assert(index.size() == time.size());
    for(int i = 0; i < index.size(); i++)
        univ_time[index[i]] = time[i];
    
    return univ_time;
}

/**
 * @brief Find the names of the camera
 * 
 * @param jCalibration 
 * @return vector<string> 
 */
vector<string> panoptic_extract_camNames(json &jCalibration)
{
    json &jcam = jCalibration["cameras"];
    vector<string> camNames;
    for (const auto& item : jCalibration["cameras"])
    {
        camNames.push_back(item["name"].get<string>());
    }
    return camNames;
}

pair<int, double> kinect_univ_diff(unordered_map<int, double> &univ_time, double selUnivTime, double offset = 0.0)
{
    double diff = DBL_MAX;
    int index = 0;
    for(auto& elm : univ_time)
    {
        double d = abs(selUnivTime - (elm.second - offset));
        if(d < diff)
        {
            diff = d;
            index = elm.first;
        }
    }
    return make_pair(index, diff);
}

void print_image(cv::Mat &img, int num_pixels=-1)
{
    int channels = img.channels();

    int count = 0;
    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            if(num_pixels >=0 && count >= num_pixels)
                return;

            cout << "Idx (" << i << ", " << j << "): ";
            if(channels == 1)
                cout << (int)img.at<uint16_t>(i, j) << " ";
            else if(channels == 3)
            {
                cout << (int)img.at<cv::Vec3b>(i, j)[0] << " ";
                cout << (int)img.at<cv::Vec3b>(i, j)[1] << " ";
                cout << (int)img.at<cv::Vec3b>(i, j)[2] << " ";
            }
            cout << endl;
            count++;
        }
    }
}

void calc_valid_dapthmask(cv::Mat src, cv::Mat &validMask, cv::Mat &nonValidMask, vector<pair<int, int>> &validPixIdx, vector<pair<int, int>> &nonValidPixIdx)
{
    validMask = cv::Mat::zeros(src.size(), src.type());
    nonValidMask = cv::Mat::zeros(src.size(), src.type());
    for(int i = 0; i < src.rows; i++)
    {
        for(int j = 0; j < src.cols; j++)
        {
            if(src.at<uint16_t>(i, j) == 0)
            {
                nonValidMask.at<uint16_t>(i, j) = 1;
                nonValidPixIdx.push_back(make_pair(i, j));
            }
            else
            {
                validMask.at<uint16_t>(i, j) = 1;
                validPixIdx.push_back(make_pair(i, j));
            }
        }
    }
}

bool read_color3D(const string &filename, RGB *&color3D, int &num_vertices)
{
    cv::Mat color = cv::imread(filename);
    if(color.empty())
    {
        cout << "Failed to read color image: " << filename << endl;
        return false;
    }

    // cv::cvtColor(color, color, cv::COLOR_BGR2BGRA); // RGB struct contains alpha channel

    num_vertices = color.rows * color.cols;
    color3D = new RGB[num_vertices];
    // memcpy(color3D, color.data, sizeof(RGB) * num_vertices);

    int idx = 0;
    for(int j = 0; j < color.cols; j++)             // Column major order. Keeping it similar to matlab. To Do: Rajrup: Change matlab code to perform memcpy here.
    {
        for(int i = 0; i < color.rows; i++)
        {
            idx = j * color.rows + i;
            color3D[idx].rgbBlue = color.at<cv::Vec3b>(i, j)[0];
            color3D[idx].rgbGreen = color.at<cv::Vec3b>(i, j)[1];
            color3D[idx].rgbRed = color.at<cv::Vec3b>(i, j)[2];
        }
    }
    color.release();

    // cv::imshow("color", color);
    // cv::waitKey(0);

    return true;
}

/**
 * @brief Converts panoptic and kinect camera calibration to our extrinsic calibration format
 * 
 * @param panoptic_calibration 
 * @param kinect_calibration 
 * @param extrinsic_calibration Extrinsic calibration in our format
 */
void panoptic_extrinsic_calibration(json &panoptic_calibration, json &kinect_calibration, vector<Calibration> &extrinsic_calibration)
{
    string camName;
    unordered_map<string, int> camMap;
    int idk;

    // const int W_BORDER = 100; // 100 pixels border. Keep border an even number
    // const int H_BORDER = 100;

    for (idk = 1; idk <= NUM_CAMS; idk++)
    {
        camName = FORMAT("50_" << setfill('0') << setw(2) << idk);
        camMap[camName] = idk;
    }

    // Scale factor
    Eigen::Matrix4f S_kinoptic2panoptic = Eigen::Matrix4f::Identity() * 100.0F; // centimeters to meters
    S_kinoptic2panoptic(3, 3) = 1.0F;

    for (const auto& panopCalibData : panoptic_calibration["cameras"])
    {
        camName = panopCalibData["name"].get<string>();
        if(camMap.find(camName) == camMap.end())
            continue;
        idk = camMap[camName];

        Eigen::Matrix4f M = Eigen::Matrix4f::Identity();

        int i, j;
        i = j = 0;
        for(auto& row : panopCalibData["R"].items())
        {
            j = 0;
            for(auto& col : row.value().items())
            {
                M(i, j) = col.value().get<float>();
                j++;
            }
            i++;
        }
        i = 0;
        j = 3;
        for(auto& row : panopCalibData["t"].items())
        {
            for(auto& col : row.value().items())
            {
                M(i, j) = col.value().get<float>();
            }
            i++;
        }
        Eigen::Matrix4f T_kinectColor2PanopticWorld = M.inverse();
        
        json &camCalibData = kinect_calibration["sensors"][idk - 1];    // idk starts from 1, but json array is 0-indexed.
        M = Eigen::Matrix4f::Identity();
        i = j = 0;
        for(auto& row : camCalibData["M_color"].items())
        {
            j = 0;
            for(auto& col : row.value().items())
            {
                M(i, j) = col.value().get<float>();
                j++;
            }
            i++;
        }

        Eigen::Matrix4f T_kinectLocal2KinectColor = M.inverse();
        Eigen::Matrix4f T_kinectLocal2PanopticWorld = T_kinectColor2PanopticWorld * S_kinoptic2panoptic * T_kinectLocal2KinectColor;

        cout << "T_kinectLocal2PanopticWorld for camID " << idk << " camName " << camName << endl;
        cout << T_kinectLocal2PanopticWorld << endl;

        Calibration &calib = extrinsic_calibration[idk - 1];
        for(i = 0; i < 3; i++)
        {
            for(j = 0; j < 3; j++)
            {
                calib.worldR[i][j] = T_kinectLocal2PanopticWorld(i, j)/100.0F;  // centimeters to meters
            }
        }
        for(i = 0; i < 3; i++)
        {
            calib.worldT[i] = T_kinectLocal2PanopticWorld(i, 3)/100.0F; // centimeters to meters
        }

        // calib.intrinsics.width = camCalibData["depth_width"].get<int>() + W_BORDER; // Add 100 to avoid boundary issues.
        // calib.intrinsics.height = camCalibData["depth_height"].get<int>() + H_BORDER; // Add 100 to avoid boundary issues.

        int w = camCalibData["depth_width"].get<int>();
        int h = camCalibData["depth_height"].get<int>();
        calib.intrinsics.width = int(ceil(w/16.0F) * 16.0F) + 5*16;         // Round off to nearest multiple of 16 and add 5*16 to avoid boundary issues.
        calib.intrinsics.height = int(ceil(h/16.0F) * 16.0F) + 5*16;        // Round off to nearest multiple of 16 and add 5*16 to avoid boundary issues.
        int W_BORDER = calib.intrinsics.width - w;
        int H_BORDER = calib.intrinsics.height - h;

        Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
        i = j = 0;
        for(auto& row : camCalibData["K_depth"].items())
        {
            j = 0;
            for(auto& col : row.value().items())
            {
                K(i, j) = col.value().get<float>();
                j++;
            }
            i++;
        }
        calib.intrinsics.fx = K(0, 0);
        calib.intrinsics.fy = K(1, 1);
        calib.intrinsics.cx = K(0, 2) + float(W_BORDER)/2.0f;    // Add 50 to avoid boundary issues.
        calib.intrinsics.cy = K(1, 2) + float(H_BORDER)/2.0f; // Add 50 to avoid boundary issues.
        calib.bCalibrated = true;
    }
    return;
}

bool read_points3D(const string &filename, Point3f *&points3D, int &num_vertices)
{
    fs::path filepath(filename);
    if(!fs::exists(filepath))
    {
        cout << "File " << filename << " does not exist" << endl;
        return false;
    }
    fs::ifstream ifs(filepath, fs::ifstream::binary | fs::ifstream::ate);
    std::streamsize size = ifs.tellg();
    ifs.seekg(0, fs::ifstream::beg);

    assert(size % (3 * sizeof(float)) == 0);
    num_vertices = size / (3 * sizeof(float));
    points3D = new Point3f[num_vertices];
    std::vector<char> buffer(size);
    if (ifs.read(buffer.data(), size))
    {
        float *arr = reinterpret_cast<float *>(buffer.data());
        for(int i = 0; i < num_vertices; i++)
        {
            points3D[i].X = arr[i * 3 + 0];
            points3D[i].Y = arr[i * 3 + 1];
            points3D[i].Z = arr[i * 3 + 2];

            // points3D[i].X = arr[0 * num_vertices + i];
            // points3D[i].Y = arr[1 * num_vertices + i];
            // points3D[i].Z = arr[2 * num_vertices + i];

            if(points3D[i].X == 0 && points3D[i].Y == 0 && points3D[i].Z == 0)
                points3D[i].Invalid = true;
        }
    }
    else
    {
        cout << "Failed to read " << filename << endl;
        num_vertices = 0;
        return false;
    }
    buffer.clear();
    return true;
}

/**
 * @brief Transform points from camera coordinate system to world.
 * 
 * @param points3D 
 * @param num_vertices 
 * @param extCalib 
 */
void transform_points3D_world(Point3f *&points3D, const int num_vertices, const Calibration &extCalib)
{
    auto &R = extCalib.worldR;
    auto &T = extCalib.worldT;

    for(int i = 0; i < num_vertices; i++)
    {
        Point3f &p = points3D[i];
        if(p.Invalid)
            continue;
        
        Point3f temp;
        temp.X = (R[0][0] * p.X + R[0][1] * p.Y + R[0][2] * p.Z + T[0])*1000.0F; // m to mm 
        temp.Y = (R[1][0] * p.X + R[1][1] * p.Y + R[1][2] * p.Z + T[1])*1000.0F;
        temp.Z = (R[2][0] * p.X + R[2][1] * p.Y + R[2][2] * p.Z + T[2])*1000.0F;
        p = temp;
    }
}

/**
 * @brief Fuse all the points clouds in the world coordinate system.
 * 
 * @param points3D 
 * @param colors3D 
 * @param num_vertices 
 * @param allPoints3D 
 * @param allColors3D 
 */
void accumulate_point3D_world(const Point3f *points3D, const RGB *colors3D, const int num_vertices, vector<Point3f> &allPoints3D, vector<RGB> &allColors3D)
{
    for(int i = 0; i < num_vertices; i++)
    {
        if(points3D[i].Invalid || (colors3D[i].rgbBlue == 0 && colors3D[i].rgbGreen == 0 && colors3D[i].rgbRed == 0))
            continue;
        allPoints3D.push_back(points3D[i]);
        allColors3D.push_back(colors3D[i]);
    }
}

cv::Mat generate_points3D2depthmap(Point3f *&points3D, const int num_vertices, const Calibration &extCalib)
{
    int width = extCalib.intrinsics.width;
    int height = extCalib.intrinsics.height;
    float fx = extCalib.intrinsics.fx;
    float fy = extCalib.intrinsics.fy;
    float cx = extCalib.intrinsics.cx;
    float cy = extCalib.intrinsics.cy;

    cv::Mat depthmap = cv::Mat(height, width, CV_16U, cv::Scalar(0));

    for(int i = 0; i < num_vertices; i++)
    {
        Point3f &p = points3D[i];
        if(p.Invalid)
            continue;
        int u = cx - (p.X / p.Z) * fx;
        int v = cy - (p.Y / p.Z) * fy;
        if(u < 0 || u >= width || v < 0 || v >= height)
        {
            cout << "u = " << u << ", v = " << v << endl;
            continue;
        }
        depthmap.at<uint16_t>(v, u) = uint16_t(p.Z * 1000.0F);
    }
    return depthmap;
}

/**
 * @brief Generate depth and color images from 3D points and colors
 * 
 * @param points3D 
 * @param colors3D 
 * @param num_vertices 
 * @param extCalib 
 * @param depth_image Output depth image
 * @param color_image Output color image
 */
void generate_points3D2images(const Point3f *points3D, const RGB *colors3D, const int num_vertices, const Calibration &extCalib, cv::Mat &depth_image, cv::Mat &color_image)
{
    int width = extCalib.intrinsics.width;
    int height = extCalib.intrinsics.height;
    float fx = extCalib.intrinsics.fx;
    float fy = extCalib.intrinsics.fy;
    float cx = extCalib.intrinsics.cx;
    float cy = extCalib.intrinsics.cy;

    depth_image = cv::Mat(height, width, CV_16U, cv::Scalar(0));
    color_image = cv::Mat(height, width, CV_8UC4, cv::Scalar(0));

    for(int i = 0; i < num_vertices; i++)
    {
        const Point3f &p = points3D[i];
        if(p.Invalid)
            continue;
        int u = cx - ((p.X / p.Z) * fx);
        int v = cy - ((p.Y / p.Z) * fy);
        if(u < 0 || u >= width || v < 0 || v >= height)
        {
            cout << "u = " << u << ", v = " << v << endl;
            continue;
        }
        
        depth_image.at<uint16_t>(v, u) = uint16_t(p.Z * 1000.0F);   // depth in mm
        color_image.at<cv::Vec4b>(v, u) = cv::Vec4b(colors3D[i].rgbBlue, colors3D[i].rgbGreen, colors3D[i].rgbRed, 255);
    }
}

bool save_image2disk(const string &folder, const cv::Mat &depth_image, const cv::Mat &color_image, int frame_id, int camera_id)
{
    if(!fs::exists(folder))
        fs::create_directories(folder);
    string filename = FORMAT(folder << "/" << frame_id << "_depth_" << camera_id << ".png");

    if(fs::exists(filename))
    {
        cout << "File " << filename << " already exists. Skip saving." << endl;
        return false;
    }

    if(!cv::imwrite(filename, depth_image))
    {
        cout << "Failed to save depth image to " << filename << endl;
        return false;
    }

    filename = FORMAT(folder << "/" << frame_id << "_color_" << camera_id << ".png");
    if(!cv::imwrite(filename, color_image))
    {
        cout << "Failed to save color image to " << filename << endl;
        return false;
    }
    return true;
}

void generate_depthmap2points3D(const cv::Mat &depthmap, Point3f *&points3D, int &num_vertices, const Calibration &extCalib)
{
    int width = extCalib.intrinsics.width;
    int height = extCalib.intrinsics.height;
    assert(depthmap.rows == height && depthmap.cols == width);
    
    float fx = extCalib.intrinsics.fx;
    float fy = extCalib.intrinsics.fy;
    float cx = extCalib.intrinsics.cx;
    float cy = extCalib.intrinsics.cy;

    float x = 0.0F, y = 0.0F, z = 0.0F;
    num_vertices = 0;
    for(int v = 0, idx = 0; v < height; v++)
    {
        y = float(v);
        for(int u = 0; u < width; u++, idx++)
        {
            x = float(u);
            z = float(depthmap.at<uint16_t>(v, u));
            if (z < 0.0001F)                        // tolerate for invalid depth
            {
                points3D[idx].Invalid = true;
                continue;
            }
            
            points3D[idx].X = (cx - x) * z / fx;    // in mm
            points3D[idx].Y = (cy - y) * z / fy;    // in mm
            points3D[idx].Z = z;                    // in mm
            points3D[idx].Invalid = false;
        }
        num_vertices = idx;
    }
}

/**
 * @brief Generate 3D points and colors from depth and color images
 * 
 * @param depth_image 
 * @param color_image 
 * @param extCalib 
 * @param points3D 
 * @param color3D 
 * @param num_vertices 
 */
void generate_images2points3D(const cv::Mat &depth_image, const cv::Mat &color_image, const Calibration &extCalib, Point3f *&points3D, RGB *&color3D, int &num_vertices)
{
    int width = extCalib.intrinsics.width;
    int height = extCalib.intrinsics.height;
    assert(depth_image.rows == height && depth_image.cols == width);
    num_vertices = width * height;
    points3D = new Point3f[num_vertices];
    color3D = new RGB[num_vertices];
    
    float fx = extCalib.intrinsics.fx;
    float fy = extCalib.intrinsics.fy;
    float cx = extCalib.intrinsics.cx;
    float cy = extCalib.intrinsics.cy;

    float x = 0.0F, y = 0.0F, z = 0.0F;
    for(int v = 0, idx = 0; v < height; v++)
    {
        y = float(v);
        for(int u = 0; u < width; u++, idx++)
        {
            x = float(u);
            z = float(depth_image.at<uint16_t>(v, u));
            if (z < 0.0001F)                        // tolerate for invalid depth
                continue;
            
            points3D[idx].X = ((cx - x) * z / fx) / 1000.0F;    // mm to m
            points3D[idx].Y = ((cy - y) * z / fy) / 1000.0F;    // mm to m
            points3D[idx].Z = z / 1000.0F;                      // mm to m
            points3D[idx].Invalid = false;

            const cv::Vec4b &c = color_image.at<cv::Vec4b>(v, u);
            color3D[idx].rgbBlue = c[0];
            color3D[idx].rgbGreen = c[1];
            color3D[idx].rgbRed = c[2];
        }
        num_vertices = idx;
    }
}

bool save_device_details(const string &folder, const vector<uint32_t> &device_ids, const vector<uint32_t> &device_serial_num)
{   
	fs::ofstream file;
	fs::path filepath{FORMAT(folder << DEVICE_FILE_NAME << ".txt")};
	file.open(filepath, ios::out | ios::trunc);
	if (!file.is_open())
	{	
		cout << "Failed to open file: " << filepath << endl;
		return false;
	}

	for (uint32_t i : device_ids)
		file << i << " " << device_serial_num[i] << endl;

	file.close();
	cout << "Device details saved to: " << filepath << endl;
	return true;
}

int panoptic_process(const string &panoptic_path, const string &seq_name, const string &out_dir, int &first_saved_frame_id, int &last_saved_frame_id)
{
    if(!fs::exists(panoptic_path))
    {
        cout << "Panoptic path does not exist: " << panoptic_path << endl;
        return -1;
    }

    if(!fs::exists(out_dir))
        fs::create_directories(out_dir);

    first_saved_frame_id = -1;
    last_saved_frame_id = -1;

    const int START_FRAME_ID = 1;
    const int MAX_FRAMES = 13750;
    vector<int> hd_index_list(MAX_FRAMES);
    iota(hd_index_list.begin(), hd_index_list.end(), 1);
    
    // string kinectImgDir = FORMAT(panoptic_path << seq_name << "/kinectImgs");
    // string kinectDepthImgDir = FORMAT(panoptic_path << seq_name << "/kinopticDepthImgs");
    // string kinectDepthDir = FORMAT(panoptic_path << seq_name << "/kinect_shared_depth");
    string calibFileName = FORMAT(panoptic_path << seq_name << "/kcalibration_" << seq_name << ".json");
    string syncTableFileName = FORMAT(panoptic_path << seq_name << "/ksynctables_" << seq_name << ".json");
    string panopcalibFileName = FORMAT(panoptic_path << seq_name << "/calibration_" << seq_name << ".json");
    string panopSyncTableFileName = FORMAT(panoptic_path << seq_name << "/synctables_" << seq_name << ".json");

    string colorImgDir = FORMAT(panoptic_path << seq_name << "/kinectImgs");
    string depthImgDir = FORMAT(panoptic_path << seq_name << "/kinectDepthImgs");

    // Output folder Path
    // Change the following if you want to save outputs on another folder
    string plyOutputDir = FORMAT(panoptic_path << seq_name << "/kinoptic_ptclouds");
    string skippedFileName = FORMAT(out_dir << "skipped_frames.txt");

	fs::ofstream ofile;
	ofile.open(skippedFileName, ios::out | ios::trunc);
	if (!ofile.is_open())
	{	
		cout << "Failed to open file: " << skippedFileName << endl;
		return -1;
	}

    if(fs::exists(plyOutputDir))
        cout << "WARNING :  " << "Directory already exists" << endl;
    else
    {
        if(fs::create_directories(plyOutputDir))
            cout << "Directory created" << endl;
        else
        {
            cout << "ERROR: " << "Directory creation failed" << endl;
            return -1;
        }
    }
    cout << "PLY files will be saved in: " << plyOutputDir << endl;

    // Other parameters
    bool bVisOutput = false;    //Turn on, if you want to visualize what's going on
    bool bRemoveFloor= false;   //Turn on, if you want to remove points from floor
    // Adjust this (0.5cm ~ 7cm), if floor points are not succesfully removed
    // Icreasing this may remove feet of people
    float floorHeightThreshold = 0.5; 
    bool bRemoveWalls = true;  //Turn on, if you want to remove points from dome surface


    // Load syncTables
    json ksync;
    loadjson(syncTableFileName, ksync);

    unordered_map<int, string> knames;
    for(int id=1; id <= NUM_CAMS; id++) 
        knames[id] = FORMAT("KINECTNODE" << id);

    json psync;
    loadjson(panopSyncTableFileName, psync); // Panoptic Sync Tables

    unordered_map<int, unordered_map<int, double>> colorUnivTime = ksync_extract_univ_time(ksync, "color", knames);
    if(colorUnivTime.size() == 0)
    {
        cout << "ERROR: " << "No color sync table found" << endl;
        return -1;
    }
    unordered_map<int, unordered_map<int, double>> depthUnivTime = ksync_extract_univ_time(ksync, "depth", knames);
    if(depthUnivTime.size() == 0)
    {
        cout << "ERROR: " << "No depth sync table found" << endl;
        return -1;
    }

    // Load Kinect Calibration File
    json kinect_calibration;
    loadjson(calibFileName, kinect_calibration); 

    json panoptic_calibration;
    loadjson(panopcalibFileName, panoptic_calibration);
    
    vector<string> panoptic_camNames = panoptic_extract_camNames(panoptic_calibration); // To search the targetCam
    if(panoptic_camNames.size() == 0)
    {
        cout << "ERROR: " << "No camera names found" << endl;
        return -1;
    }

    vector<Calibration> extrinsic_calibration(NUM_CAMS);
    panoptic_extrinsic_calibration(panoptic_calibration, kinect_calibration, extrinsic_calibration);

    for_each(hd_index_list.begin(), hd_index_list.end(), [](int &x){ x += 2; });        // This is the output frame (-2 is some weired offset in synctables)
    
    int hd_index_afterOffest;
    string out_fileName;

    // Compute Universal time
    unordered_map<int, double> hd_univ_time = psync_extract_univ_time(psync, "hd");
    if(hd_univ_time.size() == 0)
    {
        cout << "ERROR: " << "No hd sync table found" << endl;
        return -1;
    }

    time_point start, end;
    uint64_t elapsed_ms;
    start = time_now();
    int num_frames = 0;
    int frame_id = START_FRAME_ID;

    vector<Point3f> all_point3d_panopticWorld;              // point cloud output from all kinects
    vector<RGB> all_colorsv;                            // colors for point cloud 

    // vector<cv::Mat> color_images_cv2;
    // vector<cv::Mat> depth_images_cv2;
    vector<RGBD> rgbd(NUM_CAMS);
    // vector<std::pair<uint32_t, uint32_t>> image_metadata;

    /**
     * @brief device ids are always 0 to NUM_CAMS - 1
     * For panoptic, device serial number is also 0 to NUM_CAMS - 1
     * For other dataset, device serial numbers can differ, but not the device_ids
     */
    vector<uint32_t> device_ids;                            
    vector<uint32_t> device_serial_num;

    for(int idk = 1; idk <= NUM_CAMS; idk++)
    {
        uint32_t cam_serial_num = idk - 1;
        device_ids.push_back(cam_serial_num);
        device_serial_num.push_back(cam_serial_num);
    }

    if(!save_device_details(out_dir, device_ids, device_serial_num))
    {
        cout << "ERROR: " << "Device details save failed" << endl;
        return -1;
    }

    for(int idk = 1; idk <= NUM_CAMS; idk++)
    {
        string cam_serial_num = FORMAT(idk - 1);
        if(extrinsic_calibration[idk - 1].saveCalibration(cam_serial_num, out_dir, true))
            cout << "Calibration file saved for Camera ID: " << idk << endl;
        else
        {
            cout << "ERROR: " << "Calibration file save failed for Camera ID: " << idk << endl;
            return -1;
        }
    }

    for (auto hd_index : hd_index_list)
    {
        hd_index_afterOffest = hd_index - 2;    // This is the output frame (-2 is some weired offset in synctables)

        double selUnivTime = hd_univ_time[hd_index - 1];    // Remember, MATLAB index starts from 1
        // cout << "hd_index: " << hd_index << " UnivTime: " << std::fixed << setprecision(3) << selUnivTime << endl;

        cout << "============= FRAME " << frame_id << " =============" << endl;

        // Main Iteration
        bool vis_output = false;
        for(int idk = 1; idk <= NUM_CAMS; idk++)                  // Iterating 10 kinects. Change this if you want a subpart
        {
            rgbd[idk - 1].frameID = frame_id;               // This should be updated here, otherwise the frame that's lagging will have previous frame id
            rgbd[idk - 1].camID = idk - 1;                  // 0 based offset to keep similarity with our pipeline

            if(idk == 1 && bVisOutput)                      // Visualize the results from the frist kinect only. 
                vis_output = true;
            else
                vis_output = false;

            // Select corresponding frame index rgb and depth by selUnivTime
            // Note that kinects are not perfectly synchronized (it's not possible),
            // and we need to consider offset from the selcUnivTime
            
            pair<int, double> temp = kinect_univ_diff(colorUnivTime[idk], selUnivTime, 6.25);   // cindex: 0 based
            int cindex = temp.first;
            double time_distc = temp.second;

            temp = kinect_univ_diff(depthUnivTime[idk], selUnivTime, 0.0);                      // dindex: 0 based
            int dindex = temp.first;
            double time_distd = temp.second;

            // Filtering if current kinect data is far from the selected time
            // cout << "idk: " << idk << ", " << std::fixed << setprecision(4) << selUnivTime - depthUnivTime[idk][dindex] << endl;
            double diff = abs(depthUnivTime[idk][dindex] - colorUnivTime[idk][cindex]);
            if(diff > 6.5)
            {
                cout << "Skipping " << idk << ", depth-color diff " << std::fixed << setprecision(4) << diff << endl;    
                continue;
            }

            if(time_distc > 30.0 || time_distd > 17.0)
            {
                cout << "Skipping " << idk << endl;    
                continue;
            }

            string rgbFileName = FORMAT(colorImgDir << "/50_" 
                                        << setfill('0') << setw(2) << idk 
                                        << "/50_" << setfill('0') << setw(2) 
                                        << idk << "_" << setfill('0') 
                                        << setw(8) << cindex + 1 << "_transformed.png");
            string depthFileName = FORMAT(depthImgDir << "/50_" 
                                        << setfill('0') << setw(2) << idk 
                                        << "/50_" << setfill('0') << setw(2) 
                                        << idk << "_" << setfill('0') 
                                        << setw(8) << cindex + 1 << "_transformed.bin");
            
            RGB *color3D = NULL;
            Point3f *points3D = NULL;
            int n_color3D = 0;
            int n_points3D = 0;

            if(!read_color3D(rgbFileName, color3D, n_color3D))
            {
                cout << "ERROR: " << "Failed to read color3D from " << rgbFileName << endl;
                return -1;
            }

            if(!read_points3D(depthFileName, points3D, n_points3D))
            {
                cout << "ERROR: " << "Failed to read points3D from " << depthFileName << endl;
                return -1;
            }
            
            assert(n_color3D == n_points3D);

            Calibration &calib = extrinsic_calibration[idk - 1];
            cv::Mat depth;
            cv::Mat color;
            generate_points3D2images(points3D, color3D, n_points3D, calib, depth, color);
            
            rgbd[idk - 1].depth = depth;
            rgbd[idk - 1].rgb = color;       
            rgbd[idk - 1].valid = true;

            delete [] points3D;
            points3D = NULL;
            n_points3D = 0;

            delete [] color3D;
            color3D = NULL;
            n_color3D = 0;

            generate_images2points3D(depth, color, calib, points3D, color3D, n_points3D);

            transform_points3D_world(points3D, n_points3D, calib);
            // cout << "Camera " << idk << ": " << endl;

            accumulate_point3D_world(points3D, color3D, n_points3D, all_point3d_panopticWorld, all_colorsv);

            delete [] points3D;
            points3D = NULL;
            n_points3D = 0;

            delete [] color3D;
            color3D = NULL;
            n_color3D = 0;
        }

        frame_id++; // Count frames that are skipped in the middle

        if(all_point3d_panopticWorld.size() == 0)
        {
            cout << "No Point Cloud. Skip and continue ..." << endl;
            ofile << frame_id - 1 << endl;              // Write the frame id that's skipped. frame_id is already incremented 
            all_point3d_panopticWorld.clear();
            all_colorsv.clear();
            continue;
        }

        num_frames++;
        end = time_now();
        elapsed_ms = duration_ms(start, end);
        cout << "Frame rate: " << std::fixed << setprecision(2) << num_frames * 1000.0 / elapsed_ms << endl;
        // cout << "End-to-end Latency for frame number " << num_frames << " is " << elapsed_ms << " ms" << endl;

        // save images to disk. Save frames which have valid points.
        bool all_valid = true;
        for(int i = 0; i < rgbd.size(); i++)
        {
            all_valid = all_valid && rgbd[i].valid;
            all_valid = all_valid && (rgbd[i].camID >= 0);
            all_valid = all_valid && (rgbd[i].frameID >= 0);
        }

        if(all_valid)
        {
            for(int i = 0; i < rgbd.size(); i++)
            {
                if(!save_image2disk(out_dir, rgbd[i].depth, rgbd[i].rgb, rgbd[i].frameID, rgbd[i].camID))
                    cout << "WARNING: " << "Failed to save images to disk" << endl;
            }
            if(first_saved_frame_id <= 0)
                first_saved_frame_id = frame_id - 1;
            last_saved_frame_id = frame_id - 1;         // frame_id is already incremented
            cout << "Saved images for Frame ID " << last_saved_frame_id << " to disk" << endl;
            cout << "Color image dimension: " << rgbd[0].rgb.rows << " x " << rgbd[0].rgb.cols << endl;
            cout << "Depth image dimension: " << rgbd[0].depth.rows << " x " << rgbd[0].depth.cols << endl;
        }
        // for(int i = 0; i < color_images_cv2.size(); i++)
        // {
        //     cv::Mat color = color_images_cv2[i];
        //     cv::Mat depth = depth_images_cv2[i];
        //     std::pair<uint32_t, uint32_t> meta = image_metadata[i];
        //     uint32_t frame_id = meta.first;
        //     uint32_t idk = meta.second;
        //     if(!save_image2disk(out_dir, depth, color, frame_id, idk))
        //     {
        //         cout << "ERROR: " << "Failed to save images to disk" << endl;
        //         return -1;
        //     }
        // }
        // if(color_images_cv2.size() > 0)
        // {
        //     cout << "Color image dimension: " << color_images_cv2[0].rows << " x " << color_images_cv2[0].cols << endl;
        //     cout << "Depth image dimension: " << depth_images_cv2[0].rows << " x " << depth_images_cv2[0].cols << endl;
        // }

        

        // color_images_cv2.clear();
        // depth_images_cv2.clear();
        // image_metadata.clear();

        // // save to ply file
        // out_fileName = FORMAT(plyOutputDir << "/ptcloud_hd" << setfill('0') << setw(8) << hd_index_afterOffest << "_our.ply");
        // if(fs::exists(out_fileName))
        // {
        //     cout << "WARNING: " << "Already exists: " << out_fileName << endl;
        //     // continue;
        // }
        // else
        // {
        //     cout << "Saving ply file: " << out_fileName << endl;
        // }
        
        // saveToPly(out_fileName, all_point3d_panopticWorld, all_colorsv);

        all_point3d_panopticWorld.clear();
        all_colorsv.clear();
    }
}

int main()
{
	cout << "Entered Panoptic Data Preparation" << endl;

    // Processing Panoptic
    string seq_name = "160906_ian3";
    string panoptic_path = "/media/lei/Expansion/Christina/panoptic-toolbox/";
    string out_dir = FORMAT("/home/lei/data/KinectStream/panoptic_captures/" << seq_name << "/");
    string start_end_id_file = FORMAT(out_dir << "start_end_id.txt");
    
    if(!fs::exists(out_dir))
        fs::create_directories(out_dir);

    fs::ofstream ofile;
	ofile.open(start_end_id_file, ios::out | ios::trunc);
	if (!ofile.is_open())
	{	
		cout << "Failed to open file: " << start_end_id_file << endl;
		return -1;
	}

    int first_saved_frame_id = -1;
    int last_saved_frame_id = -1;

    panoptic_process(panoptic_path, seq_name, out_dir, first_saved_frame_id, last_saved_frame_id);
    assert(first_saved_frame_id >= 0 && last_saved_frame_id >= 0);
    cout << "======================================================================" << endl;

    if(first_saved_frame_id >= 0 && last_saved_frame_id >= 0)
    {
        ofile << "Start: " << first_saved_frame_id << endl;
        ofile << "End: " << last_saved_frame_id << endl;

        cout << "First saved Frame ID: " << first_saved_frame_id << endl;
        cout << "Last saved Frame ID: " << last_saved_frame_id << endl;
        cout << "NOTE: " << "Frame ID can be more than the ID of the file that throws error at the end.\nThis happens due to synchronization, e.g. Frame 500 in one camera can synchronize with frame 600 of another camera" << endl;
    }
    else
    {
        cout << "ERROR: " << "No valid frames found. Please check the input data!" << endl;
    }
    ofile.close();
    
    return 0;
}