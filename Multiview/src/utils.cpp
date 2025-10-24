//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
#include <opencv2/highgui.hpp>
#include <omp.h>
#include "utils.h"
#include "pconsts.h"

std::string PANOPTIC_DATA_PATH;
std::string USER_DATA_PATH;
std::string SEQ_NAME;
std::string QRCODE_FOLDER;
int LOG_ID;
std::string USER_TRACE_FOLDER;
std::string PTCL_STREAM_ADDRESS;
int PTCL_STREAM_PORT;
uint32_t START_FRAME;
uint32_t END_FRAME;

/**
 * Rajrup: 
 * To Do: Change this function to template later. Merge with one in calibration.cpp
 */
Point3f RotatePoint(Point3f &point, std::vector<std::vector<float>> &R)
{
	Point3f res;

	res.X = point.X * R[0][0] + point.Y * R[0][1] + point.Z * R[0][2];
	res.Y = point.X * R[1][0] + point.Y * R[1][1] + point.Z * R[1][2];
	res.Z = point.X * R[2][0] + point.Y * R[2][1] + point.Z * R[2][2];

	return res;
}

Point3f InverseRotatePoint(Point3f &point, std::vector<std::vector<float>> &R)
{
	Point3f res;

	res.X = point.X * R[0][0] + point.Y * R[1][0] + point.Z * R[2][0];
	res.Y = point.X * R[0][1] + point.Y * R[1][1] + point.Z * R[2][1];
	res.Z = point.X * R[0][2] + point.Y * R[1][2] + point.Z * R[2][2];

	return res;
}

void saveToBinary(const std::string &filename, std::vector<Point3f> &vertices, std::vector<RGB> &colors)
{
    std::ofstream fout(filename);
    uint32_t nVertices = vertices.size();
    assert(nVertices == colors.size());

//    fout << "ply\nformat binary_little_endian 1.0\n";
//    fout << "comment Created by Open3D\n";
//    fout << "element vertex " << nVertices << "\n";
//    fout << "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

    int buf_size = nVertices * (3 * sizeof(float) + 3 * sizeof(uint8_t));
    char *buf = new char[buf_size];
    int pos = 0;
    float *vertex = new float[3];
    uint8_t *color = new uint8_t[3];
    for (int i = 0; i < nVertices; i++)
    {
        // cout << to_string(colors[i].rgbRed) << endl;
        // fout << std::to_string(vertices[i].X / 1000.0f) << " "
        //     << std::to_string(vertices[i].Y / 1000.0f) << " "
        //     << std::to_string(vertices[i].Z / 1000.0f) << " "
        //     << std::to_string(colors[i].rgbRed) << " "
        //     << std::to_string(colors[i].rgbGreen) << " "
        //     << std::to_string(colors[i].rgbBlue) << "\n";
        vertex[0] = vertices[i].X / 1000.0f;
        vertex[1] = vertices[i].Y / 1000.0f;
        vertex[2] = vertices[i].Z / 1000.0f;
        color[0] = colors[i].rgbRed;
        color[1] = colors[i].rgbGreen;
        color[2] = colors[i].rgbBlue;

        memcpy(buf + pos, vertex, 3 * sizeof(float));
        pos += 3 * sizeof(float);
        memcpy(buf + pos, color, 3 * sizeof(uint8_t));
        pos += 3 * sizeof(uint8_t);
    }

    fout.write(buf, buf_size);
    fout.flush();
    fout.close();

    delete[] vertex;
    delete[] color;
    delete[] buf;
}

void saveToPly(const std::string &filename, std::vector<Point3s> &vertices, std::vector<RGB> &colors)
{
    std::ofstream fout(filename);
    uint32_t nVertices = vertices.size();
    assert(nVertices == colors.size());

    fout << "ply\nformat ascii 1.0\n";
    fout << "element vertex " << nVertices << "\n";
    fout << "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

    for (int i = 0; i < nVertices; i++)
    {
        // LOG(INFO) << to_string(colors[i].rgbRed);
        fout << std::to_string(vertices[i].X / 1000.0f) << " "
            << std::to_string(vertices[i].Y / 1000.0f) << " "
            << std::to_string(vertices[i].Z / 1000.0f) << " "
            << std::to_string(colors[i].rgbRed) << " "
            << std::to_string(colors[i].rgbGreen) << " "
            << std::to_string(colors[i].rgbBlue) << "\n";
    }
    fout.flush();
    fout.close();
}

void saveToPly(const std::string &filename, const PointCloud_t::Ptr &pclCloud)
{
    if(pcl::io::savePLYFile(filename, *pclCloud) == 0)
		LOG(INFO) << "Saved " << filename;
	else
		LOG(ERROR) << "Failed to save " << filename;
}

void saveToPlyBinary(const std::string &filename, const PointCloud_t::Ptr &pclCloud)
{
    if(pcl::io::savePLYFileBinary(filename, *pclCloud) == 0)
		LOG(INFO) << "Saved " << filename;
	else
		LOG(ERROR) << "Failed to save " << filename;
}

void saveToPly(const std::string &filename, std::vector<Point3f> &vertices, std::vector<RGB> &colors)
{
    std::ofstream fout(filename);
    uint32_t nVertices = vertices.size();
    assert(nVertices == colors.size());

    fout << "ply\nformat ascii 1.0\n";
    fout << "element vertex " << nVertices << "\n";
    fout << "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

    for (int i = 0; i < nVertices; i++)
    {
        // LOG(INFO) << to_string(colors[i].rgbRed);
        fout << std::to_string(vertices[i].X / 1000.0f) << " " 
            << std::to_string(vertices[i].Y / 1000.0f) << " " 
            << std::to_string(vertices[i].Z / 1000.0f) << " " 
            << std::to_string(colors[i].rgbRed) << " " 
            << std::to_string(colors[i].rgbGreen) << " " 
            << std::to_string(colors[i].rgbBlue) << "\n";
    }
    fout.flush();
    fout.close();
}

PointCloud_t::Ptr convertToPCL(const std::vector<Point3s> &vertices, const std::vector<RGB> &colors)
{
    uint32_t nVertices = vertices.size();
    assert(nVertices == colors.size());

    PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;
	cloud->points.resize(nVertices);

    for (uint32_t i = 0; i < nVertices; ++i)
	{
		Point_t point;

		point.x = vertices[i].X / 1000.0f;
		point.y = vertices[i].Y / 1000.0f;
		point.z = vertices[i].Z / 1000.0f;
		if (point.z == 0)
		{
			continue;
		}

		point.r = colors[i].rgbRed;
		point.g = colors[i].rgbGreen;
		point.b = colors[i].rgbBlue;
		if (point.b == 0 && point.g == 0 && point.r == 0)
		{
			continue;
		}
		cloud->points[i] = point;
	}
    return cloud;
}

PointCloud_t::Ptr convertToPCL2(const std::vector<Point3f> &vertices, const std::vector<RGB> &colors)
{
    uint32_t nVertices = vertices.size();
    assert(nVertices == colors.size());

    PointCloud_t::Ptr cloud(new PointCloud_t);
	cloud->is_dense = false;
	cloud->points.resize(nVertices);

    for (uint32_t i = 0; i < nVertices; ++i)
	{
        if(vertices[i].Z == 0)
        {
            continue;
        }

        if (colors[i].rgbRed == 0 && colors[i].rgbGreen == 0 && colors[i].rgbBlue)
		{
			continue;
		}

        cloud->points[i].x = vertices[i].X;
        cloud->points[i].y = vertices[i].Y;
        cloud->points[i].z = vertices[i].Z;
        
        cloud->points[i].r = colors[i].rgbRed;
        cloud->points[i].g = colors[i].rgbGreen;
        cloud->points[i].b = colors[i].rgbBlue;
	}
    return cloud;
}

cv::Mat color_to_opencv(const RGB *color, int width, int height, bool withAlpha)
{
    cv::Mat cv_image_with_alpha = cv::Mat(height, width, CV_8UC4, (void *)color).clone();
    if(!withAlpha)
    {
        cv::Mat cv_image_no_alpha;
        cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);
        return cv_image_no_alpha;
    }
    return cv_image_with_alpha;
}

cv::Mat depth_to_opencv(const uint16_t *depth, int width, int height)
{
    return cv::Mat(height, width, CV_16U, (void *)depth).clone();
}

cv::Mat binary_mask_to_opencv(const bool *mask, int width, int height)
{
    cv::Mat cv_mask = cv::Mat(height, width, CV_8UC1);
    for(int i = 0; i < width * height; i++)
        cv_mask.data[i] = mask[i] ? 255 : 0;
    return cv_mask;
}

// Convert a boolean array to a compressed uint8 array
uint8_t *binary_mask_to_uint8(const bool *mask, int width, int height, int &nBytes)
{
    nBytes = (width * height + 7) / 8;
    uint8_t *compressed_mask = new uint8_t[nBytes];
    memset(compressed_mask, 0, nBytes);
    // int count = 0;

#pragma omp parallel for
    for (int i = 0; i < width * height; i++)
    {
        if (mask[i])
        {
            // count++;
            compressed_mask[i / 8] |= (1 << (i % 8));
        }
    }
    // cout << "non zero in mask:" << count << endl;
    return compressed_mask;
}

// Convert a compressed uint8 array to a boolean array
bool *uint8_to_binary_mask(const uint8_t *compressed_mask, int width, int height, int &outSize)
{
    int nBytes = (width * height + 7) / 8;
    outSize = width * height;
    bool *mask = new bool[width * height];
    memset(mask, 0, width * height);

#pragma omp parallel for
    for (int i = 0; i < nBytes; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            if (compressed_mask[i] & (1 << j))
            {
                mask[i * 8 + j] = true;
            }
        }
    }
    return mask;
}

// Convert a boolean array to a compressed uint8 array
uint32_t *binary_mask_to_uint32(const bool *mask, int width, int height, int &nBytes)
{
    nBytes = (width * height + 31) / 32;
    uint32_t *compressed_mask = new uint32_t[nBytes];
    memset(compressed_mask, 0, nBytes);

// #pragma omp parallel for
    for (int i = 0; i < width * height; i++)
    {
        if (mask[i])
        {
            compressed_mask[i / 32] |= (1 << (i % 32));
        }
    }
    return compressed_mask;
}

// Convert a compressed uint8 array to a boolean array
bool *uint32_to_binary_mask(const uint32_t *compressed_mask, int width, int height, int &outSize)
{
    int nBytes = (width * height + 31) / 32;
    outSize = width * height;
    bool *mask = new bool[width * height];
    memset(mask, 0, width * height);

#pragma omp parallel for
    for (int i = 0; i < nBytes; i++)
    {
        for (int j = 0; j < 32; j++)
        {
            if (compressed_mask[i] & (1 << j))
            {
                mask[i * 32 + j] = true;
            }
        }
    }
    return mask;
}

// Convert a boolean array to a compressed uint8 array
char *binary_mask_to_char(const bool *mask, int width, int height, int &nBytes)
{
    nBytes = width * height;
    char *compressed_mask = new char[nBytes];
    memset(compressed_mask, 'f', nBytes);

#pragma omp parallel for
    for (int i = 0; i < width * height; i++)
    {
        if (mask[i])
        {
            compressed_mask[i] = 't';
        }
    }
    return compressed_mask;
}

// Convert a compressed uint8 array to a boolean array
bool *char_to_binary_mask(const char *compressed_mask, int width, int height, int &outSize)
{
    int nBytes = width * height;
    outSize = width * height;
    bool *mask = new bool[width * height];
    memset(mask, 0, width * height);

#pragma omp parallel for
    for (int i = 0; i < nBytes; i++)
    {
        if (compressed_mask[i])
        {
            mask[i] = true;
        }
    }
    return mask;
}

// Convert a boolean array to a compressed vector using RLE
vector<int> rle_binary_mask_to_vector(const bool *mask, int width, int height)
{
    vector<int> compressed_mask;

    int count = 0;
    bool curr = false;
    // always start counting 0
    for (int i = 0; i < width * height; i++)
    {
        if (mask[i] != curr) {
            compressed_mask.push_back(count);
            count = 0;
            curr = !curr;
        } else count++;
    }

    return compressed_mask;
}

// Convert a compressed vector to a boolean array using RLE
bool *rle_vector_to_binary_mask(vector<int> compressed_mask, int width, int height)
{
    bool *mask = new bool[width * height];
    memset(mask, 0, width * height);

    bool curr = false;
    int count = 0;
    for (int i = 0; i < compressed_mask.size(); i++)
    {
        for (int j=0; j<compressed_mask[i]; j++) mask[count] = curr;
        curr = !curr;
    }
    return mask;
}

string get_date()
{
	// Get current system time
    ptime timeLocal = second_clock::local_time();
	date::ymd_type ymd = timeLocal.date().year_month_day();

	return FORMAT(ymd.month << "_" << ymd.day << "_" << ymd.year);
}

void show_color_image(cv::Mat &image)
{
    cv::imshow("Color Image", image);
    cv::waitKey(0);
}

void show_depth_image(cv::Mat &image)
{
    double min;
    double max;
    cv::minMaxIdx(image, &min, &max);
    cv::Mat adj_image;
    cv::convertScaleAbs(image, adj_image, 255 / max);
    cv::imshow("Depth Image", adj_image);
    cv::waitKey(0);
}

void save_color(uint8_t *color_image, int h, int w, string path) 
{
    cv::Mat color_image_cv2(h, w, CV_8UC4, color_image);
    if(!cv::imwrite(path, color_image_cv2))
        LOG(ERROR) << "Failed to write color image: " << path;
    else 
        LOG(INFO) << "Color image saved to " << path;
}

void save_color_with_qr(uint8_t *color_image, int h, int w, string path, int frame_number)
{
    cv::Mat color_image_cv2(h, w, CV_8UC4, color_image);
    int offset_x = 20;
    int offset_y = 20;
    cv::Mat qrcode_extract = color_image_cv2(cv::Range(offset_x, offset_x+82), cv::Range(offset_y, offset_y+82));

    vector<cv::Mat> channels(color_image_cv2.channels());
    cv::split(qrcode_extract, channels);

    if(!cv::imwrite(FORMAT(path << frame_number << "_color.png") , color_image_cv2))
        LOG(ERROR) << "Failed to write color image: " << path;
    else 
        LOG(INFO) << "Color image saved to " << path;

    for(int i=0; i<channels.size(); i++)
    {
        string channel_path = FORMAT(path << frame_number << "_qr" << i << ".png");
        if(!cv::imwrite(channel_path, channels[i]))
            LOG(ERROR) << "Failed to write color image: " << channel_path;
        else 
            LOG(INFO) << "Color image saved to " << channel_path;
    }
}

void save_depth(uint8_t *colorized_depth_image, int h, int w, string path, bool withAlpha) 
{
    cv::Mat depth_image_cv2(h, w, CV_8UC4, colorized_depth_image);
    cv::cvtColor(depth_image_cv2, depth_image_cv2, cv::COLOR_BGRA2BGR);
    if(withAlpha)
        cv::cvtColor(depth_image_cv2, depth_image_cv2, cv::COLOR_BGR2BGRA);

    if(!cv::imwrite(path, depth_image_cv2))
        LOG(ERROR) << "Failed to write depth image: " << path;
    else 
        LOG(INFO) << "Color image saved to " << path;
}

bool loadjson(const string &file_path, json &j)
{
    std::ifstream ifs(file_path);
    if(!ifs.is_open())
    {
        LOG(FATAL) << "Failed to open file: " << file_path;
        return false;
    }
    LOG(INFO) << "Loading json file: " << file_path;
    j = json::parse(ifs);
    return true;
}

void parse_config(const string &config_file)
{
    json config;
    loadjson(config_file, config);

    try
    {
        
        SEQ_NAME = config["seq_name"].get<string>();
        PANOPTIC_DATA_PATH = config["panoptic_path"].get<string>();
        USER_DATA_PATH = config["user_trace_path"].get<string>();
        LOG_ID = config["log_id"].get<int>();
        USER_TRACE_FOLDER = config["user_trace_folder"].get<string>();
        QRCODE_FOLDER = config["qrcode_folder"].get<string>();
        // PTCL_STREAM_ADDRESS = config["ptcl_streamer_ip_address"].get<string>();
        // PTCL_STREAM_PORT = config["ptcl_streamer_port"].get<int>();

        LOG(INFO) << "Config file parsed successfully!";
        LOG(INFO) << "CONFIG_FILE: " << config_file;
        LOG(INFO) << "SEQ_NAME: " << SEQ_NAME;
        LOG(INFO) << "PANOPTIC_DATA_PATH: " << PANOPTIC_DATA_PATH;
        LOG(INFO) << "USER_DATA_PATH: " << USER_DATA_PATH;
        LOG(INFO) << "LOG_ID: " << LOG_ID;
        LOG(INFO) << "USER_TRACE_FOLDER: " << USER_TRACE_FOLDER;
        // LOG(INFO) << "PTCL_STREAM_ADDRESS: " << PTCL_STREAM_ADDRESS;
        // LOG(INFO) << "PTCL_STREAM_PORT: " << PTCL_STREAM_PORT;
    }
    catch (json::exception &e)
    {
        LOG(FATAL) << "Error parsing config file: " << e.what() << ", file path: " << config_file;
    }

    json j;
    string start_end_file = FORMAT(PANOPTIC_DATA_PATH << config["seq_start_end"].get<string>());
    loadjson(start_end_file, j);

    try
    {
        START_FRAME = j[SEQ_NAME]["start_idx"].get<uint32_t>();
        END_FRAME = j[SEQ_NAME]["end_idx"].get<uint32_t>();
    }
    catch (json::exception &e)
    {
        LOG(FATAL) << "Error parsing start and end frame: " << e.what() << ", file path: " << start_end_file;
    }
}

void parse_zstd_config(const string &config_file, struct zstd_options &config)
{
    json j;
    loadjson(config_file, j);

    config.compression_level = j["compression_level"].get<int>();
}

void parse_draco_config(const string &config_file, struct draco_options &config)
{
    json j;
    loadjson(config_file, j);

    config.cl = j["compression_level"].get<int>();
    config.qp = j["pos_quantization_bits"].get<int>();
}

bool create_folder(const string &folder)
{
    if (!fs::exists(folder))
    {
        if (!fs::create_directories(folder))
        {
            LOG(ERROR) << "Failed to create folder: " << folder;
            return false;
        }
        else
            LOG(INFO) << "Folder created: " << folder;
    }
    return true;
}