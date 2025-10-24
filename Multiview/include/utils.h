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
#pragma once
#include <cstdio>
#include <string>
#include <vector>
#include <iostream>
#include <vector>
#include <cstdint>
#include <chrono>
#include <fstream>
#include <unistd.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "consts.h"
#include "frustum.h"
#include "timer.h"

// Boost
#include <boost/exception/all.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/filesystem.hpp>

// Boost Stat Libraries
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

using namespace std;
using namespace Eigen;
using json = nlohmann::json;
namespace ba = boost::accumulators;
namespace fs = boost::filesystem;

// default compression level = 3, values can be from 0 to 9 (highest). Highest compression level takes more time to compress.
static const vector<int> compression_params{cv::IMWRITE_PNG_COMPRESSION};

enum SYNC_STATE 
{ 
	Subordinate, 
	Master, 
	Standalone 
};

enum CALIBRATION_PARAM
{
	RW,
	RH,
	CX,
	CY,
	FX,
	FY
};

typedef struct Point3f
{
	Point3f()
	{
		this->X = 0;
		this->Y = 0;
		this->Z = 0;
		this->Invalid = false;
	}
	Point3f(float X, float Y, float Z, bool invalid)
	{
		this->X = X;
		this->Y = Y;
		this->Z = Z;
		this->Invalid = invalid;
	}
	Point3f(float X, float Y, float Z)
	{
		this->X = X;
		this->Y = Y;
		this->Z = Z;
		this->Invalid = false;
	}
	float X;
	float Y;
	float Z;
	bool Invalid = false;
} Point3f;

// It is Point3f struct wihout Invalid field
typedef struct Point3f_S
{
	Point3f_S()
	{
		this->x = 0;
		this->y = 0;
		this->z = 0;
	}
	Point3f_S(float x, float y, float z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
	float x;
	float y;
	float z;
} Point3f_S;

typedef struct Point3s
{
	Point3s()
	{
		this->X = 0;
		this->Y = 0;
		this->Z = 0;
	}
	Point3s(short X, short Y, short Z)
	{
		this->X = X;
		this->Y = Y;
		this->Z = Z;
	}
	//meters to milimeters
	Point3s(Point3f &other)
	{
		this->X = static_cast<short>(1000 * other.X);
		this->Y = static_cast<short>(1000 * other.Y);
		this->Z = static_cast<short>(1000 * other.Z);
	}
	short X;
	short Y;
	short Z;
} Point3s;

typedef struct Point2f
{
	Point2f()
	{
		this->X = 0;
		this->Y = 0;
	}
	Point2f(float X, float Y)
	{
		this->X = X;
		this->Y = Y;
	}
	float X;
	float Y;
} Point2f;


// TODO: Rajrup: Change alpha channel to 255, 0 is fully transparent
typedef struct RGB
{
	uint8_t    rgbBlue;
	uint8_t    rgbGreen;
	uint8_t    rgbRed;
	uint8_t    rgbReserved;

	RGB()
	{
		this->rgbBlue = 0;
		this->rgbGreen = 0;
		this->rgbRed = 0;
		this->rgbReserved = 255;	// 255 is fully opaque
	}
	
	RGB(uint8_t r, uint8_t g, uint8_t b)
	{
		rgbBlue = b;
		rgbGreen = g;
		rgbRed = r;
		rgbReserved = 255;	// 255 is fully opaque
	}
	RGB(const RGB &other)
	{
		rgbBlue = other.rgbBlue;
		rgbGreen = other.rgbGreen;
		rgbRed = other.rgbRed;
		rgbReserved = other.rgbReserved;
		// rgbReserved = 255;	// 255 is fully opaque
	}
} RGB;

// It is RGB struct without reserved byte
typedef struct RGB_S
{
	RGB_S()
	{
		this->b = 0;
		this->g = 0;
		this->r = 0;
	}

	RGB_S(uint8_t r, uint8_t g, uint8_t b)
	{
		this->b = b;
		this->g = g;
		this->r = r;
	}
	
	uint8_t b;
	uint8_t g;
	uint8_t r;
} RGB_S;

typedef struct PointInfo
{
	PointInfo()
	{
		this->viewID = -1;
		this->depthIndex = 0;
	}
	
	PointInfo(int viewID, int depthIndex)
	{
		this->viewID = viewID;
		this->depthIndex = depthIndex;
	}

	int8_t	viewID;
	uint32_t depthIndex;
} PointInfo;

typedef struct Point3fV
{
	Vector3f xyz;
	bool valid;

	Point3fV()
	{
		this->xyz = Vector3f(0.0, 0.0, 0.0);
		this->valid = false;
	}
	Point3fV(float X, float Y, float Z)
	{
		this->xyz = Vector3f(X, Y, Z);
		this->valid = true;
	}
	Point3fV(float X, float Y, float Z, bool valid)
	{
		this->xyz = Vector3f(X, Y, Z);
		this->valid = valid;
	}
} Point3fV;

typedef struct PclData
{
	PointCloud_t::Ptr pcl_object;
	uint32_t frame_id;

	PclData()
	{
		pcl_object = PointCloud_t::Ptr(new PointCloud_t);
		frame_id = 0;
	}

	~PclData()
	{
		pcl_object->clear();
	}
} PclData;

using Open3D_t = vector<Eigen::Vector3d>;

typedef struct O3DData
{
	Open3D_t points;
	Open3D_t colors;
	uint32_t frame_id;

	O3DData()
	{
		frame_id = 0;
	}

	~O3DData()
	{
		points.clear();
		colors.clear();
	}
} O3DData;

typedef struct xy_table
{
	int width, height;
	float *x_table, *y_table;  

	xy_table()
	{
		x_table = NULL;
		y_table = NULL;
	}

	~xy_table()
	{
		if (x_table != NULL)
		{
			delete[] x_table;
		}
		if (y_table != NULL)
		{
			delete[] y_table;
		}
	}

	xy_table(const xy_table &table)
	{
		width = table.width;
		height = table.height;

		if(table.x_table != NULL)
			delete [] x_table;
		if(table.y_table != NULL)
			delete [] y_table;
		
		x_table = new float[width * height];
		y_table = new float[width * height];
		memcpy(x_table, table.x_table, width * height * sizeof(float));
		memcpy(y_table, table.y_table, width * height * sizeof(float));
	}

	xy_table(int w, int h)
	{
		width = w;
		height = h;
		x_table = new float[width * height];
		y_table = new float[width * height];
	}
} xy_table_t;

typedef struct UserData
{
	int frameID;
	uint32_t ts_orig; 	// in ms
	uint32_t ts_pred; 	// in ms
	Vector3f pos;	// (x, y, z) in meters
	Vector3f rot;	// (r_x, r_y, r_z) in degrees
	Vector4f quat;	// (q_x, q_y, q_z, q_w)
	Frustum frustum;
	Vector3f lookat;	// Lookat vector (l_x, l_y, l_z)
	Vector3f up;		// Up vector (u_x, u_y, u_z)

	UserData()
	{
		frameID = -1;
		ts_orig = 0;
		ts_pred = 0;
		pos = Vector3f(0.0, 0.0, 0.0);
		rot = Vector3f(0.0, 0.0, 0.0);
		quat = Vector4f(0.0, 0.0, 0.0, 0.0);
		lookat = Vector3f(0.0, 0.0, 0.0);
		up = Vector3f(0.0, 0.0, 0.0);
	}

	UserData(const UserData &other)
	{
		frameID = other.frameID;
		ts_orig = other.ts_orig;
		ts_pred = other.ts_pred;
		pos = other.pos;
		rot = other.rot;
		quat = other.quat;
		lookat = other.lookat;
		up = other.up;
		frustum = other.frustum;
	}
} UserData;

class Accumulator
{
public:
	Accumulator()
	{
		reset();
	}

	void reset()
	{
		data = {};
	}

	void add(double x)
	{
		data(x);
	}

	double count()
	{
		return ba::count(data);
	}

	double mean()
	{
		return ba::mean(data);
	}

	double var()
	{
		return ba::variance(data);
	}

	double median()
	{
		return ba::median(data);
	}

	double std()
	{
		return sqrt(var());
	}

private:
	ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance, ba::tag::median, ba::tag::count, ba::tag::sum>> data;
};

typedef struct zstd_options
{
	int compression_level;
	zstd_options() : compression_level(2) {};
} zstd_options;

typedef struct draco_options
{
	int cl; 		// compression level - compression_level
	int qp;			// quantization bits for position - pos_quantization_bits. Only cl and qp are used for point cloud compression in Draco.
	draco_options() : cl(7), qp(11) {}; // Default values obtained from https://github.com/google/draco/blob/9f856abaafb4b39f1f013763ff061522e0261c6f/src/draco/tools/draco_encoder.cc#L46
	draco_options(const draco_options &other) : cl(other.cl), qp(other.qp) {};
} draco_options;

Point3f RotatePoint(Point3f &point, std::vector<std::vector<float>> &R);
Point3f InverseRotatePoint(Point3f &point, std::vector<std::vector<float>> &R);

// To Do: Rajrup: Later all template for these functions
void saveToBinary(const std::string &filename, std::vector<Point3f> &vertices, std::vector<RGB> &colors);
void saveToPly(const std::string &filename, std::vector<Point3s> &vertices, std::vector<RGB> &colors);
void saveToPly(const std::string &filename, std::vector<Point3f> &vertices, std::vector<RGB> &colors);
void saveToPly(const std::string &filename, const PointCloud_t::Ptr &pclCloud);
void saveToPlyBinary(const std::string &filename, const PointCloud_t::Ptr &pclCloud);
PointCloud_t::Ptr convertToPCL(const std::vector<Point3s> &vertices, const std::vector<RGB> &colors);
PointCloud_t::Ptr convertToPCL2(const std::vector<Point3f> &vertices, const std::vector<RGB> &colors);

void show_color_image(cv::Mat &image);
void show_depth_image(cv::Mat &image);

void save_color(uint8_t *color_image, int h, int w, std::string path);
void save_color_with_qr(uint8_t *color_image, int h, int w, std::string path, int frame_id);
void save_depth(uint8_t *colorized_depth_image, int h, int w, std::string path, bool withAlpha=false);

cv::Mat color_to_opencv(const RGB *color, int width, int height, bool withAlpha=true);
cv::Mat depth_to_opencv(const uint16_t *depth, int width, int height);
cv::Mat binary_mask_to_opencv(const bool *mask, int width, int height);
uint8_t *binary_mask_to_uint8(const bool *mask, int width, int height, int &nBytes);
bool *uint8_to_binary_mask(const uint8_t *compressed_mask, int width, int height, int &outSize);
uint32_t *binary_mask_to_uint32(const bool *mask, int width, int height, int &nBytes);
bool *uint32_to_binary_mask(const uint32_t *compressed_mask, int width, int height, int &outSize);
char *binary_mask_to_char(const bool *mask, int width, int height, int &nBytes);
bool *char_to_binary_mask(const char *compressed_mask, int width, int height, int &outSize);

vector<int> rle_binary_mask_to_vector(const bool *mask, int width, int height);
bool *rle_vector_to_binary_mask(vector<int> compressed_mask, int width, int height);

// Other Utility Functions
std::string get_date();
bool create_folder(const std::string &folder);

bool loadjson(const std::string &file_path, json &j);
void parse_config(const std::string &config_file);
void parse_zstd_config(const std::string &config_file, struct zstd_options &config);
void parse_draco_config(const std::string &config_file, struct draco_options &config);
