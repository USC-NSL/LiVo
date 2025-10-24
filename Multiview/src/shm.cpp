#include <iostream>
#include <k4a/k4a.hpp>

#include "shm.h"
#include "frameInfo_generated.h"
#include "k4autils.h"
#include "k4aconsts.h"
#include "colorized_depth.h"

using namespace std;
using namespace boost::interprocess;
using namespace Multiview::Frame;

void destroy_shm(const char* name)
{
	shared_memory_object::remove(name);
	cout << "Destroyed shared memory " << name << endl;
}

void create_shm(const char* name, size_t size, mapped_region& region)
{
	//Create a shared memory object
	shared_memory_object shm_object(open_or_create, name, read_write);

	//Set size
	shm_object.truncate(size);

	//Map the whole shared memory in this process
	region = mapped_region(shm_object, read_write);

	cout << "Created shared memory " << name << " with size " << size << endl;
}

uint32_t get_shm_size(const string &path, string _type)
{
    // Read a frame from the directory pointed to by path

    // Reade 1st frame and 1st view. All images have same diemensions.
    int frame_number = 100;
	int view_number = 0;

    // string img_path = FORMAT(path << frame_number << "/color_" << view_number << ".png");
    string img_path;
    if(_type == "c") {
        img_path = path + to_string(frame_number) + "_color_" + to_string(view_number) + ".png";   
    } else if (_type == "d") {
        img_path = path + to_string(frame_number) + "_depth_" + to_string(view_number) + ".png";           
    }

    if(_type == "c")
    {
        cv::Mat img_mat = cv::imread(img_path, cv::IMREAD_UNCHANGED);
        // k4a::image img_kinect = opencv_to_color(img_mat);
        // return img_kinect.get_size();
        return img_mat.rows * img_mat.cols * img_mat.channels();
    }
    else if (_type == "d")
    {   
        cv::Mat img_mat = cv::imread(img_path, cv::IMREAD_ANYDEPTH);

        k4a::image dimg_kinect = opencv_to_depth(img_mat);
        int dimg_size = dimg_kinect.get_size();
        int dimg_h = dimg_kinect.get_height_pixels();
        int dimg_w = dimg_kinect.get_width_pixels();

        bool is_inverse = false; // true for disparity, false for uniform quantization
        RGB *colorized_depth = new RGB[dimg_size];
        uint16_t *depth = (uint16_t *)(void *)dimg_kinect.get_buffer();
        depth_to_color(depth, dimg_h, dimg_w, colorized_depth, is_inverse);

        uint8_t *color_buf = reinterpret_cast<uint8_t *>(colorized_depth);
        cv::Mat cv_cimg(dimg_h, dimg_w, CV_8UC4, color_buf);
        // cv::cvtColor(cv_cimg, cv_cimg, cv::COLOR_BGRA2RGB);
        return cv_cimg.rows * cv_cimg.cols * cv_cimg.channels();
    }
}