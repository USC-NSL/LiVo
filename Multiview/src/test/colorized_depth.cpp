// Quantize depth images using the following method
// https://dev.intelrealsense.com/docs/depth-image-compression-by-colorization-for-intel-realsense-depth-cameras

#include <iostream>
#include <iomanip>
#include <vector>
#include <k4a/k4a.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <boost/filesystem/fstream.hpp>
#include "k4autils.h"
#include "utils.h"
#include "k4aconsts.h"
#include <omp.h>

using namespace std;

// Assuming depth in mm
#define MIN_DEPTH 0.00001f
#define MAX_DEPTH 5000.0f
#define DEPTH_UNITS 1.0f // mm
#define COLOR_LEVELS 1529.0f

// Converts color to hue within 1529 levels
uint16_t RGB_to_D(const RGB &color)
{
	uint8_t r = color.rgbRed;
	uint8_t g = color.rgbGreen;
	uint8_t b = color.rgbBlue;

    // conversion from RGB color to quantized depth value
    if (b + g + r < 255)
    {
		return 0;
    }
    else if (r >= g && r >= b)
    {
        if (g >= b)
        {   
        	return g - b;
        }
        else
        {
            return (g - b) + 1530;
        }
    }
    else if (g >= r && g >= b)
    {
        return b - r + 510;
    }
    else if (b >= g && b >= r)
	{
        return r - g + 1020;
    }
	return 0;
}

void D_to_RGB(const uint16_t &depth, RGB &color)
{
    uint8_t r, g, b;
	uint16_t d = depth;

	// Red
	if((d >= 0 && d <= 255) || (d > 1275 && d <= 1529))
		r = 255;
	else if(d > 255 && d <= 510)
		r = 510 - d;
	else if(d > 510 && d <= 1020)
		r = 0;
	else if(d > 1020 && d <= 1275)
		r = d - 1020;
	else
	{
		cout << "R - Error: invalid depth value " << uint32_t(d) << endl;
		r = 0;
	}

	// Green
	if(d >= 0 && d <= 255)
		g = d;
	else if(d > 255 && d <= 765)
		g = 255;
	else if(d > 765 && d <= 1020)
		g = 1020 - d;
	else if(d > 1020 && d <= 1529)
		g = 0;
	else
	{
		cout << "G - Error: invalid depth value " << uint32_t(d) << endl;
		g = 0;
	}

	// Blue
	if(d >= 0 && d <= 510)
		b = 0;
	else if(d > 510 && d <= 765)
		b = d - 510;
	else if(d > 765 && d <= 1275)
		b = 255;
	else if(d > 1275 && d <= 1529)
		b = 1530 - d;
	else
	{
		cout << "B - Error: invalid depth value " << uint32_t(d) << endl;
		b = 0;
	}
	color.rgbRed = r;
	color.rgbGreen = g;
	color.rgbBlue = b;
}

template<typename F>
void make_rgb_data(const uint16_t *depth_data, RGB *rgb_data, int width, int height, F coloring_func)
{
#pragma omp parallel for
	for (auto i = 0; i < width * height; ++i)
	{
		auto d = coloring_func(float(depth_data[i]));
		D_to_RGB(d, rgb_data[i]);
	}
}

/**
 * @brief Converts depth image to RGB image
 * 
 * @param depth Depth image
 * @param h Height of depth image
 * @param w Width of depth image
 * @param colorized_depth Colorized depth image in RGB (also has alpha channel) with same dimension. 
 * @param is_inverse True: inverse colorization, False: uniform colorization
 */
void depth_to_color(const uint16_t *depth, int h, int w, RGB *colorized_depth, const bool is_inverse)
{
	float _min_depth = MIN_DEPTH;
	float _max_depth = MAX_DEPTH;
	float _depth_units = DEPTH_UNITS;

	if (is_inverse) // Disparity
	{
		float max = (1.0f / _min_depth) * _depth_units + 0.5f;
		float min = (1.0f / _max_depth) * _depth_units + 0.5f;
		auto coloring_function = [&](float data) -> uint16_t
		{
			if (data == 0.0f)
				data = min;
			float disp_value = (1.0f / data) * _depth_units + 0.5f;
			return uint16_t(((disp_value - min) / (max - min)) * COLOR_LEVELS);
		};
		make_rgb_data(depth, colorized_depth, w, h, coloring_function);
	}
	else
	{
		float min = _min_depth;
		float max = _max_depth;
		auto coloring_function = [&](float data) -> uint16_t
		{
			if (min >= max) return 0;
			if (data >= max) return COLOR_LEVELS;
			return uint16_t(((data * _depth_units - min) / (max - min)) * COLOR_LEVELS);
		};
		make_rgb_data(depth, colorized_depth, w, h, coloring_function);
	}
}

/**
 * @brief Converts RGB image to depth image
 * 
 * @param colorized_depth RGB image (also has alpha channel).
 * @param h Height of RGB image
 * @param w Width of RGB image
 * @param depth Recovered depth image with same dimension.
 * @param is_inverse True: inverse colorization, False: uniform colorization
 */
void color_to_depth(const RGB *colorized_depth, int h, int w, uint16_t *depth, const bool is_inverse)
{
	float _min_depth = MIN_DEPTH; // to avoid depth inversion, it’s offset from 0.3 to 0.29. Please see Figure 7
	float _max_depth = MAX_DEPTH;
	float _depth_units = DEPTH_UNITS;

	uint16_t hue_value = 0; // from 0 to 255 * 6 - 1 = 0-1529 by Hue colorization

	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			int idx = i * w + j;

			hue_value = RGB_to_D(colorized_depth[idx]);

			if(hue_value > 0)
			{
				if(is_inverse)
				{
					float max = 1.0f / _min_depth;
					float min = 1.0f / _max_depth;
					float disp_value = min + (max - min) * hue_value / COLOR_LEVELS;
					depth[idx] = static_cast<uint16_t>((1.0f / disp_value) / _depth_units + 0.5f);
				}
				else
				{
					float min = _min_depth;
					float max = _max_depth;
					float d_value = min + (max - min) * hue_value / COLOR_LEVELS;
					depth[idx] = static_cast<uint16_t>(d_value * _depth_units + 0.5f);
				}
			}
			else
			{
				depth[idx] = 0;
			}
		}
	}
}

void view_colorized_depth(const RGB *colorized_depth, int h, int w)
{
	cv::Mat colorized_depth_mat(h, w, CV_8UC4);
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			colorized_depth_mat.at<cv::Vec4b>(i, j) = cv::Vec4b(colorized_depth[j].rgbBlue, colorized_depth[j].rgbGreen, colorized_depth[j].rgbRed, colorized_depth[j].rgbReserved);
		}
	}
	cv::imshow("colorized depth map", colorized_depth_mat);
	cv::waitKey(0); // wait for key press
}

int view_colorized_depth_map()
{
	uint16_t color_levels = static_cast<uint16_t>(COLOR_LEVELS);
	uint16_t d, d_recover = 0;
	const int CW = 8;

	cout << setw(3) << "  " << '|' << setw(CW) << 'D' << '|' <<  setw(CW) << 'R' << '|' << setw(CW) << 'G' << '|' << setw(CW) << 'B' << '|' << setw(CW) << "D'" << '|' << endl;
	cout << "-------------------------------------------------------------------" << endl;

	int mismatch_cnt = 0;
	uint16_t *depth = new uint16_t[color_levels+1];
	RGB *colorized_depth = new RGB[color_levels+1];

	for(d = 0; d <= color_levels; d++)
	{
		RGB color;
		D_to_RGB(d, color);
		depth[d] = d;
		colorized_depth[d] = color;

		d_recover = RGB_to_D(color);
		uint32_t r = static_cast<uint32_t>(color.rgbRed);
		uint32_t g = static_cast<uint32_t>(color.rgbGreen);
		uint32_t b = static_cast<uint32_t>(color.rgbBlue);

		uint32_t d_ = static_cast<uint32_t>(d);
		uint32_t d_recover_ = static_cast<uint32_t>(d_recover); 

		if(d_ != d_recover_)
		{
			mismatch_cnt++;
			cout << setw(3) << "**" << '|' << setw(CW) << d_ << '|' << setw(CW) << r << '|' << setw(CW) << g << '|' << setw(CW) << b << '|' << setw(CW) << d_recover_ << '|' << endl;
		}
		else
			cout << setw(3) << "  " << '|' << setw(CW) << d_ << '|' <<  setw(CW) << r << '|' << setw(CW) << g << '|' << setw(CW) << b << '|' << setw(CW) << d_recover_ << '|' << endl;
	}
	cout << "MISMATCH COUNT: " << mismatch_cnt << endl;

	view_colorized_depth(colorized_depth, 100, color_levels+1);
}

// Calculates MAE between two depth image and recovers depth image after colorization. Only calulates MAE for pixels with depth value > 0.
float calc_depth_error(const uint16_t *depth, const uint16_t *depth_recover, int h, int w)
{
	float error = 0.0f;
	int valid_depth = 0;
	int invalid_depth = 0;
	for(int i = 0; i < h; i++)
	{
		for(int j = 0; j < w; j++)
		{
			if (depth[i*w+j] != 0)
			{
				error += abs(depth[i*w+j] - depth_recover[i*w+j]);
				valid_depth++;
			}
			else
				invalid_depth++;
		}
	}
	// cout << "valid depth: " << valid_depth << endl;
	// cout << "invalid depth: " << invalid_depth << endl;
	float mean_error = error / float(valid_depth);
	return mean_error;
}

// int main()
// {
// 	// ----------------------------DEPTH TO COLOR MAP-----------------------
// 	view_colorized_depth_map();

// 	// --------------------------------SETUP--------------------------------
// 	string workdir = "/data/Dropbox/Project/VolumetricVideo/KinectStream/";
//     string date = "Feb_3_2022";
//     // string path = FORMAT(DATA_PATH << date << "/views/"); // RAJRUP: Save to SSD
// 	string path = FORMAT(DATA_PATH << date << "/"); // RAJRUP: Save to SSD
// 	int frame_number = 1;
// 	int view_number = 2;

// 	// --------------------------------LOAD DATA--------------------------------
//     // Load depth image from disk
//     // string dimg_path = FORMAT(path << frame_number << "/depth_" << view_number << ".png");
// 	string dimg_path = FORMAT(path << frame_number << "_depth_" << view_number << ".png");
// 	cv::Mat dimg_mat = cv::imread(dimg_path, cv::IMREAD_ANYDEPTH); // cv::IMREAD_ANYDEPTH for depth image
// 	cout << "CV dimg_mat.rows: " << dimg_mat.rows << endl;
// 	cout << "CV dimg_mat.cols: " << dimg_mat.cols << endl;
// 	cout << "CV dimg_mat.channels: " << dimg_mat.channels() << endl;
// 	cout << "CV dimg_size: " << dimg_mat.rows * dimg_mat.cols * dimg_mat.channels() << endl;

// 	cv::imshow("Depth Original", dimg_mat);
// 	cv::waitKey(0); // Wait for a keystroke in the window

// 	// Convert opencv image to Kinect image
// 	k4a::image dimg_kinect = opencv_to_depth(dimg_mat);
// 	int dimg_size = dimg_kinect.get_size();
// 	int dimg_h = dimg_kinect.get_height_pixels();
// 	int dimg_w = dimg_kinect.get_width_pixels();
// 	cout << "K4A dimg_size: " << dimg_size << endl;
// 	cout << "K4A dimg_height: " << dimg_h << endl;
// 	cout << "K4A dimg_width: " << dimg_w << endl;

// 	// Colorize depth using quantization (levels 0-1529). 
// 	bool is_inverse = false; // true for disparity, false for uniform quantization
// 	RGB *colorized_depth = new RGB[dimg_size];
// 	uint16_t *depth = (uint16_t *)(void *)dimg_kinect.get_buffer();
// 	depth_to_color(depth, dimg_h, dimg_w, colorized_depth, is_inverse);

// 	// Convert to opencv image for debuggging
// 	uint8_t *color_buf = reinterpret_cast<uint8_t *>(colorized_depth);
// 	cv::Mat cv_cimg(dimg_h, dimg_w, CV_8UC4, color_buf);

// 	cv::imshow("Colorized Depth", cv_cimg);
// 	cv::waitKey(0); // Wait for a keystroke in the window

// 	cv::cvtColor(cv_cimg, cv_cimg, cv::COLOR_BGRA2BGR);
	
// 	cv::imwrite(FORMAT(workdir << "out/colorized_depth_" << view_number << ".png"), cv_cimg, compression_params);

// 	// Recover depth from color image
// 	uint16_t *depth_recover = new uint16_t[dimg_size];
// 	color_to_depth(colorized_depth, dimg_h, dimg_w, depth_recover, is_inverse);

// 	// Calculate error
// 	cout << "Depth Error: " << calc_depth_error(depth, depth_recover, dimg_h, dimg_w) << endl;

// 	// Convert to opencv image for debuggging
// 	cv::Mat cv_dimg(dimg_h, dimg_w,	CV_16U, depth_recover);
// 	cv::imshow("Depth Recover", cv_dimg);
// 	cv::waitKey(0); // Wait for a keystroke in the window

// 	cv::imwrite(FORMAT(workdir << "out/depth_recovered_" << view_number << ".png"), cv_dimg, compression_params);

// 	return 0;
// }