#pragma once
#include <k4a/k4a.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "calibration.h"

// GREEN SCREEN Example
// ideally, we could generalize this to many OpenCV types
cv::Mat color_to_opencv(const k4a::image& im, bool withAlpha=true);
cv::Mat depth_to_opencv(const k4a::image &im);
k4a::image opencv_to_color(const cv::Mat &im);
k4a::image opencv_to_depth(const cv::Mat &im);