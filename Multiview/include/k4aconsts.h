// Declare all the constants here
#pragma once
#include <cstdint>
#include <k4a/k4a.hpp>

#define DUMMY_CAPTURES 100

// #define CALIBRATION_INT_FILENAME "intrinsics"
// #define DEVICE_FILE_NAME "devices"
#define DEFAULT_COLOR_MODE K4A_COLOR_RESOLUTION_720P
#define DEFAULT_DEPTH_MODE K4A_DEPTH_MODE_WFOV_UNBINNED


// Change this data path to your own folder where kinect captures are present
#define DATA_PATH "/home/lei/data/KinectStream/extended/"

// Allowing at least 160 microseconds between depth cameras should ensure they do not interfere with one another.
constexpr uint32_t MIN_TIME_BETWEEN_DEPTH_CAMERA_PICTURES_USEC = 160;

// This is the maximum difference between when we expected an image's timestamp to be and when it actually occurred.

/**
 * Rajrup
 * To Do: Check how to reduce this? Probably using active USB extenders
 */
constexpr std::chrono::microseconds MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP(100);

constexpr int64_t WAIT_FOR_SYNCHRONIZED_CAPTURE_TIMEOUT = 60000;
