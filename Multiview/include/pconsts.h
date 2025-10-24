// Declare all the constants for panoptic here

#pragma once
#include <cstdint>

#define DUMMY_CAPTURES 100

// Change this data path to your own folder where kinect captures are present
// #define PANOPTIC_DATA_PATH "/home/lei/data/KinectStream/panoptic_captures/"

// Panoptic global constants
// Defined in utils.cpp
extern std::string PANOPTIC_DATA_PATH;
extern std::string USER_DATA_PATH;
extern std::string SEQ_NAME;
extern int LOG_ID;
extern std::string USER_TRACE_FOLDER;