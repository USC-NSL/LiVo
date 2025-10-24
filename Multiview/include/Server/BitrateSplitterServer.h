#pragma once
#include "utils.h"

using namespace std;

class BitrateSplitterServer
{
public:
    BitrateSplitterServer(std::string &gt_path);
    ~BitrateSplitterServer();
    void split_bitrate();
    float calc_color_rmse(cv::Mat &color_frame, uint32_t frame_id);
    float calc_depth_rmse(cv::Mat &depth_frame, uint32_t frame_id);


private:

    bool load_gt_color_parallel(cv::Mat &color_frame, uint32_t frame_id);
    bool load_gt_depth_parallel(cv::Mat &color_frame, uint32_t frame_id);
    
    std::string gt_path;
    uint32_t color_bitrate;
    uint32_t depth_bitrate;
    float depth2color_split;

};