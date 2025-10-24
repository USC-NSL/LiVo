#include "Server/BitrateSplitterServer.h"
#include "consts.h"
#include <omp.h>
#include <vector>

BitrateSplitterServer::BitrateSplitterServer(std::string &gt_path) : gt_path(gt_path)
{
    color_bitrate = 0;
    depth_bitrate = 0;
    depth2color_split = 0.0f;
}

BitrateSplitterServer::~BitrateSplitterServer()
{
}

float BitrateSplitterServer::calc_color_rmse(cv::Mat &dist_color_frame, uint32_t frame_id)
{
    cv::Mat gt_color_frame;
    if(!load_gt_color_parallel(gt_color_frame, frame_id))
    {
        LOG(ERROR) << "Failed to load color image from ground truth, so skipping ...";
        return -1.0f;
    }
    return 0.0f;
}

float BitrateSplitterServer::calc_depth_rmse(cv::Mat &dist_depth_frame, uint32_t frame_id)
{
    cv::Mat gt_depth_frame;
    if(!load_gt_depth_parallel(gt_depth_frame, frame_id))
    {
        LOG(ERROR) << "Failed to load depth image from ground truth, so skipping ...";
        return -1.0f;
    }
    return 0.0f;
}

bool BitrateSplitterServer::load_gt_color_parallel(cv::Mat &color_frame, uint32_t frame_id)
{
    cv::Mat color_images_cv2[DATASET::PANOPTIC_NCAM];

    #pragma omp parallel for num_threads(4)
    for(int device_id = 0; device_id < DATASET::PANOPTIC_NCAM; device_id++)     // Assuming device ids are 0, ..., PANOPTIC_CAM - 1
    {
        string color_image_path = FORMAT(gt_path << "color/" << frame_id << "_color_" << device_id << ".png");
        if(!fs::exists(color_image_path))
        {
            LOG(ERROR) << "Color image from " << color_image_path << " doesn't exist, so skipping ...";
            continue;
        }

        cv::Mat color_image_cv2 = cv::imread(color_image_path, cv::IMREAD_UNCHANGED); // IMREAD_UNCHANGED reads with Alpha channel
        if(color_image_cv2.empty())
        {
            LOG(ERROR) << "Failed to read color image from " << color_image_path << ", so skipping ...";
            continue;
        }

        color_images_cv2[device_id] = color_image_cv2;
    }

    for(int i = 0; i < DATASET::PANOPTIC_NCAM; i++)
    {
        if(color_images_cv2[i].empty())
        {
            LOG(ERROR) << "Failed to load color image from device " << i << ", so skipping ...";
            return false;
        }
    }

    cv::Mat top_row, bottom_row;
    cv::hconcat(color_images_cv2, 5, top_row);
    cv::hconcat(color_images_cv2 + 5, 5, bottom_row);
    cv::vconcat(top_row, bottom_row, color_frame);

    cv::imwrite(FORMAT("/home/lei/data/pipeline/server_tiled/pipeline_new/test/" << frame_id << "_gt_color.png"), color_frame);
    return true;
}

bool BitrateSplitterServer::load_gt_depth_parallel(cv::Mat &depth_frame, uint32_t frame_id)
{
    cv::Mat depth_images_cv2[DATASET::PANOPTIC_NCAM];

    #pragma omp parallel for num_threads(4)
    for (int device_id = 0; device_id < DATASET::PANOPTIC_NCAM; device_id++)     // Assuming device ids are 0, ..., PANOPTIC_CAM - 1
    {
        string depth_image_path = FORMAT(gt_path << "depth/" << frame_id << "_depth_" << device_id << ".png");
        if(!fs::exists(depth_image_path))
        {
            LOG(ERROR) << "Depth image from " << depth_image_path << " doesn't exist, so skipping ...";
            continue;
        }

        cv::Mat depth_image_cv2 = cv::imread(depth_image_path, cv::IMREAD_ANYDEPTH);
        if(depth_image_cv2.empty())
        {
            LOG(ERROR) << "Failed to read depth image from " << depth_image_path << ", so skipping ...";
            continue;
        }
        
        depth_images_cv2[device_id] = depth_image_cv2;
    }
    cv::Mat top_row, bottom_row;
    cv::hconcat(depth_images_cv2, 5, top_row);
    cv::hconcat(depth_images_cv2 + 5, 5, bottom_row);
    cv::vconcat(top_row, bottom_row, depth_frame);

    cv::imwrite(FORMAT("/home/lei/data/pipeline/server_tiled/pipeline_new/test/" << frame_id << "_gt_depth.png"), depth_frame);
    return true;
}