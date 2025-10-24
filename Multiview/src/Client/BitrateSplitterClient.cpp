#include "Client/BitrateSplitterClient.h"
#include <omp.h>
#include <vector>

#define ATOL 1.0f
// #define QUANT_STEP 0.025f
#define QUANT_STEP 0.005f

// atomic_int bwe2(-1);
// atomic_int cbwe2(-1);
// atomic_int dbwe2(-1);
atomic<float> d2c_split2(7.0f/8.0);
atomic_int num_frustum_points2(0);

BitrateSplitterClient::BitrateSplitterClient(int height, int width, int nViews, float d2c_split) : height(height), width(width), nViews(nViews)
{
    // Set color and depth bitrate
    // bwe2.store(FLAGS_cb + FLAGS_db);
    // cbwe2.store(FLAGS_cb);
    // dbwe2.store(FLAGS_db);
    d2c_split2.store(d2c_split);

    color_bitrate = 0;
    depth_bitrate = 0;
    depth2color_split = d2c_split;    // Initial split, Default is 7/8

    has_dist_view.store(false);
    dist_cviews.resize(nViews);
    dist_dviews.resize(nViews);
    gt_cviews.resize(nViews);
    gt_dviews.resize(nViews);

    frameID = 0;
}

BitrateSplitterClient::~BitrateSplitterClient()
{
}

bool BitrateSplitterClient::has_distorted_view()
{
    return has_dist_view.load();
}

void BitrateSplitterClient::allow_distorted_view()
{
    has_dist_view.store(false);
}

uint32_t BitrateSplitterClient::get_frame_id()
{
    return frameID;
}

float BitrateSplitterClient::get_depth2color_split()
{
    return depth2color_split.load();
}

bool BitrateSplitterClient::set_distorted_view(vector<View *> &view, uint32_t frame_id)
{
    if(has_dist_view.load())
    {
        LOG(ERROR) << "Distorted view already set, so skipping ...";
        return false;
    }

    // Copy the distorted view to opencv Mat
    for(int i = 0; i < nViews; i++)
    {
        dist_cviews[i] = cv::Mat(height, width, CV_8UC(CHANNEL_DIM::BGRA), (void *)view[i]->color_image);
        // Drop the alpha channel, assuming 
        cv::cvtColor(dist_cviews[i], dist_cviews[i], cv::COLOR_BGRA2BGR);
        dist_dviews[i] = cv::Mat(height, width, CV_16UC1, (void *)view[i]->depth_image).clone();
    }

    frameID = frame_id;


    // Debug
    // for(int i = 0; i < nViews; i++)
    // {
    //     cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << frameID << "_dist_color_" << i << ".png"), dist_cviews[i]);
    //     cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << frameID << "_dist_depth_" << i << ".png"), dist_dviews[i]);
    // }

    has_dist_view.store(true);
    return true;
} 

bool BitrateSplitterClient::set_distorted_view_from_disk(std::string &path, uint32_t frame_id)
{
    CHECK(fs::exists(path)) << "Path " << path << " does not exist!";
    

    // Load the distorted color and depth views
    #pragma omp parallel for num_threads(4)
    for(int i = 0; i < nViews; i++)
    {
        string color_image_path = FORMAT(path << frame_id << "_dist_color_" << i << ".png");
        string depth_image_path = FORMAT(path << frame_id << "_dist_depth_" << i << ".png");

        if(!fs::exists(color_image_path) || !fs::exists(depth_image_path))
            continue;

        dist_cviews[i] = cv::imread(color_image_path, cv::IMREAD_COLOR); // Drop the alpha channel
        if(dist_cviews[i].empty())
            continue;

        dist_dviews[i] = cv::imread(depth_image_path, cv::IMREAD_ANYDEPTH);
        if(dist_dviews[i].empty())
            continue;
    }

    for(int i = 0; i < nViews; i++)
    {
        if(dist_cviews[i].empty() || dist_dviews[i].empty())
        {
            LOG(ERROR) << "Failed to load distorted view from device " << i << ", so skipping ...";
            return false;
        }
    }

    frameID = frame_id;
    has_dist_view.store(true);
    return true;
}

bool BitrateSplitterClient::set_gt_view_from_disk(std::string &path)
{
    return set_gt_view_from_disk(path, frameID);
}

bool BitrateSplitterClient::set_gt_view_from_disk(std::string &path, uint32_t frame_id)
{
    CHECK(fs::exists(path)) << "Path " << path << " does not exist!";
    CHECK(frame_id == frameID) << "Frame ID mismatch - Received " << frame_id << ", Expected " << frameID;
    CHECK(has_distorted_view()) << "Distorted view not set";

    // Load the ground truth color and depth frames
    if(!load_gt_color_parallel(path, frame_id))
    {
        LOG(ERROR) << "Failed to load color image for frame " << frame_id << " from ground truth, so skipping ...";
        allow_distorted_view();
        return false;
    }

    if(!load_gt_depth_parallel(path, frame_id))
    {
        LOG(ERROR) << "Failed to load depth image for frame " << frame_id << " from ground truth, so skipping ...";
        allow_distorted_view();
        return false;
    }

    return true;
}

bool BitrateSplitterClient::load_gt_color_parallel(std::string &path, uint32_t frame_id)
{
    CHECK(frame_id == frameID) << "Frame ID mismatch - Received " << frame_id << ", Expected " << frameID;

    #pragma omp parallel for num_threads(4)
    for(int i = 0; i < nViews; i++)     // Assuming device ids are 0, ..., nViews - 1
    {
        string color_image_path = FORMAT(path << "color/" << frame_id << "_color_" << i << ".png");
        if(!fs::exists(color_image_path))
            continue;

        gt_cviews[i] = cv::imread(color_image_path, cv::IMREAD_COLOR); // Drop the alpha channel
        if(gt_cviews[i].empty() || gt_cviews[i].size() != dist_cviews[i].size() || gt_cviews[i].type() != dist_cviews[i].type())
            continue;
    }

    for(int i = 0; i < nViews; i++)
    {
        if(gt_cviews[i].empty())
        {
            LOG(ERROR) << "Failed to load color image from device " << i << ", so skipping ...";
            return false;
        }
    }

    // Debug
    // for(int i = 0; i < nViews; i++)
    // {
    //     cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << frame_id << "_gt_color_" << i << ".png"), gt_cviews[i]);
    // }
    return true;
}

bool BitrateSplitterClient::load_gt_depth_parallel(std::string &path, uint32_t frame_id)
{
    CHECK(frame_id == frameID) << "Frame ID mismatch - Received " << frame_id << ", Expected " << frameID;
    
    #pragma omp parallel for num_threads(4)
    for(int i = 0; i < nViews; i++)     // Assuming device ids are 0, ..., nViews - 1
    {
        string depth_image_path = FORMAT(path << "depth/" << frame_id << "_depth_" << i << ".png");
        if(!fs::exists(depth_image_path))
            continue;

        gt_dviews[i] = cv::imread(depth_image_path, cv::IMREAD_ANYDEPTH);
        if(gt_dviews[i].empty() || gt_dviews[i].size() != dist_dviews[i].size() || gt_dviews[i].type() != dist_dviews[i].type())
            continue;
    }

    for(int i = 0; i < nViews; i++)
    {
        if(gt_dviews[i].empty())
        {
            LOG(ERROR) << "Failed to load depth image from device " << i << ", so skipping ...";
            return false;
        }
    }

    // Debug
    // for(int i = 0; i < nViews; i++)
    // {
    //     cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << frame_id << "_gt_depth_" << i << ".png"), gt_dviews[i]);
    // }
    return true;
}

/**
 * @brief Calculates pixel-wise RMSE between distorted and ground truth color frames
 * Color is in BGRA format. Calculate RMSE for each channel and average them
 * @return float 
 */
float BitrateSplitterClient::calc_color_rmse()
{
    // Calculate RMSE for each channel
    float total_rmse = 0.0f;
    cv::Mat diff;
    int num_frustum_points = height * width;
    for(int i = 0; i < nViews; i++)
    {
        cv::absdiff(dist_cviews[i], gt_cviews[i], diff);
        diff.convertTo(diff, CV_32F);
        cv::pow(diff, 2, diff);
        cv::Scalar mse_per_channel = cv::mean(diff);    // Mean squared error per channel. This is a 4-element vector where last channel is 0.0
        float mse = (mse_per_channel[0] + mse_per_channel[1] + mse_per_channel[2]) / 3.0f;
        total_rmse += (mse * num_frustum_points); 
    }

    return sqrt(total_rmse / (float)(num_frustum_points * nViews));
}

/**
 * @brief Calculates pixel-wise RMSE between distorted and ground truth depth frames
 * @return float 
 */
float BitrateSplitterClient::calc_depth_rmse()
{
    float total_rmse = 0.0f;
    cv::Mat diff;
    int num_frustum_points = height * width;
    for(int i = 0; i < nViews; i++)
    {   
        cv::absdiff(dist_dviews[i], gt_dviews[i], diff);
        diff.convertTo(diff, CV_32F);
        cv::pow(diff, 2, diff);
        cv::Scalar mse_per_channel = cv::mean(diff);    // Mean squared error per channel. This is a 4-element vector where last channel is 0.0
        float mse = mse_per_channel[0];
        total_rmse += (mse * num_frustum_points); 
    }

    return sqrt(total_rmse / (float)(num_frustum_points * nViews));
}

float BitrateSplitterClient::calc_color_psnr()
{
    float rmse = calc_color_rmse();
    return 20.0f * log10(255.0f / rmse);
}

float BitrateSplitterClient::calc_depth_psnr()
{
    float rmse = calc_depth_rmse();
    return 20.0f * log10(255.0f / rmse);
}

// Function to calculate SSIM for two single-channel images
float calculateSSIM(const cv::Mat& img1, const cv::Mat& img2) {
    const double C1 = 6.5025, C2 = 58.5225;

    // Convert images to float
    cv::Mat img1Float, img2Float;
    img1.convertTo(img1Float, CV_32F);
    img2.convertTo(img2Float, CV_32F);

    // Calculate means
    cv::Mat mu1, mu2;
    cv::GaussianBlur(img1Float, mu1, cv::Size(11, 11), 1.5);
    cv::GaussianBlur(img2Float, mu2, cv::Size(11, 11), 1.5);

    // Calculate variances and covariance
    cv::Mat mu1Sq, mu2Sq, mu1Mu2;
    cv::multiply(mu1, mu1, mu1Sq);
    cv::multiply(mu2, mu2, mu2Sq);
    cv::multiply(mu1, mu2, mu1Mu2);

    cv::Mat sigma1Sq, sigma2Sq, sigma12;
    cv::Mat img1Sq, img2Sq, img1Img2;

    cv::multiply(img1Float, img1Float, img1Sq);
    cv::multiply(img2Float, img2Float, img2Sq);
    cv::multiply(img1Float, img2Float, img1Img2);

    cv::GaussianBlur(img1Sq, sigma1Sq, cv::Size(11, 11), 1.5);
    cv::GaussianBlur(img2Sq, sigma2Sq, cv::Size(11, 11), 1.5);
    cv::GaussianBlur(img1Img2, sigma12, cv::Size(11, 11), 1.5);

    sigma1Sq -= mu1Sq;
    sigma2Sq -= mu2Sq;
    sigma12 -= mu1Mu2;

    // Calculate SSIM
    cv::Mat t1, t2, t3;
    t1 = 2 * mu1Mu2 + C1;
    t2 = 2 * sigma12 + C2;
    t3 = t1.mul(t2); // Element-wise multiplication

    t1 = mu1Sq + mu2Sq + C1;
    t2 = sigma1Sq + sigma2Sq + C2;
    t1 = t1.mul(t2); // Element-wise multiplication

    cv::Mat ssimMap;
    cv::divide(t3, t1, ssimMap); // Element-wise division

    double ssim = cv::mean(ssimMap)[0]; // Average SSIM over all pixels
    return static_cast<float>(ssim);    // Return as float
}

float BitrateSplitterClient::calc_color_ssim()
{
    float total_ssim = 0.0f;
    for(int i = 0; i < nViews; i++)
    {
        std::vector<cv::Mat> channels_gt, channels_dist;
        cv::split(gt_cviews[i], channels_gt);
        cv::split(dist_cviews[i], channels_dist);

        // Calculate SSIM for each channel
        float ssimB = calculateSSIM(channels_gt[0], channels_dist[0]); // Blue channel
        float ssimG = calculateSSIM(channels_gt[1], channels_dist[1]); // Green channel
        float ssimR = calculateSSIM(channels_gt[2], channels_dist[2]); // Red channel
        total_ssim += (float)((ssimB + ssimG + ssimR) / 3.0f);
    }

    return total_ssim / (float)nViews;
}

float BitrateSplitterClient::calc_depth_ssim()
{
    float total_ssim = 0.0f;
    for(int i = 0; i < nViews; i++)
    {
        float ssim = calculateSSIM(gt_dviews[i], dist_dviews[i]);
        total_ssim += ssim;
    }

    return total_ssim / (float)nViews;
}

// void BitrateSplitterClient::split_bitrate_simple(int &out_bwe, int &out_cbwe, int &out_dbwe)
// {
//     int target_bwe = bwe2.load();
//     int dbwe_now = int(depth2color_split.load() * (float)target_bwe);
//     int cbwe_now = target_bwe - dbwe_now;
//     cbwe2.store(cbwe_now);
//     dbwe2.store(dbwe_now);
//     out_bwe = target_bwe;
//     out_cbwe = cbwe_now;
//     out_dbwe = dbwe_now;
// }

/**
 * @brief Calculates the optimal split between color and depth bitrates to minimize the RMSE
 * @param out_cRMSE 
 * @param out_dRMSE 
 * @param d2c_split 
 * @param stat          FinderStats object to store the optimization statistics. f1_x - RMSE in color, f2_x - RMSE in depth
 */

float BitrateSplitterClient::split_bitrate_adapt(FinderStats &stat)
{
    // Calculate RMSE for color and depth
    float cRMSE = calc_color_rmse();
    float dRMSE = calc_depth_rmse();

    // Find a split that minimizes the rmse. Find the intersection point, where cRMSE = dRMSE within a tolerance
    float csplit = depth2color_split.load();

    if(!finder.is_initialized())
    {
        finder.init(csplit, cRMSE, dRMSE, ATOL, QUANT_STEP);
        return csplit;
    }
    
    stat = finder.update(cRMSE, dRMSE);
    depth2color_split.store(stat.x_opt);

    return stat.x_opt;

    // LOG(INFO) << "Frame: " << frameID << ", Color RMSE: " << cRMSE << ", Depth RMSE: " << dRMSE << ", Split: " << csplit;
}