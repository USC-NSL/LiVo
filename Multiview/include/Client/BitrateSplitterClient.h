#pragma once
#include <vector>
#include <iostream>
#include <cmath>
#include <boost/algorithm/clamp.hpp>
#include <iomanip>
#include "consts.h"
#include "utils.h"
#include "view.h"

using namespace std;

#define DEPTH_BITRATE_PER_POINT 0.14213F 	// 142.13 bps per point
// #define STEP_SIZE 0.01F
#define STEP_SIZE 0.002F
#define D2C_SPLIT_MIN 0.5F
#define D2C_SPLIT_MAX 0.9F

// extern atomic_int bwe2;
// extern atomic_int cbwe2;
// extern atomic_int dbwe2;
extern atomic<float> d2c_split2;
extern atomic_int num_frustum_points2;

typedef struct FinderStats
{
    int iteration;
    float x_opt;        // Current estimate of x_opt
    float f1_x;     // Function value of f1 at x_opt
    float f2_x;     // Function value of f2 at x_opt
    float delta_f;      // Difference between f2 and f1
    float step_size;    // Step size for updating x_opt
    float quant_step;   // Quantization step size
    float atol;         // Absolute tolerance for stopping condition

    FinderStats()
    {
        iteration = 0;
        x_opt = 0.0f;
        f1_x = 0.0f;
        f2_x = 0.0f;
        step_size = 0.0f;
        delta_f = 0.0f;
        quant_step = 0.0f;
        atol = 1e-6;
    }

} FinderStats;

class OnlineIntersectionFinder {
private:
    bool initialized = false;                       // Flag to check if the object is initialized
    // float x_opt;                                    // Current estimate of x_opt
    // float f1_x_opt;                                 // Function value of f1 at x_opt
    // float f2_x_opt;                                 // Function value of f2 at x_opt
    // float atol;                                      // Absolute tolerance for stopping condition
    // int iteration;                                  // Current iteration count
    FinderStats stat;
    vector<FinderStats> stats;                      // History of x_opt values

public:
    // Constructor to initialize the object
    OnlineIntersectionFinder() {}
    OnlineIntersectionFinder(float x0, float f1_x0, float f2_x0, float tolerance = 1e-6, float q_step = 0.0f)
    {
        init(x0, f1_x0, f2_x0, tolerance, q_step);
    }

    ~OnlineIntersectionFinder() {}

    void init(float x0, float f1_x0, float f2_x0, float tolerance = 1e-6, float q_step = 0.0f)
    {
        stat.iteration = 0;
        stat.x_opt = x0;
        stat.f1_x = f1_x0;
        stat.f2_x = f2_x0;
        stat.delta_f = f2_x0 - f1_x0;
        stat.atol = tolerance;
        stat.step_size = STEP_SIZE * stat.delta_f;
        stat.quant_step = q_step;

        stats.clear();
        stats.push_back(stat);
        initialized = true;
    }

    // Method to update the estimate for x_opt
    FinderStats update(float f1_x, float f2_x) 
    {
        // Increment the iteration count
        stat.iteration++;

        // Update the function values
        stat.f1_x = f1_x;
        stat.f2_x = f2_x;

        // Calculate the difference between the current function evaluations
        stat.delta_f = f2_x - f1_x;

        // Update x_opt based on the sign of delta_f
        stat.step_size = STEP_SIZE * stat.delta_f; // Adjust step size as needed

        // Print the current state
        std::cout << "Iteration: " << stat.iteration
                  << ", prev. x_opt: " << stat.x_opt
                  << ", delta_f: " << stat.delta_f
                  << ", f1_x: " << stat.f1_x
                  << ", f2_x: " << stat.f2_x
                  << ", step_size: " << stat.step_size 
                  << std::endl;

        // If the difference is within the tolerance, return the current x_opt
        if (std::fabs(stat.delta_f) <= stat.atol) {
            return stat;
        }

        // Decrease x_opt if delta_f is positive, increase x_opt if negative
        float new_x_opt = stat.x_opt + stat.step_size;
        
        // Quantize x_opt if quant_step is set and greater than zero
        // round to nearest quantization level
        if (stat.quant_step > 0.0f)
            new_x_opt = std::round(new_x_opt / stat.quant_step) * stat.quant_step;

        // Ensure the new guess stays within the bounds [0, 1]
        stat.x_opt = boost::algorithm::clamp(new_x_opt, D2C_SPLIT_MIN, D2C_SPLIT_MAX);

        // Update the history of x_opt values
        stats.push_back(stat);
        return stat;
    }

    // Getters for optional debugging or external use
    vector<FinderStats> get_stats() const { return stats; }
    FinderStats get_last_stat() const { return stat; }
    bool is_initialized() const { return initialized; }
    float get_depth2color_split() const { return stat.x_opt; }
};

class BitrateSplitterClient
{
public:
    BitrateSplitterClient(int height, int width, int nViews=DATASET::PANOPTIC_NCAM, float d2c_split=7.0f/8.0f);
    ~BitrateSplitterClient();

    bool has_distorted_view();
    void allow_distorted_view();
    bool set_distorted_view(vector<View *> &view, uint32_t frame_id);
    bool set_gt_view_from_disk(std::string &path);
    bool set_distorted_view_from_disk(std::string &path, uint32_t frame_id);
    bool set_gt_view_from_disk(std::string &path, uint32_t frame_id);
    uint32_t get_frame_id();
    float get_depth2color_split();

    float calc_color_rmse();
    float calc_depth_rmse();
    float calc_color_psnr();
    float calc_depth_psnr();
    float calc_color_ssim();
    float calc_depth_ssim();

    void split_bitrate_simple(int &out_bwe, int &out_cbwe, int &out_dbwe);
    void split_bitrate_prev();
    float split_bitrate_adapt(FinderStats &stat);

private:

    bool load_gt_color_parallel(std::string &path, uint32_t frame_id);
    bool load_gt_depth_parallel(std::string &path, uint32_t frame_id);

    atomic_bool has_dist_view;
    vector<cv::Mat> dist_cviews, dist_dviews;
    vector<cv::Mat> gt_cviews, gt_dviews;
    uint32_t frameID;
    int height, width;
    int nViews;
    
    uint32_t total_bitrate;
    uint32_t color_bitrate;
    uint32_t depth_bitrate;
    atomic<float> depth2color_split;
    OnlineIntersectionFinder finder;
};