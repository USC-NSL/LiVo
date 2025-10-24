#include "Client/BitrateSplitterClient.h"
using namespace std;

DEFINE_int32(cb, 20000, "Color bitrate in kbps.");
DEFINE_int32(db, 120000, "depth bitrate in kbps.");

int main()
{
    std::string gt_path = "/datassd/KinectStream/panoptic_data/160906_band2_with_ground/";
    std::string dist_path = "/home/lei/data/pipeline/client_tiled/pipeline_new/test/";
    int frame_id = 2;
    BitrateSplitterClient *bs = new BitrateSplitterClient(512, 592, 10);

    CHECK(bs->has_distorted_view() == false) << "Distorted view should be empty at the beginning!";

    CHECK(bs->set_distorted_view_from_disk(dist_path, frame_id)) << "Failed to set distorted view from disk!";
    CHECK(bs->set_gt_view_from_disk(gt_path)) << "Failed to set gt view from disk!";

    float crmse = bs->calc_color_rmse();
    float drmse = bs->calc_depth_rmse();
    float cpsnr = bs->calc_color_psnr();
    float dpsnr = bs->calc_depth_psnr();
    float cssim = bs->calc_color_ssim();
    float dssim = bs->calc_depth_ssim();
    frame_id = bs->get_frame_id();

    LOG(INFO) << "Frame: " << frame_id << ", Color RMSE: " << crmse << ", Depth RMSE: " << drmse << ", Color PSNR: " << cpsnr << ", Depth PSNR: " << dpsnr << ", Color SSIM: " << cssim << ", Depth SSIM: " << dssim;

    // Example 2: Non-linear functions
    auto f1_exp = [](float x) { return std::exp(-x); };
    auto f2_sin = [](float x) { return std::sin(M_PI * x); };

    // Initialize at x0 = 0.5
    float x0 = 0.5;
    float f1_x0 = f1_exp(x0);
    float f2_x0 = f2_sin(x0);

    // Create the finder instance
    OnlineIntersectionFinder finder(x0, f1_x0, f2_x0);

    // Iteratively refine the intersection
    int max_iterations = 100;
    float x_opt = x0;

    for (int i = 0; i < max_iterations; ++i) {
        float f1_x = f1_exp(x_opt);
        float f2_x = f2_sin(x_opt);

        // Update x_opt using the finder
        FinderStats stat = finder.update(f1_x, f2_x);
        x_opt = stat.x_opt;

        // Stop if the refinement is within tolerance
        if (std::fabs(f1_x - f2_x) <= 1e-6) {
            break;
        }
    }

    // Output the refined intersection
    std::cout << "Refined intersection found at x_opt = " << std::setprecision(6) << x_opt << std::endl;
}