#include "k4autils.h"

/**
 * Rajrup: Alpha is removed from the image. Note even our ply dump doesn't contain alpha channel. 
 * To Do: Allow alpha channel as well. No need of cvtColor.
 */

cv::Mat color_to_opencv(const k4a::image& im, bool withAlpha)
{
    cv::Mat cv_image_with_alpha = cv::Mat(im.get_height_pixels(), im.get_width_pixels(), CV_8UC4, (void*)im.get_buffer()).clone();
    if(!withAlpha)
    {
        cv::Mat cv_image_no_alpha;
        cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);
        return cv_image_no_alpha;
    }
    return cv_image_with_alpha;
}

cv::Mat depth_to_opencv(const k4a::image &im)
{
    return cv::Mat(im.get_height_pixels(),
                   im.get_width_pixels(),
                   CV_16U,
                   (void *)im.get_buffer(),
                   static_cast<size_t>(im.get_stride_bytes())).clone();
}

k4a::image opencv_to_color(const cv::Mat &im)
{
    k4a::image color_image = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                im.cols,
                                                im.rows,
                                                im.step[0]);
    memcpy(color_image.get_buffer(), im.data, im.rows * im.step[0]);
    return color_image;
}

k4a::image opencv_to_depth(const cv::Mat &im)
{
    k4a::image depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                                                im.cols,
                                                im.rows,
                                                im.step[0]);
    memcpy(depth_image.get_buffer(), im.data, im.rows * im.step[0]);
    return depth_image;
}