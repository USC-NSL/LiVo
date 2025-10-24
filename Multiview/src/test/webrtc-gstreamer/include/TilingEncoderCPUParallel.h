#pragma once

#define GST_USE_UNSTABLE_API
#include <sstream>
#include <string>
#include <map>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>
#include <thread>
#include <vector>
#include "timer.h"
#include "absl/base/log_severity.h"
#include "absl/log/log.h"
#include "absl/log/globals.h"
#include "absl/log/check.h"

#define FORMAT(items) \
    static_cast<std::ostringstream &>((std::ostringstream() << std::string() << items)).str()

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using namespace boost::json;

class TilingEncoderCPUParallel
{
public:
    TilingEncoderCPUParallel();
    TilingEncoderCPUParallel(const std::string &type, int shape[2], const std::string seq_name, int start_frame_id, int end_frame_id, int view_id, const std::string &in_dir, const std::string &out_dir, int fps, uint32_t color_bitrate, uint32_t depth_bitrate);
    ~TilingEncoderCPUParallel();

    static void start_feed_static(GstElement* appsrc, guint size, gpointer user_data);
    static void stop_feed_static(GstElement* appsrc, gpointer user_data);
    static GstFlowReturn pull_static(GstElement *appsink, gpointer user_data);

    void set_appsrc();
    void set_appsink();
    void push_untiled();
    GstFlowReturn pull_untiled();
    void set_color_pipeline();
    void start_pipeline();
    void end_pipeline();

private:

    void color_to_buf(const cv::Mat &image, uint8_t *&buf, int &size, const std::string &color_format="bgra");
    void depth_to_buf(const cv::Mat (&channels)[3], uint8_t *&buf, int &size, const std::string &depth_format="yuv16");
    bool read_color(cv::Mat &image, const int frameID);
    bool read_color_untiled(cv::Mat &image, const int frameID, const int viewID);
    bool read_depth(cv::Mat (&channels)[3], const int frameID);

    bool is_push_buffer_allowed;
    std::string type;

    const std::string seq_name;
    std::string in_dir, color_dir, depth_dir, color_dir_untiled, depth_dir_untiled, out_dir;

    int frame_id, start_frame_id, end_frame_id, view_id;
    int recv_frame_id, recv_view_id;
    
    int client_uid;

    int imgShape[3];
    uint32_t imgSize;

    FPSCounter2 fps_limiter, fps_counter;
    int fps;

    GMainLoop *loop;
    GstElement *pipeline, *appsrc, *appsink;

    uint32_t color_bitrate, depth_bitrate;

    guint64 start_time;
    std::map<int, gint64> send_timestamp;
    std::map<int, int> delay;
    float total_time_taken = 0.0F;
};