#pragma once

#define GST_USE_UNSTABLE_API
#include <sstream>
#include <string>
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

#define STUN_SERVER "stun://stun.l.google.com:19302"

class WebRTCServer
{

public:
    WebRTCServer(const std::string &type, int shape[2], const std::string seq_name, int start_frame_id, int end_frame_id,  const std::string &in_dir, const std::string &out_dir, int fps, uint32_t color_bitrate, uint32_t depth_bitrate, const std::string &addr, int port);
    ~WebRTCServer();
    
    void send_ice_candidate_message(guint mlineindex, gchar *candidate);
    void on_answer_created(GstPromise *promise, gpointer user_data);
    void on_set_remote_description(GstPromise *promise, gpointer user_data);
    GstFlowReturn on_recvd_sample(GstElement *appsink, gpointer user_data);
    
    static void on_ice_candidate_static(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data);
    static void on_answer_created_static(GstPromise *promise, gpointer user_data);
    static void on_negotiation_needed_static(GstElement *webrtc, gpointer user_data);
    static void on_set_remote_description_static(GstPromise *promise, gpointer user_data);
    static void start_feed_static(GstElement* appsrc, guint size, gpointer user_data);
    static void stop_feed_static(GstElement* appsrc, gpointer user_data);
    static GstPadProbeReturn calc_encoded_bitrate_static(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);
    static GstFlowReturn on_recvd_sample_static(GstElement *appsink, gpointer user_data);

    void handle_sdp(const std::string &message);
    void push();
    void probe_stats();
    void start_pipeline();
    void start_pipeline_dummy();
    void end_pipeline();
    void ws_handler(tcp::socket socket);
    void run();

private:
    void set_pipeline_dummy(); 
    void set_color_pipeline();
    void set_color_pipeline_with_distortion_analysis();
    void set_depth_pipeline_bgra();
    void set_depth_pipeline_yuv16();
    void set_appsrc();
    void setup_appsink_distortion_analyzer();

    cv::Mat color_to_opencv(const uint8_t *img_buf, int img_height, int img_width, bool withAlpha);
    int decode_qr_code(const cv::Mat &image, int offset_x=20, int offset_y=20, int roi_width=82, int roi_height=82, int border=4*2, int view_number=0);
    int decode_qr_code_yuv(const cv::Mat &image, int offset_x=20, int offset_y=20, int roi_width=82, int roi_height=82, int border=4*2, int view_number=0);
    void color_to_buf(const cv::Mat &image, uint8_t *&buf, int &size, const std::string &color_format="bgra");
    void depth_to_buf(const cv::Mat (&channels)[3], uint8_t *&buf, int &size, const std::string &depth_format="yuv16");
    bool read_color(cv::Mat &image, const int frameID);
    bool read_depth(cv::Mat (&channels)[3], const int frameID);

    bool is_push_buffer_allowed;
    std::string type;

    const std::string seq_name;
    std::string in_dir, color_dir, depth_dir, out_dir;

    int frame_id, start_frame_id, end_frame_id;

    const std::string addr;
    int port;
    websocket::stream<tcp::socket> *ws;
    std::vector<std::thread> handler_pools;
    
    int client_uid;

    int imgShape[3];
    uint32_t imgSize;

    FPSCounter2 fps_limiter, fps_counter;
    int fps;

    GMainLoop *loop;
    GstElement *pipeline, *webrtcbin, *appsrc;

    uint32_t color_bitrate, depth_bitrate;
    gsize bytes_in_interval;
    guint64 start_time;

    cv::QRCodeDetector qrDecoder;
};