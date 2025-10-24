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

class WebRTCReceiver
{
public:
    WebRTCReceiver(const std::string &type, const std::string seq_name, int start_frame_id, int end_frame_id, const std::string &out_dir, int fps, uint32_t color_bitrate, uint32_t depth_bitrate, const std::string &addr, int port);
    ~WebRTCReceiver();

    void send_ice_candidate_message(guint mlineindex, gchar *candidate);

    static void on_ice_candidate_static(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data);
    static void on_offer_created_static(GstPromise *promise, gpointer user_data);
    static void on_negotiation_needed_static(GstElement *webrtc, gpointer user_data);
    static void on_set_remote_description_static(GstPromise *promise, gpointer user_data);
    static GstFlowReturn pull_static(GstElement *appsink, gpointer user_data);
    static GstPadProbeReturn calc_receiver_bitrate_static(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);
    
    GstFlowReturn pull();
    void handle_frame_info(const std::string &msg);
    void handle_sdp(const std::string &msg);
    void probe_stats();
    void on_eos();
    void start_pipeline();
    void start_pipeline_dummy();
    void end_pipeline();
    void ws_handler(tcp::socket socket);
    void connect();

private:
    void set_pipeline_dummy(); 
    void set_color_pipeline();
    void set_depth_pipeline_bgra();
    void set_depth_pipeline_yuv16();
    void set_appsink();

    cv::Mat color_to_opencv(const uint8_t *data, int width, int height, bool withAlpha);
    cv::Mat depth_to_opencv(const uint8_t *data, int width, int height);
    int decode_qr_code(const cv::Mat &image, int offset_x=20, int offset_y=20, int roi_width=82, int roi_height=82, int border=4*2, int view_number=0);
    int decode_qr_code_yuv(const cv::Mat &image, int offset_x=20, int offset_y=20, int roi_width=82, int roi_height=82, int border=4*2, int view_number=0); 

    int frame_id, prev_frame_id;
    int qr_loss_counter, recv_frame_counter;

    const std::string seq_name;
    std::string out_dir;
    int start_frame_id, end_frame_id;

    const std::string addr;
    int port;
    websocket::stream<tcp::socket> *ws;

    std::string type;
    uint32_t uid;

    int imgShape[3];
    uint32_t imgSize;
    int h, w, d;

    FPSCounter2 fps_counter;
    int fps;

    GMainLoop *loop;
    GstElement *pipeline, *webrtcbin, *appsink;

    uint32_t color_bitrate, depth_bitrate;
    gsize bytes_in_interval;
    guint64 start_time;

    cv::QRCodeDetector qrDecoder; 
};