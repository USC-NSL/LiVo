#pragma once

#define GST_USE_UNSTABLE_API
#include <sstream>
#include <string>
#include <iostream>
#include <thread>
#include <vector>
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect.hpp>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core/buffers_to_string.hpp>

#include "SenderFrameBuffer.h"
#include "BitrateSplitterServer.h"
#include "timer.h"

// Host webrtc signalling server on public IP
#define STUN_SERVER "stun://stun.l.google.com:19302"

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using namespace boost::json;
using namespace std;

extern atomic_int bwe2;
extern atomic_int cbwe2;
extern atomic_int dbwe2;

bool check_webrtc_plugins();
class WebRTCSender
{

public:
    WebRTCSender(const std::string &type, int shape[2], const std::string &out_dir, uint32_t color_bitrate, uint32_t depth_bitrate, const std::string &addr, int port, uint32_t fps, SenderFrameBuffer *&send_frame_buffer);
    ~WebRTCSender();
    
    static void on_ice_candidate_static(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data);
    static void on_answer_created_static(GstPromise *promise, gpointer user_data);
    static void on_negotiation_needed_static(GstElement *webrtc, gpointer user_data);
    static void on_set_remote_description_static(GstPromise *promise, gpointer user_data);
    static void start_feed_static(GstElement* appsrc, guint size, gpointer user_data);
    static void stop_feed_static(GstElement* appsrc, gpointer user_data);
    static gboolean retry_push_static(gpointer user_data);
    static GstPadProbeReturn calc_encoded_bitrate_static(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);
    static GstFlowReturn on_recvd_sample_static(GstElement *appsink, gpointer user_data);

    void send_ice_candidate_message(guint mlineindex, gchar *candidate);
    GstFlowReturn on_recvd_sample(GstElement *appsink, gpointer user_data);
    void handle_sdp(const std::string &message);
    void ws_handler(tcp::socket socket);

    void push();
    void probe_stats();
    void start_pipeline();
    void end_pipeline();
    void run();

private:
    void set_color_pipeline();
    void set_color_pipeline_with_distortion_analysis();
    void set_depth_pipeline_bgra();
    void set_depth_pipeline_yuv16();
    void set_depth_pipeline_yuv16_with_distortion_analysis();
    void set_appsrc();
    void set_encoder_properties();
    void set_encoder_properties_starline();
    void set_bitrates(int frameID);
    void configure_pipeline_components();
    void configure_appsink_distortion_analyzer();

    // Utility functions
    void color_to_buf(const cv::Mat &image, uint8_t *&buf, int &size, const std::string &color_format="bgra");
    void depth_to_buf(const cv::Mat (&channels)[3], uint8_t *&buf, int &size, const std::string &depth_format="yuv16");
    cv::Mat color_to_opencv(const uint8_t *img_buf, int img_height, int img_width, bool withAlpha);
    int decode_qr_code(const cv::Mat &image, int offset_x=20, int offset_y=20, int roi_width=82, int roi_height=82, int border=4*2, int view_number=0);
    int decode_qr_code_yuv(const cv::Mat &image, int offset_x=20, int offset_y=20, int roi_width=82, int roi_height=82, int border=4*2, int view_number=0);

    // Gstreamer pipeline required elements
    // GMainLoop *loop;
    std::string pipeline_desc;
    GstElement *pipeline, *webrtcbin, *appsrc;
    bool is_push_buffer_allowed;
    SenderFrameBuffer *send_frame_buffer;

    // Gstreamer element parameters
    uint32_t prev_color_bitrate, prev_depth_bitrate;
    uint32_t color_bitrate, depth_bitrate;
    int color_qp, depth_qp;
    gsize bytes_in_interval;
    guint64 start_time;
    BitrateSplitterServer *bs;

    // websocket signaling server
    const std::string addr;
    int port;
    websocket::stream<tcp::socket> *ws;

    // Thread pools
    // std::vector<std::thread> handler_pools;

    int frameCounter;
    int frameShape[3];
    uint32_t frameSize;
    uint8_t *frameBuf;

    std::string type;
    int client_uid;

    FPSCounter2 fps_counter;
    uint32_t fps;

    cv::QRCodeDetector qrDecoder;
};