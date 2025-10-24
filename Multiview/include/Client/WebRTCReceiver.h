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

#include "ReceiverFrameBuffer.h"
#include "Client/BitrateSplitterClient.h"
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

struct FrameData 
{
    guint64 timestamp;
    gsize bytes;
};

bool check_webrtc_plugins();
class WebRTCReceiver
{
public:
    WebRTCReceiver(const std::string &type, int shape[2], int start_frame_id, int end_frame_id, const std::string &out_dir, const std::string &addr, int port, uint32_t fps, ReceiverFrameBuffer *&recv_frame_buffer);
    ~WebRTCReceiver();

    static void on_ice_candidate_static(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data);
    static void on_offer_created_static(GstPromise *promise, gpointer user_data);
    static void on_negotiation_needed_static(GstElement *webrtc, gpointer user_data);
    static void on_set_remote_description_static(GstPromise *promise, gpointer user_data);
    static GstFlowReturn start_feed_static(GstElement *appsink, gpointer user_data);
    static GstPadProbeReturn calc_receiver_bitrate_static(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);
    static GstPadProbeReturn calc_receiver_bitrate_window_static(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);
    static GstFlowReturn on_recvd_sample_static(GstElement *appsink, gpointer user_data);

    void send_ice_candidate_message(guint mlineindex, gchar *candidate);
    GstFlowReturn on_recvd_sample(GstElement *appsink, gpointer user_data);
    void handle_frame_info(const std::string &msg);
    void handle_sdp(const std::string &msg);
    void ws_handler(tcp::socket socket);
    
    GstFlowReturn pull();
    void probe_stats();
    void start_pipeline();
    void end_pipeline();
    void on_eos();
    void connect();

private:
    void set_color_pipeline();
    void set_color_pipeline_with_distortion_analysis();
    void set_depth_pipeline_bgra();
    void set_depth_pipeline_yuv16();
    void set_depth_pipeline_yuv16_with_distortion_analysis();
    void set_appsink();
    void configure_pipeline_components();
    void configure_appsink_distortion_analyzer();

    // Utility functions
    cv::Mat color_to_opencv(const uint8_t *data, int width, int height, bool withAlpha);
    cv::Mat depth_to_opencv(const uint8_t *data, int width, int height);
    int decode_qr_code(const cv::Mat &image, int offset_x=20, int offset_y=20, int roi_width=82, int roi_height=82, int border=4*2, int view_number=0);
    int decode_qr_code_yuv(const cv::Mat &image, int offset_x=20, int offset_y=20, int roi_width=82, int roi_height=82, int border=4*2, int view_number=0);

    // Gstreamer pipeline required elements
    std::string pipeline_desc;
    GMainLoop *loop;
    GstElement *pipeline, *webrtcbin, *appsink;
    ReceiverFrameBuffer *recv_frame_buffer;

    // Gstreamer element parameters
    // uint32_t color_bitrate, depth_bitrate;
    guint64 start_time;
    std::queue<FrameData> frame_data_queue;
    gsize bytes_in_interval;
    
    // BitrateSplitterClient *bs;
    int analyzer_fc;

    // websocket signaling server
    const std::string addr;
    int port;
    websocket::stream<tcp::socket> *ws;

    int frame_id, prev_frame_id;
    int start_frame_id, end_frame_id;
    int qr_loss_counter, recv_frame_counter;
    int frameShape[3];
    uint32_t frameSize;
    int h, w, d;

    std::string type;
    uint32_t uid;

    FPSCounter2 fps_counter;
    uint32_t fps;

    std::string save_dir;
    cv::QRCodeDetector qrDecoder;

    // logging
    fs::ofstream file_cbitrate, file_dbitrate;
};