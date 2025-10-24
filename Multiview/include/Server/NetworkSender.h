#pragma once
#include <string>

// websocket
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core/buffers_to_string.hpp>

#include "view.h"
#include "calibration.h"
#include "utils.h"
#include "frustum.h"
#include "consts.h"
#include "pconsts.h"
#include "WebRTCSender.h"
#include "SenderFrameBuffer.h"

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

using namespace std;

extern atomic_bool wsserver_ready_to_init2;

extern vector<Calibration *> calibrations_init2;
extern vector<xy_table_t *> xy_tables_init2;
extern vector<uint32_t> deviceIndices_init2;
extern vector<std::string> deviceSerialNumbers_init2;

extern atomic<float> d2c_split2;

extern atomic_bool new_frustumClip2;
extern FrustumClip frustumClip_recv2;

extern atomic_bool new_frustum2;
extern Frustum frustum_recv2;
extern uint64_t frustum_sent_time2;  // Time when frustum was sent to client in ms
extern uint64_t frustum_recv_time2;  // Time when frustum was received from client in ms

extern atomic_bool new_mask2;
extern bool *mask_send2;

extern atomic_bool new_ptcl2;
extern uint8_t *ptcl_buf2;
extern uint32_t ptcl_buf_size2;

class NetworkSender 
{
public:
    NetworkSender(std::string type, bool isSaveView = false, std::string save_dir="");
    ~NetworkSender();
    void send_tiled_color(vector<View *> views, int frame_number);
    void send_tiled_depth(vector<View *> views, int frame_number);
    void send_tiled_depth_yuv16(vector<View *> views, int frame_number);
    void send_tiled_mask(vector<View *> views, int frame_number);

private:
    std::string type;
    int view_count;

    // Color and Depth image buf
    // uint8_t** color_buf;
    // uint8_t** depth_buf;

    // Placeholder memory for temporary operations on frames
    RGB** colorized_depth;
    uint8_t** depth_yuv16_buf;
    cv::Mat depth_yuv16_cv[CHANNEL_DIM::DEPTH];

    WebRTCSender *webrtc_sender;
    SenderFrameBuffer *send_cframe_buffer;
    SenderFrameBuffer *send_dframe_buffer;
    std::thread webrtc_thread;

    bool isSaveView = false;
    std::string save_dir = "";

    uint32_t sent_frame_count = 0;
};

class NetworkSenderPool{
public:
    NetworkSenderPool(bool isSaveView = false, std::string save_dir="/home/lei/data/pipeline/server_tiled/pipeline_new/test/");
    ~NetworkSenderPool();
    void run();
    void send(int capture_id, vector<View *> s_views);
    void send(int capture_id, const uint8_t *compressed_ptcl, const int size);
    bool store_view(vector<View *> s_views);
private:
    // Separate channel mode
    void send_color_thread(int viewid, RGB *color_image, int capture_id);
    void send_depth_thread(int viewid, uint16_t *depth_image, int width, int height, int capture_id);

    // Tile mode
    void send_color_tiled(vector<View *> views, int capture_id);
    void send_depth_tiled(vector<View *> views, int capture_id);
    void send_depth_tiled_yuv16(vector<View *> views, int capture_id);
    void send_mask_tiled(vector<View *> views, int capture_id);
    void send_ptcl(const uint8_t *compressed_ptcl, const int size, int capture_id);
    int view_count;

    bool isSaveView = false;
    std::string save_dir = "";

    NetworkSender *cserver;
    NetworkSender *dserver;

    vector<thread> send_pool;
    StopWatch sw_webrtc;
};

class NetworkSocketServer
{
public:
    NetworkSocketServer(const char* addr, int port, int isSendPtcl);
    ~NetworkSocketServer(){}
    void run();
    void sendMask();
private:
    void do_session(tcp::socket socket);
    void do_session2(tcp::socket socket);
    void do_session3(tcp::socket socket);
    void do_session_ptcl(tcp::socket socket);

    const char* _addr;
    int _port;
    websocket::stream<tcp::socket>* ws1;
    websocket::stream<tcp::socket>* ws2;
    websocket::stream<tcp::socket>* ws3;
    websocket::stream<tcp::socket>* ws_ptcl;

    int _isSendPtcl;
};