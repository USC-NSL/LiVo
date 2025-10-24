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
#include "WebRTCReceiver.h"
#include "ReceiverFrameBuffer.h"
#include "Client/BitrateSplitterClient.h"

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>

using namespace std;

extern atomic_bool main_worker_ready_to_init2;
extern atomic_bool wclient_ready_to_init2;

extern atomic_bool next_frame_ready2;
extern atomic_int next_frame2;

extern int wdevice_num2;

extern int cimg_h2;
extern int cimg_w2;
extern int dimg_h2;
extern int dimg_w2;

extern vector<Calibration *> calibrations_recv2;
extern vector<xy_table_t *> xy_tables_recv2;
extern vector<uint32_t> deviceIndices_recv2;
extern vector<std::string> deviceSerialNumbers_recv2;

extern atomic_bool new_frustumClip2;
extern FrustumClip frustumClip_send2;

extern atomic_bool new_frustum2;
extern Frustum frustum_send2;

extern atomic_bool new_mask2;
extern uint8_t *mask_recv2;

extern atomic_bool new_ptcl2;
extern uint8_t *compressed_ptcl_buf2;
extern uint32_t compressed_ptcl_buf_size2;

enum RECV_STATUS
{
    FRAME_WAIT = 0,
    FRAME_SKIP = 1,
    FRAME_READY = 2
};

static const char * recv_status2str[] = { "FRAME_WAIT", "FRAME_SKIP", "FRAME_READY"};

class NetworkReceiver
{
public:
    NetworkReceiver(std::string type, bool isSaveView = false, std::string save_dir="/home/lei/data/pipeline/client_tiled/pipeline_new/test/");
    ~NetworkReceiver();
    RECV_STATUS recv_tiled_color(vector<RGB *> &color_views, int requested_frame_id);
    RECV_STATUS recv_tiled_depth(vector<uint16_t *> &depth_views, int requested_frame_id);
    RECV_STATUS recv_tiled_depth_yuv16(vector<uint16_t *> &depth_views, int requested_frame_id);
    int get_latest_frame_id();
private:
    std::string type;
    int view_count;

    uint8_t *recv_buf;      // Buffer to store received data
    int recv_buf_size;      // Size of received data

    RGB** colorized_depth;  // Colorized depth buffer

    WebRTCReceiver *webrtc_receiver;
    ReceiverFrameBuffer *recv_cframe_buffer;
    ReceiverFrameBuffer *recv_dframe_buffer;
    std::thread webrtc_thread;

    bool isSaveView = false;
    std::string save_dir = "";
};

class NetworkReceiverPool
{
public:
    NetworkReceiverPool(bool isSaveView = false, std::string save_dir="/home/lei/data/pipeline/client_tiled/pipeline_new/test/");
    ~NetworkReceiverPool();
    RECV_STATUS poll_frame(vector<View *> &requested_views, int requested_frame_id);
    
private:
    int view_count;
    // vector<View *> next_views;

    bool isSaveView = false;
    std::string save_dir = "";

    vector<RGB *> color_views;
    vector<uint16_t *> depth_views;

    NetworkReceiver *creceiver;
    NetworkReceiver *dreceiver;
};

class NetworkSocketClient
{
public:
    NetworkSocketClient(const char* addr, int port, const int isSendPtcl);
    ~NetworkSocketClient();
    void run();
    void get_calibration();
    void sendFrustum();
    void sendFrustumClip();
    void get_mask();
    void sendBwe();
    void get_ptcl();

private:
    const char* _addr;
    int _port;
    net::io_context ioc1;
    tcp::resolver resolver1;
    websocket::stream<tcp::socket> ws1;
    std::string host1;

    net::io_context ioc2;
    tcp::resolver resolver2;
    websocket::stream<tcp::socket> ws2;
    std::string host2;

    net::io_context ioc3;
    tcp::resolver resolver3;
    websocket::stream<tcp::socket> ws3;
    std::string host3;

    net::io_context ioc_ptcl;
    tcp::resolver resolver_ptcl;
    websocket::stream<tcp::socket> ws_ptcl;
    std::string host_ptcl;
  
    const int _isSendPtcl;
};