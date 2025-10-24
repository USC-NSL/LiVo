#pragma once
#include <iostream>
#include <thread>
// #include <boost/interprocess/shared_memory_object.hpp>
// #include <boost/interprocess/mapped_region.hpp>
#include <k4a/k4a.hpp>
#include <boost/asio.hpp>
#include <string>

#include "k4autils.h"
#include "k4aconsts.h"
#include "frameInfo_generated.h"
#include "colorized_depth.h"
#include "shm.h"
#include "view.h"

// websocket
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core/buffers_to_string.hpp>
#include "calibration.h"
#include "utils.h"
#include "frustum.h"
#include "consts.h"
#include "pconsts.h"

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>


using namespace std;
using namespace boost::interprocess;
using namespace Multiview::Frame;

// WebRTC host address
static string WEBRTC_SENDER_ADDRESS = "127.0.0.1";

static const int PORT = 65432;
static const int TCP_BUFFER_SIZE = 1024;

extern atomic_bool wsserver_ready_to_init;

extern vector<Calibration *> calibrations_init;
extern vector<xy_table_t *> xy_tables_init;
extern vector<uint32_t> deviceIndices_init;
extern vector<string> deviceSerialNumbers_init;

extern atomic_bool new_frustumClip;
extern FrustumClip frustumClip_recv;

extern atomic_bool new_frustum;
extern Frustum frustum_recv;
extern uint64_t frustum_sent_time;  // Time when frustum was sent to client in ms
extern uint64_t frustum_recv_time;  // Time when frustum was received from client in ms

extern atomic_bool new_mask;
// extern uint8_t *mask_send;
// extern char *mask_send;
extern bool *mask_send;

extern atomic_int bwe;
extern atomic_int cbwe;
extern atomic_int dbwe;

extern atomic_bool new_ptcl;
extern uint8_t *ptcl_buf;
extern uint32_t ptcl_buf_size;


// class Mahimahi {
// public:
//     Mahimahi() {}
//     ~Mahimahi() {}
//     bool load_trace(uint64_t _start_time, string _fname);
//     int get_throughput();

// private:
//     uint64_t start_time;
//     vector<int> throughput;
//     string fname;
//     uint64_t max_time;
// };


class TcpClient
{ 
public:
    TcpClient(boost::asio::io_service& svc, std::string const& host, int const& port);
    ~TcpClient();
	void send(const uint8_t *buf, const int buf_size);
    void close();

private:
	void send_to_socket(const uint8_t *buf, const int buf_size);
    boost::asio::io_service& io_service;
    boost::asio::ip::tcp::socket socket;
};

class WebRTCServer {
public:
    WebRTCServer(int view_number, string type, int view_count, bool isTile = false, bool isSaveView = false);
    ~WebRTCServer() {}
    void send_color(RGB *color_image, int frame_number);
    void send_depth(uint16_t *depth_image, int width, int height, int frame_number);
    void send_binary_mask(bool *binary_mask, int width, int height, int frame_number);

    void send_tiled_color(vector<View *> views, int frame_number);
    void send_tiled_depth(vector<View *> views, int frame_number);
    void send_tiled_depth_yuv16(vector<View *> views, int frame_number);
    void send_tiled_mask(vector<View *> views, int frame_number);

private:

    void add_qr_bgra(RGB *buf, int width, int height, const string &qr_folder, int frameID, bool addText = false);
    void add_qr_yuv16(cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], const string &qr_folder, int frameID, bool isScaled=true, bool addText=false);
    // void depth_to_yuv16(uint16_t *depth, cv::Mat &depth_yuv16_cv, int height, int width, bool isScaled = true);
    void depth_to_yuv16(uint16_t *depth, cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], int height, int width, bool isScaled=true);
    void flatten_depth_yuv16(const cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], uint8_t *depth_yuv16_buf, int height, int width);

    bool isTile = false;
    bool isSaveView = false;
    int view_count;

    // Shared memoryint view_number, string _type
    mapped_region region;
    uint32_t shm_size;
    uint8_t *shm_ptr;

    // FrameInfo
    TcpClient* client;
    flatbuffers::FlatBufferBuilder* builder;

    // Frame Metadata
    int view_number;
    string _type;

    // Color and Depth image buf
    uint8_t** color_buf;
    uint8_t** depth_buf;
    RGB** colorized_depth;
    uint8_t** depth_yuv16_buf;
    // uint8_t* binary_mask_temp;
    // char* binary_mask_temp;
    
    cv::Mat depth_yuv16_cv[CHANNEL_DIM::DEPTH];

    int curr_counter;
};

class WebRTCServerPool{
public:
    WebRTCServerPool(int _view_count, bool _isTile, bool isSaveView = false);
    ~WebRTCServerPool() {}
    void run();
    void send(int capture_id, vector<View *> s_views);
    void send(int capture_id, const uint8_t *compressed_ptcl, const int size);
    bool store_view(vector<View *> s_views);
private:
    vector<WebRTCServer*> cserver_pool;
    vector<WebRTCServer*> dserver_pool;
    // vector<WebRTCServer*> mserver_pool;

    // Separate channel mode
    void send_color_thread(int viewid, RGB *color_image, int capture_id);
    void send_depth_thread(int viewid, uint16_t *depth_image, int width, int height, int capture_id);

    // Tile mode
    void send_color_tiled(vector<View *> views, int capture_id);
    void send_depth_tiled(vector<View *> views, int capture_id);
    void send_mask_tiled(vector<View *> views, int capture_id);
    void send_ptcl(const uint8_t *compressed_ptcl, const int size, int capture_id);
    

    // void send_binary_mask_thread(int viewid, bool *binary_mask, int width, int height, int capture_id);
    int view_count;

    bool isTile = false;
    bool isSaveView = false;

    // // Mahimahi Experiment
    // bool isMahi = false;
    // string mahi_metafile;
    // Mahimahi *up;
    // Mahimahi *down;

    // bool load_trace();
    // int get_throughput();

    vector<thread> send_pool;
};

void run_server();

class WebSocketServer{
public:
    WebSocketServer(const char* addr, int port, int isSendPtcl);
    ~WebSocketServer(){}
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
