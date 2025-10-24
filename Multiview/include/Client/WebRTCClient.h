#pragma once
#include <thread>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <k4a/k4a.hpp>
#include <boost/asio.hpp>
#include <opencv2/objdetect.hpp>

#include "k4autils.h"
#include "k4aconsts.h"
#include "frameInfo_generated.h"
#include "colorized_depth.h"
#include "view.h"
#include "calibration.h"
#include "utils.h"
#include "frustum.h"
#include <signal.h>
#include "consts.h"
#include "pconsts.h"

using namespace std;
using namespace boost::interprocess;
using boost::asio::ip::tcp;
using namespace Multiview::Frame;

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/strand.hpp>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>

static const int CLIENT_PORT = 65452;

#define POOL_LENGTH 200

// Circular buffer pool to keep received frames for synchronization
extern uint8_t** cframe_pool;
extern uint8_t** dframe_pool;
extern int* cframe_index;
extern int* dframe_index;
extern int cframe_max_index;
extern int dframe_max_index;

extern int wdevice_num;

extern atomic_bool main_worker_ready_to_init;
extern atomic_bool wclient_ready_to_init;

extern atomic<int> cframe_counter[10];
extern atomic<int> dframe_counter[10];

extern atomic_bool next_frame_ready;
extern atomic_int next_frame;

extern int cimg_h;
extern int cimg_w;
extern int dimg_h;
extern int dimg_w;

// extern vector<View *> *next_views;
extern vector<View *> next_views;

extern vector<Calibration *> calibrations_recv;
extern vector<xy_table_t *> xy_tables_recv;
extern vector<uint32_t> deviceIndices_recv;
extern vector<string> deviceSerialNumbers_recv;

extern atomic_bool new_frustumClip;
extern FrustumClip frustumClip_send;

extern atomic_bool new_frustum;
extern Frustum frustum_send;

extern atomic_bool new_mask;
extern uint8_t *mask_recv;

extern atomic_int bwe;

extern atomic_bool new_ptcl;
extern uint8_t *compressed_ptcl_buf;
extern uint32_t compressed_ptcl_buf_size;

class SignalHandlerRegister {
public:
  SignalHandlerRegister()
  {
    struct sigaction newHandler;
    newHandler.sa_handler = &SignalHandler;
    sigaddset(&newHandler.sa_mask, SIGQUIT);
    newHandler.sa_flags = 0;

    sigaction(SIGINT, &newHandler, &m_oldHandler);
  }

  ~SignalHandlerRegister()
  {
    sigaction(SIGINT, &m_oldHandler, NULL);
  }

private:
  static void SignalHandler(int sigNum)
  {
    cout << "Recv SIGINT, exiting" << endl;
    exit(0);
  }
  
  struct sigaction m_oldHandler;
};

class TcpServer
{ 
public:
    TcpServer(boost::asio::io_service& svc, int const& port);
    ~TcpServer();

    boost::asio::io_service& io_service;
    boost::asio::ip::tcp::acceptor acceptor;
    boost::asio::ip::tcp::socket socket;
private:
    int port;
};

class WebRTCClient {
public:
    WebRTCClient(int view_number, string type, bool isTile, bool isSaveView);
    ~WebRTCClient() {}
    void recv_old();
    void recv();
    void recv_color(int new_view_number, int new_frame_number);
    void recv_depth(int new_view_number, int new_frame_number);
    void recv_tiled_color(int new_frame_number);
    void recv_tiled_depth(int new_frame_number);
    void recv_tiled_depth_yuv16(int new_frame_number);
private:
    void yuv16_to_depth(uint16_t *depth, int height, int width, bool isScaled=true);
    int detect_framenum_qrcode(RGB *buf, int width, int height, int border=4*2, bool rmCode=true);
    int detect_framenum_qrcode_yuv16(uint8_t *buf, int width, int height, int border=4*2, bool isScaled=true);

    bool isTile;
    bool isSaveView;

    // Shared memory
    mapped_region region;
    uint32_t shm_size;
    uint8_t *shm_ptr;

    TcpServer* server;

    // Frame Metadata
    int view_number;
    string _type;

    cv::QRCodeDetector m_qrDecoder;

    // websocket client for color and depth channels
    net::io_context ioc;
    tcp::resolver resolver;
    websocket::stream<tcp::socket> ws;
    std::string host;

    int recv_counter;
    int curr_counter;

    const int offset_x = 20;
    const int offset_y = 20;

    cv::Rect rectZero;
};

class WebRTCClientPool {
public:
    WebRTCClientPool(int _view_count, bool isTile, bool isSaveView);
    WebRTCClientPool(const string &path, int _view_count, bool isTile, bool isSaveView);
    ~WebRTCClientPool() {}
    void run();
    bool store_view();
private:
    vector<thread> cthread_pool;
    vector<thread> dthread_pool;

    void client_init(int view_number, string type);
    void client_signal();

    // RGB* buf2color(int view, int frame);
    // uint16_t* buf2depth(int view, int frame);

    void buf2color(int view, int frame);
    void buf2depth(int view, int frame);
    void buf2depth_yuv16(int view, int frame);

    // vector<View *> load_view();
    void load_view();

    int view_count;
    bool isTile;
    bool isSaveView;

    RGB** c_frame_temp;
    uint16_t** d_frame_temp;

    uint8_t** depth_buf;
    RGB** colorized_depth;

    string m_WorkDir;
};


class WebSocketClient{
public:
    WebSocketClient(const char* addr, int port, const int isSendPtcl);
    ~WebSocketClient();
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