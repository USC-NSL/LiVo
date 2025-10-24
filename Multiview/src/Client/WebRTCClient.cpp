#include <iostream>
#include <future>

#include "Client/WebRTCClient.h"
#include "shm.h"
#include "view.h"
#include "consts.h"
#include <signal.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#define SHM_BUFFER_SIZE 30      // Number of buffers in shared memory. 30 frames when streamed at 30 fps will contain 1 sec of data.
#define PTCL_MAX_POINTS 2000000 // Maximum number of points in point cloud is 2 million. Used for allocating memory for compressed point cloud buffer.

using namespace std;
using namespace boost::interprocess;
using boost::asio::ip::tcp;
using namespace Multiview::Frame;
namespace fs = boost::filesystem;

// websocket
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include "timer.h"
#include "consts.h"

#include <omp.h>


namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>
using json = nlohmann::json;
using namespace std;

uint8_t** cframe_pool;
uint8_t** dframe_pool;
int* cframe_index;
int* dframe_index;
int cframe_max_index;
int dframe_max_index;

int wdevice_num = 10;

atomic_bool main_worker_ready_to_init(false);
atomic_bool wclient_ready_to_init(false);

atomic<int> cframe_counter[10];
atomic<int> dframe_counter[10];
atomic_bool next_frame_ready(false);
atomic_int next_frame(0);

int cimg_h = 0;
int cimg_w = 0;
int dimg_h = 0;
int dimg_w = 0;

vector<Calibration *> calibrations_recv;
vector<xy_table_t *> xy_tables_recv;
vector<uint32_t> deviceIndices_recv;
vector<string> deviceSerialNumbers_recv;

atomic_bool new_frustumClip(false);
FrustumClip frustumClip_send;

atomic_bool new_frustum(false);
Frustum frustum_send;

atomic_bool new_mask(false);
uint8_t *mask_recv;

atomic_int bwe(1);

uint8_t *compressed_ptcl_buf;
uint32_t compressed_ptcl_buf_size;

// vector<View *> *next_views;
vector<View *> next_views;

TcpServer::TcpServer(boost::asio::io_service& svc, int const& port): io_service(svc), port(port),
acceptor(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)), socket(io_service) 
{
    SignalHandlerRegister signalHdlRegister;
    
    acceptor.accept(socket);
    LOG(INFO) << "TCP Server Listening on port " << port;
}

TcpServer::~TcpServer() 
{
    LOG(INFO) << "Closing connection ...";
    socket.close();
}

WebRTCClient::WebRTCClient(int view_number, string type, bool isTile = false, bool isSaveView = false): 
    view_number(view_number), _type(type), 
    isTile(isTile), isSaveView(isSaveView),
    resolver(ioc), ws(ioc),
    recv_counter(0), curr_counter(0),
    rectZero(offset_x, offset_y, 82, 82) 
{
    //Remove shared memory on construction and destruction
    string sshm_name_with_view = "test_shm_recv" + _type + to_string(view_number);
    const char* shm_name_with_view = sshm_name_with_view.c_str();
    destroy_shm(shm_name_with_view);
    
    //Create a shared memory object
    // shm_size = get_shm_size(path, "c");
    if(_type != "d_yuv16")
        shm_size = cimg_h * cimg_w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA;
    else
        shm_size = dimg_h * dimg_w * CHANNEL_DIM::DEPTH * PIXEL_SIZE::DEPTH;
    // shm_name_with_view = ("test_shm_recv" + _type + to_string(view_number)).c_str();
    
    if(!isTile) 
        create_shm(shm_name_with_view, shm_size, region);
    else 
        create_shm(shm_name_with_view, shm_size * wdevice_num * SHM_BUFFER_SIZE, region);
    
    // create_shm(shm_name_with_view, shm_size, region);
    shm_ptr = static_cast<uint8_t *>(region.get_address());

    if (!isTile) 
    {
        if (_type == "c") 
        {
            for(int i=0; i<POOL_LENGTH; i++) 
                cframe_pool[view_number * POOL_LENGTH + i] = new uint8_t[shm_size];
        } 
        else if (_type == "d") 
        {
            for(int i=0; i<POOL_LENGTH; i++) 
                dframe_pool[view_number * POOL_LENGTH + i] = new uint8_t[shm_size];
        }
        
        cframe_counter[view_number] = 0;
        dframe_counter[view_number] = 0;
    } 
    else 
    {
        // In tile mode there's only view 0, we still need to initialize cframe_pool and dframe_pool
        if (_type == "c" || _type == "c_bgra") 
        {            
            for(int i=0; i<wdevice_num * POOL_LENGTH; i++) 
                cframe_pool[i] = new uint8_t[shm_size];
        } 
        else if (_type == "d" || _type == "d_yuv16") 
        {
            for(int i=0; i<wdevice_num * POOL_LENGTH; i++) 
                dframe_pool[i] = new uint8_t[shm_size];
        }
        else
            LOG(FATAL) << "Unknown type: " << _type;
        
        for (int j=0; j<wdevice_num; j++) 
        {
            cframe_counter[j] = 0;
            dframe_counter[j] = 0;
        }
    }

    // Create socket
    boost::asio::io_service io_service;
    int _port = CLIENT_PORT + view_number;
    if (_type == "d" || _type == "d_yuv16") 
        _port += wdevice_num;
    server = new TcpServer(io_service, _port);

    // Create websocket client
    while(1) 
    {
        try
        {
            string ws_host = string("localhost");
            auto const ws_port = to_string(_type == "c" || _type == "c_bgra" ? 5552 : 5562);

            // Look up the domain name
            auto const results = resolver.resolve(ws_host, ws_port);

            LOG(INFO) << "resolved";

            // Make the connection on the IP address we get from a lookup
            auto ep = net::connect(ws.next_layer(), results);
            LOG(INFO) << "connected";

            // Update the host_ string. This will provide the value of the
            // Host HTTP header during the WebSocket handshake.
            // See https://tools.ietf.org/html/rfc7230#section-5.4
            ws_host += ':' + std::to_string(ep.port());

            // Set a decorator to change the User-Agent of the handshake
            ws.set_option(websocket::stream_base::decorator(
                [](websocket::request_type& req)
                {
                    req.set(http::field::user_agent,
                        std::string(BOOST_BEAST_VERSION_STRING) +
                            " websocket-client-coro");
                }));
            
            // Perform the websocket handshake
            ws.handshake(ws_host, "/");
            break;

        } 
        catch(boost::system::system_error const& e) 
        {
            if (e.code() == errc::connection_refused) 
            {
                LOG(INFO) << "Websocket Client Waiting to Connect to Host...";
                sleep(1);
            } 
            else
            {
                LOG(ERROR) << "Connection failed: " << e.what();
                break;
            }
        }
    }
}

// Old recv logic with flat buffer from WebRTC receiver
void WebRTCClient::recv_old() {
    StopWatch watch;
    while(1) {
        char* buf = new char[6];
        boost::system::error_code error_code;
        boost::asio::read(server->socket, boost::asio::buffer(buf, 24), error_code);

        // Deserialize frame_info
        auto frame_info = flatbuffers::GetRoot<FrameInfo>(buf);
        auto new_frame_number = frame_info->frame_number();
        auto new_view_number = frame_info->view_number();

        // LOG(INFO) << "Receiving " << new_frame_number;

        if (error_code == boost::system::errc::success) {
            // Copy from shared memory to frame_pool in cv::Mat
            if (!isTile) {
                if (_type == "c") {
                    recv_color(new_view_number, new_frame_number);
                } else if (_type == "d") {
                    recv_depth(new_view_number, new_frame_number);
                }
            } else {
                if (_type == "c") {
                    recv_tiled_color(new_frame_number);
                } else if (_type == "d") {
                    recv_tiled_depth(new_frame_number);
                }
                
                //  << "Current Unix Timestamp: " << watch.Curr() << " ms for frame " << new_frame_number << "\n";
            }        
        }

        else if (error_code == boost::asio::error::eof) {
            LOG(ERROR) << "WebRTCClient end of file";
            break;
        }

    }

}

void WebRTCClient::recv() 
{
    StopWatch watch;
    while(1) 
    {
        if (isTile) 
        {
            boost::beast::flat_buffer buffer;
            ws.read(buffer);

            std::string response(boost::asio::buffer_cast<const char*>(buffer.data()), buffer.size());
            // std::LOG(INFO) << "Received counter: " << response << " " << _type;
            
            if (_type == "c" || _type =="c_bgra")
            {
                StopWatch watch;
                watch.Restart();
                recv_tiled_color(stoi(response));
                // LOG(INFO) << "Time to receive color: " << watch.ElapsedMs() << " ms";
            }
            else if (_type == "d")
                recv_tiled_depth(stoi(response));
            else if(_type == "d_yuv16")
            {
                StopWatch watch;
                watch.Restart();
                recv_tiled_depth_yuv16(stoi(response));
                // LOG(INFO) << "Time to receive depth yuv16: " << watch.ElapsedMs() << " ms";
            }
            else
                LOG(FATAL) << "Unknown type: " << _type;
            recv_counter++;

            ws.write(net::buffer(string("done")));
        }
    }
}

// TODO: remove this and use tiled versions
void WebRTCClient::recv_color(int new_view_number, int new_frame_number) {
    memcpy(cframe_pool[new_view_number*POOL_LENGTH+new_frame_number%POOL_LENGTH], shm_ptr, shm_size);
    cframe_counter[new_view_number].fetch_add(1);
}

// TODO: remove this and use tiled versions
void WebRTCClient::recv_depth(int new_view_number, int new_frame_number) {
    memcpy(dframe_pool[new_view_number*POOL_LENGTH+new_frame_number%POOL_LENGTH], shm_ptr, shm_size);
    dframe_counter[new_view_number].fetch_add(1);
}

int WebRTCClient::detect_framenum_qrcode(RGB *buf, int width, int height, int border, bool rmCode)
{
    cv::Mat img = color_to_opencv(buf, width, height);

	int offset_x = 20;
	int offset_y = 20;

	cv::Mat tmp;
	cv::extractChannel(img, tmp, 0);    // Only check for blue channel. If required, we can test for all channels, but its time consuming.
	cv::Mat qrcode_img = tmp(cv::Range(offset_x, offset_x+82), cv::Range(offset_y, offset_y+82));

	// LOG(INFO) << "-----------------------------------------------";
	// Detect qr code in each channel

	vector<cv::Point> corners;
	corners.push_back(cv::Point(border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, qrcode_img.rows - border));
	corners.push_back(cv::Point(border, qrcode_img.rows - border));

	// string decode_info = m_qrDecoder.detectAndDecode(qrcode_img, corners);
	string decode_info = m_qrDecoder.decode(qrcode_img, corners);
	// LOG(INFO) << "Frame_number: " << decode_info;
	if(decode_info.length() > 0)
	{
		int32_t frame_id = stoi(decode_info);
		return frame_id;
	}

    // Remove QR code from image
    if(rmCode)
    {
        img(rectZero) = cv::Scalar(0,0,0);
        buf = reinterpret_cast<RGB *>(img.data);
    }

	return -1;
}

void WebRTCClient::recv_tiled_color(int new_frame_number) 
{   
    // Iterate from curr_counter to recv_counter
    int temp_counter = recv_counter;
    if (curr_counter == temp_counter) return;

    string outdir = "/datassd/pipeline/client_tiled/pipeline_new/test/";

    // 1*10 tiled layout (Server side seems wrong. Need to check)
    /*
    for (int id = curr_counter+1; id <= temp_counter; id++) {
        int cframeID = -1;

        for (int view_number = 0; view_number < wdevice_num; view_number++) {            
            uint8_t *buf = shm_ptr + 10 * cimg_w * cimg_h * 4 * (id % 30) + view_number * cimg_w * cimg_h * 4;

            if(view_number == 0) 
            {
                RGB *color_buf = reinterpret_cast<RGB *>(buf);
                cframeID = detect_framenum_qrcode(color_buf, cimg_w, cimg_h);         // cframeID == -1 if no QR code detected
                LOG(INFO) << "cframeID: " << cframeID << " received new frame number: " << new_frame_number;

                // // DEBUG: Save duplicates for debugging
                // if (isSaveView) {
                //     if(cframeID > 0 && cframeID == cframeID_prev)
                //     {
                //         save_color(shm_ptr, cimg_h * 10, cimg_w, FORMAT("/home/lei/data/pipeline/client_tiled/" << cframeID << "_color_dup.jpg"));
                //         save_color(frame_c_prev, cimg_h * 10, cimg_w, FORMAT("/home/lei/data/pipeline/client_tiled/" << cframeID << "_color.jpg"));
                //     }
                //     memcpy(frame_c_prev, shm_ptr, cimg_h * 10 * cimg_w * 4);
                //     cframeID_prev = cframeID;
                // }
                // save_color(buf, dimg_h, dimg_w, FORMAT("/home/lei/data/pipeline/client_tiled/" << id << "_0_color.png"));
            }
            if (cframeID != -1) {
                memcpy(cframe_pool[view_number+cframeID%POOL_LENGTH*wdevice_num], buf, shm_size);
                // LOG(INFO) << "memcpy " << cframeID << " to " << view_number+cframeID%POOL_LENGTH*wdevice_num;
                // memcpy(cframe_pool[view_number+cframeID%POOL_LENGTH], buf, shm_size);
                // LOG(INFO) << "memcpy " << cframeID << " to " << view_number+cframeID%POOL_LENGTH;
                cframe_index[cframeID%POOL_LENGTH] = cframeID;
                if (cframeID > cframe_max_index) cframe_max_index = cframeID;     
            }
        }
        // DEBUG: store image to local
        if (isSaveView && (cframeID % 10 == 0))
            for (int i = 0; i < wdevice_num; i++) {
                save_color(cframe_pool[cframeID%POOL_LENGTH * wdevice_num + i], dimg_h, dimg_w, FORMAT("/home/lei/data/pipeline/client_standalone/" << cframeID << "_" << i << "_color.png"));        
                // save_color(cframe_pool[cframeID%POOL_LENGTH + i], dimg_h, dimg_w, FORMAT("/home/lei/data/pipeline/client_standalone/" << cframeID << "_" << i << "_color.png"));        
            }
    }
    */

    // 2*5 tiled layout
    for (int id = curr_counter+1; id <= temp_counter; id++) 
    {
        uint8_t *buf = shm_ptr + (10 * cimg_w * cimg_h * CHANNEL_DIM::BGRA * (id % SHM_BUFFER_SIZE));
        RGB *color_buf = reinterpret_cast<RGB *>(buf);
        int cframeID = detect_framenum_qrcode(color_buf, cimg_w * 5, cimg_h * 2);         // cframeID == -1 if no QR code detected
        // LOG(INFO) << "cframeID: " << cframeID << " received new frame number: " << new_frame_number;

        // If QR not detected, skip
        if (cframeID != -1)
        {
            for (int view_number = 0; view_number < wdevice_num; view_number++) 
            {
                auto h = cimg_h;
                auto w = cimg_w;
                int x = view_number / 5 * h, y = view_number % 5 * (w * CHANNEL_DIM::BGRA);
                
                for (int j = 0; j < h; j++)
                    memcpy(cframe_pool[(cframeID % POOL_LENGTH) * wdevice_num + view_number] + (j * w * CHANNEL_DIM::BGRA), buf + ((x + j) * w * 5 * CHANNEL_DIM::BGRA + y), w * CHANNEL_DIM::BGRA);
                cframe_index[cframeID % POOL_LENGTH] = cframeID;
                if (cframeID > cframe_max_index) 
                    cframe_max_index = cframeID;   
            }
        }
        else
        {
            LOG(ERROR) << "Color - No QR code detected for local counter: " << id;
        }

        // DEBUG: store image to local
        if (isSaveView && (cframeID % 100 == 0))
        {
            if(!fs::exists(outdir))
                fs::create_directories(outdir);
            for (int i = 0; i < wdevice_num; i++)
                save_color(cframe_pool[cframeID % POOL_LENGTH * wdevice_num + i], cimg_h, cimg_w, FORMAT(outdir << cframeID << "_color_bgra_" << i << ".png"));
        }
    }
    curr_counter = temp_counter;
    return;
}

void WebRTCClient::recv_tiled_depth(int new_frame_number) 
{
    // Iterate from curr_counter to recv_counter
    int temp_counter = recv_counter;
    if (curr_counter == temp_counter) return;

    string outdir = "/home/lei/data/pipeline/client_tiled/pipeline_new/test/";

    // // 1*10 tiled layout
    /*
    for (int id = curr_counter+1; id <= temp_counter; id++) {
        int dframeID = -1;
        // if(frame_d_prev == NULL)
        //     frame_d_prev = new uint8_t[dimg_h * 10 * dimg_w * 4];

        for (int view_number = 0; view_number < wdevice_num; view_number++) {
            uint8_t *buf = shm_ptr + 10 * shm_size * (id % 30) + view_number * dimg_w * dimg_h * 4;

            if(view_number == 0)
            {
                //detect frame number from QR code
                RGB *colorized_depth_buf = reinterpret_cast<RGB *>(buf);
                dframeID = detect_framenum_qrcode(colorized_depth_buf, dimg_w, dimg_h);   // dframeID == -1 if no QR code detected
                LOG(INFO) << "dframeID: " << dframeID;

                // // DEBUG: Save duplicates for debugging
                // if(dframeID > 0 && dframeID == dframeID_prev)
                // {
                //     save_depth(shm_ptr, dimg_h * 10, dimg_w, FORMAT("/home/lei/data/pipeline/client_tiled/" << dframeID << "_depth_dup.jpg"));
                //     save_depth(frame_d_prev, dimg_h * 10, dimg_w, FORMAT("/home/lei/data/pipeline/client_tiled/" << dframeID << "_depth.jpg"));
                // }
                // memcpy(frame_d_prev, shm_ptr, dimg_h * 10 * dimg_w * 4);
                // dframeID_prev = dframeID;
                // save_depth(buf, dimg_h, dimg_w, FORMAT("/home/lei/data/pipeline/client_tiled/" << id << "_0_depth.png"));
            }

            if (dframeID != -1) {
                memcpy(dframe_pool[view_number+dframeID%POOL_LENGTH*wdevice_num], buf, shm_size);
                // LOG(INFO) << "memcpy " << dframeID << " to " << view_number+dframeID%POOL_LENGTH*wdevice_num;
                // memcpy(dframe_pool[view_number+dframeID%POOL_LENGTH], buf, shm_size);
                // LOG(INFO) << "memcpy " << dframeID << " to " << view_number+dframeID%POOL_LENGTH;
                dframe_index[dframeID%POOL_LENGTH] = dframeID;
                if (dframeID > dframe_max_index) dframe_max_index = dframeID;   
            }
        }

        // DEBUG: store image to local
        if (isSaveView && (dframeID % 10 == 0))
            for (int i = 0; i < wdevice_num; i++) {
                save_depth(dframe_pool[dframeID%POOL_LENGTH * wdevice_num + i], dimg_h, dimg_w, FORMAT("/home/lei/data/pipeline/client_standalone/" << dframeID << "_" << i << "_depth.png"));        
                // save_depth(dframe_pool[dframeID%POOL_LENGTH + i], dimg_h, dimg_w, FORMAT("/home/lei/data/pipeline/client_standalone/" << dframeID << "_" << i << "_depth.png"));        
            }
    }
    */

    
    // 2*5 tiled layout
    for (int id = curr_counter+1; id <= temp_counter; id++) 
    {
        uint8_t *buf = shm_ptr + (10 * dimg_w * dimg_h * CHANNEL_DIM::BGRA * (id % SHM_BUFFER_SIZE));

        RGB *colorized_depth_buf = reinterpret_cast<RGB *>(buf);
        int dframeID = detect_framenum_qrcode(colorized_depth_buf, dimg_w * 5, dimg_h * 2);   // dframeID == -1 if no QR code detected
        // LOG(INFO) << "dframeID: " << dframeID;

        if (dframeID != -1)
        {
            for (int view_number = 0; view_number < wdevice_num; view_number++) 
            {
                auto h = dimg_h;
                auto w = dimg_w;
                int x = view_number / 5 * h, y = view_number % 5 * w * 4;
                           
                for (int j = 0; j < h; j++) 
                    memcpy(dframe_pool[(dframeID % POOL_LENGTH) * wdevice_num + view_number] + (j * w * CHANNEL_DIM::BGRA), buf + ((x + j) * w * 5 * CHANNEL_DIM::BGRA + y), w * CHANNEL_DIM::BGRA);
                dframe_index[dframeID%POOL_LENGTH] = dframeID;
                if (dframeID > dframe_max_index) dframe_max_index = dframeID;                
            }
        }
         else
        {
            LOG(ERROR) << "Depth - No QR code detected for local counter: " << id;
        }

        // DEBUG: store image to local
        if (isSaveView && (dframeID % 10 == 0))
        {
            if(!fs::exists(outdir))
                fs::create_directories(outdir);
            for (int i = 0; i < wdevice_num; i++)
                save_color(dframe_pool[dframeID % POOL_LENGTH * wdevice_num + i], dimg_h, dimg_w, FORMAT(outdir << dframeID << "_depth_bgra_" << i << ".png"));
        }
    }

    curr_counter = temp_counter;
    return;
}

/**
 * @brief Detect frame number from QR code from u channel buffer
 * QR code is located at (20, 20) with size 82*82. We don't need to remove it, since u channel has no data
 * @param buf The buffer passed is only for u channel
 * @param width 
 * @param height 
 * @param border 
 * @param isScaled 
 * @return int 
 */
int WebRTCClient::detect_framenum_qrcode_yuv16(uint8_t *buf, int width, int height, int border, bool isScaled)
{
    cv::Mat img_u = cv::Mat(height, width, CV_16UC1, buf);

	int offset_x = 20;
	int offset_y = 20;

	cv::Mat qrcode_img = cv::Mat::zeros(82, 82, CV_8UC1);

    for(int i=0; i<qrcode_img.rows; i++)
    {
        for(int j=0; j<qrcode_img.cols; j++)
        {
            float pixel = (img_u.at<uint16_t>(i+offset_x, j+offset_y) * 255.0F) / 65535.0F;
            qrcode_img.at<uint8_t>(i,j) = uint8_t(pixel);
        }
    }

	// LOG(INFO) << "-----------------------------------------------";
	// Detect qr code in u channel

	vector<cv::Point> corners;
	corners.push_back(cv::Point(border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, qrcode_img.rows - border));
	corners.push_back(cv::Point(border, qrcode_img.rows - border));

	// string decode_info = m_qrDecoder.detectAndDecode(qrcode_img, corners);
	string decode_info = m_qrDecoder.decode(qrcode_img, corners);
	// LOG(INFO) << "Frame_number: " << decode_info;
	if(decode_info.length() > 0)
	{
		int32_t frame_id = stoi(decode_info);
		return frame_id;
	}

	return -1;
}

/**
 * @brief In-place convert yuv16 to depth
 * 
 * @param buf 
 * @param height 
 * @param width 
 * @param isScaled Unscale y channel if true
 */
void WebRTCClient::yuv16_to_depth(uint16_t *depth, int height, int width, bool isScaled)
{
    if(isScaled)
    {
        for(int i=0; i<height*width; i++)
        {
            float d = (depth[i] * 6000.0F) /  65535.0F;
            depth[i] = uint16_t(d);
        }
    }
}

void WebRTCClient::recv_tiled_depth_yuv16(int new_frame_number) 
{
    // Iterate from curr_counter to recv_counter
    int temp_counter = recv_counter;
    if (curr_counter == temp_counter) return;

    string outdir = "/datassd/pipeline/client_tiled/pipeline_new/test/";

    // 2*5 tiled layout
    for (int id = curr_counter+1; id <= temp_counter; id++) 
    {
        uint8_t *buf = shm_ptr + (10 * dimg_w * dimg_h * CHANNEL_DIM::DEPTH * PIXEL_SIZE::DEPTH * (id % SHM_BUFFER_SIZE));

        // Memory layout: flattened y channel, followed by u channel, followed by v channel
        // Unscale y channel
        uint16_t *buf_y = reinterpret_cast<uint16_t *>(buf);
        yuv16_to_depth(buf_y, dimg_h * 2, dimg_w * 5);

        uint8_t *buf_u = buf + (10 * dimg_w * dimg_h * PIXEL_SIZE::DEPTH); // u channel 
        int dframeID = detect_framenum_qrcode_yuv16(buf_u, dimg_w * 5, dimg_h * 2);   // dframeID == -1 if no QR code detected
        // LOG(INFO) << "dframeID: " << dframeID;

        if (dframeID != -1)
        {
            for (int view_number = 0; view_number < wdevice_num; view_number++) 
            {
                auto h = dimg_h;
                auto w = dimg_w;
                int x = view_number / 5 * h, y = view_number % 5 * (w * PIXEL_SIZE::DEPTH);

                uint8_t *shm_buf = shm_ptr + (10 * h * w * CHANNEL_DIM::DEPTH * PIXEL_SIZE::DEPTH * (id % SHM_BUFFER_SIZE));
                for (int j = 0; j < h; j++)
                    memcpy(dframe_pool[(dframeID % POOL_LENGTH) * wdevice_num + view_number] + (j * w * PIXEL_SIZE::DEPTH), shm_buf + ((x + j) * 5 * w * PIXEL_SIZE::DEPTH + y), w * PIXEL_SIZE::DEPTH);

                dframe_index[dframeID % POOL_LENGTH] = dframeID;
                if (dframeID > dframe_max_index) 
                    dframe_max_index = dframeID;                
            }
        }
        else
        {
            LOG(ERROR) << "Depth YUV16 - No QR code detected for local counter: " << id;
        }

        // DEBUG: store image to local
        if (isSaveView && (dframeID % 100 == 0))
        {
            if(!fs::exists(outdir))
                fs::create_directories(outdir);
            
            for (int i = 0; i < wdevice_num; i++) 
            {
                uint8_t *depth_buf = dframe_pool[(dframeID % POOL_LENGTH * wdevice_num) + i];
                cv::Mat depth_image_cv2(dimg_h, dimg_w, CV_16UC1, depth_buf);
                string path = FORMAT(outdir << dframeID << "_depth_yuv16_" << i << ".png");
                if(!cv::imwrite(path, depth_image_cv2))
                    LOG(ERROR) << "Failed to write depth image: " << path;
            }
        }
    }

    curr_counter = temp_counter;
    return;
}

WebRTCClientPool::WebRTCClientPool(int _view_count, bool isTile = false, bool isSaveView = false) : 
                                view_count(_view_count), isTile(isTile), isSaveView(isSaveView) 
{
    cframe_pool = new uint8_t*[POOL_LENGTH * wdevice_num];
    dframe_pool = new uint8_t*[POOL_LENGTH * wdevice_num];
    cframe_index = new int[POOL_LENGTH]{-1};
    dframe_index = new int[POOL_LENGTH]{-1};
    cframe_max_index = -1;
    dframe_max_index = -1;
    // next_views = new vector<View *>[POOL_LENGTH];

    if(wdevice_num != view_count)
        LOG(ERROR) << "wdevice_num != view_count";
    for (int i=0; i<view_count; i++) 
    {
        View* v = new View();
        v->initView(dimg_w, dimg_h, cimg_w, cimg_h);

        next_views.push_back(v);
    }

    c_frame_temp = new RGB*[view_count];
    d_frame_temp = new uint16_t*[view_count];
    depth_buf = new uint8_t*[view_count];
    colorized_depth = new RGB*[view_count];

    for (int i=0; i<view_count; i++) 
    {
        c_frame_temp[i] = new RGB[cimg_h * cimg_w * CHANNEL_DIM::BGRA];
        d_frame_temp[i] = new uint16_t[cimg_h * cimg_w * CHANNEL_DIM::BGRA];
        depth_buf[i] = new uint8_t[cimg_h * cimg_w * CHANNEL_DIM::BGRA]; 
        colorized_depth[i] = new RGB[cimg_h * cimg_w * CHANNEL_DIM::BGRA];
    }
}

// Added for standalone client
WebRTCClientPool::WebRTCClientPool(const string &path, int _view_count, bool isTile = false, bool isSaveView = false) : view_count(_view_count), isTile(isTile), isSaveView(isSaveView) 
{

    m_WorkDir = path;

    //----------------- READ DEVICE DETAILS FROM FILE -----------------//
    fs::ifstream file;
	fs::path filepath{FORMAT(m_WorkDir << DEVICE_FILE_NAME << ".txt")};
	file.open(filepath, ios::in);
	if(!file.is_open())
		LOG(FATAL) << "Failed to open file: " << filepath;

	if(!deviceIndices_recv.empty())
		deviceIndices_recv.clear();
	if(!deviceSerialNumbers_recv.empty())
		deviceSerialNumbers_recv.clear();

	uint32_t device_index;
	string device_serial_number;

	while(file >> device_index >> device_serial_number)
	{
		deviceIndices_recv.push_back(device_index);
		deviceSerialNumbers_recv.push_back(device_serial_number);
	}
	
	file.close();
	LOG(INFO) << "Device details loaded from: " << filepath;
    
    //----------------- READ CALIBRATION FROM FILE -----------------//
    for (uint32_t i : deviceIndices_recv)
	{
        // Only supports panoptic for now
        Calibration* temp = new Calibration();
		if(!temp->loadCalibration(deviceSerialNumbers_recv[i], m_WorkDir, true))
		{
			LOG(FATAL) << "Failed to load calibration for camera ID " << i << " serial ID: " << deviceSerialNumbers_recv[i];
		}
        calibrations_recv.push_back(temp);
	}

    cimg_h = 512;
    cimg_w = 592;
    dimg_h = 512;
    dimg_w = 592;

    cframe_pool = new uint8_t*[POOL_LENGTH*wdevice_num];
    dframe_pool = new uint8_t*[POOL_LENGTH*wdevice_num];
    cframe_index = new int[POOL_LENGTH]{-1};
    dframe_index = new int[POOL_LENGTH]{-1};
    cframe_max_index = -1;
    dframe_max_index = -1;
    // next_views = new vector<View *>[POOL_LENGTH];

    if(wdevice_num != view_count)
        LOG(ERROR) << "wdevice_num != view_count";
    for (int i=0; i<view_count; i++) 
    {
        View* v = new View();
        v->initView(dimg_w, dimg_h, cimg_w, cimg_h);

        next_views.push_back(v);
    }

    c_frame_temp = new RGB*[view_count];
    d_frame_temp = new uint16_t*[view_count];
    depth_buf = new uint8_t*[view_count];
    colorized_depth = new RGB*[view_count];

    for (int i=0; i<10; i++) 
    {
        c_frame_temp[i] = new RGB[cimg_h * cimg_w * CHANNEL_DIM::BGRA];
        d_frame_temp[i] = new uint16_t[cimg_h * cimg_w * CHANNEL_DIM::BGRA];
        depth_buf[i] = new uint8_t[cimg_h * cimg_w * CHANNEL_DIM::BGRA]; 
        colorized_depth[i] = new RGB[cimg_h * cimg_w * CHANNEL_DIM::BGRA];
    }
    LOG(INFO) << "Client calibrations ready";
    main_worker_ready_to_init.store(true);
}

void WebRTCClientPool::client_init(int view_number, string type) {
    WebRTCClient client(view_number, type, isTile, isSaveView);
    // For collecting frameinfo
    client.recv();
}

void WebRTCClientPool::run() {
    if(!isTile) {
        for (int i=0; i<view_count; i++) {
            cthread_pool.push_back(thread(&WebRTCClientPool::client_init, this, i, "c"));
            dthread_pool.push_back(thread(&WebRTCClientPool::client_init, this, i, "d"));
        }

        thread t(&WebRTCClientPool::client_signal, this);
        t.join();
        for (int i=0; i<view_count; i++) {
            cthread_pool[i].join();
            dthread_pool[i].join();
        }
    } else {

        cthread_pool.push_back(thread(&WebRTCClientPool::client_init, this, 0, "c"));

        // // Rajrup: DEPTH_BGRA
        // dthread_pool.push_back(thread(&WebRTCClientPool::client_init, this, 0, "d"));

        // Rajrup: DEPTH_YUV16
        dthread_pool.push_back(thread(&WebRTCClientPool::client_init, this, 0, "d_yuv16"));

        thread t(&WebRTCClientPool::client_signal, this);
        t.join();
        cthread_pool[0].join();
        dthread_pool[0].join();
    }
}

// Load views and maintain 30fps
void WebRTCClientPool::client_signal() 
{
    StopWatch sw;
    // sw.Restart();
    while(true) 
    {
        bool is_next = true;
        int lag_counter = 0;

        // sleep() doesn't take float.
        // TODO: Rajrup: Change this to use chrono 
        while(bool x = next_frame_ready.load(memory_order_relaxed) == true)
            sleep_ms(1);
            // sleep(0.001); 
 
        // Current available frame number
        int curr_min_index = cframe_max_index < dframe_max_index ? cframe_max_index : dframe_max_index;
        if (next_frame == 0 && curr_min_index >= 0)
            next_frame = curr_min_index - 1;
        
        // Move next_frame forward
        if (next_frame < curr_min_index) 
        {
            next_frame.fetch_add(1);
            // LOG(INFO) << "WebRTCClient received " << next_frame;
            load_view();
            next_frame_ready.store(true);
        } 
        else
            sleep_ms(1);
            // sleep(0.001);
    }
}

void WebRTCClientPool::buf2color(int view, int frame) 
{
    // LOG(INFO) << "buf2color " << frame << " " << view  +frame*wdevice_num;
    c_frame_temp[view] = reinterpret_cast<RGB *>(cframe_pool[view + frame * wdevice_num]);
    
    // LOG(INFO) << "buf2color " << frame << " " << view  +frame*wdevice_num;
    // c_frame_temp[view] = reinterpret_cast<RGB *>(cframe_pool[view * POOL_LENGTH + frame]);
    return;
}

void WebRTCClientPool::buf2depth(int view, int frame) 
{
    // LOG(INFO) << "buf2depth " << frame << " " << view  +frame*wdevice_num;
    depth_buf[view] = dframe_pool[view + frame * wdevice_num];

    // LOG(INFO) << "buf2depth " << frame << " " << view * POOL_LENGTH + frame;
    // depth_buf[view] = dframe_pool[view * POOL_LENGTH + frame];
    colorized_depth[view] = reinterpret_cast<RGB *>(depth_buf[view]);

    color_to_depth(colorized_depth[view], dimg_h, dimg_w, d_frame_temp[view], false);
    return;
}

void WebRTCClientPool::buf2depth_yuv16(int view, int frame) 
{
    d_frame_temp[view] = reinterpret_cast<uint16_t *>(dframe_pool[view + frame * wdevice_num]);
    return;
}

// Load view for next_frame. If frames are lost, roll to the next available frame
void WebRTCClientPool::load_view() 
{
    int frame = next_frame;
    // LOG(INFO) << "LOAD VIEW: Finding frame: " << frame;

    // Calculate available frame index
    int curr_cframe_index = frame % POOL_LENGTH;
    int curr_dframe_index = frame % POOL_LENGTH;

    // // Option 1: Choose newest c and d frame. They might be not sync
    // if (cframe_index[curr_cframe_index] < frame) {
    //     int start = curr_cframe_index;
    //     while (cframe_index[curr_cframe_index] < frame) {
    //         curr_cframe_index = (curr_cframe_index + 1) % POOL_LENGTH;
    //         if (curr_cframe_index == start) {
    //             LOG(INFO) << "CFRAME OVERLAPPED! " << frame << " is using " << frame % POOL_LENGTH;
    //             curr_cframe_index = frame % POOL_LENGTH;
    //             break;
    //         }            
    //     }    
    // }

    // if (dframe_index[curr_dframe_index] < frame) {
    //     int start = curr_dframe_index;
    //     while (dframe_index[curr_dframe_index] < frame) {
    //         curr_dframe_index = (curr_dframe_index + 1) % POOL_LENGTH;
    //         if (curr_dframe_index == start) {
    //             LOG(INFO) << "DFRAME OVERLAPPED! " << frame << " is using " << frame % POOL_LENGTH;
    //             curr_dframe_index = frame % POOL_LENGTH;
    //             break;
    //         }            
    //     }    
    // }

    /**
     * @brief TODO Rajrup: Remove the while loop to find the frame. 
     * Logic: If cframe_index[curr_cframe_index] != frame, then the frame is lost or corrupt.
     */
    // Option 2: Definitely choose synchronized c and d
    if (cframe_index[curr_cframe_index] < frame || dframe_index[curr_dframe_index] < frame) 
    {
        int start = curr_cframe_index;
        while (cframe_index[curr_cframe_index] < frame && dframe_index[curr_dframe_index] < frame) 
        {
            LOG(INFO) << "In loop: Finding - " << frame << ", Received - Color: " << cframe_index[curr_cframe_index] << ", Depth: " << dframe_index[curr_dframe_index];
            curr_cframe_index = (curr_cframe_index + 1) % POOL_LENGTH;
            curr_dframe_index = (curr_dframe_index + 1) % POOL_LENGTH;
            if (curr_cframe_index == start) 
            {
                LOG(ERROR) << "FRAME OVERLAPPED! " << frame << " is using " << frame % POOL_LENGTH;
                curr_cframe_index = frame % POOL_LENGTH;
                curr_dframe_index = frame % POOL_LENGTH;

                break;
            }            
        }    
    }

    int cframeID = cframe_index[curr_cframe_index];
    int dframeID = dframe_index[curr_dframe_index];

    // LOG(INFO) << "WebRTCClient load view " << frame << " " << cframeID << " " << dframeID << " " << curr_cframe_index << " " << curr_dframe_index;
    // if (curr_cframe_index != frame % POOL_LENGTH || curr_dframe_index != frame % POOL_LENGTH)
    //     LOG(INFO) << "WebRTCClient using different frame! " << frame << " " << cframeID << " " << dframeID;

    // If frames are lost - QR code lost, drop this frame on the client side
    // LOG(INFO) << "Finding - " << frame << " Found - Color: " << cframeID << ", Depth: " << dframeID;
    if(cframeID != dframeID || cframeID != frame || dframeID != frame)
    {
        LOG(ERROR) << "Color and Depth frame mismatch for frame: " << frame << ", Color: " << cframeID << ", Depth: " << dframeID;
#pragma omp parallel for
        for (int i=0; i<view_count; i++) 
            next_views[i]->valid = false;
        return;
    };

#pragma omp parallel for
    for (int i=0; i<view_count * 2; i++) 
    {
        if (i % 2) 
            buf2color(i/2, curr_cframe_index);
        else
        {
            // // Rajrup: DEPTH_BGRA
            // buf2depth(i/2, curr_dframe_index);

            // Rajrup: DEPTH_YUV16
            buf2depth_yuv16(i/2, curr_dframe_index);
        }
    }

#pragma omp parallel for
    for (int i=0; i<view_count; i++) {
        next_views[i]->viewID = i;
        next_views[i]->frameID = cframeID;
        next_views[i]->valid = true;
        next_views[i]->color_image = c_frame_temp[i];
        next_views[i]->depth_image = d_frame_temp[i];
    }
    return;
}

// Stores latest received images
bool WebRTCClientPool::store_view()
{
	for(int i=0; i<view_count; i++)
	{
        cv::Mat color_image_cv2(cimg_h, cimg_w, CV_8UC4, next_views[i]->color_image);
        cv::Mat depth_image_cv2(dimg_h, dimg_w, CV_16U, next_views[i]->depth_image);

		string color_image_path = FORMAT("/home/lei/test/output/client/" << next_frame << "_color_" << i << ".png");
		string depth_image_path = FORMAT("/home/lei/test/output/client/" << next_frame << "_depth_" << i << ".png");
		
		if(!cv::imwrite(color_image_path, color_image_cv2, compression_params))
		{
			LOG(INFO) << "Failed to write color image: " << color_image_path;
			return false;
		}
		if(!cv::imwrite(depth_image_path, depth_image_cv2, compression_params))
		{
			LOG(INFO) << "Failed to write depth image: " << depth_image_path;
			return false;
		}
	}
	return true;
}

WebSocketClient::WebSocketClient(const char* addr, int port, const int isSendPtcl): 
        _addr(addr), _port(port), 
        resolver1(ioc1), ws1(ioc1), 
        resolver2(ioc2), ws2(ioc2), 
        resolver3(ioc3), ws3(ioc3),
        resolver_ptcl(ioc_ptcl), ws_ptcl(ioc_ptcl),
        _isSendPtcl(isSendPtcl) 
{
    if(_isSendPtcl)
    {
        compressed_ptcl_buf = new uint8_t[PTCL_MAX_POINTS * 3 * (sizeof(float) + sizeof(uint8_t))];
        compressed_ptcl_buf_size = 0;
    }

    while(1) 
    {
        try
        {
            host1 = string(_addr);
            host2 = string(_addr);
            host3 = string(_addr);
            host_ptcl = string(_addr);
            auto const port1 = to_string(_port);
            auto const port2 = to_string(_port+1);
            auto const port3 = to_string(_port+2);
            auto const port_ptcl = to_string(_port+3);

            // Look up the domain name
            auto const results1 = resolver1.resolve(host1, port1);
            auto const results2 = resolver2.resolve(host2, port2);
            auto const results3 = resolver3.resolve(host3, port3);
            auto const results_ptcl = resolver_ptcl.resolve(host_ptcl, port_ptcl);

            LOG(INFO) << "Server host and port resolved";

            // Make the connection on the IP address we get from a lookup
            auto ep1 = net::connect(ws1.next_layer(), results1);
            LOG(INFO) << "Websocket client connected to server: " << ep1.address().to_string() << ":" << ep1.port();
            LOG(INFO) << "This socket will receive calibrations and send frustums";
            
            auto ep2 = net::connect(ws2.next_layer(), results2);
            LOG(INFO) << "Websocket client connected to server: " << ep2.address().to_string() << ":" << ep2.port();
            LOG(INFO) << "This socket send masks";

            auto ep3 = net::connect(ws3.next_layer(), results3);
            LOG(INFO) << "Websocket client connected to server: " << ep3.address().to_string() << ":" << ep3.port();
            LOG(INFO) << "This socket will send client side mahimahi bandwdith estimate (bwe)";

            // Update the host_ string. This will provide the value of the
            // Host HTTP header during the WebSocket handshake.
            // See https://tools.ietf.org/html/rfc7230#section-5.4
            host1 += ':' + std::to_string(ep1.port());
            host2 += ':' + std::to_string(ep2.port());
            host3 += ':' + std::to_string(ep3.port());

            // Set a decorator to change the User-Agent of the handshake
            ws1.set_option(websocket::stream_base::decorator(
                [](websocket::request_type& req)
                {
                    req.set(http::field::user_agent,
                        std::string(BOOST_BEAST_VERSION_STRING) +
                            " websocket-client-coro");
                }));
            
            ws2.set_option(websocket::stream_base::decorator(
                [](websocket::request_type& req)
                {
                    req.set(http::field::user_agent,
                        std::string(BOOST_BEAST_VERSION_STRING) +
                            " websocket-client-coro");
                }));
            
            ws3.set_option(websocket::stream_base::decorator(
                [](websocket::request_type& req)
                {
                    req.set(http::field::user_agent,
                        std::string(BOOST_BEAST_VERSION_STRING) +
                            " websocket-client-coro");
                }));
            
            if(_isSendPtcl)
            {
                auto ep_ptcl = net::connect(ws_ptcl.next_layer(), results_ptcl);
                LOG(INFO) << "Websocket client connected to server: " << ep_ptcl.address().to_string() << ":" << ep_ptcl.port();
                LOG(INFO) << "This socket will receive point clouds";

                host_ptcl += ':' + std::to_string(ep_ptcl.port());
                ws_ptcl.set_option(websocket::stream_base::decorator(
                    [](websocket::request_type& req)
                    {
                        req.set(http::field::user_agent,
                            std::string(BOOST_BEAST_VERSION_STRING) +
                                " websocket-client-coro");
                    }));
            }

            // Perform the websocket handshake
            ws1.handshake(host1, "/");
            ws2.handshake(host2, "/");
            ws3.handshake(host3, "/");
            if(_isSendPtcl)
                ws_ptcl.handshake(host_ptcl, "/");
            break;

        } 
        catch(boost::system::system_error const& e)
        {
            if (e.code() == errc::connection_refused) 
            {
                LOG(INFO) << "Websocket Client Waiting to Connect to Host...";
                sleep(1);
            } 
            else 
            {
                LOG(INFO) << "Connection failed: " << e.what();
                break;
            }
        }
    }
}

WebSocketClient::~WebSocketClient() {
    ws1.close(websocket::close_code::normal);
    ws2.close(websocket::close_code::normal);
    ws3.close(websocket::close_code::normal);

}

void WebSocketClient::run() 
{
    try
    {
        thread t1 = std::thread(&WebSocketClient::get_calibration, this);       
        thread t2 = std::thread(&WebSocketClient::sendFrustum, this);
        // thread t2 = std::thread(&WebSocketClient::sendFrustumClip, this);
        thread t3 = std::thread(&WebSocketClient::get_mask, this);
        thread t4 = std::thread(&WebSocketClient::sendBwe, this);
        thread t5 = _isSendPtcl ? std::thread(&WebSocketClient::get_ptcl, this) : std::thread();
        
        t1.join();
        t2.join();
        // t3.join();
        t4.join();
        t5.join();
    }
    catch (const std::exception& e)
    {
        LOG(ERROR) << "Error: " << e.what();
        return;
    }
}

/**
 * @brief Receives camera calibrations from server over websocket.
 */
void WebSocketClient::get_calibration() 
{
    LOG(INFO) << "This thread will receive calibrations over socket: " << ws1.next_layer().remote_endpoint().address().to_string() << ":" << ws1.next_layer().remote_endpoint().port();
    ws1.read_message_max(20ull << 20); // sets the limit to 20 miB
    try
    {         
        auto const  text = "hello";

        // Send the message
        ws1.write(net::buffer(std::string(text)));
        LOG(INFO) << "Handshake sent to Server";
        ws1.binary(true);

        for(;;) 
        {
            // This buffer will hold the incoming message
            beast::flat_buffer buffer (1024*1024);
            // // Read a message into our buffer
            try 
            {
                ws1.read(buffer);

                // parse array from json format
                auto text_data = json::from_msgpack(buffers_begin(buffer.data()), buffers_end(buffer.data()));

                if (text_data[0] == "calibration"){
                    LOG(INFO) << "Received handshake from Server";
                    wdevice_num = text_data[1];

                    for (int i=0; i<wdevice_num; i++) {
                        Calibration* temp = new Calibration;
                        temp->worldT = text_data[2+8*i].get<vector<float>>();
                        temp->worldR = text_data[3+8*i].get<vector<vector<float>>>();
                        temp->bCalibrated = true;

                        temp->intrinsics.fx = text_data[4+8*i];
                        temp->intrinsics.fy = text_data[5+8*i];
                        temp->intrinsics.cx = text_data[6+8*i];
                        temp->intrinsics.cy = text_data[7+8*i];
                        temp->intrinsics.width = text_data[8+8*i];
                        temp->intrinsics.height = text_data[9+8*i];
                        
                        calibrations_recv.push_back(temp);
                    }

                    cimg_w = text_data[8];
                    dimg_w = text_data[8];
                    cimg_h = text_data[9];
                    dimg_h = text_data[9];

                    // for (int i=0; i<wdevice_num; i++) {
                    //     xy_table* temp = new xy_table;
                    //     temp->width = text_data[1+2*wdevice_num+4*i];
                    //     temp->height = text_data[1+2*wdevice_num+4*i+1];

                    //     temp->x_table = new float[temp->width * temp->height];
                    //     temp->y_table = new float[temp->width * temp->height];
                        
                    //     auto temp_x = text_data[1+2*wdevice_num+4*i+2].get<vector<float>>();
                    //     temp->x_table = temp_x.data();

                    //     auto temp_y = text_data[1+2*wdevice_num+4*i+3].get<vector<float>>();
                    //     temp->y_table = temp_y.data();

                    //     xy_tables_recv.push_back(temp);
                    // }

                    deviceIndices_recv = text_data[2+8*wdevice_num].get<vector<uint32_t>>();
                    deviceSerialNumbers_recv = text_data[2+8*wdevice_num+1].get<vector<string>>();

                    LOG(INFO) << "Client get calibrations";
                    main_worker_ready_to_init.store(true);
                }
            } 
            
            catch (std::exception const& e)
            {
                LOG(ERROR) << "Error on websocket recv: " << e.what();
            }

        }
    }
    catch(std::exception const& e)
    {
        LOG(ERROR) << "Error on read: " << e.what();
        return;
    }
    return;
}

/**
 * @brief Sends viewer's frustum to server over websocket.
 */
void WebSocketClient::sendFrustum() 
{
    LOG(INFO) << "This thread will send frustums over socket: " << ws1.next_layer().remote_endpoint().address().to_string() << ":" << ws1.next_layer().remote_endpoint().port();
    ws1.binary(true);
    for(;;) 
    {
        while(bool x = new_frustum.load(memory_order_relaxed) == false)
            sleep_ms(1);

        // LOG(INFO) << "Sending frustum to server";

        try
        { 
            // serialization
            auto text_data = json::array();
            
            // LOG(INFO) << "Sending frustum to server";
            // frustum_send.print();

            // Add frustum instrinsics
            text_data.push_back(frustum_send.angle);
            text_data.push_back(frustum_send.ratio);
            text_data.push_back(frustum_send.nearD);
            text_data.push_back(frustum_send.farD);

            // Add frustum extrinsics
            for(int i=0; i<3; i++)
                text_data.push_back(frustum_send.pos(i));
            
            for(int i=0; i<4; i++)
                text_data.push_back(frustum_send.quat(i));

            // Add frame ID of the frustum
            text_data.push_back(frustum_send.frameID);

            // Add send timestamp in ms
            uint64_t timestamp = time_now_ms();
            text_data.push_back(timestamp);

            // // Send the message
            ws1.write(net::buffer(json::to_msgpack(text_data)));
        }
        catch(std::exception const& e)
        {
            LOG(ERROR) << "Error on send: " << e.what();
            new_frustum.store(false);
        }
        new_frustum.store(false);
    }
    return;
}

/**
 * @brief Sends viewer's frustum (in clip format) to server over websocket.
 */
void WebSocketClient::sendFrustumClip() 
{
    LOG(INFO) << "This thread will send frustums over socket: " << ws1.next_layer().remote_endpoint().address().to_string() << ":" << ws1.next_layer().remote_endpoint().port();
    for(;;) 
    {
        while(bool x = new_frustumClip.load(memory_order_relaxed) == false) 
        {
            sleep(0.01);
        }

        LOG(INFO) << "Sending frustum to server";

        try
        { 
            // serialization
            auto text_data = json::array();
            
            for (int i=0; i<N_FRUSTRUM_PLANE; i++) {
                for (int j=0; j<frustumClip_send.pl[i].eq.size(); j++)
                    text_data.push_back(frustumClip_send.pl[i].eq[j]);
            }

            // // Send the message
            ws1.write(net::buffer(json::to_msgpack(text_data)));
        }
        catch(std::exception const& e)
        {
            LOG(ERROR) << "Error on send: " << e.what();
            new_frustumClip.store(false);
        }
        new_frustumClip.store(false);
    }
    return;
}

/**
 * @brief Receives masks from server over websocket.
 */
void WebSocketClient::get_mask() 
{
    LOG(INFO) << "This thread will receive masks over socket: " << ws2.next_layer().remote_endpoint().address().to_string() << ":" << ws2.next_layer().remote_endpoint().port();
    ws2.read_message_max(20ull << 20); // sets the limit to 20 miB
    try
    {         
        auto const text = "hello";

        // Send the message
        ws2.write(net::buffer(std::string(text)));
        LOG(INFO) << "Handshake sent to Server";
        ws2.binary(true);

        for(;;) {
            // This buffer will hold the incoming message
            try {
                if (_isSendPtcl == 0) {
                    beast::multi_buffer buffer;
                    ws2.read(buffer);
                    int counter = 0;

                    // mask_recv = new uint8_t[buffer.size()];
                    // for (auto it = boost::asio::buffers_begin(buffer.data()); it != boost::asio::buffers_end(buffer.data()); it++) {
                    //     mask_recv[counter] = *it;
                    //     counter ++ ;
                    // }
                }
            }
            
            catch (std::exception const& e)
            {
                LOG(ERROR) << "Error on websocket recv: " << e.what();
                if (_isSendPtcl == 0) 
                    new_mask.store(false);
            }
        }
    }
    catch(std::exception const& e)
    {
        LOG(ERROR) << "Error on read: " << e.what();
        return;
    }
    return;
}

/**
 * @brief Sends client side mahimahi bandwidth estimate (bwe) to server over websocket.
 */
void WebSocketClient::sendBwe() 
{
    LOG(INFO) << "This thread will send bwe over socket: " << ws3.next_layer().remote_endpoint().address().to_string() << ":" << ws3.next_layer().remote_endpoint().port();

    try
    {
        auto const text = "hello";

        // Send the message
        ws3.write(net::buffer(std::string(text)));
        LOG(INFO) << "Handshake sent to Server";
        ws3.binary(true);

        int prev_sent_bwe = -1;
        for(;;) 
        {
            try
            {
                int bwe_now = bwe.load();
                if (bwe_now != prev_sent_bwe) 
                {
                    // serialization
                    auto text_data = json::array();
                    
                    text_data.push_back(bwe_now);

                    // Send the message
                    ws3.write(net::buffer(json::to_msgpack(text_data)));
                    // LOG(INFO) << "Sent bwe to server: " << bwe_now;
                    // sleep(0.01);
                    
                    prev_sent_bwe = bwe_now;
                }
                sleep_ms(10);
            }
            catch(std::exception const& e)
            {
                LOG(ERROR) << "Error on send: " << e.what();
            }
        }
    }
    catch(std::exception const& e)
    {
        LOG(ERROR) << "Error on read: " << e.what();
        return;
    }
    return;
}

/**
 * @brief Receives point clouds from server over websocket
 */
void WebSocketClient::get_ptcl() 
{
    LOG(INFO) << "This thread will receive point clouds over socket: " << ws_ptcl.next_layer().remote_endpoint().address().to_string() << ":" << ws_ptcl.next_layer().remote_endpoint().port();
    if(_isSendPtcl == 0)
    {
        LOG(FATAL) << "Only used for receiving point clouds";
        return;
    }

    ws_ptcl.read_message_max(0ull); // A value of zero indicates a limit of the maximum value of a uint64_t in bytes.
    try
    {         
        auto const text = "hello";

        // Send the message
        ws_ptcl.write(net::buffer(std::string(text)));
        LOG(INFO) << "Handshake sent to Server";
        ws_ptcl.binary(true);

        for(;;) 
        {
            try 
            {
                while(bool x = next_frame_ready.load(memory_order_relaxed) == true)
                    sleep_ms(1);

                // This buffer will hold the incoming message
                beast::multi_buffer buffer;
                ws_ptcl.read(buffer);
                LOG(INFO) << "Received data. Data Size: " << buffer.size();

                int frameID = 0;

                int counter = 0;
                compressed_ptcl_buf_size = 0;

                for (auto it = boost::asio::buffers_begin(buffer.data()); it != boost::asio::buffers_end(buffer.data()); ++it)
                {
                    if(counter < sizeof(frameID))
                    {
                        int msb = (uint8_t)*it;
                        frameID = frameID | (msb << (counter * 8));
                    }
                    else
                    {
                        // LOG(INFO) << "test";
                        compressed_ptcl_buf[compressed_ptcl_buf_size++] = *it;
                        // break;
                    }
                    counter++;
                }
                next_frame = frameID;

                LOG(INFO) << "Received - Frame ID: " << frameID << " Compressed ptcl Size: " << compressed_ptcl_buf_size << " bytes";
                next_frame_ready.store(true);
            }
            catch (std::exception const& e)
            {
                LOG(ERROR) << "Error on websocket recv: " << e.what();
                next_frame_ready.store(false);
            }
        }
    }
    catch(std::exception const& e)
    {
        LOG(ERROR) << "Error on read: " << e.what();
        return;
    }
    return;
}