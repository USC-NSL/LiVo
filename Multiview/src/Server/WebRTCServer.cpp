#include "Server/WebRTCServer.h"
#include "DataPlayback.h"
#include <boost/exception_ptr.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <chrono>
#include "timer.h"

#include <omp.h>
#include <zstd.h>

#define SHM_BUFFER_SIZE 30      // Number of buffers in shared memory. 30 frames when streamed at 30 fps will contain 1 sec of data.
#define PTCL_MAX_POINTS 2000000 // Maximum number of points in point cloud. Used for allocating memory for compressed point cloud buffer.

using namespace std;
using namespace boost::interprocess;
using boost::asio::ip::tcp;
using namespace Multiview::Frame;
using json = nlohmann::json;

namespace fs = boost::filesystem;

atomic_bool wsserver_ready_to_init(false);

vector<Calibration *> calibrations_init;
vector<xy_table_t *> xy_tables_init;
vector<uint32_t> deviceIndices_init;
vector<string> deviceSerialNumbers_init;

atomic_bool new_frustumClip(false);
FrustumClip frustumClip_recv;

atomic_bool new_frustum(false);
Frustum frustum_recv;
uint64_t frustum_sent_time; // Time when frustum was sent to client in ms
uint64_t frustum_recv_time; // Time when frustum was received from client in ms

atomic_bool new_mask(false);
// uint8_t *mask_send;
// char *mask_send;
bool *mask_send;

atomic_int bwe(-1);
atomic_int cbwe(-1);
atomic_int dbwe(-1);

atomic_bool new_ptcl(false);
uint8_t *ptcl_buf;
uint32_t ptcl_buf_size;

TcpClient::TcpClient(boost::asio::io_service& svc, std::string const& host, int const& port): io_service(svc), socket(io_service) 
{
    LOG(INFO) << "TCP Client Connecting " << host << ":" << port;
    while(1) {
        try {
            socket.connect(boost::asio::ip::tcp::endpoint(
                boost::asio::ip::address::from_string(host), port));
                break;
        } catch(boost::system::system_error const& e) {
            if (e.code() == errc::connection_refused) {
                // LOG(INFO) << "TCP client waiting for host";
                sleep(1);
            } else {
                LOG(INFO) << "Connection failed: " << e.what();
                break;
            }
        }
    }
}

TcpClient::~TcpClient() 
{
    LOG(INFO) << "Closing connection ...";
    socket.close();
}

void TcpClient::send(const uint8_t *buf, const int buf_size) {
    send_to_socket(buf, buf_size);
}

void TcpClient::close() 
{
    socket.close();
}

void TcpClient::send_to_socket(const uint8_t *buf, const int buf_size)
{
    uint32_t data_sent_in_bytes = 0;
    try
    {
        const uint32_t rc = boost::asio::write(socket, boost::asio::buffer(buf, buf_size));
        data_sent_in_bytes = rc;
    }
    catch (boost::system::system_error e)
    {
        LOG(ERROR) << boost::diagnostic_information(e);
    }
    return;
}

/**
 * @brief Construct a new WebRTC Server between Multiview Server and python sender
 * 
 * @param view_number 0 if tiled, number of views if not tiled.
 * @param type Either of types: ["c_bgra", "d_bgra", "d_yuv16]. "c_bgra" is color image in BGRA format, "d_bgra" is colorized depth image in BGRA format, "d_yuv16" is depth image in YUV16 format.
 * @param view_count Number of views.
 * @param isTile true, if views are tiled (used now), false (earlier version, will be deprecated)
 * @param isSaveView Save view to local for debugging.
 */
WebRTCServer::WebRTCServer(int view_number, string type, int view_count, bool isTile, bool isSaveView): 
        view_number(view_number), 
        _type(type), 
        view_count(view_count), 
        isTile(isTile), isSaveView(isSaveView),
        curr_counter(0)
{
    string sshm_name_with_view = "test_shm" + _type + to_string(view_number);
    const char* shm_name_with_view = sshm_name_with_view.c_str();
    LOG(INFO) << "WebRTC Server shm name " << shm_name_with_view << " type "<< _type;
    destroy_shm(shm_name_with_view);

    LOG(INFO) << "WebRTC Server Init " << view_number << " " << type << " " << view_count;

    //Create a shared memory object
    if(_type != "d_yuv16")
        shm_size = calibrations_init[0]->intrinsics.height * calibrations_init[0]->intrinsics.width * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA;
    else
        shm_size = calibrations_init[0]->intrinsics.height * calibrations_init[0]->intrinsics.width * CHANNEL_DIM::DEPTH * PIXEL_SIZE::DEPTH;
    // shm_name_with_view = sshm_name_with_view.c_str();
    if(!isTile)
        create_shm(shm_name_with_view, shm_size, region);
    else
        create_shm(shm_name_with_view, shm_size * view_count * SHM_BUFFER_SIZE, region);
    
    LOG(INFO) << "Shared memory created " << view_number << " " << type << " " << view_count;
    shm_ptr = static_cast<uint8_t *>(region.get_address());

    // Create a socket and connect to the server.
    boost::asio::io_service svc;
    int _port = PORT+view_number;
    if (_type == "d" || _type == "d_yuv16") _port += 10;
    client = new TcpClient(svc, WEBRTC_SENDER_ADDRESS, _port);

    if (!isTile) 
    {
        builder = new flatbuffers::FlatBufferBuilder(TCP_BUFFER_SIZE);

        // initially use flatbuffer to send frame shape
        auto frame_info = CreateFrameInfo(*builder, calibrations_init[0]->intrinsics.width, calibrations_init[0]->intrinsics.height);
        builder->Finish(frame_info);
        
        uint8_t *frame_info_buf = builder->GetBufferPointer();
        int frame_info_size = builder->GetSize(); // Returns the size of the buffer.

        client->send(frame_info_buf, frame_info_size); // Send metadata to notify client to read frame.
        builder->Clear();    // Clear builder to reuse.
    } 
    else 
    {
        builder = new flatbuffers::FlatBufferBuilder(TCP_BUFFER_SIZE);

        // initially use flatbuffer to send frame shape
        auto frame_info = CreateFrameInfo(*builder, calibrations_init[0]->intrinsics.width * 5, calibrations_init[0]->intrinsics.height * 2);
        builder->Finish(frame_info);
        
        uint8_t *frame_info_buf = builder->GetBufferPointer();
        int frame_info_size = builder->GetSize(); // Returns the size of the buffer.

        client->send(frame_info_buf, frame_info_size); // Send metadata to notify client to read frame.
        builder->Clear();    // Clear builder to reuse.
    }

    assert(calibrations_init.size() == view_count);

    color_buf = new uint8_t*[view_count];
    depth_buf = new uint8_t*[view_count];
    colorized_depth = new RGB*[view_count];
    depth_yuv16_buf = new uint8_t*[view_count];

    for (int i=0; i<view_count; i++) 
    {
        color_buf[i] = new uint8_t[shm_size];           // This should be used for _type == "c_bgra" || _type == "c"
        depth_buf[i] = new uint8_t[shm_size];           // This should be used for _type == "d_bgra" || _type == "d"
        colorized_depth[i] = new RGB[shm_size];         // This should be used for _type == "d_bgra" || _type == "d"
        depth_yuv16_buf[i] = new uint8_t[shm_size];     // This should be used for _type == "d_yuv16"
    }

    for(int i=0; i<CHANNEL_DIM::DEPTH; i++)
        depth_yuv16_cv[i] = cv::Mat(calibrations_init[0]->intrinsics.height, calibrations_init[0]->intrinsics.width, CV_16UC1);

    // TODO: Rajrup: Replace 10 with view_count?
    mask_send = new bool[calibrations_init[0]->intrinsics.width * calibrations_init[0]->intrinsics.height * 10];

    // Set color and depth bitrate
    bwe.store(FLAGS_cb + FLAGS_db);
    cbwe.store(FLAGS_cb);
    dbwe.store(FLAGS_db);
}

// TODO: remove this and use tiled versions
void WebRTCServer::send_color(RGB *color_image, int frame_number) 
{ 
    // --------------------------------SHARE DATA--------------------------------
    // Add metadata to flatbuffer
    auto frame_info = CreateFrameInfo(*builder, frame_number, view_number);
    builder->Finish(frame_info);

    uint8_t *frame_info_buf = builder->GetBufferPointer();
    int frame_info_size = builder->GetSize(); // Returns the size of the buffer that `GetBufferPointer()` points to.

    // color_image in RGB* to uint8_t* 
    uint8_t *color_buf = reinterpret_cast<uint8_t *>(color_image);   
    // // DEBUG: store image to local
    // save_color(color_buf, 512, 910, FORMAT("/home/lei/data/pipeline/server/" << frame_number << "_color_" << view_number << ".png"));
    //Copy data to shared memory
    memcpy(shm_ptr, color_buf, shm_size);

    client->send(frame_info_buf, frame_info_size); // Send metadata to notify client to read frame.
    builder->Clear();    // Clear builder to reuse.
}

// TODO: remove this and use tiled versions
void WebRTCServer::send_depth(uint16_t *depth_image, int width, int height, int frame_number) 
{
    auto frame_info = CreateFrameInfo(*builder, frame_number, view_number);
    builder->Finish(frame_info);
    uint8_t *frame_info_buf = builder->GetBufferPointer();
    int frame_info_size = builder->GetSize(); // Returns the size of the buffer that `GetBufferPointer()` points to.

    // depth_image in uint16_t* to uint8_t*
    RGB *colorized_depth = new RGB[width * height * (int)sizeof(RGB)];
    depth_to_color(depth_image, height, width, colorized_depth, false);
    uint8_t *depth_buf = reinterpret_cast<uint8_t *>(colorized_depth);
    // // DEBUG: store image to local. Here we save colorized depth image
    // save_color(depth_buf, height, width, FORMAT("/home/lei/data/pipeline/server/" << frame_number << "_depth_" << view_number << ".png"));    
    memcpy(shm_ptr, depth_buf, shm_size);

    client->send(frame_info_buf, frame_info_size); // Send metadata to notify client to read frame.
    builder->Clear();    // Clear builder to reuse.
}

void WebRTCServer::add_qr_bgra(RGB *buf, int width, int height, const string &qr_folder, int frameID, bool addText)
{
	// Adding QR code to 1st view
	cv::Mat qr_code = cv::imread(FORMAT(qr_folder << frameID << "_qrcode.png"), cv::IMREAD_GRAYSCALE);
    if(qr_code.empty())
    {
        LOG(FATAL) << "QR code image not found!";
        return;
    }
	cv::Mat img_view0 = color_to_opencv(buf, width, height);

	int offset_x, offset_y;
	offset_x = 20;
	offset_y = 20;
	for(int i = 0; i < qr_code.rows; i++)
	{
		for(int j = 0; j < qr_code.cols; j++)
		{
			assert(i+offset_y < img_view0.rows && j+offset_x < img_view0.cols);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[0] = qr_code.at<uint8_t>(i, j);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[1] = qr_code.at<uint8_t>(i, j);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[2] = qr_code.at<uint8_t>(i, j);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[3] = 255;
		}
	}

    if(addText)
    {
        // Adding text to 1st view
        int text_offset_x, text_offset_y;
        text_offset_x = offset_x + qr_code.cols + 200;
        text_offset_y = offset_y + qr_code.rows;
        cv::putText(img_view0, FORMAT(frameID), cv::Point(text_offset_x, text_offset_y), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 255, 0, 255), 8);
    }

	memcpy(buf, &img_view0.ptr<cv::Vec4b>(0)[0], img_view0.rows * img_view0.cols * sizeof(cv::Vec4b));

    // cv::Mat img_view0 = color_to_opencv(buf, width, height);
    // cv::Mat qrcode_rgb;
    // cv::cvtColor(qr_code, qrcode_rgb, cv::COLOR_GRAY2BGRA);

    // int offset_x, offset_y;
	// offset_x = 20;
	// offset_y = 20;
    // qr_code.copyTo(img_view0(cv::Range(offset_x, offset_x+qr_code.rows), cv::Range(offset_y, offset_y+qr_code.cols)));
    // memcpy(view.color_image, &img_view0.ptr<cv::Vec4b>(0)[0], img_view0.rows * img_view0.cols * sizeof(cv::Vec4b));

	// For debugging, remove it later
	// if(frameID % 40 == 0)
	// {
	// 	img_view0 = color_to_opencv(view.color_image, view.colorWidth, view.colorHeight);
	// 	cv::imwrite(FORMAT("/home/lei/data/pipeline/server_tiled/" << frameID << "_color_withqr_0.png"), img_view0);

	// 	cv::Mat qrcode_extract = img_view0(cv::Range(offset_x, offset_x+82), cv::Range(offset_y, offset_y+82));

	// 	vector<cv::Mat> channels(img_view0.channels());
	// 	cv::split(qrcode_extract, channels);

	// 	for(int i=0; i<channels.size(); i++)
	// 	{
	// 		string channel_path = FORMAT("/home/lei/data/pipeline/server_tiled/" << frameID << "_qrextract" << i << ".png");
	// 		if(!cv::imwrite(channel_path, channels[i]))
	// 			LOG(ERROR) << "Failed to write color image: " << channel_path;
	// 		else 
	// 			LOG(INFO) << "Color image saved to " << channel_path;
	// 	}
	// }
}

/**
 * @brief Tiled color assumes that format is in BGRA
 * 
 * @param views 
 * @param frame_number 
 */
// Note: Rajrup: frame_number != view->frameID, check wserver->send(m_capture_id, s_views) in MultiviewServerPool.cpp. m_capture_id is already incremented.
// Remove frame_number from here and use view->frameID
void WebRTCServer::send_tiled_color(vector<View *> views, int frame_number) 
{
    // reusing view_number slot for bwe
    int bwe_now = bwe.load();
    int cbwe_now = cbwe.load();
    int dbwe_now = dbwe.load();
    // auto frame_info = CreateFrameInfo(*builder, frame_number, bwe_now);
    auto frame_info = CreateFrameInfo(*builder, frame_number, cbwe_now);
    builder->Finish(frame_info);
    uint8_t *frame_info_buf = builder->GetBufferPointer();
    int frame_info_size = builder->GetSize(); // Returns the size of the buffer that `GetBufferPointer()` points to.
    LOG(INFO) << "Frame: " << frame_number << ", BWE: " << bwe_now << ", CBWE: " << cbwe_now << ", DBWE: " << dbwe_now;

    // LOG(INFO) << "Sending tiled color - frame_number: "<< frame_number << ", view_number: " << views[0]->frameID;
    string outdir = "/home/lei/data/pipeline/server_tiled/pipeline_new/test/";

    #pragma omp parallel for
    for (int i=0; i<view_count; i++) 
    {
        auto view = views[i];
        if(i==0)
        {
            add_qr_bgra(view->color_image, view->colorWidth, view->colorHeight, QRCODE_FOLDER, view->frameID, false);
        }
        // else
        // {
        //     // DEBUG    
        //     add_qr_bgra(view->color_image, view->colorWidth, view->colorHeight, QRCODE_FOLDER, view->frameID, true);
        // }

        if(view->frameID != frame_number)
            LOG(ERROR) << "Frame number mismatch: " << view->frameID << " " << frame_number;

        color_buf[i] = reinterpret_cast<uint8_t *>(view->color_image);

        // //  Tile with 10*1 format (I think this is incorrect)
        // memcpy(shm_ptr + 10 * view->depthHeight * view->depthWidth * CHANNEL_DIM::BGRA * (curr_counter % SHM_BUFFER_SIZE) + view->viewID * view->depthHeight * view->depthWidth * CHANNEL_DIM::BGRA, color_buf[i], view->depthHeight * view->depthWidth * CHANNEL_DIM::BGRA);

        // Tile with 2*5 format
        auto h = view->colorHeight;
        auto w = view->colorWidth;
        auto id = view->viewID;

        // TODO: Rajrup: Replace 10 with view_count?
        /**
         * @brief The memory layout of shm_ptr is as follows:
         * 1st row of 5*2 layout is placed one after another in memory ||BRGA|BRGA| ---- w * 5 ----- |BRGA|BRGA||. Then comes the second row and so on, till 10 rows are filled.
         * This is done since the gstreamer memory layout expects the data to be in this format. 
         * Find BGRA layout here: https://gstreamer.freedesktop.org/documentation/additional/design/mediatype-video-raw.html?gi-language=c 
         */
        int x = id / 5 * h, y = id % 5 * (w * CHANNEL_DIM::BGRA);
        for (int j=0; j<h; j++)
            memcpy(shm_ptr + (10 * h * w * CHANNEL_DIM::BGRA * (curr_counter % SHM_BUFFER_SIZE)) + ((x + j) * w * 5 * CHANNEL_DIM::BGRA) + y, color_buf[i] + (j * w * CHANNEL_DIM::BGRA), w * CHANNEL_DIM::BGRA);

        if (isSaveView && (frame_number % 10 == 0))
        {
            if(!fs::exists(outdir))
                fs::create_directories(outdir);
            save_color(color_buf[i], h, w, FORMAT(outdir << frame_number << "_color_bgra_" << view->viewID << ".png"));
        }
    }
    // LOG(INFO) << "Sending tiled color: " << frame_number;
    if (isSaveView && (frame_number % 10 == 0))
    {
        auto h = views[0]->colorHeight;
        auto w = views[0]->colorWidth;
        if(!fs::exists(outdir))
            fs::create_directories(outdir);
        save_color(shm_ptr + (10 * h * w * CHANNEL_DIM::BGRA * (curr_counter % SHM_BUFFER_SIZE)), h * 2, w * 5, FORMAT(outdir << frame_number << "_color_bgra.png"));
    }

    // if (isSaveView && frame_number % 10 == 0)
    //     save_color(shm_ptr, views[0]->depthHeight * 2, views[0]->depthWidth * 5, FORMAT("/home/lei/data/pipeline/server_tiled/" << frame_number << "_color.png"));

    client->send(frame_info_buf, frame_info_size); // Send metadata to notify client to read frame.
    builder->Clear();    // Clear builder to reuse.
    curr_counter++;
}

/**
 * @brief Tiled depth assumes that format is in BGRA (colorized depth)
 * 
 * @param views 
 * @param frame_number 
 */
// Note: Rajrup: frame_number != view->frameID, check wserver->send(m_capture_id, s_views) in MultiviewServerPool.cpp. m_capture_id is already incremented.
void WebRTCServer::send_tiled_depth(vector<View *> views, int frame_number) 
{
    // reusing view_number slot for bwe
    int bwe_now = bwe.load();
    int dbwe_now = dbwe.load();
    // auto frame_info = CreateFrameInfo(*builder, frame_number, bwe_now);
    auto frame_info = CreateFrameInfo(*builder, frame_number, dbwe_now);
    builder->Finish(frame_info);
    uint8_t *frame_info_buf = builder->GetBufferPointer();
    int frame_info_size = builder->GetSize(); // Returns the size of the buffer that `GetBufferPointer()` points to.
    // LOG(INFO) << "Frame: " << frame_number << ", BWE: " << bwe_now;
    
    // LOG(INFO) << "Sending tiled depth - frame_number: "<< frame_number << ", view_number: " << views[0]->frameID;
    string outdir = "/home/lei/data/pipeline/server_tiled/pipeline_new/test/";

    #pragma omp parallel for
    for (int i=0; i<view_count; i++) 
    {
        auto view = views[i];
        depth_to_color(view->depth_image, view->depthHeight, view->depthWidth, colorized_depth[i], false);

        if(i==0)   
        {
            add_qr_bgra(colorized_depth[i], view->depthWidth, view->depthHeight, QRCODE_FOLDER, view->frameID);
        }

        if(view->frameID != frame_number)
            LOG(ERROR) << "Frame number mismatch: " << view->frameID << " " << frame_number;

        depth_buf[i] = reinterpret_cast<uint8_t *>(colorized_depth[i]);

        // Tile with 2*5 format
        auto h = view->depthHeight;
        auto w = view->depthWidth;
        auto id = view->viewID;
        int x = id / 5 * h, y = id % 5 * (w * CHANNEL_DIM::BGRA);
        for (int j=0; j<h; j++) 
            memcpy(shm_ptr + (10 * h * w * CHANNEL_DIM::BGRA * (curr_counter % SHM_BUFFER_SIZE)) + ((x + j) * w * 5 * CHANNEL_DIM::BGRA + y), depth_buf[i] + (j * w * CHANNEL_DIM::BGRA), w * CHANNEL_DIM::BGRA);

        if (isSaveView && (frame_number % 10 == 0))
        {
            if(!fs::exists(outdir))
                fs::create_directories(outdir);
            save_depth(depth_buf[i], h, w, FORMAT(outdir << frame_number << "_depth_bgra_" << view->viewID << ".png"), true);
        }
    }

    // LOG(INFO) << "Sending tiled depth: " << frame_number;
    if (isSaveView && (frame_number % 10 == 0))
    {
        auto h = views[0]->depthHeight;
        auto w = views[0]->depthWidth;
        if(!fs::exists(outdir))
            fs::create_directories(outdir);
        save_depth(shm_ptr + (10 * h * w * CHANNEL_DIM::BGRA * (curr_counter % SHM_BUFFER_SIZE)), h * 2, w * 5, FORMAT(outdir << frame_number << "_depth_bgra.png"), true);
    }

    // if (frame_number % 10 == 0)
    //     save_depth(shm_ptr, views[0]->depthHeight * 2, views[0]->depthWidth * 5, FORMAT("/home/lei/data/pipeline/server_tiled/" << frame_number << "_depth.png"));

    client->send(frame_info_buf, frame_info_size); // Send metadata to notify client to read frame.
    builder->Clear();    // Clear builder to reuse.
    curr_counter++;
}

/**
 * @brief Converts 16-bit depth image to 16-bit YUV16 image.
 * Y channel has the depth image. U and V channels are set to constant.
 * Depth image is scaled to 
 * @param depth 
 * @param height 
 * @param width 
 * @param depth_yuv16_cv 3 channel 16-bit image in cv::Mat format CV_16UC3
 * @param isScaled Kinect depth range is 5 meters. We scale the depth between 0 - 6000 (max) to 0 - 65535 (max).
 */
void WebRTCServer::depth_to_yuv16(uint16_t *depth, cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], int height, int width, bool isScaled)
{
    for(int i=0; i<height; i++)
    {
        for(int j=0; j<width; j++)
        {
            if(isScaled)
            {
                float d = (depth[i * width + j] * 65535.0F) / 6000.0F;
                depth_yuv16_cv[0].at<uint16_t>(i, j) = uint16_t(d);
            }
            else
                depth_yuv16_cv[0].at<uint16_t>(i, j) = depth[i * width + j];
            
            depth_yuv16_cv[1].at<uint16_t>(i, j) = 32768;
            depth_yuv16_cv[2].at<uint16_t>(i, j) = 32768;
        }
    }
}

/**
 * @brief Adds QR code to YUV16 depth image in UV channel.
 * It assumes UV channel have constant values.
 * @param depth_yuv16_cv 
 * @param qr_folder 
 * @param frameID 
 * @param isScaled 
 * @param addText 
 */
void WebRTCServer::add_qr_yuv16(cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], const string &qr_folder, int frameID, bool isScaled, bool addText)
{
    // Adding QR code to 1st view
	cv::Mat qr_code = cv::imread(FORMAT(qr_folder << frameID << "_qrcode.png"), cv::IMREAD_GRAYSCALE);
    if(qr_code.empty())
    {
        LOG(FATAL) << "QR code image not found!";
        return;
    }

    int offset_x, offset_y;
    offset_x = 20;
    offset_y = 20;
    for(int i=0; i<qr_code.rows; i++)
    {
        for(int j=0; j<qr_code.cols; j++)
        {
            assert(i+offset_y < depth_yuv16_cv.rows && j+offset_x < depth_yuv16_cv.cols);
            if(isScaled)
            {
                float pixel = (qr_code.at<uint8_t>(i, j) * 65535.0F) / 255.0F ;
                depth_yuv16_cv[1].at<uint16_t>(i + offset_y, j + offset_x) = uint16_t(pixel);
                depth_yuv16_cv[2].at<uint16_t>(i + offset_y, j + offset_x) = uint16_t(pixel);
            }
            else
            {
                depth_yuv16_cv[1].at<uint16_t>(i + offset_y, j + offset_x) = qr_code.at<uint8_t>(i, j);
                depth_yuv16_cv[2].at<uint16_t>(i + offset_y, j + offset_x) = qr_code.at<uint8_t>(i, j);
            }
        }
    }

    if(addText)
    {
        // Adding text to 1st view
        int text_offset_x, text_offset_y;
        text_offset_x = offset_x + qr_code.cols + 200;
        text_offset_y = offset_y + qr_code.rows;
        cv::Mat temp;
        cv::merge(vector<cv::Mat>{depth_yuv16_cv[0], depth_yuv16_cv[1], depth_yuv16_cv[2]}, temp);
        cv::putText(temp, FORMAT(frameID), cv::Point(text_offset_x, text_offset_y), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 255, 0), 8);
        cv::split(temp, vector<cv::Mat>{depth_yuv16_cv[0], depth_yuv16_cv[1], depth_yuv16_cv[2]});
    }
}

/**
 * @brief Flatten 16-bit YUV16 depth image to 8-bit buffer.
 * The flatten needs to follow gstreamer memory format for Y444_16LE.
 * Find Y444 layout here: https://gstreamer.freedesktop.org/documentation/additional/design/mediatype-video-raw.html?gi-language=c 
 * @param depth_yuv16_cv 
 * @param depth_yuv16_buf 
 */
void WebRTCServer::flatten_depth_yuv16(const cv::Mat (&depth_yuv16_cv)[3], uint8_t *depth_yuv16_buf, int height, int width)
{
    for(int i=0; i<CHANNEL_DIM::DEPTH; i++)
    {
        memcpy(depth_yuv16_buf + (i * height * width * PIXEL_SIZE::DEPTH), depth_yuv16_cv[i].data, height * width * PIXEL_SIZE::DEPTH);
    }
}

void WebRTCServer::send_tiled_depth_yuv16(vector<View *> views, int frame_number) 
{
    // reusing view_number slot for bwe
    int bwe_now = bwe.load();
    int dbwe_now = dbwe.load();
    // auto frame_info = CreateFrameInfo(*builder, frame_number, bwe_now);
    auto frame_info = CreateFrameInfo(*builder, frame_number, dbwe_now);
    builder->Finish(frame_info);
    uint8_t *frame_info_buf = builder->GetBufferPointer();
    int frame_info_size = builder->GetSize();
    // LOG(INFO) << "Frame: " << frame_number << ", BWE: " << bwe_now;

    string outdir = "/home/lei/data/pipeline/server_tiled/pipeline_new/test/";

    // #pragma omp parallel for
    for (int i=0; i<view_count; i++) 
    {
        auto view = views[i];
        auto h = view->depthHeight;
        auto w = view->depthWidth;
        auto id = view->viewID;

        depth_to_yuv16(view->depth_image, depth_yuv16_cv, h, w); // Scaled depth image in YUV16

        if(i==0)
        {
            add_qr_yuv16(depth_yuv16_cv, QRCODE_FOLDER, view->frameID); // Add scaled QR code in UV channel
        }

        if(view->frameID != frame_number)
            LOG(ERROR) << "Frame number mismatch: " << view->frameID << " " << frame_number;

        flatten_depth_yuv16(depth_yuv16_cv, depth_yuv16_buf[i], h, w); // Flatten YUV16 image to buffer

        // //  Tile with 10*1 format (I think this is incorrect)
        // memcpy(shm_ptr + 10 * view->depthHeight * view->depthWidth * 4 * (curr_counter % SHM_BUFFER_SIZE) + view->viewID * view->depthHeight * view->depthWidth * 4, depth_buf[i], view->depthHeight * view->depthWidth * 4);

        // Tile with 2*5 format
        // int x = id / 5 * h, y = id % 5 * (w * CHANNEL_DIM::BGRA);
        // for (int j=0; j<h; j++) 
        //     memcpy(shm_ptr + (10 * h * w * CHANNEL_DIM::BGRA * (curr_counter % SHM_BUFFER_SIZE)) + ((x + j) * w * 5 * CHANNEL_DIM::BGRA + y), depth_buf[i] + (j * w * CHANNEL_DIM::BGRA), w * CHANNEL_DIM::BGRA);

        int x = id / 5 * h, y = id % 5 * (w * PIXEL_SIZE::DEPTH);
        for(int k=0; k<CHANNEL_DIM::DEPTH; k++)
        {
            uint8_t *depth_channel_buf = depth_yuv16_cv[k].data;
            uint8_t *shm_buf = shm_ptr + 
                                (10 * h * w * CHANNEL_DIM::DEPTH * PIXEL_SIZE::DEPTH * (curr_counter % SHM_BUFFER_SIZE)) + 
                                (k * 10 * h * w * PIXEL_SIZE::DEPTH);
            for(int j=0; j<h; j++)
            {
                memcpy(shm_buf + ((x + j) * 5 * w * PIXEL_SIZE::DEPTH + y), depth_channel_buf + (j * w * PIXEL_SIZE::DEPTH), w * PIXEL_SIZE::DEPTH);
            }
        }

        // DEBUG: store separate image to local
        // if (frame_number % 5 == 0)
        //     save_depth(depth_buf[i],  views[0]->depthHeight, views[0]->depthWidth, FORMAT("/home/lei/data/pipeline/server_standalone/" << frame_number << "_depth_" << view->viewID  << ".png"));

        if(isSaveView && (frame_number % 10 == 0))
        {
            if(!fs::exists(outdir))
                fs::create_directories(outdir);
            
            uint8_t *depth_channel_buf = depth_yuv16_cv[0].data;
            cv::Mat depth_image_cv2(h, w, CV_16UC1, depth_channel_buf);
            string path = FORMAT(outdir << frame_number << "_depth_yuv16_" << view->viewID << ".png"); 
            if(!cv::imwrite(path, depth_image_cv2))
                LOG(ERROR) << "Failed to write depth image: " << path;
            else 
                LOG(INFO) << "Depth image saved to " << path;

            depth_channel_buf = depth_yuv16_cv[1].data;
            depth_image_cv2 = cv::Mat(h, w, CV_16UC1, depth_channel_buf);
            path = FORMAT(outdir << frame_number << "_depth_yuv16_" << view->viewID << "_qr_u.png");
            if(!cv::imwrite(path, depth_image_cv2))
                LOG(ERROR) << "Failed to write depth image: " << path;

        }
    }

    if(isSaveView && (frame_number % 10 == 0))
    {
        auto h = views[0]->depthHeight;
        auto w = views[0]->depthWidth;
        if(!fs::exists(outdir))
            fs::create_directories(outdir);
        string path = FORMAT(outdir << frame_number << "_depth_yuv16.png");
        cv::Mat depth_image_cv2 = cv::Mat(h * 2, w * 5, CV_16UC1, shm_ptr + (10 * h * w * CHANNEL_DIM::DEPTH * PIXEL_SIZE::DEPTH * (curr_counter % SHM_BUFFER_SIZE)));
        if(!cv::imwrite(path, depth_image_cv2))
            LOG(ERROR) << "Failed to write depth image: " << path;
        else
            LOG(INFO) << "Depth image saved to " << path;
    }

    client->send(frame_info_buf, frame_info_size); // Send metadata to notify client to read frame.
    builder->Clear();    // Clear builder to reuse.
    curr_counter++;
}

void WebRTCServer::send_tiled_mask(vector<View *> views, int frame_number) {
    while(bool x = new_mask.load(memory_order_relaxed) == true) {
        sleep(0.001);
    }

// #pragma omp parallel for

    for (int i=0; i<view_count; i++) {
        auto view = views[i];
        int binary_mask_size = view->colorWidth * view->colorHeight * sizeof(bool);
        memcpy(mask_send + view->viewID * binary_mask_size , view->binary_mask, binary_mask_size);
    }
    new_mask.store(true);
}

WebRTCServerPool::WebRTCServerPool(int _view_count, bool _isTile, bool isSaveView) : 
    view_count(_view_count),
    isTile(_isTile), isSaveView(isSaveView) 
{
    // For tile mode, only create a server for either color or depth
    if (isTile) 
    {
        auto cserver = new WebRTCServer(0, "c", view_count, true, isSaveView);
        cserver_pool.push_back(cserver);
        
        // // Rajrup: DEPTH_BGRA
        // auto dserver = new WebRTCServer(0, "d", view_count, true, isSaveView);
        // dserver_pool.push_back(dserver);

        // Rajrup: DEPTH_YUV16
        auto dserver = new WebRTCServer(0, "d_yuv16", view_count, true, isSaveView);
        dserver_pool.push_back(dserver);
    } 
    else 
    {
        for (int i = 0; i < view_count; i++) 
        {
            auto cserver = new WebRTCServer(i, "c", view_count, false, isSaveView);
            cserver_pool.push_back(cserver);
            
            auto dserver = new WebRTCServer(i, "d", view_count, false, isSaveView);
            dserver_pool.push_back(dserver);
        }
    }
}

void WebRTCServerPool::send_color_thread(int viewid, RGB *color_image, int capture_id) {
    cserver_pool[viewid]->send_color(color_image, capture_id);
}

void WebRTCServerPool::send_depth_thread(int viewid, uint16_t *depth_image, int width, int height, int capture_id) {
    dserver_pool[viewid]->send_depth(depth_image, width, height, capture_id);
}

void WebRTCServerPool::send_color_tiled(vector<View *> views, int capture_id) {
    cserver_pool[0]->send_tiled_color(views, capture_id);
}

void WebRTCServerPool::send_depth_tiled(vector<View *> views, int capture_id) {
    // // Rajrup: DEPTH_BGRA
    // dserver_pool[0]->send_tiled_depth(views, capture_id);
    
    // Rajrup: DEPTH_YUV16
    dserver_pool[0]->send_tiled_depth_yuv16(views, capture_id);
}

void WebRTCServerPool::send_mask_tiled(vector<View *> views, int capture_id) {
    cserver_pool[0]->send_tiled_mask(views, capture_id);
}

void WebRTCServerPool::send_ptcl(const uint8_t *compressed_ptcl, const int size, int capture_id)
{
    while(bool x = new_ptcl.load(memory_order_relaxed) == true)
        sleep_ms(1);

    // Add Frame ID to the beginning of the buffer
    int frame_id = capture_id;
    memcpy(ptcl_buf, &frame_id, sizeof(int));

    // Copy compressed point cloud to buffer
    memcpy(ptcl_buf + sizeof(int), compressed_ptcl, size);
    ptcl_buf_size = sizeof(int) + size;

    new_ptcl.store(true);
}

// Stores latest received images. TODO: remove this
bool WebRTCServerPool::store_view(vector<View *> s_views)
{
	for(auto view : s_views)
	{
        cv::Mat color_image_cv2(view->colorHeight, view->colorWidth, CV_8UC4, view->color_image);
        cv::Mat depth_image_cv2(view->depthHeight, view->depthWidth, CV_16U, view->depth_image);

		string color_image_path = FORMAT("/home/lei/test/output/server/" << view->frameID << "_color_" << view->viewID << ".png");
		string depth_image_path = FORMAT("/home/lei/test/output/server/" << view->frameID << "_depth_" << view->viewID << ".png");
		
		if(!cv::imwrite(color_image_path, color_image_cv2, compression_params))
		{
			LOG(ERROR) << "Failed to write color image: " << color_image_path;
			return false;
		}
		if(!cv::imwrite(depth_image_path, depth_image_cv2, compression_params))
		{
			LOG(ERROR) << "Failed to write depth image: " << depth_image_path;
			return false;
		}
	}
	return true;
}

void WebRTCServerPool::send(int capture_id, vector<View *> s_views) {
    if (!isTile) 
    {
        // Send frames in separate channels
    
        for (auto view : s_views) {
            send_pool.push_back(thread(&WebRTCServerPool::send_color_thread, this, view->viewID, view->color_image, capture_id)); 
            send_pool.push_back(thread(&WebRTCServerPool::send_depth_thread, this, view->viewID, view->depth_image, view->depthWidth, view->depthHeight, capture_id));
            // send_pool.push_back(thread(&WebRTCServerPool::send_binary_mask_thread, this, view->viewID, view->binary_mask, view->depthWidth, view->depthHeight, capture_id));
        }

        for (int i=0; i<2*view_count; i++) {
            send_pool[i].join();
        }


    } else {
        // Send tiled frames.
        send_pool.push_back(thread(&WebRTCServerPool::send_color_tiled, this, s_views, capture_id)); 
        send_pool.push_back(thread(&WebRTCServerPool::send_depth_tiled, this, s_views, capture_id));
        // send_pool.push_back(thread(&WebRTCServerPool::send_mask_tiled, this, s_views, capture_id));
        for (int i=0; i<send_pool.size(); i++) {
            send_pool[i].join();
        }
    }

    send_pool.clear();
}

void WebRTCServerPool::send(int capture_id, const uint8_t *compressed_ptcl, const int size)
{

    if(size > PTCL_MAX_POINTS * (3 * sizeof(float) + 3 * sizeof(uint8_t)))
        LOG(FATAL) << "Point cloud size exceeds maximum allowed size of " << PTCL_MAX_POINTS << " poins";

    send_ptcl(compressed_ptcl, size, capture_id);
}

WebSocketServer::WebSocketServer(const char* addr, int port, int isSendPtcl): _addr(addr), _port(port), _isSendPtcl(isSendPtcl)
{
    if(_isSendPtcl)
    {
        // Memory for compressed point cloud buffer (used for sending point cloud); Assume PTCL_MAX_POINTS million points.
        // Format: Frame ID (uint32_t), compressed point cloud (uint8_t buffer)
        ptcl_buf = new uint8_t[sizeof(uint32_t) + PTCL_MAX_POINTS * (3 * sizeof(float) + 3 * sizeof(uint8_t))];
        ptcl_buf_size = 0;
    }
}

void WebSocketServer::run()
{
    try
    {
        auto const address = net::ip::make_address(_addr);
        // port1: sending calibrations and receiving frustums
        auto const port1 = static_cast<unsigned short>(_port);
        // port2: sending masks
        auto const port2 = static_cast<unsigned short>(_port+1);
        // port3: receiving client side mahimahi bandwidth
        auto const port3 = static_cast<unsigned short>(_port+2);
        // port4: sending ptcl
        auto const port4 = static_cast<unsigned short>(_port+3);

        LOG(INFO) << "WebSocket server listening on " << address << ": " << port1 << ", Purpose: " << "Send calibrations and receive frustums";
        LOG(INFO) << "WebSocket server listening on " << address << ": " << port2 << ", Purpose: " << "Send masks";
        LOG(INFO) << "WebSocket server listening on " << address << ": " << port3 << ", Purpose: " << "Receive client side mahimahi bandwidth";
        if(_isSendPtcl)
            LOG(INFO) << "WebSocket server listening on " << address << ": " << port4 << ", Purpose: " << "Send ptcl";
        

        // The io_context is required for all I/O
        net::io_context ioc1{1};
        net::io_context ioc2{1};
        net::io_context ioc3{1};
        net::io_context ioc4{1};

        // The acceptor receives incoming connections
        tcp::acceptor acceptor1{ioc1, {address, port1}};
        tcp::acceptor acceptor2{ioc2, {address, port2}};
        tcp::acceptor acceptor3{ioc3, {address, port3}};
        tcp::acceptor acceptor4{ioc4, {address, port4}};

        for(;;)
        {
            // This will receive the new connection
            tcp::socket socket1{ioc1};
            tcp::socket socket2{ioc2};
            tcp::socket socket3{ioc3};
            tcp::socket socket4{ioc4};

            // Block until we get a connection
            acceptor1.accept(socket1);
            acceptor2.accept(socket2);
            acceptor3.accept(socket3);
            if(_isSendPtcl)
                acceptor4.accept(socket4);

            // Launch the session, transferring ownership of the socket
            std::thread(
                &WebSocketServer::do_session,
                this,
                std::move(socket1)).detach();
            
            std::thread(
                &WebSocketServer::do_session2, 
                this,
                std::move(socket2)).detach();

            std::thread(
                &WebSocketServer::do_session3, 
                this,
                std::move(socket3)).detach();
            
            if(_isSendPtcl)
                std::thread(
                    &WebSocketServer::do_session_ptcl, 
                    this,
                    std::move(socket4)).detach();
        }
    }
    catch (const std::exception& e)
    {
        LOG(ERROR) << "Error: " << e.what();
        return;
    }
}

/**
 * @brief Sends calibrations after connection setup over websocket.
 * Receives frustums from client over websocket, once client starts viewing point cloud.
 * @param socket 
 */
void WebSocketServer::do_session(tcp::socket socket)
{
    LOG(INFO) << "Websocket server received connection request";
    LOG(INFO) << "This socket will send calibrations and receive frustums";
    try
    {
        // Construct the stream by moving in the socket
        ws1 = new websocket::stream<tcp::socket> {std::move(socket)};

        // Set a decorator to change the Server of the handshake
        ws1->set_option(websocket::stream_base::decorator(
        [](websocket::response_type& res)
            {
                res.set(http::field::server,
                    std::string(BOOST_BEAST_VERSION_STRING) +
                        " websocket-server-sync");
            }));

        // Accept the websocket handshake
        ws1->accept();

        LOG(INFO) << "This thread will send calibrations and receive frustums over socket: " << ws1->next_layer().local_endpoint().address().to_string() << ":" << ws1->next_layer().local_endpoint().port();

        CamInt camInt;
        Vector3f pos;
        Vector3f rot;
        Vector4f quat;

        for(;;)
        {
            // LOG(INFO) << "waiting for messages";
            // This buffer will hold the incoming message
            beast::flat_buffer buffer;

            // Read a message
            ws1->read(buffer);
            string text = beast::buffers_to_string(buffer.data());

            // handling client messages
            if (text == "hello") 
            {
                // send out calibrations
                ws1->binary(true);

                // serialization
                auto text_data = json::array();

                text_data.push_back("calibration");
                // device num
                text_data.push_back(calibrations_init.size());

                for (auto it : calibrations_init) {
                    text_data.push_back(it->worldT);
                    text_data.push_back(it->worldR);
                    text_data.push_back(it->intrinsics.fx);
                    text_data.push_back(it->intrinsics.fy);
                    text_data.push_back(it->intrinsics.cx);
                    text_data.push_back(it->intrinsics.cy);
                    text_data.push_back(it->intrinsics.width);
                    text_data.push_back(it->intrinsics.height);
                }

                // // xy_tables are only used in Kinect dataset, not for Panoptic.
                // for (auto it : xy_tables_init) {
                //     LOG(INFO) << it->width;
                //     text_data.push_back(it->width);
                //     text_data.push_back(it->height);
                //
                //     vector<float> x_table_data {it->x_table, it->x_table + it->width*it->height};
                //     text_data.push_back(x_table_data);
                //     vector<float> y_table_data {it->y_table, it->y_table + it->width*it->height};
                //     text_data.push_back(y_table_data);
                // }

                text_data.push_back(deviceIndices_init);
                text_data.push_back(deviceSerialNumbers_init);

                ws1->write(net::buffer(json::to_msgpack(text_data)));                
                LOG(INFO) << "Calibrations sent to client";
            } 
            else 
            {
                try
                {
                    // LOG(INFO) << "Received New Frustum from Client";

                    // parse array from json format
                    auto text_data = json::from_msgpack(buffers_begin(buffer.data()), buffers_end(buffer.data()));

                    // TODO: Rajrup: Check if this sleep blocks CPU
                    // Wait until last frustum is loaded by server
                    while(bool x = new_frustum.load(memory_order_relaxed) == true)
                        sleep_ms(1);

                    camInt.angle = text_data[0];
                    camInt.ratio = text_data[1];
                    camInt.nearD = text_data[2];
                    camInt.farD = text_data[3];

                    for(int i=0; i<3; i++)
                        pos[i] = text_data[4+i];
                    
                    for(int i=0; i<4; i++)
                        quat[i] = text_data[7+i];
                    
                    calc_frustum_from_pos_rot2(pos, rot, quat, camInt, frustum_recv);
                    frustum_recv.frameID = text_data[11];
                    frustum_sent_time = text_data[12];
                    frustum_recv_time = time_now_ms();
                    // cout << endl << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
                    // cout << "Sent Time (in ms): " << frustum_sent_time << endl;
                    // cout << "Received Time (in ms): " << frustum_recv_time << endl;
                    // cout << "Latency (in ms): " << frustum_recv_time - frustum_sent_time << endl;
                    // frustum_recv.print();

                    // new_frustum.store(false);
                    new_frustum.store(true);
                } 
                catch(std::exception const& e) 
                {
                    LOG(ERROR) << "Error: " << e.what();
                    new_frustum.store(false);
                }

                // ********************** Frustum Clip Old ********************** //
                // try
                // {
                //     // parse array from json format
                //     LOG(INFO) << "Received New Frustum from Client";
                //     auto text_data = json::from_msgpack(buffers_begin(buffer.data()), buffers_end(buffer.data()));
                    
                //     // TODO: Rajrup: Check if this sleep blocks CPU
                //     // Wait until last frustum is loaded by server
                //     while(bool x = new_frustumClip.load(memory_order_relaxed) == true) {
                //         sleep(0.01);
                //     }

                //     for (int i=0; i<N_FRUSTRUM_PLANE; i++) {
                //         for (int j=0; j<10; j++)
                //             frustumClip_recv.pl[i].eq[j] = text_data[10*i+j];
                //     }         
                //     frustumClip_recv.print();

                //     new_frustumClip.store(true);
                // } 
                // catch(std::exception const& e) 
                // {
                //     LOG(ERROR) << "Error: " << e.what();
                //     new_frustumClip.store(false);
                // }
            }            
        }
    }
    catch(beast::system_error const& se)
    {
        // This indicates that the session was closed
        if(se.code() != websocket::error::closed)
            LOG(ERROR) << "Error: " << se.code().message();
    }
    catch(std::exception const& e)
    {
        LOG(ERROR) << "Error: " << e.what();
    }
}

/**
 * @brief Sends masks to client over websocket.
 * @param socket 
 */
void WebSocketServer::do_session2(tcp::socket socket)
{
    LOG(INFO) << "Websocket server received connection request";
    LOG(INFO) << "This socket will send masks";
    try
    {
        // Construct the stream by moving in the socket
        ws2 = new websocket::stream<tcp::socket> {std::move(socket)};

        // Set a decorator to change the Server of the handshake
        ws2->set_option(websocket::stream_base::decorator(
        [](websocket::response_type& res)
            {
                res.set(http::field::server,
                    std::string(BOOST_BEAST_VERSION_STRING) +
                        " websocket-server-sync");
            }));

        // Accept the websocket handshake
        ws2->accept();
        LOG(INFO) << "This thread will send masks over socket: " << ws2->next_layer().local_endpoint().address().to_string() << ":" << ws2->next_layer().local_endpoint().port();
        for(;;)
        {
            LOG(INFO) << "waiting for messages";
            // This buffer will hold the incoming message
            beast::flat_buffer buffer;

            // Read a message
            ws2->read(buffer);
            string text = beast::buffers_to_string(buffer.data());

            // handling client messages
            if (text == "hello") {
                // send out calibrations
                ws2->binary(true);

                int temp = 0;
                for(;;) 
                {
                    if (_isSendPtcl == 0) 
                    {
                        // TODO: Rajrup: Check if this sleep blocks CPU
                        while(bool x = new_mask.load(memory_order_relaxed) == false)
                            sleep(0.001);

                        try
                        {
                            // int original_size = (calibrations_init[0]->intrinsics.width * calibrations_init[0]->intrinsics.height + 7) / 8 * 10;
                            // int compressed_size = ZSTD_compressBound(original_size);

                            // uint8_t *compressed_mask_send = new uint8_t[compressed_size];
                            // ZSTD_compress(compressed_mask_send, compressed_size, mask_send, original_size, 1);
                            
                            // ws2->write(net::buffer(compressed_mask_send, compressed_size));
                            // new_mask.store(false);

                            auto compressed_mask = rle_binary_mask_to_vector(mask_send, calibrations_init[0]->intrinsics.width, calibrations_init[0]->intrinsics.height);
                            ws2->write(net::buffer(compressed_mask, compressed_mask.size()*sizeof(int)));

                            new_mask.store(false);
                        }
                        catch(std::exception const& e)
                        {
                            // LOG(ERROR) << "[WebRTC] Error on send mask: " << e.what();
                            new_mask.store(false);
                        }
                    }
                    // else {
                    //     try
                    //     {
                    //         // TODO: once ptcl generation is fixed, replace this part
                    //         string str = to_string(temp);
                    //         const char* cstr = str.c_str();
                    //         temp++;

                    //         ws2->write(net::buffer(cstr, strlen(cstr)));   
                    //         LOG(INFO) << "ws2 sending data";
                    //     }
                    //     catch(std::exception const& e)
                    //     {
                    //         // LOG(ERROR) << "[WebRTC] Error on send mask: " << e.what();
                    //     }
                    // }
                }
            }
        }
    }
    catch(beast::system_error const& se)
    {
        // This indicates that the session was closed
        if(se.code() != websocket::error::closed)
            LOG(ERROR) << "Error: " << se.code().message();
    }
    catch(std::exception const& e)
    {
        LOG(ERROR) << "Error: " << e.what();
    }
}

/**
 * @brief Receives client side mahimahi bandwidth estimate (bwe) over websocket.
 * @param socket 
 */
void WebSocketServer::do_session3(tcp::socket socket)
{
    LOG(INFO) << "Websocket server received connection request";
    LOG(INFO) << "This socket will receive client side mahimahi bandwidth estimate (bwe)";
    try
    {
        LOG(INFO) << "Websocket server received connection request";
        // Construct the stream by moving in the socket
        ws3 = new websocket::stream<tcp::socket> {std::move(socket)};

        // Set a decorator to change the Server of the handshake
        ws3->set_option(websocket::stream_base::decorator(
        [](websocket::response_type& res)
            {
                res.set(http::field::server,
                    std::string(BOOST_BEAST_VERSION_STRING) +
                        " websocket-server-sync");
            }));

        // Accept the websocket handshake
        ws3->accept();
        LOG(INFO) << "This thread will receive bwe over socket: " << ws3->next_layer().local_endpoint().address().to_string() << ":" << ws3->next_layer().local_endpoint().port();
        for(;;)
        {
            LOG(INFO) << "waiting for messages";
            // This buffer will hold the incoming message
            beast::flat_buffer buffer;

            // Read a message
            ws3->read(buffer);
            string text = beast::buffers_to_string(buffer.data());

            // handling client messages
            if (text == "hello") 
            {
                // send out calibrations
                ws3->binary(true);
                LOG(INFO) << "ws3 received hello";

                for(;;)
                {
                    // This buffer will hold the incoming message
                    beast::flat_buffer buffer;

                    // Read a message
                    ws3->read(buffer);
                    string text = beast::buffers_to_string(buffer.data());
                    // ws3->binary(true);

                    try
                    {
                        if(FLAGS_use_client_bitrate)
                        {
                            // parse array from json format
                            auto text_data = json::from_msgpack(buffers_begin(buffer.data()), buffers_end(buffer.data()));
                            bwe.store(text_data[0]);
                            // LOG(INFO) << "Received bwe from client: " << bwe.load();
                        }
                    }
                    catch(std::exception const& e) 
                    {
                        LOG(ERROR) << "Error: " << e.what();
                    }
                            
                }
            }
        }
    }
    catch(beast::system_error const& se)
    {
        // This indicates that the session was closed
        if(se.code() != websocket::error::closed)
            LOG(ERROR) << "Error: " << se.code().message();
    }
    catch(std::exception const& e)
    {
        LOG(ERROR) << "Error: " << e.what();
    }
}

/**
 * @brief Sends point clouds to client over websocket.
 * @param socket 
 */
void WebSocketServer::do_session_ptcl(tcp::socket socket)
{
    LOG(INFO) << "Websocket server received connection request";
    LOG(INFO) << "This socket will send point clouds";
    if (_isSendPtcl == 0) 
    {
        LOG(FATAL) << "Only used for sending point clouds";
        return;
    }
    try
    {
        LOG(INFO) << "Websocket server received connection request";
        LOG(INFO) << "This socket will send ptcl";
        // Construct the stream by moving in the socket
        ws_ptcl = new websocket::stream<tcp::socket> {std::move(socket)};

        // Set a decorator to change the Server of the handshake
        ws_ptcl->set_option(websocket::stream_base::decorator(
        [](websocket::response_type& res)
            {
                res.set(http::field::server,
                    std::string(BOOST_BEAST_VERSION_STRING) +
                        " websocket-server-sync");
            }));

        // Accept the websocket handshake
        ws_ptcl->accept();
        LOG(INFO) << "This thread will send point clouds over socket: " << ws_ptcl->next_layer().local_endpoint().address().to_string() << ":" << ws_ptcl->next_layer().local_endpoint().port();
        // Waiting for client to send hello
        for(;;)
        {
            LOG(INFO) << "waiting for messages";
            // This buffer will hold the incoming message
            beast::flat_buffer buffer;

            // Read a message
            ws_ptcl->read(buffer);
            string text = beast::buffers_to_string(buffer.data());

            // handling client messages
            if (text == "hello") 
            {
                LOG(INFO) << "WebSocket for ptcl transmission - received handshake from client";

                // send out calibrations
                ws_ptcl->binary(true);
                int temp = 0;
                // Send ptcl
                for(;;)
                {
                    // TODO: Rajrup: Check if this sleep blocks CPU
                    while(bool x = new_ptcl.load(memory_order_relaxed) == false)
                        sleep_ms(1);
                    try
                    {
                        // Send entire compressed ptcl
                        ws_ptcl->write(net::buffer(ptcl_buf, ptcl_buf_size));

                        new_ptcl.store(false);
                        LOG(INFO) << "Sent data. Data Size: " << ptcl_buf_size;
                    }
                    catch(std::exception const& e)
                    {
                        new_ptcl.store(false);
                        // LOG(ERROR) << "[WebRTC] Error on send mask: " << e.what();
                    }
                }
            }
        }
    }
    catch(beast::system_error const& se)
    {
        // This indicates that the session was closed
        if(se.code() != websocket::error::closed)
            LOG(ERROR) << "Error: " << se.code().message();
    }
    catch(std::exception const& e)
    {
        LOG(ERROR) << "Error: " << e.what();
    }
}