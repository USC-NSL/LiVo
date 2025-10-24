#include "Client/NetworkReceiver.h"
#include "colorized_depth.h"

#define MAX_RECV_FRAME_CAPACITY 30  // Store 30 frames. 30 frames when streamed at 30 fps will contain 1 sec of data.

#define PTCL_MAX_POINTS 2000000 // Maximum number of points in point cloud is 2 million. Used for allocating memory for compressed point cloud buffer.

atomic_bool main_worker_ready_to_init2(false);
atomic_bool next_frame_ready2(false);
atomic_int next_frame2(0);

int cimg_h2 = 0;
int cimg_w2 = 0;
int dimg_h2 = 0;
int dimg_w2 = 0;

vector<Calibration *> calibrations_recv2;
vector<xy_table_t *> xy_tables_recv2;
vector<uint32_t> deviceIndices_recv2;
vector<std::string> deviceSerialNumbers_recv2;

atomic_bool new_frustumClip2(false);
FrustumClip frustumClip_send2;

atomic_bool new_frustum2(false);
Frustum frustum_send2;

atomic_bool new_mask2(false);
uint8_t *mask_recv2;

uint8_t *compressed_ptcl_buf2;
uint32_t compressed_ptcl_buf_size2;

NetworkReceiver::NetworkReceiver(std::string type, bool isSaveView, std::string save_dir): type(type), isSaveView(isSaveView), save_dir(save_dir)
{
    LOG(INFO) << "WebRTC Server Init for type " << type;
    view_count = deviceIndices_recv2.size();
    if(cimg_h2 != calibrations_recv2[0]->intrinsics.height || cimg_w2 != calibrations_recv2[0]->intrinsics.width)
        LOG(FATAL) << "cimh_h2: " << cimg_h2 << " != calibrations_recv2[0]->intrinsics.height: " << calibrations_recv2[0]->intrinsics.height << " || cimg_w2: " << cimg_w2 << " != calibrations_recv2[0]->intrinsics.width: " << calibrations_recv2[0]->intrinsics.width;

    if(dimg_h2 != calibrations_recv2[0]->intrinsics.height || dimg_w2 != calibrations_recv2[0]->intrinsics.width)
        LOG(FATAL) << "dimg_h2: " << dimg_h2 << " != calibrations_recv2[0]->intrinsics.height: " << calibrations_recv2[0]->intrinsics.height << " || dimg_w2: " << dimg_w2 << " != calibrations_recv2[0]->intrinsics.width: " << calibrations_recv2[0]->intrinsics.width;

    int imgShape[2] = {calibrations_recv2[0]->intrinsics.height * 2, calibrations_recv2[0]->intrinsics.width * 5};
    int view_size = 0;
    if(type == "c" || type == "c_bgra")
    {
        view_size = calibrations_recv2[0]->intrinsics.height * calibrations_recv2[0]->intrinsics.width * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA;

        recv_cframe_buffer = new ReceiverFrameBuffer(type, MAX_RECV_FRAME_CAPACITY, view_size * view_count);
        webrtc_receiver = new WebRTCReceiver(type, imgShape, FLAGS_start_frame_id, FLAGS_end_frame_id, save_dir, WEBRTC_SERVER_HOST_NEW, WEBRTC_CSERVER_PORT, FLAGS_client_fps, recv_cframe_buffer);
    }
    else if(type == "d" || type == "d_bgra")
    {
        view_size = calibrations_recv2[0]->intrinsics.height * calibrations_recv2[0]->intrinsics.width * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA;

        recv_dframe_buffer = new ReceiverFrameBuffer(type, MAX_RECV_FRAME_CAPACITY, view_size * view_count);
        webrtc_receiver = new WebRTCReceiver(type, imgShape, FLAGS_start_frame_id, FLAGS_end_frame_id, save_dir, WEBRTC_SERVER_HOST_NEW, WEBRTC_DSERVER_PORT, FLAGS_client_fps, recv_dframe_buffer);

        colorized_depth = new RGB*[view_count];
        for (int i=0; i<view_count; i++) 
            colorized_depth[i] = new RGB[view_size];
    }
    else if(type == "d_yuv16")
    {
        view_size = calibrations_recv2[0]->intrinsics.height * calibrations_recv2[0]->intrinsics.width * PIXEL_SIZE::DEPTH; // Only y-channel is stored. So, CHANNEL_DIM::DEPTH isn't multiplied.

        recv_dframe_buffer = new ReceiverFrameBuffer(type, MAX_RECV_FRAME_CAPACITY, view_size * view_count);
        webrtc_receiver = new WebRTCReceiver(type, imgShape, FLAGS_start_frame_id, FLAGS_end_frame_id, save_dir, WEBRTC_SERVER_HOST_NEW, WEBRTC_DSERVER_PORT, FLAGS_client_fps, recv_dframe_buffer);
    }
    else
    {
        LOG(FATAL) << "Unknown type: " << type;
    }

    recv_buf_size = view_size * view_count;
    recv_buf = new uint8_t[recv_buf_size];

    if(check_webrtc_plugins())
        LOG(INFO) << "WebRTC plugins found.";
    else
        LOG(FATAL) << "WebRTC plugins not found.";

    std::thread wt(&WebRTCReceiver::connect, webrtc_receiver);
    webrtc_thread = std::move(wt);
}

NetworkReceiver::~NetworkReceiver()
{
    if(webrtc_thread.joinable())
        webrtc_thread.join();
    
    if(webrtc_receiver != NULL)
        delete webrtc_receiver;

    if(recv_cframe_buffer != NULL)
        delete recv_cframe_buffer;

    if(recv_dframe_buffer != NULL)
        delete recv_dframe_buffer;

    if(recv_buf != NULL)
        delete[] recv_buf;

    if(colorized_depth != NULL)
    {
        for (int i=0; i<view_count; i++)
            delete[] colorized_depth[i];
        delete[] colorized_depth;
    }
}

int NetworkReceiver::get_latest_frame_id()
{
    if(type == "c" || type == "c_bgra")
        return recv_cframe_buffer->get_latest_frame_id();
    else if(type == "d" || type == "d_bgra" || type == "d_yuv16")
        return recv_dframe_buffer->get_latest_frame_id();
    else
        LOG(FATAL) << "Unknown type: " << type;
    return -1;
}

RECV_STATUS NetworkReceiver::recv_tiled_color(vector<RGB *> &color_views, int requested_frame_id)
{
    StopWatch sw;
    sw.Restart();

    // int latest_frame_id = recv_cframe_buffer->get_latest_frame_id();
    // if(latest_frame_id < requested_frame_id)
    // {
    //     LOG(WARNING) << "Color, Requested frame id: " << requested_frame_id << " is not available. Latest frame id: " << latest_frame_id;
    //     return false;
    // }

    // Find the requested frame in the recv buffer. Assuming the frames are received in increasing order of frame id.
    int removed_frame_id = -1;
    int recv_loop_count = 0;
    while(removed_frame_id < requested_frame_id)   // Remove all frames before the requested_frame_id.
    {
        REMOVE_STATUS status = recv_cframe_buffer->remove_frame(recv_buf, recv_buf_size, requested_frame_id, removed_frame_id); 
        
        if(status == REMOVE_STATUS::REMOVE_FAIL)    // The buffer is probably empty. So wait
        {
            LOG(ERROR) << "Color, Empty buffer. Requested Frame " << requested_frame_id << ". Waiting...";
            return RECV_STATUS::FRAME_WAIT;
        }
        else if(status == REMOVE_STATUS::REMOVE_NOT_FOUND)  // Frame not found. Possibly skipped.
        {
            LOG(WARNING) << "Color, Frame not found in buffer. Requested Frame " << requested_frame_id;
            return RECV_STATUS::FRAME_SKIP;
        }
        else if(status == REMOVE_STATUS::REMOVE_SUCCESS)    // Frame found and removed
        {
            LOG(INFO) << "Color, Removed frame id: " << removed_frame_id << " from buffer. Requested Frame " << requested_frame_id;
        }
        else
            LOG(FATAL) << "Color, Unknown status: " << status;
        recv_loop_count++;
    }

    // Check for me. Remove later.
    if(recv_loop_count > 1)
        LOG(WARNING) << "Color, Frame matching required " << recv_loop_count << " loops";
    
    // Frame id might not match the requested frame id. This can happen if the frame is skipped (qr loss, frame loss, etc.)
    if(removed_frame_id != requested_frame_id)
    {
        LOG(ERROR) << "Color, Removed frame id: " << removed_frame_id << " is not equal to requested frame id: " << requested_frame_id;
        return RECV_STATUS::FRAME_SKIP;
    }

    auto h = calibrations_recv2[0]->intrinsics.height;
    auto w = calibrations_recv2[0]->intrinsics.width;

    #pragma omp parallel for
    for(int view_id = 0; view_id < view_count; view_id++)
    {
        int x = view_id / 5 * h, y = view_id % 5 * (w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA);
        uint8_t *color_view = reinterpret_cast<uint8_t *>(color_views[view_id]);

        for(int j = 0; j < h; j++)
            memcpy(
                    color_view + (j * w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA), 
                    recv_buf + ((x + j) * w * 5 * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA + y), 
                    w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA
                );
    }

    if(isSaveView && (requested_frame_id % 100 == 0))
    {
        if(!fs::exists(save_dir))
            fs::create_directories(save_dir);
        for(int i = 0; i < view_count; i++)
            save_color(reinterpret_cast<uint8_t *>(color_views[i]), h, w, FORMAT(save_dir << requested_frame_id << "_color_bgra_" << i << ".png"));
    }
    return RECV_STATUS::FRAME_READY;
}

RECV_STATUS NetworkReceiver::recv_tiled_depth(vector<uint16_t *> &depth_views, int requested_frame_id)
{
    StopWatch sw;
    sw.Restart();

    // int latest_frame_id = recv_dframe_buffer->get_latest_frame_id();
    // if(latest_frame_id < requested_frame_id)
    // {
    //     LOG(WARNING) << "Depth, Requested frame id: " << requested_frame_id << " is not available. Latest frame id: " << latest_frame_id;
    //     return false;
    // }

    // Find the requested frame in the recv buffer. Assuming the frames are received in increasing order of frame id.
    int removed_frame_id = -1;
    int recv_loop_count = 0;
    while(removed_frame_id < requested_frame_id)   // Remove all frames before the requested_frame_id.
    {
        REMOVE_STATUS status = recv_dframe_buffer->remove_frame(recv_buf, recv_buf_size, requested_frame_id, removed_frame_id); 
        
        if(status == REMOVE_STATUS::REMOVE_FAIL)    // The buffer is probably empty. So wait
        {
            LOG(ERROR) << "Depth, Empty buffer. Requested Frame " << requested_frame_id;
            return RECV_STATUS::FRAME_WAIT;
        }
        else if(status == REMOVE_STATUS::REMOVE_NOT_FOUND)  // Frame not found. Possibly skipped.
        {
            LOG(WARNING) << "Depth, Frame not found in buffer. Requested Frame " << requested_frame_id;
            return RECV_STATUS::FRAME_SKIP;
        }
        else if(status == REMOVE_STATUS::REMOVE_SUCCESS)    // Frame found and removed
        {
            LOG(INFO) << "Depth, Removed frame id: " << removed_frame_id << " from buffer. Requested Frame " << requested_frame_id;
        }
        else
            LOG(FATAL) << "Depth, Unknown status: " << status;
        recv_loop_count++;
    }

    // Check for me. Remove later.
    if(recv_loop_count > 1)
        LOG(WARNING) << "Depth, Frame matching required " << recv_loop_count << " loops";
    
    // Frame id might not match the requested frame id. This can happen if the frame is skipped (qr loss, frame loss, etc.)
    if(removed_frame_id != requested_frame_id)
    {
        LOG(ERROR) << "Depth, Removed frame id: " << removed_frame_id << " is not equal to requested frame id: " << requested_frame_id;
        return RECV_STATUS::FRAME_SKIP;
    }

    auto h = calibrations_recv2[0]->intrinsics.height;
    auto w = calibrations_recv2[0]->intrinsics.width;

    #pragma omp parallel for
    for(int view_id = 0; view_id < view_count; view_id++)
    {
        int x = view_id / 5 * h, y = view_id % 5 * (w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA);
        uint8_t *colorized_depth_view = reinterpret_cast<uint8_t *>(colorized_depth[view_id]);

        for(int j = 0; j < h; j++)
            memcpy(
                    colorized_depth_view + (j * w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA), 
                    recv_buf + ((x + j) * w * 5 * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA + y), 
                    w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA
                );
        
        // Colorized depth is received in BGRA format. Convert it to depth.
        color_to_depth(colorized_depth[view_id], h, w, depth_views[view_id], false);
    }

    if(isSaveView && (requested_frame_id % 100 == 0))
    {
        if(!fs::exists(save_dir))
            fs::create_directories(save_dir);
        for(int i = 0; i < view_count; i++)
            save_color(reinterpret_cast<uint8_t *>(colorized_depth[i]), h, w, FORMAT(save_dir << requested_frame_id << "_depth_bgra_" << i << ".png"));
    }
    return RECV_STATUS::FRAME_READY;
}

RECV_STATUS NetworkReceiver::recv_tiled_depth_yuv16(vector<uint16_t *> &depth_views, int requested_frame_id)
{
    StopWatch sw;
    sw.Restart();

    // int latest_frame_id = recv_dframe_buffer->get_latest_frame_id();
    // if(latest_frame_id < requested_frame_id)
    // {
    //     LOG(WARNING) << "Depth YUV16, Requested frame id: " << requested_frame_id << " is not available. Latest frame id: " << latest_frame_id;
    //     // LOG(WARNING) << "Depth YUV16, Skipping frame id: " << requested_frame_id;
    //     return false;
    // }

    // Find the requested frame in the recv buffer. Assuming the frames are received in increasing order of frame id.
    int removed_frame_id = -1;
    int recv_loop_count = 0;
    while(removed_frame_id < requested_frame_id)   // Remove all frames before the requested_frame_id.
    {
        REMOVE_STATUS status = recv_dframe_buffer->remove_frame(recv_buf, recv_buf_size, requested_frame_id, removed_frame_id); 
        
        if(status == REMOVE_STATUS::REMOVE_FAIL)    // The buffer is probably empty. So wait
        {
            LOG(ERROR) << "Depth YUV16, Empty buffer. Requested Frame " << requested_frame_id;
            return RECV_STATUS::FRAME_WAIT;
        }
        else if(status == REMOVE_STATUS::REMOVE_NOT_FOUND)  // Frame not found. Possibly skipped.
        {
            LOG(WARNING) << "Depth YUV16, Frame not found in buffer. Requested Frame " << requested_frame_id;
            return RECV_STATUS::FRAME_SKIP;
        }
        else if(status == REMOVE_STATUS::REMOVE_SUCCESS)    // Frame found and removed
        {
            LOG(INFO) << "Depth YUV16, Removed frame id: " << removed_frame_id << " from buffer. Requested Frame " << requested_frame_id;
        }
        else
            LOG(FATAL) << "Depth YUV16, Unknown status: " << status;
        recv_loop_count++;
    }

    // Check for me. Remove later.
    if(recv_loop_count > 1)
        LOG(WARNING) << "Depth YUV16, Frame matching required " << recv_loop_count << " loops";
    
    // Frame id might not match the requested frame id. This can happen if the frame is skipped (qr loss, frame loss, etc.)
    if(removed_frame_id != requested_frame_id)
    {
        LOG(ERROR) << "Depth YUV16, Removed frame id: " << removed_frame_id << " is not equal to requested frame id: " << requested_frame_id;
        return RECV_STATUS::FRAME_SKIP;
    }

    auto h = calibrations_recv2[0]->intrinsics.height;
    auto w = calibrations_recv2[0]->intrinsics.width;

    #pragma omp parallel for
    for(int view_id = 0; view_id < view_count; view_id++)
    {
        int x = view_id / 5 * h, y = view_id % 5 * (w * PIXEL_SIZE::DEPTH);
        uint8_t *depth_view = reinterpret_cast<uint8_t *>(depth_views[view_id]);

        for(int j = 0; j < h; j++)
            memcpy(
                    depth_view + (j * w * PIXEL_SIZE::DEPTH), 
                    recv_buf + ((x + j) * 5 * w * PIXEL_SIZE::DEPTH + y), 
                    w * PIXEL_SIZE::DEPTH
                );
    }

    if(isSaveView && (requested_frame_id % 100 == 0))
    {
        if(!fs::exists(save_dir))
            fs::create_directories(save_dir);
        
        for (int i = 0; i < view_count; i++) 
        {
            cv::Mat depth_image_cv2(h, w, CV_16UC1, reinterpret_cast<uint8_t *>(depth_views[i]));
            std::string path = FORMAT(save_dir << requested_frame_id << "_depth_yuv16_" << i << ".png");
            if(!cv::imwrite(path, depth_image_cv2))
                LOG(ERROR) << "Depth YUV16, Failed to write depth image: " << path;
        }
    }
    return RECV_STATUS::FRAME_READY;
}

NetworkReceiverPool::NetworkReceiverPool(bool isSaveView, std::string save_dir): isSaveView(isSaveView), save_dir(save_dir)
{
    view_count = deviceIndices_recv2.size();
    if(cimg_h2 != calibrations_recv2[0]->intrinsics.height || cimg_w2 != calibrations_recv2[0]->intrinsics.width)
        LOG(FATAL) << "cimh_h2: " << cimg_h2 << " != calibrations_recv2[0]->intrinsics.height: " << calibrations_recv2[0]->intrinsics.height << " || cimg_w2: " << cimg_w2 << " != calibrations_recv2[0]->intrinsics.width: " << calibrations_recv2[0]->intrinsics.width;

    if(dimg_h2 != calibrations_recv2[0]->intrinsics.height || dimg_w2 != calibrations_recv2[0]->intrinsics.width)
        LOG(FATAL) << "dimg_h2: " << dimg_h2 << " != calibrations_recv2[0]->intrinsics.height: " << calibrations_recv2[0]->intrinsics.height << " || dimg_w2: " << dimg_w2 << " != calibrations_recv2[0]->intrinsics.width: " << calibrations_recv2[0]->intrinsics.width;

    // for (int i=0; i<view_count; i++)
    // {
    //     View* v = new View();
    //     v->initView(dimg_w2, dimg_h2, cimg_w2, cimg_h2);
    //     next_views.push_back(v);
    // }

    // Only supports tiled mode
    creceiver = new NetworkReceiver("c", isSaveView, save_dir);
    int color_view_size = calibrations_recv2[0]->intrinsics.height * calibrations_recv2[0]->intrinsics.width;
    color_views.resize(view_count);
    for(int i = 0; i < view_count; i++)
        color_views[i] = new RGB[color_view_size];

    // // Rajrup: DEPTH_BGRA
    // dreceiver = new NetworkReceiver("d", isSaveView, save_dir);
    // int depth_view_size = calibrations_recv2[0]->intrinsics.height * calibrations_recv2[0]->intrinsics.width;
    // depth_views.resize(view_count);
    // for(int i = 0; i < view_count; i++)
    //     depth_views[i] = new uint16_t[depth_view_size];

    // Rajrup: DEPTH_YUV16
    dreceiver = new NetworkReceiver("d_yuv16", isSaveView, save_dir);
    int depth_view_size = calibrations_recv2[0]->intrinsics.height * calibrations_recv2[0]->intrinsics.width; // Only y-channel is stored. So, CHANNEL_DIM::DEPTH isn't multiplied.
    depth_views.resize(view_count);
    for(int i = 0; i < view_count; i++)
        depth_views[i] = new uint16_t[depth_view_size];
}

NetworkReceiverPool::~NetworkReceiverPool()
{
    if(creceiver != NULL)
        delete creceiver;

    if(dreceiver != NULL)
        delete dreceiver;

    for(int i = 0; i < view_count; i++)
    {
        if(color_views[i] != NULL)
            delete[] color_views[i];

        if(depth_views[i] != NULL)
            delete[] depth_views[i];
    }
}

/**
 * @brief If either color or depth frame is missing, the this frames returns false. In that case, the requested views are inconsistent.
 * 
 * @param requested_views This function assumes that vector<View *> requested_views is already allocated and has the same size as view_count.
 * @param requested_frame_id  
 */
RECV_STATUS NetworkReceiverPool::poll_frame(vector<View *> &requested_views, int requested_frame_id)
{   
    StopWatch sw;
    sw.Restart();

    RECV_STATUS status = RECV_STATUS::FRAME_WAIT;
    // Color views
    while(status == RECV_STATUS::FRAME_WAIT)
    {
        status = creceiver->recv_tiled_color(color_views, requested_frame_id);
        sleep_ms(5);
        if(sw.ElapsedMs() > PIPELINE_DATA_MAX_WAIT)
            LOG(FATAL) << "Color, Waited for color frame for 20 sec! Quitting.";
    }
    
    if(status == RECV_STATUS::FRAME_SKIP)
    {
        LOG(ERROR) << "Color, Failed to receive color frame for frame id: " << requested_frame_id << ". Skipping entire frame.";
        return RECV_STATUS::FRAME_SKIP;
    }
    LOG(INFO) << "Color, frame received for frame id: " << requested_frame_id;


    // // Rajrup: DEPTH_BGRA
    // sw.Restart();
    // if(!dreceiver->recv_tiled_depth(depth_views, requested_frame_id))
    // {
    //     LOG(ERROR) << "Failed to receive depth frame for frame id: " << requested_frame_id << ". Skipping entire frame.";
    //     return false;
    // }
    // LOG(INFO) << "Depth frame received for frame id: " << requested_frame_id << " in " << sw.ElapsedMs() << " ms";

    // Rajrup: DEPTH_YUV16
    status = RECV_STATUS::FRAME_WAIT;
    while(status == RECV_STATUS::FRAME_WAIT)
    {
        status = dreceiver->recv_tiled_depth_yuv16(depth_views, requested_frame_id);
        sleep_ms(5);
        if(sw.ElapsedMs() > PIPELINE_DATA_MAX_WAIT)
            LOG(FATAL) << "Depth YUV16, Waited for depth frame for 20 sec! Quitting.";
    }

    if(status == RECV_STATUS::FRAME_SKIP)
    {
        LOG(ERROR) << "Depth YUV16, Failed to receive depth frame for frame id: " << requested_frame_id << ". Skipping entire frame.";
        return RECV_STATUS::FRAME_SKIP;
    }
    LOG(INFO) << "Depth YUV16 frame received for frame id: " << requested_frame_id;

    #pragma omp parallel for
    for(int i = 0; i < view_count; i++)
    {
        requested_views[i]->viewID = i;
        requested_views[i]->frameID = requested_frame_id;
        requested_views[i]->valid = true;

        // It should be okay to only use pointer than copying the entire data.
        // We will call this function frame by frame. So, the data will not be overwritten.
        requested_views[i]->color_image = color_views[i];
        requested_views[i]->depth_image = depth_views[i];
    }

    // Debug
    // if(requested_frame_id % 10 == 0)
    // {
    //     if(!fs::exists(save_dir))
    //         fs::create_directories(save_dir);
    //     for(int i = 0; i < view_count; i++)
    //     {
    //         save_color(reinterpret_cast<uint8_t *>(color_views[i]), calibrations_recv2[0]->intrinsics.height, calibrations_recv2[0]->intrinsics.width, FORMAT(save_dir << requested_frame_id << "_color_bgra_" << i << ".png"));
    //         cv::Mat depth_image_cv2(calibrations_recv2[0]->intrinsics.height, calibrations_recv2[0]->intrinsics.width, CV_16UC1, reinterpret_cast<uint8_t *>(depth_views[i]));
    //         std::string path = FORMAT(save_dir << requested_frame_id << "_depth_yuv16_" << i << ".png");
    //         if(!cv::imwrite(path, depth_image_cv2))
    //             LOG(ERROR) << "Failed to write depth image: " << path;
    //     }
    // }

    LOG(INFO) << "Frame ID - " << requested_frame_id << " received in " << sw.ElapsedMs() << " ms";

    return RECV_STATUS::FRAME_READY;
}

NetworkSocketClient::NetworkSocketClient(const char* addr, int port, const int isSendPtcl): 
        _addr(addr), _port(port), 
        resolver1(ioc1), ws1(ioc1), 
        resolver2(ioc2), ws2(ioc2), 
        resolver3(ioc3), ws3(ioc3),
        resolver_ptcl(ioc_ptcl), ws_ptcl(ioc_ptcl),
        _isSendPtcl(isSendPtcl) 
{
    if(_isSendPtcl)
    {
        compressed_ptcl_buf2 = new uint8_t[PTCL_MAX_POINTS * 3 * (sizeof(float) + sizeof(uint8_t))];
        compressed_ptcl_buf_size2 = 0;
    }

    while(1) 
    {
        try
        {
            host1 = std::string(_addr);
            host2 = std::string(_addr);
            host3 = std::string(_addr);
            host_ptcl = std::string(_addr);
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

NetworkSocketClient::~NetworkSocketClient() {
    ws1.close(websocket::close_code::normal);
    ws2.close(websocket::close_code::normal);
    ws3.close(websocket::close_code::normal);

}

void NetworkSocketClient::run() 
{
    try
    {
        thread t1 = std::thread(&NetworkSocketClient::get_calibration, this);       
        thread t2 = std::thread(&NetworkSocketClient::sendFrustum, this);
        // thread t2 = std::thread(&NetworkSocketClient::sendFrustumClip, this);
        thread t3 = std::thread(&NetworkSocketClient::get_mask, this);
        thread t4 = std::thread(&NetworkSocketClient::sendBwe, this);
        thread t5 = _isSendPtcl ? std::thread(&NetworkSocketClient::get_ptcl, this) : std::thread();
        
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
void NetworkSocketClient::get_calibration() 
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

                if (text_data[0] == "calibration")
                {
                    LOG(INFO) << "Received handshake from Server";
                    int wdevice_num = text_data[1];

                    for (int i=0; i<wdevice_num; i++) 
                    {
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
                        
                        calibrations_recv2.push_back(temp);
                    }

                    cimg_w2 = text_data[8];
                    dimg_w2 = text_data[8];
                    cimg_h2 = text_data[9];
                    dimg_h2 = text_data[9];

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

                    deviceIndices_recv2 = text_data[2 + 8 * wdevice_num].get<vector<uint32_t>>();
                    deviceSerialNumbers_recv2 = text_data[2 + 8 * wdevice_num + 1].get<vector<std::string>>();

                    if(deviceIndices_recv2.size() != wdevice_num)
                        LOG(FATAL) << "deviceIndices_recv2.size() != wdevice_num";

                    if(deviceSerialNumbers_recv2.size() != wdevice_num)
                        LOG(FATAL) << "deviceSerialNumbers_recv2.size() != wdevice_num";

                    LOG(INFO) << "Client received calibrations";

                    int flag_idx = 2 + 8 * wdevice_num + 2;
                    FLAGS_start_frame_id = text_data[flag_idx];
                    FLAGS_end_frame_id = text_data[flag_idx + 1];
                    FLAGS_client_fps = text_data[flag_idx + 2];
                    FLAGS_cb = text_data[flag_idx + 3];
                    FLAGS_db = text_data[flag_idx + 4];
                    FLAGS_mm_trace_name = text_data[flag_idx + 5];
                    FLAGS_server_cull = text_data[flag_idx + 6];

                    main_worker_ready_to_init2.store(true);
                }
            } 
            
            catch (std::exception const& e)
            {
                LOG(FATAL) << "Error on websocket recv: " << e.what();
            }

        }
    }
    catch(std::exception const& e)
    {
        LOG(FATAL) << "Error on read: " << e.what();
        return;
    }
    return;
}

/**
 * @brief Sends viewer's frustum to server over websocket.
 */
void NetworkSocketClient::sendFrustum() 
{
    LOG(INFO) << "This thread will send frustums over socket: " << ws1.next_layer().remote_endpoint().address().to_string() << ":" << ws1.next_layer().remote_endpoint().port();
    ws1.binary(true);
    for(;;) 
    {
        while(bool x = new_frustum2.load(memory_order_relaxed) == false)
            sleep_ms(1);

        // LOG(INFO) << "Sending frustum to server";

        try
        { 
            // serialization
            auto text_data = json::array();
            
            // LOG(INFO) << "Sending frustum to server";
            // frustum_send.print();

            // Add frustum instrinsics
            text_data.push_back(frustum_send2.angle);
            text_data.push_back(frustum_send2.ratio);
            text_data.push_back(frustum_send2.nearD);
            text_data.push_back(frustum_send2.farD);

            // Add frustum extrinsics
            for(int i=0; i<3; i++)
                text_data.push_back(frustum_send2.pos(i));
            
            for(int i=0; i<4; i++)
                text_data.push_back(frustum_send2.quat(i));

            // Add frame ID of the frustum
            text_data.push_back(frustum_send2.frameID);

            // Add send timestamp in ms
            uint64_t timestamp = time_now_ms();
            text_data.push_back(timestamp);

            // // Send the message
            ws1.write(net::buffer(json::to_msgpack(text_data)));
        }
        catch(std::exception const& e)
        {
            LOG(FATAL) << "Error on send: " << e.what();
            new_frustum2.store(false);
        }
        new_frustum2.store(false);
    }
    return;
}

/**
 * @brief Sends viewer's frustum (in clip format) to server over websocket.
 */
void NetworkSocketClient::sendFrustumClip() 
{
    LOG(INFO) << "This thread will send frustums over socket: " << ws1.next_layer().remote_endpoint().address().to_string() << ":" << ws1.next_layer().remote_endpoint().port();
    for(;;) 
    {
        while(bool x = new_frustumClip2.load(memory_order_relaxed) == false) 
        {
            sleep(0.01);
        }

        LOG(INFO) << "Sending frustum to server";

        try
        { 
            // serialization
            auto text_data = json::array();
            
            for (int i=0; i<N_FRUSTRUM_PLANE; i++) {
                for (int j=0; j<frustumClip_send2.pl[i].eq.size(); j++)
                    text_data.push_back(frustumClip_send2.pl[i].eq[j]);
            }

            // // Send the message
            ws1.write(net::buffer(json::to_msgpack(text_data)));
        }
        catch(std::exception const& e)
        {
            LOG(FATAL) << "Error on send: " << e.what();
            new_frustumClip2.store(false);
        }
        new_frustumClip2.store(false);
    }
    return;
}

/**
 * @brief Receives masks from server over websocket.
 */
void NetworkSocketClient::get_mask() 
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
                LOG(FATAL) << "Error on websocket recv: " << e.what();
                if (_isSendPtcl == 0) 
                    new_mask2.store(false);
            }
        }
    }
    catch(std::exception const& e)
    {
        LOG(FATAL) << "Error on read: " << e.what();
        return;
    }
    return;
}

/**
 * @brief Sends client side mahimahi bandwidth estimate (bwe) to server over websocket.
 */
void NetworkSocketClient::sendBwe() 
{
    LOG(INFO) << "This thread will send bwe over socket: " << ws3.next_layer().remote_endpoint().address().to_string() << ":" << ws3.next_layer().remote_endpoint().port();

    try
    {
        auto const text = "hello";

        // Send the message
        ws3.write(net::buffer(std::string(text)));
        LOG(INFO) << "Handshake sent to Server";
        ws3.binary(true);

        // int prev_sent_bwe = -1;
        // int prev_sent_cbwe = -1;
        // int prev_sent_dbwe = -1;
        float prev_sent_d2c_split = 0.0f;
        for(;;) 
        {
            try
            {
                // int bwe_now = bwe2.load();
                // int cbwe_now = cbwe2.load();
                // int dbwe_now = dbwe2.load();

                // // if (bwe_now != prev_sent_bwe || cbwe_now != prev_sent_cbwe || dbwe_now != prev_sent_dbwe)
                // if(abs(bwe_now - prev_sent_bwe) > 100 || abs(cbwe_now - prev_sent_cbwe) > 100 || abs(dbwe_now - prev_sent_dbwe) > 100)  // Send only if there is a change of 100 kbps
                // {
                //     // serialization
                //     auto text_data = json::array();
                    
                //     text_data.push_back(bwe_now);
                //     text_data.push_back(cbwe_now);
                //     text_data.push_back(dbwe_now);

                //     // Send the message
                //     ws3.write(net::buffer(json::to_msgpack(text_data)));
                //     // LOG(INFO) << "Sent bwe to server: " << bwe_now;
                //     // sleep(0.01);
                    
                //     prev_sent_bwe = bwe_now;
                //     prev_sent_cbwe = cbwe_now;
                //     prev_sent_dbwe = dbwe_now;
                // }

                float d2c_split_now = d2c_split2.load();
                if(d2c_split_now != prev_sent_d2c_split)
                {
                    // serialization
                    auto text_data = json::array();
                    text_data.push_back(d2c_split_now);

                    // Send the message
                    ws3.write(net::buffer(json::to_msgpack(text_data)));
                    // LOG(INFO) << "Sent d2c_split to server: " << d2c_split_now;
                    
                    prev_sent_d2c_split = d2c_split_now;
                }
                sleep_ms(10);
            }
            catch(std::exception const& e)
            {
                LOG(FATAL) << "Error on send: " << e.what();
            }
        }
    }
    catch(std::exception const& e)
    {
        LOG(FATAL) << "Error on read: " << e.what();
        return;
    }
    return;
}

/**
 * @brief Receives point clouds from server over websocket
 */
void NetworkSocketClient::get_ptcl() 
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
                while(bool x = next_frame_ready2.load(memory_order_relaxed) == true)
                    sleep_ms(1);

                // This buffer will hold the incoming message
                beast::multi_buffer buffer;
                ws_ptcl.read(buffer);
                LOG(INFO) << "Received data. Data Size: " << buffer.size();

                int frameID = 0;

                int counter = 0;
                compressed_ptcl_buf_size2 = 0;

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
                        compressed_ptcl_buf2[compressed_ptcl_buf_size2++] = *it;
                        // break;
                    }
                    counter++;
                }
                next_frame2 = frameID;

                LOG(INFO) << "Received - Frame ID: " << frameID << " Compressed ptcl Size: " << compressed_ptcl_buf_size2 << " bytes";
                next_frame_ready2.store(true);
            }
            catch (std::exception const& e)
            {
                LOG(FATAL) << "Error on websocket recv: " << e.what();
                next_frame_ready2.store(false);
            }
        }
    }
    catch(std::exception const& e)
    {
        LOG(FATAL) << "Error on read: " << e.what();
        return;
    }
    return;
}