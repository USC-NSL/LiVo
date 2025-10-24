#include "NetworkSender.h"
#include "DataPlayback.h"
#include <omp.h>

namespace fs = boost::filesystem;

atomic_bool wsserver_ready_to_init2(false);

vector<Calibration *> calibrations_init2;
vector<xy_table_t *> xy_tables_init2;
vector<uint32_t> deviceIndices_init2;
vector<std::string> deviceSerialNumbers_init2;

atomic_bool new_frustumClip2(false);
FrustumClip frustumClip_recv2;

atomic_bool new_frustum2(false);
Frustum frustum_recv2;
uint64_t frustum_sent_time2; // Time when frustum was sent to client in ms
uint64_t frustum_recv_time2; // Time when frustum was received from client in m

atomic_bool new_mask2(false);
bool *mask_send2;

atomic_int bwe2(-1);
atomic_int cbwe2(-1);
atomic_int dbwe2(-1);
atomic<float> d2c_split2(7.0f/8.0);

atomic_bool new_ptcl2(false);
uint8_t *ptcl_buf2;
uint32_t ptcl_buf_size2;

#define MAX_SEND_FRAME_CAPACITY 30  // Store 30 frames. 30 frames when streamed at 30 fps will contain 1 sec of data.
#define PTCL_MAX_POINTS 2000000 // Maximum number of points in point cloud. Used for allocating memory for compressed point cloud buffer.

NetworkSender::NetworkSender(std::string type, bool isSaveView, std::string save_dir): 
        type(type), 
        isSaveView(isSaveView),
        save_dir(save_dir),
        sent_frame_count(0)
{
    LOG(INFO) << "WebRTC Server Init for type " << type;
    
    view_count = deviceIndices_init2.size();
    assert(view_count == deviceSerialNumbers_init2.size() && view_count == calibrations_init2.size());

    // Set color and depth bitrate
    bwe2.store(FLAGS_cb + FLAGS_db);
    cbwe2.store(FLAGS_cb);
    dbwe2.store(FLAGS_db);

    // color_buf = NULL;
    // depth_buf = NULL;
    // colorized_depth = NULL;
    // depth_yuv16_buf = NULL;
    send_cframe_buffer = NULL;
    send_dframe_buffer = NULL;
    webrtc_sender = NULL;

    // Tile width = 2 and tile height = 5. TODO: Rajrup: hardcoded values. Change to dynamic values.
    int imgShape[2] = {calibrations_init2[0]->intrinsics.height * 2, calibrations_init2[0]->intrinsics.width * 5};
    int view_size = 0;
    if(type == "c" || type == "c_bgra")
    {
        view_size = calibrations_init2[0]->intrinsics.height * calibrations_init2[0]->intrinsics.width * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA;
        // color_buf = new uint8_t*[view_count];
        // for (int i=0; i<view_count; i++) 
        //     color_buf[i] = new uint8_t[view_size];
        
        send_cframe_buffer = new SenderFrameBuffer(type, MAX_SEND_FRAME_CAPACITY, view_size * view_count);
        webrtc_sender = new WebRTCSender(type, imgShape, save_dir, cbwe2, dbwe2, WEBRTC_SERVER_HOST_NEW, WEBRTC_CSERVER_PORT, FLAGS_server_fps, send_cframe_buffer);
    }
    else if(type == "d" || type == "d_bgra")
    {
        view_size = calibrations_init2[0]->intrinsics.height * calibrations_init2[0]->intrinsics.width * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA;
        // depth_buf = new uint8_t*[view_count];
        colorized_depth = new RGB*[view_count];
        for (int i=0; i<view_count; i++) 
        {
            // depth_buf[i] = new uint8_t[view_size];
            colorized_depth[i] = new RGB[view_size];
        }

        send_dframe_buffer = new SenderFrameBuffer(type, MAX_SEND_FRAME_CAPACITY, view_size * view_count);
        webrtc_sender = new WebRTCSender(type, imgShape, save_dir, cbwe2, dbwe2, WEBRTC_SERVER_HOST_NEW, WEBRTC_DSERVER_PORT, FLAGS_server_fps, send_dframe_buffer);
    }
    else if(type == "d_yuv16")
    {
        view_size = calibrations_init2[0]->intrinsics.height * calibrations_init2[0]->intrinsics.width * CHANNEL_DIM::DEPTH * PIXEL_SIZE::DEPTH;
        depth_yuv16_buf = new uint8_t*[view_count];
        for (int i=0; i<view_count; i++) 
            depth_yuv16_buf[i] = new uint8_t[view_size];
        
        for(int i=0; i<CHANNEL_DIM::DEPTH; i++)
            depth_yuv16_cv[i] = cv::Mat(calibrations_init2[0]->intrinsics.height, calibrations_init2[0]->intrinsics.width, CV_16UC1);

        send_dframe_buffer = new SenderFrameBuffer(type, MAX_SEND_FRAME_CAPACITY, view_size * view_count);
        webrtc_sender = new WebRTCSender(type, imgShape, save_dir, cbwe2, dbwe2, WEBRTC_SERVER_HOST_NEW, WEBRTC_DSERVER_PORT, FLAGS_server_fps, send_dframe_buffer);
    }
    else
        LOG(FATAL) << "Unknown type: " << type;

    int mask_size = calibrations_init2[0]->intrinsics.width * calibrations_init2[0]->intrinsics.height * view_count;
    mask_send2 = new bool[mask_size];

    LOG(INFO) << "Starting WebRTC Server for type " << type;

    if(webrtc_sender == NULL)
        LOG(FATAL) << "WebRTC Server is NULL";

    if(check_webrtc_plugins())
        LOG(INFO) << "WebRTC plugins found";
    else
        LOG(FATAL) << "WebRTC plugins not found";
    
    // Start webrtc sender thread - One for each type
    std::thread wt(&WebRTCSender::run, webrtc_sender);
    webrtc_thread = std::move(wt);
}

NetworkSender::~NetworkSender()
{
    // if(color_buf != NULL)
    // {
    //     for (int i=0; i<view_count; i++) 
    //         delete[] color_buf[i];
    //     delete[] color_buf;
    // }

    // if(depth_buf != NULL)
    // {
    //     for (int i=0; i<view_count; i++) 
    //         delete[] depth_buf[i];
    //     delete[] depth_buf;
    // }

    // Join webrtc thread
    if(webrtc_thread.joinable())
        webrtc_thread.join();

    if(colorized_depth != NULL)
    {
        for (int i=0; i<view_count; i++) 
            delete[] colorized_depth[i];
        delete[] colorized_depth;
    }

    if(depth_yuv16_buf != NULL)
    {
        for (int i=0; i<view_count; i++) 
            delete[] depth_yuv16_buf[i];
        delete[] depth_yuv16_buf;
    }

    if(mask_send2 != NULL)
        delete[] mask_send2;

    if(webrtc_sender != NULL)
        delete webrtc_sender;

    if(send_cframe_buffer != NULL)
        delete send_cframe_buffer;

    if(send_dframe_buffer != NULL)
        delete send_dframe_buffer;
}

/**
 * @brief Tiled color assumes that format is in BGRA
 * 
 * @param views 
 * @param frame_number 
 */
// Note: Rajrup: frame_number != view->frameID, check wserver->send(m_capture_id, s_views) in MultiviewServerPool.cpp. m_capture_id is already incremented.
// Remove frame_number from here and use view->frameID
void NetworkSender::send_tiled_color(vector<View *> views, int frame_number) 
{
    StopWatch sw;
    // LOG(INFO) << "Sending tiled color - frame_number: "<< frame_number << ", view_number: " << views[0]->frameID;
    if(views[0]->frameID != frame_number)
        LOG(FATAL) << "Color, Frame number mismatch: " << views[0]->frameID << " " << frame_number;

    if(views.size() != view_count)
        LOG(FATAL) << "Color, View count mismatch: " << views.size() << " " << view_count;

    // Insert frame to SenderFrameBuffer
    if(send_cframe_buffer == NULL)
        LOG(FATAL) << "Color, Sender Color Frame Buffer is NULL";
    
    /**
     * TODO: Rajrup: Skip frame if buffer is full and don't retry
     */
    // if(!send_cframe_buffer->insert_cframe(views, isSaveView, save_dir))
    //     LOG(ERROR) << "Failed to insert frame to Sender Color Frame Buffer. Skipping frame: " << frame_number;

    while(!send_cframe_buffer->insert_cframe(views, isSaveView, save_dir))
    {
        LOG(WARNING) << "Color, Failed to insert frame " << frame_number << " to Sender Color BGRA Frame Buffer. Waiting for buffer to clear ...";
        sleep_ms(10);
    }
    LOG(INFO) << "Color, Inserted frame " << frame_number << " to Sender Color BGRA Frame Buffer in " << sw.ElapsedMs() << " ms";
}

/**
 * @brief Tiled depth assumes that format is in BGRA (colorized depth)
 * 
 * @param views 
 * @param frame_number 
 */
// Note: Rajrup: frame_number != view->frameID, check wserver->send(m_capture_id, s_views) in MultiviewServerPool.cpp. m_capture_id is already incremented.
void NetworkSender::send_tiled_depth(vector<View *> views, int frame_number) 
{
    // LOG(INFO) << "Sending tiled depth - frame_number: "<< frame_number << ", view_number: " << views[0]->frameID;
    if(views[0]->frameID != frame_number)
        LOG(FATAL) << "Depth, Frame number mismatch: " << views[0]->frameID << " " << frame_number;

    if(views.size() != view_count)
        LOG(FATAL) << "Depth, View count mismatch: " << views.size() << " " << view_count;

    // Insert frame to SenderFrameBuffer
    if(send_dframe_buffer == NULL)
        LOG(FATAL) << "Depth, Sender Depth Frame Buffer is NULL";
    
    /**
     * TODO: Rajrup: Skip frame if buffer is full and don't retry
     */
    // if(!send_dframe_buffer->insert_dframe(views, colorized_depth, isSaveView, save_dir))
    //     LOG(ERROR) << "Failed to insert frame to Sender Depth Frame Buffer. Skipping frame: " << frame_number;

    while(!send_dframe_buffer->insert_dframe(views, colorized_depth, isSaveView, save_dir))
    {
        LOG(ERROR) << "Depth, Failed to insert frame " << frame_number << " to Sender Depth BGRA Frame Buffer. Waiting for buffer to clear ...";
        sleep_ms(10);
    }
}

void NetworkSender::send_tiled_depth_yuv16(vector<View *> views, int frame_number) 
{
    StopWatch sw;
    // LOG(INFO) << "Sending tiled depth yuv16 - frame_number: "<< frame_number << ", view_number: " << views[0]->frameID;
    if(views[0]->frameID != frame_number)
        LOG(FATAL) << "Depth YUV16, Frame number mismatch: " << views[0]->frameID << " " << frame_number;

    if(views.size() != view_count)
        LOG(FATAL) << "Depth YUV16, View count mismatch: " << views.size() << " " << view_count;

    // Insert frame to SenderFrameBuffer
    if(send_dframe_buffer == NULL)
        LOG(FATAL) << "Depth YUV16, Sender Depth Frame Buffer is NULL";
    
    /**
     * TODO: Rajrup: Skip frame if buffer is full and don't retry
     */
    // if(!send_dframe_buffer->insert_dframe_yuv16(views, depth_yuv16_cv, isSaveView, save_dir))
    //     LOG(ERROR) << "Failed to insert frame to Sender Depth Frame Buffer. Skipping frame: " << frame_number;
    
    while(!send_dframe_buffer->insert_dframe_yuv16(views, depth_yuv16_cv, depth_yuv16_buf, isSaveView, save_dir))
    {
        LOG(WARNING) << "Depth YUV16, Failed to insert frame " << frame_number << " to Sender Depth YUV16 Frame Buffer. Waiting for buffer to clear ...";
        sleep_ms(10);
    }
    LOG(INFO) << "Depth YUV16, Inserted frame " << frame_number << " to Sender Depth YUV16 Frame Buffer in " << sw.ElapsedMs() << " ms";
}

NetworkSenderPool::NetworkSenderPool(bool isSaveView, std::string save_dir) : isSaveView(isSaveView), save_dir(save_dir), sw_webrtc()
{
    // Only supports tiled mode
    cserver = new NetworkSender("c", isSaveView, save_dir);
    
    // // Rajrup: DEPTH_BGRA
    // dserver = new NetworkSender("d", isSaveView, save_dir);

    // Rajrup: DEPTH_YUV16
    dserver = new NetworkSender("d_yuv16", isSaveView, save_dir);
}

NetworkSenderPool::~NetworkSenderPool()
{
    if(cserver != NULL)
        delete cserver;

    if(dserver != NULL)
        delete dserver;
}

void NetworkSenderPool::send_color_tiled(vector<View *> views, int capture_id) 
{
    cserver->send_tiled_color(views, capture_id);
}

void NetworkSenderPool::send_depth_tiled(vector<View *> views, int capture_id) 
{
    dserver->send_tiled_depth(views, capture_id);
}

void NetworkSenderPool::send_depth_tiled_yuv16(vector<View *> views, int capture_id) 
{    
    // Rajrup: DEPTH_YUV16
    dserver->send_tiled_depth_yuv16(views, capture_id);
}

void NetworkSenderPool::send(int capture_id, vector<View *> s_views) 
{
    // LOG(INFO) << "Server - Frame ID: " << capture_id << ", SEND Time: " << sw_webrtc.Curr() << " ms";
    // Send tiled frames.
    send_pool.push_back(thread(&NetworkSenderPool::send_color_tiled, this, s_views, capture_id)); 

    // Rajrup: DEPTH_BGRA
    // send_pool.push_back(thread(&NetworkSenderPool::send_depth_tiled, this, s_views, capture_id));

    // Rajrup: DEPTH_YUV16
    send_pool.push_back(thread(&NetworkSenderPool::send_depth_tiled_yuv16, this, s_views, capture_id));

    // Rajrup: MASK
    // send_pool.push_back(thread(&WebRTCServerPool::send_mask_tiled, this, s_views, capture_id));

    for (int i=0; i<send_pool.size(); i++) {
        send_pool[i].join();
    }

    send_pool.clear();
}

NetworkSocketServer::NetworkSocketServer(const char* addr, int port, int isSendPtcl): _addr(addr), _port(port), _isSendPtcl(isSendPtcl)
{
    if(_isSendPtcl)
    {
        // Memory for compressed point cloud buffer (used for sending point cloud); Assume PTCL_MAX_POINTS million points.
        // Format: Frame ID (uint32_t), compressed point cloud (uint8_t buffer)
        ptcl_buf2 = new uint8_t[sizeof(uint32_t) + PTCL_MAX_POINTS * (3 * sizeof(float) + 3 * sizeof(uint8_t))];
        ptcl_buf_size2 = 0;
    }
}

void NetworkSocketServer::run()
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
                &NetworkSocketServer::do_session,
                this,
                std::move(socket1)).detach();
            
            std::thread(
                &NetworkSocketServer::do_session2, 
                this,
                std::move(socket2)).detach();

            std::thread(
                &NetworkSocketServer::do_session3, 
                this,
                std::move(socket3)).detach();
            
            if(_isSendPtcl)
                std::thread(
                    &NetworkSocketServer::do_session_ptcl, 
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
void NetworkSocketServer::do_session(tcp::socket socket)
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
            std::string text = beast::buffers_to_string(buffer.data());

            // handling client messages
            if (text == "hello") 
            {
                // send out calibrations
                ws1->binary(true);

                // serialization
                auto text_data = json::array();

                LOG(INFO) << "Sending calibrations to client";

                text_data.push_back("calibration");
                text_data.push_back(calibrations_init2.size()); // device num

                for (auto it : calibrations_init2) {
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

                text_data.push_back(deviceIndices_init2);
                text_data.push_back(deviceSerialNumbers_init2);

                // Sending flags from server to client
                LOG(INFO) << "Sending flags to client";

                // text_data.push_back(FLAGS_seq_name);
                // text_data.push_back(FLAGS_config_file);
                text_data.push_back(FLAGS_start_frame_id);
                text_data.push_back(FLAGS_end_frame_id);
                text_data.push_back(FLAGS_server_fps);
                text_data.push_back(FLAGS_cb);
                text_data.push_back(FLAGS_db);
                text_data.push_back(FLAGS_mm_trace_name);
                text_data.push_back(FLAGS_server_cull);
                
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
                    while(bool x = new_frustum2.load(memory_order_relaxed) == true)
                        sleep_ms(1);

                    camInt.angle = text_data[0];
                    camInt.ratio = text_data[1];
                    camInt.nearD = text_data[2];
                    camInt.farD = text_data[3];

                    for(int i=0; i<3; i++)
                        pos[i] = text_data[4+i];
                    
                    for(int i=0; i<4; i++)
                        quat[i] = text_data[7+i];
                    
                    calc_frustum_from_pos_rot2(pos, rot, quat, camInt, frustum_recv2);
                    frustum_recv2.frameID = text_data[11];
                    frustum_sent_time2 = text_data[12];
                    frustum_recv_time2 = time_now_ms();
                    // cout << endl << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
                    // cout << "Sent Time (in ms): " << frustum_sent_time << endl;
                    // cout << "Received Time (in ms): " << frustum_recv_time << endl;
                    // cout << "Latency (in ms): " << frustum_recv_time - frustum_sent_time << endl;
                    // frustum_recv.print();

                    // new_frustum.store(false);
                    new_frustum2.store(true);
                } 
                catch(std::exception const& e) 
                {
                    LOG(FATAL) << "Error: " << e.what();
                    new_frustum2.store(false);
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
            LOG(FATAL) << "Error: " << se.code().message();
    }
    catch(std::exception const& e)
    {
        LOG(FATAL) << "Error: " << e.what();
    }
}

/**
 * @brief Sends masks to client over websocket.
 * @param socket 
 */
void NetworkSocketServer::do_session2(tcp::socket socket)
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
            std::string text = beast::buffers_to_string(buffer.data());

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
                        while(bool x = new_mask2.load(memory_order_relaxed) == false)
                            sleep(0.001);

                        try
                        {
                            // int original_size = (calibrations_init[0]->intrinsics.width * calibrations_init[0]->intrinsics.height + 7) / 8 * 10;
                            // int compressed_size = ZSTD_compressBound(original_size);

                            // uint8_t *compressed_mask_send = new uint8_t[compressed_size];
                            // ZSTD_compress(compressed_mask_send, compressed_size, mask_send, original_size, 1);
                            
                            // ws2->write(net::buffer(compressed_mask_send, compressed_size));
                            // new_mask.store(false);

                            auto compressed_mask = rle_binary_mask_to_vector(mask_send2, calibrations_init2[0]->intrinsics.width, calibrations_init2[0]->intrinsics.height);
                            ws2->write(net::buffer(compressed_mask, compressed_mask.size()*sizeof(int)));

                            new_mask2.store(false);
                        }
                        catch(std::exception const& e)
                        {
                            // LOG(ERROR) << "[WebRTC] Error on send mask: " << e.what();
                            new_mask2.store(false);
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
            LOG(FATAL) << "Error: " << se.code().message();
    }
    catch(std::exception const& e)
    {
        LOG(FATAL) << "Error: " << e.what();
    }
}

/**
 * @brief Receives client side mahimahi bandwidth estimate (bwe) over websocket.
 * @param socket 
 */
void NetworkSocketServer::do_session3(tcp::socket socket)
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
            std::string text = beast::buffers_to_string(buffer.data());

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
                    std::string text = beast::buffers_to_string(buffer.data());
                    // ws3->binary(true);

                    try
                    {
                        // if(FLAGS_use_client_bitrate)
                        // {
                        //     // parse array from json format
                        //     auto text_data = json::from_msgpack(buffers_begin(buffer.data()), buffers_end(buffer.data()));
                        //     bwe2.store(text_data[0]);
                        //     cbwe2.store(text_data[1]);
                        //     dbwe2.store(text_data[2]);
                        //     LOG(INFO) << "Received bwe from client - bwe: " << bwe2.load() << ", cbwe: " << cbwe2.load() << ", dbwe: " << dbwe2.load();
                        // }

                        if(FLAGS_use_client_bitrate_split)
                        {
                            // parse array from json format
                            auto text_data = json::from_msgpack(buffers_begin(buffer.data()), buffers_end(buffer.data()));
                            d2c_split2.store(text_data[0]);
                            LOG(INFO) << "Received d2c_split from client - d2c_split: " << d2c_split2.load();
                        }
                    }
                    catch(std::exception const& e) 
                    {
                        LOG(FATAL) << "Error: " << e.what();
                    }
                            
                }
            }
        }
    }
    catch(beast::system_error const& se)
    {
        // This indicates that the session was closed
        if(se.code() != websocket::error::closed)
            LOG(FATAL) << "Error: " << se.code().message();
    }
    catch(std::exception const& e)
    {
        LOG(FATAL) << "Error: " << e.what();
    }
}

/**
 * @brief Sends point clouds to client over websocket.
 * @param socket 
 */
void NetworkSocketServer::do_session_ptcl(tcp::socket socket)
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
            std::string text = beast::buffers_to_string(buffer.data());

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
                    while(bool x = new_ptcl2.load(memory_order_relaxed) == false)
                        sleep_ms(1);
                    try
                    {
                        // Send entire compressed ptcl
                        ws_ptcl->write(net::buffer(ptcl_buf2, ptcl_buf_size2));

                        new_ptcl2.store(false);
                        LOG(INFO) << "Sent data. Data Size: " << ptcl_buf_size2;
                    }
                    catch(std::exception const& e)
                    {
                        new_ptcl2.store(false);
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