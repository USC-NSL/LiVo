#include "Client/WebRTCReceiver.h"
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>
#include <boost/random.hpp>
#include "consts.h"
#include "pconsts.h"

#define WEBRTC_JITTER_BUFFER_LATENCY 100

bool check_webrtc_plugins()
{
    GstPlugin *plugin;
    std::string needed[] = {"opus", "vpx", "nice", "webrtc", "dtls", "srtp", "rtp",
                            "rtpmanager", "videotestsrc", "audiotestsrc", "nvcodec"};
    
    for (auto &name : needed)
    {
        plugin = gst_registry_find_plugin(gst_registry_get(), name.c_str());
        if (!plugin)
        {
            std::cerr << name << " plugin not found. Exiting." << std::endl;
            return false;
        }
    }
    return true;
}

int random_uid(int min, int max)
{
    boost::random::mt19937 gen;

    // Seed the generator with the current time
    gen.seed(static_cast<unsigned int>(std::time(0)));
    
    boost::random::uniform_int_distribution<> dist(min, max);
    return dist(gen);
}

WebRTCReceiver::WebRTCReceiver(const std::string &type, int shape[2], int start_frame_id, int end_frame_id, const std::string &out_dir, const std::string &addr, int port, uint32_t fps, ReceiverFrameBuffer *&recv_frame_buffer)
    : type(type), addr(addr), port(port), fps(fps), fps_counter(), qrDecoder(), save_dir(out_dir), recv_frame_buffer(recv_frame_buffer)
{
    uid = random_uid(1, 10000);

    frame_id = start_frame_id;
    this->start_frame_id = start_frame_id;
    this->end_frame_id = end_frame_id;
    prev_frame_id = frame_id-1;
    qr_loss_counter = 0;
    recv_frame_counter = 0;

    frameShape[0] = h = shape[0];
    frameShape[1] = w = shape[1];

    if(type == "c" or type == "c_bgra")
    {
        d = 4;
        frameShape[2] = CHANNEL_DIM::BGRA;
        frameSize = frameShape[0] * frameShape[1] * frameShape[2] * PIXEL_SIZE::BGRA;
    }
    else if(type == "d" or type == "d_bgra")
    {
        d = 4;
        frameShape[2] = CHANNEL_DIM::BGRA;
        frameSize = frameShape[0] * frameShape[1] * frameShape[2] * PIXEL_SIZE::BGRA;
    }
    else if(type == "d_yuv16")
    {
        d = 1;
        frameShape[2] = CHANNEL_DIM::DEPTH;
        frameSize = frameShape[0] * frameShape[1] * frameShape[2] * PIXEL_SIZE::DEPTH;
    }
    else
        LOG(FATAL) << "Unknown type: " << type;

    pipeline_desc = "";
    loop = NULL;
    pipeline = NULL;
    webrtcbin = NULL;
    appsink = NULL;
    ws = NULL;

    bytes_in_interval = 0;
    start_time = g_get_monotonic_time();

    // open the file to log the bitrate
    if(FLAGS_save_cbitrate_file != "")
    {
        fs::path filepath_bitrate{FLAGS_save_cbitrate_file};
        file_cbitrate.open(filepath_bitrate, ios::out | ios::trunc);
        if(!file_cbitrate.is_open())
            LOG(FATAL) << "Failed to open file: " << filepath_bitrate;
        file_cbitrate << "FrameID,Bitrate(in kbps)" << endl;
    }

    if(FLAGS_save_dbitrate_file != "")
    {
        fs::path filepath_bitrate{FLAGS_save_dbitrate_file};
        file_dbitrate.open(filepath_bitrate, ios::out | ios::trunc);
        if(!file_dbitrate.is_open())
            LOG(FATAL) << "Failed to open file: " << filepath_bitrate;
        file_dbitrate << "FrameID,Bitrate(in kbps)" << endl;
    }

    std::string gt_path = FORMAT(PANOPTIC_DATA_PATH << FLAGS_seq_name << "/");
    // bs = new BitrateSplitterClient(gt_path, shape[0], shape[1]);
    analyzer_fc = 0;
}


WebRTCReceiver::~WebRTCReceiver()
{
    if(ws != NULL)
        delete ws;
    if(webrtcbin != NULL)
        gst_object_unref(webrtcbin);
    if(appsink != NULL)
        gst_object_unref(appsink);
    if(pipeline != NULL)
        gst_object_unref(pipeline);
}

void WebRTCReceiver::send_ice_candidate_message(guint mlineindex, gchar *candidate)
{
    LOG(INFO) << "Sending ICE candidate: mlineindex=" << mlineindex << ", candidate=" << candidate;

    object ice_json;
    ice_json["candidate"] = candidate;
    ice_json["sdpMLineIndex"] = mlineindex;

    object msg_json;
    msg_json["type"] = "candidate";
    msg_json["ice"] = ice_json;

    std::string text = serialize(msg_json);
    ws->write(net::buffer(text));

    LOG(INFO) << "ICE candidate sent";
}

void WebRTCReceiver::on_ice_candidate_static(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data)
{
    LOG(INFO) << "ICE candidate generated: mlineindex=" << mlineindex << ", candidate=" << candidate;

    auto *receiver = static_cast<WebRTCReceiver *>(user_data);
    receiver->send_ice_candidate_message(mlineindex, candidate);
}

void WebRTCReceiver::on_offer_created_static(GstPromise *promise, gpointer user_data)
{
    auto receiver = static_cast<WebRTCReceiver *>(user_data);
    auto ws = receiver->ws;
    auto webrtcbin = receiver->webrtcbin;
    
    LOG(INFO) << "Offer created";
    GstWebRTCSessionDescription *offer = NULL;
    const GstStructure *reply = gst_promise_get_reply(promise);
    gst_structure_get(reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, NULL);
    GstPromise *local_promise = gst_promise_new();
    g_signal_emit_by_name(webrtcbin, "set-local-description", offer, local_promise);

    object sdp_json;
    sdp_json["type"] = "offer";
    sdp_json["sdp"] = gst_sdp_message_as_text(offer->sdp);
    std::string text = serialize(sdp_json);
    ws->write(net::buffer(text));

    LOG(INFO) << "Local description set and offer sent: " << text;

    gst_webrtc_session_description_free(offer);
}

void WebRTCReceiver::on_negotiation_needed_static(GstElement *webrtc, gpointer user_data)
{
    LOG(INFO) << "Negotiation needed";

    auto *receiver = static_cast<WebRTCReceiver *>(user_data);
    auto webrtcbin = receiver->webrtcbin;

    GstPromise *promise = gst_promise_new_with_change_func(on_offer_created_static, receiver, NULL);
    g_signal_emit_by_name(webrtcbin, "create-offer", NULL, promise);
}

void WebRTCReceiver::on_set_remote_description_static(GstPromise *promise, gpointer user_data)
{
    LOG(INFO) << "Remote description set";
}

GstFlowReturn WebRTCReceiver::start_feed_static(GstElement *appsink, gpointer user_data)
{
    auto client = static_cast<WebRTCReceiver *>(user_data);
    return client->pull();
}

GstPadProbeReturn WebRTCReceiver::calc_receiver_bitrate_static(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) 
{
    auto receiver = static_cast<WebRTCReceiver *>(user_data);

    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    receiver->bytes_in_interval += gst_buffer_get_size(buffer);

    guint64 current_time = g_get_monotonic_time();
    if ((current_time - receiver->start_time) > G_USEC_PER_SEC) // Calculate every second
    { 
        gdouble bitrate = (receiver->bytes_in_interval * 8.0) / 1000.0; // kbps
        LOG(INFO) << "Type: " << receiver->type << ", Frame ID: " << receiver->frame_id << ", Estimated bitrate: " << bitrate << " kbps";

        // Reset counters
        receiver->bytes_in_interval = 0;
        receiver->start_time = current_time;
    }

    return GST_PAD_PROBE_OK;
}

/**
 * @brief Finds the bitrate of the receiver in last near 1-second window
 * 
 * @param pad 
 * @param info 
 * @param user_data 
 * @return GstPadProbeReturn 
 */
GstPadProbeReturn WebRTCReceiver::calc_receiver_bitrate_window_static(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) 
{
    auto receiver = static_cast<WebRTCReceiver *>(user_data);

    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    gsize frame_size = gst_buffer_get_size(buffer);
    guint64 current_time = g_get_monotonic_time();

    // Store the current frame's data
    receiver->frame_data_queue.push({current_time, frame_size});
    receiver->bytes_in_interval += frame_size;

    // Remove frames outside the 1-second window
    while (!receiver->frame_data_queue.empty() &&
           (current_time - receiver->frame_data_queue.front().timestamp) > G_USEC_PER_SEC) {
        receiver->bytes_in_interval -= receiver->frame_data_queue.front().bytes;
        receiver->frame_data_queue.pop();
    }

    // Calculate the exact time interval based on the oldest timestamp in the queue
    guint64 time_interval_usec = current_time - receiver->frame_data_queue.front().timestamp;
    gdouble time_interval_sec = time_interval_usec / 1e6; // Convert microseconds to seconds

    // Calculate the bitrate based on the exact time interval
    gdouble bitrate = (receiver->bytes_in_interval * 8.0) / (1000.0 * time_interval_sec); // kbps
    LOG(INFO) << "Type: " << receiver->type << ", Frame ID: " << receiver->frame_id + 1
              << ", Estimated bitrate (last ~1 sec): " << bitrate << " kbps";

    if(receiver->type == "c" or receiver->type == "c_bgra")
    {
        if(receiver->file_cbitrate.is_open())
            receiver->file_cbitrate << receiver->frame_id + 1 << "," << bitrate << endl;
    }
    else if(receiver->type == "d" or receiver->type == "d_bgra" or receiver->type == "d_yuv16")
    {
        if(receiver->file_dbitrate.is_open())
            receiver->file_dbitrate << receiver->frame_id + 1 << "," << bitrate << endl;
    }

    return GST_PAD_PROBE_OK;
}


void WebRTCReceiver::probe_stats()
{
    return;
}

void WebRTCReceiver::handle_frame_info(const std::string &msg)
{
    LOG(INFO) << "Received frame info: " << msg;
    return;
}

void WebRTCReceiver::handle_sdp(const std::string &msg)
{
    if(ws == NULL)
        LOG(FATAL) << "Websocket is not initialized";

    if(webrtcbin == NULL)
        LOG(FATAL) << "WebRTCbin is not initialized";
    
    value jv = parse(msg);
    object obj = jv.as_object();
    std::string type = obj["type"].as_string().c_str();

    if(type == "answer")
    {
        LOG(INFO) << "Received answer: " << msg;

        std::string sdp = obj["sdp"].as_string().c_str();

        GstSDPMessage *sdp_message;
        gst_sdp_message_new_from_text(sdp.c_str(), &sdp_message);
        GstWebRTCSessionDescription *answer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp_message);
        GstPromise *promise = gst_promise_new_with_change_func(on_set_remote_description_static, this, NULL);
        g_signal_emit_by_name(webrtcbin, "set-remote-description", answer, promise);
        gst_webrtc_session_description_free(answer);

        LOG(INFO) << "Setting remote description";
    }
    else if(type == "candidate")
    {
        LOG(INFO) << "Received ICE candidate: " << msg;

        object ice = obj["ice"].as_object();
        std::string candidate = ice["candidate"].as_string().c_str();
        guint sdpMLineIndex = ice["sdpMLineIndex"].as_int64();
        g_signal_emit_by_name(webrtcbin, "add-ice-candidate", sdpMLineIndex, candidate.c_str());

        LOG(INFO) << "Added ICE candidate";
    }
    else
    {
        LOG(FATAL) << "Unknown message type: " << msg;
    }

}

void WebRTCReceiver::on_eos()
{
    LOG(INFO) << "Got eos";
    return;
}

GstFlowReturn WebRTCReceiver::on_recvd_sample_static(GstElement *appsink, gpointer user_data)
{
    auto server = static_cast<WebRTCReceiver *>(user_data);
    return server->on_recvd_sample(appsink, user_data);
}

GstFlowReturn WebRTCReceiver::on_recvd_sample(GstElement *appsink, gpointer user_data)
{
    StopWatch sw;
    GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 10 * GST_MSECOND);
    if(sample == NULL)
    {
        LOG(ERROR) << "Failed to get sample from appsink";
        return GST_FLOW_ERROR;
    }

    /**
     * @brief Calculate bitrate split at the receiver, but only call it in 1 thread. I chose the color thread.
     * Another option is to call it in MultiViewClientPoolNew.cpp, but that requires BitrateSplitterClient to be a member of MultiViewClientPoolNew.
     */
    // if(type == "c" || type == "c_bgra")
    // {
    //     bs->split_bitrate();
    // }

    analyzer_fc++;
    // if(analyzer_fc % 100 != 0)
    // {
    //     gst_sample_unref(sample);
    //     return GST_FLOW_OK;
    // }

    GstMapInfo map;
    GstBuffer* gst_buffer = gst_sample_get_buffer(sample);
    if (!gst_buffer_map(gst_buffer, &map, GST_MAP_READ)) 
    {
        LOG(ERROR) << "Failed to map buffer from sample";
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    uint8_t *data = map.data;
    int size = map.size;

    if(type == "c" || type == "c_bgra")
    {
        cv::Mat res(frameShape[0], frameShape[1], CV_8UC(CHANNEL_DIM::BGRA), data);
        int frameID = decode_qr_code(res);

        if(frameID != -1)
        {
            // float rmse = bs->calc_color_rmse(res, frameID);
            // if(rmse < 0.0f)
            //     LOG(ERROR) << "Type: " << type << ", Frame ID: " << frameID << " failed to calculate RMSE";
            // else
            //     LOG(INFO) << "Type: " << type << ", Frame ID: " << frameID << ", RMSE: " << rmse;
            LOG(INFO) << "Type: " << type << ", Frame ID: " << frameID << " received for analysis";
        }
        else
        {
            // cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << analyzer_fc << "_color_analyze.png"), res);
            LOG(ERROR) << "Type: " << type << ", Failed to detect QR code in analyzer";
        }
    }
    else if(type == "d_yuv16")
    {
        cv::Mat channel_y = cv::Mat(frameShape[0], frameShape[1], CV_16UC1, data);
        cv::Mat channel_u = cv::Mat(frameShape[0], frameShape[1], CV_16UC1, data + channel_y.total() * channel_y.elemSize());
        int frameID = decode_qr_code_yuv(channel_u);

        if(frameID != -1)
        {
            // float rmse = bs->calc_depth_rmse(channel_y, frameID);
            // if(rmse < 0.0f)
            //     LOG(ERROR) << "Type: " << type << ", Frame ID: " << frameID << " failed to calculate RMSE";
            // else
            //     LOG(INFO) << "Type: " << type << ", Frame ID: " << frameID << ", RMSE: " << rmse;
            LOG(INFO) << "Type: " << type << ", Frame ID: " << frameID << " received for analysis";
        }
        else
        {
            // cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << analyzer_fc << "_depth_analyze_y.png"), channel_y);
            // cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << analyzer_fc << "_depth_analyze_u.png"), channel_u);
            LOG(ERROR) << "Type: " << type << ", Failed to detect QR code in analyzer";
        }
    }
    else
        LOG(FATAL) << "Invalid type: " << type;
    
    gst_buffer_unmap(gst_buffer, &map);
    gst_sample_unref(sample);

    LOG(INFO) << "Type: " << type << ", Frame analysis time: " << sw.ElapsedMs() << " ms";

    return GST_FLOW_OK;
}

void WebRTCReceiver::configure_appsink_distortion_analyzer()
{
    GstElement *distortion_analyzer = gst_bin_get_by_name(GST_BIN(pipeline), "distortion_analyzer");
    if (!distortion_analyzer) 
    {
        LOG(ERROR) << "Failed to get appsink element.";
        return;
    }

    // Configure appsink properties
    // g_object_set(distortion_analyzer, "emit-signals", TRUE, "sync", FALSE, NULL);
    g_signal_connect(distortion_analyzer, "new-sample", G_CALLBACK(WebRTCReceiver::on_recvd_sample_static), this);
    g_object_set(distortion_analyzer, "buffer-list", true, NULL);
    g_object_set(distortion_analyzer, "drop", true, NULL);
}

void WebRTCReceiver::configure_pipeline_components()
{
    GError *g_error = NULL;
    pipeline = gst_parse_launch(pipeline_desc.c_str(), &g_error);

    if(g_error)
	{
		g_printerr("Failed to parse launch: %s\n", g_error->message);
		g_error_free(g_error);
        end_pipeline();
		return;
	}
    
    if (!pipeline)
	{
		g_printerr("Pipeline could not be created.\n");
        end_pipeline();
		return;
	}

    webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "recvonly");
    GstElement *rtpbin = gst_bin_get_by_name(GST_BIN(pipeline), "rtpbin");
    GstElement *videodepay = gst_bin_get_by_name(GST_BIN(pipeline), "videodepay");
    GstElement *decoder = gst_bin_get_by_name(GST_BIN(pipeline), "viddec");

    if(!webrtcbin || !rtpbin || !videodepay)
    {
        g_printerr("Failed to get webrtcbin or rtpbin or videodepay.\n");
        end_pipeline();
        return;
    }

    // Calculate encoded bitrate in the receiver by probing the source pad (output) of the rtph265depay
    GstPad *videodepay_src_pad = gst_element_get_static_pad(videodepay, "src");
    if(!videodepay_src_pad)
    {
        LOG(ERROR) << "Failed to get videodepay src pad";
        end_pipeline();
        return;
    }
    // gst_pad_add_probe(videodepay_src_pad, GST_PAD_PROBE_TYPE_BUFFER, calc_receiver_bitrate_static, this, nullptr);
    // gst_pad_add_probe(videodepay_src_pad, GST_PAD_PROBE_TYPE_BUFFER, calc_receiver_bitrate_window_static, this, nullptr);

    // GstPad *decoder_sink_pad = gst_element_get_static_pad(decoder, "sink");
    // if(!decoder_sink_pad)
    // {
    //     LOG(ERROR) << "Failed to get decoder sink pad";
    //     end_pipeline();
    //     return;
    // }
    // // gst_pad_add_probe(decoder_sink_pad, GST_PAD_PROBE_TYPE_BUFFER, calc_receiver_bitrate_static, this, nullptr);
    // gst_pad_add_probe(decoder_sink_pad, GST_PAD_PROBE_TYPE_BUFFER, calc_receiver_bitrate_window_static, this, nullptr);

    // Set webrtcbin properties
    g_object_set(webrtcbin, "latency", WEBRTC_JITTER_BUFFER_LATENCY, NULL);

    g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(on_negotiation_needed_static), this);
    g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(on_ice_candidate_static), this);

    // Add transceiver to the webrtcbin
    GstWebRTCRTPTransceiver *transceiver = NULL;
    GstWebRTCRTPTransceiverDirection direction_v = GST_WEBRTC_RTP_TRANSCEIVER_DIRECTION_RECVONLY;
    // GstCaps *caps_v = gst_caps_from_string("application/x-rtp,media=video,encoding-name=(string)H265, clock-rate=90000,payload=96, extmap-1=http://www.ietf.org/id/draft-holmer-rmcat-transport-wide-cc-extensions-01");
    GstCaps *caps_v = gst_caps_from_string("application/x-rtp,media=video,encoding-name=(string)H265, clock-rate=90000,payload=96");
    g_signal_emit_by_name(webrtcbin, "add-transceiver", direction_v, caps_v, &transceiver);
    gst_caps_unref(caps_v);

    if (!transceiver) {
        g_printerr("Failed to add transceiver.\n");
        return;
    }

    // Get Transceivers
    GArray *transceivers = NULL;
    g_signal_emit_by_name(webrtcbin, "get-transceivers", &transceivers);
    LOG(INFO) << "Type: " << type << ", #Transceivers: " << transceivers->len;

    // Set Retransmission
    g_object_set(rtpbin, "do-retransmission", true, NULL);
    for(int i=0; i<transceivers->len; i++)
    {
        GstWebRTCRTPTransceiver* transceiver = g_array_index(transceivers, GstWebRTCRTPTransceiver*, i);
        g_object_set(transceiver, "do-nack", true, NULL);
    }

    set_appsink();
}

void WebRTCReceiver::set_color_pipeline()
{
    // NVCODEC: H265 for color
    pipeline_desc = "webrtcbin name=recvonly bundle-policy=max-bundle stun-server=" + std::string(STUN_SERVER) + " ! "
                    "rtph265depay name=videodepay ! queue ! h265parse ! nvh265dec name=viddec ! queue ! videoconvert ! "
                    "video/x-raw, format=BGRA, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
                    "queue ! appsink name=sink emit-signals=true";
    
    LOG(INFO) << "Color pipeline:\n" << pipeline_desc;
    configure_pipeline_components();
}

void WebRTCReceiver::set_color_pipeline_with_distortion_analysis()
{
    // pipeline_desc = "webrtcbin name=recvonly bundle-policy=max-bundle stun-server=" + std::string(STUN_SERVER) + " ! "
    //                 "rtph265depay name=videodepay ! queue ! h265parse ! nvh265dec name=viddec ! queue ! videoconvert ! "
    //                 "video/x-raw, format=BGRA, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
    //                 " tee name=t ! queue ! appsink name=sink emit-signals=true "
    //                 "t. ! queue ! appsink name=distortion_analyzer emit-signals=true";
    /**
     * @brief tee element doesn't do buffer copy, rather performs zero-copy. This results in the same buffer being passed to both the sink and the distortion_analyzer.
     * Removing qr code in the sink buffer will also remove it in the distortion_analyzer buffer.
     * This results in qr code miss. The pipeline below (I guess) might do buffer copy and hence the qr code is detected in the distortion_analyzer.
     */
    pipeline_desc = "webrtcbin name=recvonly bundle-policy=max-bundle stun-server=" + std::string(STUN_SERVER) + " ! "
                    "rtph265depay name=videodepay ! queue ! h265parse ! nvh265dec name=viddec ! "
                    "tee name=t ! queue ! videoconvert ! video/x-raw, format=BGRA, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
                    "queue ! appsink name=sink emit-signals=true "
                    "t. ! queue ! videoconvert ! video/x-raw, format=BGRA, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
                    "queue ! appsink name=distortion_analyzer emit-signals=true";
    
    // For ablation study for parallel decoding
    // pipeline_desc = "webrtcbin name=recvonly bundle-policy=max-bundle stun-server=" + std::string(STUN_SERVER) + " ! "
    //                 "rtph265depay name=videodepay ! queue ! "
    //                 "tee name = t ! queue !  h265parse ! nvh265dec name=viddec ! "
    //                 "queue ! videoconvert ! video/x-raw, format=BGRA, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
    //                 "queue ! appsink name=sink emit-signals=true "
    //                 "t. ! queue !  h265parse ! nvh265dec name=viddec2 ! "
    //                 "queue ! videoconvert ! video/x-raw, format=BGRA, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
    //                 "queue ! appsink name=distortion_analyzer emit-signals=true";

    LOG(INFO) << "Color pipeline:\n" << pipeline_desc;
    configure_pipeline_components();
    configure_appsink_distortion_analyzer();
}

void WebRTCReceiver::set_depth_pipeline_bgra()
{
    return;
}

void WebRTCReceiver::set_depth_pipeline_yuv16()
{
    // NVCODEC: H265 for depth (YUV16)
    pipeline_desc = "webrtcbin name=recvonly bundle-policy=max-bundle stun-server=" + std::string(STUN_SERVER) + " ! "
                    "rtph265depay name=videodepay ! queue ! h265parse ! nvh265dec name=viddec ! queue ! videoconvert ! "
                    "video/x-raw, format=Y444_16LE, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
                    "queue ! appsink name=sink emit-signals=true";
    
    LOG(INFO) << "Depth YUV16 pipeline:\n" << pipeline_desc;
    configure_pipeline_components();
}

void WebRTCReceiver::set_depth_pipeline_yuv16_with_distortion_analysis()
{
    // NVCODEC: H265 for depth (YUV16)
    // pipeline_desc = "webrtcbin name=recvonly bundle-policy=max-bundle stun-server=" + std::string(STUN_SERVER) + " ! "
    //                 "rtph265depay name=videodepay ! queue ! h265parse ! nvh265dec name=viddec ! queue ! videoconvert ! "
    //                 "video/x-raw, format=Y444_16LE, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
    //                 " tee name=t ! queue ! appsink name=sink emit-signals=true "
    //                 "t. ! queue ! appsink name=distortion_analyzer emit-signals=true";
    /**
     * @brief Check the comment in set_color_pipeline_with_distortion_analysis() for the reason of using the below pipeline.
     */
    pipeline_desc = "webrtcbin name=recvonly bundle-policy=max-bundle stun-server=" + std::string(STUN_SERVER) + " ! "
                    "rtph265depay name=videodepay ! queue ! h265parse ! nvh265dec name=viddec ! "
                    "tee name=t ! queue ! videoconvert ! video/x-raw, format=Y444_16LE, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
                    "queue ! appsink name=sink emit-signals=true "
                    "t. ! queue ! videoconvert ! video/x-raw, format=Y444_16LE, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
                    "queue ! appsink name=distortion_analyzer emit-signals=true";

    // For ablation study for parallel decoding
    // pipeline_desc = "webrtcbin name=recvonly bundle-policy=max-bundle stun-server=" + std::string(STUN_SERVER) + " ! "
    //                 "rtph265depay name=videodepay ! queue ! "
    //                 "tee name = t ! queue !  h265parse ! nvh265dec name=viddec ! "
    //                 "queue ! videoconvert ! video/x-raw, format=Y444_16LE, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
    //                 "queue ! appsink name=sink emit-signals=true "
    //                 "t. ! queue !  h265parse ! nvh265dec name=viddec2 ! "
    //                 "queue ! videoconvert ! video/x-raw, format=Y444_16LE, width=" + std::to_string(frameShape[1]) + ", height=" + std::to_string(frameShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
    //                 "queue ! appsink name=distortion_analyzer emit-signals=true";
    
    LOG(INFO) << "Depth YUV16 pipeline:\n" << pipeline_desc;
    configure_pipeline_components();
    configure_appsink_distortion_analyzer();
}

void WebRTCReceiver::set_appsink()
{
     if(pipeline == NULL)
    {
        g_printerr("Pipeline is not initialized.\n");
        end_pipeline();
        return;
    }

    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    if(!appsink)
    {
        g_printerr("Failed to get appsink.\n");
        end_pipeline();
        return;
    }

    g_signal_connect(appsink, "new-sample", G_CALLBACK(start_feed_static), this);
    g_object_set(appsink, "buffer-list", true, NULL);
    g_object_set(appsink, "drop", true, NULL);
}

void WebRTCReceiver::start_pipeline()
{
    if(type == "c" || type == "c_bgra")
        set_color_pipeline();
        // set_color_pipeline_with_distortion_analysis();
    else if(type == "d" || type == "d_bgra")
        set_depth_pipeline_bgra();
    else if(type == "d_yuv16")
        set_depth_pipeline_yuv16();
        // set_depth_pipeline_yuv16_with_distortion_analysis();

    if(!pipeline)
    {
        g_printerr("Pipeline is not initialized.\n");
        return;
    }

    if(!webrtcbin)
    {
        g_printerr("WebRTCbin is not initialized.\n");
        return;
    }

    // Start the pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);

	if (ret == GST_STATE_CHANGE_FAILURE)
	{
		g_printerr("Unable to set the pipeline to the playing state.\n");
		gst_object_unref(pipeline);
		gst_object_unref(webrtcbin);
		return;
	}

    LOG(INFO) << "Type: " << type << ", Pipeline State: Started";
}

void WebRTCReceiver::end_pipeline()
{
    if(appsink != NULL)
        gst_object_unref(appsink);
    appsink = NULL;

    if(webrtcbin != NULL)
        gst_object_unref(webrtcbin);
    webrtcbin = NULL;

    if(pipeline != NULL)
    {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }
    pipeline = NULL;
    
    LOG(INFO) << "Pipeline State: Ended";
    return;
}

int WebRTCReceiver::decode_qr_code(const cv::Mat &image, int offset_x, int offset_y, int roi_width, int roi_height, int border, int view_number)
{
    int start_x = (view_number % 5) * (frameShape[1] / 5);
    int start_y = (view_number / 5) * (frameShape[0] / 2);

    cv::Mat img_blue;
	cv::extractChannel(image, img_blue, 0);    // Only check for blue channel. If required, we can test for all channels, but its time consuming.

    // Extract the region of interest (ROI) based on the offset and size
	cv::Mat qrcode_img = img_blue(cv::Range(start_y + offset_y, start_y + offset_y + roi_height), cv::Range(start_x + offset_x, start_x + offset_x + roi_width));

    // QR Code border points
    std::vector<cv::Point> corners;
	corners.push_back(cv::Point(border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, qrcode_img.rows - border));
	corners.push_back(cv::Point(border, qrcode_img.rows - border));

    // Detect QR code in the ROI
    std::string decode_info = qrDecoder.decode(qrcode_img, corners);

    if(decode_info.empty())
    {
        LOG(ERROR) << "Failed to decode QR code";
        return -1;
    }

    return std::stoi(decode_info);
}

int WebRTCReceiver::decode_qr_code_yuv(const cv::Mat &image, int offset_x, int offset_y, int roi_width, int roi_height, int border, int view_number)
{
    int start_x = (view_number % 5) * (frameShape[1] / 5);
    int start_y = (view_number / 5) * (frameShape[0] / 2);

	cv::Mat qrcode_img = cv::Mat::zeros(roi_height, roi_width, CV_8UC1);

    for(int i=0; i<qrcode_img.rows; i++)
    {
        for(int j=0; j<qrcode_img.cols; j++)
        {
            float pixel = (image.at<uint16_t>(start_y + offset_y + i, start_x + offset_x + j) * 255.0F) / 65535.0F;
            qrcode_img.at<uint8_t>(i,j) = uint8_t(pixel);
        }
    }

	// Detect qr code in u channel
	std::vector<cv::Point> corners;
	corners.push_back(cv::Point(border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, qrcode_img.rows - border));
	corners.push_back(cv::Point(border, qrcode_img.rows - border));

	std::string decode_info = qrDecoder.decode(qrcode_img, corners);
    
    if(decode_info.empty())
    {
        LOG(ERROR) << "Failed to decode QR code";
        return -1;
    }

    return std::stoi(decode_info);  
}

cv::Mat WebRTCReceiver::color_to_opencv(const uint8_t *data, int width, int height, bool withAlpha)
{
    cv::Mat image;
    return image;
}

cv::Mat WebRTCReceiver::depth_to_opencv(const uint8_t *data, int width, int height)
{
    cv::Mat image;
    return image;
}

/**
 * @brief Only emit GST_FLOW_ERROR when the pipeline should be stopped
 * @return GstFlowReturn GST_FLOW_OK if successful, GST_FLOW_ERROR otherwise
 */
GstFlowReturn WebRTCReceiver::pull()
{
    StopWatch sw;
    sw.Restart();

    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 10 * GST_MSECOND);
    // LOG(INFO) << "Type: " << type << ", Pull from webrtc";
    if(sample == NULL)
    {
        LOG(ERROR) << "Failed to get sample from appsink";
        return GST_FLOW_ERROR;
    }

    GstMapInfo map;
    GstBuffer* gst_buffer = gst_sample_get_buffer(sample);
    if (!gst_buffer_map(gst_buffer, &map, GST_MAP_READ)) 
    {
        LOG(ERROR) << "Failed to map buffer from sample";
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }

    // Pointing to the data, not copying
    uint8_t *data = map.data;
    int size = map.size;

    if(type == "c" || type == "c_bgra")
    {
        // LOG(INFO) << "Type: " << type << ", Inserting, Size: " << size << ", Width: " << w << ", Height: " << h;
        if(!recv_frame_buffer->insert_cframe(data, size, w, h, frame_id))
        {
            LOG(ERROR) << "Color, Failed to insert color frame to circular buffer";
            // gst_buffer_unmap(gst_buffer, &map);
            // gst_sample_unref(sample);
            // return GST_FLOW_OK;
        }
    }
    else if(type == "d" || type == "d_bgra")
    {
        if(!recv_frame_buffer->insert_dframe(data, size, w, h, frame_id))
        {
            LOG(ERROR) << "Depth, Failed to insert depth frame to circular buffer";
            // gst_buffer_unmap(gst_buffer, &map);
            // gst_sample_unref(sample);
            // return GST_FLOW_OK;
        }
    }
    else if(type == "d_yuv16")
    {
        // LOG(INFO) << "Type: " << type << ", Inserting, Size: " << size << ", Width: " << w << ", Height: " << h;
        if(!recv_frame_buffer->insert_dframe_yuv16(data, size, w, h, frame_id))
        {
            LOG(ERROR) << "Depth YUV16, Failed to insert depth frame to circular buffer";
            // gst_buffer_unmap(gst_buffer, &map);
            // gst_sample_unref(sample);
            // return GST_FLOW_OK;
        }
    }
    else
        LOG(FATAL) << "Invalid type: " << type;

    if(frame_id == -1) // Couldn't detect QR code
    {
        frame_id = prev_frame_id + 1; // Interpolate frame id
        qr_loss_counter++;
    }
    prev_frame_id = frame_id;
    LOG(INFO) << "Type: " << type << ", WebRTC Received Frame ID: " << frame_id;

    // int frame_id_detected = -1;
    // if(type != "d_yuv16")
    // {
    //     cv::Mat res(h, w, CV_8UC(d), data);
    //     frame_id_detected = decode_qr_code(res);

    //     if (frame_id_detected == -1) 
    //     {
    //         LOG(ERROR) << "Type: " << type << ", Invalid QR code";
    //         qr_loss_counter++;

    //         std::string out_img_path;
    //         if(type == "c" || type == "c_bgra")
    //             out_img_path = FORMAT(save_dir << "/" << recv_frame_counter << "_color_gst.png");
    //         else if(type == "d" || type == "d_bgra")
    //             out_img_path = FORMAT(save_dir << "/" << recv_frame_counter << "_depth_gst.png");
    //         cv::imwrite(out_img_path, res);
    //         LOG(INFO) << "Type: " << type << ", Saved Image to: " << out_img_path;
    //     } 
    //     else 
    //     {
    //         if (frame_id_detected != frame_id) 
    //         {
    //             LOG(WARNING) << "Type: " << type << ", Received Frame: " << frame_id_detected << ", Required Frame: " << frame_id;
    //             frame_id = frame_id_detected;
    //         }
    //         if (frame_id_detected <= prev_frame_id) 
    //         {
    //             LOG(WARNING) << "Type: " << type << ", Received Frame: " << frame_id_detected << ", Previous Frame: " << prev_frame_id;
    //         }
    //         prev_frame_id = frame_id_detected;
    //         LOG(INFO) << "Type: " << type << ", Received Frame ID: " << frame_id_detected;
    //     }
    // }
    // else if (type == "d_yuv16") 
    // {   
    //     cv::Mat channel_y = cv::Mat(frameShape[0], frameShape[1], CV_16UC1, data);
    //     cv::Mat channel_u = cv::Mat(frameShape[0], frameShape[1], CV_16UC1, data + channel_y.total() * channel_y.elemSize());

    //     // cv::Mat res(d, h, CV_16UC1, data);
    //     // cv::Mat u_channel = res.row(1);
    //     frame_id_detected = decode_qr_code_yuv(channel_u);

    //     if (frame_id_detected == -1) 
    //     {
    //         LOG(ERROR) << "Type: " << type << ", Invalid QR code";
    //         qr_loss_counter++;
    //     }
    //     else 
    //     {
    //         if (frame_id_detected != frame_id) 
    //         {
    //             LOG(WARNING) << "Type: " << type << ", Received Frame: " << frame_id_detected << ", Required Frame: " << frame_id;
    //             frame_id = frame_id_detected;
    //         }
    //         if (frame_id_detected <= prev_frame_id) 
    //         {
    //             LOG(WARNING) << "Type: " << type << ", Received Frame: " << frame_id_detected << ", Previous Frame: " << prev_frame_id;
    //         }
    //         prev_frame_id = frame_id_detected;
    //         LOG(INFO) << "Type: " << type << ", Received Frame ID: " << frame_id_detected;
    //     }
    // }

    // if (frame_id % 100 == 0) 
    // {
    //     int lost_frames = (frame_id - start_frame_id + 1) - (recv_frame_counter + 1);
    //     LOG(INFO) << "Type: " << type << ", Received Frame: " << frame_id_detected << ", Received Frame Counter: " << recv_frame_counter + 1 << ", Lost Frame: " << lost_frames << ", QR loss: " << qr_loss_counter << ", Time Taken: " << sw.ElapsedMs() << " ms";
    // }

    float fps_pull = fps_counter.fps_counter();
    if(recv_frame_counter % 10 == 0)
        LOG(INFO) << "Type: " << type << ", FPS: " << fps_pull;
    recv_frame_counter++;

    gst_buffer_unmap(gst_buffer, &map);
    // gst_buffer_unref(gst_buffer);
    gst_sample_unref(sample);

    if(frame_id >= end_frame_id)
    {
        LOG(INFO) << "Type: " << type << ", Received all frames. Ending pipeline";

        if(file_cbitrate.is_open())
            file_cbitrate.close();
        
        if(file_dbitrate.is_open())
            file_dbitrate.close();
        
        end_pipeline();
        return GST_FLOW_EOS;
    }
    return GST_FLOW_OK;
}

void WebRTCReceiver::ws_handler(tcp::socket socket)
{
    try
    {
        this->ws = new websocket::stream<tcp::socket>(std::move(socket));
        ws->handshake(addr, "/");
        LOG(INFO) << "Type: " << type << ", WebSocket connection established with server: " << addr << ":" << port;
        ws->write(net::buffer(FORMAT("HELLO " << uid)));

        // start_pipeline();

        for (;;)
        {
            beast::flat_buffer buffer;
            LOG(INFO) << "Type: " << type << ", Waiting for message...";
            ws->read(buffer);
            auto msg = beast::buffers_to_string(buffer.data());
            std::vector<std::string> data;
            boost::split(data, msg, boost::is_any_of(" "));

            std::size_t found = msg.find("frame_info");

            if(data[0] == "HELLO_OK")
            {
                int shape[3] = {std::stoi(data[1]), std::stoi(data[2]), std::stoi(data[3])};

                if(frameShape[0] != shape[0] || frameShape[1] != shape[1] || frameShape[2] != shape[2])
                {
                    LOG(ERROR) << "Type: " << type << ", Received image shape does not match with the expected shape";
                    LOG(ERROR) << "Type: " << type << ", Expected shape: " << frameShape[0] << " " << frameShape[1] << " " << frameShape[2];
                    LOG(ERROR) << "Type: " << type << ", Received shape: " << shape[0] << " " << shape[1] << " " << shape[2];
                    LOG(ERROR) << "Type: " << type << ", Ending pipeline";
                    end_pipeline();
                    return;
                }

                LOG(INFO) << "Type: " << type << ", Received message - HELLO_OK and image shape: " << shape[0] << " " << shape[1] << " " << shape[2];

                int imgSize = 0;
                if(type != "d_yuv16")
                    imgSize = shape[0] * shape[1] * shape[2] * PIXEL_SIZE::BGRA;
                else
                    imgSize = shape[0] * shape[1] * shape[2] * PIXEL_SIZE::DEPTH;
                
                if(frameSize != imgSize)
                {
                    LOG(ERROR) << "Type: " << type << ", Received image size does not match with the expected size";
                    LOG(ERROR) << "Type: " << type << ", Expected size: " << frameSize;
                    LOG(ERROR) << "Type: " << type << ", Received size: " << imgSize;
                    LOG(ERROR) << "Type: " << type << ", Ending pipeline";
                    end_pipeline();
                    return;
                }

                ws->write(net::buffer(std::string("OFFER_REQUEST")));
                // start_pipeline_dummy();
                start_pipeline();

                // Create a task to probe stats
                // std::thread{&WebRTCReceiver::probe_stats, this}.detach();
            }
            else if(data[0] == "ERROR")
            {
                LOG(INFO) << "Type: " << type << ", Received server error: " << msg;
                LOG(ERROR) << "Type: " << type << ", Ending pipeline";
                end_pipeline();
            }
            else if(found!=std::string::npos)
            {
                handle_frame_info(msg);
            }
            else
            {
                // Handle SDP message
                LOG(INFO) << "Type: " << type << ", Handle SDP message";
                handle_sdp(msg);
            }
        }
    }
    catch (beast::system_error const& se)
	{
		if (se.code() == websocket::error::closed)
        {
            LOG(INFO) << "Type: " << type << ", WebSocket connection closed by server.";
        }
        else if (se.code() == net::error::eof)
        {
            LOG(WARNING) << "Type: " << type << ", Connection closed unexpectedly (EOF) by server.";
        }
        else
        {
            LOG(ERROR) << "Type: " << type << ", WebSocket error: " << se.code().message();
        }
	}
	catch (std::exception const& e)
	{
		LOG(ERROR) << "Type: " << type << ", Exception: " << e.what();
	}
}

void WebRTCReceiver::connect()
{
    try
    {
        net::io_context ioc;
        tcp::resolver resolver(ioc);
        auto const results = resolver.resolve(addr, std::to_string(port));

        websocket::stream<tcp::socket> ws{ioc};
        net::connect(ws.next_layer(), results.begin(), results.end());

        // std::thread t(
        //                 &WebRTCReceiver::ws_handler, 
        //                 this, 
        //                 std::move(ws.next_layer())
        //             );
        // ioc.run();

        // t.join();

        ws_handler(std::move(ws.next_layer()));
    }
    catch (std::exception const& e)
    {
        LOG(FATAL) << "Type: " << type << ", Exception: " << e.what();
    }
}