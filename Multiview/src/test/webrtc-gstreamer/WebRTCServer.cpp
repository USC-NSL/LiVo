#include "WebRTCServer.h"
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>

#define CHANNEL_BGRA 4
#define CHANNEL_DEPTH 3

#define PIXEL_SIZE_BGRA 1
#define PIXEL_SIZE_DEPTH 2

WebRTCServer::WebRTCServer(const std::string &type, int shape[2], const std::string seq_name, int start_frame_id, int end_frame_id, const std::string &in_dir, const std::string &out_dir, int fps, uint32_t color_bitrate, uint32_t depth_bitrate, const std::string &addr, int port)
    : type(type), seq_name(seq_name), in_dir(in_dir), out_dir(out_dir), fps(fps), color_bitrate(color_bitrate), depth_bitrate(depth_bitrate), addr(addr), port(port), fps_limiter(fps), fps_counter(), qrDecoder()
{
    color_dir = FORMAT(in_dir << "/" << seq_name << "/tiled/color/");
    depth_dir = FORMAT(in_dir << "/" << seq_name << "/tiled/depth/");

    is_push_buffer_allowed = false;
    frame_id = start_frame_id;
    this->start_frame_id = start_frame_id;
    this->end_frame_id = end_frame_id;

    imgShape[0] = shape[0];
    imgShape[1] = shape[1];
    if(type == "c" or type == "c_bgra")
    {
        imgShape[2] = 4;
        imgSize = imgShape[0] * imgShape[1] * imgShape[2] * PIXEL_SIZE_BGRA;
    }
    else if(type == "d" or type == "d_bgra")
    {
        imgShape[2] = 4;
        imgSize = imgShape[0] * imgShape[1] * imgShape[2] * PIXEL_SIZE_BGRA;
    }
    else if(type == "d_yuv16")
    {
        imgShape[2] = 3;
        imgSize = imgShape[0] * imgShape[1] * imgShape[2] * PIXEL_SIZE_DEPTH;
    }
    else
        LOG(FATAL) << "Unknown type: " << type;

    loop = NULL;
    pipeline = NULL;
    webrtcbin = NULL;
    appsrc = NULL;
    ws = NULL;

    bytes_in_interval = 0;
    start_time = g_get_monotonic_time();
}

WebRTCServer::~WebRTCServer()
{
    if(ws != NULL)
        delete ws;
    if(webrtcbin != NULL)
        gst_object_unref(webrtcbin);
    if(appsrc != NULL)
        gst_object_unref(appsrc);
    if(pipeline != NULL)
        gst_object_unref(pipeline);
    
    for(int i=0; i<handler_pools.size(); i++)
    {
        if(handler_pools[i].joinable())
            handler_pools[i].join();
    }

    LOG(INFO) << "WebRTCServer destroyed";
}

void WebRTCServer::on_answer_created(GstPromise *promise, gpointer user_data)
{
	LOG(INFO) << "Answer created";
	GstWebRTCSessionDescription *answer = NULL;
	const GstStructure *reply = gst_promise_get_reply(promise);
	gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, NULL);
	GstPromise *local_promise = gst_promise_new();
	g_signal_emit_by_name(webrtcbin, "set-local-description", answer, local_promise);

	object sdp_json;
	sdp_json["type"] = "answer";
	sdp_json["sdp"] = gst_sdp_message_as_text(answer->sdp);
	std::string text = serialize(sdp_json);
	ws->write(net::buffer(text));

	LOG(INFO) << "Local description set and answer sent: " << text;

	gst_webrtc_session_description_free(answer);
}

void WebRTCServer::on_answer_created_static(GstPromise *promise, gpointer user_data)
{   
    auto server = static_cast<WebRTCServer *>(user_data); // Use this to access member functions
    auto webrtcbin = server->webrtcbin;
    auto ws = server->ws;

	LOG(INFO) << "Answer created";
	GstWebRTCSessionDescription *answer = NULL;
	const GstStructure *reply = gst_promise_get_reply(promise);
	gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, NULL);
	GstPromise *local_promise = gst_promise_new();
	g_signal_emit_by_name(webrtcbin, "set-local-description", answer, local_promise);

	object sdp_json;
	sdp_json["type"] = "answer";
	sdp_json["sdp"] = gst_sdp_message_as_text(answer->sdp);
	std::string text = serialize(sdp_json);

	ws->write(net::buffer(text));

	LOG(INFO) << "Local description set and answer sent: " << text;

	gst_webrtc_session_description_free(answer);
}

void WebRTCServer::on_set_remote_description(GstPromise *promise, gpointer user_data)
{
	LOG(INFO) << "Remote description set, creating answer";

    /**
     * GstPromise *promise = gst_promise_new_with_change_func(on_answer_created, ws, NULL);
     * 
     * Problem: argument of type "void (WebRTCServer::*)(GstPromise *promise, gpointer user_data)" (aka "void (WebRTCServer::*)(_GstPromise *promise, gpointer user_data)") is incompatible with parameter of type "GstPromiseChangeFunc" (aka "void (*)(_GstPromise *promise, gpointer user_data)")
     * 
     * Reason: gst_promise_new_with_change_func expects a function pointer with the signature void (*)(GstPromise *promise, gpointer user_data) which is GstPromiseChangeFunc. void (*) is a C-style function pointer which is incompatible with a C++ member function pointer, particularly functions that are part of a class.
     */
    /**
     * Option 1: Use of lambda function to wrap the class member function. 
     * To resolve this, we need to use a static member function or a lambda function that wraps the class member function. 
     * This a classic problem when using C++ with C libraries.
     * 
     * Problem: This works sometimes. E.g., g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(WebRTCServer::on_negotiation_needed), this); -> Doesn't work
     */

    GstPromise *answer_promise = gst_promise_new_with_change_func(
                                                                    [](GstPromise *promise, gpointer user_data) {
                                                                        auto server = static_cast<WebRTCServer*>(user_data);
                                                                        server->on_answer_created(promise, user_data);
                                                                    },
                                                                    this, 
                                                                    NULL
                                                                );

	g_signal_emit_by_name(webrtcbin, "create-answer", NULL, answer_promise);
}

void WebRTCServer::on_set_remote_description_static(GstPromise *promise, gpointer user_data)
{
    LOG(INFO) << "Remote description set, creating answer";
    /**
     * Option 2: Use of static member function
     * Static member functions behave like C-style functions and do not have an implicit this pointer.
     * We can pass implicit this pointer as user_data to the static member function to access the class member functions and variables.
     */

    auto server = static_cast<WebRTCServer *>(user_data); // Use this to access member functions
    auto webrtcbin = server->webrtcbin;
    auto ws = server->ws;

    GstPromise *answer_promise = gst_promise_new_with_change_func(on_answer_created_static, user_data, NULL);

	g_signal_emit_by_name(webrtcbin, "create-answer", NULL, answer_promise);
}

void WebRTCServer::handle_sdp(const std::string &msg)
{
    if(ws == NULL)
        LOG(FATAL) << "Websocket is not initialized";

    if(webrtcbin == NULL)
        LOG(FATAL) << "WebRTCbin is not initialized";
    
    value jv = parse(msg);
    object obj = jv.as_object();
    std::string type = obj["type"].as_string().c_str();

    if (type == "offer")
    {
        LOG(INFO) << "Received offer: " << msg;

        std::string sdp = obj["sdp"].as_string().c_str();

        GstSDPMessage *sdp_message;
        gst_sdp_message_new_from_text(sdp.c_str(), &sdp_message);
        GstWebRTCSessionDescription *offer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp_message);

        /**
         * GstPromise *promise = gst_promise_new_with_change_func(on_set_remote_description, &ws, NULL);
         * 
         * Problem: argument of type "void (WebRTCServer::*)(GstPromise *promise, gpointer user_data)" (aka "void (WebRTCServer::*)(_GstPromise *promise, gpointer user_data)") is incompatible with parameter of type "GstPromiseChangeFunc" (aka "void (*)(_GstPromise *promise, gpointer user_data)")
         * 
         * Reason: gst_promise_new_with_change_func expects a function pointer with the signature void (*)(GstPromise *promise, gpointer user_data) which is GstPromiseChangeFunc. void (*) is a C-style function pointer which is incompatible with a C++ member function pointer, particularly functions that are part of a class. 
         * To resolve this, we need to use a static member function or a lambda function that wraps the class member function. 
         * This a classic problem when using C++ with C libraries.
         */

        // GstPromise *promise = gst_promise_new_with_change_func(
        //                                                         [](GstPromise *promise, gpointer user_data) {
        //                                                             auto server = static_cast<WebRTCServer*>(user_data);
        //                                                             server->on_set_remote_description(promise, user_data);
        //                                                         },
        //                                                         this, 
        //                                                         NULL
        //                                                     );

        GstPromise *promise = gst_promise_new_with_change_func(on_set_remote_description_static, this, NULL);

        g_signal_emit_by_name(webrtcbin, "set-remote-description", offer, promise);
        gst_webrtc_session_description_free(offer);

        LOG(INFO) << "Setting remote description";
    }
    else if (type == "candidate")
    {
        LOG(INFO) << "Received ICE candidate: " << msg;

        object ice = obj["ice"].as_object();
        std::string candidate = ice["candidate"].as_string().c_str();
        guint sdpMLineIndex = ice["sdpMLineIndex"].as_int64();
        g_signal_emit_by_name(webrtcbin, "add-ice-candidate", sdpMLineIndex, candidate.c_str());

        LOG(INFO) << "Added ICE candidate";
    }
    else
        LOG(FATAL) << "Unknown message type: " << msg;
}

void WebRTCServer::send_ice_candidate_message(guint mlineindex, gchar *candidate)
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

void WebRTCServer::on_ice_candidate_static(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data)
{
    LOG(INFO) << "ICE candidate generated: mlineindex=" << mlineindex << ", candidate=" << candidate;

    auto server = static_cast<WebRTCServer *>(user_data); // Use this to access member functions
	server->send_ice_candidate_message(mlineindex, candidate);
}

// void WebRTCServer::on_negotiation_needed(GstElement *webrtc, gpointer user_data)
// {
//     LOG(INFO) << "Negotiation needed";
//     return;
// }

void WebRTCServer::on_negotiation_needed_static(GstElement *webrtc, gpointer user_data)
{
    auto server = static_cast<WebRTCServer *>(user_data); // Use this to access member functions
    LOG(INFO) << "Negotiation needed";
}

void WebRTCServer::start_feed_static(GstElement* appsrc, guint size, gpointer user_data)
{
    auto server = static_cast<WebRTCServer *>(user_data);
    server->is_push_buffer_allowed = true;
    server->push();
    return;
}

void WebRTCServer::stop_feed_static(GstElement* appsrc, gpointer user_data)
{
    auto server = static_cast<WebRTCServer *>(user_data);
    server->is_push_buffer_allowed = false;
    return;
}

void WebRTCServer::set_pipeline_dummy()
{
    GError *g_error = NULL;

	const gchar *pipeline_desc = "webrtcbin name=sendonly stun-server=stun://stun.l.google.com:19302 "
                               	"videotestsrc name=source pattern=ball is-live=true ! videoconvert name=convert ! "
                                "queue name=queue ! vp8enc name=videnc ! rtpvp8pay name=videopay ! "
    							"application/x-rtp, media=video,encoding-name=VP8, payload=96 ! sendonly.";
    pipeline = gst_parse_launch(pipeline_desc, &g_error);
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

    webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "sendonly");
    GstElement *rtpbin = gst_bin_get_by_name(GST_BIN(pipeline), "rtpbin");
    GstElement *videopay = gst_bin_get_by_name(GST_BIN(pipeline), "videopay");
    GstElement *encoder = gst_bin_get_by_name(GST_BIN(pipeline), "videnc");

    if(!webrtcbin || !rtpbin || !videopay || !encoder)
    {
        g_printerr("Failed to get webrtcbin, rtpbin, videopay or encoder.\n");
        end_pipeline();
        return;
    }

    GstWebRTCRTPTransceiverDirection direction_v = GST_WEBRTC_RTP_TRANSCEIVER_DIRECTION_SENDONLY;
    GstCaps *caps_v = gst_caps_from_string("application/x-rtp,media=video,payload=96,clock-rate=90000,encoding-name=VP8");

    // Get Transceivers
    GArray *transceivers = NULL;
    g_signal_emit_by_name(webrtcbin, "get-transceivers", &transceivers);

    for(int i=0; i<transceivers->len; i++)
    {
        GstWebRTCRTPTransceiver* transceiver = g_array_index(transceivers, GstWebRTCRTPTransceiver*, i);
        g_object_set(transceiver, "do-nack", true, NULL);
        g_object_set(transceiver, "direction", direction_v, NULL);
        g_object_set(transceiver, "codec-preferences", caps_v, NULL);
        
        // Get kind of transceiver
        GstWebRTCKind kind;
        g_object_get(transceiver, "kind", &kind, NULL);
        LOG(INFO) << "Transceiver Kind: " << kind;
    }

    // Set Negotiation Needed

    /**
     * For some reason the following code does not work.
     * ChatGPT's response: The error occurs because even though the lambda doesn't capture any variables, the compiler still doesn't treat it as a plain function pointer. 
     * GLib's G_CALLBACK macro expects a function pointer, but the lambda without captures isn't automatically convertible to a function pointer in C++.
     * To use a lambda with GLib's g_signal_connect, we should either use a static function or a C-style function. Below is how to handle this using a static member function, which is the recommended approach for this scenario
     */
    // g_signal_connect(webrtcbin, "on-negotiation-needed",
    //                 G_CALLBACK([](GstElement *webrtc, gpointer user_data) -> void
    //                 {
    //                     LOG(INFO) << "Negotiation needed";
    //                     return;
    //                 }),
    //                 &ws);

    /**
     * Why static member function work here?
     * 
     * ChatGPT's response: The error occurs because even though the lambda doesn't capture any variables, the compiler still doesn't treat it as a plain function pointer. 
     * GLib's G_CALLBACK macro expects a function pointer, but the lambda without captures isn't automatically convertible to a function pointer in C++.
     * To use a lambda with GLib's g_signal_connect, we should either use a static function or a C-style function. Below is how to handle this using a static member function, which is the recommended approach for this scenario.
     * 
     * Static member functions do not have an implicit this pointer. They are essentially the same as global functions but have access to the private and protected members of the class if passed an instance of the class.
     * This makes them compatible with the G_CALLBACK macro, which expects a function pointer with a specific signature.
     * 
     * Inside the static member function, you can manually pass the this pointer (the instance of the class) as the user_data parameter.
     * This allows the static function to call non-static member functions and access member variables using the instance pointer.
     * 
     */
    g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(WebRTCServer::on_negotiation_needed_static), this);
    g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(WebRTCServer::on_ice_candidate_static), this);

    // Set Retransmission
    g_object_set(rtpbin, "do-retransmission", true, NULL);

    // Encoder Properties
    g_object_set(encoder, "deadline", 1, NULL);
    // g_object_set(encoder, "rc-mode", 2, NULL);
    // g_object_set(encoder, "bitrate", color_bitrate, NULL);
	gst_object_unref(encoder);
    return;
}

GstPadProbeReturn WebRTCServer::calc_encoded_bitrate_static(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) 
{
    auto server = static_cast<WebRTCServer *>(user_data);

    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    server->bytes_in_interval += gst_buffer_get_size(buffer);

    guint64 current_time = g_get_monotonic_time();
    if ((current_time - server->start_time) > G_USEC_PER_SEC) // Calculate every second
    { 
        gdouble bitrate = (server->bytes_in_interval * 8.0) / 1000.0; // kbps
        LOG(INFO) << "Type: " << server->type << ", Frame ID: " << server->frame_id << ", Estimated bitrate: " << bitrate << " kbps";

        // Reset counters
        server->bytes_in_interval = 0;
        server->start_time = current_time;
    }

    return GST_PAD_PROBE_OK;
}

void WebRTCServer::set_color_pipeline()
{
    GError *g_error = NULL;

    // NVCODEC: H265 for color
    std::string pipeline_desc = "webrtcbin name=sendonly stun-server=" + std::string(STUN_SERVER) + " "
                                "appsrc name=source emit-signals=True do-timestamp=True ! queue ! "
                                "video/x-raw,format=BGRA,width=" + std::to_string(imgShape[1]) + ",height=" + std::to_string(imgShape[0]) + ",framerate=(fraction)" + std::to_string(fps) + "/1 ! videoconvert ! queue ! "
                                "nvh265enc name=videnc ! h265parse ! queue ! rtph265pay name=videopay ! sendonly.";

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

    webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "sendonly");
    GstElement *rtpbin = gst_bin_get_by_name(GST_BIN(pipeline), "rtpbin");
    GstElement *videopay = gst_bin_get_by_name(GST_BIN(pipeline), "videopay");
    GstElement *encoder = gst_bin_get_by_name(GST_BIN(pipeline), "videnc");

    if(!webrtcbin || !rtpbin || !videopay || !encoder)
    {
        g_printerr("Failed to get webrtcbin, rtpbin, videopay or encoder.\n");
        end_pipeline();
        return;
    }

    GstWebRTCRTPTransceiverDirection direction_v = GST_WEBRTC_RTP_TRANSCEIVER_DIRECTION_SENDONLY;
    GstCaps *caps_v = gst_caps_from_string("application/x-rtp,media=video,encoding-name=(string)H265, clock-rate=90000,payload=96");

    // Get Transceivers
    GArray *transceivers = NULL;
    g_signal_emit_by_name(webrtcbin, "get-transceivers", &transceivers);

    for(int i=0; i<transceivers->len; i++)
    {
        GstWebRTCRTPTransceiver* transceiver = g_array_index(transceivers, GstWebRTCRTPTransceiver*, i);
        g_object_set(transceiver, "do-nack", true, NULL);
        g_object_set(transceiver, "direction", direction_v, NULL);
        g_object_set(transceiver, "codec-preferences", caps_v, NULL);
        
        // Get kind of transceiver
        GstWebRTCKind kind;
        g_object_get(transceiver, "kind", &kind, NULL);
        LOG(INFO) << "Transceiver Kind: " << kind;
    }

    g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(WebRTCServer::on_negotiation_needed_static), this);
    g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(WebRTCServer::on_ice_candidate_static), this);

    // Set Retransmission
    g_object_set(rtpbin, "do-retransmission", true, NULL);

    // Encoder Properties
    // g_object_set(encoder, "deadline", 1, NULL);
    g_object_set(encoder, "rc-mode", 2, NULL);
    g_object_set(encoder, "bitrate", color_bitrate, NULL);

    // Bitrate Probe - Adding probe pad to the output of the encoder
    GstPad *videnc_src_pad = gst_element_get_static_pad(encoder, "src");
    if(!videnc_src_pad)
    {
        LOG(ERROR) << "Failed to get videnc src pad";
        end_pipeline();
        return;
    }
    // gst_pad_add_probe(videnc_src_pad, GST_PAD_PROBE_TYPE_BUFFER, calc_encoded_bitrate_static, this, NULL);
	gst_object_unref(encoder);

    set_appsrc();
}

GstFlowReturn WebRTCServer::on_recvd_sample_static(GstElement *appsink, gpointer user_data)
{
    auto server = static_cast<WebRTCServer *>(user_data);
    return server->on_recvd_sample(appsink, user_data);
}

GstFlowReturn WebRTCServer::on_recvd_sample(GstElement *appsink, gpointer user_data)
{
    GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 10 * GST_MSECOND);
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

    uint8_t *data = map.data;
    int size = map.size;

    cv::Mat res(imgShape[0], imgShape[1], CV_8UC(CHANNEL_BGRA), data);
    int frame_id_detected = decode_qr_code(res);

    if(frame_id_detected != -1)
        LOG(INFO) << "Frame ID: " << frame_id_detected << " received for analysis";
    else
        LOG(ERROR) << "Invalid QR code";
    
    gst_buffer_unmap(gst_buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

void WebRTCServer::setup_appsink_distortion_analyzer()
{
    GstElement *distortion_analyzer = gst_bin_get_by_name(GST_BIN(pipeline), "distortion_analyzer");
    if (!distortion_analyzer) 
    {
        LOG(ERROR) << "Failed to get appsink element.";
        return;
    }

    // Configure appsink properties
    // g_object_set(distortion_analyzer, "emit-signals", TRUE, "sync", FALSE, NULL);
    g_signal_connect(distortion_analyzer, "new-sample", G_CALLBACK(WebRTCServer::on_recvd_sample_static), this);
    g_object_set(distortion_analyzer, "buffer-list", true, NULL);
    g_object_set(distortion_analyzer, "drop", true, NULL);
}

void WebRTCServer::set_color_pipeline_with_distortion_analysis()
{
    GError *g_error = NULL;

    std::string pipeline_desc = "webrtcbin name=sendonly stun-server=" + std::string(STUN_SERVER) + " "
                                "appsrc name=source emit-signals=True do-timestamp=True ! queue ! "
                                "video/x-raw,format=BGRA,width=" + std::to_string(imgShape[1]) + ",height=" + std::to_string(imgShape[0]) + ",framerate=(fraction)" + std::to_string(fps) + "/1 ! videoconvert ! queue ! "
                                "nvh265enc name=videnc ! tee name=t ! queue ! h265parse ! rtph265pay name=videopay ! sendonly. "
                                "t. ! queue ! h265parse ! nvh265dec name=viddec ! queue ! videoconvert ! "
                                "video/x-raw, format=BGRA, width=" + std::to_string(imgShape[1]) + ", height=" + std::to_string(imgShape[0]) + ", framerate=(fraction)" + std::to_string(fps) + "/1 ! "
                                "queue ! appsink name=distortion_analyzer emit-signals=true";

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

    webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "sendonly");
    GstElement *rtpbin = gst_bin_get_by_name(GST_BIN(pipeline), "rtpbin");
    GstElement *videopay = gst_bin_get_by_name(GST_BIN(pipeline), "videopay");
    GstElement *encoder = gst_bin_get_by_name(GST_BIN(pipeline), "videnc");

    if(!webrtcbin || !rtpbin || !videopay || !encoder)
    {
        g_printerr("Failed to get webrtcbin, rtpbin, videopay or encoder.\n");
        end_pipeline();
        return;
    }

    GstWebRTCRTPTransceiverDirection direction_v = GST_WEBRTC_RTP_TRANSCEIVER_DIRECTION_SENDONLY;
    GstCaps *caps_v = gst_caps_from_string("application/x-rtp,media=video,encoding-name=(string)H265, clock-rate=90000,payload=96");

    // Get Transceivers
    GArray *transceivers = NULL;
    g_signal_emit_by_name(webrtcbin, "get-transceivers", &transceivers);

    for(int i=0; i<transceivers->len; i++)
    {
        GstWebRTCRTPTransceiver* transceiver = g_array_index(transceivers, GstWebRTCRTPTransceiver*, i);
        g_object_set(transceiver, "do-nack", true, NULL);
        g_object_set(transceiver, "direction", direction_v, NULL);
        g_object_set(transceiver, "codec-preferences", caps_v, NULL);
        
        // Get kind of transceiver
        GstWebRTCKind kind;
        g_object_get(transceiver, "kind", &kind, NULL);
        LOG(INFO) << "Transceiver Kind: " << kind;
    }

    g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(WebRTCServer::on_negotiation_needed_static), this);
    g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(WebRTCServer::on_ice_candidate_static), this);

    // Set Retransmission
    g_object_set(rtpbin, "do-retransmission", true, NULL);

    // Encoder Properties
    // g_object_set(encoder, "deadline", 1, NULL);
    g_object_set(encoder, "rc-mode", 2, NULL);
    g_object_set(encoder, "bitrate", color_bitrate, NULL);

    // Bitrate Probe - Adding probe pad to the output of the encoder
    GstPad *videnc_src_pad = gst_element_get_static_pad(encoder, "src");
    if(!videnc_src_pad)
    {
        LOG(ERROR) << "Failed to get videnc src pad";
        end_pipeline();
        return;
    }
    gst_pad_add_probe(videnc_src_pad, GST_PAD_PROBE_TYPE_BUFFER, calc_encoded_bitrate_static, this, NULL);
	gst_object_unref(encoder);

    set_appsrc();
    setup_appsink_distortion_analyzer();
}

void WebRTCServer::set_depth_pipeline_bgra()
{
    return;
}

void WebRTCServer::set_depth_pipeline_yuv16()
{
    GError *g_error = NULL;

    // NVCODEC: H265 for depth (YUV16)
	std::string pipeline_desc = "webrtcbin name=sendonly stun-server=" + std::string(STUN_SERVER) + " "
                               	"appsrc name=source emit-signals=True do-timestamp=True ! queue ! "
                                "video/x-raw,format=Y444_16LE,width=" + std::to_string(imgShape[1]) + ",height=" + std::to_string(imgShape[0]) + ",framerate=(fraction)" + std::to_string(fps) + "/1 ! videoconvert ! queue ! "
    							"nvh265enc name=videnc ! h265parse ! queue ! rtph265pay name=videopay ! sendonly.";

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

    webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "sendonly");
    GstElement *rtpbin = gst_bin_get_by_name(GST_BIN(pipeline), "rtpbin");
    GstElement *videopay = gst_bin_get_by_name(GST_BIN(pipeline), "videopay");
    GstElement *encoder = gst_bin_get_by_name(GST_BIN(pipeline), "videnc");

    if(!webrtcbin || !rtpbin || !videopay || !encoder)
    {
        g_printerr("Failed to get webrtcbin, rtpbin, videopay or encoder.\n");
        end_pipeline();
        return;
    }

    GstWebRTCRTPTransceiverDirection direction_v = GST_WEBRTC_RTP_TRANSCEIVER_DIRECTION_SENDONLY;
    GstCaps *caps_v = gst_caps_from_string("application/x-rtp,media=video,encoding-name=(string)H265, clock-rate=90000,payload=96");

    // Get Transceivers
    GArray *transceivers = NULL;
    g_signal_emit_by_name(webrtcbin, "get-transceivers", &transceivers);

    for(int i=0; i<transceivers->len; i++)
    {
        GstWebRTCRTPTransceiver* transceiver = g_array_index(transceivers, GstWebRTCRTPTransceiver*, i);
        g_object_set(transceiver, "do-nack", true, NULL);
        g_object_set(transceiver, "direction", direction_v, NULL);
        g_object_set(transceiver, "codec-preferences", caps_v, NULL);
        
        // Get kind of transceiver
        GstWebRTCKind kind;
        g_object_get(transceiver, "kind", &kind, NULL);
        LOG(INFO) << "Transceiver Kind: " << kind;
    }

    g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(WebRTCServer::on_negotiation_needed_static), this);
    g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(WebRTCServer::on_ice_candidate_static), this);

    // Set Retransmission
    g_object_set(rtpbin, "do-retransmission", true, NULL);

    // Encoder Properties
    // g_object_set(encoder, "deadline", 1, NULL);
    g_object_set(encoder, "rc-mode", 2, NULL);
    g_object_set(encoder, "bitrate", depth_bitrate, NULL);

    // Bitrate Probe - Adding probe pad to the output of the encoder
    GstPad *videnc_src_pad = gst_element_get_static_pad(encoder, "src");
    if(!videnc_src_pad)
    {
        LOG(ERROR) << "Failed to get videnc src pad";
        end_pipeline();
        return;
    }
    // gst_pad_add_probe(videnc_src_pad, GST_PAD_PROBE_TYPE_BUFFER, calc_encoded_bitrate_static, this, NULL);
	gst_object_unref(encoder);

    set_appsrc();
}

void WebRTCServer::set_appsrc()
{
    if(pipeline == NULL)
    {
        g_printerr("Pipeline is not initialized.\n");
        end_pipeline();
        return;
    }

    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "source");
    if(!appsrc)
    {
        g_printerr("Failed to get appsrc.\n");
        end_pipeline();
        return;
    }

    g_object_set(appsrc, "format", GST_FORMAT_TIME, NULL);
    g_object_set(appsrc, "emit-signals", true, NULL);
    g_object_set(appsrc, "max-bytes", 1000000000, NULL);
    g_object_set(appsrc, "is-live", true, NULL);
    g_object_set(appsrc, "leaky-type", 1, NULL);
    g_object_set(appsrc, "do-timestamp", true, NULL);

    g_signal_connect(appsrc, "need-data", G_CALLBACK(WebRTCServer::start_feed_static), this);
    g_signal_connect(appsrc, "enough-data", G_CALLBACK(WebRTCServer::stop_feed_static), this);
}

void WebRTCServer::start_pipeline_dummy()
{
    set_pipeline_dummy();

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

void WebRTCServer::start_pipeline()
{
    if(type == "c" || type == "c_bgra")
        set_color_pipeline();
        // set_color_pipeline_with_distortion_analysis();
    else if(type == "d" || type == "d_bgra")
        set_depth_pipeline_bgra();
    else if(type == "d_yuv16")
        set_depth_pipeline_yuv16();

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
		gst_object_unref(appsrc);
		gst_object_unref(webrtcbin);
        gst_object_unref(pipeline);
		return;
	}

    LOG(INFO) << "Type: " << type << ", Pipeline State: Started";
}

void WebRTCServer::end_pipeline()
{
    if(webrtcbin != NULL)
        gst_object_unref(webrtcbin);
    webrtcbin = NULL;

    if(appsrc != NULL)
        gst_object_unref(appsrc);
    appsrc = NULL;

    if(pipeline != NULL)
    {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }
    pipeline = NULL;
    
    LOG(INFO) << "Pipeline State: Ended";
    return;
}

/**
 * @brief 
 * 
 * @param image 
 * @param buf stores NULL if image is empty or other errors.
 * @param size size of the buffer
 * @param color_format Only "bgra" is supported.
 */
void WebRTCServer::color_to_buf(const cv::Mat &image, uint8_t *&buf, int &size, const std::string &color_format)
{
    if(image.empty())
    {
        LOG(ERROR) << "Empty image";
        return;
    }

    if(buf != NULL)
    {
        delete[] buf;
        buf = NULL;
    }

    if(color_format == "bgra")
    {
        size = image.total() * image.elemSize();
        if(size != imgSize)
        {
            LOG(FATAL) << "Size mismatch: " << size << " != " << imgSize;
            return;
        }

        buf = new uint8_t[size];
        memcpy(buf, image.data, size);
    }
    else
        LOG(ERROR) << "Invalid color format: " << color_format;
}

void WebRTCServer::depth_to_buf(const cv::Mat (&channels)[3], uint8_t *&buf, int &size, const std::string &depth_format)
{
    if(channels[0].empty() || channels[1].empty() || channels[2].empty())
    {
        LOG(ERROR) << "Empty image";
        return;
    }

    if(buf != NULL)
    {
        delete[] buf;
        buf = NULL;
    }

    if(depth_format == "yuv16")
    {
        int channel0_size = channels[0].total() * channels[0].elemSize();
        int channel1_size = channels[1].total() * channels[1].elemSize();
        int channel2_size = channels[2].total() * channels[2].elemSize();

        size = channel0_size + channel1_size + channel2_size;
        if(size != imgSize)
        {
            LOG(FATAL) << "Size mismatch: " << size << " != " << imgSize;
            return;
        }

        buf = new uint8_t[size];

        memcpy(buf, channels[0].data, channel0_size);
        memcpy(buf + channel0_size, channels[1].data, channel1_size);
        memcpy(buf + channel0_size + channel1_size, channels[2].data, channel2_size);
    }
    else
        LOG(ERROR) << "Invalid depth format: " << depth_format;
}

int WebRTCServer::decode_qr_code(const cv::Mat &image, int offset_x, int offset_y, int roi_width, int roi_height, int border, int view_number)
{
    int start_x = (view_number % 5) * (imgShape[1] / 5);
    int start_y = (view_number / 5) * (imgShape[0] / 2);

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

int WebRTCServer::decode_qr_code_yuv(const cv::Mat &image, int offset_x, int offset_y, int roi_width, int roi_height, int border, int view_number)
{
    int start_x = (view_number % 5) * (imgShape[1] / 5);
    int start_y = (view_number / 5) * (imgShape[0] / 2);

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

bool WebRTCServer::read_color(cv::Mat &image, const int frameID)
{
    image = cv::imread(FORMAT(color_dir << frameID << "_color.png"), cv::IMREAD_UNCHANGED);

    if(image.empty())
    {
        LOG(ERROR) << "Failed to read color image for frame " << frameID;
        return false;
    }

    return true;
}

bool WebRTCServer::read_depth(cv::Mat (&channels)[3], const int frameID)
{
    channels[0] = cv::imread(FORMAT(depth_dir << frameID << "_depth_0.png"), cv::IMREAD_ANYDEPTH);
    channels[1] = cv::imread(FORMAT(depth_dir << frameID << "_depth_1.png"), cv::IMREAD_ANYDEPTH);
    channels[2] = cv::imread(FORMAT(depth_dir << frameID << "_depth_2.png"), cv::IMREAD_ANYDEPTH);

    if(channels[0].empty() || channels[1].empty() || channels[2].empty())
    {
        LOG(ERROR) << "Failed to read depth image for frame " << frameID;
        return false;
    }

    return true;
}

void WebRTCServer::push()
{
    if(!is_push_buffer_allowed)
    {
        LOG(INFO) << "Push buffer not allowed, waiting for frames ...";
        return;
    }
    
    fps_limiter.rate_limit();

    StopWatch sw;
    sw.Restart();

    uint8_t *data = NULL;
    int size = 0;

    if(type == "c" || type == "c_bgra")
    {
        // Read color image
        cv::Mat color;
        if(!read_color(color, frame_id))
        {
            LOG(ERROR) << "Failed to read color image for frame " << frame_id;
            return;
        }
        color_to_buf(color, data, size, "bgra");
    }
    else if(type == "d_yuv16")
    {
        // Read depth image
        cv::Mat channels[3];
        if(!read_depth(channels, frame_id))
        {
            LOG(ERROR) << "Failed to read depth image for frame " << frame_id;
            return;
        }
        depth_to_buf(channels, data, size, "yuv16");
    }
    else
    {
        LOG(ERROR) << "Invalid type: " << type;
        return;
    }

    if(data == NULL)
    {
        LOG(ERROR) << "Empty data";
        return;
    }

    GstBuffer *gst_buf = gst_buffer_new_allocate(NULL, size, NULL);
    gst_buffer_fill(gst_buf, 0, data, size);

    int frame_id_detected = -1;
    if (type == "c" || type == "c_bgra") 
    {
        cv::Mat res(imgShape[0], imgShape[1], CV_8UC(CHANNEL_BGRA), data);
        frame_id_detected = decode_qr_code(res);
    } 
    else if (type == "d_yuv16") 
    {
        cv::Mat channel_y = cv::Mat(imgShape[0], imgShape[1], CV_16UC1, data);
        cv::Mat channel_u = cv::Mat(imgShape[0], imgShape[1], CV_16UC1, data + channel_y.total() * channel_y.elemSize());
        cv::Mat channel_v = cv::Mat(imgShape[0], imgShape[1], CV_16UC1, data + channel_y.total() * channel_y.elemSize() + channel_u.total() * channel_u.elemSize());
        frame_id_detected = decode_qr_code_yuv(channel_u);
    }

    if(frame_id_detected == -1)
        LOG(WARNING) << "Type: " << type << ", Failed to detect frame ID, Actual Frame ID: " << frame_id;
    
    GstCaps *caps = NULL;
    if (type == "c" || type == "c_bgra")
        caps = gst_caps_from_string(FORMAT("video/x-raw,format=BGRA,width=" << imgShape[1] << ",height=" << imgShape[0] << ",framerate=(fraction)" << fps << "/1").c_str());
    else if (type == "d_yuv16") 
        caps = gst_caps_from_string(FORMAT("video/x-raw,format=Y444_16LE,width=" << imgShape[1] << ",height=" << imgShape[0] << ",framerate=(fraction)" << fps << "/1").c_str());
    else 
        LOG(FATAL) << "Invalid type: " << type;

    GstSample *sample = gst_sample_new(gst_buf, caps, NULL, NULL);
    GstFlowReturn gst_flow_return = gst_app_src_push_sample(GST_APP_SRC(appsrc), sample);
    if (gst_flow_return != GST_FLOW_OK)
    {
        LOG(ERROR) << "Failed to push sample to appsrc, stop sending data";
        is_push_buffer_allowed = false;
        end_pipeline();
        return;
    }
    LOG(INFO) << "Type: " << type << ", Pushed Frame ID: " << frame_id;

    gst_caps_unref(caps);
    gst_sample_unref(sample);
    gst_buffer_unref(gst_buf);
    delete[] data;

    if (frame_id % 100 == 0)
        LOG(INFO) << "Type: " << type << ", Sent Frame: " << frame_id << ", Time taken: " << sw.ElapsedMs() << " ms";

    frame_id++;
    // LOG(INFO) << "Type: " << type << ", FPS: " << fps_counter.fps_counter();

    if(frame_id >= end_frame_id)
    {
        LOG(INFO) << "Type: " << type << ", Reached End Frame ID: " << end_frame_id;
        LOG(INFO) << "Type: " << type << ", Ending pipeline!";
        is_push_buffer_allowed = false;
        end_pipeline();
    }
    
    return;
}

void WebRTCServer::probe_stats()
{
    // TODO: Implement
    return;
}

void WebRTCServer::ws_handler(tcp::socket socket)
{
    try
	{
		this->ws = new websocket::stream<tcp::socket>(std::move(socket));
		ws->accept();

		LOG(INFO) << "WebSocket connection accepted from client: " << ws->next_layer().remote_endpoint().address().to_string();

		for (;;)
		{
			beast::flat_buffer buffer;
			ws->read(buffer);
			auto msg = beast::buffers_to_string(buffer.data());
            std::vector<std::string> data;
            boost::split(data, msg, boost::is_any_of(" "));

            if(data[0] == "HELLO")
            {
                client_uid = uint32_t(std::stoi(data[1]));
                LOG(INFO) << "Received HELLO from client id " << client_uid;
                LOG(INFO) << "Sending HELLO_OK to client id " << client_uid;
                std::string response = FORMAT("HELLO_OK " << imgShape[0] << " " << imgShape[1] << " " << imgShape[2]);
                ws->write(net::buffer(response));
            }
            else if(data[0] == "OFFER_REQUEST")
            {
                LOG(INFO) << "received offer request";
                LOG(INFO) << "STARTING PIPELINE";
                // start_pipeline_dummy();
                start_pipeline();

                // Create a task to probe stats
                // std::thread{&WebRTCServer::probe_stats, this}.detach();
            }
            else if(data[0] == "ERROR")
            {
                LOG(INFO) << "Received client error: " << msg;
                LOG(ERROR) << "Ending pipeline";
                end_pipeline();
            }
            else
            {
                // Handle SDP message
                LOG(INFO) << "Handle SDP message";
                handle_sdp(msg);
            }

		}
	}
	catch (beast::system_error const& se)
	{
		if (se.code() == websocket::error::closed)
        {
            LOG(INFO) << "WebSocket connection closed by client.";
        }
        else if (se.code() == net::error::eof)
        {
            LOG(WARNING) << "Connection closed unexpectedly (EOF) by client.";
        }
        else
        {
            LOG(ERROR) << "WebSocket error: " << se.code().message();
        }
	}
	catch (std::exception const& e)
	{
		LOG(ERROR) << "Exception: " << e.what();
	}
}

void WebRTCServer::run()
{
    try
	{
		auto const _addr = net::ip::make_address(addr.c_str());
		auto const _port = static_cast<unsigned short>(port);

		// The io_context is required for all I/O
		net::io_context ioc{1};

		// The acceptor receives incoming connections
		// tcp::acceptor acceptor{ioc, tcp::endpoint{tcp::v4(), SERVER_PORT}};			// tcp::v4() points to localhost
		tcp::acceptor acceptor{ioc, {_addr, _port}};
        
		for(;;)
		{
			tcp::socket socket{ioc};
            LOG(INFO) << "Waiting for incoming TCP connection...";
			acceptor.accept(socket);                        // Blocking call, waits for incoming connections

            // This for accepting multiple client connections.
            // std::thread t(
            //             &WebRTCServer::ws_handler, 
            //             this, 
            //             std::move(socket));
            // handler_pools.push_back(std::move(t));

            // This is for accepting only one client connection.
            ws_handler(std::move(socket));
            sleep_ms(10);
		}

        for(int i=0; i<handler_pools.size(); i++)
        {
            if(handler_pools[i].joinable())
                handler_pools[i].join();
        }

        LOG(INFO) << "All threads joined";
	}
	catch (std::exception const& e)
	{
		LOG(ERROR) << "Exception: " << e.what();
	}
}