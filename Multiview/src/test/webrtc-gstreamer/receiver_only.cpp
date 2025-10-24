#define GST_USE_UNSTABLE_API
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <iostream>
#include <thread>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using namespace boost::json;

#define STUN_SERVER "stun://stun.l.google.com:19302"

// Connect to localhost signalling server
// #define SERVER_HOST "127.0.0.1"
// #define SERVER_PORT 8000

// Connect to remote signalling server with public IP
#define SERVER_HOST "68.181.32.205"
#define SERVER_PORT 5252

GMainLoop *loop;
GstElement *pipeline, *webrtcbin;

void send_ice_candidate_message(websocket::stream<tcp::socket>& ws, guint mlineindex, gchar *candidate)
{
    std::cout << "Sending ICE candidate: mlineindex=" << mlineindex << ", candidate=" << candidate << std::endl;

    object ice_json;
    ice_json["candidate"] = candidate;
    ice_json["sdpMLineIndex"] = mlineindex;

    object msg_json;
    msg_json["type"] = "candidate";
    msg_json["ice"] = ice_json;

    std::string text = serialize(msg_json);
    ws.write(net::buffer(text));

    std::cout << "ICE candidate sent" << std::endl;
}

void on_offer_created(GstPromise *promise, gpointer user_data)
{
    std::cout << "Offer created" << std::endl;

    websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
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

    std::cout << "Local description set and offer sent: " << text << std::endl;

    gst_webrtc_session_description_free(offer);
}

void on_negotiation_needed(GstElement *webrtc, gpointer user_data)
{
    std::cout << "Negotiation needed" << std::endl;

    websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
    GstPromise *promise = gst_promise_new_with_change_func(on_offer_created, ws, NULL);
    g_signal_emit_by_name(webrtcbin, "create-offer", NULL, promise);
}

void on_set_remote_description(GstPromise *promise, gpointer user_data)
{
    std::cout << "Remote description set" << std::endl;
}

void on_ice_candidate(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data)
{
    std::cout << "ICE candidate generated: mlineindex=" << mlineindex << ", candidate=" << candidate << std::endl;

    websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
    send_ice_candidate_message(*ws, mlineindex, candidate);
}

void start_pipeline(websocket::stream<tcp::socket> &ws)
{
    GError *error = NULL;

    const gchar *pipeline_desc = "webrtcbin name=recvonly stun-server=stun://stun.l.google.com:19302 "
                                 "recvonly. ! application/x-rtp, media=video, encoding-name=VP8, payload=96 ! rtpvp8depay ! vp8dec ! videoconvert ! autovideosink";

    pipeline = gst_parse_launch(pipeline_desc, &error);

    if (error)
    {
        g_printerr("Failed to parse launch: %s\n", error->message);
        g_error_free(error);
        return;
    }

    if (!pipeline)
    {
        g_printerr("Pipeline could not be created.\n");
        return;
    }

    webrtcbin = gst_bin_get_by_name(GST_BIN(pipeline), "recvonly");
    g_assert(webrtcbin != NULL);

    g_object_set(webrtcbin, "bundle-policy", GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE, NULL);
    
    g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(on_negotiation_needed), &ws);
    g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(on_ice_candidate), &ws);

    // Add transceiver to the webrtcbin
    GstWebRTCRTPTransceiver *transceiver = NULL;
    GstWebRTCRTPTransceiverDirection direction_v = GST_WEBRTC_RTP_TRANSCEIVER_DIRECTION_RECVONLY;
    GstCaps *caps_v = gst_caps_from_string("application/x-rtp,media=video,payload=96,clock-rate=90000,encoding-name=VP8");
    g_signal_emit_by_name(webrtcbin, "add-transceiver", direction_v, caps_v, &transceiver);
    gst_caps_unref(caps_v);

    if (!transceiver) {
        g_printerr("Failed to add transceiver.\n");
        return;
    }

    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(pipeline);
        gst_object_unref(webrtcbin);
        return;
    }

    std::cout << "GStreamer pipeline set to playing" << std::endl;
}

void handle_websocket_session(tcp::socket socket)
{
    try
    {
        websocket::stream<tcp::socket> ws{std::move(socket)};
        ws.handshake(SERVER_HOST, "/");

        std::cout << "WebSocket connection established" << std::endl;

        start_pipeline(ws);

        for (;;)
        {
            beast::flat_buffer buffer;
            ws.read(buffer);

            auto text = beast::buffers_to_string(buffer.data());
            value jv = parse(text);
            object obj = jv.as_object();
            std::string type = obj["type"].as_string().c_str();

            if (type == "answer")
            {
                std::cout << "Received answer: " << text << std::endl;

                std::string sdp = obj["sdp"].as_string().c_str();

                GstSDPMessage *sdp_message;
                gst_sdp_message_new_from_text(sdp.c_str(), &sdp_message);
                GstWebRTCSessionDescription *answer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp_message);
                GstPromise *promise = gst_promise_new_with_change_func(on_set_remote_description, &ws, NULL);
                g_signal_emit_by_name(webrtcbin, "set-remote-description", answer, promise);
                gst_webrtc_session_description_free(answer);

                std::cout << "Setting remote description" << std::endl;
            }
            else if (type == "candidate")
            {
                std::cout << "Received ICE candidate: " << text << std::endl;

                object ice = obj["ice"].as_object();
                std::string candidate = ice["candidate"].as_string().c_str();
                guint sdpMLineIndex = ice["sdpMLineIndex"].as_int64();
                g_signal_emit_by_name(webrtcbin, "add-ice-candidate", sdpMLineIndex, candidate.c_str());

                std::cout << "Added ICE candidate" << std::endl;
            }
        }
    }
    catch (beast::system_error const& se)
    {
        if (se.code() != websocket::error::closed)
        {
            std::cerr << "Error: " << se.code().message() << std::endl;
        }
    }
    catch (std::exception const& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}

int main(int argc, char *argv[])
{
    gst_init(&argc, &argv);
    loop = g_main_loop_new(NULL, FALSE);

    try
    {
        net::io_context ioc;
        tcp::resolver resolver(ioc);
        auto const results = resolver.resolve(SERVER_HOST, std::to_string(SERVER_PORT));

        websocket::stream<tcp::socket> ws{ioc};
        net::connect(ws.next_layer(), results.begin(), results.end());
        // ws.handshake(SERVER_HOST, "/");

        // std::cout << "Connected to WebSocket server" << std::endl;

        std::thread{handle_websocket_session, std::move(ws.next_layer())}.detach();

        ioc.run();
    }
    catch (std::exception const& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    g_main_loop_run(loop);

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    gst_object_unref(webrtcbin);
    g_main_loop_unref(loop);

    std::cout << "WebRTC recvonly stopped" << std::endl;

    return 0;
}
