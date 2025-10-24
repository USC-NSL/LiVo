#define GST_USE_UNSTABLE_API
#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <boost/json.hpp>
#include <iostream>
#include <thread>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using namespace boost::json;

#define STUN_SERVER "stun://stun.l.google.com:19302"
#define SERVER_PORT 8000

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

void on_answer_created(GstPromise *promise, gpointer user_data)
{
  std::cout << "Answer created" << std::endl;

  websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
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

  std::cout << "Local description set and answer sent: " << text << std::endl;

  gst_webrtc_session_description_free(answer);
}

void on_negotiation_needed(GstElement *webrtc, gpointer user_data)
{
  std::cout << "Negotiation needed" << std::endl;
}

void on_set_remote_description(GstPromise *promise, gpointer user_data)
{
  std::cout << "Remote description set, creating answer" << std::endl;

  websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
  GstPromise *answer_promise = gst_promise_new_with_change_func(on_answer_created, ws, NULL);

  g_signal_emit_by_name(webrtcbin, "create-answer", NULL, answer_promise);
}

void on_ice_candidate(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data)
{
  std::cout << "ICE candidate generated: mlineindex=" << mlineindex << ", candidate=" << candidate << std::endl;

  websocket::stream<tcp::socket>* ws = static_cast<websocket::stream<tcp::socket>*>(user_data);
  send_ice_candidate_message(*ws, mlineindex, candidate);
}

void handle_websocket_session(tcp::socket socket)
{
  try
  {
    websocket::stream<tcp::socket> ws{std::move(socket)};
    ws.accept();

    std::cout << "WebSocket connection accepted" << std::endl;

    GstStateChangeReturn ret;
    GError *error = NULL;

    pipeline = gst_pipeline_new("pipeline");
    // GstElement *v4l2src = gst_element_factory_make("v4l2src", "source");
    // Use videotestsrc instead of v4l2src with is-live=true pattern=ball
    GstElement *v4l2src = gst_element_factory_make("videotestsrc", "source");
    GstElement *videoconvert = gst_element_factory_make("videoconvert", "convert");
    GstElement *queue = gst_element_factory_make("queue", "queue");
    GstElement *vp8enc = gst_element_factory_make("vp8enc", "encoder");
    GstElement *rtpvp8pay = gst_element_factory_make("rtpvp8pay", "pay");
    webrtcbin = gst_element_factory_make("webrtcbin", "sendrecv");

    if (!pipeline || !v4l2src || !videoconvert || !queue || !vp8enc || !rtpvp8pay || !webrtcbin)
    {
      g_printerr("Not all elements could be created.\n");
      return;
    }

    // g_object_set(v4l2src, "device", "/dev/video0", NULL);    // Remove this, since no webcam is connected
    // g_object_set(v4l2src, "pattern", 0, NULL);      // color bars
    g_object_set(v4l2src, "pattern", 18, NULL);     // ball pattern
    g_object_set(v4l2src, "is-live", true, NULL);
    g_object_set(vp8enc, "deadline", 1, NULL);

    gst_bin_add_many(GST_BIN(pipeline), v4l2src, videoconvert, queue, vp8enc, rtpvp8pay, webrtcbin, NULL);

    if (!gst_element_link_many(v4l2src, videoconvert, queue, vp8enc, rtpvp8pay, NULL))
    {
      g_printerr("Elements could not be linked.\n");
      gst_object_unref(pipeline);
      return;
    }

    GstPad *rtp_src_pad = gst_element_get_static_pad(rtpvp8pay, "src");
    GstPad *webrtc_sink_pad = gst_element_get_request_pad(webrtcbin, "sink_%u");
    gst_pad_link(rtp_src_pad, webrtc_sink_pad);
    gst_object_unref(rtp_src_pad);
    gst_object_unref(webrtc_sink_pad);

    g_signal_connect(webrtcbin, "on-negotiation-needed", G_CALLBACK(on_negotiation_needed), &ws);
    g_signal_connect(webrtcbin, "on-ice-candidate", G_CALLBACK(on_ice_candidate), &ws);

    ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);

    if (ret == GST_STATE_CHANGE_FAILURE)
    {
      g_printerr("Unable to set the pipeline to the playing state.\n");
      gst_object_unref(pipeline);
      return;
    }

    std::cout << "GStreamer pipeline set to playing" << std::endl;

    for (;;)
    {
      beast::flat_buffer buffer;
      ws.read(buffer);

      auto text = beast::buffers_to_string(buffer.data());
      value jv = parse(text);
      object obj = jv.as_object();
      std::string type = obj["type"].as_string().c_str();

      if (type == "offer")
      {
        std::cout << "Received offer: " << text << std::endl;

        std::string sdp = obj["sdp"].as_string().c_str();

        GstSDPMessage *sdp_message;
        gst_sdp_message_new_from_text(sdp.c_str(), &sdp_message);
        GstWebRTCSessionDescription *offer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp_message);
        GstPromise *promise = gst_promise_new_with_change_func(on_set_remote_description, &ws, NULL);
        g_signal_emit_by_name(webrtcbin, "set-remote-description", offer, promise);
        gst_webrtc_session_description_free(offer);

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

void start_server()
{
  try
  {
    net::io_context ioc{1};
    tcp::acceptor acceptor{ioc, tcp::endpoint{tcp::v4(), SERVER_PORT}};

    for (;;)
    {
      tcp::socket socket{ioc};
      acceptor.accept(socket);
      std::cout << "Accepted new TCP connection" << std::endl;
      std::thread{handle_websocket_session, std::move(socket)}.detach();
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

  std::cout << "Starting WebRTC server" << std::endl;

  std::thread server_thread(start_server);
  g_main_loop_run(loop);

  server_thread.join();

  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(pipeline);
  g_main_loop_unref(loop);

  std::cout << "WebRTC server stopped" << std::endl;

  return 0;
}
