#include "TilingEncoderGPU.h"
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>

#define CHANNEL_BGRA 4
#define CHANNEL_DEPTH 3

#define PIXEL_SIZE_BGRA 1
#define PIXEL_SIZE_DEPTH 2

TilingEncoderGPU::TilingEncoderGPU(const std::string &type, int shape[2], const std::string seq_name, int start_frame_id, int end_frame_id, const std::string &in_dir, const std::string &out_dir, int fps, uint32_t color_bitrate, uint32_t depth_bitrate)
    : type(type), seq_name(seq_name), in_dir(in_dir), out_dir(out_dir), color_bitrate(color_bitrate), depth_bitrate(depth_bitrate), fps(fps), fps_limiter(fps), fps_counter()
{
    color_dir = FORMAT(in_dir << "/" << seq_name << "/tiled/color/");
    depth_dir = FORMAT(in_dir << "/" << seq_name << "/tiled/depth/");

    color_dir_untiled = FORMAT(in_dir << "/" << seq_name << "/color/");
    depth_dir_untiled = FORMAT(in_dir << "/" << seq_name << "/depth/");

    is_push_buffer_allowed = false;
    frame_id = start_frame_id;
    recv_frame_id = start_frame_id;
    recv_view_id = 0;
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
    appsrc = NULL;
    appsink = NULL;

    start_time = g_get_monotonic_time();
}

TilingEncoderGPU::~TilingEncoderGPU()
{
    if(appsrc != NULL)
        gst_object_unref(appsrc);
    if(appsink != NULL)
        gst_object_unref(appsink);
    if(pipeline != NULL)
        gst_object_unref(pipeline);

    LOG(INFO) << "TilingEncoderGPU destroyed";
}

void TilingEncoderGPU::start_feed_static(GstElement* appsrc, guint size, gpointer user_data)
{
    auto server = static_cast<TilingEncoderGPU *>(user_data);
    server->is_push_buffer_allowed = true;
    // server->push();
    server->push_untiled();
    return;
}

void TilingEncoderGPU::stop_feed_static(GstElement* appsrc, gpointer user_data)
{
    auto server = static_cast<TilingEncoderGPU *>(user_data);
    server->is_push_buffer_allowed = false;
    return;
}

void TilingEncoderGPU::set_appsrc()
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

    g_signal_connect(appsrc, "need-data", G_CALLBACK(TilingEncoderGPU::start_feed_static), this);
    g_signal_connect(appsrc, "enough-data", G_CALLBACK(TilingEncoderGPU::stop_feed_static), this);
}

void TilingEncoderGPU::set_appsink()
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

    g_signal_connect(appsink, "new-sample", G_CALLBACK(pull_static), this);
    g_object_set(appsink, "buffer-list", true, NULL);
    g_object_set(appsink, "drop", true, NULL);
}

void TilingEncoderGPU::set_color_pipeline()
{
    GError *g_error = NULL;

    // NVCODEC: H265 for color
    std::string pipeline_desc = "appsrc name=source emit-signals=True do-timestamp=True ! queue ! "
                                "video/x-raw,format=BGRA,width=" + std::to_string(imgShape[1]) + ",height=" + std::to_string(imgShape[0]) + ",framerate=(fraction)" + std::to_string(fps) + "/1 ! videoconvert ! queue ! "
                                "nvh265enc name=videnc ! h265parse ! queue ! appsink name=sink emit-signals=true";

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

    GstElement *encoder = gst_bin_get_by_name(GST_BIN(pipeline), "videnc");

    if(!encoder)
    {
        g_printerr("Failed to get encoder.\n");
        end_pipeline();
        return;
    }

    // Encoder Properties
    // g_object_set(encoder, "deadline", 1, NULL);
    g_object_set(encoder, "rc-mode", 2, NULL);
    g_object_set(encoder, "bitrate", color_bitrate, NULL);
	gst_object_unref(encoder);

    set_appsrc();
    set_appsink();

    LOG(INFO) << "Color Pipeline Set";
}

void TilingEncoderGPU::color_to_buf(const cv::Mat &image, uint8_t *&buf, int &size, const std::string &color_format)
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

void TilingEncoderGPU::depth_to_buf(const cv::Mat (&channels)[3], uint8_t *&buf, int &size, const std::string &depth_format)
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

bool TilingEncoderGPU::read_color(cv::Mat &image, const int frameID)
{
    image = cv::imread(FORMAT(color_dir << frameID << "_color.png"), cv::IMREAD_UNCHANGED);

    if(image.empty())
    {
        LOG(ERROR) << "Failed to read color image for frame " << frameID;
        return false;
    }

    return true;
}

bool TilingEncoderGPU::read_color_untiled(cv::Mat &image, const int frameID, const int viewID)
{
    std::string file_path = FORMAT(color_dir_untiled << frameID << "_color_" << viewID << ".png");
    image = cv::imread(file_path, cv::IMREAD_UNCHANGED);

    if(image.empty())
    {
        LOG(ERROR) << "Failed to read color image file: " << file_path;
        return false;
    }

    return true;
}

bool TilingEncoderGPU::read_depth(cv::Mat (&channels)[3], const int frameID)
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

void TilingEncoderGPU::push()
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
    
    GstCaps *caps = NULL;
    if (type == "c" || type == "c_bgra")
        caps = gst_caps_from_string(FORMAT("video/x-raw,format=BGRA,width=" << imgShape[1] << ",height=" << imgShape[0] << ",framerate=(fraction)" << fps << "/1").c_str());
    else if (type == "d_yuv16") 
        caps = gst_caps_from_string(FORMAT("video/x-raw,format=Y444_16LE,width=" << imgShape[1] << ",height=" << imgShape[0] << ",framerate=(fraction)" << fps << "/1").c_str());
    else 
        LOG(FATAL) << "Invalid type: " << type;

    GstSample *sample = gst_sample_new(gst_buf, caps, NULL, NULL);

    send_timestamp[frame_id] = g_get_monotonic_time();
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

void TilingEncoderGPU::push_untiled()
{
    if(!is_push_buffer_allowed)
    {
        LOG(INFO) << "Push buffer not allowed, waiting for frames ...";
        return;
    }
    
    fps_limiter.rate_limit();

    StopWatch sw;
    sw.Restart();

    std::vector<uint8_t *> data(10, NULL);
    int size = 0;
    if(type == "c" || type == "c_bgra")
    {
        // Read color image
        for(int view_id=0; view_id<10; view_id++)
        {
            cv::Mat color;
            if(!read_color_untiled(color, frame_id, view_id))
            {
                LOG(ERROR) << "Failed to read color image for frame " << frame_id;
                return;
            }
            color_to_buf(color, data[view_id], size, "bgra");
        }
    }
    else
    {
        LOG(ERROR) << "Invalid type: " << type;
        return;
    }

    std::vector<GstSample *> samples(10, NULL);
    for(int view_id=0; view_id<10; view_id++)
    {
        if(data[view_id] == NULL)
        {
            LOG(FATAL) << "Empty data";
            return;
        }
        GstBuffer *gst_buf = gst_buffer_new_allocate(NULL, size, NULL);
        gst_buffer_fill(gst_buf, 0, data[view_id], size);
        
        GstCaps *caps = NULL;
        if (type == "c" || type == "c_bgra")
            caps = gst_caps_from_string(FORMAT("video/x-raw,format=BGRA,width=" << imgShape[1] << ",height=" << imgShape[0] << ",framerate=(fraction)" << fps << "/1").c_str());
        else if (type == "d_yuv16") 
            caps = gst_caps_from_string(FORMAT("video/x-raw,format=Y444_16LE,width=" << imgShape[1] << ",height=" << imgShape[0] << ",framerate=(fraction)" << fps << "/1").c_str());
        else 
            LOG(FATAL) << "Invalid type: " << type;

        GstSample *sample = gst_sample_new(gst_buf, caps, NULL, NULL);
        samples[view_id] = sample;
    }
    
    send_timestamp[frame_id] = g_get_monotonic_time();
    for(int view_id=0; view_id<10; view_id++)
    {
        GstFlowReturn gst_flow_return = gst_app_src_push_sample(GST_APP_SRC(appsrc), samples[view_id]);
        if (gst_flow_return != GST_FLOW_OK)
        {
            LOG(ERROR) << "Failed to push sample to appsrc, stop sending data";
            is_push_buffer_allowed = false;
            end_pipeline();
            return;
        }
    }
    LOG(INFO) << "Type: " << type << ", Pushed Untiled Frame ID: " << frame_id;

    for(int view_id=0; view_id<10; view_id++)
    {
        gst_caps_unref(gst_sample_get_caps(samples[view_id]));
        gst_buffer_unref(gst_sample_get_buffer(samples[view_id]));
        delete[] data[view_id];
    }

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

}

GstFlowReturn TilingEncoderGPU::pull_static(GstElement *appsink, gpointer user_data)
{
    auto client = static_cast<TilingEncoderGPU *>(user_data);
    // return client->pull();
    return client->pull_untiled();
}

GstFlowReturn TilingEncoderGPU::pull()
{
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 10 * GST_MSECOND);

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

    if(send_timestamp.find(recv_frame_id) == send_timestamp.end())
    {
        LOG(ERROR) << "Type: " << type << ", Frame ID: " << recv_frame_id << " not found in send_timestamp";
    }

    gint64 send_time = send_timestamp[recv_frame_id];
    gint64 recv_time = g_get_monotonic_time();
    
    // Get the time taken to receive the frame in ms
    float time_taken = (recv_time - send_time) / 1000.0;
    LOG(INFO) << "Type: " << type << ", Pulled Frame ID: " << recv_frame_id << ", Time taken: " << time_taken << " ms";

    recv_frame_id++;
    
    gst_buffer_unmap(gst_buffer, &map);
    gst_sample_unref(sample);

    if(recv_frame_id > end_frame_id)
    {
        LOG(INFO) << "Type: " << type << ", Received all frames. Ending pipeline";
        end_pipeline();
        return GST_FLOW_EOS;
    }

    return GST_FLOW_OK;
}

GstFlowReturn TilingEncoderGPU::pull_untiled()
{
    GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 10 * GST_MSECOND);

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

    recv_view_id++;
    if(recv_view_id == 10)
    {
        gint64 send_time = send_timestamp[recv_frame_id];
        gint64 recv_time = g_get_monotonic_time();
        
        // Get the time taken to receive the frame in ms
        float time_taken = (recv_time - send_time) / 1000.0;
        LOG(INFO) << "Type: " << type << ", Pulled Untiled Frame ID: " << recv_frame_id << ", Time taken: " << time_taken << " ms";
        
        recv_frame_id++;
        recv_view_id = 0;
    }

    gst_buffer_unmap(gst_buffer, &map);
    gst_sample_unref(sample);

    if(recv_frame_id > end_frame_id)
    {
        LOG(INFO) << "Type: " << type << ", Received all frames. Ending pipeline";
        end_pipeline();
        return GST_FLOW_EOS;
    }
    return GST_FLOW_OK;
}

void TilingEncoderGPU::end_pipeline()
{
    if(appsrc != NULL)
        gst_object_unref(appsrc);
    appsrc = NULL;

    if(appsink != NULL)
        gst_object_unref(appsink);
    appsink = NULL;

    if(pipeline != NULL)
    {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }
    pipeline = NULL;
    
    LOG(INFO) << "Pipeline State: Ended";
    return;
}

void TilingEncoderGPU::start_pipeline()
{
    if(type == "c" || type == "c_bgra")
        set_color_pipeline();
    else
        LOG(FATAL) << "Invalid type: " << type;

    if(!pipeline)
    {
        g_printerr("Pipeline is not initialized.\n");
        return;
    }

    // Start the pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);

	if (ret == GST_STATE_CHANGE_FAILURE)
	{
		g_printerr("Unable to set the pipeline to the playing state.\n");
		gst_object_unref(appsrc);
        gst_object_unref(appsink);
        gst_object_unref(pipeline);
		return;
	}

    LOG(INFO) << "Type: " << type << ", Pipeline State: Started";
}