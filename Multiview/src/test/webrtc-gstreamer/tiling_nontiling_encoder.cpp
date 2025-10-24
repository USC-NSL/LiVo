#include "TilingEncoderGPU.h"
#include "TilingEncoderCPU.h"
#include "TilingEncoderCPUParallel.h"
#include <boost/filesystem.hpp>
#include "gflags/gflags.h"

// Host webrtc signalling server on public IP
#define SERVER_HOST "68.181.32.205"
#define SERVER_PORT 5252

#define TARGET_FPS 15.0F
#define OUTDIR_BASE_PATH "/home/lei/data/pipeline/server_tiled/pipeline_new/"
#define SRC_PATH "/home/lei/data/KinectStream/panoptic_captures/"

namespace fs = boost::filesystem;

bool check_webrtc_plugins()
{
    GstPlugin *plugin;
    std::string needed[] = {"opus", "vpx", "nice", "webrtc", "dtls", "srtp", "rtp", "x265",
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

void run_webrtc_server(const string &_type, int shape[3], const string &out_dir, float fps, uint32_t color_bitrate, uint32_t depth_bitrate)
{
    // WebRTCServer server(_type, shape, out_dir, fps, color_bitrate, depth_bitrate);
}

DECLARE_string(seq_name);
DECLARE_int32(ncaptures);
DECLARE_int32(fps);
DECLARE_int32(color_bitrate);
DECLARE_int32(depth_bitrate);
DECLARE_int32(start_frame_id);

DEFINE_string(seq_name, "" , "Sequence name.");
DEFINE_int32(ncaptures, 4000, "Number of frames to capture");
DEFINE_int32(fps, TARGET_FPS , "Sender FPS");
DEFINE_int32(color_bitrate, 10000, "Sender color channel bitrate");
DEFINE_int32(depth_bitrate, 20000, "Sender depth channel bitrate");
DEFINE_int32(start_frame_id, -1, "Start frame id");

void tiled_GPU(int argc, char *argv[])
{
    gst_init(&argc, &argv);
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);

    gflags::SetUsageMessage("Usage: sender_webrtc_only [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    absl::SetMinLogLevel((absl::LogSeverityAtLeast)absl::LogSeverity::kInfo);

    if(FLAGS_seq_name == "")
        LOG(FATAL) << "Sequence name not provided.";

    if(FLAGS_start_frame_id == -1)
        LOG(FATAL) << "Start frame id not provided.";

    LOG(INFO) << "Sequence name: " << FLAGS_seq_name;
    LOG(INFO) << "Number of captures: " << FLAGS_ncaptures;
    LOG(INFO) << "FPS: " << FLAGS_fps;
    LOG(INFO) << "Color bitrate: " << FLAGS_color_bitrate;
    LOG(INFO) << "Depth bitrate: " << FLAGS_depth_bitrate;
    LOG(INFO) << "Start frame id: " << FLAGS_start_frame_id;

    if(!check_webrtc_plugins())
        LOG(FATAL) << "WebRTC plugins not found.";

    int start_frame_id = FLAGS_start_frame_id;
    int end_frame_id = FLAGS_start_frame_id + FLAGS_ncaptures - 1;
    std::string seq_name = FLAGS_seq_name;
    std::string in_dir = SRC_PATH;

    if(!fs::exists(in_dir))
        LOG(FATAL) << "Input directory does not exist: " << in_dir;

    std::string out_dir = FORMAT(OUTDIR_BASE_PATH << FLAGS_seq_name << "/test");
    
    if(fs::exists(out_dir))
        fs::remove_all(out_dir);

    if(fs::create_directories(out_dir))
        LOG(INFO) << "Output directory created: " << out_dir;
    else
        LOG(FATAL) << "Failed to create output directory: " << out_dir;

    int shape[3] = {512 * 2, 592 * 5};

    // Thread for color stream
    std::vector<std::thread> thread_pool;
    TilingEncoderGPU server_color("c", shape, seq_name, start_frame_id, end_frame_id, in_dir, out_dir, FLAGS_fps, FLAGS_color_bitrate, FLAGS_depth_bitrate);
    std::thread t_color(&TilingEncoderGPU::start_pipeline, &server_color);
    thread_pool.push_back(std::move(t_color));

    // Thread to mimick depth stream by running a parallel color stream
    TilingEncoderGPU server_depth("c", shape, seq_name, start_frame_id, end_frame_id, in_dir, out_dir, FLAGS_fps, FLAGS_color_bitrate, FLAGS_depth_bitrate);
    std::thread t_depth(&TilingEncoderGPU::start_pipeline, &server_depth);
    thread_pool.push_back(std::move(t_depth));

    for(auto &t : thread_pool)
        t.join();

    LOG(INFO) << "GMain loop started";
    g_main_loop_run(loop);
    gst_object_unref(loop);

    LOG(INFO) << "WebRTC server stopped";
}

void untiled_GPU(int argc, char *argv[])
{
    gst_init(&argc, &argv);
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);

    gflags::SetUsageMessage("Usage: sender_webrtc_only [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    absl::SetMinLogLevel((absl::LogSeverityAtLeast)absl::LogSeverity::kInfo);

    if(FLAGS_seq_name == "")
        LOG(FATAL) << "Sequence name not provided.";

    if(FLAGS_start_frame_id == -1)
        LOG(FATAL) << "Start frame id not provided.";

    LOG(INFO) << "Sequence name: " << FLAGS_seq_name;
    LOG(INFO) << "Number of captures: " << FLAGS_ncaptures;
    LOG(INFO) << "FPS: " << FLAGS_fps;
    LOG(INFO) << "Color bitrate: " << FLAGS_color_bitrate;
    LOG(INFO) << "Depth bitrate: " << FLAGS_depth_bitrate;
    LOG(INFO) << "Start frame id: " << FLAGS_start_frame_id;

    if(!check_webrtc_plugins())
        LOG(FATAL) << "WebRTC plugins not found.";

    int start_frame_id = FLAGS_start_frame_id;
    int end_frame_id = FLAGS_start_frame_id + FLAGS_ncaptures - 1;
    std::string seq_name = FLAGS_seq_name;
    std::string in_dir = SRC_PATH;

    if(!fs::exists(in_dir))
        LOG(FATAL) << "Input directory does not exist: " << in_dir;

    std::string out_dir = FORMAT(OUTDIR_BASE_PATH << FLAGS_seq_name << "/test");
    
    if(fs::exists(out_dir))
        fs::remove_all(out_dir);

    if(fs::create_directories(out_dir))
        LOG(INFO) << "Output directory created: " << out_dir;
    else
        LOG(FATAL) << "Failed to create output directory: " << out_dir;

    int shape[3] = {512, 592};

    // Thread for color stream
    std::vector<std::thread> thread_pool;
    TilingEncoderGPU server_color("c", shape, seq_name, start_frame_id, end_frame_id, in_dir, out_dir, FLAGS_fps, FLAGS_color_bitrate, FLAGS_depth_bitrate);
    std::thread t_color(&TilingEncoderGPU::start_pipeline, &server_color);
    thread_pool.push_back(std::move(t_color));

    // Thread to mimick depth stream by running a parallel color stream
    TilingEncoderGPU server_depth("c", shape, seq_name, start_frame_id, end_frame_id, in_dir, out_dir, FLAGS_fps, FLAGS_color_bitrate, FLAGS_depth_bitrate);
    std::thread t_depth(&TilingEncoderGPU::start_pipeline, &server_depth);
    thread_pool.push_back(std::move(t_depth));

    for(auto &t : thread_pool)
        t.join();

    LOG(INFO) << "GMain loop started";
    g_main_loop_run(loop);
    gst_object_unref(loop);

    LOG(INFO) << "WebRTC server stopped";
}

void tiled_CPU(int argc, char *argv[])
{
    gst_init(&argc, &argv);
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);

    gflags::SetUsageMessage("Usage: sender_webrtc_only [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    absl::SetMinLogLevel((absl::LogSeverityAtLeast)absl::LogSeverity::kInfo);

    if(FLAGS_seq_name == "")
        LOG(FATAL) << "Sequence name not provided.";

    if(FLAGS_start_frame_id == -1)
        LOG(FATAL) << "Start frame id not provided.";

    LOG(INFO) << "Sequence name: " << FLAGS_seq_name;
    LOG(INFO) << "Number of captures: " << FLAGS_ncaptures;
    LOG(INFO) << "FPS: " << FLAGS_fps;
    LOG(INFO) << "Color bitrate: " << FLAGS_color_bitrate;
    LOG(INFO) << "Depth bitrate: " << FLAGS_depth_bitrate;
    LOG(INFO) << "Start frame id: " << FLAGS_start_frame_id;

    if(!check_webrtc_plugins())
        LOG(FATAL) << "WebRTC plugins not found.";

    int start_frame_id = FLAGS_start_frame_id;
    int end_frame_id = FLAGS_start_frame_id + FLAGS_ncaptures - 1;
    std::string seq_name = FLAGS_seq_name;
    std::string in_dir = SRC_PATH;

    if(!fs::exists(in_dir))
        LOG(FATAL) << "Input directory does not exist: " << in_dir;

    std::string out_dir = FORMAT(OUTDIR_BASE_PATH << FLAGS_seq_name << "/test");
    
    if(fs::exists(out_dir))
        fs::remove_all(out_dir);

    if(fs::create_directories(out_dir))
        LOG(INFO) << "Output directory created: " << out_dir;
    else
        LOG(FATAL) << "Failed to create output directory: " << out_dir;

    int shape[3] = {512 * 2, 592 * 5};

    // Thread for color stream
    std::vector<std::thread> thread_pool;
    TilingEncoderCPU server_color("c", shape, seq_name, start_frame_id, end_frame_id, in_dir, out_dir, FLAGS_fps, FLAGS_color_bitrate, FLAGS_depth_bitrate);
    std::thread t_color(&TilingEncoderCPU::start_pipeline, &server_color);
    thread_pool.push_back(std::move(t_color));

    // Thread to mimick depth stream by running a parallel color stream
    TilingEncoderCPU server_depth("c", shape, seq_name, start_frame_id, end_frame_id, in_dir, out_dir, FLAGS_fps, FLAGS_color_bitrate, FLAGS_depth_bitrate);
    std::thread t_depth(&TilingEncoderCPU::start_pipeline, &server_depth);
    thread_pool.push_back(std::move(t_depth));

    for(auto &t : thread_pool)
        t.join();

    LOG(INFO) << "GMain loop started";
    g_main_loop_run(loop);
    gst_object_unref(loop);

    LOG(INFO) << "WebRTC server stopped";
}

void untiled_CPU(int argc, char *argv[])
{
    gst_init(&argc, &argv);
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);

    gflags::SetUsageMessage("Usage: sender_webrtc_only [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    absl::SetMinLogLevel((absl::LogSeverityAtLeast)absl::LogSeverity::kInfo);

    if(FLAGS_seq_name == "")
        LOG(FATAL) << "Sequence name not provided.";

    if(FLAGS_start_frame_id == -1)
        LOG(FATAL) << "Start frame id not provided.";

    LOG(INFO) << "Sequence name: " << FLAGS_seq_name;
    LOG(INFO) << "Number of captures: " << FLAGS_ncaptures;
    LOG(INFO) << "FPS: " << FLAGS_fps;
    LOG(INFO) << "Color bitrate: " << FLAGS_color_bitrate;
    LOG(INFO) << "Depth bitrate: " << FLAGS_depth_bitrate;
    LOG(INFO) << "Start frame id: " << FLAGS_start_frame_id;

    if(!check_webrtc_plugins())
        LOG(FATAL) << "WebRTC plugins not found.";

    int start_frame_id = FLAGS_start_frame_id;
    int end_frame_id = FLAGS_start_frame_id + FLAGS_ncaptures - 1;
    std::string seq_name = FLAGS_seq_name;
    std::string in_dir = SRC_PATH;

    if(!fs::exists(in_dir))
        LOG(FATAL) << "Input directory does not exist: " << in_dir;

    std::string out_dir = FORMAT(OUTDIR_BASE_PATH << FLAGS_seq_name << "/test");
    
    if(fs::exists(out_dir))
        fs::remove_all(out_dir);

    if(fs::create_directories(out_dir))
        LOG(INFO) << "Output directory created: " << out_dir;
    else
        LOG(FATAL) << "Failed to create output directory: " << out_dir;

    int shape[3] = {512, 592};

    // Thread for color stream
    std::vector<std::thread> thread_pool;
    TilingEncoderCPU server_color("c", shape, seq_name, start_frame_id, end_frame_id, in_dir, out_dir, FLAGS_fps, FLAGS_color_bitrate, FLAGS_depth_bitrate);
    std::thread t_color(&TilingEncoderCPU::start_pipeline, &server_color);
    thread_pool.push_back(std::move(t_color));

    // Thread to mimick depth stream by running a parallel color stream
    TilingEncoderCPU server_depth("c", shape, seq_name, start_frame_id, end_frame_id, in_dir, out_dir, FLAGS_fps, FLAGS_color_bitrate, FLAGS_depth_bitrate);
    std::thread t_depth(&TilingEncoderCPU::start_pipeline, &server_depth);
    thread_pool.push_back(std::move(t_depth));

    for(auto &t : thread_pool)
        t.join();

    LOG(INFO) << "GMain loop started";
    g_main_loop_run(loop);
    gst_object_unref(loop);

    LOG(INFO) << "WebRTC server stopped";
}

void untiled_CPU_parallel(int argc, char *argv[])
{
    gst_init(&argc, &argv);
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);

    gflags::SetUsageMessage("Usage: sender_webrtc_only [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    absl::SetMinLogLevel((absl::LogSeverityAtLeast)absl::LogSeverity::kInfo);

    if(FLAGS_seq_name == "")
        LOG(FATAL) << "Sequence name not provided.";

    if(FLAGS_start_frame_id == -1)
        LOG(FATAL) << "Start frame id not provided.";

    LOG(INFO) << "Sequence name: " << FLAGS_seq_name;
    LOG(INFO) << "Number of captures: " << FLAGS_ncaptures;
    LOG(INFO) << "FPS: " << FLAGS_fps;
    LOG(INFO) << "Color bitrate: " << FLAGS_color_bitrate;
    LOG(INFO) << "Depth bitrate: " << FLAGS_depth_bitrate;
    LOG(INFO) << "Start frame id: " << FLAGS_start_frame_id;

    if(!check_webrtc_plugins())
        LOG(FATAL) << "WebRTC plugins not found.";

    int start_frame_id = FLAGS_start_frame_id;
    int end_frame_id = FLAGS_start_frame_id + FLAGS_ncaptures - 1;
    std::string seq_name = FLAGS_seq_name;
    std::string in_dir = SRC_PATH;

    if(!fs::exists(in_dir))
        LOG(FATAL) << "Input directory does not exist: " << in_dir;

    std::string out_dir = FORMAT(OUTDIR_BASE_PATH << FLAGS_seq_name << "/test");
    
    if(fs::exists(out_dir))
        fs::remove_all(out_dir);

    if(fs::create_directories(out_dir))
        LOG(INFO) << "Output directory created: " << out_dir;
    else
        LOG(FATAL) << "Failed to create output directory: " << out_dir;

    int shape[3] = {512, 592};

    std::vector<TilingEncoderCPUParallel *> server_colors(10);
    std::vector<TilingEncoderCPUParallel *> server_depths(10);
    std::vector<std::thread> thread_pool;
    for(int view_id = 0; view_id < 10; view_id++)
    {
        // Color stream
        server_colors[view_id] = new TilingEncoderCPUParallel("c", shape, seq_name, start_frame_id, end_frame_id, view_id, in_dir, out_dir, FLAGS_fps, FLAGS_color_bitrate, FLAGS_depth_bitrate);
        std::thread t_color(&TilingEncoderCPUParallel::start_pipeline, server_colors[view_id]);
        thread_pool.push_back(std::move(t_color));
        
        // Depth stream to mimick color stream
        server_depths[view_id] = new TilingEncoderCPUParallel("c", shape, seq_name, start_frame_id, end_frame_id, view_id, in_dir, out_dir, FLAGS_fps, FLAGS_color_bitrate, FLAGS_depth_bitrate);
        std::thread t_depth(&TilingEncoderCPUParallel::start_pipeline, server_depths[view_id]);
        thread_pool.push_back(std::move(t_depth));
    }

    for(auto &t : thread_pool)
        t.join();

    LOG(INFO) << "GMain loop started";
    g_main_loop_run(loop);
    gst_object_unref(loop);

    LOG(INFO) << "WebRTC server stopped";
}

int main(int argc, char *argv[])
{
    // tiled_GPU(argc, argv);
    // untiled_GPU(argc, argv);

    // tiled_CPU(argc, argv);
    // untiled_CPU(argc, argv);

    untiled_CPU_parallel(argc, argv);
    return 0;
}