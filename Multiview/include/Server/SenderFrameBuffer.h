#pragma once
#include <string>
#include <vector>
#include <boost/filesystem/fstream.hpp>
#include "view.h"

using namespace std;
namespace fs = boost::filesystem;

void add_qr_bgra(RGB *buf, int width, int height, const std::string &qr_folder, int frameID, bool addText = false);
void add_qr_yuv16(cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], const std::string &qr_folder, int frameID, bool isScaled=true, bool addText=false);
void depth_to_yuv16(uint16_t *depth, cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], int height, int width, bool isScaled=true);
void flatten_depth_yuv16(const cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], uint8_t *depth_yuv16_buf, int height, int width);

class SenderFrameBuffer {
public:
    SenderFrameBuffer(const std::string &type, const uint32_t max_elem, const uint32_t elem_size);
    ~SenderFrameBuffer();
    bool insert_cframe(const vector<View *> &views, bool isSaveView=false, std::string save_dir="");
    bool insert_dframe(const vector<View *> &views, RGB **colorized_depth, bool isSaveView=false, std::string save_dir="");
    bool insert_dframe_yuv16(const vector<View *> views, cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], uint8_t **depth_yuv16_buf, bool isSaveView=false, std::string save_dir="");
    bool remove_frame(uint8_t *data, uint32_t data_size, int &frameID);

private:
    bool copy_in_gst_format_color(uint8_t *buf, const vector<View *> views, bool isSaveView=false, std::string save_dir="");
    bool copy_in_gst_format_depth(uint8_t *buf, const vector<View *> views, RGB **colorized_depth, bool isSaveView=false, std::string save_dir="");
    bool copy_in_gst_format_depth_yuv16(uint8_t *buf, const vector<View *> views, cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], uint8_t **depth_yuv16_buf, bool isSaveView=false, std::string save_dir="");

    uint8_t *buffer;                    // Pointer to the buffer memory
    int *frameIds;                      // Frame IDs of the frames in the buffer

    int head, tail;                     // Head and tail indices (non-atomic)
    std::atomic<int> size{0};           // Atomic size (number of valid elements)

	uint32_t max_elem;
	uint32_t elem_size;
	uint32_t max_buf_size;

    std::string type;
    fs::ofstream file_webrtc;
    StopWatch sw;
};