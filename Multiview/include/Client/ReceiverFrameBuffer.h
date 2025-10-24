#pragma once
#include <string>
#include <vector>
#include <boost/filesystem/fstream.hpp>
#include "view.h"

enum REMOVE_STATUS
{
    REMOVE_SUCCESS = 0,
    REMOVE_FAIL = 1,
    REMOVE_NOT_FOUND = 2
};

static const char * remove_status2str[] = { "REMOVE_SUCCESS", "REMOVE_FAIL", "REMOVE_NOT_FOUND"};

namespace fs = boost::filesystem;

class ReceiverFrameBuffer
{
public:
    ReceiverFrameBuffer(const std::string &type, const uint32_t max_elem, const uint32_t elem_size);
    ~ReceiverFrameBuffer();
    bool insert_cframe(uint8_t *data, uint32_t data_size, int w, int h, int &frameID);
    bool insert_dframe(uint8_t *data, uint32_t data_size, int w, int h, int &frameID);
    bool insert_dframe_yuv16(uint8_t *data, uint32_t data_size, int w, int h, int &frameID);
    REMOVE_STATUS remove_frame(uint8_t *data, uint32_t data_size, int requested_frameID, int &removed_frameID);
    int get_latest_frame_id();
private:
    void yuv16_to_depth(uint16_t *depth, int height, int width, bool isScaled=true);
    int detect_framenum_qrcode(RGB *buf, int width, int height, int border=4*2, bool rmCode=true);
    int detect_framenum_qrcode_yuv16(uint8_t *buf, int width, int height, int border=4*2, bool isScaled=true);

    uint8_t *buffer;                        // Pointer to the buffer memory
    int *frameIds;                          // Frame IDs of the frames in the buffer
    std::atomic<int> latest_frame_id{-1};   // Latest frame ID

    int head, tail;                         // Head and tail indices (non-atomic)
    std::atomic<int> size{0};               // Atomic size (number of valid elements)

    uint32_t max_elem;
	uint32_t elem_size;
	uint32_t max_buf_size;

    std::string type;
    cv::QRCodeDetector m_qrDecoder;
    cv::Rect m_rectZero;

    fs::ofstream file_webrtc;
    StopWatch sw;
};