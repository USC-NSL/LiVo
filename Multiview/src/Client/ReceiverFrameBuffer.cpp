#include "Client/ReceiverFrameBuffer.h"
#include <omp.h>
#include "utils.h"

static const int QR_BOX[4] = {20, 20, 82, 82}; // x, y, w, h

ReceiverFrameBuffer::ReceiverFrameBuffer(const std::string &type, const uint32_t max_elem, const uint32_t elem_size) : head(0), tail(0), max_elem(max_elem), elem_size(elem_size), 
m_qrDecoder(), m_rectZero(QR_BOX[0], QR_BOX[1], QR_BOX[2], QR_BOX[3]), type(type)
{
    max_buf_size = max_elem * elem_size;
    latest_frame_id.store(-1, std::memory_order_release);

    // Preallocate memory for the buffer
    buffer = new uint8_t[max_buf_size];
    frameIds = new int[max_elem];
    std::fill(frameIds, frameIds + max_elem, -1);

    // std::string method = "livo_nocull";
	std::string method = "livo";		// TODO: Rajrup: Changee this to livo.

	std::string path = FORMAT("/datassd/pipeline_cpp/server_tiled/e2e_latency/" << method << "/"); 
	create_folder(path);

    fs::path filepath_webrtc{FORMAT(path << "0_" << type << "_webrtc_receiver_timestamp.txt")};
    file_webrtc.open(filepath_webrtc, ios::out | ios::trunc);
    file_webrtc << "FrameID,Time" << endl;
}

ReceiverFrameBuffer::~ReceiverFrameBuffer()
{
    delete[] buffer;
    delete[] frameIds;
}

/**
 * @brief In-place convert yuv16 to depth
 * 
 * @param buf 
 * @param height 
 * @param width 
 * @param isScaled Unscale y channel if true
 */
void ReceiverFrameBuffer::yuv16_to_depth(uint16_t *depth, int height, int width, bool isScaled)
{
    // TODO: Rajrup: Can be parallelized
    if(isScaled)
    {
        for(int i=0; i<height*width; i++)
        {
            float d = (depth[i] * 6000.0F) /  65535.0F;
            depth[i] = uint16_t(d);
        }
    }
}

int ReceiverFrameBuffer::detect_framenum_qrcode(RGB *buf, int width, int height, int border, bool rmCode)
{
    cv::Mat img = cv::Mat(height, width, CV_8UC4, buf);

	cv::Mat tmp;
	cv::extractChannel(img, tmp, 0);    // Only check for blue channel. If required, we can test for all channels, but its time consuming.
	cv::Mat qrcode_img = tmp(cv::Range(QR_BOX[0], QR_BOX[0] + QR_BOX[2]), cv::Range(QR_BOX[1], QR_BOX[1] + QR_BOX[3]));

	// LOG(INFO) << "-----------------------------------------------";
	// Detect qr code in each channel

	vector<cv::Point> corners;
	corners.push_back(cv::Point(border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, qrcode_img.rows - border));
	corners.push_back(cv::Point(border, qrcode_img.rows - border));

	// string decode_info = m_qrDecoder.detectAndDecode(qrcode_img, corners);
	string decode_info = m_qrDecoder.decode(qrcode_img, corners);
	// LOG(INFO) << "Frame_number: " << decode_info;

    int frame_id = -1;
	if(decode_info.length() > 0)
		frame_id = stoi(decode_info);

    // Remove QR code from image
    if(rmCode)
        cv::rectangle(img, m_rectZero, cv::Scalar(0, 0, 0, 255), cv::FILLED); // Remove QR code from image in-place, so the buffer contains the image without QR code

	return frame_id;
}

int ReceiverFrameBuffer::detect_framenum_qrcode_yuv16(uint8_t *buf, int width, int height, int border, bool isScaled)
{
    cv::Mat img_u = cv::Mat(height, width, CV_16UC1, buf);
	cv::Mat qrcode_img = cv::Mat::zeros(QR_BOX[2], QR_BOX[3], CV_8UC1);

    for(int i=0; i<qrcode_img.rows; i++)
    {
        for(int j=0; j<qrcode_img.cols; j++)
        {
            float pixel = (img_u.at<uint16_t>(i + QR_BOX[0], j + QR_BOX[1]) * 255.0F) / 65535.0F;
            qrcode_img.at<uint8_t>(i,j) = uint8_t(pixel);
        }
    }

	// LOG(INFO) << "-----------------------------------------------";
	// Detect qr code in u channel

	vector<cv::Point> corners;
	corners.push_back(cv::Point(border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, border));
	corners.push_back(cv::Point(qrcode_img.cols - border, qrcode_img.rows - border));
	corners.push_back(cv::Point(border, qrcode_img.rows - border));

	// string decode_info = m_qrDecoder.detectAndDecode(qrcode_img, corners);
	string decode_info = m_qrDecoder.decode(qrcode_img, corners);
	// LOG(INFO) << "Frame_number: " << decode_info;
	if(decode_info.length() > 0)
	{
		int32_t frame_id = stoi(decode_info);
		return frame_id;
	}

	return -1;
}

int ReceiverFrameBuffer::get_latest_frame_id()
{
    return latest_frame_id.load(std::memory_order_acquire);
}

/**
 * @brief Insert data into the buffer
 * 
 * @param data 
 * @param data_size 
 * @param w             Width of tiled frame 
 * @param h             Height of tiled frame
 */
bool ReceiverFrameBuffer::insert_cframe(uint8_t *data, uint32_t data_size, int w, int h, int &frameID) 
{
    // Atomically check if the buffer has space (non-blocking)
    if(size.load(std::memory_order_acquire) == max_elem)
        return false; // Buffer is full
    
    // Copy the data to the buffer
    if(data_size != elem_size)
    {
        LOG(ERROR) << "Type - " << type << " : Buffer element size mismatch: " << elem_size << " != " << data_size;
        return false;
    }

    RGB *color_buf = reinterpret_cast<RGB *>(data);
    int cframeID = detect_framenum_qrcode(color_buf, w, h);         // cframeID == -1 if no QR code detected
    frameID = cframeID;
    
    // Debug
    // if(cframeID % 10 == 0)
    // {
    //     cv::Mat res(h, w, CV_8UC(CHANNEL_DIM::BGRA), data);
    //     cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << cframeID << "_color_gst.png"), res);
    // }

    if(cframeID == -1)
    {
        LOG(WARNING) << "Type - " << type << " : Failed to detect QR code in color frame. Expected frame ID: " << latest_frame_id.load() + 1;
        cv::Mat res(h, w, CV_8UC(CHANNEL_DIM::BGRA), data);
        // cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << latest_frame_id.load() + 1 << "_color_gst.png"), res);
        return false;
    }
    file_webrtc << cframeID << "," << sw.Curr() << endl;

    // Copy the data to the buffer
    memcpy(&buffer[tail * elem_size], data, elem_size);
    frameIds[tail] = cframeID;
    
    // Update the tail position (circular behavior)
    tail = (tail + 1) % max_elem;

    LOG(INFO) << "Type - " << type << " : Inserted " << type << " frame ID " << cframeID << ", #Elems: " << size.load() + 1;

    // Atomically store the latest frame ID
    latest_frame_id.store(cframeID, std::memory_order_release);

    // Atomically increase the size
    size.fetch_add(1, std::memory_order_release);

    return true;
}

bool ReceiverFrameBuffer::insert_dframe(uint8_t *data, uint32_t data_size, int w, int h, int &frameID)
{
    // Atomically check if the buffer has space (non-blocking)
    if(size.load(std::memory_order_acquire) == max_elem)
        return false; // Buffer is full
    
    // Copy the data to the buffer
    if(data_size != elem_size)
    {
        LOG(ERROR) << "Type - " << type << " : Buffer element size mismatch: " << elem_size << " != " << data_size;
        return false;
    }

    RGB *colorized_depth_buf = reinterpret_cast<RGB *>(data);
    int dframeID = detect_framenum_qrcode(colorized_depth_buf, w, h);   // dframeID == -1 if no QR code detected
    frameID = dframeID;

    if(dframeID == -1)
    {
        LOG(WARNING) << "Type - " << type << " : Failed to detect QR code in depth frame. Expected frame ID: " << latest_frame_id.load() + 1;
        cv::Mat res(h, w, CV_8UC(CHANNEL_DIM::BGRA), data);
        cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << latest_frame_id.load() + 1 << "_depth_gst.png"), res);
        return false;
    }
    file_webrtc << dframeID << "," << sw.Curr() << endl;

    // Copy the data to the buffer
    memcpy(&buffer[tail * elem_size], data, elem_size);
    frameIds[tail] = dframeID;

    // Update the tail position (circular behavior)
    tail = (tail + 1) % max_elem;

    LOG(INFO) << "Type - " << type << " : Inserted " << type << " frame ID " << dframeID << ", #Elems: " << size.load() + 1;

    // Atomically store the latest frame ID
    latest_frame_id.store(dframeID, std::memory_order_release);

    // Atomically increase the size
    size.fetch_add(1, std::memory_order_release);

    return true;
}

bool ReceiverFrameBuffer::insert_dframe_yuv16(uint8_t *data, uint32_t data_size, int w, int h, int &frameID)
{
    // Atomically check if the buffer has space (non-blocking)
    if(size.load(std::memory_order_acquire) == max_elem)
        return false; // Buffer is full
    
    // Copy the data to the buffer
    if(elem_size != (w * h * PIXEL_SIZE::DEPTH))
    {
        LOG(ERROR) << "Type - " << type << " : Buffer element size mismatch: " << elem_size << " != " << (w * h * PIXEL_SIZE::DEPTH);
        return false;
    }

    // Check if the data size is correct, which has y, u, v channels
    if(data_size != elem_size * CHANNEL_DIM::DEPTH)
    {
        LOG(ERROR) << "Type - " << type << " : Buffer element size mismatch: " << elem_size * CHANNEL_DIM::DEPTH << " != " << data_size;
        return false;
    }

    uint16_t *buf_y = reinterpret_cast<uint16_t *>(data);
    yuv16_to_depth(buf_y, h, w);    // Unscale y channel

    uint8_t *buf_u = data + (w * h * PIXEL_SIZE::DEPTH); // u channel
    int dframeID = detect_framenum_qrcode_yuv16(buf_u, w, h);   // dframeID == -1 if no QR code detected
    frameID = dframeID;
    
    // Debug
    // if(dframeID % 10 == 0)
    // {
    //     cv::Mat channel_y = cv::Mat(h, w, CV_16UC1, buf_y);
    //     cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << dframeID << "_depth_gst_y.png"), channel_y);

    //     cv::Mat channel_u = cv::Mat(h, w, CV_16UC1, buf_u);
    //     cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/" << dframeID << "_depth_gst_u.png"), channel_u);
    // }

    if(dframeID == -1)
    {
        LOG(WARNING) << "Type - " << type << " : Failed to detect QR code in depth frame. Expected frame ID: " << latest_frame_id.load() + 1;
        cv::Mat channel_y = cv::Mat(h, w, CV_16UC1, data);
        cv::Mat channel_u = cv::Mat(h, w, CV_16UC1, data + channel_y.total() * channel_y.elemSize());
        cv::Mat channel_v = cv::Mat(h, w, CV_16UC1, data + channel_y.total() * channel_y.elemSize() + channel_u.total() * channel_u.elemSize());
        // cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/"<< latest_frame_id.load() + 1 << "_depth_gst_y.png"), channel_y);
        // cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/"<< latest_frame_id.load() + 1 << "_depth_gst_u.png"), channel_u);
        // cv::imwrite(FORMAT("/home/lei/data/pipeline/client_tiled/pipeline_new/test/"<< latest_frame_id.load() + 1 << "_depth_gst_v.png"), channel_v);
        return false;
    }
    file_webrtc << dframeID << "," << sw.Curr() << endl;

    // Copy the data to the buffer
    memcpy(&buffer[tail * elem_size], data, elem_size); // Only copy the y channel
    frameIds[tail] = dframeID;

    // Update the tail position (circular behavior)
    tail = (tail + 1) % max_elem;

    LOG(INFO) << "Type - " << type << " : Inserted " << type << " frame ID " << dframeID << ", #Elems: " << size.load() + 1;

    // Atomically store the latest frame ID
    latest_frame_id.store(dframeID, std::memory_order_release);

    // Atomically increase the size
    size.fetch_add(1, std::memory_order_release);

    return true;
}

/**
 * @brief Remove data from the buffer till the requested frame ID
 * Stops at the requested frame ID. 
 * @param data Pass the buffer after allocating memory for data_size
 * @param data_size 
 * @param requested_frameID 
 * @param removed_frameID -1, if something went wrong, or returns the removed frame ID
 * @return REMOVE_SUCCESS - Frame matched and removed, REMOVE_FAIL - empty or other, REMOVE_NOT_FOUND - Frame not found, possibly skipped (due to loss, qr loss or other reasons)
 */
REMOVE_STATUS ReceiverFrameBuffer::remove_frame(uint8_t *data, uint32_t data_size, int requested_frameID, int &removed_frameID) 
{
    removed_frameID = -1;

    // Atomically check if the buffer is empty (non-blocking)
    if (size.load(std::memory_order_acquire) == 0) {
        return REMOVE_STATUS::REMOVE_FAIL; // Buffer is empty
    }

    if(data_size != elem_size)
    {
        LOG(ERROR) << "Type - " << type << ": Buffer element size mismatch: " << elem_size << " != " << data_size;
        return REMOVE_STATUS::REMOVE_FAIL;
    }

    if(frameIds[head] == -1)
    {
        LOG(ERROR) << "Type - " << type << ": Frame ID is -1. Expected frame ID: " << requested_frameID;
        return REMOVE_STATUS::REMOVE_FAIL;
    }

    if(requested_frameID < frameIds[head])
    {
        LOG(WARNING) << "Type - " << type << ": Requested frame ID: " << requested_frameID << " is less than the head frame ID: " << frameIds[head];
        return REMOVE_STATUS::REMOVE_NOT_FOUND;
    }

    // Copy the data from the buffer
    memcpy(data, &buffer[head * elem_size], elem_size);
    removed_frameID = frameIds[head];

    // Update the head position (circular behavior)
    head = (head + 1) % max_elem;

    LOG(INFO) << "Type - " << type << ": Removed " << type << " frame ID " << removed_frameID << ", #Elems: " << size.load() - 1;

    // Atomically decrease the size
    size.fetch_sub(1, std::memory_order_release);

    return REMOVE_STATUS::REMOVE_SUCCESS;
}