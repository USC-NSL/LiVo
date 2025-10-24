#include "Server/SenderFrameBuffer.h"
#include <omp.h>
#include "utils.h"
#include "colorized_depth.h"

// Utilities
void add_qr_bgra(RGB *buf, int width, int height, const std::string &qr_folder, int frameID, bool addText)
{
	// Adding QR code to 1st view
	cv::Mat qr_code = cv::imread(FORMAT(qr_folder << frameID << "_qrcode.png"), cv::IMREAD_GRAYSCALE);
    if(qr_code.empty())
    {
        LOG(FATAL) << "QR code image not found!";
        return;
    }
	cv::Mat img_view0 = color_to_opencv(buf, width, height);

	int offset_x, offset_y;
	offset_x = 20;
	offset_y = 20;
	for(int i = 0; i < qr_code.rows; i++)
	{
		for(int j = 0; j < qr_code.cols; j++)
		{
			assert(i+offset_y < img_view0.rows && j+offset_x < img_view0.cols);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[0] = qr_code.at<uint8_t>(i, j);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[1] = qr_code.at<uint8_t>(i, j);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[2] = qr_code.at<uint8_t>(i, j);
			img_view0.at<cv::Vec4b>(i + offset_y, j + offset_x)[3] = 255;
		}
	}

    if(addText)
    {
        // Adding text to 1st view
        int text_offset_x, text_offset_y;
        text_offset_x = offset_x + qr_code.cols + 200;
        text_offset_y = offset_y + qr_code.rows;
        cv::putText(img_view0, FORMAT(frameID), cv::Point(text_offset_x, text_offset_y), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(0, 255, 0, 255), 8);
        LOG(INFO) << "Added QR text to color BGRA image";
    }

	memcpy(buf, &img_view0.ptr<cv::Vec4b>(0)[0], img_view0.rows * img_view0.cols * sizeof(cv::Vec4b));
}

/**
 * @brief Adds QR code to YUV16 depth image in UV channel.
 * It assumes UV channel have constant values.
 * @param depth_yuv16_cv 
 * @param qr_folder 
 * @param frameID 
 * @param isScaled 
 * @param addText 
 */
void add_qr_yuv16(cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], const std::string &qr_folder, int frameID, bool isScaled, bool addText)
{
    // Adding QR code to 1st view
	cv::Mat qr_code = cv::imread(FORMAT(qr_folder << frameID << "_qrcode.png"), cv::IMREAD_GRAYSCALE);
    if(qr_code.empty())
    {
        LOG(FATAL) << "QR code image not found!";
        return;
    }

    int offset_x, offset_y;
    offset_x = 20;
    offset_y = 20;
    for(int i=0; i<qr_code.rows; i++)
    {
        for(int j=0; j<qr_code.cols; j++)
        {
            assert(i+offset_y < depth_yuv16_cv.rows && j+offset_x < depth_yuv16_cv.cols);
            if(isScaled)
            {
                float pixel = (qr_code.at<uint8_t>(i, j) * 65535.0F) / 255.0F ;
                depth_yuv16_cv[1].at<uint16_t>(i + offset_y, j + offset_x) = uint16_t(pixel);
                depth_yuv16_cv[2].at<uint16_t>(i + offset_y, j + offset_x) = uint16_t(pixel);
            }
            else
            {
                depth_yuv16_cv[1].at<uint16_t>(i + offset_y, j + offset_x) = qr_code.at<uint8_t>(i, j);
                depth_yuv16_cv[2].at<uint16_t>(i + offset_y, j + offset_x) = qr_code.at<uint8_t>(i, j);
            }
        }
    }

    if(addText)
    {
        // Adding text to 1st view
        int text_offset_x, text_offset_y;
        text_offset_x = offset_x + qr_code.cols + 200;
        text_offset_y = offset_y + qr_code.rows;

        for(int i=0; i<qr_code.rows + 20; i++)
        {
            for(int j=0; j<qr_code.cols + 100; j++)
            {
                depth_yuv16_cv[0].at<uint16_t>(i + offset_y, j + offset_x + 250) = 65535;
                depth_yuv16_cv[1].at<uint16_t>(i + offset_y, j + offset_x + 250) = 65535;
                depth_yuv16_cv[2].at<uint16_t>(i + offset_y, j + offset_x + 250) = 65535;
            }
        }

        cv::Mat temp;
        cv::merge(vector<cv::Mat>{depth_yuv16_cv[0], depth_yuv16_cv[1], depth_yuv16_cv[2]}, temp);
        cv::putText(temp, FORMAT(frameID), cv::Point(text_offset_x, text_offset_y), cv::FONT_HERSHEY_SIMPLEX, 3.0, cv::Scalar(255, 255, 255), 8);
        cv::split(temp, vector<cv::Mat>{depth_yuv16_cv[0], depth_yuv16_cv[1], depth_yuv16_cv[2]});
        LOG(INFO) << "Added QR text to depth YUV16 image";
    }
}

/**
 * @brief Converts 16-bit depth image to 16-bit YUV16 image.
 * Y channel has the depth image. U and V channels are set to constant.
 * Depth image is scaled to 
 * @param depth 
 * @param height 
 * @param width 
 * @param depth_yuv16_cv 3 channel 16-bit image in cv::Mat format CV_16UC3
 * @param isScaled Kinect depth range is 5 meters. We scale the depth between 0 - 6000 (max) to 0 - 65535 (max).
 */
void depth_to_yuv16(uint16_t *depth, cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], int height, int width, bool isScaled)
{
    // TODO: Rajrup: Can be parallelized
    for(int i=0; i<height; i++)
    {
        for(int j=0; j<width; j++)
        {
            if(isScaled)
            {
                float d = (depth[i * width + j] * 65535.0F) / 6000.0F;
                depth_yuv16_cv[0].at<uint16_t>(i, j) = uint16_t(d);
            }
            else
                depth_yuv16_cv[0].at<uint16_t>(i, j) = depth[i * width + j];
            
            depth_yuv16_cv[1].at<uint16_t>(i, j) = 32768;
            depth_yuv16_cv[2].at<uint16_t>(i, j) = 32768;
        }
    }
}

/**
 * @brief Flatten 16-bit YUV16 depth image to 8-bit buffer.
 * The flatten needs to follow gstreamer memory format for Y444_16LE.
 * Find Y444 layout here: https://gstreamer.freedesktop.org/documentation/additional/design/mediatype-video-raw.html?gi-language=c 
 * @param depth_yuv16_cv 
 * @param depth_yuv16_buf 
 */
void flatten_depth_yuv16(const cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], uint8_t *depth_yuv16_buf, int height, int width)
{
    for(int i=0; i<CHANNEL_DIM::DEPTH; i++)
    {
        memcpy(depth_yuv16_buf + (i * height * width * PIXEL_SIZE::DEPTH), depth_yuv16_cv[i].data, height * width * PIXEL_SIZE::DEPTH);
    }
}

SenderFrameBuffer::SenderFrameBuffer(const std::string &type, const uint32_t max_elem, const uint32_t elem_size) : head(0), tail(0), max_elem(max_elem), elem_size(elem_size), type(type)
{
    max_buf_size = max_elem * elem_size;

    // Preallocate memory for the buffer
    buffer = new uint8_t[max_buf_size];
    frameIds = new int[max_elem];
    std::fill(frameIds, frameIds + max_elem, -1);

    // std::string method = "livo_nocull";
	std::string method = "livo";		// TODO: Rajrup: Change this to livo

	std::string path = FORMAT("/datassd/pipeline_cpp/server_tiled/e2e_latency/" << method << "/"); 
	create_folder(path);

    fs::path filepath_webrtc{FORMAT(path << "0_" << type << "_webrtc_sender_timestamp.txt")};
    file_webrtc.open(filepath_webrtc, ios::out | ios::trunc);
    file_webrtc << "FrameID,Time" << endl;
}

SenderFrameBuffer::~SenderFrameBuffer() 
{
    delete[] buffer;
}

// Insert color data into the buffer
bool SenderFrameBuffer::insert_cframe(const vector<View *> &views, bool isSaveView, std::string save_dir) 
{
    // Atomically check if the buffer has space (non-blocking)
    if(size.load(std::memory_order_acquire) == max_elem)
        return false; // Buffer is full

    // Check if elem_size is size of 10 color views
    uint32_t data_size = views[0]->colorHeight * views[0]->colorWidth * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA * views.size();
    if(elem_size != data_size)
    {
        LOG(ERROR) << "Type - " << type << ": Buffer element size mismatch: " << elem_size << " != " << data_size;
        return false;
    }
    
    // Copy the data to the buffer
    if(!copy_in_gst_format_color(&buffer[tail * elem_size], views, isSaveView, save_dir))
        return false;
    frameIds[tail] = views[0]->frameID;

    // Update the tail position (circular behavior)
    tail = (tail + 1) % max_elem;

    LOG(INFO) << "Type - " << type << ": Produced data, #Elems: " << size.load() + 1;

    // Atomically increase the size
    size.fetch_add(1, std::memory_order_release);

    return true;
}

// Insert depth data into the buffer
bool SenderFrameBuffer::insert_dframe(const vector<View *> &views, RGB **colorized_depth, bool isSaveView, std::string save_dir)
{
    // Atomically check if the buffer has space (non-blocking)
    if(size.load(std::memory_order_acquire) == max_elem)
        return false; // Buffer is full

    // Check if elem_size is size of 10 depth bgra views
    uint32_t data_size = views[0]->colorHeight * views[0]->colorWidth * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA * views.size();
    if(elem_size != data_size)
    {
        LOG(ERROR) << "Type - " << type << ": Buffer element size mismatch: " << elem_size << " != " << data_size;
        return false;
    }

    // Copy the data to the buffer
    if(!copy_in_gst_format_depth(&buffer[tail * elem_size], views, colorized_depth, isSaveView, save_dir))
        return false;
    frameIds[tail] = views[0]->frameID;
    
    // Update the tail position (circular behavior)
    tail = (tail + 1) % max_elem;

    LOG(INFO) << "Type - " << type << ": Produced data, #Elems: " << size.load() + 1;

    // Atomically increase the size
    size.fetch_add(1, std::memory_order_release);

    return true;
}

// Insert depth yuv444 data into the buffer
bool SenderFrameBuffer::insert_dframe_yuv16(const vector<View *> views, cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], uint8_t **depth_yuv16_buf, bool isSaveView, std::string save_dir)
{
    // Atomically check if the buffer has space (non-blocking)
    if(size.load(std::memory_order_acquire) == max_elem)
        return false; // Buffer is full

    // Check if elem_size is size of 10 depth yuv16 views
    uint32_t data_size = views[0]->depthHeight * views[0]->depthWidth * CHANNEL_DIM::DEPTH * PIXEL_SIZE::DEPTH * views.size();
    if(elem_size != data_size)
    {
        LOG(ERROR) << "Type - " << type << ": Buffer element size mismatch: " << elem_size << " != " << data_size;
        return false;
    }

    // Copy the data to the buffer
    if(!copy_in_gst_format_depth_yuv16(&buffer[tail * elem_size], views, depth_yuv16_cv, depth_yuv16_buf, isSaveView, save_dir))
        return false;
    frameIds[tail] = views[0]->frameID;
    
    // Update the tail position (circular behavior)
    tail = (tail + 1) % max_elem;

    LOG(INFO) << "Type - " << type << ": Produced data, #Elems: " << size.load() + 1;

    // Atomically increase the size
    size.fetch_add(1, std::memory_order_release);

    return true;
}

// Remove data from the buffer
bool SenderFrameBuffer::remove_frame(uint8_t *data, uint32_t data_size, int &frameID)
{
    // Atomically check if the buffer is empty (non-blocking)
    if (size.load(std::memory_order_acquire) == 0) {
        frameID = -1;
        return false; // Buffer is empty
    }

    if(data_size != elem_size)
    {
        LOG(ERROR) << "Type - " << type << ": Buffer element size mismatch: " << elem_size << " != " << data_size;
        frameID = -1;
        return false;
    }

    // Copy the data from the buffer
    memcpy(data, &buffer[head * elem_size], elem_size);
    frameID = frameIds[head];
    file_webrtc << frameID << "," << sw.Curr() << endl;

    // Update the head position (circular behavior)
    head = (head + 1) % max_elem;

    LOG(INFO) << "Type - " << type << ": Consumed data, #Elems: " << size.load() - 1;

    // Atomically decrease the size
    size.fetch_sub(1, std::memory_order_release);

    return true;
}

/**
 * @brief Tiled color assumes that format is in BGRA. Add QR code to view 0.
 * 
 * @param buf 
 * @param views
 */
bool SenderFrameBuffer::copy_in_gst_format_color(uint8_t *buf, const vector<View *> views, bool isSaveView, std::string save_dir)
{
    if(views.size() != 10)
    {
        LOG(ERROR) << "Type - " << type << " :View count " << views.size() << " is not 10";
        return false;
    }
    int frame_id = views[0]->frameID;

    // #pragma omp parallel for
    for (int i=0; i < views.size(); i++) 
    {
        auto view = views[i];
        if(i == 0)  // Add QR code to view 0
        {
            add_qr_bgra(view->color_image, view->colorWidth, view->colorHeight, QRCODE_FOLDER, view->frameID);

            // DEBUG
            // add_qr_bgra(view->color_image, view->colorWidth, view->colorHeight, QRCODE_FOLDER, view->frameID, true);
        }

        uint8_t *color_data = reinterpret_cast<uint8_t *>(view->color_image);

        // //  Tile with 10*1 format (I think this is incorrect)
        // memcpy(shm_ptr + 10 * view->depthHeight * view->depthWidth * CHANNEL_DIM::BGRA * (curr_counter % SHM_BUFFER_SIZE) + view->viewID * view->depthHeight * view->depthWidth * CHANNEL_DIM::BGRA, color_buf[i], view->depthHeight * view->depthWidth * CHANNEL_DIM::BGRA);

        // Tile with 2*5 format
        auto h = view->colorHeight;
        auto w = view->colorWidth;
        auto id = view->viewID;

        // TODO: Rajrup: Replace 10 with view_count?
        /**
         * @brief The memory layout of shm_ptr is as follows:
         * 1st row of 5*2 layout is placed one after another in memory ||BRGA|BRGA| ---- w * 5 ----- |BRGA|BRGA||. Then comes the second row and so on, till 2 rows are filled.
         * This is done since the gstreamer memory layout expects the data to be in this format. 
         * Find BGRA layout here: https://gstreamer.freedesktop.org/documentation/additional/design/mediatype-video-raw.html?gi-language=c 
         */
        int x = id / 5 * h, y = id % 5 * (w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA);
        for (int j=0; j < h; j++)
            memcpy(
                    buf + ((x + j) * w * 5 * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA) + y, 
                    color_data + (j * w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA), 
                    w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA
                );
        // if (isSaveView && (frame_id % 10 == 0))
        // {
        //     if(!fs::exists(save_dir))
        //         fs::create_directories(save_dir);
        //     save_color(color_data, h, w, FORMAT(save_dir << frame_id << "_color_bgra_" << view->viewID << ".png"));
        // }
    }

    if (isSaveView && (frame_id % 10 == 0))
    {
        auto h = views[0]->colorHeight;
        auto w = views[0]->colorWidth;
        if(!fs::exists(save_dir))
            fs::create_directories(save_dir);
        save_color(buf, h * 2, w * 5, FORMAT(save_dir << frame_id << "_color_bgra.png"));
    }
    return true;
}
    
/**
 * @brief Tiled depth assumes that format is in BGRA. Add QR code to view 0.
 * 
 * @param buf 
 * @param views 
 * @param colorized_depth Used as the storage for storing colorized depth image temporarily. It's like scratch
 * @param isSaveView 
 * @param save_dir 
 * @return true 
 * @return false 
 */
bool SenderFrameBuffer::copy_in_gst_format_depth(uint8_t *buf, const vector<View *> views, RGB **colorized_depth, bool isSaveView, std::string save_dir)
{
    if(views.size() != 10)
    {
        LOG(ERROR) << "Type - " << type << " :View count " << views.size() << " is not 10";
        return false;
    }
    int frame_id = views[0]->frameID;

    // #pragma omp parallel for
    for (int i=0; i < views.size(); i++) 
    {
        auto view = views[i];
        depth_to_color(view->depth_image, view->depthHeight, view->depthWidth, colorized_depth[i], false);

        if(i==0)   
        {
            add_qr_bgra(colorized_depth[i], view->depthWidth, view->depthHeight, QRCODE_FOLDER, view->frameID);
            
            // DEBUG
            // add_qr_bgra(colorized_depth[i], view->depthWidth, view->depthHeight, QRCODE_FOLDER, view->frameID, true);
        }

        uint8_t *depth_data = reinterpret_cast<uint8_t *>(colorized_depth[i]);

        // //  Tile with 10*1 format (I think this is incorrect)
        // memcpy(shm_ptr + 10 * view->depthHeight * view->depthWidth * CHANNEL_DIM::BGRA * (curr_counter % SHM_BUFFER_SIZE) + view->viewID * view->depthHeight * view->depthWidth * CHANNEL_DIM::BGRA, color_buf[i], view->depthHeight * view->depthWidth * CHANNEL_DIM::BGRA);

        // Tile with 2*5 format
        auto h = view->depthHeight;
        auto w = view->depthWidth;
        auto id = view->viewID;

        // TODO: Rajrup: Replace 10 with view_count?
        /**
         * @brief The memory layout of shm_ptr is as follows:
         * 1st row of 5*2 layout is placed one after another in memory ||BRGA|BRGA| ---- w * 5 ----- |BRGA|BRGA||. Then comes the second row and so on, till 10 rows are filled.
         * This is done since the gstreamer memory layout expects the data to be in this format. 
         * Find BGRA layout here: https://gstreamer.freedesktop.org/documentation/additional/design/mediatype-video-raw.html?gi-language=c 
         */
        int x = id / 5 * h, y = id % 5 * (w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA);
        for (int j=0; j<h; j++)
            memcpy(
                    buf + ((x + j) * w * 5 * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA) + y, 
                    depth_data + (j * w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA), 
                    w * CHANNEL_DIM::BGRA * PIXEL_SIZE::BGRA
                );

        if (isSaveView && (frame_id % 10 == 0))
        {
            if(!fs::exists(save_dir))
                fs::create_directories(save_dir);
            save_color(depth_data, h, w, FORMAT(save_dir << frame_id << "_color_bgra_" << view->viewID << ".png"));
        }
    }
    // LOG(INFO) << "Sending tiled color: " << frame_number;
    if (isSaveView && (frame_id % 10 == 0))
    {
        auto h = views[0]->depthHeight;
        auto w = views[0]->depthWidth;
        if(!fs::exists(save_dir))
            fs::create_directories(save_dir);
        save_color(buf, h * 2, w * 5, FORMAT(save_dir << frame_id << "_color_bgra.png"));
    }
    return true;
}

/**
 * @brief Tiled depth assumes that format is in YUV16. Add QR code to view 0.
 *
 * @param buf
 * @param views
 * @param depth_yuv16_cv Used as the storage for storing YUV16 depth image temporarily. It's like scratch
 * @param depth_yuv16_buf Used as the storage for storing flattened YUV16 depth image temporarily. It's like scratch
 * @param isSaveView
 * @param save_dir
 * @return true
 * @return false
 */

bool SenderFrameBuffer::copy_in_gst_format_depth_yuv16(uint8_t *buf, const vector<View *> views, cv::Mat (&depth_yuv16_cv)[CHANNEL_DIM::DEPTH], uint8_t **depth_yuv16_buf, bool isSaveView, std::string save_dir)
{
    if(views.size() != 10)
    {
        LOG(ERROR) << "Type - " << type << " :View count " << views.size() << " is not 10";
        return false;
    }
    int frame_id = views[0]->frameID;

    /**
     * @brief TODO: Rajrup: Make this omp parallel.
     * For that, we need to change flatten_depth_yuv16 to take depth_yuv16_cv[i][3] where i is view index.
     */
    // #pragma omp parallel for
    for (int i=0; i < views.size(); i++) 
    {
        auto view = views[i];
        auto h = view->depthHeight;
        auto w = view->depthWidth;
        auto id = view->viewID;
        
        depth_to_yuv16(view->depth_image, depth_yuv16_cv, h, w); // Scaled depth image in YUV16
        if(i==0)
        {
            add_qr_yuv16(depth_yuv16_cv, QRCODE_FOLDER, view->frameID); // Add scaled QR code in UV channel

            // DEBUG
            // add_qr_yuv16(depth_yuv16_cv, QRCODE_FOLDER, view->frameID, true, true);
        }

        flatten_depth_yuv16(depth_yuv16_cv, depth_yuv16_buf[i], h, w); // Flatten YUV16 image to buffer

        // //  Tile with 10*1 format (I think this is incorrect)
        // memcpy(shm_ptr + 10 * view->depthHeight * view->depthWidth * 4 * (curr_counter % SHM_BUFFER_SIZE) + view->viewID * view->depthHeight * view->depthWidth * 4, depth_buf[i], view->depthHeight * view->depthWidth * 4);

        // Tile with 2*5 format
        // int x = id / 5 * h, y = id % 5 * (w * CHANNEL_DIM::BGRA);
        // for (int j=0; j<h; j++) 
        //     memcpy(shm_ptr + (10 * h * w * CHANNEL_DIM::BGRA * (curr_counter % SHM_BUFFER_SIZE)) + ((x + j) * w * 5 * CHANNEL_DIM::BGRA + y), depth_buf[i] + (j * w * CHANNEL_DIM::BGRA), w * CHANNEL_DIM::BGRA);

        int x = id / 5 * h, y = id % 5 * (w * PIXEL_SIZE::DEPTH);
        for(int k=0; k<CHANNEL_DIM::DEPTH; k++)
        {
            uint8_t *depth_channel_buf = depth_yuv16_cv[k].data;
            for(int j=0; j<h; j++)
            {
                memcpy(
                        buf + (k * 10 * h * w * PIXEL_SIZE::DEPTH) + ((x + j) * 5 * w * PIXEL_SIZE::DEPTH + y), 
                        depth_channel_buf + (j * w * PIXEL_SIZE::DEPTH), 
                        w * PIXEL_SIZE::DEPTH
                    );
            }
        }

        // DEBUG: store separate image to local
        // if (frame_number % 5 == 0)
        //     save_depth(depth_buf[i],  views[0]->depthHeight, views[0]->depthWidth, FORMAT("/home/lei/data/pipeline/server_standalone/" << frame_number << "_depth_" << view->viewID  << ".png"));

        // if(isSaveView && (frame_id % 10 == 0))
        // {
        //     if(!fs::exists(save_dir))
        //         fs::create_directories(save_dir);
            
        //     uint8_t *depth_channel_buf = depth_yuv16_cv[0].data;
        //     cv::Mat depth_image_cv2(h, w, CV_16UC1, depth_channel_buf);
        //     std::string path = FORMAT(save_dir << frame_id << "_depth_yuv16_" << view->viewID << ".png"); 
        //     if(!cv::imwrite(path, depth_image_cv2))
        //         LOG(ERROR) << "Failed to write depth image: " << path;
        //     else 
        //         LOG(INFO) << "Depth image saved to " << path;

        //     depth_channel_buf = depth_yuv16_cv[1].data;
        //     depth_image_cv2 = cv::Mat(h, w, CV_16UC1, depth_channel_buf);
        //     path = FORMAT(save_dir << frame_id << "_depth_yuv16_" << view->viewID << "_qr_u.png");
        //     if(!cv::imwrite(path, depth_image_cv2))
        //         LOG(ERROR) << "Failed to write depth image: " << path;

        // }
    }

    if(isSaveView && (frame_id % 10 == 0))
    {
        auto h = views[0]->depthHeight;
        auto w = views[0]->depthWidth;
        if(!fs::exists(save_dir))
            fs::create_directories(save_dir);
        // cv::Mat depth_image_cv2 = cv::Mat(h * 2, w * 5, CV_16UC1, buf);
        cv::Mat channel_y = cv::Mat(h * 2, w * 5, CV_16UC1, buf);
        cv::Mat channel_u = cv::Mat(h * 2, w * 5, CV_16UC1, buf + channel_y.total() * channel_y.elemSize());
        cv::Mat channel_v = cv::Mat(h * 2, w * 5, CV_16UC1, buf + channel_y.total() * channel_y.elemSize() + channel_u.total() * channel_u.elemSize());
        std::string path = FORMAT(save_dir << frame_id << "_depth_yuv16_y.png");
        if(!cv::imwrite(path, channel_y))
            LOG(ERROR) << "Type - " << type << " :Failed to write depth image: " << path;
        else
            LOG(INFO) << "Type - " << type << " :Depth image saved to " << path;

        path = FORMAT(save_dir << frame_id << "_depth_yuv16_u.png");
        if(!cv::imwrite(path, channel_u))
            LOG(ERROR) << "Type - " << type << " :Failed to write depth image: " << path;
        else
            LOG(INFO) << "Type - " << type << " :Depth image saved to " << path;

        path = FORMAT(save_dir << frame_id << "_depth_yuv16_v.png");
        if(!cv::imwrite(path, channel_v))
            LOG(ERROR) << "Type - " << type << " :Failed to write depth image: " << path;
        else
            LOG(INFO) << "Type - " << type << " :Depth image saved to " << path;
    }
    return true;
}