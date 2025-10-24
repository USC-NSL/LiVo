#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include "test/ffmpeg.h"
// #include "utils.h"

using namespace std;

#define MAX_DEVICES 4
#define MAX_FRAMES 10

// av_err2str returns a temporary array. This doesn't work in gcc.
// This function can be used as a replacement for av_err2str.
static const char* av_make_error(int errnum) {
    static char str[AV_ERROR_MAX_STRING_SIZE];
    memset(str, 0, sizeof(str));
    return av_make_error_string(str, AV_ERROR_MAX_STRING_SIZE, errnum);
}


// // Load images from m_work_dir
// bool load_view(string &m_work_dir, int n_frames, vector<vector<cv::Mat>> &cframes, vector<vector<cv::Mat>> &dframes)
// {
// 	time_point start, end;
// 	uint64_t elapsed_ms;

// 	start = time_now();

// 	vector<cv::Mat> color_images_cv2;
// 	vector<cv::Mat> depth_images_cv2;
	
// 	for(uint32_t i = 0; i < MAX_DEVICES; i++)
// 	{
// 		string color_image_path = FORMAT(m_work_dir << "/views/" << n_frames + 1 << "/color_" << i << ".png");
// 		string depth_image_path = FORMAT(m_work_dir << "/views/" << n_frames + 1 << "/depth_" << i << ".png");

// 		cv::Mat color_image_cv2 = cv::imread(color_image_path, cv::IMREAD_UNCHANGED); // IMREAD_UNCHANGED reads with Alpha channel
// 		cv::Mat depth_image_cv2 = cv::imread(depth_image_path, cv::IMREAD_ANYDEPTH);

// 		if(color_image_cv2.empty() || depth_image_cv2.empty())
// 		{
// 			cout << "Failed to read images from " << color_image_path << " and/or " << depth_image_path << endl;
// 			return false;
// 		}
// 		color_images_cv2.push_back(color_image_cv2);
// 		depth_images_cv2.push_back(depth_image_cv2);

// 		// show_color_image(color_image_cv2);
// 		// show_depth_image(depth_image_cv2);
// 	}

// 	cframes.push_back(color_images_cv2);
// 	dframes.push_back(depth_images_cv2);

// 	end = time_now();
// 	elapsed_ms = duration_ms(start, end);
// 	cout << "Loaded frame in " << elapsed_ms << " ms" << endl;
// 	return true;
// }

static void encode(AVCodecContext *enc_ctx, AVFrame *frame, AVPacket *pkt,
                   FILE *outfile)
{
    int ret;

    /* send the frame to the encoder */
    if (frame)
        printf("Send frame %3" PRId64 "\n", frame->pts);

    ret = avcodec_send_frame(enc_ctx, frame);
    if (ret < 0) {
        fprintf(stderr, "Error sending a frame for encoding\n");
        exit(1);
    }

    while (ret >= 0) {
        ret = avcodec_receive_packet(enc_ctx, pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            return;
        else if (ret < 0) {
            fprintf(stderr, "Error during encoding\n");
            exit(1);
        }

        printf("Write packet %3" PRId64 " (size=%5d)\n", pkt->pts, pkt->size);
        fwrite(pkt->data, 1, pkt->size, outfile);
        av_packet_unref(pkt);
    }
}

int main(int argc, char **argv)
{
    const char *filename, *codec_name;
    const AVCodec *codec;
    AVCodecContext *c= NULL;
    int i, ret, x, y;
    FILE *f;
    AVFrame *frame;
    AVPacket *pkt;
    uint8_t endcode[] = { 0, 0, 1, 0xb7 };

    // if (argc <= 2) {
    //     fprintf(stderr, "Usage: %s <output file> <codec name>\n", argv[0]);
    //     exit(0);
    // }
    filename = "/data/Dropbox/Project/VolumetricVideo/KinectStream/out/test.264";
    // codec_name = argv[2];
    
    avcodec_register_all();
    av_register_all();

    /* find the mpeg1video encoder */
    // codec = avcodec_find_encoder_by_name(codec_name);
	// codec = avcodec_find_encoder(AV_CODEC_ID_MPEG2VIDEO);
	codec = avcodec_find_encoder(AV_CODEC_ID_H264);
	
    if (!codec) {
        fprintf(stderr, "Codec '%s' not found\n", codec_name);
        exit(1);
    }

	cout << "Codec name: " << codec->name << endl;

    c = avcodec_alloc_context3(codec);
    if (!c) {
        fprintf(stderr, "Could not allocate video codec context\n");
        exit(1);
    }

    pkt = av_packet_alloc();
    if (!pkt)
        exit(1);

    /* put sample parameters */
    c->bit_rate = 400000;

    /* resolution must be a multiple of two */
    c->width = 352;
    c->height = 288;
    /* frames per second */
    c->time_base = (AVRational){1, 25};
    c->framerate = (AVRational){25, 1};

    /* emit one intra frame every ten frames
     * check frame pict_type before passing frame
     * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
     * then gop_size is ignored and the output of encoder
     * will always be I frame irrespective to gop_size
     */
    c->gop_size = 10;
    c->max_b_frames = 0;
    c->pix_fmt = AV_PIX_FMT_YUV420P;
	c->qmin = 12;
	c->qmax = 42;

    if (codec->id == AV_CODEC_ID_H264)
        av_opt_set(c->priv_data, "preset", "slow", 0);

    /* open it */
    ret = avcodec_open2(c, codec, NULL);
    if (ret < 0) {
        fprintf(stderr, "Could not open codec: %s\n", av_make_error(ret));
        exit(1);
    }

    f = fopen(filename, "wb");
    if (!f) {
        fprintf(stderr, "Could not open %s\n", filename);
        exit(1);
    }

    frame = av_frame_alloc();
    if (!frame) {
        fprintf(stderr, "Could not allocate video frame\n");
        exit(1);
    }
    frame->format = c->pix_fmt;
    frame->width  = c->width;
    frame->height = c->height;

    ret = av_frame_get_buffer(frame, 0);
    if (ret < 0) {
        fprintf(stderr, "Could not allocate the video frame data\n");
        exit(1);
    }

    /* encode 1 second of video */
    for (i = 0; i < 25; i++) {
        fflush(stdout);

        /* Make sure the frame data is writable.
           On the first round, the frame is fresh from av_frame_get_buffer()
           and therefore we know it is writable.
           But on the next rounds, encode() will have called
           avcodec_send_frame(), and the codec may have kept a reference to
           the frame in its internal structures, that makes the frame
           unwritable.
           av_frame_make_writable() checks that and allocates a new buffer
           for the frame only if necessary.
         */
        ret = av_frame_make_writable(frame);
        if (ret < 0)
            exit(1);

        /* Prepare a dummy image.
           In real code, this is where you would have your own logic for
           filling the frame. FFmpeg does not care what you put in the
           frame.
         */
        /* Y */
        for (y = 0; y < c->height; y++) {
            for (x = 0; x < c->width; x++) {
                frame->data[0][y * frame->linesize[0] + x] = x + y + i * 3;
            }
        }

        /* Cb and Cr */
        for (y = 0; y < c->height/2; y++) {
            for (x = 0; x < c->width/2; x++) {
                frame->data[1][y * frame->linesize[1] + x] = 128 + y + i * 2;
                frame->data[2][y * frame->linesize[2] + x] = 64 + x + i * 5;

				// frame->data[1][y * frame->linesize[1] + x] = 0;
                // frame->data[2][y * frame->linesize[2] + x] = 0;
            }
        }

        frame->pts = i;

        /* encode the image */
        encode(c, frame, pkt, f);
    }

    /* flush the encoder */
    encode(c, NULL, pkt, f);

    /* Add sequence end code to have a real MPEG file.
       It makes only sense because this tiny examples writes packets
       directly. This is called "elementary stream" and only works for some
       codecs. To create a valid file, you usually need to write packets
       into a proper file format or protocol; see muxing.c.
     */
    if (codec->id == AV_CODEC_ID_MPEG1VIDEO || codec->id == AV_CODEC_ID_MPEG2VIDEO || codec->id == AV_CODEC_ID_MPEG4) 
        fwrite(endcode, 1, sizeof(endcode), f);
    fclose(f);

    avcodec_free_context(&c);
    av_frame_free(&frame);
    av_packet_free(&pkt);

    return 0;
}

// int main_()
// {
// 	cout << "Starting with FFmpeg" << endl;

// 	string m_work_dir = "/home/rajrup/kinect/data/VolumetricVideo/KinectStream/kinect_captures/Feb_3_2022";

// 	vector<vector<cv::Mat>> cframes;
// 	vector<vector<cv::Mat>> dframes;

// 	for(uint32_t n_frame = 0; n_frame < MAX_FRAMES; n_frame++)
// 	{
// 		if(!load_view(m_work_dir, n_frame, cframes, dframes))
// 		{
// 			cout << "Failed to load view " << n_frame << endl;
// 			return -1;
// 		}
// 		assert(cframes[n_frame].size() == dframes[n_frame].size());
// 	}

// 	cout << "Number of color frames: " << cframes.size() << endl;
// 	cout << "Number of depth frames: " << dframes.size() << endl;
	
// 	return 0;
// }