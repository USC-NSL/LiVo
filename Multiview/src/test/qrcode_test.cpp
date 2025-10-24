#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "test/qrcode.h"
#include "pconsts.h"
#include "consts.h"
#include "utils.h"
#include "timer.h"

using namespace std;

DEFINE_int32(log_level, 0, "0 - INFO, 1 - WARNING, 2 - ERROR, 3 - FATAL.");
DEFINE_string(config_file, "/home/lei/rajrup/KinectStream/Multiview/config/panoptic.json", "Path to config file.");

void display(cv::Mat &im, cv::Mat &bbox)
{
	int n = bbox.rows;
	for(int i = 0 ; i < n ; i++)
	{
		line(im, 
			cv::Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), 
			cv::Point2i(bbox.at<float>((i+1) % n,0), 
			bbox.at<float>((i+1) % n,1)), 
			cv::Scalar(255,0,0), 3
			);
	}
	imshow("Result", im);
}

void detect(cv::Mat &test, int border=0)
{
	cv::QRCodeDetector qrDecoder = cv::QRCodeDetector();
	cv::Mat bbox, rectifiedImage;
	
	vector<cv::Point> corners;
	cv::Mat cropped_image = test(cv::Range(border, test.rows - border), cv::Range(border, test.cols - border));

	corners.push_back(cv::Point(0, 0));
	corners.push_back(cv::Point(cropped_image.cols - 0, 0));
	corners.push_back(cv::Point(cropped_image.cols - 0, cropped_image.rows - 0));
	corners.push_back(cv::Point(0, cropped_image.rows - 0));

	LOG(INFO) << "Decoding QR Code";
	StopWatch timer;
	timer.Restart();
	
	// string decode_info = qrDecoder.detectAndDecode(test, corners);
	string decode_info = qrDecoder.decode(cropped_image, corners);
	LOG(INFO) << "Decoding QR Code Done in " << timer.ElapsedMs() << " ms";
	if(decode_info.length() > 0)
	{
		cout << "QR Code Detected: " << decode_info << endl;
		cout << "Corners: " << corners << endl;
	}
	else
	{
		cout << "QR Code NOT Detected" << endl;
	}
}

int main(int argc, char* argv[])
{
	gflags::SetUsageMessage("Usage: qrcode_test [OPTION] ...");

    // Parse command line options
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    absl::SetMinLogLevel((absl::LogSeverityAtLeast)FLAGS_log_level);

    string config_file = FLAGS_config_file;
    parse_config(config_file);

	// string path = FORMAT(PANOPTIC_DATA_PATH << SEQ_NAME << "/"); // RAJRUP: Save to SSD
	// string imgfile = FORMAT(path << START_FRAME << "_color_" << 0 << ".png");
	// string outbinfile = FORMAT("/data/Dropbox/Project/VolumetricVideo/KinectStream/out/" << START_FRAME << "_bin_" << 0 << "_qrcode.png");
	// string outcolorfile = FORMAT("/data/Dropbox/Project/VolumetricVideo/KinectStream/out" << START_FRAME << "_color_" << 0 << "_qrcode.png");

	// // Read image
	// cv::Mat inputImage = cv::imread(imgfile);

	// cv::Mat test;
	// cv::QRCodeEncoder::Params params;
	// params.mode = cv::QRCodeEncoder::MODE_BYTE;
	// // params.correction_level = cv::QRCodeEncoder::CORRECT_LEVEL_L;
	// // params.version = 10;
	// LOG(INFO) << "Creating QR Code Encoder";
	// cv::Ptr<cv::QRCodeEncoder> qrEncoder = cv::QRCodeEncoder::create(params);

	// LOG(INFO) << "Encoding QR Code";
	// StopWatch timer;
	// timer.Restart();
	// qrEncoder->encode("H", test);
	// LOG(INFO) << "Encoding QR Code Done in " << timer.ElapsedMs() << " ms";

	// cout << "Bin QR Code size: " << test.size() << ", dims: " << test.dims << endl;
	// cv::imwrite(outbinfile, test);

	// cv::Mat color_qrcode;
	// cv::cvtColor(test, color_qrcode, cv::COLOR_GRAY2RGB);
	// cout << "Color QR Code size: " << color_qrcode.size() << ", dims: " << color_qrcode.dims << endl;
	// cv::imwrite(outcolorfile, color_qrcode);

	// detect(test);
	string inputdir = "/data/Dropbox/Project/VolumetricVideo/KinectStream/out/";
	LOG(INFO) << "Detecting QR Code without border";

	int border = 0;
	for(int frameId = START_FRAME; frameId < START_FRAME+10; frameId++)
	{
		string imgfile = FORMAT(inputdir << frameId << "_qrcode_cropped.png");
		cv::Mat inputImage = cv::imread(imgfile, cv::IMREAD_GRAYSCALE);
		detect(inputImage, border);

		imgfile = FORMAT(inputdir << frameId << "_qrcode_cropped.jpg");
		inputImage = cv::imread(imgfile, cv::IMREAD_GRAYSCALE);
		detect(inputImage, border);
	}

	border = 4 * 2; // 2 is scale factor in segno
	LOG(INFO) << "Detecting QR Code with border";

	for(int frameId = START_FRAME; frameId < START_FRAME+10; frameId++)
	{
		string imgfile = FORMAT(inputdir << frameId << "_qrcode.png");
		cv::Mat inputImage = cv::imread(imgfile, cv::IMREAD_GRAYSCALE);
		detect(inputImage, border);

		imgfile = FORMAT(inputdir << frameId << "_qrcode.jpg");
		inputImage = cv::imread(imgfile, cv::IMREAD_GRAYSCALE);
		detect(inputImage, border);
	}

	// cv::Mat test2 = cv::imread("/data/Dropbox/Project/VolumetricVideo/KinectStream/out/sample_qrcode_segno.jpg", cv::IMREAD_GRAYSCALE);

	// detect(test2);
}
