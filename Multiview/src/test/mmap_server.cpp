#include <iostream>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <k4a/k4a.hpp>
#include <boost/asio.hpp>
#include "k4autils.h"
#include "utils.h"
#include "consts.h"
#include "k4aconsts.h"
#include "frameInfo_generated.h"

#define SHM_NAME "test_shm"

using namespace std;
using namespace boost::interprocess;
using boost::asio::ip::tcp;
using namespace Multiview::Frame;

extern string IP_ADDRESS;
extern int PORT;

static const int TCP_BUFFER_SIZE = 1024;

class TcpClient
{ 
public:

    TcpClient(boost::asio::io_service& svc, std::string const& host, int const& port) 
        : io_service(svc), socket(io_service) 
    {
        cout << "Trying to connect ..." << std::endl;
        socket.connect(boost::asio::ip::tcp::endpoint(
                        boost::asio::ip::address::from_string(host), port));
    }

    ~TcpClient() 
    {
        cout << "Closing connection ..." << endl;
        socket.close();
    }

	void send(const uint8_t *buf, const int buf_size) {
        send_to_socket(buf, buf_size);
        // socket.send(boost::asio::buffer(message));
    }

    void close() 
    {
        socket.close();
    }

private:

	void send_to_socket(const uint8_t *buf, const int buf_size)
    {
        uint32_t data_sent_in_bytes = 0;
        // std::chrono::steady_clock::time_point tx_begin = std::chrono::steady_clock::now();
        try
        {
            const uint32_t rc = boost::asio::write(socket, boost::asio::buffer(buf, buf_size));
            data_sent_in_bytes = rc;
        }
        catch (boost::system::system_error e)
        {
            std::cerr << boost::diagnostic_information(e);
        }
        // std::chrono::steady_clock::time_point tx_end = std::chrono::steady_clock::now();
        std::cout << " sent " << data_sent_in_bytes << " bytes" << std::endl;
        return;
    }

    boost::asio::io_service& io_service;
    boost::asio::ip::tcp::socket socket;
};

void destroy_shm(const char* name)
{
	shared_memory_object::remove(name);
	cout << "Destroyed shared memory " << name << endl;
}

void create_shm(const char* name, size_t size, mapped_region& region)
{
	//Create a shared memory object
	shared_memory_object shm_object(open_or_create, name, read_write);

	//Set size
	shm_object.truncate(size);

	//Map the whole shared memory in this process
	region = mapped_region(shm_object, read_write);

	cout << "Created shared memory " << name << " with size " << size << endl;
}

uint32_t get_image_size(const string &path, bool color)
{
    // Read a frame from the directory pointed to by path

    // Reade 1st frame and 1st view. All images have same diemensions.
    int frame_number = 1;
	int view_number = 0;

    string img_path = FORMAT(path << frame_number << "/color_" << view_number << ".png");
    cv::Mat img_mat;
    k4a::image img_kinect;
    if(color)
    {
        // color image
        img_mat = cv::imread(img_path, cv::IMREAD_UNCHANGED);
        img_kinect = opencv_to_color(img_mat);
    }
    else
    {   
        // depth image
        img_mat = cv::imread(img_path, cv::IMREAD_ANYDEPTH);
        img_kinect = opencv_to_depth(img_mat);
    }
	return img_kinect.get_size();
}

int main()
{
	cout << "Hello from CPP Server!" << endl;

    IP_ADDRESS = "127.0.0.1";
    PORT = 65432;

	//Remove shared memory on construction and destruction
	struct shm_remove
	{
		shm_remove() { destroy_shm(SHM_NAME); }
		~shm_remove() { destroy_shm(SHM_NAME); }
	} remover;

	// --------------------------------SETUP--------------------------------
    string date = "Feb_3_2022";
    string path = FORMAT(DATA_PATH << date << "/views/"); // RAJRUP: Save to SSD
	int frame_number = 1;
	int view_number = 0;

    //Create a shared memory object
	mapped_region region;
    uint32_t shm_size = get_image_size(path, true);
	create_shm(SHM_NAME, shm_size, region);
	uint8_t *shm_ptr = static_cast<uint8_t *>(region.get_address());

    // Create a socket and connect to the server.
	boost::asio::io_service svc;
	TcpClient client(svc, IP_ADDRESS, PORT);

    // flatbuffer for serialization
    flatbuffers::FlatBufferBuilder builder(TCP_BUFFER_SIZE);

    // --------------------------------LOAD DATA--------------------------------
    // Load color image from disk
    string img_path = FORMAT(path << frame_number << "/color_" << view_number << ".png");
	cv::Mat img_mat = cv::imread(img_path, cv::IMREAD_UNCHANGED); // cv::IMREAD_ANYDEPTH for depth image
	cout << "CV img_mat.rows: " << img_mat.rows << endl;
	cout << "CV img_mat.cols: " << img_mat.cols << endl;
	cout << "CV img_mat.channels: " << img_mat.channels() << endl;
	cout << "CV img_size: " << img_mat.rows * img_mat.cols * img_mat.channels() << endl;

	// Convert opencv image to Kinect image
	k4a::image img_kinect = opencv_to_color(img_mat);
	uint32_t img_size = img_kinect.get_size();
	cout << "K4A img_size: " << img_size << endl;

    // Add metadata to flatbuffer
	auto frame_info = CreateFrameInfo(builder, frame_number, view_number);
	builder.Finish(frame_info);
    // This must be called after `Finish()`.
	uint8_t *frame_info_buf = builder.GetBufferPointer();
    int frame_info_size = builder.GetSize(); // Returns the size of the buffer that `GetBufferPointer()` points to.

    // --------------------------------SHARE DATA--------------------------------
	// To Do: Write a loop here to continuously send data to the python client.

    //Copy data to shared memory
	memcpy(shm_ptr, img_kinect.get_buffer(), img_size);
	
	client.send(frame_info_buf, frame_info_size); // Send metadata to notify client to read frame.
    builder.Clear();    // Clear builder to reuse.

    // --------------------------------CLEANUP--------------------------------
    // Prevent client to exit, otherwise shared memory will be removed.
    cout << "Press any key to exit..." << endl;
	cin.get();

    client.close();
    builder.Release();
    sleep(1);
	return 0;
}