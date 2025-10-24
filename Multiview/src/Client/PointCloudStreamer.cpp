#include "Client/PointCloudStreamer.h"
#include <iostream>

PointCloudStreamer::PointCloudStreamer(int port = 9000) : 
									m_io_service(), m_socket(m_io_service),
									m_acceptor(m_io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
{
	m_isConnected = false;
	m_acceptor.async_accept(m_socket,
							[this](boost::system::error_code ec)
							{
								m_isConnected = true;
								cout << "Unity client connected" << endl;
							});

    cout << "Waiting for client to connect..." << endl;
    m_io_service.run();
}

PointCloudStreamer::~PointCloudStreamer()
{
	if(m_socket.is_open())
		m_socket.close();
}

bool PointCloudStreamer::isConnected()
{
	return m_isConnected;
}

void PointCloudStreamer::close()
{
	m_isConnected = false; 
	if(m_socket.is_open())
		m_socket.close(); 
}

void PointCloudStreamer::start()
{
    if(m_isConnected)
        m_io_service.run();
    
    cout << "No client's have connected..." << endl;
}

bool PointCloudStreamer::sendPointCloud(const uint8_t *buf, const int32_t size)
{
	if(!m_isConnected)
    {
        cout << "Not connected" << endl;
        return false;
    }
	
	static int ptclID = 1;
	int numPoints = size / (sizeof(Point3f_S) + sizeof(RGB_S));
	assert(size % (sizeof(Point3f_S) + sizeof(RGB_S)) == 0);

	int32_t v_size = numPoints * sizeof(Point3f_S);
    int32_t c_size = numPoints * sizeof(RGB_S);

	// Send the size of the point cloud
	boost::asio::write(m_socket, boost::asio::buffer(&size, sizeof(int32_t)));

    // cout << "-----------------------------------------PtCL ID: "<< ptclID << "-----------------------------------------" << endl;
    // cout << "Sending Ptcl of size " << size << " bytes" << endl;
    // cout << "#Points: " << numPoints << endl;

	// Send Ptcl
	int written = 0;
    try
    {
        while(written < size)
            written += boost::asio::write(m_socket, boost::asio::buffer(buf + written, size - written));
        assert(written == size);
    }
    catch (exception& e)
    {
        cout << "Error: Client might have closed" << endl;
        cerr << "Exception: " << e.what() << "\n";
        m_socket.close();
    }

	Point3f_S first_vertex = *(Point3f_S *)buf;
	Point3f_S last_vertex = *(Point3f_S *)(buf + v_size - sizeof(Point3f_S));
	RGB_S first_color = *(RGB_S *)(buf + v_size);
	RGB_S last_color = *(RGB_S *)(buf + size - sizeof(RGB_S));

	// cout << "Sent Ptcl of size " << size << " bytes" << endl;
    // cout << "First and Last point: " << endl;
    // cout << 0 << ": (" << first_vertex.x << ", " << first_vertex.y << ", " << first_vertex.z << ") RGBA (" << (int)first_color.r << ", " << (int)first_color.g << ", " << (int)first_color.b << ", 255)" << endl;
    // cout << numPoints - 1 << ": (" << last_vertex.x << ", " << last_vertex.y << ", " << last_vertex.z << ") RGBA (" << (int)last_color.r << ", " << (int)last_color.g << ", " << (int)last_color.b << ", 255)" << endl;

    ptclID++;

	return true;
}