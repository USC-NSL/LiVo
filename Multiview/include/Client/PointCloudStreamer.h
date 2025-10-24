#pragma once
#include <boost/asio.hpp>
#include <vector>
#include "utils.h"

using namespace std;

class PointCloudStreamer
{

public:
	PointCloudStreamer(int port);
	~PointCloudStreamer();
	void close();
	void start();
	bool isConnected();
	bool sendPointCloud(const vector<Point3f_S>& vertices, const vector<RGB_S>& colors);
	bool sendPointCloud(const uint8_t *buf, const int32_t size);

private:
	boost::asio::io_service m_io_service;
	boost::asio::ip::tcp::socket m_socket;
	boost::asio::ip::tcp::acceptor m_acceptor;

	bool m_isConnected;
};