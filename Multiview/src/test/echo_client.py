# echo-client.py

import socket
from struct import *

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65432  # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
	s.connect((HOST, PORT))

	for i in range(10):
		data = pack('!i', i)	# Send in network order
		s.send(data)
		print("Sent: ", i)