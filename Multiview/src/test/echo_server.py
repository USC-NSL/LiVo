# echo-server.py

import socket
from struct import *

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(4) # Size of integer = 4 bytes
            if not data:
                break
            data = unpack('!i', data)[0]
            print(f"Received {data}, type {type(data)}")