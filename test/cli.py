import socket
import os
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
MESSAGE = b"1"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    time.sleep(1)
