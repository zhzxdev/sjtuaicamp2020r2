import socket as S

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = S.socket(S.AF_INET, S.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1)
    print("received message: %s" % data)