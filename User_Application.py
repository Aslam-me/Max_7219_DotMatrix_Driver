import time
import socket

UDP_IP = '192.168.0.44'
UDP_PORT = 16350

def print_in_Dot_Matrix_Display(string):
    
    with open("/dev/DotMatrixDisplay", "w") as Dot_Matrix_Display:
        Dot_Matrix_Display.write(string)


if __name__ == "__main__":
    sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    print("UDP Server Started")
    print_in_Dot_Matrix_Display("UDP Server Started    ")
    

    while True:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        print("received message: %s" % data)
        print_in_Dot_Matrix_Display(data)