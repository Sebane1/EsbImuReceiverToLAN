import socket
import traceback

import sys

UDP_IP = "0.0.0.0"
UDP_PORT = 6970

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))
    
    print("Listening for ESP32 logs on UDP port " + str(UDP_PORT) + "...")
    
    while True:
        data, addr = sock.recvfrom(1024)
        sys.stdout.write(data.decode('utf-8', 'replace'))
        sys.stdout.flush()

except Exception as e:
    print("An error occurred: " + str(e))
    traceback.print_exc()
finally:
    try:
        input("Press Enter to exit...")
    except SyntaxError:
        pass
