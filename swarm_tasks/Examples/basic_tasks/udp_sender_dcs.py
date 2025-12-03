'''
import socket
import sys
#from dronekit import connect, VehicleMode


import socket, struct, time
from math import radians,cos,sin,asin,sqrt,pi

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#server_address = ('192.168.29.42', 12008)
server_address = ('192.168.29.69', 12009)

msg = 'ok'
while True:
	
	data = input("Enter: ")
	sent = sock.sendto(str(data).encode(), server_address)
	print ("msg111", msg)
	time.sleep(0.01)
'''	
	
import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('192.168.6.203', 12008)
while True:
    # Prompt the user to enter data
    data = input("Enter data to send: ")

    # Send data to the server
    sock.sendto(str(data).encode(), server_address)
    print("Data sent:", data)

    time.sleep(0.01)

