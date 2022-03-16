# -*- coding: utf-8 -*-
"""
Created on Sat Feb 19 14:00:24 2022

@author: WIN 10 OS
"""

import socket
import serial
import time

HEADER = 64
PORT = 5050 
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "D"
SERVER = "192.168.43.239"
ADDR = (SERVER, PORT)
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1, rtscts = True)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)
print("connecting")


def _write(x_arr):
    for x in x_arr:
        arduino.write(bytes(str(x) + ',' , 'utf-8'))
        print(x,'arduino:', arduino.readline().decode() )
        time.sleep(0.1)


while True:
    req = client.recv(2048).decode(FORMAT)[:-1]
    if req == DISCONNECT_MESSAGE:
        break
    else:
        data = list(map(int,req.split(',')))
        print('data',data)
        _write(data)


