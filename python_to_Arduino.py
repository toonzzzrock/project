# -*- coding: utf-8 -*-
"""
Created on Sat Feb 19 15:28:43 2022

@author: WIN 10 OS
"""

# Importing Libraries
import serial
import time
arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)

def write_read(x,y):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    arduino.write(bytes(y, 'utf-8'))
    time.sleep(0.05)


# steel, MSE
value = write_read(steel,MSE)
print(value) # printing the value