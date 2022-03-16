
import socket
import serial
import time

HEADER = 64
PORT = 5050 
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "D"
SERVER = "192.168.43.239"
ADDR = (SERVER, PORT)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)
print("connecting")

arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)
def _write(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.1)
while True:
    #num = input("Enter a number: ") # Taking input from user
    req = client.recv(2048).decode(FORMAT)[1:6]
    _write(num)
    print('arduino: ',arduino.readline().decode())