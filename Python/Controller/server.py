# -*- coding: utf-8 -*-
"""
Created on Mon Mar 14 16:30:37 2022

@author: WIN 10 OS
"""

import pygame
import math
import socket

HEADER = 64
PORT = 5050
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "D"
SERVER = socket.gethostbyname(socket.gethostname())
print(SERVER)
ADDR = (SERVER, PORT)
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(ADDR)
print('server starting')
server.listen()
conn, addr = server.accept()
print(f'server accept {addr}')
Pi_2 = math.pi/2
pygame.init()

h = 400
# Set up the drawing window
screen = pygame.display.set_mode([h, h])
max_theta = math.pi/3
max_speed = 200
min_speed = 80
max_scale = 130
min_scale = 90
# Run until the user asks to quit
running = True
control = 1
Shift_mode = 0
Con_speed = min_speed
theta_temp = 0
speed_temp = 0
min_ = 3
speed_scale = lambda speed : int( (max_scale - min_scale) / (max_speed - min_speed) * (speed - min_speed) + min_scale)

def _send(speed, theta):
    global theta_temp, speed_temp
    if abs(theta - theta_temp) >= min_ or abs(theta_temp - speed_temp)>= min_:
        theta_temp = theta
        speed_temp = speed 
        msg = str(speed_scale(speed)).zfill(3) + str(theta).zfill(3) + ','
        #msg = '!' + str(speed).zfill(3) + str(theta).zfill(3)
        print(f'msg: {msg}')
        message = msg.encode(FORMAT)
        conn.send(message)
    
    
while running:
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE: # min speed and servo
                control = 1 - control
            if event.key == pygame.K_q: # constant speed free servo
                Shift_mode = 1 - Shift_mode
            if event.key == pygame.K_ESCAPE: 
                running = False
            # Did the user click the window close button? If so, stop the loop.
            if event.type == pygame.QUIT:
                running = False

    # Fill the background with white
    
    x, y = pygame.mouse.get_pos()
    temp_speed = math.dist((x, y),(h//2, h//2))
    theta = math.atan2(y-h//2, x-h//2) + Pi_2
    
    sc_color = [255, 255, 255]
    
    
    
    if theta >= max_theta:
        theta = max_theta
        sc_color[1] = 0
    elif theta <= -max_theta:
        theta = -max_theta
        sc_color[1] = 0
    
    if temp_speed >= max_speed:
        temp_speed = max_speed
        sc_color[2] = 0
    elif temp_speed <= min_speed:
        temp_speed = min_speed
        sc_color[2] = 0
    if Shift_mode:
        temp_speed = Con_speed
        sc_color = [100,100,100]
    else:
        Con_speed = temp_speed
        
    if control:
        Shift_mode = 0
        temp_speed = min_speed
        theta = 0
        sc_color = [0,0,0]
    
    screen.fill(sc_color)
    
    max_h = h/2 *(1 + math.tan(Pi_2 - max_theta))
    pygame.draw.line(screen, (255, 0, 100), (0, h - max_h), (h//2, h//2))
    pygame.draw.line(screen, (255, 0, 100), (h, h - max_h), (h//2, h//2))
    pygame.draw.line(screen, (255, 0, 100), (h//2, 0), (h//2, h//2))
    pygame.draw.circle(screen, (255, 0, 0), (x, y), 6)
    pygame.draw.line(screen, (255, 0, 0), (x, y), (h//2, h//2))
    # Draw a solid blue circle in the center
    pygame.draw.circle(screen, (0, 0, 255), (h//2, h//2), 15)

    pygame.draw.circle(screen, (0, 0, 255), (h//2, h//2), max_speed, 3)
    pygame.draw.circle(screen, (0, 0, 255), (h//2, h//2), min_speed, 3)
    
    
    #print(temp_speed, math.degrees(Pi_2 + theta))
    _send(temp_speed, int(math.degrees(Pi_2 + theta)))
    # Flip the display
    
    pygame.time.delay(100)
    pygame.display.flip()

# Done! Time to quit.
pygame.quit()
message = DISCONNECT_MESSAGE.encode(FORMAT)
conn.send(message)
conn.close()