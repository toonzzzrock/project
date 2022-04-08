# -*- coding: utf-8 -*-
"""
Created on Fri Apr  1 19:11:04 2022

@author: WIN 10 OS
"""

import numpy as np
import math
import random
import time
import matplotlib.pyplot as plt
#lane path y = ax^2+bx+c
y_eqa = [0.06,1,-1]
a,b,c = y_eqa

#initial state of vehical [x,y.psi]
init_state_val = [20,25,0]

#const
dt = 0.1
time_step = 75
l = 2
lr = 1
angle_limit = np.pi/3
epsi_weight = 3
def cal_x_0(x_c, y_c):
    #newton-raphson method
    k = c - y_c
    x = x_c
    for i in range (3):
        t = a * x
        x = (t * x * (4 * t + 3 * b) - b * k + x_c) / (6 * t * (t + b) + b**2 + 2 * a * k + 1)
    return x

def lane_cal(x_c, y_c, psi):
    #error point in road
    x_r = cal_x_0(x_c, y_c)
    y_r = a * x_r**2 + b * x_r + c
    
    dy_dx = 2 * a * x_r + b
    
    # heading error
    epsi = math.atan(dy_dx) - psi
    
    #cross-track error vector size
    CTE = abs(x_r-x_c)*math.sqrt(1+1/dy_dx**2)
    
    #left-right
    car_to_point = math.cos(psi) * (y_r - y_c) - math.sin(psi) * (x_r - x_c)
        #car_to_point = math.asin((math.cos(psi)*(y_r-y_c)-math.sin(psi)*(x_r-x_c))/1000)
    CTE *= car_to_point / abs(car_to_point)
    
    return CTE, epsi, x_r, y_r

def moving(steel, speed, init, dt_real, noise = False, bias = True):
    if bias:
        dt_real+= 0.2
    if noise:
        steel += random.choice([1,-1]) * random.uniform(steel/8, steel/3)
    beta = math.atan(lr * math.tan(steel)/ l)
    x = init[0] + math.cos(init[2] + beta) * speed * dt_real
    y = init[1] + math.sin(init[2] + beta) * speed * dt_real
    psi = (init[2] + speed / l * math.cos(beta) * math.tan(steel) * dt_real) % (2*math.pi)
    
    return [x, y, psi]
  
class Stanley():
    def __init__(self, K):
        self.error_state_Stanley = np.zeros(time_step)
        self.init_state_Stanley = np.array([init_state_val for i in range(time_step)], float)
        self.K = K
        
    def controller(self):
        CTE,epsi, x_r, y_r = lane_cal(*self.init_state_Stanley[i, :3])
        self.error_state_Stanley[i] = CTE**2 + epsi_weight * epsi**2
        
        #Stanley_cal
        steel_angle = epsi + math.atan(self.K * CTE / speed)
        
        #angle_limit
        if steel_angle > angle_limit:
            steel_angle = angle_limit
        elif steel_angle < -angle_limit:
            steel_angle = -angle_limit
        
        return steel_angle, x_r, y_r
def controller_All():

    start = time.time()
    #Stanley
    Stanley_steel_angle, x_r, y_r = Stanley_control.controller()
    end = time.time()
    Stanley_dt_real = end - start
    
    Stanley_control.init_state_Stanley[i+1] = moving(Stanley_steel_angle, speed, Stanley_control.init_state_Stanley[i], Stanley_dt_real)

def main(Stanley_K, inp_speed, plot = False):
    #controller setup
    global Stanley_control, i, speed
    speed = inp_speed
    Stanley_control = Stanley(Stanley_K)
    
    #run
    for i in range(time_step-1):
        controller_All() 
    if plot:
        plt.plot(Stanley_control.error_state_Stanley)
        plt.show()
    
    return sum(Stanley_control.error_state_Stanley)/time_step


'''
# Stanley controller

list_K = np.linspace(1, 10, 300)
for K in list_K:
    try:
        init_state = np.array([[0,0,0,0] for i in range(time_step)], float)    
        error_state = 0
        controller(K)
        error_state = error_state/time_step
        
        x = np.linspace(min(init_state[:,0]), max(init_state[:,0]), 100)
        y = a*x**2 + b*x + c  
        plt.title(f'min {K} with:{error_state}')
        plt.plot(init_state[:,0],init_state[:,1])
        plt.plot(x, y)
        plt.legend(['state','lane'])
        plt.axis('scaled')
        plt.show()
        
    except ValueError:
        print('error')
        def f(x_list):
            y_list = []
            for x in x_list:
                y_list.append(2*a**2*x**3+3*a*b*x**2+(b**2+2*a*(c-x_c)+1)*x+b*c-b*y_c-x_c)
            return y_list
        x_c,y_c,_,_ = init_state[i]
        k = c-y_c
        x = x_c
        list_x = []
        for ii in range (100):
            t = a*x
            x = (t*x*(4*t + 3*b) - b*k + x_c)/(6*t*(t + b) + b**2 + 2*a*k + 1)
            
            if ii%5:
                list_x.append(x)
                plt.plot(list_x, f(list_x), '.r')
                plt.text(x, f([x]), str(ii))
                x_plot = np.linspace(min(list_x), max(list_x), 200)
                plt.plot(x_plot,f(x_plot))
                plt.show()
                plt.pause(0.2)'''
        