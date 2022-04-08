# -*- coding: utf-8 -*-
"""
Created on Sat Feb 19 17:45:48 2022

@author: WIN 10 OS
"""
import math
import numpy as np
import serial
import time
from gekko import GEKKO
import cv2 as cv
import Lane_detect_clean as LD

arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)
vfunc = np.vectorize(lambda t: t.value[0])
h = 1000
w = 1000
speed = 120
mtx, dist, newcameramtx = 0
init_state = [w//2,h//2,np.pi/2]

def send_msg(*msg_arr):
    for msg in msg_arr: 
        arduino.write(bytes(msg, 'utf-8'))
        time.sleep(0.05)
    
class Calculator():
    def __init__(self):
        self.coeff = [] 
        
    def cal_x_0(self,x_c, y_c):
        #newton-raphson method
        a,b,c = self.coeff
        k = c - y_c
        x = x_c
        for i in range (30):
            t = a * x
            x = (t * x * (4 * t + 3 * b) - b * k + x_c) / (6 * t * (t + b) + b**2 + 2 * a * k + 1)
        return x
    
    def lane_cal(self,x_c, y_c, psi):
        #error point in road
        a,b,c = self.coeff
        
        x_r = self.cal_x_0(x_c, y_c)
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
        
        return CTE, epsi


class PID():
    def __init__(self, Kp, Ki, Kd):
        self.integral_term = 0
        self.temp_error = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = 0.1
        self.angle_limit = np.pi/3
    def controller(self,CTE):
        
        self.integral_term += CTE
        
        #PID_cal
        P = CTE * self.Kp
        I = self.integral_term * self.Ki * self.dt
        D = (CTE - self.temp_error) * self.Kd / self.dt
        steel_angle = P + I + D
        
        #angle limit
        if steel_angle > self.angle_limit:
            steel_angle = self.angle_limit
        elif steel_angle < -self.angle_limit:
            steel_angle = -self.angle_limit
            
        self.temp_error = CTE
        
        return steel_angle

class Stanley():
    def __init__(self, K, speed):
        self.K = K
        self.speed = speed
        self.angle_limit = np.pi/3
    def controller(self,epsi,CTE):
        
        #Stanley_cal
        steel_angle = epsi + math.atan(self.K * CTE / self.speed)
        
        #angle limit
        if steel_angle > self.angle_limit:
            steel_angle = self.angle_limit
        elif steel_angle < -self.angle_limit:
            steel_angle = -self.angle_limit
        
        return steel_angle
    
class MPC():
    def __init__(self, N, speed):
        self.N = N
        self.cte_weight = 1
        self.epsi_weight = 10
        self.angle_limit = np.pi/3
        self.speed = speed
        self.lr_l = 0.5
        self.l = 2
        self.dt = 0.1
    def controller(self,a,b,c):
        
        #setup model
        m = GEKKO(remote=False)
        m.options.DIAGLEVEL = 0
        
        def m_lane_cal(x_c,psi,x_r):
            dy = 2 * a * x_r + b
            epsi = m.atan(dy) - psi
            cte_2 = (x_r - x_c)**2 * (1+1/dy**2)
            return cte_2, epsi
        
        
        # Variable
        # state array [x,y,psi]
        x_r = m.Array(m.Var,(self.N))
        x = m.Array(m.Var,(self.N+1, 3))
        # delta output state array [delta]
        u = m.Array(m.Var,(self.N))
        
        for r in range(self.N):
            x_r[r].value = 20
         
        #fix first stage
        [m.fix(x[0][idx], val = val)for idx, val in enumerate(init_state)]
        m.fix(u[0], val = 0)
        # MPC controller
        for t in range(self.N):
            m.Equations([x[t + 1][0] == x[t][0] + self.speed * m.cos(x[t][2] + m.atan(self.lr_l * m.tan(u[t]))) * self.dt, #x
                        x[t + 1][1] == x[t][1] + self.speed * m.sin(x[t][2] + m.atan(self.lr_l * m.tan(u[t]))) * self.dt, #y 
                        x[t + 1][2] == x[t][2] + self.speed * m.cos(m.atan(self.lr_l * m.tan(u[t]))) * m.tan(u[t]) / self.l * self.dt #psi
                        #u[t] delta
                        ])
            
            u[t].upper = self.angle_limit
            u[t].lower = -self.angle_limit
            
            m.Equation(2 * a**2 * x_r[t]**3 + 3 * a * b * x_r[t] ** 2 + (b**2 + 2 * a * (c - x[t][1]) + 1) * x_r[t] + b * c - b * x[t][1] - x[t][0] == 0 )
        
        #error J cost function
        for ii in range(self.N):
            cte_2, epsi = m_lane_cal(x[ii,0], x[ii,2], x_r[ii])
            m.Obj(self.cte_weight * cte_2 + self.epsi_weight * epsi**2)
        
        #solve
        m.options.IMODE = 3
        m.solve()
        
        #change to value 
        #stage = vfunc(x)
        #x_r = vfunc(x_r)
        u = vfunc(u)
        return u[1]
        

cal = Calculator()

#Gain
PID_K = [1, 0.2, 0.1]
Stanley_K = 1.1
MPC_N = 20
#controller setup
PID_control = PID(*PID_K)
Stanley_control = Stanley(Stanley_K,speed)
MPC_control = MPC(MPC_N,speed)


def main():
    SE = 0
    cap = cv.VideoCapture()
    cap.open(0, cv.CAP_DSHOW)
    n = 0
    while cap.isOpened():
        n+= 1
        ret, frame = cap.read()  
        dst = cv.undistort(frame, mtx, dist, None, newcameramtx)
        cal.coeff = LD.main(dst)
        CTE, epsi = cal.lane_cal(*init_state)
        steel = PID_control(CTE)
        init_state[-1] = steel
        SE += CTE**2
        send_msg(steel, SE/n)
        

