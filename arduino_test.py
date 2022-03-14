# -*- coding: utf-8 -*-
"""
Created on Wed Mar  9 13:23:14 2022

@author: WIN 10 OS
"""

import serial
import numpy as np
N = 4
M = 2
Vx = 0.0045
Vy = 0.0051
Va = 0.0068
X = np.zeros(N)
U = np.zeros(M)
H = np.zeros((M,N))
R = np.array([[Vx,0], [0,Vy]])
Z = np.zeros(M)
F = np.identity(N)
I = np.identity(N)
G = np.zeros((N,M))
P = np.zeros((N,N))
Q = np.zeros((N,N))
H[0,0] = H[1,2] = 1

def kalman_assign(dt):
    F[0,1] = F[2,3] = dt
    G[1,0] = G[3,1] = dt
    G[0,0] = G[2,1] = 0.5 * dt ** 2
    dt_2 = pow(dt,2) * Va
    dt_3 = 0.5 * pow(dt,3) * Va
    dt_4 = 0.25 * pow(dt,4) * Va
    for i in [0,2]:
        Q[i][i] = dt_4;
        Q[i+1][i] = Q[i][i+1] = dt_3;
        Q[i+1][i+1] = dt_2;
        
def Predict():
    global X, P
    X = np.matmul(F,X) + np.matmul(G,U)
    P = np.matmul(np.matmul(F,P),np.transpose(F)) + Q

def Update():
    global K, X, Z, P
    temp = np.linalg.inv( np.matmul(np.matmul(H ,P), np.transpose(H)) + R)
    K = np.matmul(np.matmul(P, np.transpose(H)), temp)
    
    X +=  np.matmul(K, Z - np.matmul(H, X))
    
    Z = np.matmul(H, X)
    
    temp = I - np.matmul(K,H)
    P = np.matmul(np.matmul(temp, P), np.transpose(temp)) + np.matmul(np.matmul(K, R), np.transpose(K))
    

ser = serial.Serial("COM7", 9600)
while True:
     cc=ser.readline().decode("utf-8")
     data = list(map(float,cc.split(',')))
     print('data',data)
     
     kalman_assign(0.1)
     # MEASURE
     U[0] = data[0]
     U[1] = data[1]
     #UPDATE
     Update()
     Predict()
     print('state', Z)
     print('\n')
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     