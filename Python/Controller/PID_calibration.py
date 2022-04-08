# -*- coding: utf-8 -*-
"""
Created on Mon Mar 28 15:49:07 2022

@author: WIN 10 OS
"""

import PID as pid
speed = 3
# Choose an inializaon for the parameter vector K = [KP, KI, KD]
K = [0.386,	0.003, 0.140]
# Define potenal changes
P = [5, 5, 5]
# Define pid.controller(K) as the funcon that returns the ∑ for driving for the track with parameter vector K 
# Calculate the error
best_err = pid.main(K, speed)
best_K = K
threshold = 0.00001
while sum(P) > threshold:
    for i in range(3):
        K[i] = K[i] + P[i]
        err = pid.main(K, speed)
        if err < best_err: # There was some improvement
            best_err = err
            best_K = K
            P[i] = P[i] * 1.1
        else: # There was no improvement
            K[i] = K[i] - 2*P[i] # Go into the other direcon
            err = pid.main(K, speed)
            if err < best_err: # There was an improvement
                best_err = err
                best_K = K
                P[i] = P[i] * 1.05
            else: # There was no improvement
                K[i] = K[i] + P[i]
            # As there was no improvement, the step size in either
            # direcon, the step size might simply be too big.
                P[i] = P[i] * 0.95
print(best_K, best_err)
