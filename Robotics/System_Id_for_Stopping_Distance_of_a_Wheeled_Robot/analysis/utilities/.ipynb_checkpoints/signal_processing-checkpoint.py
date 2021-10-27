#!/bin/bash

import math
import numpy as np
import pandas as pd

# ----------------------------------
def euclidiean_dist(x,y):
    return math.sqrt(math.pow(x,2) + math.pow(y,2))


# ----------------------------------
def dist_calc(target_x, target_y):
    dist = []
    
    for (x,y) in zip(target_x, target_y):
        dist.append(euclidiean_dist(x,y))
    
    return min(dist)


# ----------------------------------
def calc_speed(distance, timestamp):
    dt = np.diff(timestamp)
    
    dd = np.diff(distance)
    
    speed = dd/dt
    
    return speed


# ----------------------------------
def moving_average_filter(data, window_size):
    numbers_series = pd.Series(data)

    windows = numbers_series.rolling(window_size)

    moving_averages = windows.mean()

    moving_averages_list = moving_averages.tolist()

    without_nans = moving_averages_list[window_size - 1:]
    
    return without_nans

# ----------------------------------
# It looks through the z[i:i+window_size] and comes 
# with a filtered speed of the robot with a bit more conservative heuristic
# if robot is:
#     - stationary for last #window_size frame set the speed to 0
#     - not stationary for last #window_size frame leave the speed as it is 
# It helps to come up with a bit more conservative speed values
def stop_start_filter(speed,window_size):
    speed_threshold = 0.1
    filtered_speed = []
    
    # Fill the first window_size element as there won't be any decision about them
    for idx in range(0, window_size):
        filtered_speed.append(0)
    
    for ele_idx in range(window_size, len(speed)):
        if maybeStationary(speed, window_size, ele_idx, speed_threshold):
            filtered_speed.append(speed[ele_idx])
        else:
            filtered_speed.append(0)
    return filtered_speed
        
def maybeStationary(data, window_size, ele_idx, speed_threshold):
    not_stationary = False
    for i in range(window_size):
        if (abs(data[ele_idx-i]) > speed_threshold):
            not_stationary = True
    return not_stationary