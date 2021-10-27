#!/bin/bash

import bagpy
from bagpy import bagreader

import math
import numpy as np
import pandas as pd

# ----------------------------------
def rosbag_topic_to_df(bagfile_path, topic): 
    b = bagreader(bagfile_path)
    data = b.message_by_topic(topic)
    df = pd.read_csv(data)
    return df

# ----------------------------------
def range_azimuth_to_x_y(range_val, azimuth_val):
    x = []
    y = []

    for rho, phi in zip(range_val, azimuth_val):
        x_, y_ = pol2cart(rho, phi)
        x.append(x_)
        y.append(y_)
   
    return (x,y)

# ----------------------------------
def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

# ----------------------------------
def range_and_azimuth_from_mask(df_t_k, ranges_col_names, mask):
    # sensor related info
    # fov := (360 - min_angle - max_angle) (-0.5, 5)
    laser_fov = 345.5
    laser_numb_of_samples = 720
    
    range_val = [df_t_k[ranges_col_names[i]] for i in range(mask[0], mask[1])]
    range_azimuth = [laser_fov*i/laser_numb_of_samples*(np.pi/180) for i in range(mask[0], mask[1])]
    
    filtered_range_val = []
    filtered_range_azimuth = []
    
    # filter the values    
    for index, ele in enumerate(range_val):
        if not math.isinf(ele):
            filtered_range_val.append(ele)
            filtered_range_azimuth.append(range_azimuth[index])
    
    return filtered_range_val, filtered_range_azimuth
