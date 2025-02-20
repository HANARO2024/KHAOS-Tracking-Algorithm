import pandas as pd
import matplotlib.pyplot as plt
from config import *
from label import *
import os
fig_size = (10, 10)

class imu:
    def __init__(self, file_path):
        file_path = os.path.abspath(file_path)
        self.data = pd.read_csv(file_path)
        self.file_path = file_path
        self.t = (self.data['current_time'] - self.data['current_time'].min()) / 1000
        self.dt = (self.t[len(self.t)-1] - self.t[0]) / (len(self.t) - 1)
        self.a0 = quaternion()
        self.m0 = quaternion()
        
        for idx in acc:
            self.data[idx] *= g
        
        for idx in gyro:
            self.data[idx] *= np.pi/180

    def zero_offset(self):
        zero_data = self.data[self.data['state'] == 1.0]
        
        for idx in acc:
            acc[idx].mean = np.mean(zero_data[idx])
            acc[idx].stdev = np.std(zero_data[idx])
        
        for idx in gyro:
            gyro[idx].mean = np.mean(zero_data[idx])
            gyro[idx].stdev = np.std(zero_data[idx])
            
        for idx in mag:
            mag[idx].mean = np.mean(zero_data[idx])
            mag[idx].stdev = np.std(zero_data[idx])