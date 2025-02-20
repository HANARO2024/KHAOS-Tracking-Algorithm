import pandas as pd
import data
import matplotlib.pyplot as plt
import config

file_path = r"C:\Users\82108\Desktop\Hanaro\Data Processing\Data\identity3-B_lora_data.csv"
fig_size = (10, 10)

if __name__ == '__main__':
    rocket = data.data_format(file_path)
    rocket.preprocessing()
    rocket.ekf()
    rocket.rk4()
    rocket.postprocessing()
    rocket.plot_imu_trajectory()

# Memo
'''
좌표계: 
body, obs

데이터:
acc[i] = [ax, ay, az]
gyro[i] = [gx, gy, gz]
mag[i] = [mx, my, mz]

acc['body'][i]
gyro['body'][i]
'''