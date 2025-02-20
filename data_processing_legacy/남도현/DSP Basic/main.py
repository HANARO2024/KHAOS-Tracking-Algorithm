import pandas as pd
import matplotlib.pyplot as plt
import config as cf
import data
import preprocessing
import algorithm
from trajectories import IMUTrajectory, GPSTrajectory, SimpleTrajectory

file_path = r"C:\Users\82108\Desktop\Hanaro\Data Processing\Data\identity3-B_lora_data.csv"
fig_size = (10, 10)

if __name__ == '__main__':
    file = data.Data(file_path=file_path)
    pre = preprocessing.PreProcessing(file.data)
    pre.find_index()
    pre.initialization()

