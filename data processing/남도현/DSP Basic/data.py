import pandas as pd
import numpy as np
from quaternion import *
import matplotlib.pyplot as plt
import config
import os
import plot_trajectory as trajectory


class DataFormat:
    def __init__(self, ax, ay, az, gx, gy, gz, mx, my, mz, lat, lon, pres, state=None):
        self.ax = ax
        self.ay = ay
        self.az = az
        self.gx = gx
        self.gy = gy
        self.gz = gz
        self.mx = mx
        self.my = my
        self.mz = mz
        self.vx = None
        self.vy = None
        self.vz = None
        self.rx = None
        self.ry = None
        self.rz = None
        self.lat = lat
        self.lon = lon
        self.pres = pres
        self.alt = 44307.69396 * (1.0 - pow(self.pres / 101325, 0.190284))
        self.q = None
        self.state = state


class Data:
    def __init__(self, file_path):
        file_path = os.path.abspath(file_path)
        self.csv = pd.read_csv(file_path)
        self.file_path = file_path
        self.data = [DataFormat(ax=self.csv.data['xAxisAcc'][i],
                                ay=self.csv.data['yAxisAcc'][i],
                                az=self.csv.data['zAxisAcc'][i],
                                gx=self.csv.data['xAxisAngVal'][i] * np.pi / 180,
                                gy=self.csv.data['yAxisAngVal'][i] * np.pi / 180,
                                gz=self.csv.data['zAxisAngVal'][i] * np.pi / 180,
                                mx=self.csv.data['xAxisMagF'][i],
                                my=self.csv.data['yAxisMagF'][i],
                                mz=self.csv.data['zAxisMagF'][i],
                                lat=self.csv.data['gpsLatitude'] * 1e-7,
                                lon=self.csv.data['gpsLongitude'] * 1e-7,
                                pres=self.csv.data['pressure']
                                )
                     for i in range(len(self.csv.data['xAxisAcc']))]


    def ekf(self):
        for idx in range(config.index["launch"], config.index["touchdown"]):
            self.predict(idx)
            self.update(idx)

    def summary(self):
        pass

    def plot_data(self, x, y):
        plt.figure(figsize=config.fig_size)
        plt.plot(x, y)
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_kalman_trajectory(self, ax, color=None, label=None):
        imu_trajectory = trajectory.IMUTrajectory(rotator=self.rotator, acc=self.acc, dt=self.dt, gravity=self.g0)
        imu_trajectory.calculate_trajectory(config.index["launch"], config.index["touchdown"])
        imu_trajectory.plot_trajectory(ax, color, label)
        # rocket.animate_trajectory()

    def plot_gps_trajectory(self, ax, color=None, label=None):
        gps_trajectory = trajectory.GPSTrajectory(latitude=self.latitude, longitude=self.longitude,
                                                  altitude=self.altitude)
        gps_trajectory.calculate_trajectory()
        gps_trajectory.plot_trajectory(ax, color, label)
        for i in range(len(gps_trajectory.altitude)):
            if max(gps_trajectory.altitude) == gps_trajectory.altitude[i]:
                config.index["apogee"] = i
                print(i)

    def print_time(self):
        for x in config.index:
            if x == "stand_by":
                pass
            else:
                config.time[x] = (config.index[x] - config.index["launch"]) * self.dt
        print(config.time)

    def plot_simple_trajectory(self, ax, color=None, label=None):
        simple_trajectory \
            = trajectory.SIMPLETrajectory(acc=self.acc, dt=self.dt, gravity=self.g0, gyro=self.gyro,
                                          rotator0=self.rotator[config.index["launch"]])
        simple_trajectory.calculate_trajectory(config.index["launch"], config.index["touchdown"])
        simple_trajectory.plot_trajectory(ax, color, label)
