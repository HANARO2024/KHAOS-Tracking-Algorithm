import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import config
import os
import data
import plot_trajectory as trajectory


class PreProcessing:
    def __init__(self, data):
        self.data = data

    def find_index(self):
        config.index["armed"] = 0
        i = config.index["armed"]

        config.ground_gravity = np.sqrt(
            self.data[i].ax ** 2 +
            self.data[i].ay ** 2 +
            self.data[i].az ** 2
        )

        while (abs(config.ground_gravity - np.sqrt(self.data[i].ax ** 2 + self.data[i].ay ** 2 + self.data[i].az ** 2))
               < config.ground_gravity * 0.05):
            i += 1
        config.index["launch"] = i

        i = len(self.data) - 1
        while (abs(config.ground_gravity - np.sqrt(self.data[i].ax ** 2 + self.data[i].ay ** 2 + self.data[i].az ** 2))
               < config.ground_gravity * 0.05):
            i -= 1
        config.index["touchdown"] = i
        # print(config.index)

    def initialization(self):
        ax0 = np.mean(data[config.index["armed"]:config.index["launch"]].ax)
        ay0 = np.mean(data[config.index["armed"]:config.index["launch"]].ay)
        az0 = np.mean(data[config.index["armed"]:config.index["launch"]].az)
        a0 = [ax0, ay0, az0]
        config.ground_gravity = a0

        ground_gravity = [0, 0, (-1) * config.ground_gravity]
        axis = np.cross(a0, ground_gravity)
        axis = axis / norm(axis)
        angle = np.asin(np.linalg.norm(axis) / a0 ** 2)
        # self.rotator[config.index["launch"]] = quaternion.build_from_angle(0, angle, axis)
        self.rotator[config.index["launch"]] = \
            [np.cos(angle),
             np.sin(angle) * axis[0],
             np.sin(angle) * axis[1],
             np.sin(angle) * axis[2]]

        gx0 = np.mean(data[config.index["armed"]:config.index["launch"]].gx)
        gy0 = np.mean(data[config.index["armed"]:config.index["launch"]].gy)
        gz0 = np.mean(data[config.index["armed"]:config.index["launch"]].gz)

        for i in range(self.data):
            self.data[i].gx -= gx0
            self.data[i].gy -= gy0
            self.data[i].gz -= gz0

        mx0 = np.mean(data[config.index["armed"]:config.index["launch"]].mx)
        my0 = np.mean(data[config.index["armed"]:config.index["launch"]].my)
        mz0 = np.mean(data[config.index["armed"]:config.index["launch"]].mz)
        config.m0 = [0, mx0, my0, mz0]
        config.std_err = data.DataFormat(
            ax=np.std(data[config.index["armed"]:config.index["launch"]].ax),
            ay=np.std(data[config.index["armed"]:config.index["launch"]].ay),
            az=np.std(data[config.index["armed"]:config.index["launch"]].az),
            gx=np.std(data[config.index["armed"]:config.index["launch"]].gx),
            gy=np.std(data[config.index["armed"]:config.index["launch"]].gy),
            gz=np.std(data[config.index["armed"]:config.index["launch"]].gz),
            mx=np.std(data[config.index["armed"]:config.index["launch"]].mx),
            my=np.std(data[config.index["armed"]:config.index["launch"]].my),
            mz=np.std(data[config.index["armed"]:config.index["launch"]].mz),
            lat=None,
            lon=None,
            pres=None
        )
