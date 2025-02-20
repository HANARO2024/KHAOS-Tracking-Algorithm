import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import config
import os
import plot_trajectory as trajectory


class SensorData:
    def __init__(self, file_path):
        file_path = os.path.abspath(file_path)
        self.data = pd.read_csv(file_path)
        self.file_path = file_path

        self.gyro0 = 0
        self.mg = None
        self.gg = None
        self.dt = 0.1

        self.acc = {
            'x': self.data['xAxisAcc'],
            'y': self.data['yAxisAcc'],
            'z': self.data['zAxisAcc']
        }

        self.gyro = {
            'x': self.data['xAxisAngVal'] * np.pi / 180,
            'y': self.data['yAxisAngVal'] * np.pi / 180,
            'z': self.data['zAxisAngVal'] * np.pi / 180
        }

        self.mag = {
            'x': self.data['xAxisMagF'],
            'y': self.data['yAxisMagF'],
            'z': self.data['zAxisMagF']
        }

        self.pos = {
            'x': [np.array([]) for _ in range(len(self.mag['x']))],
            'y': [np.array([]) for _ in range(len(self.mag['x']))],
            'z': [np.array([]) for _ in range(len(self.mag['x']))]
        }

        self.latitude = self.data['gpsLatitude'] * 1e-7
        self.longitude = self.data['gpsLongitude'] * 1e-7
        self.pressure = self.data['pressure']
        self.altitude = 44307.69396 * (1.0 - pow(self.pressure / 101325, 0.190284))
        self.ground_altitude = min(self.altitude)
        self.altitude -= self.ground_altitude
        self.time = self.data['deviceTime'] / 1000 / self.dt

        # self.rotator = [quaternion() for _ in range(len(self.mag['x']))]
        self.rotator = [np.array([]) for _ in range(len(self.mag['x']))]
        self.v = [np.array([]) for _ in range(len(self.mag['x']))]
        self.r = [np.array([]) for _ in range(len(self.mag['x']))]

        self.P = None
        self.Q = None
        self.v = None
        self.R = None

    def find_index(self):
        i = 0
        # while self.data['flag'][i] == 'ffff':
        #     i += 1
        # config.index["stand_by"] = i
        #
        # i = config.index["stand_by"]
        # while abs(self.acc['x'][i]**2 + self.acc['y'][i]**2 + self.acc['z'][i]**2 - 1) < config.a_threshold:
        #     i += 1
        # config.index["launch"] = i

        config.index["stand by"] = 0
        i = config.index["stand by"]

        self.g0 = np.sqrt(self.acc['x'][config.index["stand by"]] ** 2 +
                          self.acc['y'][config.index["stand by"]] ** 2 +
                          self.acc['z'][config.index["stand by"]] ** 2)
        while abs(self.g0 - np.sqrt(
                self.acc['x'][i] ** 2 + self.acc['y'][i] ** 2 + self.acc['z'][i] ** 2)) < self.g0 * 0.05:
            i += 1
        config.index["launch"] = i

        # while abs(self.g0 - np.sqrt(self.acc['x'][i] ** 2 + self.acc['y'][i] ** 2 + self.acc['z'][i] ** 2)) > self.g0 * 0.05:
        #     i += 1
        # config.index["apogee"] = i
        # How to separate states?

        i = len(self.acc['x']) - 1
        while abs(self.g0 - np.sqrt(
                self.acc['x'][i] ** 2 + self.acc['y'][i] ** 2 + self.acc['z'][i] ** 2)) < self.g0 * 0.05:
            i -= 1
        config.index["touchdown"] = i
        print(config.index)

    def set_initial_condition(self):
        self.find_index()
        gx0 = self.acc['x'][config.index["stand by"]:config.index["launch"]].mean()
        gy0 = self.acc['y'][config.index["stand by"]:config.index["launch"]].mean()
        gz0 = self.acc['z'][config.index["stand by"]:config.index["launch"]].mean()
        gg0 = [gx0, gy0, gz0]
        gg0 = np.array(gg0)

        self.gg = [0, 0, (-1) * self.g0]
        self.gg = np.array(self.gg)

        axis = np.cross(gg0, self.gg)
        axis = axis / norm(axis)
        angle = np.asin(np.linalg.norm(axis) / self.g0 / np.linalg.norm(gg0))
        # self.rotator[config.index["launch"]] = quaternion.build_from_angle(0, angle, axis)
        self.rotator[config.index["launch"]] = \
            [np.cos(angle),
             np.sin(angle) * axis[0],
             np.sin(angle) * axis[1],
             np.sin(angle) * axis[2]]

        mx0 = self.mag['x'][config.index["stand by"]:config.index["launch"]].mean()
        my0 = self.mag['y'][config.index["stand by"]:config.index["launch"]].mean()
        mz0 = self.mag['z'][config.index["stand by"]:config.index["launch"]].mean()
        self.magg = [0, mx0, my0, mz0]

        self.P = np.eye(4) * 0.1
        self.w = [0.0,
                  self.mag['x'][config.index["stand by"]:config.index["launch"]].std(),
                  self.mag['y'][config.index["stand by"]:config.index["launch"]].std(),
                  self.mag['z'][config.index["stand by"]:config.index["launch"]].std()
                  ]
        self.Q = covariance(self.w)
        self.v = [0.0,
                  self.gyro['x'][config.index["stand by"]:config.index["launch"]].std() * self.dt,
                  self.gyro['y'][config.index["stand by"]:config.index["launch"]].std() * self.dt,
                  self.gyro['z'][config.index["stand by"]:config.index["launch"]].std() * self.dt
                  ]
        self.R = covariance(self.v)

    def predict(self, idx):
        wx, wy, wz = self.gyro['x'][idx], self.gyro['y'][idx], self.gyro['z'][idx]
        axis = np.array([self.gyro['x'][idx], self.gyro['y'][idx], self.gyro['z'][idx]])
        theta = norm(axis) * self.dt
        axis = axis / norm(axis)

        q = [sin(theta / 2), cos(theta / 2) * axis[0], cos(theta / 2) * axis[1], cos(theta / 2) * axis[2]]
        self.rotator[idx + 1] = matrix_right(q) * self.rotator[idx]

        O = array((
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0]
        ))
        F = np.eye(4) + O * self.dt / 2
        self.P = np.dot(F, np.dot(self.P, np.transpose(F))) + self.Q

    def update(self, idx):
        # q = self.rotator[idx].q
        q = self.rotator[idx]
        m0 = self.magg
        # z = quaternion.build_from_list(None, [0, self.mag['x'][idx], self.mag['y'][idx], self.mag['z'][idx]])
        z = np.array([0, self.mag['x'][idx], self.mag['y'][idx], self.mag['z'][idx]])

        # H = array([
        #     [2 * (q[0] * m0[0] + q[2] * m0[2] - q[3] * m0[1]),
        #      2 * (q[1] * m0[0] + q[2] * m0[1] + q[3] * m0[2]),
        #      2 * (-q[2] * m0[0] + q[1] * m0[2] - q[0] * m0[1]),
        #      2 * (-q[3] * m0[0] - q[0] * m0[2] + q[1] * m0[1])],
        #
        #     [2 * (-q[3] * m0[2] + q[0] * m0[1] + q[1] * m0[0]),
        #      2 * (q[0] * m0[0] - q[1] * m0[2] + q[2] * m0[1]),
        #      2 * (q[3] * m0[0] + q[2] * m0[2] + q[1] * m0[1]),
        #      2 * (-q[2] * m0[0] - q[1] * m0[1] - q[0] * m0[2])],
        #
        #     [2 * (q[2] * m0[1] - q[3] * m0[0] + q[0] * m0[2]),
        #      2 * (q[3] * m0[0] - q[0] * m0[2] - q[1] * m0[1]),
        #      2 * (q[0] * m0[0] + q[1] * m0[2] + q[2] * m0[1]),
        #      2 * (-q[1] * m0[0] + q[2] * m0[2] - q[3] * m0[1])]
        # ])
        # H = (quaternion_to_quaternion_matrix(self.rotator) * quaternion_to_quaternion_matrix(self.imu.m0) * quaternion_to_quaternion_matrix(
        #     self.rotator.conjugate()) * quaternion_to_quaternion_matrix(self.rotator.conjugate()))
        # H = quaternion_matrix_left(self.rotator[idx]) * quaternion_matrix_left(m0) * quaternion_matrix_left(self.rotator[idx])
        H = (matrix_left(self.rotator[idx]) *
             matrix_left(m0) *
             matrix_left(self.rotator[idx]) *
             matrix_left(self.rotator[idx]))

        # Kalman gain
        # S = dot(H_Jacobian, dot(self.P, H_Jacobian.T)) + self.R
        # K = dot(self.P, dot(H_Jacobian.T, np.linalg.inv(S)))

        # S = np.dot(H, np.dot(self.P, np.transpose(H))) + self.R
        # K = np.dot(self.P, np.dot(H.T, np.linalg.inv(S)))

        S = np.dot(H, np.dot(self.P, np.transpose(H))) + self.R
        K = np.dot(self.P, np.dot(np.transpose(H), np.linalg.inv(S)))

        # State update
        self.rotator[idx + 1] = self.rotator[idx] + np.dot(K, z - np.dot(H, self.rotator[idx]))
        self.rotator[idx + 1] = self.rotator[idx + 1] / norm(self.rotator[idx + 1])

        # Covariance update
        self.P = self.P - np.dot(K, np.dot(H, self.P))

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
