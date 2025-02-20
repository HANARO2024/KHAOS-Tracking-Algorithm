import pandas as pd
import numpy as np
from quaternion import *
import matplotlib.pyplot as plt
import config
import os


def matrix_left(q):
    q0, q1, q2, q3 = q
    Q_left = array([
        [q0, -q1, -q2, -q3],
        [q1, q0, -q3, q2],
        [q2, q3, q0, -q1],
        [q3, -q2, q1, q0]
    ])
    return Q_left


def matrix_right(q):
    q0, q1, q2, q3 = q
    Q_right = np.array([
        [q0, -q1, -q2, -q3],
        [q1, q0, q3, -q2],
        [q2, -q3, q0, q1],
        [q3, q2, -q1, q0]
    ])
    return Q_right


def covariance(v):
    covar = [[0 for _ in range(len(v))] for _ in range(len(v))]
    for i in range(len(v)):
        for j in range(len(v)):
            covar[i][j] = v[i] ** 2 if i == j else 0
    return covar

def lat_lon_to_xy(lat, lon, lat0, lon0, R=6371000):
    # Convert degrees to radians
    lat, lon, lat0, lon0 = map(np.radians, [lat, lon, lat0, lon0])

    # Calculate relative coordinates
    x = R * np.cos(lat0) * (lon - lon0)
    y = R * (lat - lat0)
    return x, y


# Example usage:
lat, lon = 37.7749, -122.4194  # Rocket location (San Francisco)
lat0, lon0 = 34.0522, -118.2437  # Reference point (Los Angeles)

x, y = lat_lon_to_xy(lat, lon, lat0, lon0)
print(f"Relative Coordinates: x = {x:.2f} m, y = {y:.2f} m")


class data_format:
    def __init__(self, file_path):
        file_path = os.path.abspath(file_path)
        self.data = pd.read_csv(file_path)
        self.file_path = file_path
        self.dt = 0.1
        config.L = len(self.data['xAxisAcc'])

        self.acceleration = {
            'body': [
                np.array([self.data['xAxisAcc'][idx],
                          self.data['yAxisAcc'][idx],
                          self.data['zAxisAcc'][idx]])
                for idx in range(config.L)
            ],
            'obs': [
                np.array([0, 0, 0]) for _ in range(config.L)
            ]
        }

        self.gyroscopic_rate = {
            'body': [
                np.array([self.data['xAxisAngVal'][idx],
                          self.data['yAxisAngVal'][idx],
                          self.data['zAxisAngVal'][idx]])
                for idx in range(config.L)
            ],
            'obs': []
        }

        self.magnetic_field = {
            'body': [
                np.array([self.data['xAxisMagF'][idx],
                          self.data['yAxisMagF'][idx],
                          self.data['zAxisMagF'][idx]])
                for idx in range(config.L)
            ],
            'obs': [0, 0, 0]
        }

        self.velocity = {
            'body': [
                np.array([0, 0, 0]) for idx in range(config.L)
            ],
            'obs': [
                np.array([0, 0, 0]) for idx in range(config.L)
            ]
        }

        self.position = {
            'imu': [
                np.array([0, 0, 0]) for idx in range(config.L)
            ],
            'gps': [
                np.array([0, 0, 0]) for idx in range(config.L)
            ]
        }

        self.gravitational_acceleration = {
            'body': [0, 0, 0],
            'obs': [0, 0, 0],
            # 'norm': 0
        }

        self.axis = np.array([[1, 0, 0],
                              [0, 1, 0],
                              [0, 0, 1]])

        self.time = self.data['time']
        self.rotator = [np.array([]) for _ in range(config.L)]

        self.P = None
        self.w = None
        self.Q = None
        self.v = None
        self.R = None

    def offset(self):
        # calibration step
        pass

    def find_index(self):
        i = 0
        config.index["stand_by"] = 0
        i = config.index["stand_by"]
        config.g = norm(self.acceleration['body'][config.index["stand_by"]])

        while abs(config.g - norm(self.acceleration['body'][i])) / config.g < config.acceleration_threshold:
            i += 1
        config.index["launch"] = i

        i = config.L - 1
        while abs(config.g - norm(self.acceleration['body'][i])) / config.g < config.acceleration_threshold:
            i -= 1
        config.index["touchdown"] = i

    def preprocessing(self):
        self.find_index()
        self.gravitational_acceleration['body'] = (
            np.mean(self.acceleration['body'][config.index["stand_by"]:config.index["launch"]],
                    axis=0)
        )
        config.g = norm(self.gravitational_acceleration['body'])

        self.gravitational_acceleration['obs'] = [0, 0, (-1) * config.g]

        axis = np.cross(self.gravitational_acceleration['body'], self.gravitational_acceleration['obs'])
        angle = np.asin(np.linalg.norm(axis)
                        / norm(self.gravitational_acceleration['body'])
                        / norm(self.gravitational_acceleration['obs']))

        self.rotator[config.index["launch"]] = [
            np.cos(angle / 2),
            np.sin(angle / 2) * axis[0],
            np.sin(angle / 2) * axis[1],
            np.sin(angle / 2) * axis[2]
        ]

        self.magnetic_field['obs'] \
            = np.append([0],
                        np.mean(self.magnetic_field['body'][config.index["stand_by"]:config.index["launch"]],
                                axis=0))

        self.P = np.eye(4) * 0.1
        self.w \
            = np.append([0],
                        np.std(self.magnetic_field['body'][config.index["stand_by"]:config.index["launch"]],
                               axis=0))
        self.Q = covariance(self.w)
        self.v \
            = np.append([0],
                        np.std(self.gyroscopic_rate['body'][config.index["stand_by"]:config.index["launch"]],
                               axis=0)) * self.dt
        self.R = covariance(self.v)

    def predict(self, idx):
        wx, wy, wz = self.gyroscopic_rate['body'][idx]
        q = [0, 0.5 * wx * self.dt, 0.5 * wy * self.dt, 0.5 * wz * self.dt]
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
        q = self.rotator[idx]
        m0 = self.magnetic_field['obs']
        z = np.append([0],self.magnetic_field['body'][idx])
        H = (matrix_left(self.rotator[idx]) *
             matrix_left(m0) *
             matrix_left(self.rotator[idx]) *
             matrix_left(self.rotator[idx]))

        # Kalman gain

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

    def postprocessing(self):
        for idx in range(config.index["launch"], config.index["touchdown"]):
            q = self.rotator[idx]
            a_body = np.append([0], self.acceleration['body'][idx])
            q_conj = [q[0], -q[1], -q[2], -q[3]]
            a_obs_quat = matrix_left(q).dot(a_body).dot(matrix_right(q_conj))

            self.acceleration['obs'][idx][0] = a_obs_quat[1] - self.gravitational_acceleration['obs'][0]
            self.acceleration['obs'][idx][1] = a_obs_quat[2] - self.gravitational_acceleration['obs'][1]
            self.acceleration['obs'][idx][2] = a_obs_quat[3] - self.gravitational_acceleration['obs'][2]

    def rk4(self):
        for idx in range(config.index["launch"], config.index["touchdown"] - 1):
            dt = self.dt
            a_obs = self.acceleration['obs'][idx]
            # Current velocity and position
            v_current = self.velocity['obs'][idx]
            p_current = self.position['imu'][idx]

            # RK4 steps
            k1_v = np.array([a_obs[i] * dt for i in range(len(a_obs))])
            k1_p = np.array([v_current[i] * dt for i in range(len(v_current))])

            k2_v = (a_obs + 0.5 * k1_v) * dt
            k2_p = (v_current + 0.5 * k1_v) * dt

            k3_v = (a_obs + 0.5 * k2_v) * dt
            k3_p = (v_current + 0.5 * k2_v) * dt

            k4_v = (a_obs + k3_v) * dt
            k4_p = (v_current + k3_v) * dt

            # Update velocity and position
            v_next = v_current + (k1_v + 2 * k2_v + 2 * k3_v + k4_v) / 6
            p_next = p_current + (k1_p + 2 * k2_p + 2 * k3_p + k4_p) / 6

            # Save results
            self.velocity['obs'][idx + 1] = v_next
            self.position['imu'][idx + 1] = p_next

    def plot_imu_trajectory(self):
        x = [self.position['imu'][i][0] for i in range(config.index["launch"], config.index["touchdown"])]
        y = [self.position['imu'][i][1] for i in range(config.index["launch"], config.index["touchdown"])]
        z = [self.position['imu'][i][2] for i in range(config.index["launch"], config.index["touchdown"])]

        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(x, y, z, label="Rocket Trajectory", color="blue", linewidth=2)

        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        ax.set_zlabel("Z Position (m)")
        ax.set_title("Rocket Trajectory in Observer Frame")

        ax.legend()

        ax.grid(True)
        plt.show()
