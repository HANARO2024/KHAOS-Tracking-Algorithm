import numpy as np
from imu import *
from quaternion import *
from numpy import *
from rk4 import *

def rotate_axis(rotator):
    axis = {
        'x': None,
        'y': None,
        'z': None
    }

    for key in axis_label:
        axis[key] = rotation(rotator, axis_label[key])
    return axis


def rotate_coordinate(rotator, v):
    axis = rotate_axis(rotator)
    R = array([
        axis['x'].q[1:],
        axis['y'].q[1:],
        axis['z'].q[1:]
    ])
    R = R.T
    return np.dot(R, v)


def covariance(v):
    covar = [[0 for _ in range(len(v))] for _ in range(len(v))]
    for i in range(len(v)):
        for j in range(len(v)):
            covar[i][j] = v[i] ** 2 if i == j else 0
    return covar


class imu_ekf:
    def __init__(self, file_path):
        self.imu = imu(file_path=file_path)
        self.imu.zero_offset()
        self.rotator = [quaternion() for _ in range(len(self.imu.data['mx']))]

        for i in range(self.imu.N):
            if (self.imu.data['state'] == 2.0).any() or (self.imu.data[variable_label[Y].idx] > ground_height_threshold).any():

                self.start = i
                break

        for i in range(self.imu.N - 1, 0, -1):
            if (self.imu.data[variable_label[Y].idx] < ground_height_threshold).any():

                self.end = i
                break

        self.a = [quaternion() for _ in range(self.end - self.start + 1)]
        self.v = [quaternion() for _ in range(self.end - self.start + 1)]
        self.r = [quaternion() for _ in range(self.end - self.start + 1)]

        # State vector [q0, q1, q2, q3]
        z = quaternion([0, 0, 0, 1])
        if data_label['az'].mean < 0:
            z = quaternion([0, 0, 0, -1])

        axis = np.cross(z.vector(), self.imu.a0.vector())
        axis = np.array(axis[:3])
        angle = asin(np.linalg.norm(axis))
        self.rotator[0] = quaternion(angle=angle, axis=axis)

        # Covariance matrix
        self.P = eye(4) * 0.1

        self.w = [0.0, [data_label[key].stdev * self.imu.dt for key in ['gx', 'gy', 'gz']]]

        # Process noise covariance
        self.Q = covariance(self.w)

        # Measurement noise
        self.v = [data_label[key].stdev/data_label[key].mean for key in ['mx', 'my', 'mz']]

        # Measurement noise covariance
        self.R = covariance(self.v)

        # Time step
        self.dt = self.imu.dt

    # file:///C:/Users/82108/Desktop/Hanaro/Data%20Processing/References/quaternion-Based_Iterative_Extended_Kalman_Filter_for_Sensor_Fusion_of_Vision_Sensor_and_IMU_in_6-DOF_Displacement_Monitoring.pdf

    # def initialization(self):
    #     # self.swap_axis('x', 'z')
    #     # self.swap_axis('y', 'z')
    #     # for axis in ['x', 'y', 'z']:
    #     #     self.reverse_axis(axis)
    #
    #     z = quaternion([0, 0, 0, 1])
    #     a0 = quaternion([0, data_label['ax'].mean, data_label['ay'].mean, data_label['az'].mean]).normalize()
    #     if data_label['az'].mean < 0:
    #         z = quaternion([0, 0, 0, -1])
    #
    #     axis = np.cross(z.vector(), a0.vector())
    #     angle = asin(np.linalg.norm(axis))
    #     return quaternion(angle=angle, axis=axis)

    def reverse_axis(self, axis):
        data_key = ['a', 'g', 'm']
        for sub_key in data_key:
            key = sub_key + axis
            self.imu.data[key] *= -1

    def swap_axis(self, axis1, axis2):
        data_key = ['a', 'g', 'm']
        for sub_key in data_key:
            key1 = sub_key + axis1
            key2 = sub_key + axis2
            self.imu.data[key1], self.imu.data[key2] = self.imu.data[key2], self.imu.data[key1]

    def predict(self, idx):
        # Gyroscope measurements (rad/s)
        wx, wy, wz = self.imu.data['gx'][idx], self.imu.data['gy'][idx], self.imu.data['gz'][idx]

        # quaternion rate equation
        O = array((
            [0, -wx, -wy, -wz],
            [wx, 0, wz, -wy],
            [wy, -wz, 0, wx],
            [wz, wy, -wx, 0]
        ))

        # State prediction
        self.rotator[idx] = self.rotator[idx] + 0.5 * np.dot(O, self.rotator[idx].q) * self.dt
        # CHECK!
        self.rotator[idx] = self.rotator[idx].normalize()

        # Jacobian of the prediction function
        F = eye(4) + O * self.dt

        # Covariance prediction
        self.P = dot(F, dot(self.P, F.T)) + self.Q

    def update(self, idx):
        # Normalize magnetometer measurements

        z = quaternion([0, self.imu.data['mx'][idx], self.imu.data['my'][idx], self.imu.data['mz'][idx]]).normalize()
        q = self.rotator[idx].q
        m0 = self.imu.m0.q

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
        #
        # H_Jacobian = array([
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
        H = quaternion_to_matrix(self.rotator) * quaternion_to_matrix(self.imu.m0) * quaternion_to_matrix(self.rotator.conjugate()) * quaternion_to_matrix(self.rotator.conjugate())

        # Kalman gain
        # S = dot(H_Jacobian, dot(self.P, H_Jacobian.T)) + self.R
        # K = dot(self.P, dot(H_Jacobian.T, np.linalg.inv(S)))
        S = dot(H, dot(self.P, H.T)) + self.R
        K = dot(self.P, dot(H.T, np.linalg.inv(S)))

        # State update
        self.rotator[idx + 1] = self.rotator[idx] + dot(K, z - dot(H, self.rotator[idx].q))
        self.rotator[idx + 1] = self.rotator[idx + 1].normalize()

        # Covariance update
        self.P = self.P - dot(K, dot(H, self.P))

    # def update_orientation(self, idx):
    #     z = quaternion([0, 0, 0, 1])
    #     self.rotator[idx] = self.rotator.normalize()
    #     self.orientation[idx] = self.rotator * z * self.rotator.conjugate()

    def extended_kalman_filter(self):
        # self.initialization()
        for i in range(self.start, self.end):
            idx = i - self.start
            self.predict(idx)
            self.update(idx)
            # self.update_orientation(i)

    def simulate(self):
        for i in range(self.end - self.start + 1):
            acc = [self.imu.data[key] for key in ['ax', 'ay', 'az']]
            self.a[i] = rotate_coordinate(self.rotator[i], acc)
        imu_rk4 = rk4(a=self.a, dt=self.dt)
        imu_rk4.integrate()

        self.r = imu_rk4.r

    def plot_path(self):
        fig = plt.figure(figsize=fig_size)
        graph = fig.add_subplot(1, 1, 1, projection="3d")
        X = [self.r[i].q[1] for i in range(len(self.r))]
        Y = [self.r[i].q[2] for i in range(len(self.r))]
        Z = [self.r[i].q[3] for i in range(len(self.r))]
        graph.scatter(X, Y, Z)

        plot_path = self.imu.file_path.replace('.csv', '_path.jpg')
        plt.savefig(plot_path)
        plt.show()


if __name__ == '__main__':
    file_path = r"C:\Users\82108\Desktop\Hanaro\Avionics\ID3\SD log\ID3 2024.06.29\FC1_DATA_LOG240629.csv"
    obj = imu_ekf(file_path)
    obj.simulate()
    obj.plot_path()

