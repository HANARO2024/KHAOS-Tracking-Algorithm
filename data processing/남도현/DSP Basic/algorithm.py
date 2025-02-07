class Algorithm:
    def __init__(self, data):
        self.data = data


    def predict(self):
        # wx, wy, wz = self.gyro['x'][idx], self.gyro['y'][idx], self.gyro['z'][idx]
        # axis = np.array([self.gyro['x'][idx], self.gyro['y'][idx], self.gyro['z'][idx]])
        # theta = norm(axis) * self.dt
        # axis = axis / norm(axis)
        #
        # q = [sin(theta / 2), cos(theta / 2) * axis[0], cos(theta / 2) * axis[1], cos(theta / 2) * axis[2]]
        # self.rotator[idx + 1] = matrix_right(q) * self.rotator[idx]
        #
        # O = array((
        #     [0, -wx, -wy, -wz],
        #     [wx, 0, wz, -wy],
        #     [wy, -wz, 0, wx],
        #     [wz, wy, -wx, 0]
        # ))
        # F = np.eye(4) + O * self.dt / 2
        # self.P = np.dot(F, np.dot(self.P, np.transpose(F))) + self.Q
        pass

    def update(self):
        # # q = self.rotator[idx].q
        # q = self.rotator[idx]
        # m0 = self.magg
        # # z = quaternion.build_from_list(None, [0, self.mag['x'][idx], self.mag['y'][idx], self.mag['z'][idx]])
        # z = np.array([0, self.mag['x'][idx], self.mag['y'][idx], self.mag['z'][idx]])
        #
        # # H = array([
        # #     [2 * (q[0] * m0[0] + q[2] * m0[2] - q[3] * m0[1]),
        # #      2 * (q[1] * m0[0] + q[2] * m0[1] + q[3] * m0[2]),
        # #      2 * (-q[2] * m0[0] + q[1] * m0[2] - q[0] * m0[1]),
        # #      2 * (-q[3] * m0[0] - q[0] * m0[2] + q[1] * m0[1])],
        # #
        # #     [2 * (-q[3] * m0[2] + q[0] * m0[1] + q[1] * m0[0]),
        # #      2 * (q[0] * m0[0] - q[1] * m0[2] + q[2] * m0[1]),
        # #      2 * (q[3] * m0[0] + q[2] * m0[2] + q[1] * m0[1]),
        # #      2 * (-q[2] * m0[0] - q[1] * m0[1] - q[0] * m0[2])],
        # #
        # #     [2 * (q[2] * m0[1] - q[3] * m0[0] + q[0] * m0[2]),
        # #      2 * (q[3] * m0[0] - q[0] * m0[2] - q[1] * m0[1]),
        # #      2 * (q[0] * m0[0] + q[1] * m0[2] + q[2] * m0[1]),
        # #      2 * (-q[1] * m0[0] + q[2] * m0[2] - q[3] * m0[1])]
        # # ])
        # # H = (quaternion_to_quaternion_matrix(self.rotator) * quaternion_to_quaternion_matrix(self.imu.m0) * quaternion_to_quaternion_matrix(
        # #     self.rotator.conjugate()) * quaternion_to_quaternion_matrix(self.rotator.conjugate()))
        # # H = quaternion_matrix_left(self.rotator[idx]) * quaternion_matrix_left(m0) * quaternion_matrix_left(self.rotator[idx])
        # H = (matrix_left(self.rotator[idx]) *
        #      matrix_left(m0) *
        #      matrix_left(self.rotator[idx]) *
        #      matrix_left(self.rotator[idx]))
        #
        # # Kalman gain
        # # S = dot(H_Jacobian, dot(self.P, H_Jacobian.T)) + self.R
        # # K = dot(self.P, dot(H_Jacobian.T, np.linalg.inv(S)))
        #
        # # S = np.dot(H, np.dot(self.P, np.transpose(H))) + self.R
        # # K = np.dot(self.P, np.dot(H.T, np.linalg.inv(S)))
        #
        # S = np.dot(H, np.dot(self.P, np.transpose(H))) + self.R
        # K = np.dot(self.P, np.dot(np.transpose(H), np.linalg.inv(S)))
        #
        # # State update
        # self.rotator[idx + 1] = self.rotator[idx] + np.dot(K, z - np.dot(H, self.rotator[idx]))
        # self.rotator[idx + 1] = self.rotator[idx + 1] / norm(self.rotator[idx + 1])
        #
        # # Covariance update
        # self.P = self.P - np.dot(K, np.dot(H, self.P))
        pass



