from .imu_trajectory import IMUTrajectory
import numpy as np
from scipy.spatial.transform import Rotation as R
import config

class SIMPLETrajectory(IMUTrajectory):
    def __init__(self, acc, dt, gravity, gyro, rotator0, direction):
        super().__init__(acc=acc, dt=dt, rotator=None, direction=direction, gravity=gravity)
        self.gyro = gyro
        self.rotator0 = rotator0
        print(self.rotator0)
        self.rotator = self.generate_rotator()

    def generate_rotator(self):
        """Generate rotation matrices using gyro data."""
        rotator = [self.rotator0 for _ in range(config.index["stand_by"], config.index["launch"])]
        for i in range(config.index["launch"], config.index["touchdown"]):
            angular_velocity = np.array([self.gyro["x"][i], self.gyro["y"][i], self.gyro["z"][i]])
            angle = np.linalg.norm(angular_velocity) * self.dt
            if angle > 0:
                axis = angular_velocity / np.linalg.norm(angular_velocity)
                delta_rotation = R.from_rotvec(axis * angle)
                current_rotation = R.from_quat(rotator[-1])
                new_rotation = (current_rotation * delta_rotation).as_quat()
            else:
                new_rotation = rotator[-1]
            rotator.append(new_rotation)
        return rotator

    def calculate_trajectory(self, launch_index, touchdown_index):
        super().calculate_trajectory(launch_index, touchdown_index, False)

    def plot_trajectory(self, ax, color, label):
        super().plot_trajectory(ax, color, label)