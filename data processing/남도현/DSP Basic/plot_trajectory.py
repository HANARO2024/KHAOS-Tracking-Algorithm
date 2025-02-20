import numpy as np
import matplotlib.pyplot as plt
import config
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

class PlotTrajectory:
    def __init__(self, data):

class IMUTrajectory:
    def __init__(self, rotator, acc, dt, gravity):
        self.rotator = rotator  # List of quaternions
        self.acc = acc  # Dictionary of accelerations in body frame
        self.dt = dt  # Time step
        self.pos = {"x": [], "y": [], "z": []}  # Position in observer frame
        self.gravity = gravity

    def acceleration_in_observer_frame(self, index, velocity):
        # Transform acceleration from body frame to observer frame
        quaternion = self.rotator[index]
        rotation_matrix = R.from_quat(quaternion).as_matrix()
        acc_body = np.array([self.acc['x'][index], self.acc['y'][index], self.acc['z'][index]])
        return rotation_matrix @ acc_body + [0, 0, self.gravity]

    def rk4_step(self, position, velocity, index):
        dt = self.dt
        # k1
        a1 = self.acceleration_in_observer_frame(index, velocity)
        v1 = velocity
        p1 = position

        # k2
        a2 = self.acceleration_in_observer_frame(index, velocity + 0.5 * a1 * dt)
        v2 = velocity + 0.5 * a1 * dt
        p2 = position + 0.5 * v1 * dt

        # k3
        a3 = self.acceleration_in_observer_frame(index, velocity + 0.5 * a2 * dt)
        v3 = velocity + 0.5 * a2 * dt
        p3 = position + 0.5 * v2 * dt

        # k4
        a4 = self.acceleration_in_observer_frame(index, velocity + a3 * dt)
        v4 = velocity + a3 * dt
        p4 = position + v3 * dt

        # Combine
        velocity_next = velocity + (a1 + 2 * a2 + 2 * a3 + a4) * dt / 6
        position_next = position + (v1 + 2 * v2 + 2 * v3 + v4) * dt / 6
        return position_next, velocity_next

    def calculate_trajectory(self, launch_index, touchdown_index):
        velocity = np.array([0, 0, 0])  # Initial velocity in observer frame
        position = np.array([0, 0, 0])  # Initial position in observer frame

        # Iterate over the specified time steps
        for i in range(launch_index, touchdown_index):
            position, velocity = self.rk4_step(position, velocity, i)
            # Store position
            self.pos["x"].append(position[0])
            self.pos["y"].append(position[1])
            self.pos["z"].append(position[2])

    def plot_trajectory(self, ax, color, label):
        if ax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(self.pos["x"], self.pos["y"], self.pos["z"], label=label, color=color)

            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("Z (m)")
            ax.legend()
            plt.show()

        # Plot trajectory
        ax.plot(self.pos["x"], self.pos["y"], self.pos["z"], label=label, color=color)

        # ax.set_xlabel("X (m)")
        # ax.set_ylabel("Y (m)")
        # ax.set_zlabel("Z (m)")
        # ax.legend()
        # plt.show()

    def animate_trajectory(self):
        # Ensure self.pos["x"], self.pos["y"], and self.pos["z"] are initialized
        if not all(k in self.pos for k in ("x", "y", "z")):
            raise ValueError("Position data is not properly initialized.")

        fig, ax = plt.subplots()
        ax.set_xlim(min(self.pos["x"]), max(self.pos["x"]))
        ax.set_ylim(min(self.pos["y"]), max(self.pos["y"]))

        point, = ax.plot([], [], 'ro')

        def update(frame):
            # Update position for animation
            point.set_data(self.pos["x"][frame], self.pos["y"][frame])
            return point,

        # anim = FuncAnimation(fig, update, frames=len(self.pos["x"]), interval=50, blit=True)
        # plt.show()


class GPSTrajectory:
    def __init__(self, latitude, longitude, altitude):
        self.latitude = np.radians(latitude)
        self.latitude0 = self.latitude[config.index["stand_by"]]
        self.longitude = np.radians(longitude)
        self.longitude0 = self.longitude[config.index["stand_by"]]
        self.altitude = altitude
        self.pos = {"x": [], "y": [], "z": []}  # Position in observer frame

    def haversine(self, lat, lon, radius=6378*1000):
        # lat, lon = map(np.radians, [lat, lon])
        dlat = lat - self.latitude0
        dlon = lon - self.longitude0

        # Haversine formula
        a = np.sin(dlat / 2) ** 2 + np.cos(self.latitude0) * np.cos(lat) * np.sin(dlon / 2) ** 2
        distance = 2 * radius * np.arcsin(np.sqrt(a))
        if dlat == 0:
            x = distance * np.sign(dlon)
            y = 0
        elif dlon == 0:
            x = 0
            y = distance * np.sign(dlat)
        else:
            x = distance / (dlat ** 2 + dlon ** 2) ** 0.5 * dlat
            y = distance / (dlat ** 2 + dlon ** 2) ** 0.5 * dlon

        # temp = distance / (dlat ** 2 + dlon ** 2) ** 0.5
        # return np.array([dlon, dlat]) * temp

        # x = distance * np.cos(lat) * dlon
        # y = distance * dlat
        # x = radius * dlat
        # y = radius * dlon
        # print(x, y)
        return np.array([x, y])

    def calculate_trajectory(self):
        x = []
        y = []

        for lat, lon in zip(self.latitude, self.longitude):
            dx, dy = self.haversine(lat, lon)
            x.append(dx)
            y.append(dy)

        # x = np.cumsum(x)
        # y = np.cumsum(y)
        z = self.altitude
        print(max(z))

        self.pos["x"] = np.array(x)
        self.pos["y"] = np.array(y)
        self.pos["z"] = np.array(z)

    def plot_trajectory(self, ax, color, label):
        if ax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(self.pos["x"], self.pos["y"], self.pos["z"], label=label, color=color)

            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_zlabel("Z (m)")
            ax.legend()
            plt.show()

        # Plot trajectory
        ax.plot(self.pos["x"], self.pos["y"], self.pos["z"], label=label, color=color)


class SIMPLETrajectory(IMUTrajectory):
    def __init__(self, acc, dt, gravity, gyro, rotator0):
        super().__init__(acc=acc, dt=dt, rotator=None, gravity=gravity)
        self.gyro = gyro
        self.rotator0 = rotator0
        print(self.rotator0)
        self.rotator = self.generate_rotator()

    def generate_rotator(self):
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
        super().calculate_trajectory(launch_index, touchdown_index)

    def plot_trajectory(self, ax, color, label):
        super().plot_trajectory(ax, color, label)