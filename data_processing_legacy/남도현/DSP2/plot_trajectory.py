import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R


class RocketTrajectory:
    def __init__(self, rotator, acc, dt):
        self.rotator = rotator  # List of quaternions
        self.acc = acc  # Dictionary of accelerations in body frame
        self.dt = dt  # Time step
        self.pos = {"x": [], "y": [], "z": []}  # Position in observer frame

    def acceleration_in_observer_frame(self, index, velocity):
        # Transform acceleration from body frame to observer frame
        quaternion = self.rotator[index]
        rotation_matrix = R.from_quat(quaternion).as_matrix()
        acc_body = np.array([self.acc['x'][index], self.acc['y'][index], self.acc['z'][index]])
        return rotation_matrix @ acc_body

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

    def plot_trajectory(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot trajectory
        ax.plot(self.pos["x"], self.pos["y"], self.pos["z"], label="Trajectory")

        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_zlabel("Z Position")
        ax.legend()
        plt.show()

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

        anim = FuncAnimation(fig, update, frames=len(self.pos["x"]), interval=50, blit=True)
        plt.show()

# Example Usage:
# rocket = RocketTrajectory(rotator, acc, dt)
# rocket.calculate_trajectory(config.index["launch"], config.index["touchdown"])
# rocket.plot_trajectory()
# rocket.animate_trajectory()
