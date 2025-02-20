import numpy as np
from scipy.spatial.transform import Rotation

def matrix_right(q):
    q0, q1, q2, q3 = q
    Q_right = np.array([
        [q0, -q1, -q2, -q3],
        [q1, q0, q3, -q2],
        [q2, -q3, q0, q1],
        [q3, q2, -q1, q0]
    ])
    return Q_right

class IMUTrajectory:
    def __init__(self, rotator, direction, acc, dt, gravity):
        self.rotator = rotator  # List of quaternions
        self.rotator_direction = direction
        self.acc = acc  # Dictionary of accelerations in body frame
        self.dt = dt  # Time step
        self.pos = {"x": [], "y": [], "z": []}  # Position in observer frame
        self.gravity = -gravity

    def acceleration_in_observer_frame(self, index, velocity, kalman=True):
        # Transform acceleration from body frame to observer frame
        if kalman: quaternion = matrix_right(self.rotator[index]) @ self.rotator_direction
        else: quaternion = self.rotator[index]
        # rotation_matrix = R.from_quat(quaternion).as_matrix()
        acc_body = np.array([self.acc['x'][index], self.acc['y'][index], self.acc['z'][index]])
        q0, *q = quaternion
        q = np.array(q)
        # return rotation_matrix @ acc_body + [0, 0, self.gravity]
        # quaternion rotation
        return (q0**2 - np.linalg.norm(q)**2) * acc_body + 2 * np.dot(q, acc_body) * q + 2 * q0 * np.cross(q, acc_body) + [0, 0, self.gravity]


    def rk4_step(self, position, velocity, index, kalman=True):
        dt = self.dt
        # k1
        a1 = self.acceleration_in_observer_frame(index, velocity, kalman)
        v1 = velocity
        p1 = position

        # k2
        a2 = self.acceleration_in_observer_frame(index, velocity + 0.5 * a1 * dt, kalman)
        v2 = velocity + 0.5 * a1 * dt
        p2 = position + 0.5 * v1 * dt

        # k3
        a3 = self.acceleration_in_observer_frame(index, velocity + 0.5 * a2 * dt, kalman)
        v3 = velocity + 0.5 * a2 * dt
        p3 = position + 0.5 * v2 * dt

        # k4
        a4 = self.acceleration_in_observer_frame(index, velocity + a3 * dt, kalman)
        v4 = velocity + a3 * dt
        p4 = position + v3 * dt

        # Combine
        velocity_next = velocity + (a1 + 2 * a2 + 2 * a3 + a4) * dt / 6
        position_next = position + (v1 + 2 * v2 + 2 * v3 + v4) * dt / 6
        return position_next, velocity_next

    def calculate_trajectory(self, launch_index, touchdown_index, kalman=True):
        velocity = np.array([0, 0, 0])  # Initial velocity in observer frame
        position = np.array([0, 0, 0])  # Initial position in observer frame

        # Iterate over the specified time steps
        for i in range(launch_index, touchdown_index):
            position, velocity = self.rk4_step(position, velocity, i, kalman)
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