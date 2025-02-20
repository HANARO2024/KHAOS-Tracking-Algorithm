import numpy as np
import config

class GPSTrajectory:
    def __init__(self, latitude, longitude, altitude):
        self.latitude = np.radians(latitude)
        self.latitude0 = self.latitude[config.index["stand_by"]]
        self.longitude = np.radians(longitude)
        self.longitude0 = self.longitude[config.index["stand_by"]]
        self.altitude = altitude
        self.pos = {"x": [], "y": [], "z": []}  # Position in observer frame

    def haversine(self, lat, lon, radius=6378*1000):
        """Calculate distance using haversine formula."""
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
        print(f'max alt: {max(z)}')

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