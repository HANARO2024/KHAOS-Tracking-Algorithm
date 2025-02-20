import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import os

file_path = r"C:\Users\82108\Desktop\Hanaro\Data Processing\Data\launch_10_6_data  6.csv"
fig_size = (10, 10)

index = {
    "stand_by": 18,
    "launch": 1973
}

data = pd.read_csv(file_path)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


def pressure_to_altitude(pressure):
    # Constants
    sea_level_pressure = 101325  # Pressure at sea level in Pa
    temperature_lapse_rate = 0.0065  # Temperature lapse rate in K/m
    sea_level_temperature = 288.15  # Standard temperature at sea level in K
    gravitational_acceleration = 9.80665  # Gravitational acceleration in m/s^2
    gas_constant_for_air = 287.05  # Specific gas constant for dry air in J/(kg·K)

    # Altitude formula derived from the barometric formula
    altitude = (sea_level_temperature / temperature_lapse_rate) * (
            1 - (pressure / sea_level_pressure) ** (
            gas_constant_for_air * temperature_lapse_rate / gravitational_acceleration)
    )

    return altitude


def plot_position_XYZ():
    ax.scatter(data['px'][index["stand_by"]:],
               data['py'][index["stand_by"]:],
               data['pz'][index["stand_by"]:],
               marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


altitude = [pressure_to_altitude(p) for p in data['pressure']]
ground_alt = min(altitude[index["launch"]:])
altitude = [z - ground_alt for z in altitude]


def plot_position_XYZp():
    ax.scatter(data['px'][index["launch"]:],
               data['py'][index["launch"]:],
               altitude[index["launch"]:],
               marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

plot_position_XYZp()
plt.show()

# Animation
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_xlim(min(data['px'][index["launch"]:]), max(data['px'][index["launch"]:]))
# ax.set_ylim(min(data['py'][index["launch"]:]), max(data['py'][index["launch"]:]))
# ax.set_zlim(min(altitude[index["launch"]:]), max(altitude[index["launch"]:]))
#
# sc = ax.scatter([], [], [], marker='o')
# # Animation update function
# def update(i):
#     # Update scatter plot data with the next point
#     sc._offsets3d = (data['px'][index["stand_by"]:index["stand_by"] + i],
#                      data['py'][index["stand_by"]:index["stand_by"] + i],
#                      altitude[index["stand_by"]:index["stand_by"] + i])
#     return sc,
#
# # Create animation
# ani = FuncAnimation(fig, update, frames=range(1, len(data['px'][index["stand_by"]:])),
#                     interval=50, blit=True)
#
# plt.show()