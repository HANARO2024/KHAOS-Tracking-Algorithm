import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
from numpy.linalg import inv
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

file_path = r"C:\Users\82108\Desktop\Hanaro\Data Processing\Data\identity3-B_lora_data.csv"
file = pd.read_csv(file_path)


def get_gyro(i):
    p = file['xAxisAngVal'][i]*np.pi/180
    q = file['yAxisAngVal'][i]*np.pi/180
    r = file['zAxisAngVal'][i]*np.pi/180
    return np.array([p, q, r])

G = 9.799
j = 1e-7
print(j)
def get_accel(i):
    ax = file['xAxisAcc'][i]
    ay = file['yAxisAcc'][i]
    az = file['zAxisAcc'][i]
    return np.array([ax, ay, az])

def get_alt(i):
    return 44307.69396*(1.0 - pow(file['pressure'][i]/101325, 0.190284))

def get_time(i):
    return file['time'][i]/1000

def haversine(lat, lon, radius=6371):
    lat, lon = map(np.radians, [lat*j, lon*j])
    
    # Differences in coordinates
    dlat = lat - np.radians(file['gpsLatitude'][0]*j)
    dlon = lon - np.radians(file['gpsLongitude'][0]*j)
    print(dlat, dlon)
    
    # Haversine formula
    a = np.sin(dlat / 2)**2 + np.cos(np.radians(file['gpsLatitude'][0]*j)) * np.cos(lat) * np.sin(dlon / 2)**2
    c = 2 * np.arcsin(np.sqrt(a))
    tmp = radius * c * 1000 / (dlat**2 + dlon**2)**0.5
    
    return np.array([dlon, dlat])*tmp

def get_gps(i):
    return haversine(file['gpsLatitude'][i], file['gpsLongitude'][i])

def regression_angle(x, y):
    x = np.array(x)
    y = np.array(y)
    x = np.nan_to_num(x, nan=0)
    y = np.nan_to_num(y, nan=0)
    x_mean = np.mean(x)
    y_mean = np.mean(y)

    slope = np.sum((x - x_mean) * (y - y_mean)) / np.sum((x - x_mean) ** 2)
    angle = math.atan(slope)

    return angle

class Quaternion:
    def __init__(self, *args):
        if len(args) == 4:
            self.q0 = args[0]
            self.q1 = args[1]
            self.q2 = args[2]
            self.q3 = args[3]
        elif len(args) == 2:
            s, v = args[0], args[1]
            self.q0 = s
            self.q1, self.q2, self.q3 = v[0], v[1], v[2]
        else:
            raise ValueError()
    
    def __mul__(self, other):
        if type(other) == Quaternion:
            return Quaternion(
                self.q0*other.q0-(self.q1*other.q1+self.q2*other.q2+self.q3*other.q3),
                self.q0*other.q1+self.q1*other.q0+self.q2*other.q3-self.q3*other.q2,
                self.q0*other.q2-self.q1*other.q3+self.q2*other.q0+self.q3*other.q1,
                self.q0*other.q3+self.q1*other.q2-self.q2*other.q1+self.q3*other.q0
            )
        else:
            return Quaternion(self.q0*other, self.q1*other, self.q2*other, self.q3*other)
    
    def conj(self):
        return Quaternion(self.q0, -self.q1, -self.q2, -self.q3)
    
    def norm(self):
        return np.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)
    
    def normalized(self):
        return self*(1/self.norm())
    
    def vector(self):
        return np.array([self.q1, self.q2, self.q3])
    
    def scalar(self):
        return self.q0
    
    @classmethod
    def rot(cls, *args):
        if len(args) == 2:
            theta, axis = args[0], args[1]
            return Quaternion(
                np.cos(theta/2), np.sin(theta/2) * axis/np.linalg.norm(axis)
            )
        elif len(args) == 1:
            w = args[0]
            theta = np.linalg.norm(w)
            return cls.rot(theta, w)
    
# Input parameters.
n_samples = len(file)

s = np.array([0, 0, 0])
v = np.array([0, 0, 0])
g = np.array([0, 0, -G])


# axis: ground -> rocket
if get_accel(0)[2] < 0:
    u = -1.0
    axis = Quaternion.rot(np.pi, np.array([1, 0, 0]))
else:
    u = 1.0
    axis = Quaternion.rot(0, np.array([1, 0, 0]))
    
loc_save = []
x_save, y_save, z_save = [], [], []
alt_save = []
gpsx_save = []
gpsy_save = []

loc_save.append(s)
x_save.append((axis.conj()*Quaternion(0, np.array([1, 0, 0]))*axis).vector())   
y_save.append((axis.conj()*Quaternion(0, np.array([0, 1, 0]))*axis).vector())
z_save.append((axis.conj()*Quaternion(0, np.array([0, 0, u]))*axis).vector())
alt_save.append(get_alt(0))
gpx, gpy = get_gps(0)
gpsx_save.append(gpx)
gpsy_save.append(gpy)

top = 0


for i in range(n_samples-1):
    accel = get_accel(i)
    gyro = get_gyro(i)

    inv_axis = axis.conj()
    ground_a = inv_axis * Quaternion(0, accel) * inv_axis.conj()
    
    dt = get_time(i+1) - get_time(i)

    s = s + v*dt
    if s[2] < -100: 
        s -= v*dt
        top = i
    v = v + (ground_a.vector() + g)*dt

    axis = (Quaternion.rot(gyro*dt)*axis).normalized()

    loc_save.append(s)
    x_save.append((axis.conj()*Quaternion(0, np.array([(1+u)/2, (1-u)/2, 0]))*axis).vector())
    y_save.append((axis.conj()*Quaternion(0, np.array([(1-u)/2, (1+u)/2, 0]))*axis).vector())
    z_save.append((axis.conj()*Quaternion(0, np.array([0, 0, u]))*axis).vector())
    alt_save.append(get_alt(i))
    gpsx, gpsy = get_gps(i)
    gpsx_save.append(gpsx)
    gpsy_save.append(gpsy)

loc_save = np.array(loc_save)
x_save = np.array(x_save)
y_save = np.array(y_save)
z_save = np.array(z_save)
alt_save = np.array(alt_save)
gpsx_save = np.array(gpsx_save)
gpsy_save = np.array(gpsy_save)

linreg_gps = regression_angle(gpsx_save[:top], gpsy_save[:top])
linreg_imu = regression_angle(loc_save[:,0], loc_save[:,1])
theta = linreg_imu - linreg_gps

print(linreg_gps, linreg_imu)

gpsx_tmp = gpsx_save*np.cos(theta) - gpsy_save*np.sin(theta)
gpsy_save = gpsy_save*np.cos(theta) + gpsx_save*np.cos(theta)
gpsx_save = gpsx_tmp

fig, axes = plt.subplots()

axes.plot(file['time']/1000, loc_save[:,2] + get_alt(0), 'b', label='imu', markersize=0.2)
axes.plot(file['time']/1000, alt_save, 'r', label='alt', markersize=0.2)
axes.legend(loc='lower right')
axes.set_title('H')
axes.set_xlabel('Time [sec]')
axes.set_ylabel('H')

# plt.savefig('C:\\Users\\lukek\\문서\\GitHub\\HARANG2\\data processing\\김형찬\\identity3-B_lora_data_alt_plot.png')
plt.show()

dim = plt.figure().add_subplot(projection = '3d')
dim.plot(loc_save[:,0], loc_save[:,1], loc_save[:,2], label = 'trajectory')
dim.legend()

fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))

xquiver = ax.quiver(0, 0, 0, 0, 0, 0)
yquiver = ax.quiver(0, 0, 0, 0, 0, 0)
zquiver = ax.quiver(0, 0, 0, 0, 0, 0)

ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)
ax.set_zlim(0, 200)

def update(i):
    global xquiver, yquiver, zquiver
    xquiver.remove()
    yquiver.remove()
    zquiver.remove()
    # loc = loc_save[i]
    loc = np.array([gpsx_save[i], gpsy_save[i], alt_save[i]])
    xquiver = ax.quiver(*(list(loc)+list(x_save[i]*100)), color = 'blue')
    yquiver = ax.quiver(*(list(loc)+list(y_save[i]*100)), color = 'blue')
    zquiver = ax.quiver(*(list(loc)+list(z_save[i]*100)), color = 'red')

print(gpsx_save, gpsy_save)

ax.plot(loc_save[:,0], loc_save[:,1], loc_save[:,2], 'b', label = 'traject1ry')
ax.plot(gpsx_save, gpsy_save, alt_save, 'g', label = 'traject2ry')
ax.legend()
ani = FuncAnimation(fig, update, frames=range(n_samples), interval=(file['time'][n_samples-1] - file['time'][0])/n_samples)
plt.show()