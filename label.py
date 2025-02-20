from quaternion import *

X = 'time'
Y = 'rel'

class id_plot_label():
    def __init__(self, name="", idx=None, x=0, y=0, color_idx=0, dimension="", mean=0, stdev=0):
        self.name = name
        self.idx = idx
        self.x = x
        self.y = y
        self.color_idx = color_idx
        self.dimension = dimension
        self.mean = mean
        self.stdev = stdev


point_label = {
    "FC": {
        'Apogee': id_plot_label(name='Apogee', color_idx=2),
        'Drogue': id_plot_label(name='Drogue', color_idx=3),
        'Main': id_plot_label(name='Main', color_idx=4),
    }
}

variable_label = {
    'temp': id_plot_label(name='Temperature', idx='temp', dimension='C'),
    'time': id_plot_label(name='Time', idx='time', dimension='s'),
    'abs': id_plot_label(name='Altitude', idx='abs_alt', dimension='m'),
    'rel': id_plot_label(name='Altitude', idx='rel_alt', dimension='m'),
    'pres': id_plot_label(name='Pressure', idx='pres', dimension='hPa')
}

acc = {
    'ax': id_plot_label(name='ax', idx='ax', dimension='[m/s**2]'),
    'ay': id_plot_label(name='ay', idx='ay', dimension='[m/s**2]'),
    'az': id_plot_label(name='az', idx='az', dimension='[m/s**2]')
}

gyro = {
    'gx': id_plot_label(name='gx', idx='gx', dimension='[rad/s]'),
    'gy': id_plot_label(name='gy', idx='gy', dimension='[rad/s]'),
    'gz': id_plot_label(name='gz', idx='gz', dimension='[rad/s]')
}

mag = {
    'mx': id_plot_label(name='mx', idx='mx', dimension='[gauss]'),
    'my': id_plot_label(name='my', idx='my', dimension='[gauss]'),
    'mz': id_plot_label(name='mz', idx='mz', dimension='[gauss]')
}

axis = {
    'x': quaternion([0, 1, 0, 0]),
    'y': quaternion([0, 0, 1, 0]),
    'z': quaternion([0, 0, 0, 1])
}