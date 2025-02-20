from matplotlib import rcParams
import numpy as np
from data import data_format

# figure
# fig_size = (10, 7)
# line_width = 2.5
# scatter_marker = 'o'
# s = 120
# dpi_default = 600

# font
# text_font_size = 20
# label_font_size = 20
# title_font_size = 25
# title_font_weight = 'bold'
# legend_font_size = 15
# textbox_linespacing = 1.8

index = {
    "armed": 0,
    "launch": 0,
    "apogee": 0,
    "drogue": 0,
    "main": 0,
    "touchdown": 0
}

a_threshold = 0.05

ground_gravity = 0
ground_magnetic_field = None

std_err = None