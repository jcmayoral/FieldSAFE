#!/usr/bin/python
import numpy as np
import math


def to_lanlong(dx,dy, previous):
    r_earth = 6378137
    latitude  = previous[0] + (dy / r_earth) * (180 / np.pi);
    longitude = previous[1] + (dx / r_earth) * (180 / np.pi) / np.cos(latitude * np.pi/180)
    return latitude, longitude
