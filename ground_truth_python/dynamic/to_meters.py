import numpy as np
import math

def to_meters(lat, lon):
    #Position, decimal degrees
    #Earth’s radius, sphere
    R=6378137
    #offsets in meters
    dn = 0
    de = 0
    #Coordinate offsets in radians
    dLat = dn/R
    dLon = de/(R*math.cos(math.pi*lat/180))
    #OffsetPosition, decimal degrees
    latO = lat + dLat * 180/np.pi
    lonO = lon + dLon * 180/np.pi
    return latO, lonO
