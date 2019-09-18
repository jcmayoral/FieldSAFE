#! /usr/bin/python
import pandas
import numpy as np
import matplotlib.pyplot as plt
from deg2utm import deg2utm
import time
import to_meters
import to_lanlong
import matplotlib.image as mpimg

# Load dynamic ground truth data
#original line
table =  pandas.read_table('dynamic_ground_truth.txt', delimiter="," )

tables = list()

for i in range(5):
    tables.append(table.loc[table["track_id"] == i])

custom_table =  pandas.read_table('../../ros_recording/dynamic_1_test.txt', delimiter="," )


# Load tractor path in GPS coordinates
tableTractor = pandas.read_table('tractor_gps_coordinates.csv', delimiter="," )

# Load transformation between GPS coordinates and pixel coordinates
transform = np.loadtxt('../static/utm2PixelsTransformMatrix.csv', delimiter=',', unpack=True);
transform= np.transpose(transform.reshape(3,3))

imgMap=mpimg.imread('../static/static_ground_truth.png')

fig, ax = plt.subplots()
ax.cla()
ax.imshow(imgMap)


for frame in range(min(tables[0].frame),max(tables[0].frame)-1,5):

    #dynamic obstacles are consider to be correct
    #all mapped around it.
    timestamp = tables[0]["timestamp"][frame]
    person_poses = dict()

    for i in range(5):
        custom_clockIdx = np.argmin(abs(tables[i]["timestamp"][:] - timestamp))
        x = tables[i]["x"][custom_clockIdx]
        y = tables[i]["y"][custom_clockIdx]
        label = tables[i]["label"][custom_clockIdx]
        state = table["state"][custom_clockIdx]
        lost = table["lost"][custom_clockIdx]
        occluded = table["occluded"][custom_clockIdx]
        timestamp = table["timestamp"][custom_clockIdx];
        #%  Show the dynamic obstacles (x'es in figure)
        plotInds = not(occluded) and not(lost) and label == 'human';
        if True:#plotInds:
            ax.plot(x,y,'kx');
            person_poses[i] = [x,y]

    #% Get corresponding tractor location
    clockIdx = np.argmin(abs(tableTractor.clock - timestamp))
    tractor_lat = tableTractor["lat"][clockIdx]
    tractor_lon = tableTractor["lon"][clockIdx]

    [utm_x,utm_y,utmzone] = deg2utm(tractor_lat, tractor_lon);
    tractor_xy = transform.dot([[utm_x],[utm_y],[1]])
    ax.plot(tractor_xy[1], tractor_xy[0],'gx');


    #Start matching of the approach
    custom_clockIdx = np.argmin(abs(custom_table["timestamp"][:] - timestamp))
    if abs(custom_table["timestamp"][custom_clockIdx] - timestamp) > 1.0:
        print("this should be a false detection... too delayed")
        #continue

    selected_timestamp = custom_table["timestamp"][custom_clockIdx]


    distance = np.zeros(5)*-100
    #This because a single message can detect several
    while True:
        #coordinates in utm.. supposed to be at least
        x = custom_table["x"][custom_clockIdx]
        y = custom_table["y"][custom_clockIdx]
        custom_clockIdx +=1
        object_xy = transform.dot([[x],[y],[1]])
        ax.plot(object_xy[1], object_xy[0],'bx');

        for key, values in person_poses.items():
            dx = np.power(person_poses[key][0] -object_xy[0],2)
            dy = np.power(person_poses[key][1] -object_xy[1],2)
            distance[key] = np.sqrt(dx+dy)

        closest_person_id = np.argmin(distance)
        print ("Closest person " , closest_person_id , "with distance " , distance[closest_person_id])

        if (custom_table["timestamp"][custom_clockIdx] != selected_timestamp):
            break
    plt.pause(0.1)
plt.close()
