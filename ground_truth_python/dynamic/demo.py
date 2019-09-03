#! /usr/bin/python
import pandas
import numpy as np
#import cv2
import matplotlib.pyplot as plt
import pygeodesy
from deg2utm import deg2utm
import time
import utm
import to_meters
import to_lanlong
import matplotlib.image as mpimg

# Load dynamic ground truth data

#original line
#table = readtable('dynamic_ground_truth.txt','ReadVariableNames',true,'Delimiter',',');
table =  pandas.read_table('dynamic_ground_truth.txt', delimiter="," )


tables = list()

for i in range(5):
    tables.append(table.loc[table["track_id"] == i])

custom_table =  pandas.read_table('../../ros_recording/dynamic_1_test.txt', delimiter="," )


# Load tractor path in GPS coordinates
#tableTractor = pandas.read_table('tractor_gps_coordinates.csv','ReadVariableNames',true,'Delimiter',',');
tableTractor = pandas.read_table('tractor_gps_coordinates.csv', delimiter="," )

# Load transformation between GPS coordinates and pixel coordinates
#transform = dlmread('../static/utm2PixelsTransformMatrix.csv', ',', 0, 0);
transform = np.loadtxt('../static/utm2PixelsTransformMatrix.csv', delimiter=',', unpack=True);
transform= np.transpose(transform.reshape(3,3))



#plt.figure()

#% Load static map for overlaying positions
#imgMap = cv2.imread('../static/static_ground_truth.png');
imgMap=mpimg.imread('../static/static_ground_truth.png')
#% Visualize
#figure(1)
#TODO imshow(imgMap)
#% Plotting options
#axImg = gca;
#ax1 = plt.axes('position',axImg.Position,'Ydir','reverse');
#set(ax1, 'Color', 'none','TickDir','out')
#xlim([0 size(imgMap,2)]);
#xlim = np.zeros(1+  len(imgMap[1]))
#ylim([0 size(imgMap,1)]);
#ylim = np.zeros(1 +len(imgMap[0]))



#output = set([]) # initialize an empty set
#plt.draw()
#plt.imshow(imgMap)
#ax.imshow(imgMap)

#% Run through all annotated frames
#for frame=min(table.frame):5:max(table.frame)

fig, ax = plt.subplots()
ax.cla()
ax.imshow(imgMap)


for frame in range(min(tables[0].frame),max(tables[0].frame)-1,5):

    #delete(allchild(ax1))
    #hold on;
    #Get corresponding frame information
    #mask = table.frame == frame;


    if frame == 0:
        orig_x = tableTractor["lat"][frame]
        orig_y = tableTractor["lon"][frame]
        origin_array = utm.from_latlon(orig_x, orig_y)

    #dynamic obstacles are consider to be correct
    #all mapped around it.
    timestamp = tables[0]["timestamp"][frame]

    for i in range(5):
        custom_clockIdx = np.argmin(abs(tables[i]["timestamp"][:] - timestamp))

        #id = tables[i]["track_id"][frame];
        x = tables[i]["x"][custom_clockIdx]
        y = tables[i]["y"][custom_clockIdx]
        label = tables[i]["label"][custom_clockIdx]
        state = table["state"][custom_clockIdx]
        lost = table["lost"][custom_clockIdx]
        occluded = table["occluded"][custom_clockIdx]
        #timestamp = unique(table["timestamp"][frame]);
        timestamp = table["timestamp"][custom_clockIdx];
        #%  Show the dynamic obstacles (x'es in figure)
        plotInds = not(occluded) and not(lost) and label == 'human';
        if plotInds:
            A_inv = np.linalg.inv(transform)
            #person_xy = A_inv.dot([[x],[y],[1]])
            #plt.plot(person_xy[0],person_xy[1],'mx');
            ax.plot(x,y,'kx');

    #% Get corresponding tractor location
    clockIdx = np.argmin(abs(tableTractor.clock - timestamp))
    #print (clockIdx, "ODX")
    #closest_tractor_pose = tableTractor.ix[(tableTractor['clock'] - clockIdx).abs().argsort()[:1]]
    #print tableTractor.loc[tableTractor['clock'] > clockIdx]
    #tractor_lat = closest_tractor_pose["lat"]
    #tractor_lon = closest_tractor_pose["lon"]
    tractor_lat = tableTractor["lat"][clockIdx]
    tractor_lon = tableTractor["lon"][clockIdx]
    """
    pygeodesy
    """
    """
    r = pygeodesy.toUtm8(tractor_lat, tractor_lon)
    utm_x = r.northing
    utm_y = r.easting
    tractor_xy = transform.dot([[utm_x],[utm_y],[1]])
    plt.plot(tractor_xy[0], tractor_xy[1],'bx');
    """
    #print(utm_x, utm_y, "UTM")
    #utm_array = np.array([[utm_x],[utm_y],[1]])

    """
    utm
    """
    """
    conversion_utm = utm.from_latlon(tractor_lat, tractor_lon)
    utm_array = np.array([[conversion_utm[0]],[conversion_utm[1]],[1]])
    tractor_xy = transform.dot(utm_array)
    plt.plot(tractor_xy[0], tractor_xy[1],'rx');
    """
    #print(orig_x)
    #utm_array[0] += origin_array[0];
    #utm_array[1] += origin_array[1];
    #print (utm_array)
    """
    custom
    """
    [utm_x,utm_y,utmzone] = deg2utm(tractor_lat, tractor_lon);
    tractor_xy = transform.dot([[utm_x],[utm_y],[1]])
    ax.plot(tractor_xy[1], tractor_xy[0],'gx');


    #pixel_tractor_xy =  transform.dot(utm_array)
    #% Show the tractor path (* in figure)
    #print ("UTM" , utm_array)
    #print ("Result" , pixel_tractor_xy)
    #tractor_x, tractor_y = to_meters.to_meters(tractor_lat, tractor_lon)
    #plt.plot(utm_array[0], utm_array[1],'bx');
    #plt.plot(tractor_xy[0], tractor_xy[1],'bx');

    #plt.plot(tractor_xy[0], tractor_xy[1],'gx');
    #plt.plot(tractor_la,tractor_lon,'bx');
    #time.sleep(1)
    #% Plotting
    #set(ax1, 'Color', 'none')
    #xlim([0 size(imgMap,2)]);
    #xlim = np.zeros(1 + len(imgMap[1]))
    #ylim([0 size(imgMap,1)]);
    #ylim = np.zeros(1 + len(imgMap[0]))

    #if frame==min(table.frame):
    #    plt.legend('Dynamic obstacles (humans)','Tractor')





    """
    #FROM HERE IT IS MY FUCKING PROBLEM0
    """
    custom_clockIdx = np.argmin(abs(custom_table["timestamp"][:] - timestamp))
    if abs(custom_table["timestamp"][custom_clockIdx] - timestamp) > 1.0:
        print("this should be a false positive")
        continue

    selected_timestamp = custom_table["timestamp"][custom_clockIdx]
    while True:
        #coordinates in utm.. supposed to be at least
        x = custom_table["x"][custom_clockIdx]
        y = custom_table["y"][custom_clockIdx]
        custom_clockIdx +=1

        #tl, tl2 =  to_meters.to_meters(tractor_lat, tractor_lon)
        #x += tl
        #y += tl2
        #o_lat, o_lon = to_lanlong.to_lanlong(x,y, [tractor_lat, tractor_lon])
        #print (tractor_lat, tractor_lon, "Tractor")
        #print (o_lat,o_lon, "object")
        #print ("tractor to meters ", to_meters.to_meters(tractor_lat, tractor_lon))
        #lat += orig_x
        #lon += orig_y
        #[c_utm_x,c_utm_y,c_utmzone] = deg2utm(o_lat, o_lon);
        #object_xy = transform.dot([[c_utm_x],[c_utm_y],[1]])
        object_xy = transform.dot([[x],[y],[1]])
        ax.plot(object_xy[1], object_xy[0],'bx');

        if (custom_table["timestamp"][custom_clockIdx] != selected_timestamp):
            break
    #plt.plot(x,y,'bx');
    plt.pause(0.1)
    #plt.clf()
    #plt.cla()
plt.close()

#    end
#plt.show()
#plt.close()
#    drawnow;
#end
