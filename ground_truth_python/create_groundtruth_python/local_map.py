#!/usr/bin/python
from __future__ import division
import rospy
import cv2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from numpy import fabs,sqrt, floor
from numpy.linalg import norm
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rosbag
import argparse
import os

help_text = 'This is a script that converts PointCloud2 message to RGB images'

class Lidar2Image:
    def __init__(self, save_image=False, ros = False, filegroup="images"):
        self.save_image = save_image
        #TODO Add in metadata file
        self.pixels_per_meter = 100
        self.filegroup = filegroup

        self.bridge = CvBridge()
        self.counter = 1
        #10 meters
        self.pixels_number = int(10*self.pixels_per_meter)

        #assume symmetric
        self.range= [-3,3]

        self.max_value = 255

        self.size =  int(self.pixels_number*self.pixels_number)
        self.create_folder()

        if ros:
            rospy.init_node("lidar_to_image")
            rospy.Subscriber("/velodyne_points", PointCloud2, self.topic_cb, queue_size=100)
            rospy.loginfo("Node initialized")
            rospy.spin()

    def create_folder(self):
        #path = os.getcwd()
        try:
            os.mkdir(self.filegroup)
        except OSError:
            print ("Creation of the directory %s failed" % self.filegroup)
        else:
            print ("Successfully created the directory %s " % self.filegroup)

    def scalar_to_color(self,x):
        color = self.max_value/(self.range[1]-self.range[0]) * x + self.max_value/2;
        return int(round(color))


    def save_image_to_file(self,img):
        name = os.path.join(self.filegroup , str(self.counter)+'.jpeg') #self.transformed_image.header.stamp
        #cv2_img.dtype='uint8'
        cv2.imwrite(name, img)
        self.counter += 1

    def ros_to_cv(self):
        try:
            # Convert your ROS Image message to OpenCV2
            #self.transformed_image.data = np.array(self.transformed_image.data)
            #self.transformed_image.data = np.round(self.transformed_image.data/self.max_value)
            self.transformed_image.data = np.array(self.transformed_image.data)

            #cv2_img = self.bridge.imgmsg_to_cv2(Image(),"mono8")
            cv2_img = self.bridge.imgmsg_to_cv2(self.transformed_image,"rgb8")
        except CvBridgeError, e:
            print(e)
            return
        return cv2_img


    def topic_cb(self,msg):
        #self.transformed_image.data = [False] * self.size * 3
        gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        self.process_pc(gen)

    def process_pc(self, pc):
        #cvMat = cv2.CreateImage(self.pixels_number, self.pixels_number, cv.CV_32FC3)
        #cvMat = cv2.Mat(2,2, CV_8UC3, Scalar(0,0,255));

        rgb_color=(0, 0, 255)
        cvMat = np.zeros((self.pixels_number, self.pixels_number, 3), np.uint8)
        color = tuple(reversed(rgb_color))
        cvMat[:] = color

        for i in range(self.size-1, -1, -1):
            try:
                point = pc.next()
                x = point[0]
                y = point[1]
                z = point[2]
                cell_x = int(x*self.pixels_per_meter)
                cell_y = int(y*self.pixels_per_meter)
                feature = z#*point[0]*point[1]
            except:
                continue

            if z > self.range[1] or z < self.range[0]:
                continue

            cell_x = int(self.pixels_number/2) + cell_x

            #if cell_x > self.ray_number:
            #    continue

            cell_y = int(self.pixels_number/2) + cell_y
            #if cell_y > self.ray_number:
            #    continue

            #index =  int(fabs(cell_x+ self.transformed_image.height * cell_y))
            #if self.size - index < 0:
            #    continue

            if cell_x > self.pixels_number-1 or cell_y > self.pixels_number-1:
                continue

            if cell_x < 0 or cell_y < 0:
                continue
            probability = 1 /fabs(feature)
            #update = min(fabs(feature), self.max_value)

            #number of values
            cvMat[cell_x,cell_y,0] += 1
            color_val = self.scalar_to_color(z)
            if color_val < 0 or color_val > 255:
                print (color_val)
            #higher value
            cvMat[cell_x,cell_y,1] = max(cvMat[cell_x,cell_y,1],color_val)
            cvMat[cell_x,cell_y,2] = min(cvMat[cell_x,cell_y,2],color_val)

        #TODO Normalize R channel
        #max_overlapping = np.max(cvMat[:,:,0])

        if self.save_image:
            self.save_image_to_file(cvMat)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = help_text)
    parser.add_argument("--bag", "-b", help="set input bagname")
    parser.add_argument("--group", "-g", default="image")
    parser.add_argument("--topic", "-t", default="/velodyne_points")

    args = parser.parse_args()
    bag = rosbag.Bag(args.bag, mode="r")
    lidar2image = Lidar2Image(save_image=True, filegroup=args.group)

    for topic, msg, t in bag.read_messages(topics=args.topic):
        lidar2image.topic_cb(msg)
