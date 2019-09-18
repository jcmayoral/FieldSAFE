#!/usr/bin/python
import rospy
import cv2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from numpy import fabs,sqrt, floor
from numpy.linalg import norm
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class Lidar2Image:
    def __init__(self, save_image=False):
        rospy.init_node("lidar_to_image")
        self.save_image = save_image
        self.x_resolution = 10
        self.y_resolution = 10

        self.bridge = CvBridge()
        self.counter = 1
        self.pixels_number = 1024

        self.max_value = 255
        self.step = 25

        self.size =  int(self.pixels_number*self.pixels_number)

        rospy.Subscriber("/velodyne_points", PointCloud2, self.topic_cb, queue_size=100)
        rospy.loginfo("Node initialized")
        rospy.spin()

    def save_image_to_file(self,img):
        name = str(self.counter) #self.transformed_image.header.stamp
        #cv2_img.dtype='uint8'
        cv2.imwrite(''+name+'.jpeg', img)
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

        #cvMat = cv2.CreateImage(self.pixels_number, self.pixels_number, cv.CV_32FC3)
        #cvMat = cv2.Mat(2,2, CV_8UC3, Scalar(0,0,255));

        rgb_color=(255, 255, 255)
        cvMat = np.zeros((self.pixels_number, self.pixels_number, 3), np.uint8)
        color = tuple(reversed(rgb_color))
        cvMat[:] = color

        for i in range(self.size-1, -1, -1):
            try:
                point = gen.next()
                x = point[0]
                y = point[1]
                z = point[2]
                cell_x = int(x*self.x_resolution)
                cell_y = int(y*self.y_resolution)
                feature = z#*point[0]*point[1]
            except:
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


            if True:#z > -0.5:
                cvMat[cell_x,cell_y,0] = 0
                cvMat[cell_x,cell_y,1] = 0
                cvMat[cell_x,cell_y,2] = 255

        if self.save_image:
            print "to save"
            self.save_image_to_file(cvMat)

if __name__ == '__main__':
    Lidar2Image(save_image=True)
