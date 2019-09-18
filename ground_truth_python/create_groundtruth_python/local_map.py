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
        self.transformed_image = Image()
        self.transformed_image.header.frame_id = "velodyne"
        self.x_resolution = 10
        self.y_resolution = 10

        self.bridge = CvBridge()
        self.counter = 1

        self.pixels_number = 1024
        self.transformed_image.height = self.pixels_number
        self.transformed_image.width = self.pixels_number
        self.transformed_image.encoding = "8UC1"
        self.transformed_image.step = self.pixels_number/8

        self.max_value = 255
        self.step = 25

        self.size =  int(self.pixels_number*self.pixels_number)
        self.transformed_image.data = [0] * self.size

        self.publisher = rospy.Publisher("/scan_velodyne_hack/image_raw", Image, queue_size=1)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.topic_cb, queue_size=100)
        rospy.loginfo("Node initialized")
        rospy.spin()

    def save_image_to_file(self):
        try:
            # Convert your ROS Image message to OpenCV2
            #self.transformed_image.data = np.array(self.transformed_image.data)
            #self.transformed_image.data = np.round(self.transformed_image.data/self.max_value)
            print(np.any(np.isnan(self.transformed_image.data)))
            print(np.max(self.transformed_image.data))
            self.transformed_image.data = np.array(self.transformed_image.data)

            #cv2_img = self.bridge.imgmsg_to_cv2(Image(),"mono8")
            cv2_img = self.bridge.imgmsg_to_cv2(self.transformed_image,"8UC1")
        except CvBridgeError, e:
            print(e)
            return
        # Save your OpenCV2 image as a jpeg
        print ("sAVING")
        name = str(self.counter) #self.transformed_image.header.stamp
        #cv2_img.dtype='uint8'
        cv2.imwrite(''+name+'.jpeg', cv2_img.astype('uint8')*255)
        self.counter += 1


    def topic_cb(self,msg):
        self.transformed_image.data = [False] * self.size

        gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))

        for i in range(self.size-1, -1, -1):
            try:
                point = gen.next()
                x = point[0]
                y = point[1]
                z = point[2]
                cell_x = round(x*self.x_resolution)
                cell_y = round(y*self.y_resolution)
                feature = z#*point[0]*point[1]
            except:
                continue

            cell_x = (self.pixels_number/2) + cell_x

            #if cell_x > self.ray_number:
            #    continue

            cell_y = (self.pixels_number/2) + cell_y
            #if cell_y > self.ray_number:
            #    continue

            index =  int(fabs(cell_x+ self.transformed_image.height * cell_y))
            if self.size - index < 0:
                continue

            probability = 1 /fabs(feature)
            print probability
            #update = min(fabs(feature), self.max_value)


            if fabs(feature) < 0.5:
                self.transformed_image.data[index] = True
            else:
                self.transformed_image.data[index] = False

        self.transformed_image.header.stamp = rospy.Time.now()
        self.publisher.publish(self.transformed_image)

        if self.save_image:
            print "to save"
            self.save_image_to_file()

if __name__ == '__main__':
    Lidar2Image(save_image=True)
