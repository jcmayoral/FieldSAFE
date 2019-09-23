#!/usr/bin/python
import cv2
import argparse
import os
from sensor_msgs.msg import PointCloud2
import rospy
import numpy as np
import pptk

help_text = 'This is a script that converts RGB images to PointCloud2 messages'


class ImageToPc():
    def __init__(self, extension, folder, topic=None, index = -1):
        rospy.init_node("pointcloud_decoder")
        self.index = index
        self.counter = 1
        self.folder = folder
        self.extension = extension
        self.task_done = False
        self.x_resolution = 0.5
        self.y_resolution = 0.5
        self.range= [-3,3]

    def get_next_image(self):
        try:
            if self.index == -1:
                img_name = os.path.join(self.folder , str(self.counter)+self.extension) #self.transformed_image.header.stamp
                im = cv2.imread(img_name, cv2.IMREAD_COLOR)
            else:
                img_name = os.path.join(self.folder , str(self.index)+self.extension) #self.transformed_image.header.stamp
                im = cv2.imread(img_name, cv2.IMREAD_COLOR)
                self.task_done = True

            if im is None:
                self.task_done = True
            self.counter = self.counter + 1
            return im
        except cv2.error as e:
            print ("ERROR")
            self.task_done = True
            return None

    def compute_pc(self, img):
        height, width, channels = img.shape
        x_offset = height/2
        y_offset = width/2

        points = list()

        for i in range(height):
            for j in range(width):
                max_height = ((float(img[i,j,1])-127)/255) * (self.range[1]-self.range[0])
                min_height = ((float(img[i,j,2])-127)/255) * (self.range[1]-self.range[0])
                number_sample = img[i,j,0]
                x = (i - x_offset)/self.x_resolution
                y = (j - y_offset)/self.y_resolution
                if number_sample == 0:
                    continue

                if (number_sample == 0):
                    continue
                points_offset = np.fabs(max_height-min_height)/number_sample

                if points_offset == 0:
                    points.append([x,y,min_height])
                    continue


                for z in np.arange(min_height, max_height, points_offset):
                    print (z, min_height, max_height)
                    points.append([x,y,z])

        viewer = pptk.viewer(points)
        raw_input("Press Enter to continue...")
        viewer.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = help_text)
    parser.add_argument("--folder", "-b", help="set input bagname", default=False)
    parser.add_argument("--extension", "-e", help="set image extension", default=".jpeg")
    parser.add_argument("--topic", "-t", default="/velodyne_points")
    parser.add_argument("--index", "-i", default=-1)


    args = parser.parse_args()
    image2PC = ImageToPc(extension=args.extension, folder=args.folder, topic = args.topic, index = args.index)

    while True:
        current_image = image2PC.get_next_image()
        if current_image is None:
            break

        image2PC.compute_pc(current_image)
        if image2PC.task_done:
            break


    print("Finished")
