#!/usr/bin/python
import cv2
import argparse
import os
from sensor_msgs.msg import PointCloud2
import numpy as np
import pptk
import time
import copy
import pyprind
import ast

help_text = 'This is a script that converts RGB images to PointCloud2 messages'


class ImageToPc():
    def __init__(self, extension, folder, topic=None, index = -1):
        #rospy.init_node("pointcloud_decoder")
        self.index = index
        self.counter = 1
        self.folder = folder
        self.extension = extension
        self.task_done = False
        #100 cm per meter

        #self.points = list()
        #self.viewer = pptk.viewer(self.points)
        self.load_params()

    def load_params(self):
        f = open("params.txt","r")
        f.seek(0)
        params = ast.literal_eval(f.read())
        f.close()
        print(params)
        self.range = [params["range_min"], params["range_max"]]
        self.meters = params["meters"]
        self.pixels_per_meter = params["pix_per_meter"]


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
        print("computing %d points ", height * width)
        pbar = pyprind.ProgBar(height*width)
        self.points = list()

        z_scaler = np.fabs(self.range[1]-self.range[0])/255
        print(z_scaler)


        for i in range(height):
            for j in range(width):
                r,g,b = img[i,j,:]

                #NOT WORKING TODO
                #max_height = np.log(float(g)/255) - np.log(1-float(g)/255)
                #min_height = np.log(float(b)/255) - np.log(1-float(b)/255)

                max_height = ((float(b)-127)/255) * (self.range[1]-self.range[0])
                min_height = ((float(g)-127)/255) * (self.range[1]-self.range[0])

                x = ((float(i) - x_offset)/self.pixels_per_meter)
                y = ((float(j) - y_offset)/self.pixels_per_meter)

                if (r < 4):
                    pbar.update()
                    continue

                #if points_offset == 0:
                    #self.points.append([x,y,min_height])
                    #pbar.update()
                    #continue

                #if np.fabs(max_height - min_height) > (self.range[1] - self.range[0]):
                #    pbar.update()
                #    self.points.append([x,y,0)
                #    continue
                #print np.fabs(max_height - min_height)

                z = min_height
                for _ in range(1+r):
                    z = copy.copy(z) + z_scaler
                    self.points.append([x,y,z])

                #print(max_height, min_height)

                #for z in np.arange(min_height, max_height, z_scaler):
                #    self.points.append([x,y,z])

                pbar.update()

                #
        print("number of points %d "% len(self.points))
        #self.viewer.close()
        self.viewer = pptk.viewer(self.points)

        if self.index > -1:
            raw_input("Press Enter to continue...")
        else:
            time.sleep(1)

        if self.task_done:
            self.viewer.close()


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
