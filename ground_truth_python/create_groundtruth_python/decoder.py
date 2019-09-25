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
from PIL import Image
import random

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
        if self.index == -1:
            img_name = os.path.join(self.folder , str(self.counter)+self.extension) #self.transformed_image.header.stamp
        else:
            img_name = os.path.join(self.folder , str(self.index)+self.extension) #self.transformed_image.header.stamp
            self.task_done = True

        try:
            #im_raw = Image.open(img_name)
            #im = np.asarray(im_raw)
            im = cv2.imread(img_name, cv2.IMREAD_COLOR)
            #im = cv2.imread(img_name)
            #im = cv2.cvtColor(im, cv2.COLOR_RGB2HSV).astype(np.float64)


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

        z_scaler =  np.fabs(self.range[1]-self.range[0])/255
        print(z_scaler)


        for i in range(height):
            for j in range(width):
                #r counter
                #g mean
                #b variance
                r,g,b = img[i,j,:]
                #if g >60 or b > 100:
                #    print (r,g,b)
                #    pbar.update()
                #    continue
                #print (r,g,b)
                #NOT WORKING TODO
                #if g == 0 or b == 0:
                #    pbar.update()
                #    continue
                #max_height = np.log(float(g)/255) - np.log(1-float(g)/255)
                #min_height = np.log(float(b)/255) - np.log(1-float(b)/255)


                #variance_height -= mean_height

                x = ((float(i) - x_offset)/self.pixels_per_meter)
                y = ((float(j) - y_offset)/self.pixels_per_meter)

                if (r == 0):
                    pbar.update()
                    continue

                #if points_offset == 0:
                    #self.points.append([x,y,min_height])
                    #pbar.update()
                    #continue

                #if np.fabs(max_height - min_height) > (self.range[1] - self.range[0]):
                #    pbar.update()
                #    self.points.append([x,y,0)
                ##continue

                #if g == 0 and b == 0:
                #    print(mean_height, std_height, "A")
                #    pbar.update()
                #    continue
                mean_height = ((float(b)-127)/255) * (self.range[1]-self.range[0])
                std_height = ((float(g))/255) * (self.range[1]-self.range[0])
                print (mean_height, std_height)
                #TODO calculate number of samples according something
                for _ in range(r/2):
                    z = random.uniform(mean_height - std_height, mean_height + std_height)
                    self.points.append([x,y,z])
                #self.points.append([x,y,mean_height])
                #if b == g:
                #    print("NO height")
                #    pbar.update()
                #    continue

                #for w in np.arange(min_height, max_height, z_scaler):
                #    self.points.append([x,y,w])

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
