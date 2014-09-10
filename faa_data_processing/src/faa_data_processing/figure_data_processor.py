#!/usr/bin/python
from __future__ import print_function,division
import roslib; roslib.load_manifest('faa_data_processing')
import rospy
import rosbag

import cv2
import cv
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import argparse
import os
import subprocess
from faa_utilities import FindData
from faa_utilities import FileTools

FILE_TOOLS = FileTools()
DISPLAY_IMAGES = True


class FigureDataProcessor(object):
    def __init__(self,overwrite=False):
        self.bridge = CvBridge()
        self.overwrite = overwrite
        self.data_image_topic = "/camera/data_image"
        self.fd = FindData(overwrite=overwrite)
        self.figure = None
        self.background = None
        if DISPLAY_IMAGES:
            cv2.namedWindow("Image FG")
            cv2.namedWindow("Image Thresh")
            cv2.namedWindow("Image Morphed")

    def find_and_process_data(self,directory):
        paths = self.find_data(directory)
        self.process_data(paths)

    def find_data(self,directory):
        return self.fd.find_figure_data(directory)

    def process_data(self,bag_paths):
        for bag_path in bag_paths:
            self.params = FILE_TOOLS.get_params_from_bag_path(bag_path)
            self.threshold = self.params['faa_image_processing']['threshold']
            self.morph_kernel_size = self.params['faa_image_processing']['morph_kernel_size']
            self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(self.morph_kernel_size,self.morph_kernel_size))
            figure_path = FILE_TOOLS.create_figure_path_from_bag_path(bag_path)
            self.raw_data = FILE_TOOLS.get_raw_data_from_bag_path(bag_path)
            if figure_path is None or self.raw_data is None:
                continue
            print("Processing data in {0}".format(bag_path))
            self.walkway_time_rel_max = numpy.max(self.raw_data['walkway']['time_rel'])
            self.figure = None
            image_count = 0
            bag = rosbag.Bag(bag_path)
            for topic, msg, t in bag.read_messages(topics=[self.data_image_topic]):
                self.process_data_image(msg,image_count)
                image_count += 1
            # print(image_count)
            cv2.imwrite(figure_path,self.figure,[cv2.IMWRITE_PNG_COMPRESSION,0])
            bag.close()

    def display_image(self,image,window):
        if DISPLAY_IMAGES:
            cv2.imshow(window, image)
            cv2.waitKey(10)

    def process_data_image(self,image,count):
        try:
            image_cv = self.bridge.imgmsg_to_cv(image, "bgr8")
            image_np = numpy.asarray(image_cv)
            image_fg = image_np[:,:,2]
            self.display_image(image_fg,"Image FG")
            retval,image_thresh = cv2.threshold(image_fg,self.threshold,255,cv2.THRESH_BINARY)
            self.display_image(image_thresh,"Image Thresh")
            image_morphed = cv2.morphologyEx(image_thresh,cv2.MORPH_OPEN,self.morph_kernel)
            self.display_image(image_morphed,"Image Morphed")
            contours,hierarcy = cv2.findContours(image_morphed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            if count > 0:
                if count%5 == 0:
                    walkway_data = self.raw_data['walkway']
                    data_secs_match = walkway_data[walkway_data['time_secs']==image.header.stamp.secs]
                    data_time_match = data_secs_match[data_secs_match['time_nsecs']==image.header.stamp.nsecs]
                    if len(data_time_match) > 0:
                        time_rel = data_time_match[0]['time_rel']
                        blue = int(time_rel*255/self.walkway_time_rel_max)
                        green = int((self.walkway_time_rel_max - time_rel)*255/self.walkway_time_rel_max)
                        red = 0
                        color = (blue,green,red)
                        cv2.drawContours(self.figure,contours,-1,color,-1)

                    # count_max = 200
                    # if count <= count_max:
                    #     blue = int(count*255/count_max)
                    #     green = int((count_max-count)*255/count_max)
                    #     red = 0
                    # else:
                    #     green = 0
                    #     count_max_new = 1121 - count_max
                    #     count = count - count_max
                    #     red = int(count*255/count_max_new)
                    #     blue = int((count_max_new-count)*255/count_max_new)
                    # color = (blue,green,red)
                    # cv2.drawContours(self.figure,contours,-1,color,-1)
            else:
                self.figure = cv2.cvtColor(image_np[:,:,1],cv2.COLOR_GRAY2RGB)
        except CvBridgeError, e:
            print(e)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", help="Directory where bag files are located")
    parser.add_argument("-o", "--overwrite", dest='overwrite', default=False, action="store_true", help="Reanalyze all experiments and overwrite previous data (default is to only analyze new data)")
    args = parser.parse_args()

    fdp = FigureDataProcessor(args.overwrite)
    fdp.find_and_process_data(args.directory)
