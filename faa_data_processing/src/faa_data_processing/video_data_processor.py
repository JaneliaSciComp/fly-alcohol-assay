#!/usr/bin/python
from __future__ import print_function
import roslib; roslib.load_manifest('faa_data_processing')
import rospy
import rosbag

import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import argparse
import os
import subprocess
from faa_utilities import FindData
from faa_utilities import FileTools

file_tools = FileTools()


class VideoDataProcessor(object):
    def __init__(self,overwrite=False):
        self.bridge = CvBridge()
        self.overwrite = overwrite
        self.data_image_topic = "/camera/data_image"
        self.fd = FindData(overwrite=overwrite)
        quality_low = "-preset veryfast -crf 28"
        quality_med = "-preset medium -crf 23"
        quality_high = "-preset veryslow -crf 16"
        self.quality = quality_high

    def find_and_process_data(self,directory):
        paths = self.find_data(directory)
        self.process_data(paths)

    def find_data(self,directory):
        return self.fd.find_video_data(directory)

    def process_data(self,bag_paths):
        for bag_path in bag_paths:
            images_paths = file_tools.create_images_paths_from_bag_path(bag_path,self.overwrite)
            if images_paths is None:
                continue
            print("Processing data in {0}".format(bag_path))
            image_count = 0
            bag = rosbag.Bag(bag_path)
            for topic, msg, t in bag.read_messages(topics=[self.data_image_topic]):
                self.process_data_image(msg,images_paths,image_count)
                image_count += 1
            images_paths_bash,video_paths = file_tools.get_video_paths(bag_path,images_paths)
            self.create_videos(images_paths_bash,video_paths)
            bag.close()

    def process_data_image(self,data,paths,count):
        try:
          image_cv = self.bridge.imgmsg_to_cv(data, "bgr8")
          image_np = numpy.asarray(image_cv)
          orig_path = paths['original'].format(count=count)
          cv2.imwrite(orig_path,image_np[:,:,0],[cv2.IMWRITE_PNG_COMPRESSION,0])
          bg_path = paths['background'].format(count=count)
          cv2.imwrite(bg_path,image_np[:,:,1],[cv2.IMWRITE_PNG_COMPRESSION,0])
          fg_path = paths['foreground'].format(count=count)
          cv2.imwrite(fg_path,image_np[:,:,2],[cv2.IMWRITE_PNG_COMPRESSION,0])
        except CvBridgeError, e:
          print(e)

    def create_videos(self,images_paths_bash,video_paths):
        cmd_str = "avconv -f image2 -i {input} -vcodec libx264 {quality} -threads 8 {output}"
        for path_name in images_paths_bash:
            images_path = images_paths_bash[path_name]
            video_path = video_paths[path_name]
            first_pass = cmd_str.format(input=images_path,
                                        quality=self.quality,
                                        output=video_path)
            # print(first_pass)
            subprocess.check_call(first_pass,shell=True)
            # print(video_path)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", help="Directory where bag files are located")
    parser.add_argument("-o", "--overwrite", dest='overwrite', default=False, action="store_true", help="Reanalyze all experiments and overwrite previous data (default is to only analyze new data)")
    args = parser.parse_args()

    vdp = VideoDataProcessor(args.overwrite)
    vdp.find_and_process_data(args.directory)
