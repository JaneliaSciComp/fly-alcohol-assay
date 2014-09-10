#!/usr/bin/env python
import roslib
roslib.load_manifest('faa_data_save')
import rospy
import threading
import cv2
import numpy
import os
# import subprocess

from faa_data_save.msg import SaveData
from faa_utilities import FileTools

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


file_tools = FileTools()

class VideoSaver(object):

    def __init__(self):
        self.lock = threading.Lock()
        self.saving = False
        self.bridge = CvBridge()
        self.save_sub = rospy.Subscriber('/faa_data_save/save_data', SaveData, self.save_data_callback)
        self.image_sub = rospy.Subscriber("/camera/image_trimmed",Image,self.trimmed_image_callback)
        self.image_count = 0
        # quality_low = "-preset veryfast -crf 28"
        # quality_med = "-preset medium -crf 23"
        # quality_high = "-preset veryslow -crf 16"
        # self.quality = quality_high
        self.video_setup = False
        self.video_writer_opened = False

    def save_data_callback(self,data):
        if data.saving:
            self.image_count = 0
            # self.setup_image_saving()
            self.setup_video_saving()
            self.saving = data.saving
        else:
            # self.save_video_file()
            self.saving = data.saving
            self.finish_video_saving()

    def trimmed_image_callback(self,data):
        if not self.video_writer_opened and self.video_setup:
            self.lock.acquire()
            try:
              image_cv = self.bridge.imgmsg_to_cv(data, "mono8")
              image_np = numpy.asarray(image_cv)
              height,width = image_np.shape
              self.video_writer = cv2.VideoWriter(self.video_path,cv2.cv.CV_FOURCC('X','V','I','D'),self.fps,(width,height))
              self.video_writer_opened = True
            except CvBridgeError, e:
              print e
            self.lock.release()

        if self.saving and self.video_writer_opened:
            self.lock.acquire()
            try:
              image_cv = self.bridge.imgmsg_to_cv(data, "mono8")
              image_np = numpy.asarray(image_cv)
              # path = self.image_path.format(count=self.image_count)
              # cv2.imwrite(path,image_np,[cv2.IMWRITE_PNG_COMPRESSION,0])
              self.video_writer.write(image_np)
              self.image_count += 1
            except CvBridgeError, e:
              print e
            self.lock.release()

    # def setup_image_saving(self):
    #     images_path = rospy.get_param('/faa_experiment/images_path')
    #     image_name_base = "image_{count:06d}.png"
    #     self.image_path = os.path.join(images_path,image_name_base)

    def setup_video_saving(self):
        trial_path = rospy.get_param('/faa_experiment/trial_path')
        experiment_name = rospy.get_param('/faa_experiment/experiment_name')
        trial_name = rospy.get_param('/faa_experiment/trial_name')
        time_str = rospy.get_param('/faa_experiment/trial_start_time')
        file_name = "Video-" + experiment_name + '-' + trial_name + '-' + time_str + '.avi'
        self.video_path = os.path.join(trial_path,file_name)
        self.fps = rospy.get_param('/camera1394_node/frame_rate')
        self.video_setup = True

    def finish_video_saving(self):
        self.video_setup = False
        self.video_writer_opened = False
        self.video_writer.release()

    # def save_video_file(self):
    #     self.lock.acquire()
    #     images_path = rospy.get_param('/faa_experiment/images_path')
    #     trial_path = rospy.get_param('/faa_experiment/trial_path')
    #     input = os.path.join(images_path,"image_%06d.png")

    #     experiment_name = rospy.get_param('/faa_experiment/experiment_name')
    #     trial_name = rospy.get_param('/faa_experiment/trial_name')
    #     time_str = rospy.get_param('/faa_experiment/trial_start_time')
    #     file_name = "Video" + experiment_name + trial_name + '-' + time_str + '.mp4'
    #     output = os.path.join(trial_path,file_name)

    #     first_pass = "ffmpeg -f image2 -i {input} -vcodec libx264 {quality} -threads 8 {output}".format(input=input,quality=self.quality,output=output)
    #     # subprocess.check_call(first_pass,shell=True)
        # self.lock.release()


def main():
    rospy.init_node('faa_video_save', anonymous=True)
    vs = VideoSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main()

