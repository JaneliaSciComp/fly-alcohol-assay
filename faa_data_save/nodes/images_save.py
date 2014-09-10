#!/usr/bin/env python
import roslib
roslib.load_manifest('faa_data_save')
import rospy
import cv2
import numpy
import os

from faa_data_save.msg import SaveData
from faa_utilities import FileTools

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


file_tools = FileTools()

class ImagesSaver(object):

    def __init__(self):
        self.saving = False
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_trimmed",Image,self.trimmed_image_callback)
        self.setup_image_saving()

    def setup_image_saving(self):
        self.image_count = 0
        # images_path = rospy.get_param('/faa_experiment/images_path')
        images_path = os.path.expanduser('~/tmp/faa_images')
        os.makedirs(images_path)
        image_name_base = "image_{count:06d}.png"
        self.image_path = os.path.join(images_path,image_name_base)

    def trimmed_image_callback(self,data):
        try:
          image_cv = self.bridge.imgmsg_to_cv(data, "mono8")
          image_np = numpy.asarray(image_cv)
          path = self.image_path.format(count=self.image_count)
          cv2.imwrite(path,image_np,[cv2.IMWRITE_PNG_COMPRESSION,0])
          self.image_count += 1
        except CvBridgeError, e:
          print e


def main():
    rospy.init_node('faa_images_save', anonymous=True)
    iss = ImagesSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main()

