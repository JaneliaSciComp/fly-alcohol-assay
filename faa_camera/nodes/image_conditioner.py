#!/usr/bin/env python
import roslib
roslib.load_manifest('faa_camera')
import sys
import rospy
import cv
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Conditioner(object):

  def __init__(self):
    self.image_pub = rospy.Publisher("image_conditioned",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_rect",Image,self.callback)

    self.display_image_window = False
    if self.display_image_window:
      cv.NamedWindow("Image Conditioned", 1)

  def callback(self,data):
    try:
      image_cv = self.bridge.imgmsg_to_cv(data, "passthrough")
    except CvBridgeError, e:
      print e

    image_np = numpy.asarray(image_cv)
    image_conditioned = cv2.flip(image_np,1)

    image_cv = cv.fromarray(image_conditioned)
    if self.display_image_window:
      cv.ShowImage("Image Conditioned", image_cv)
      cv.WaitKey(3)

    try:
      image_conditioned = self.bridge.cv_to_imgmsg(image_cv, "passthrough")
      image_conditioned.header = data.header
      self.image_pub.publish(image_conditioned)
      # self.image_pub.publish(self.bridge.cv_to_imgmsg(image_cv, "passthrough"))
    except CvBridgeError, e:
      print e

def main(args):
  rospy.init_node('faa_image_conditioner', anonymous=True)
  conditioner = Conditioner()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  if conditioner.display_image_window:
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

