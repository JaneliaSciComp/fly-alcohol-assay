#!/usr/bin/env python
import roslib
roslib.load_manifest('faa_image_processing')
import sys
import rospy
import cv
import cv2
import numpy
import os
import copy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from faa_utilities import FileTools
from faa_actuation.msg import ActuationState

from faa_image_processing import Tracker, Drawer, Parameters
from faa_image_processing.srv import SaveImage, SaveImageResponse
from faa_image_processing.srv import SetTracking, SetTrackingResponse
from faa_image_processing.srv import GetParameters, GetParametersResponse
from faa_image_processing.srv import SetParameter, SetParameterResponse
from faa_image_processing.srv import SetArrayParameter, SetArrayParameterResponse
from faa_image_processing.srv import SetStatus, SetStatusResponse
from faa_image_processing.msg import TrackingData, TunnelData

file_tools = FileTools()

class ImageProcessor(object):

  def __init__(self):
    self.reusing_bg_images = rospy.get_param('/camera/faa_image_processing/reusing_bg_images')

    self.image_p_pub = rospy.Publisher("image_processed",Image)
    self.image_d_pub = rospy.Publisher("data_image",Image)

    self.tracking_data_pub = rospy.Publisher("tracking_data",TrackingData)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_conditioned",Image,self.conditioned_image_callback)

    self.actuation_state_sub = rospy.Subscriber("/faa_actuation/actuation_state",ActuationState,self.actuation_state_callback)

    self.tracking = False
    self.drawing = False
    self.tracker = Tracker()
    self.drawer = Drawer()
    self.parameters = Parameters()
    self.tracker.setParameters(self.parameters)
    self.drawer.setParameters(self.parameters)

    self.display_images = False
    if self.display_images:
      # cv.NamedWindow("Image Processed", 1)
      # cv.NamedWindow("Image Tracked", 1)
      cv.NamedWindow("Image Data", 1)

    self.image_conditioned = None
    self.sbi = rospy.Service('/faa_image_processing/save_background_image', SaveImage, self.save_background_image_callback)
    self.st = rospy.Service('/faa_image_processing/set_tracking', SetTracking, self.set_tracking_callback)
    self.gp = rospy.Service('/faa_image_processing/get_parameters', GetParameters, self.get_parameters_callback)
    self.sp = rospy.Service('/faa_image_processing/set_parameter', SetParameter, self.set_parameter_callback)
    self.sap = rospy.Service('/faa_image_processing/set_array_parameter', SetArrayParameter, self.set_array_parameter_callback)
    self.ss = rospy.Service('/faa_image_processing/set_status', SetStatus, self.set_status_callback)

  def conditioned_image_callback(self,data):
    self.parameters.increment_image_count()
    self.tracker.setParameters(self.parameters)
    self.drawer.setParameters(self.parameters)
    try:
      image_cv = self.bridge.imgmsg_to_cv(data, "passthrough")
    except CvBridgeError, e:
      print e

    image_np = numpy.asarray(image_cv)
    self.image_conditioned = numpy.copy(image_np)

    if self.tracking:
      # data_tracked,image_tracked = self.tracker.processImage(image_np)
      data_tracked,data_image = self.tracker.processImage(image_np)
      tracking_data = TrackingData()
      tracking_data.header = data.header
      tracking_data.image_count = self.parameters.image_count
      # rospy.logwarn(str(data_tracked))
      # try:
      #   data_tracked_tunnels = data_tracked['tunnels']
      # except KeyError:
      #   data_tracked_tunnels = []
      # if 0 < len(data_tracked_tunnels):
      #   for tunnel in range(len(data_tracked_tunnels)):
      tunnel_data_list = []
      tunnels = range(self.parameters.tunnel_count)
      for tunnel in tunnels:
        tunnel_data = TunnelData()
        enabled = self.parameters.tunnels_enabled[tunnel]
        tunnel_data.tunnel = tunnel
        tunnel_data.enabled = enabled
        tunnel_data.gate0 = ""
        tunnel_data.gate1 = ""
        tunnel_data.gate2 = ""
        tunnel_data.fly_x = 0
        tunnel_data.fly_y = 0
        tunnel_data.fly_angle = 0
        tunnel_data.chamber = ""
        tunnel_data.blob_x = 0
        tunnel_data.blob_y = 0
        tunnel_data.blob_area = 0
        tunnel_data.blob_slope = 0
        tunnel_data.blob_ecc = 0
        if enabled:
          try:
            tunnel_data.gate0 = data_tracked[tunnel]['gate0']
            tunnel_data.gate1 = data_tracked[tunnel]['gate1']
            tunnel_data.gate2 = data_tracked[tunnel]['gate2']
          except KeyError:
            pass
          try:
            tunnel_data.fly_x = data_tracked[tunnel]['fly_x']
            tunnel_data.fly_y = data_tracked[tunnel]['fly_y']
            tunnel_data.fly_angle = data_tracked[tunnel]['fly_angle']
            tunnel_data.chamber = data_tracked[tunnel]['chamber']
            tunnel_data.blob_x = data_tracked[tunnel]['blob_x']
            tunnel_data.blob_y = data_tracked[tunnel]['blob_y']
            tunnel_data.blob_area = data_tracked[tunnel]['blob_area']
            tunnel_data.blob_slope = data_tracked[tunnel]['blob_slope']
            tunnel_data.blob_ecc = data_tracked[tunnel]['blob_ecc']
          except KeyError:
            pass

        tunnel_data_list.append(tunnel_data)
      tracking_data.tunnel_data = tunnel_data_list
      self.tracking_data_pub.publish(tracking_data)
      data_image_cv = cv.fromarray(data_image)
      try:
        data_img = self.bridge.cv_to_imgmsg(data_image_cv, "bgr8")
        data_img.header = data.header
        self.image_d_pub.publish(data_img)
      except CvBridgeError, e:
        print e
      # image_tracked_cv = cv.fromarray(image_tracked)
      # if self.display_images:
      #   cv.ShowImage("Image Tracked", image_tracked_cv)
      #   cv.WaitKey(3)
    else:
      data_tracked = {}

    if self.drawing:
      image_processed = self.drawer.processImage(image_np,data_tracked)
    else:
      image_processed = cv2.cvtColor(image_np,cv2.COLOR_GRAY2RGB)

    image_cv = cv.fromarray(image_processed)
    # if self.display_images:
    #   cv.ShowImage("Image Processed", image_cv)
    #   cv.WaitKey(3)

    try:
      self.image_p_pub.publish(self.bridge.cv_to_imgmsg(image_cv, "bgr8"))
    except CvBridgeError, e:
      print e

    # image_trimmed = self.trim_image(image_np)
    # image_trimmed_cv = cv.fromarray(image_trimmed)
    # if self.display_images:
    #   cv.ShowImage("Image Data", image_trimmed_cv)
    #   cv.WaitKey(3)
    # try:
    #   self.image_d_pub.publish(self.bridge.cv_to_imgmsg(image_trimmed_cv, "mono8"))
    # except CvBridgeError, e:
    #   print e

  # def trim_image(self,image_o):
  #   image_t = numpy.zeros((ty,tx*self.parameters.tunnel_count),image_o.dtype)
  #   for tunnel in range(self.parameters.tunnel_count):
  #     if self.parameters.tunnels_enabled[tunnel]:
  #       x_offset_o = self.parameters.tunnel_x_offsets[tunnel]
  #       x_offset_t = tunnel*tx
  #       tx0_o = self.parameters.tunnel_mask['x0'] + x_offset_o
  #       ty0_o = self.parameters.tunnel_mask['y0']
  #       tx1_o = self.parameters.tunnel_mask['x1'] + x_offset_o
  #       ty1_o = self.parameters.tunnel_mask['y1']
  #       tx0_t = x_offset_t
  #       ty0_t = 0
  #       tx1_t = self.parameters.tunnel_mask['x1'] + x_offset_t
  #       ty1_t = self.parameters.tunnel_mask['y1']
  #       image_t[ty0_t:ty1_t,tx0_t:tx1_t] = image_o[ty0_o:ty1_o,tx0_o:tx1_o]
  #   return image_t

  def save_background_image_callback(self,req):
    if self.image_conditioned is not None:
      self.drawing = True
      (path,filename) = os.path.split(req.image_path)
      if not self.reusing_bg_images:
        image_background = numpy.copy(self.image_conditioned)
      else:
        image_background = file_tools.read_image_file(filename)
      file_tools.write_image_file(req.image_path,image_background)
      # rospy.logwarn("save_background_image_callback: " + filename)
      if filename == 'bg_gates_opened.png':
        self.tracker.setBgImageGatesOpened(image_background)
      elif filename == 'bg_gates_closed.png':
        self.tracker.setBgImageGatesClosed(image_background)
    return SaveImageResponse("success")

  def set_tracking_callback(self,req):
    self.tracking = req.tracking
    self.parameters.reset_image_count()
    self.tracker.setParameters(self.parameters)
    self.drawer.setParameters(self.parameters)
    return SetTrackingResponse("success")

  def actuation_state_callback(self,req):
    self.tracker.setTunnelsState(req.tunnels_state)

  def get_parameters_callback(self,req):
    return GetParametersResponse(self.parameters.get_parameters_json())

  def set_parameter_callback(self,req):
    self.parameters.set_parameter(req.name,req.value)
    self.tracker.setParameters(self.parameters)
    self.drawer.setParameters(self.parameters)
    return SetParameterResponse("success")

  def set_array_parameter_callback(self,req):
    self.parameters.set_array_parameter(req.name,req.array)
    self.tracker.setParameters(self.parameters)
    self.drawer.setParameters(self.parameters)
    return SetArrayParameterResponse("success")

  def set_status_callback(self,req):
    self.parameters.status = req.status
    self.tracker.setParameters(self.parameters)
    self.drawer.setParameters(self.parameters)
    return SetStatusResponse("success")


def main(args):
  rospy.init_node('faa_image_processing', anonymous=True)
  ip = ImageProcessor()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  if ip.display_images:
    cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

