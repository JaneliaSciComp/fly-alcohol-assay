from __future__ import print_function,division
import roslib
roslib.load_manifest('faa_image_processing')
import rospy
import math
import json
import cv2


class Parameters(object):

    def __init__(self):
        self.update_parameters_from_parameter_server()
        self.reset_image_count()
        self.update()
        self.status = None

    def reset_image_count(self):
        self.image_count = 0

    def increment_image_count(self):
        self.image_count += 1

    def update_parameters_from_parameter_server(self):
        self.calibration_line_thickness = rospy.get_param("/faa_image_processing/calibration_line_thickness")
        self.gate_height = rospy.get_param("/faa_image_processing/gate_height")
        self.gate_0_1_dist_mm = rospy.get_param("/faa_image_processing/gate_0_1_dist_mm")
        self.gate_y_offsets = rospy.get_param("/faa_image_processing/gate_y_offsets")
        self.min_ecc = rospy.get_param("/faa_image_processing/min_ecc")
        self.morph_kernel_size = rospy.get_param("/faa_image_processing/morph_kernel_size")
        self.threshold = rospy.get_param("/faa_image_processing/threshold")
        self.tunnel_height = rospy.get_param("/faa_image_processing/tunnel_height")
        self.tunnel_width = rospy.get_param("/faa_image_processing/tunnel_width")
        self.tunnel_x_offsets = rospy.get_param("/faa_image_processing/tunnel_x_offsets")
        self.tunnel_y_offset = rospy.get_param("/faa_image_processing/tunnel_y_offset")
        self.tunnels_enabled = rospy.get_param("/faa_experiment/tunnels_enabled")

        self.image_height = rospy.get_param("/faa_image_processing/image_height")
        self.image_width = rospy.get_param("/faa_image_processing/image_width")

    def get_parameters_json(self):
        parameters = {}
        parameters['calibration_line_thickness'] = self.calibration_line_thickness
        parameters['gate_height'] = self.gate_height
        parameters['gate_0_1_dist_mm'] = self.gate_0_1_dist_mm
        parameters['gate_y_offsets'] = self.gate_y_offsets
        parameters['threshold'] = self.threshold
        parameters['tunnel_height'] = self.tunnel_height
        parameters['tunnel_width'] = self.tunnel_width
        parameters['tunnel_x_offsets'] = self.tunnel_x_offsets
        parameters['tunnel_y_offset'] = self.tunnel_y_offset
        parameters['tunnels_enabled'] = self.tunnels_enabled
        parameters_json = json.dumps(parameters,sort_keys=True,separators=(',', ':'))
        return parameters_json

    def set_parameter(self,name,value):
        name = str(name).lower()
        if name == 'calibration_line_thickness':
            value = int(value)
            self.calibration_line_thickness = value
            rospy.set_param("/faa_image_processing/calibration_line_thickness",value)
        elif name == 'gate_height':
            value = int(value)
            self.gate_height = value
            rospy.set_param("/faa_image_processing/gate_height",value)
        elif name == 'gate_0_1_dist_mm':
            self.gate_0_1_dist_mm = value
            rospy.set_param("/faa_image_processing/gate_0_1_dist_mm",value)
        elif name == 'min_ecc':
            value = int(value)
            self.min_ecc = value
            rospy.set_param("/faa_image_processing/min_ecc",value)
        elif name == 'morph_kernel_size':
            value = int(value)
            self.morph_kernel_size = value
            rospy.set_param("/faa_image_processing/morph_kernel_size",value)
        elif name == 'threshold':
            value = int(value)
            self.threshold = value
            rospy.set_param("/faa_image_processing/threshold",value)
        elif name == 'tunnel_height':
            value = int(value)
            self.tunnel_height = value
            rospy.set_param("/faa_image_processing/tunnel_height",value)
        elif name == 'tunnel_width':
            value = int(value)
            self.tunnel_width = value
            rospy.set_param("/faa_image_processing/tunnel_width",value)
        elif name == 'tunnel_y_offset':
            value = int(value)
            self.tunnel_y_offset = value
            rospy.set_param("/faa_image_processing/tunnel_y_offset",value)
        self.update()

    def set_array_parameter(self,name,array):
        name = str(name).lower()
        if name == 'gate_y_offsets':
            self.gate_y_offsets = array
            rospy.set_param("/faa_image_processing/gate_y_offsets",array)
        elif name == 'tunnel_x_offsets':
            self.tunnel_x_offsets = array
            rospy.set_param("/faa_image_processing/tunnel_x_offsets",array)
        elif name == 'tunnels_enabled':
            self.tunnels_enabled = [bool(item) for item in array]
        self.update()

    def update(self):
        # if 'debug' not in kwargs:
        #     kwargs.update({'debug': DEBUG})
        if (self.tunnel_height + self.tunnel_y_offset) > self.image_height:
            self.tunnel_height = self.image_height - self.tunnel_y_offset
            self.set_parameter('tunnel_height',self.tunnel_height)
        self.tunnel_mask = {'x0': 0,
                            'y0': 0,
                            'x1': self.tunnel_width,
                            'y1': self.tunnel_height}
        self.tunnel_count = len(self.tunnel_x_offsets)
        self.gate_mask = {'x0': 0,
                          'y0': 0,
                          'x1': self.tunnel_mask['x1'],
                          'y1': self.gate_height}
        self.gate_count = len(self.gate_y_offsets)
        # self.gate_0_1_dist_mm = 100.8
        self.pixels_per_mm = (self.gate_y_offsets[0] - self.gate_y_offsets[1])/self.gate_0_1_dist_mm
        self.origin_x_offset_tunnel = int(self.tunnel_width/2)
        self.origin_y_offset_tunnel = self.gate_y_offsets[0] - self.tunnel_y_offset
        self.gate_half_height = int((self.gate_mask['y1'] - self.gate_mask['y0'])/2)
        self.gate_y_centers = [offset + self.gate_half_height for offset in self.gate_y_offsets]

        self.gate_bg_masks = []
        self.gate_bg_masks.append({'x0': 0,
                                   'y0': (self.gate_y_centers[0] + self.gate_y_centers[1])/2 - self.tunnel_y_offset,
                                   'x1': self.tunnel_mask['x1'],
                                   'y1': self.tunnel_mask['y1']})
        self.gate_bg_masks.append({'x0': 0,
                                   'y0': (self.gate_y_centers[1] + self.gate_y_centers[2])/2 - self.tunnel_y_offset,
                                   'x1': self.tunnel_mask['x1'],
                                   'y1': self.gate_bg_masks[0]['y0']})
        self.gate_bg_masks.append({'x0': 0,
                                   'y0': self.tunnel_mask['y0'],
                                   'x1': self.tunnel_mask['x1'],
                                   'y1': self.gate_bg_masks[1]['y0']})
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(self.morph_kernel_size,self.morph_kernel_size))
