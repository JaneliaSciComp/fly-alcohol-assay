from __future__ import print_function,division
import roslib
roslib.load_manifest('faa_image_processing')
import rospy

import os
import cv2
import numpy
import math
import time
import threading


class Tracker(object):

    def __init__(self,*args,**kwargs):
        self.lock = threading.Lock()
        self.parameters = None
        self.bg_gates_opened = None
        self.bg_gates_closed = None
        self.tunnels_state = None
        self.data_tracked_previous = None

    def setParameters(self,parameters):
        self.lock.acquire()
        self.parameters = parameters
        self.lock.release()

    def setBgImageGatesOpened(self,image):
        self.lock.acquire()
        self.bg_gates_opened = image
        self.lock.release()

    def setBgImageGatesClosed(self,image):
        self.lock.acquire()
        self.bg_gates_closed = image
        self.lock.release()

    def setTunnelsState(self,tunnels_state):
        self.lock.acquire()
        self.tunnels_state = tunnels_state
        self.lock.release()

    def processImage(self,image):
        data_tracked = {}
        # image_tracked = numpy.copy(image)
        data_image = numpy.zeros((self.parameters.tunnel_height,
                                  self.parameters.tunnel_width*self.parameters.tunnel_count,
                                  3),
                                 image.dtype)
        if self.lock.acquire(False) and \
               (self.parameters is not None) and \
               (self.bg_gates_opened is not None) and \
               (self.bg_gates_closed is not None) and \
               (self.tunnels_state is not None):
            tunnels = range(self.parameters.tunnel_count)
            enabled_tunnels = [tunnel for tunnel in tunnels if self.parameters.tunnels_enabled[tunnel]]
            for tunnel in enabled_tunnels:
                tunnel_state = self.tunnels_state[tunnel]
                tunnel_x_offset = self.parameters.tunnel_x_offsets[tunnel]
                tunnel_y_offset = self.parameters.tunnel_y_offset
                # Apply tunnel mask
                tx0 = self.parameters.tunnel_mask['x0'] + tunnel_x_offset
                ty0 = self.parameters.tunnel_mask['y0'] + tunnel_y_offset
                tx1 = self.parameters.tunnel_mask['x1'] + tunnel_x_offset
                ty1 = self.parameters.tunnel_mask['y1'] + tunnel_y_offset
                image_tunnel = image[ty0:ty1,tx0:tx1]
                image_tunnel_bg_gates_opened = self.bg_gates_opened[ty0:ty1,tx0:tx1]
                image_tunnel_bg_gates_closed = self.bg_gates_closed[ty0:ty1,tx0:tx1]
                image_tunnel_bg = numpy.copy(image_tunnel_bg_gates_closed)
                # image_tunnel_bg_sub = numpy.zeros(image_tunnel.shape,numpy.uint8)
                for gate in range(self.parameters.gate_count):
                    gx0 = self.parameters.gate_bg_masks[gate]['x0']
                    gy0 = self.parameters.gate_bg_masks[gate]['y0']
                    gx1 = self.parameters.gate_bg_masks[gate]['x1']
                    gy1 = self.parameters.gate_bg_masks[gate]['y1']
                    gate_state = tunnel_state.gates_state[gate]
                    if gate_state == 'open':
                        image_tunnel_bg[gy0:gy1,gx0:gx1] = image_tunnel_bg_gates_opened[gy0:gy1,gx0:gx1]
                        # image_tunnel_bg_sub[gy0:gy1,gx0:gx1] = cv2.absdiff(image_tunnel[gy0:gy1,gx0:gx1],
                        #                                                    image_tunnel_bg_gates_opened[gy0:gy1,gx0:gx1])
                    # elif gate_state == 'close':
                    #     image_tunnel_bg[gy0:gy1,gx0:gx1] = image_tunnel_bg_gates_closed[gy0:gy1,gx0:gx1])
                        # image_tunnel_bg_sub[gy0:gy1,gx0:gx1] = cv2.absdiff(image_tunnel[gy0:gy1,gx0:gx1],
                        #                                                    image_tunnel_bg_gates_closed[gy0:gy1,gx0:gx1])
                    # else:
                    #     rospy.logwarn('gate_state is not open or close!')
                    #     return data_tracked,image_tracked
                image_tunnel_fg = cv2.absdiff(image_tunnel,image_tunnel_bg)

                data_x_offset = tunnel*self.parameters.tunnel_width
                dx0 = data_x_offset
                dx1 = data_x_offset + self.parameters.tunnel_width
                data_image[:,dx0:dx1,0] = image_tunnel
                data_image[:,dx0:dx1,1] = image_tunnel_bg
                data_image[:,dx0:dx1,2] = image_tunnel_fg

                # retval,image_tunnel_thresh = cv2.threshold(image_tunnel_fg,
                #                                            0,
                #                                            255,
                #                                            cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                retval,image_tunnel_thresh = cv2.threshold(image_tunnel_fg,
                                                           self.parameters.threshold,
                                                           255,
                                                           cv2.THRESH_BINARY)
                # retval,image_tunnel_thresh = cv2.threshold(image_tunnel_bg_sub,
                #                                            self.parameters.threshold,255,cv2.THRESH_BINARY)
                kernel = self.parameters.morph_kernel
                image_tunnel_morphed = cv2.morphologyEx(image_tunnel_thresh,cv2.MORPH_OPEN,kernel)
                # image_tracked[ty0:ty1,tx0:tx1] = image_tunnel_bg_sub
                # image_tracked[ty0:ty1,tx0:tx1] = image_tunnel_thresh
                # contours = self._findContours(image_tunnel_thresh)
                # image_tracked[ty0:ty1,tx0:tx1] = image_tunnel_morphed
                contours = self._findContours(image_tunnel_morphed)
                contour_count = len(contours)
                data_tunnel = {}
                if contour_count == 0:
                    data_tunnel = {}
                elif contour_count == 1:
                    data_tunnel = self._processContour(contours[0])
                    data_tunnel = self._calculateTunnelData(data_tunnel)
                elif (contour_count < 5) and (self.data_tracked_previous is not None):
                    data_tunnel_previous = self.data_tracked_previous[tunnel]
                    if len(data_tunnel_previous) > 0:
                        data_tunnel_list = []
                        for contour_n in range(contour_count):
                            d_t = self._processContour(contours[contour_n])
                            d_t = self._calculateTunnelData(d_t)
                            data_tunnel_list.append(d_t)
                        data_tunnel = self._selectBestDataTunnel(data_tunnel_list,data_tunnel_previous)
                data_tunnel['contours'] = contours
                for gate in range(len(tunnel_state.gates_state)):
                    data_tunnel['gate{0}'.format(gate)] = tunnel_state.gates_state[gate]
                data_tracked[tunnel] = data_tunnel
            self.lock.release()
        else:
            try:
                self.lock.release()
            except RuntimeError:
                pass
        self.data_tracked_previous = data_tracked
        return data_tracked,data_image

    def _selectBestDataTunnel(self,data_tunnel_list,data_tunnel_previous):
        data_tunnel = {}
        dist_min = 1000000
        # first throw away all data that is potentially bad
        for d_t in data_tunnel_list:
            # gates are most likely places to get spurious detections
            # so reject all blobs within the gate boundaries, but this
            # will fail when the real fly is on top of a gate
            if (d_t['chamber'] == 'gate2') or (d_t['chamber'] == 'gate1') or (d_t['chamber'] == 'gate0'):
                data_tunnel_list.remove(d_t)
        # if only one blob is left, consider it good data
        if len(data_tunnel_list) == 1:
            data_tunnel = data_tunnel_list[0]
        # otherwise pick the blob that is closest in distance to the
        # previously detected fly
        else:
            for d_t in data_tunnel_list:
                try:
                    x = d_t['fly_x']
                    x_p = data_tunnel_previous['fly_x']
                    y = d_t['fly_y']
                    y_p = data_tunnel_previous['fly_y']
                    dist = math.sqrt((x-x_p)**2 + (y-y_p)**2)
                    if dist < dist_min:
                        dist_min = dist
                        data_tunnel = d_t
                except KeyError:
                    pass
        return data_tunnel

    def _findContours(self,image):
        # Finding contours affects image so make copy
        image = numpy.copy(image)
        contours,hierarcy = cv2.findContours(image,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def _processContour(self,contour):
        moments = cv2.moments(contour)
        data = self._momentsToData(moments)
        slope = data['blob_slope']
        if not math.isnan(slope):
            if self.parameters.min_ecc < data['blob_ecc']:
                angle = math.atan2(-slope,1)
            else:
                angle = 0
        else:
            angle = 0
        data['blob_angle'] = angle
        return data

    def _momentsToData(self,moments):
        if moments['m00'] != 0:
            xc = moments['m10']/moments['m00']
            yc = moments['m01']/moments['m00']
        else:
            xc = 0
            yc = 0
        area = moments['m00']
        slope,ecc = self._findSlopeEcc(moments['mu20'],moments['mu11'],moments['mu11'],moments['mu02'])
        data = {
            'blob_x' : xc,
            'blob_y' : yc,
            'blob_area' : area,
            'blob_slope' : slope,
            'blob_ecc' : ecc,
            }
        return data

    def _findSlopeEcc(self,A,B,C,D):
        slope = float('NaN')
        ecc = 0
        if C == 0:
            # divide by zero
            return slope,ecc
        inside = A*A + 4*B*C - 2*A*D + D*D
        if inside < 0:
            # complex answer
            return slope,ecc
        inside = math.sqrt(inside)
        evalA = 0.5*(A+D-inside)
        evalB = 0.5*(A+D+inside)
        evecA1 = (-A+D+inside)/(-2*C)
        evecB1 = (-A+D-inside)/(-2*C)
        rise = 1
        try:
            if evalB < evalA:
                run = evecA1
                ecc = evalA/evalB
            else:
                run = evecB1
                ecc = evalB/evalA
                slope = rise/run
        except (ZeroDivisionError):
            pass
        return slope,ecc

    # def _calculateTunnelData(self,data_image,gates_state):
    def _calculateTunnelData(self,data):
        data_tunnel = data
        data_tunnel['fly_x'] = (data['blob_x'] - self.parameters.origin_x_offset_tunnel)/self.parameters.pixels_per_mm
        data_tunnel['fly_y'] = (self.parameters.origin_y_offset_tunnel - data['blob_y'])/self.parameters.pixels_per_mm

        blob_angle = data['blob_angle']
        if 0 < blob_angle:
            fly_angle = blob_angle - math.pi/2
        else:
            fly_angle = blob_angle + math.pi/2
        fly_angle = round(fly_angle*(180/math.pi))
        data_tunnel['fly_angle'] = fly_angle
        blob_y = data['blob_y'] + self.parameters.tunnel_y_offset
        if blob_y <= (self.parameters.gate_y_offsets[2]+self.parameters.gate_height):
            data_tunnel['chamber'] = 'gate2'
        elif (self.parameters.gate_y_offsets[2]+self.parameters.gate_height) < blob_y <= self.parameters.gate_y_offsets[1]:
            data_tunnel['chamber'] = 'end'
        elif self.parameters.gate_y_offsets[1] < blob_y <= (self.parameters.gate_y_offsets[1]+self.parameters.gate_height):
            data_tunnel['chamber'] = 'gate1'
        elif (self.parameters.gate_y_offsets[1]+self.parameters.gate_height) < blob_y <= self.parameters.gate_y_offsets[0]:
            data_tunnel['chamber'] = 'walkway'
        elif self.parameters.gate_y_centers[0] < blob_y <= (self.parameters.gate_y_offsets[0]+self.parameters.gate_height):
            data_tunnel['chamber'] = 'gate0'
        elif (self.parameters.gate_y_offsets[0]+self.parameters.gate_height) < blob_y:
            data_tunnel['chamber'] = 'start'
        else:
            data_tunnel['chamber'] = 'unknown'

        # for gate in range(len(gates_state)):
        #     data_tunnel['gate{0}'.format(gate)] = gates_state[gate]

        return data_tunnel

