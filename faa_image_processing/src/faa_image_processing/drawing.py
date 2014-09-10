from __future__ import print_function,division
import os
import cv2
import numpy
import math
import time
import threading


class Drawer(object):

    def __init__(self,*args,**kwargs):
        self.lock = threading.Lock()
        self.parameters = None
        self.frame_count = 0
        self.font_face = cv2.FONT_HERSHEY_PLAIN
        self.font_scale = 1.5
        self.font_thickness = 2
        self.color_text = (255,0,255)
        self.color_line = (0,255,0)
        self.color_rect = (255,0,0)
        self.color_gate_calib = (0,255,0)
        self.color_tunnel_calib = (255,0,0)
        self.color_cont = (0,0,255)
        self.color_line_slope = (255,0,0)
        self.color_x_axis = (0,255,255)
        self.color_y_axis = (255,255,0)
        self.line_slope_length = 40

    def setParameters(self,parameters):
        self.lock.acquire()
        self.parameters = parameters
        self.lock.release()

    def processImage(self,image,data_tunnels):
        if self.lock.acquire(False) and \
               (self.parameters is not None):
            image_color = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
            # try:
            #     data_tunnels = data['tunnels']
            # except KeyError:
            #     data_tunnels = {}
            image_processed = self._annotateFullImage(image_color,data_tunnels)
            self.lock.release()
            return image_processed
        else:
            try:
                self.lock.release()
            except RuntimeError:
                pass
            return image

    def _annotateFullImage(self,image,data_tunnels):
        text_spacing = 50
        for tunnel in range(self.parameters.tunnel_count):
            tunnel_offset = self.parameters.tunnel_x_offsets[tunnel]
            if tunnel%2 == 0:
                text_offset = self.parameters.tunnel_x_offsets[tunnel]
                text_y = self.parameters.gate_y_offsets[1] + text_spacing
                color_text = self.color_text
                text_x = int(text_offset + 1.1*self.parameters.tunnel_width)
            else:
                text_offset = self.parameters.tunnel_x_offsets[tunnel-1]
                text_y = int((self.parameters.gate_y_offsets[1] + self.parameters.gate_y_offsets[0])/2) + text_spacing
                color_text = (0,125,255)
                text_x = int(text_offset + 1.5*self.parameters.tunnel_width)
            if tunnel == 0:
                cv2.putText(image,
                            self.parameters.status,
                            (text_x,self.parameters.gate_y_offsets[1] - text_spacing),
                            self.font_face,self.font_scale,(255,255,255),self.font_thickness)
            elif tunnel == 2:
                cv2.putText(image,
                            "Frame: ",
                            (text_x,self.parameters.gate_y_offsets[1] - text_spacing),
                            self.font_face,self.font_scale,(255,255,255),self.font_thickness)
                cv2.putText(image,
                            str(self.parameters.image_count),
                            (text_x,self.parameters.gate_y_offsets[1] - int(text_spacing/2)),
                            self.font_face,self.font_scale,(255,255,255),self.font_thickness)
            if not self.parameters.tunnels_enabled[tunnel]:
                continue
            tunnel_dot_x = int(tunnel_offset+self.parameters.tunnel_width/2)
            tunnel_dot_y = int(self.parameters.gate_y_offsets[2]/2)
            # tunnel_dot_r = int(tunnel_dot_y*0.75)
            tunnel_dot_r = int(self.parameters.tunnel_width*0.25)
            cv2.circle(image,(tunnel_dot_x,tunnel_dot_y),tunnel_dot_r,color_text,-1)
            if tunnel%2 == 0:
                cv2.putText(image,
                            "<----",
                            (text_x,text_y),
                            self.font_face,self.font_scale,color_text,self.font_thickness)
            else:
                cv2.putText(image,
                            "---->",
                            (text_x,text_y),
                            self.font_face,self.font_scale,color_text,self.font_thickness)
            text_y += text_spacing
            cv2.putText(image,
                        "Tunnel: " + str(tunnel+1),
                        (text_x,text_y),
                        self.font_face,self.font_scale,color_text,self.font_thickness)
            text_y += text_spacing

            if len(data_tunnels) == 0:
                # Draw gates
                for gate in range(self.parameters.gate_count):
                    cv2.rectangle(image,
                                  (self.parameters.gate_mask['x0']+tunnel_offset,self.parameters.gate_mask['y0']+self.parameters.gate_y_offsets[gate]),
                                  (self.parameters.gate_mask['x1']+tunnel_offset,self.parameters.gate_mask['y1']+self.parameters.gate_y_offsets[gate]),
                                  self.color_gate_calib,self.parameters.calibration_line_thickness)
                # Draw tunnels
                cv2.rectangle(image,
                              (self.parameters.tunnel_mask['x0']+tunnel_offset,self.parameters.tunnel_mask['y0']+self.parameters.tunnel_y_offset),
                              (self.parameters.tunnel_mask['x1']+tunnel_offset,self.parameters.tunnel_mask['y1']+self.parameters.tunnel_y_offset),
                              self.color_tunnel_calib,self.parameters.calibration_line_thickness)

            else:
                rectangle_thickness = 4
                try:
                    data_tunnel = data_tunnels[tunnel]
                except KeyError:
                    data_tunnel = {}
                try:
                    fly_x = data_tunnel['fly_x']
                    cv2.putText(image,
                                "x: {0: .1f}".format(fly_x),
                                (text_x,text_y),
                                self.font_face,self.font_scale,color_text,self.font_thickness)
                except KeyError:
                    pass
                text_y += text_spacing
                try:
                    fly_y = data_tunnel['fly_y']
                    cv2.putText(image,
                                "y: {0: .1f}".format(fly_y),
                                (text_x,text_y),
                                self.font_face,self.font_scale,color_text,self.font_thickness)
                except KeyError:
                    pass
                text_y += text_spacing
                try:
                    angle = data_tunnel['fly_angle']
                    cv2.putText(image,
                                "Angle: " + str(angle),
                                (text_x,text_y),
                                self.font_face,self.font_scale,color_text,self.font_thickness)
                except KeyError:
                    pass
                text_y += text_spacing
                try:
                    chamber = data_tunnel['chamber']
                    if chamber == 'end':
                        cv2.rectangle(image,
                                      (self.parameters.tunnel_mask['x0']+tunnel_offset,self.parameters.gate_y_offsets[2]+self.parameters.gate_height),
                                      (self.parameters.tunnel_mask['x1']+tunnel_offset,self.parameters.gate_y_offsets[1]),
                                      self.color_rect,rectangle_thickness)
                        cv2.putText(image,
                                    "End",
                                    (text_x,text_y),
                                    self.font_face,self.font_scale,color_text,self.font_thickness)
                    elif chamber == 'walkway':
                        cv2.rectangle(image,
                                      (self.parameters.tunnel_mask['x0']+tunnel_offset,self.parameters.gate_y_offsets[1]+self.parameters.gate_height),
                                      (self.parameters.tunnel_mask['x1']+tunnel_offset,self.parameters.gate_y_offsets[0]),
                                      self.color_rect,rectangle_thickness)
                        cv2.putText(image,
                                    "Walkway",
                                    (text_x,text_y),
                                    self.font_face,self.font_scale,color_text,self.font_thickness)
                    elif chamber == 'start':
                        cv2.rectangle(image,
                                      (self.parameters.tunnel_mask['x0']+tunnel_offset,self.parameters.gate_y_offsets[0]+self.parameters.gate_height),
                                      (self.parameters.tunnel_mask['x1']+tunnel_offset,self.parameters.tunnel_mask['y1']+self.parameters.tunnel_y_offset),
                                      self.color_rect,rectangle_thickness)
                        cv2.putText(image,
                                    "Start",
                                    (text_x,text_y),
                                    self.font_face,self.font_scale,color_text,self.font_thickness)
                    elif chamber == 'gate2':
                        cv2.rectangle(image,
                                      (self.parameters.tunnel_mask['x0']+tunnel_offset,self.parameters.gate_y_offsets[2]),
                                      (self.parameters.tunnel_mask['x1']+tunnel_offset,self.parameters.gate_y_offsets[2]+self.parameters.gate_height),
                                      self.color_rect,rectangle_thickness)
                        cv2.putText(image,
                                    "Gate 3",
                                    (text_x,text_y),
                                    self.font_face,self.font_scale,color_text,self.font_thickness)
                    elif chamber == 'gate1':
                        cv2.rectangle(image,
                                      (self.parameters.tunnel_mask['x0']+tunnel_offset,self.parameters.gate_y_offsets[1]),
                                      (self.parameters.tunnel_mask['x1']+tunnel_offset,self.parameters.gate_y_offsets[1]+self.parameters.gate_height),
                                      self.color_rect,rectangle_thickness)
                        cv2.putText(image,
                                    "Gate 2",
                                    (text_x,text_y),
                                    self.font_face,self.font_scale,color_text,self.font_thickness)
                    elif chamber == 'gate0':
                        cv2.rectangle(image,
                                      (self.parameters.tunnel_mask['x0']+tunnel_offset,self.parameters.gate_y_offsets[0]),
                                      (self.parameters.tunnel_mask['x1']+tunnel_offset,self.parameters.gate_y_offsets[0]+self.parameters.gate_height),
                                      self.color_rect,rectangle_thickness)
                        cv2.putText(image,
                                    "Gate 1",
                                    (text_x,text_y),
                                    self.font_face,self.font_scale,color_text,self.font_thickness)
                    else:
                        cv2.putText(image,
                                    "Unknown",
                                    (text_x,text_y),
                                    self.font_face,self.font_scale,color_text,self.font_thickness)
                except KeyError:
                    pass
                try:
                    text_x = int(tunnel_offset + 1.1*self.parameters.tunnel_width)
                    for gate in range(3):
                        if data_tunnel['gate{0}'.format(gate)] == 'open':
                            text = "G{gate}".format(gate=gate+1)
                            color = (0,255,0)
                        elif data_tunnel['gate{0}'.format(gate)] == 'close':
                            text = "G{gate}".format(gate=gate+1)
                            color = (0,0,255)
                        else:
                            text = ""
                            color = (0,0,255)
                        cv2.putText(image,
                                    text,
                                    (text_x,self.parameters.gate_y_centers[gate]),
                                    self.font_face,self.font_scale,color,self.font_thickness)
                except KeyError:
                    pass

                tunnel_x_offset = self.parameters.tunnel_x_offsets[tunnel]
                tunnel_y_offset = self.parameters.tunnel_y_offset
                tx0 = self.parameters.tunnel_mask['x0'] + tunnel_x_offset
                ty0 = self.parameters.tunnel_mask['y0'] + tunnel_y_offset
                tx1 = self.parameters.tunnel_mask['x1'] + tunnel_x_offset
                ty1 = self.parameters.tunnel_mask['y1'] + tunnel_y_offset
                image_tunnel = image[ty0:ty1,tx0:tx1]
                image[ty0:ty1,tx0:tx1] = self._annotateTunnelImage(image_tunnel,data_tunnel)

        return image

    def _annotateTunnelImage(self,image_tunnel,data_tunnel):
        cv2.line(image_tunnel,
                 (self.parameters.tunnel_mask['x0'],self.parameters.origin_y_offset_tunnel),
                 (self.parameters.tunnel_mask['x1'],self.parameters.origin_y_offset_tunnel),
                 self.color_x_axis)
        cv2.line(image_tunnel,
                 (self.parameters.origin_x_offset_tunnel,self.parameters.tunnel_mask['y0']),
                 (self.parameters.origin_x_offset_tunnel,self.parameters.tunnel_mask['y1']),
                 self.color_y_axis)
        try:
            contours = data_tunnel['contours']
            cv2.drawContours(image_tunnel,contours,-1,self.color_cont,-1)
        except KeyError:
            pass
        image_tunnel = self._drawSlopeLine(image_tunnel,data_tunnel)
        try:
            y = int(data_tunnel['blob_y'])
            cv2.line(image_tunnel,(self.parameters.tunnel_mask['x0'],y),(self.parameters.tunnel_mask['x1'],y),self.color_line)
        except KeyError:
            pass
        return image_tunnel

    def _drawSlopeLine(self,image_tunnel,data_tunnel):
        try:
            x0 = data_tunnel['blob_x']
            y0 = data_tunnel['blob_y']
            slope = data_tunnel['blob_slope']
            ecc = data_tunnel['blob_ecc']
            if (not math.isnan(slope)) and (self.parameters.min_ecc < ecc):
                L = self.line_slope_length/2
                delta_x = math.sqrt(math.pow(L,2)/(math.pow(slope,2)+1))
                delta_y = delta_x*slope
                x1 = x0 - delta_x
                y1 = y0 - delta_y
                x2 = x0 + delta_x
                y2 = y0 + delta_y
                cv2.line(image_tunnel,(int(x0),int(y0)),(int(x1),int(y1)),self.color_line_slope)
                cv2.line(image_tunnel,(int(x0),int(y0)),(int(x2),int(y2)),self.color_line_slope)
        except KeyError:
            pass
        return image_tunnel
