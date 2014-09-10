#!/usr/bin/env python
import roslib
roslib.load_manifest('faa_data_save')
import rospy
import threading

import os

from faa_data_save.msg import SaveData
from faa_image_processing.msg import TrackingData
from faa_utilities import FileTools

from faa_status.msg import Status


file_tools = FileTools()

class TrackingSaver(object):

    def __init__(self):
        self.lock = threading.Lock()
        self.saving = False
        self.save_sub = rospy.Subscriber('/faa_data_save/save_data', SaveData, self.save_data_callback)
        self.tracking_data_sub = rospy.Subscriber("/camera/tracking_data",TrackingData,self.tracking_data_callback)
        self.status_sub = rospy.Subscriber("/faa_status/status", Status, self.status_callback)

        self.tracking_data_fid = None
        self.tracking_data_line = '{time_secs},{time_nsecs},{status},{tunnel},{enabled},{gate0},{gate1},{gate2},{fly_x},{fly_y},{fly_angle},{chamber},{blob_x},{blob_y},{blob_area},{blob_slope},{blob_ecc}\n'
        self.status = ""

    def save_data_callback(self,data):
        if data.saving:
            self.open_tracking_data_file()
            self.saving = data.saving
        else:
            self.saving = data.saving
            self.close_tracking_data_file()

    def tracking_data_callback(self,tracking_data):
        if self.saving:
            self.lock.acquire()
            self.tracking_data_write(tracking_data)
            self.lock.release()

    def status_callback(self,data):
        self.status = data.status

    def open_tracking_data_file(self):
        experiment_name = rospy.get_param('/faa_experiment/experiment_name')
        trial_name = rospy.get_param('/faa_experiment/trial_name')
        time_str = rospy.get_param('/faa_experiment/trial_start_time')
        file_name = "TrackingData-" + experiment_name + '-' + trial_name + '-' + time_str + '.txt'
        trial_path = rospy.get_param('/faa_experiment/trial_path')
        file_path = os.path.join(trial_path,file_name)
        header = self.tracking_data_line.format(time_secs='time_secs',
                                                time_nsecs='time_nsecs',
                                                status='status',
                                                tunnel='tunnel',
                                                enabled='enabled',
                                                gate0='gate0',
                                                gate1='gate1',
                                                gate2='gate2',
                                                fly_x='fly_x',
                                                fly_y='fly_y',
                                                fly_angle='fly_angle',
                                                chamber='chamber',
                                                blob_x='blob_x',
                                                blob_y='blob_y',
                                                blob_area='blob_area',
                                                blob_slope='blob_slope',
                                                blob_ecc='blob_ecc')
        self.lock.acquire()
        self.tracking_data_fid = open(file_path,'w')
        self.tracking_data_fid.write(header)
        self.lock.release()

    def close_tracking_data_file(self):
        self.lock.acquire()
        self.tracking_data_fid.close()
        self.lock.release()

    def tracking_data_write(self,tracking_data):
        tunnel_count = len(tracking_data.tunnel_data)
        for tunnel in range(tunnel_count):
            line = self.tracking_data_line.format(time_secs=tracking_data.header.stamp.secs,
                                                  time_nsecs=tracking_data.header.stamp.nsecs,
                                                  status=self.status,
                                                  tunnel=tracking_data.tunnel_data[tunnel].tunnel,
                                                  enabled=tracking_data.tunnel_data[tunnel].enabled,
                                                  gate0=tracking_data.tunnel_data[tunnel].gate0,
                                                  gate1=tracking_data.tunnel_data[tunnel].gate1,
                                                  gate2=tracking_data.tunnel_data[tunnel].gate2,
                                                  fly_x=tracking_data.tunnel_data[tunnel].fly_x,
                                                  fly_y=tracking_data.tunnel_data[tunnel].fly_y,
                                                  fly_angle=tracking_data.tunnel_data[tunnel].fly_angle,
                                                  chamber=tracking_data.tunnel_data[tunnel].chamber,
                                                  blob_x=tracking_data.tunnel_data[tunnel].blob_x,
                                                  blob_y=tracking_data.tunnel_data[tunnel].blob_y,
                                                  blob_area=tracking_data.tunnel_data[tunnel].blob_area,
                                                  blob_slope=tracking_data.tunnel_data[tunnel].blob_slope,
                                                  blob_ecc=tracking_data.tunnel_data[tunnel].blob_ecc)
            self.tracking_data_fid.write(line)

def main():
    rospy.init_node('faa_tracking_save', anonymous=True)
    ts = TrackingSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main()

