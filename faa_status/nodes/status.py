#!/usr/bin/env python
import roslib; roslib.load_manifest('faa_status')
import rospy

from faa_status.msg import TunnelsEnabled
from faa_image_processing.srv import SetArrayParameter

class StatusListener(object):
    def __init__(self):
        rospy.init_node('faa_status')
        rospy.Subscriber("/faa_status/tunnels_enabled", TunnelsEnabled, self.callback)
        rospy.wait_for_service('/faa_image_processing/set_array_parameter')
        self.set_image_array_parameter = rospy.ServiceProxy('/faa_image_processing/set_array_parameter',SetArrayParameter)

    def callback(self,data):
        rospy.set_param("/faa_experiment/tunnels_enabled",data.tunnels_enabled)
        try:
            self.set_image_array_parameter('tunnels_enabled', data.tunnels_enabled)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def listener(self):
        rospy.spin()


if __name__ == '__main__':
    sl = StatusListener()
    sl.listener()
