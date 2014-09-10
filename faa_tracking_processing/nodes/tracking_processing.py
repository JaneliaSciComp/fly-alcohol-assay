#!/usr/bin/env python
import roslib
roslib.load_manifest('faa_tracking_processing')
import rospy

from faa_tracking_processing.msg import EnteredChamber

from faa_image_processing.msg import TrackingData


class TrackingProcessor(object):

  def __init__(self):
    self.entered_start_chamber_pub = rospy.Publisher("/faa_tracking_processing/entered_start_chamber",EnteredChamber)
    self.entered_end_chamber_pub = rospy.Publisher("/faa_tracking_processing/entered_end_chamber",EnteredChamber)
    self.entered_walkway_chamber_pub = rospy.Publisher("/faa_tracking_processing/entered_walkway_chamber",EnteredChamber)

    self.tracking_data_sub = rospy.Subscriber("/camera/tracking_data",TrackingData,self.tracking_data_callback)

    self.chambers_prev = {}

  def tracking_data_callback(self,tracking_data):
    tunnels_data = tracking_data.tunnel_data
    for tunnel in range(len(tunnels_data)):
      tunnel_data = tunnels_data[tunnel]
      chamber = tunnel_data.chamber
      try:
        chamber_prev = self.chambers_prev[tunnel]
      except KeyError:
        chamber_prev = 'unknown'
      if (chamber == 'start') and (chamber_prev != 'start'):
        self.entered_start_chamber_pub.publish(EnteredChamber(tunnel))
      elif (chamber == 'end') and (chamber_prev != 'end'):
        self.entered_end_chamber_pub.publish(EnteredChamber(tunnel))
      elif (chamber == 'walkway') and (chamber_prev != 'walkway'):
        self.entered_walkway_chamber_pub.publish(EnteredChamber(tunnel))
      self.chambers_prev[tunnel] = chamber


def main():
  rospy.init_node('faa_tracking_processing', anonymous=True)
  tp = TrackingProcessor()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
  main()

