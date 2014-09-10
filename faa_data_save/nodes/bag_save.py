#!/usr/bin/env python
import roslib
roslib.load_manifest('faa_data_save')
import rospy
import subprocess
import os
import signal

from faa_data_save.msg import SaveData


class BagSaver(object):

    def __init__(self):
        self.save_sub = rospy.Subscriber('/faa_data_save/save_data', SaveData, self.save_data_callback)

    def save_data_callback(self,data):
        if data.saving:
            trial_path = rospy.get_param('/faa_experiment/trial_path')
            experiment_name = rospy.get_param('/faa_experiment/experiment_name')
            trial_name = rospy.get_param('/faa_experiment/trial_name')
            time_str = rospy.get_param('/faa_experiment/trial_start_time')
            file_name = "Bag-" + experiment_name + '-' + trial_name + '-' + time_str + '.bag'
            file_name_backup = "Bag-" + experiment_name + '-' + trial_name + '-' + time_str + '.orig.bag'
            self.bag_path = os.path.join(trial_path,file_name)
            self.bag_path_backup = os.path.join(trial_path,file_name_backup)
            record_topics = "/camera/data_image /camera/tracking_data"
            call_string = 'rosbag record -b 0 -O {bag_path} {topics}'.format(bag_path=self.bag_path,
                                                                             topics=record_topics)
            subprocess.Popen(call_string,shell=True)
        else:
            self.find_record_pids()
            self.kill_record_pids()
            # self.compress_bag_file()

    def find_record_pids(self):
        p_pid = subprocess.Popen('pidof record',shell=True,stdout=subprocess.PIPE)
        out = p_pid.stdout.readlines()
        # rospy.logwarn("out = %s" % (str(out)))
        if 0 < len(out):
            p_list_str = out[0].rsplit()
            self.pid = [int(s) for s in p_list_str]
            # rospy.logwarn("pid_list = %s" % (str(self.pid)))
        else:
            self.pid = None

    def kill_record_pids(self):
        if self.pid is not None:
            for p in self.pid:
                os.kill(p,signal.SIGINT)

    def compress_bag_file(self):
        tries = 0
        while not os.path.exists(self.bag_path):
            rospy.sleep(1)
            tries += 1
            if tries > 60:
                return
        subprocess.check_call('rosbag compress ' + self.bag_path,shell=True)
        tries = 0
        while not os.path.exists(self.bag_path_backup):
            rospy.sleep(1)
            tries += 1
            if tries > 60:
                return
        subprocess.check_call('rm ' + self.bag_path_backup,shell=True)


def main():
    rospy.init_node('faa_bag_save', anonymous=True)
    bs = BagSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main()

