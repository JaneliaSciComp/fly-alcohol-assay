#! /usr/bin/env python

import roslib; roslib.load_manifest('faa_actionlib')
import rospy
import os

import actionlib

import faa_actionlib.msg
from faa_status.msg import Status,Duration

from faa_actuation.srv import UpdateGate
from faa_actuation.srv import UpdateLed
from faa_actuation.srv import SetOdorValveOn
from faa_actuation.srv import SetOdorValvesOff
from faa_actuation.srv import StartCurrentControllerPwm, StartCurrentControllerPwmResponse
from faa_actuation.srv import StopCurrentControllerPwm, StopCurrentControllerPwmResponse

from faa_image_processing.srv import SaveImage
from faa_image_processing.srv import SetStatus
from faa_image_processing.srv import SetTracking


olfactometerOdor0 = 0
olfactometerOdor1 = 1
olfactometerEthanol = 2
vialDummy = 0
vialOdor0 = vialDummy
vialOdor1 = vialDummy

def turn_off_leds():
    service = '/faa_actuation/update_led'
    rospy.wait_for_service(service)
    try:
        ul = rospy.ServiceProxy(service,UpdateLed)
        resp = ul('all','all','off')
        return resp.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def turn_on_leds():
    service = '/faa_actuation/update_led'
    rospy.wait_for_service(service)
    try:
        ul = rospy.ServiceProxy(service,UpdateLed)
        resp = ul('all','0','on')
        return resp.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def update_all_gates(angle):
    service = '/faa_actuation/update_gate'
    rospy.wait_for_service(service)
    try:
        ug = rospy.ServiceProxy(service,UpdateGate)
        resp = ug('all','all',angle)
        return resp.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def update_gate(tunnel,gate,angle):
    service = '/faa_actuation/update_gate'
    rospy.wait_for_service(service)
    try:
        ug = rospy.ServiceProxy(service,UpdateGate)
        resp = ug(tunnel,gate,angle)
        return resp.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def save_background_image(filename):
    service = '/faa_image_processing/save_background_image'
    rospy.wait_for_service(service)
    try:
        sbi = rospy.ServiceProxy(service,SaveImage)
        resp = sbi(filename)
        return resp.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def set_odor_valve_on(device,valve):
    service = '/faa_actuation/set_odor_valve_on'
    rospy.wait_for_service(service)
    try:
        sovo = rospy.ServiceProxy(service,SetOdorValveOn)
        resp = sovo(device,valve)
        return resp.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def set_odor_valves_off(device):
    service = '/faa_actuation/set_odor_valves_off'
    rospy.wait_for_service(service)
    try:
        sovso = rospy.ServiceProxy(service,SetOdorValvesOff)
        resp = sovso(device)
        return resp.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def start_current_controller_pwm(percent_capacity,duration_on,duration_off):
    service = '/faa_actuation/start_current_controller_pwm'
    rospy.wait_for_service(service)
    try:
        startccp = rospy.ServiceProxy(service,StartCurrentControllerPwm)
        resp = startccp(percent_capacity,duration_on,duration_off)
        return resp.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def stop_current_controller_pwm():
    service = '/faa_actuation/stop_current_controller_pwm'
    rospy.wait_for_service(service)
    try:
        stopccp = rospy.ServiceProxy(service,StopCurrentControllerPwm)
        resp = stopccp()
        return resp.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def set_tracking(tracking):
    service = '/faa_image_processing/set_tracking'
    rospy.wait_for_service(service)
    try:
        st = rospy.ServiceProxy(service,SetTracking)
        resp = st(tracking)
        return resp.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

class StatusUpdater(object):
    def __init__(self):
        self.pub_status = rospy.Publisher('/faa_status/status',Status)
        self.pub_duration = rospy.Publisher('/faa_status/duration',Duration)
        self.service_status_image = '/faa_image_processing/set_status'
        rospy.wait_for_service(self.service_status_image)

    def set_status_experiment(self,status):
        self.pub_status.publish(Status(status))

    def set_status_duration(self,duration):
        self.pub_duration.publish(Duration(duration))

    def set_status_image(self,status):
        try:
            ss = rospy.ServiceProxy(self.service_status_image,SetStatus)
            resp = ss(status)
            return resp.status
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


class SaveBackgroundImagesAction(object):
    # create messages that are used to publish feedback/result
    _feedback = faa_actionlib.msg.EmptyFeedback()
    _result   = faa_actionlib.msg.EmptyResult()

    def __init__(self):
        self._action_name = 'save_background_images'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                faa_actionlib.msg.EmptyAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running SaveBackgroundImagesAction')
        status_updater.set_status_experiment('Setup')
        status_updater.set_status_image('Setup')

        experiment_path = rospy.get_param('/faa_experiment/experiment_path')
        reusing_bg_images = rospy.get_param('/faa_actionlib/faa_actionlib_server/reusing_bg_images')
        if not reusing_bg_images:
            update_all_gates('close')
            rospy.sleep(3)
        save_background_image(os.path.join(experiment_path,'bg_gates_closed.png'))
        if not reusing_bg_images:
            turn_off_leds()
            update_all_gates('open')
            rospy.sleep(3)
        save_background_image(os.path.join(experiment_path,'bg_gates_opened.png'))

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class LoadFliesAction(object):
    # create messages that are used to publish feedback/result
    _feedback = faa_actionlib.msg.EmptyFeedback()
    _result   = faa_actionlib.msg.EmptyResult()

    def __init__(self):
        self._action_name = 'load_flies'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                faa_actionlib.msg.EmptyAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running LoadFliesAction')
        status_updater.set_status_image("Load")
        status_updater.set_status_experiment("Load")

        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor0,vialDummy))
        set_odor_valve_on(olfactometerOdor0,vialDummy)
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor1,vialDummy))
        set_odor_valve_on(olfactometerOdor1,vialDummy)
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerEthanol,vialDummy))
        set_odor_valve_on(olfactometerEthanol,vialDummy)
        rospy.loginfo('update_all_gates("close")')
        update_all_gates('close')
        rospy.loginfo('turn_off_leds()')
        turn_off_leds()

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class AcclimateFliesAction(object):
    # create messages that are used to publish feedback/result
    _feedback = faa_actionlib.msg.EmptyFeedback()
    _result   = faa_actionlib.msg.EmptyResult()

    def __init__(self):
        self._action_name = 'acclimate_flies'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                faa_actionlib.msg.EmptyAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running AcclimateFliesAction')
        acclimation_duration = rospy.get_param('/faa_experiment/acclimation_duration')
        if acclimation_duration > 0:
            status_updater.set_status_image("Acclimate")
            status_updater.set_status_experiment("Acclimate")
            status_updater.set_status_duration(acclimation_duration)

            rospy.loginfo('acclimating for {0} seconds'.format(acclimation_duration))
            rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor0,vialDummy))
            set_odor_valve_on(olfactometerOdor0,vialDummy)
            rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor1,vialDummy))
            set_odor_valve_on(olfactometerOdor1,vialDummy)
            rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerEthanol,vialDummy))
            set_odor_valve_on(olfactometerEthanol,vialDummy)
            rospy.loginfo('update_gate("all","0","open")')
            update_gate('all','0','open')
            rospy.loginfo('turn_off_leds()')
            turn_off_leds()
            set_tracking(True)
            rospy.sleep(acclimation_duration)
        else:
            rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor0,vialDummy))
            set_odor_valve_on(olfactometerOdor0,vialDummy)
            rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor1,vialDummy))
            set_odor_valve_on(olfactometerOdor1,vialDummy)
            rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerEthanol,vialDummy))
            set_odor_valve_on(olfactometerEthanol,vialDummy)
            rospy.loginfo('turn_off_leds()')
            turn_off_leds()
            set_tracking(True)

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class WalkToStartAction(object):
    # create messages that are used to publish feedback/result
    _feedback = faa_actionlib.msg.EmptyFeedback()
    _result   = faa_actionlib.msg.EmptyResult()

    def __init__(self):
        self._action_name = 'walk_to_start'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                faa_actionlib.msg.EmptyAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running WalkToStartAction')
        status_updater.set_status_image("Walk To Start")

        rospy.loginfo('turn_on_leds()')
        turn_on_leds()
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor0,vialDummy))
        set_odor_valve_on(olfactometerOdor0,vialDummy)
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor1,vialDummy))
        set_odor_valve_on(olfactometerOdor1,vialDummy)
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerEthanol,vialDummy))
        set_odor_valve_on(olfactometerEthanol,vialDummy)
        rospy.loginfo('update_gate("all","0","open")')
        update_gate('all','0','open')
        pretrial = rospy.get_param('/faa_experiment/pretrial')
        if not pretrial:
            rospy.loginfo('update_gate("all","1","open")')
            update_gate('all','1','open')

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class WaitInStartAction(object):
    # create messages that are used to publish feedback/result
    _feedback = faa_actionlib.msg.EmptyFeedback()
    _result   = faa_actionlib.msg.EmptyResult()

    def __init__(self):
        self._action_name = 'wait_in_start'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                faa_actionlib.msg.EmptyAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running WaitInStartAction')
        start_chamber_duration = rospy.get_param('/faa_experiment/start_chamber_duration')
        status_updater.set_status_image("Wait in Start")
        status_updater.set_status_experiment("Wait in Start")

        rospy.loginfo('waiting in start chambers for {0} seconds'.format(start_chamber_duration))
        rospy.loginfo('turn_off_leds()')
        turn_off_leds()
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor0,vialDummy))
        set_odor_valve_on(olfactometerOdor0,vialDummy)
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor1,vialDummy))
        set_odor_valve_on(olfactometerOdor1,vialDummy)
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerEthanol,vialDummy))
        set_odor_valve_on(olfactometerEthanol,vialDummy)
        pretrial = rospy.get_param('/faa_experiment/pretrial')
        if not pretrial:
            status_updater.set_status_duration(start_chamber_duration)
            rospy.sleep(start_chamber_duration)
        else:
            start_chamber_duration_pretrial = rospy.get_param('/faa_experiment/start_chamber_duration_pretrial')
            status_updater.set_status_duration(start_chamber_duration_pretrial)
            rospy.sleep(start_chamber_duration_pretrial)

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class WalkToEndAction(object):
    # create messages that are used to publish feedback/result
    _feedback = faa_actionlib.msg.EmptyFeedback()
    _result   = faa_actionlib.msg.EmptyResult()

    def __init__(self):
        self._action_name = 'walk_to_end'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                faa_actionlib.msg.EmptyAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running WalkToEndAction')
        status_updater.set_status_image("Walk To End")

        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor0,vialDummy))
        set_odor_valve_on(olfactometerOdor0,vialDummy)
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor1,vialDummy))
        set_odor_valve_on(olfactometerOdor1,vialDummy)
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerEthanol,vialDummy))
        set_odor_valve_on(olfactometerEthanol,vialDummy)
        rospy.loginfo('update_gate("all","0","open")')
        update_gate('all','0','open')
        rospy.loginfo('update_gate("all","1","open")')
        update_gate('all','1','open')

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class WaitInEndAction(object):
    # create messages that are used to publish feedback/result
    _feedback = faa_actionlib.msg.EmptyFeedback()
    _result   = faa_actionlib.msg.EmptyResult()

    def __init__(self):
        self._action_name = 'wait_in_end'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                faa_actionlib.msg.EmptyAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running WaitInEndAction')

        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor0,vialDummy))
        set_odor_valve_on(olfactometerOdor0,vialDummy)
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerOdor1,vialDummy))
        set_odor_valve_on(olfactometerOdor1,vialDummy)

        end_chamber_params = rospy.get_param('/faa_experiment/end_chamber')

        status_updater.set_status_image("Air")
        status_updater.set_status_experiment("End Chamber Air")
        status_updater.set_status_duration(end_chamber_params['air_before_duration'])
        rospy.loginfo('air in end chambers for {0} seconds'.format(end_chamber_params['air_before_duration']))
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerEthanol,vialDummy))
        set_odor_valve_on(olfactometerEthanol,vialDummy)
        rospy.sleep(end_chamber_params['air_before_duration'])

        status_updater.set_status_image("Ethanol")
        status_updater.set_status_experiment("End Chamber Ethanol")
        status_updater.set_status_duration(end_chamber_params['ethanol_duration'])
        rospy.loginfo('ethanol in end chambers for {0} seconds'.format(end_chamber_params['ethanol_duration']))
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerEthanol,end_chamber_params['ethanol_vial']))
        set_odor_valve_on(olfactometerEthanol,end_chamber_params['ethanol_vial'])
        if end_chamber_params['lights_enabled']:
            start_current_controller_pwm(end_chamber_params['lights_percent_capacity'],
                                         end_chamber_params['lights_duration_on'],
                                         end_chamber_params['lights_duration_off'])
        rospy.sleep(end_chamber_params['ethanol_duration'])
        if end_chamber_params['lights_enabled']:
            stop_current_controller_pwm()

        status_updater.set_status_image("Air")
        status_updater.set_status_experiment("End Chamber Air")
        status_updater.set_status_duration(end_chamber_params['air_after_duration'])
        rospy.loginfo('air in end chambers for {0} seconds'.format(end_chamber_params['air_after_duration']))
        rospy.loginfo('set_odor_valve_on({0},{1})'.format(olfactometerEthanol,vialDummy))
        set_odor_valve_on(olfactometerEthanol,vialDummy)
        rospy.sleep(end_chamber_params['air_after_duration'])

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('faa_actionlib_server')
    status_updater = StatusUpdater()
    SaveBackgroundImagesAction()
    LoadFliesAction()
    AcclimateFliesAction()
    WalkToStartAction()
    WaitInStartAction()
    WalkToEndAction()
    WaitInEndAction()
    rospy.spin()
