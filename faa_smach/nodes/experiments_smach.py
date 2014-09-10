#!/usr/bin/env python
import roslib; roslib.load_manifest('faa_smach')
import rospy
import smach
import smach_ros
import os

from smach_ros import SimpleActionState
from std_msgs.msg import Empty

from faa_utilities import FileTools
from faa_actuation.srv import UpdateGate
from faa_actionlib.msg import EmptyAction
from faa_tracking_processing.msg import EnteredChamber
from faa_status.msg import Status,TunnelsToCatch,TunnelsEnabled,Trial,Duration
from faa_data_save.msg import SaveData


file_tools = FileTools()


class StatusUpdater(object):
    def __init__(self):
        self.pub_status = rospy.Publisher('/faa_status/status',Status)
        self.pub_ttc = rospy.Publisher('/faa_status/tunnels_to_catch',TunnelsToCatch)
        self.pub_duration = rospy.Publisher('/faa_status/duration',Duration)

    def set_status(self,status):
        self.pub_status.publish(Status(status))

    def set_ttc(self,tunnels_to_catch=None):
        if tunnels_to_catch is None:
            tunnels_to_catch = []
        self.pub_ttc.publish(TunnelsToCatch(tunnels_to_catch))

    def set_duration(self,duration):
        self.pub_duration.publish(Duration(duration))


class TrialUpdater(object):
    def __init__(self):
        self.pub_trial = rospy.Publisher('/faa_status/trial',Trial)
        self.pub_save_data = rospy.Publisher('/faa_data_save/save_data',SaveData)

    def update_trial(self,trial,trial_count):
        file_tools.setup_data_files(trial)
        self.pub_save_data.publish(SaveData(True))
        self.pub_trial.publish(Trial(trial,trial_count))

    def finish_trial(self):
        self.pub_save_data.publish(SaveData(False))

class SetupExperiment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        rospy.wait_for_service('/faa_actuation/update_gate')
        self.update_gate = rospy.ServiceProxy('/faa_actuation/update_gate',UpdateGate)

    def execute(self, userdata):
        rospy.loginfo('Executing state SETUP_EXPERIMENT')
        file_tools.create_experiment_path()
        rospy.set_param('/faa_experiment/pretrial',True)
        resp = self.update_gate('all','all','open')

        return 'succeeded'


class SaveExperimentParameters(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SAVE_EXPERIMENT_PARAMETERS')
        file_tools.save_all_data_params()
        return 'succeeded'


class CheckPretrial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['run_normally','no_acclimation','aborted','preempted'])
        self.checked = False

    def execute(self, userdata):
        rospy.loginfo('Executing state CHECK_PRETRIAL')
        if not self.checked:
            self.checked = True
            pretrial = rospy.get_param('/faa_experiment/pretrial')
            acclimation_duration = rospy.get_param('/faa_experiment/acclimation_duration')
            if pretrial and (acclimation_duration <= 0):
                return 'no_acclimation'

        return 'run_normally'


class CheckTrial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['run_trial','end_experiment','aborted','preempted'])
        self.trial = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state CHECK_TRIAL')
        self.trial_count = rospy.get_param('/faa_experiment/trial_count')

        if self.trial < self.trial_count:
            if self.trial == 0:
                rospy.set_param('/faa_experiment/pretrial',False)
            self.trial += 1
            trial_updater.update_trial(self.trial,self.trial_count)
            return 'run_trial'
        else:
            self.trial = 0
            return 'end_experiment'


class FinishTrial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FINISH_TRIAL')
        trial_updater.finish_trial()
        return 'succeeded'


class FinishExperiment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FINISH_EXPERIMENT')
        status_updater.set_status('Finished')
        return 'succeeded'


class CatchInChamber(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succeeded','aborted','preempted'])
        self.chamber_name = ''
        self.state_name = ''
        self.chamber_subscriber = ''
        self.status = ''
        self.chamber_gate = ''
        self.walkway_gate = ''
        self.executing = False

    def execute(self, userdata):
        rospy.logwarn('Executing state ' + self.state_name)
        self.executing = True

        rospy.wait_for_service('/faa_actuation/update_gate')
        self.update_gate = rospy.ServiceProxy('/faa_actuation/update_gate',UpdateGate)

        rospy.Subscriber(self.chamber_subscriber, EnteredChamber, self.entered_chamber_callback)
        rospy.Subscriber("/faa_status/tunnels_enabled", TunnelsEnabled, self.tunnels_enabled_callback)
        self.pub_tunnels_enabled = rospy.Publisher('/faa_status/tunnels_enabled',TunnelsEnabled)

        rospy.Subscriber('/faa_tracking_processing/entered_walkway_chamber', EnteredChamber, self.entered_walkway_callback)

        tunnels_enabled = rospy.get_param("/faa_experiment/tunnels_enabled")
        tunnel_count = len(tunnels_enabled)
        self.tunnels_to_catch = [tunnel for tunnel in range(tunnel_count) if tunnels_enabled[tunnel]]
        self.tunnels_caught = []
        status_updater.set_status(self.status)
        status_updater.set_ttc(self.tunnels_to_catch)

        if len(self.tunnels_to_catch) != 0:
            self.return_outcome = None
            self.timeout = rospy.get_param("/faa_experiment/timeout_duration")
            status_updater.set_duration(self.timeout)
            self.time_start = rospy.get_time()
        else:
            self.return_outcome = 'succeeded'
        while self.return_outcome is None:
            time_now = rospy.get_time()
            if (time_now - self.time_start) > self.timeout:
                tunnels_enabled = rospy.get_param("/faa_experiment/tunnels_enabled")
                for tunnel in self.tunnels_to_catch:
                    tunnels_enabled[tunnel] = False
                self.pub_tunnels_enabled.publish(TunnelsEnabled(tunnels_enabled))
                self.return_outcome = 'succeeded'
            rospy.sleep(0.2)
        self.executing = False
        return self.return_outcome

    def entered_chamber_callback(self,data):
        tunnel = data.tunnel
        if tunnel in self.tunnels_to_catch:
            try:
                resp = self.update_gate(str(tunnel),self.chamber_gate,'close')
                rospy.loginfo('Caught fly in {0} chamber in tunnel {1} in {2}'.format(self.chamber_name,
                                                                                      tunnel,
                                                                                      self.state_name))
                self.tunnels_to_catch.remove(tunnel)
                self.tunnels_caught.append(tunnel)
                status_updater.set_ttc(self.tunnels_to_catch)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        if len(self.tunnels_to_catch) == 0:
            self.return_outcome = 'succeeded'

    def entered_walkway_callback(self,data):
        if self.executing:
            tunnel = data.tunnel
            try:
                resp = self.update_gate(str(tunnel),self.walkway_gate,'close')
                rospy.logwarn('Caught fly in walkway in tunnel {0}, state_name = {1}, walkway_gate = {2}'.format(tunnel,self.state_name,self.walkway_gate))
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    def tunnels_enabled_callback(self,data):
        tunnels_enabled = data.tunnels_enabled
        tunnel_count = len(tunnels_enabled)
        for tunnel in range(tunnel_count):
            # remove tunnels that have been disabled
            if not tunnels_enabled[tunnel] and tunnel in self.tunnels_to_catch:
                self.tunnels_to_catch.remove(tunnel)
            # add tunnels that have be reenabled
            if tunnels_enabled[tunnel]:
                if tunnel not in self.tunnels_to_catch:
                    if tunnel not in self.tunnels_caught:
                        self.tunnels_to_catch.append(tunnel)
                        self.tunnels_to_catch.sort()
        status_updater.set_ttc(self.tunnels_to_catch)
        if len(self.tunnels_to_catch) == 0:
            self.return_outcome = 'succeeded'


class CatchInStart(CatchInChamber):
    def __init__(self):
        super(CatchInStart, self).__init__()
        self.chamber_name = 'start'
        self.state_name = 'CATCH_IN_START'
        self.chamber_subscriber = '/faa_tracking_processing/entered_start_chamber'
        self.status = "Walk To Start"
        self.chamber_gate = '0'
        self.walkway_gate = '1'

class CatchInEnd(CatchInChamber):
    def __init__(self):
        super(CatchInEnd, self).__init__()
        self.chamber_name = 'end'
        self.state_name = 'CATCH_IN_END'
        self.chamber_subscriber = '/faa_tracking_processing/entered_end_chamber'
        self.status = "Walk To End"
        self.chamber_gate = '1'
        self.walkway_gate = '2'


def monitor_cb(ud, msg):
    return False

class ExperimentsSmach(object):
    def __init__(self):

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('SETUP_EXPERIMENT', SetupExperiment(),
                                   transitions={'succeeded':'WAIT_TO_SAVE_BACKGROUND_IMAGES',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('WAIT_TO_SAVE_BACKGROUND_IMAGES',
                                   smach_ros.MonitorState("/faa_controls/save_background_images",
                                                          Empty,
                                                          monitor_cb),
                                   transitions={'invalid':'SAVE_BACKGROUND_IMAGES',
                                                'valid':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('SAVE_BACKGROUND_IMAGES',
                                   SimpleActionState('/faa_actionlib/save_background_images',
                                                     EmptyAction),
                                   transitions={'succeeded':'GET_EXPERIMENT_PARAMETERS',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('GET_EXPERIMENT_PARAMETERS',
                                   smach_ros.MonitorState("/faa_controls/parameters_initialized",
                                                          Empty,
                                                          monitor_cb),
                                   transitions={'invalid':'SAVE_EXPERIMENT_PARAMETERS',
                                                'valid':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('SAVE_EXPERIMENT_PARAMETERS', SaveExperimentParameters(),
                                   transitions={'succeeded':'WAIT_TO_LOAD_FLIES',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('WAIT_TO_LOAD_FLIES',
                                   smach_ros.MonitorState("/faa_controls/load_flies",
                                                          Empty,
                                                          monitor_cb),
                                   transitions={'invalid':'LOAD_FLIES',
                                                'valid':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('LOAD_FLIES',
                                   SimpleActionState('/faa_actionlib/load_flies',
                                                     EmptyAction),
                                   transitions={'succeeded':'WAIT_TO_RUN_EXPERIMENT',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('WAIT_TO_RUN_EXPERIMENT',
                                   smach_ros.MonitorState("/faa_controls/run_experiment",
                                                          Empty,
                                                          monitor_cb),
                                   transitions={'invalid':'ACCLIMATE_FLIES',
                                                'valid':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('ACCLIMATE_FLIES',
                                   SimpleActionState('/faa_actionlib/acclimate_flies',
                                                     EmptyAction),
                                   transitions={'succeeded':'RUN_EXPERIMENT',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            self.sm_experiment = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

            with self.sm_experiment:
                smach.StateMachine.add('CHECK_PRETRIAL', CheckPretrial(),
                                       transitions={'run_normally':'WALK_TO_START',
                                                    'no_acclimation':'CHECK_TRIAL',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('WALK_TO_START',
                                       SimpleActionState('/faa_actionlib/walk_to_start',
                                                         EmptyAction),
                                       transitions={'succeeded':'CATCH_IN_START',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('CATCH_IN_START', CatchInStart(),
                                       transitions={'succeeded':'WAIT_IN_START',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('WAIT_IN_START',
                                       SimpleActionState('/faa_actionlib/wait_in_start',
                                                         EmptyAction),
                                       transitions={'succeeded':'CHECK_TRIAL',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('CHECK_TRIAL', CheckTrial(),
                                       transitions={'run_trial':'WALK_TO_END',
                                                    'end_experiment':'succeeded',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('WALK_TO_END',
                                       SimpleActionState('/faa_actionlib/walk_to_end',
                                                         EmptyAction),
                                       transitions={'succeeded':'CATCH_IN_END',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('CATCH_IN_END', CatchInEnd(),
                                       transitions={'succeeded':'WAIT_IN_END',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('WAIT_IN_END',
                                       SimpleActionState('/faa_actionlib/wait_in_end',
                                                         EmptyAction),
                                       transitions={'succeeded':'FINISH_TRIAL',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('FINISH_TRIAL', FinishTrial(),
                                       transitions={'succeeded':'WALK_TO_START',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})

            smach.StateMachine.add('RUN_EXPERIMENT', self.sm_experiment,
                                   transitions={'succeeded':'FINISH_EXPERIMENT',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('FINISH_EXPERIMENT', FinishExperiment(),
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

    def execute(self):
        outcome = self.sm.execute()


if __name__ == '__main__':
    rospy.init_node('faa_experiments_smach')
    status_updater = StatusUpdater()
    trial_updater = TrialUpdater()
    es = ExperimentsSmach()
    sis = smach_ros.IntrospectionServer('faa_experiments', es.sm, '/SM_ROOT')
    sis.start()
    es.execute()
    sis.stop()
