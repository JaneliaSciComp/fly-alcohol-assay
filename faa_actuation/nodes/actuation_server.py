#!/usr/bin/env python
import roslib; roslib.load_manifest('faa_actuation')
import rospy
import copy
import threading
import time

from faa_actuation import Actuation
from faa_actuation.srv import UpdateLed, UpdateLedResponse
from faa_actuation.srv import UpdateGate, UpdateGateResponse
from faa_actuation.srv import SetOdorValveOn, SetOdorValveOnResponse
from faa_actuation.srv import SetOdorValvesOff, SetOdorValvesOffResponse
from faa_actuation.srv import SetMfcFlowRate, SetMfcFlowRateResponse
from faa_actuation.srv import GetMfcFlowRateSetting, GetMfcFlowRateSettingResponse
from faa_actuation.srv import GetActuationInfo, GetActuationInfoResponse
from faa_actuation.srv import GetPwmControllerInfo, GetPwmControllerInfoResponse
from faa_actuation.srv import SetPwmControllerValue, SetPwmControllerValueResponse
from faa_actuation.srv import StartCurrentControllerPwm, StartCurrentControllerPwmResponse
from faa_actuation.srv import StopCurrentControllerPwm, StopCurrentControllerPwmResponse

from faa_actuation.msg import TunnelState
from faa_actuation.msg import ActuationState


class ActuationServer(object):

    def __init__(self):
        rospy.init_node('faa_actuation')
        rospy.set_param('/faa_actuation/initialized',False)
        self.hardware = rospy.get_param('/faa_actuation/faa_actuation/hardware')
        if self.hardware:
            rospy.loginfo("usb hardware mode")
            self.actuation = Actuation()
        else:
            rospy.loginfo("no usb hardware test mode")
            time.sleep(10)
        rospy.set_param('/faa_actuation/initialized',True)
        self.lock = threading.Lock()
        self.tunnel_count = 6
        self.gate_count = 3
        self.gates_state = ['close']*self.gate_count
        self.tunnel_state = TunnelState()
        self.tunnel_state.gates_state = copy.deepcopy(self.gates_state)
        self.actuation_state = ActuationState()
        for tunnel in range(self.tunnel_count):
            self.actuation_state.tunnels_state.append(copy.deepcopy(self.tunnel_state))

        self.update_led = rospy.Service('update_led', UpdateLed, self.handle_update_led)
        self.update_gate = rospy.Service('update_gate', UpdateGate, self.handle_update_gate)
        self.get_pwm_controller_info = rospy.Service('get_pwm_controller_info', GetPwmControllerInfo, self.handle_get_pwm_controller_info)
        self.set_pwm_controller_value = rospy.Service('set_pwm_controller_value', SetPwmControllerValue, self.handle_set_pwm_controller_value)

        self.set_odor_valve_on = rospy.Service('set_odor_valve_on', SetOdorValveOn, self.handle_set_odor_valve_on)
        self.set_odor_valves_off = rospy.Service('set_odor_valves_off', SetOdorValvesOff, self.handle_set_odor_valves_off)
        self.set_mfc_flow_rate = rospy.Service('set_mfc_flow_rate', SetMfcFlowRate, self.handle_set_mfc_flow_rate)
        self.get_mfc_flow_rate_setting = rospy.Service('get_mfc_flow_rate_setting', GetMfcFlowRateSetting, self.handle_get_mfc_flow_rate_setting)
        self.state_pub = rospy.Publisher('actuation_state', ActuationState)
        # while not rospy.is_shutdown():
        #     self.lock.acquire()
        #     self.state_pub.publish(self.actuation_state)
        #     self.lock.release()
        #     rospy.sleep(0.25)
        self.update_gate_actuation_state('all','all','close')

        self.start_current_controller_pwm = rospy.Service('start_current_controller_pwm', StartCurrentControllerPwm, self.handle_start_current_controller_pwm)
        self.stop_current_controller_pwm = rospy.Service('stop_current_controller_pwm', StopCurrentControllerPwm, self.handle_stop_current_controller_pwm)

        self.main()

    def update_gate_actuation_state(self,tunnel,gate,angle):
        self.lock.acquire()
        tunnel = str(tunnel).lower()
        gate = str(gate).lower()
        angle = str(angle).lower()
        if (tunnel != 'all') and (gate != 'all'):
            tunnel = int(tunnel)
            gate = int(gate)
            self.actuation_state.tunnels_state[tunnel].gates_state[gate] = angle
        elif tunnel != 'all':
            tunnel = int(tunnel)
            for gate in range(self.gate_count):
                self.actuation_state.tunnels_state[tunnel].gates_state[gate] = angle
        elif (gate != 'all'):
            gate = int(gate)
            for tunnel in range(self.tunnel_count):
                self.actuation_state.tunnels_state[tunnel].gates_state[gate] = angle
        elif (tunnel == 'all') and (gate == 'all'):
            for tunnel in range(self.tunnel_count):
                for gate in range(self.gate_count):
                    self.actuation_state.tunnels_state[tunnel].gates_state[gate] = angle
        self.state_pub.publish(self.actuation_state)
        self.lock.release()

    def handle_update_led(self,req):
        rospy.loginfo('updateLed: tunnel={0},led={1},duty_cycle={2}'.format(req.tunnel,req.led,req.duty_cycle))
        if self.hardware and self.actuation.pwm_controller is not None:
            self.actuation.pwm_controller.updateLed(req.tunnel,req.led,req.duty_cycle)
        return UpdateLedResponse("success")

    def handle_update_gate(self,req):
        rospy.loginfo('updateGate: tunnel={0},gate={1},angle={2}'.format(req.tunnel,req.gate,req.angle))
        self.update_gate_actuation_state(req.tunnel,req.gate,req.angle)
        if self.hardware and self.actuation.pwm_controller is not None:
            self.actuation.pwm_controller.updateGate(req.tunnel,req.gate,req.angle)
        return UpdateGateResponse("success")

    def handle_get_pwm_controller_info(self,req):
        if self.hardware and self.actuation.pwm_controller is not None:
            dev_info = self.actuation.pwm_controller.getDevInfo()
            gate_open_servo_angle = dev_info['gate_open_servo_angle']
            gate_close_servo_angle = dev_info['gate_close_servo_angle']
            led_on_duty_cycle = dev_info['led_on_duty_cycle']
        else:
            gate_open_servo_angle = 15
            gate_close_servo_angle = 35
            led_on_duty_cycle = 50
        return GetPwmControllerInfoResponse(gate_open_servo_angle=gate_open_servo_angle,
                                            gate_close_servo_angle=gate_close_servo_angle,
                                            led_on_duty_cycle=led_on_duty_cycle)

    def handle_set_pwm_controller_value(self,req):
        if self.hardware and self.actuation.pwm_controller is not None:
            if req.value_name.lower() == 'open':
                self.actuation.pwm_controller.setGateOpenServoAngle(req.value_amount)
            elif req.value_name.lower() == 'close':
                self.actuation.pwm_controller.setGateCloseServoAngle(req.value_amount)
            elif req.value_name.lower() == 'on':
                self.actuation.pwm_controller.setLedOnDutyCycle(req.value_amount)
        else:
            pass
        return SetPwmControllerValueResponse("success")

    def handle_set_odor_valve_on(self,req):
        rospy.loginfo('setOdorValveOn: device={0},valve={1}'.format(req.device,req.valve))
        if self.hardware and self.actuation.olfactometers is not None:
            try:
                self.actuation.olfactometers[req.device].setOdorValveOn(req.valve)
            except IndexError:
                return SetOdorValveOnResponse("failure")
        return SetOdorValveOnResponse("success")

    def handle_set_odor_valves_off(self,req):
        rospy.loginfo('setOdorValvesOff: device={0}'.format(req.device))
        if self.hardware and self.actuation.olfactometers is not None:
            try:
                self.actuation.olfactometers[req.device].setOdorValvesOff()
            except IndexError:
                return SetOdorValvesOffResponse("failure")
        return SetOdorValvesOffResponse("success")

    def handle_set_mfc_flow_rate(self,req):
        rospy.loginfo('setMfcFlowRate: device={0},mfc={1},percent_capacity={2}'.format(req.device,req.mfc,req.percent_capacity))
        rospy.set_param('mfc_flow_rates/percent_capacity_device{0}_mfc{1}'.format(req.device,req.mfc),req.percent_capacity)
        if self.hardware and self.actuation.olfactometers is not None:
            try:
                self.actuation.olfactometers[req.device].setMfcFlowRate(req.mfc,req.percent_capacity)
            except IndexError:
                return SetMfcFlowRateResponse("failure")
        return SetMfcFlowRateResponse("success")

    def handle_get_mfc_flow_rate_setting(self,req):
        if self.hardware and self.actuation.olfactometers is not None:
            try:
                percent_capacity = self.actuation.olfactometers[req.device].getMfcFlowRateSetting(req.mfc)
            except IndexError:
                percent_capacity = (req.device + 10) + 2*(req.mfc + 2)
        else:
            percent_capacity = (req.device + 10) + 2*(req.mfc + 2)
        rospy.loginfo('updateMfcFlowRateSettings: device={0},mfc={1},percent_capacity={2}'.format(req.device,req.mfc,percent_capacity))
        rospy.set_param('mfc_flow_rates/percent_capacity_device{0}_mfc{1}'.format(req.device,req.mfc),percent_capacity)
        return GetMfcFlowRateSettingResponse(percent_capacity)

    def handle_get_actuation_info(self,req):
        if self.hardware:
            pass
        return GetActuationInfoResponse("")

    def handle_start_current_controller_pwm(self,req):
        if self.hardware and self.actuation.current_controller is not None:
            self.actuation.current_controller.startPwmAll(req.percent_capacity,req.duration_on,req.duration_off)
        return StartCurrentControllerPwmResponse("success")

    def handle_stop_current_controller_pwm(self,req):
        if self.hardware and self.actuation.current_controller is not None:
            self.actuation.current_controller.stopPwmAll()
        return StopCurrentControllerPwmResponse("success")

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    actuation_server = ActuationServer()
