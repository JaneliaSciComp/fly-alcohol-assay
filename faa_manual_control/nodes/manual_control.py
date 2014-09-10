#!/usr/bin/env python
import roslib
roslib.load_manifest('faa_manual_control')
import rospy
import sys

from PyQt4 import QtGui,QtCore

from faa_manual_control import ManualControlMainWindow
from faa_actuation.srv import UpdateGate
from faa_actuation.srv import UpdateLed
from faa_actuation.srv import GetPwmControllerInfo
from faa_actuation.srv import SetPwmControllerValue
from faa_actuation.srv import SetOdorValveOn
from faa_actuation.srv import SetOdorValvesOff
from faa_actuation.srv import GetMfcFlowRateSetting
from faa_actuation.srv import SetMfcFlowRate
from faa_actuation.srv import StartCurrentControllerPwm
from faa_actuation.srv import StopCurrentControllerPwm



class ManualControl(ManualControlMainWindow):

  def __init__(self):
    super(ManualControl,self).__init__()
    rospy.wait_for_service('/faa_actuation/update_gate')
    self.update_gate = rospy.ServiceProxy('/faa_actuation/update_gate',UpdateGate)
    rospy.wait_for_service('/faa_actuation/update_led')
    self.update_led = rospy.ServiceProxy('/faa_actuation/update_led',UpdateLed)
    rospy.wait_for_service('/faa_actuation/get_pwm_controller_info')
    self.get_pwm_controller_info = rospy.ServiceProxy('/faa_actuation/get_pwm_controller_info',GetPwmControllerInfo)
    rospy.wait_for_service('/faa_actuation/set_pwm_controller_value')
    self.set_pwm_controller_value = rospy.ServiceProxy('/faa_actuation/set_pwm_controller_value',SetPwmControllerValue)
    rospy.wait_for_service('/faa_actuation/set_odor_valve_on')
    self.set_odor_valve_on = rospy.ServiceProxy('/faa_actuation/set_odor_valve_on',SetOdorValveOn)
    rospy.wait_for_service('/faa_actuation/set_odor_valves_off')
    self.set_odor_valves_off = rospy.ServiceProxy('/faa_actuation/set_odor_valves_off',SetOdorValvesOff)
    rospy.wait_for_service('/faa_actuation/get_mfc_flow_rate_setting')
    self.get_mfc_flow_rate_setting = rospy.ServiceProxy('/faa_actuation/get_mfc_flow_rate_setting',GetMfcFlowRateSetting)
    rospy.wait_for_service('/faa_actuation/set_mfc_flow_rate')
    self.set_mfc_flow_rate = rospy.ServiceProxy('/faa_actuation/set_mfc_flow_rate',SetMfcFlowRate)
    rospy.wait_for_service('/faa_actuation/start_current_controller_pwm')
    self.start_current_controller_pwm = rospy.ServiceProxy('/faa_actuation/start_current_controller_pwm',StartCurrentControllerPwm)
    rospy.wait_for_service('/faa_actuation/stop_current_controller_pwm')
    self.stop_current_controller_pwm = rospy.ServiceProxy('/faa_actuation/stop_current_controller_pwm',StopCurrentControllerPwm)
    self.initialize()

  def openAllGates(self):
    self.update_gate('all','all','open')

  def closeAllGates(self):
    self.update_gate('all','all','close')

  def openAllGatesByTunnel(self,tunnel):
    self.update_gate(str(tunnel),'all','open')

  def closeAllGatesByTunnel(self,tunnel):
    self.update_gate(str(tunnel),'all','close')

  def openAllGatesByGate(self,gate):
    self.update_gate('all',str(gate),'open')

  def closeAllGatesByGate(self,gate):
    self.update_gate('all',str(gate),'close')

  def openGate(self,tunnel,gate):
    self.update_gate(str(tunnel),str(gate),'open')

  def closeGate(self,tunnel,gate):
    self.update_gate(str(tunnel),str(gate),'close')


  def turnOnAllLeds(self):
    self.update_led('all','all','on')

  def turnOffAllLeds(self):
    self.update_led('all','all','off')

  def turnOnAllLedsByTunnel(self,tunnel):
    self.update_led(str(tunnel),'all','on')

  def turnOffAllLedsByTunnel(self,tunnel):
    self.update_led(str(tunnel),'all','off')

  def turnOnAllLedsByLed(self,led):
    self.update_led('all',str(led),'on')

  def turnOffAllLedsByLed(self,led):
    self.update_led('all',str(led),'off')

  def turnOnLed(self,tunnel,led):
    self.update_led(str(tunnel),str(led),'on')

  def turnOffLed(self,tunnel,led):
    self.update_led(str(tunnel),str(led),'off')


  def setGateOpenServoAngle(self,angle):
    self.set_pwm_controller_value("open",angle)

  def setGateCloseServoAngle(self,angle):
    self.set_pwm_controller_value("close",angle)

  def setLedOnDutyCycle(self,duty_cycle):
    self.set_pwm_controller_value("on",duty_cycle)

  def setAllGatesServoAngle(self,angle):
    self.update_gate('all','all',str(angle))

  def setAllGatesServoAngleByTunnel(self,tunnel,angle):
    self.update_gate(str(tunnel),'all',str(angle))

  def setAllGatesServoAngleByGate(self,gate,angle):
    self.update_gate('all',str(gate),str(angle))

  def setGateServoAngle(self,tunnel,gate,angle):
    self.update_gate(str(tunnel),str(gate),str(angle))

  def setAllLedsDutyCycle(self,duty_cycle):
    self.update_led('all','all',str(duty_cycle))

  def setAllLedsDutyCycleByTunnel(self,tunnel,duty_cycle):
    self.update_led(str(tunnel),'all',str(duty_cycle))

  def setAllLedsDutyCycleByLed(self,led,duty_cycle):
    self.update_led('all',str(led),str(duty_cycle))

  def setLedDutyCycle(self,tunnel,led,duty_cycle):
    self.update_led(str(tunnel),str(led),str(duty_cycle))



# def shutdown_gui():
#   app.quit()
#   sys.exit(app.exec_())

def main():
  rospy.init_node('faa_manual_control', anonymous=True)
  # global mainWindow, app
  app = QtGui.QApplication(sys.argv)
  mainWindow = ManualControl()
  mainWindow.main()
  # rospy.on_shutdown(shutdown_gui)
  app.exec_()
  # try:
  #   app.exec_()
  # except KeyboardInterrupt:
  #   print "Shutting down"
  # sys.exit(app.exec_())
  # QtCore.QCoreApplication.instance().quit()


if __name__ == '__main__':
    main()

