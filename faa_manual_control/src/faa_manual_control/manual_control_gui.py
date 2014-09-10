import sys
from PyQt4 import QtCore
from PyQt4 import QtGui
import functools
from manual_control_ui import Ui_MainWindow

NUM_TUNNELS = 6
NUM_GATES_PER_TUNNEL = 3
NUM_LEDS_PER_TUNNEL = 2
Open = True
Close = False
On = True
Off = False
ODOR_0 = 0
ODOR_1 = 1
ETHANOL = 2
GATE_LED_DIGITAL_TAB_INDEX = 0
GATE_LED_ANALOG_TAB_INDEX = 1
OLFACTOMETER_TAB_INDEX = 2
HIGH_CURRENT_LED_TAB_INDEX = 3

class ManualControlMainWindow(QtGui.QMainWindow,Ui_MainWindow):

    def __init__(self,parent=None):
        super(ManualControlMainWindow,self).__init__(parent)
        self.setupUi(self)
        self.makeCheckBoxArrays()
        self.groupRadioButtons()
        self.connectActions()

    def initialize(self):
        self.tabWidget.setCurrentIndex(GATE_LED_DIGITAL_TAB_INDEX)

        self.gateLedDigitalControlTab.setEnabled(True)
        self.closeAllGates()
        self.turnOffAllLeds()
        self.initializeGateCheckBoxes(Close)
        self.initializeLedCheckBoxes(Off)

        self.gateLedAnalogControlTab.setEnabled(True)
        self.analogGateTunnel = NUM_TUNNELS
        self.analogGate = NUM_GATES_PER_TUNNEL
        self.analogLedTunnel = NUM_TUNNELS
        self.analogLed = NUM_LEDS_PER_TUNNEL
        self.updateDeviceInfoLabels()

        self.olfactometersControlTab.setEnabled(True)
        self.updateMfcValues()

        self.highCurrentLedControlTab.setEnabled(True)
        self.pwmValues = {'percent_capacity':20,
                          'duration_on':500,
                          'duration_off':500}
        self.percentCapacityLabel.setNum(self.pwmValues['percent_capacity'])
        self.percentCapacityHorizontalSlider.setValue(self.pwmValues['percent_capacity'])
        self.durationOnLabel.setNum(self.pwmValues['duration_on'])
        self.durationOnHorizontalSlider.setValue(self.pwmValues['duration_on'])
        self.durationOffLabel.setNum(self.pwmValues['duration_off'])
        self.durationOffHorizontalSlider.setValue(self.pwmValues['duration_off'])

    def groupRadioButtons(self):
        self.ethanol_valve_group = QtGui.QButtonGroup()
        self.ethanol_valve_group.addButton(self.ethanolValve0RadioButton)
        self.ethanol_valve_group.addButton(self.ethanolValve1RadioButton)
        self.ethanol_valve_group.addButton(self.ethanolValve2RadioButton)
        self.ethanol_valve_group.addButton(self.ethanolValve3RadioButton)
        self.ethanol_valve_group.addButton(self.ethanolValve4RadioButton)
        self.ethanol_valve_group.addButton(self.ethanolValve5RadioButton)
        self.ethanol_valve_group.addButton(self.ethanolValve6RadioButton)
        self.ethanol_valve_group.addButton(self.ethanolValve7RadioButton)
        self.ethanol_valve_group.setExclusive(False)
        self.ethanolValve0RadioButton.setChecked(False)
        self.ethanolValve1RadioButton.setChecked(False)
        self.ethanolValve2RadioButton.setChecked(False)
        self.ethanolValve3RadioButton.setChecked(False)
        self.ethanolValve4RadioButton.setChecked(False)
        self.ethanolValve5RadioButton.setChecked(False)
        self.ethanolValve6RadioButton.setChecked(False)
        self.ethanolValve7RadioButton.setChecked(False)
        self.ethanol_valve_group.setExclusive(True)

        self.odor0_valve_group = QtGui.QButtonGroup()
        self.odor0_valve_group.addButton(self.odor0Valve0RadioButton)
        self.odor0_valve_group.addButton(self.odor0Valve1RadioButton)
        self.odor0_valve_group.addButton(self.odor0Valve2RadioButton)
        self.odor0_valve_group.addButton(self.odor0Valve3RadioButton)
        self.odor0_valve_group.addButton(self.odor0Valve4RadioButton)
        self.odor0_valve_group.addButton(self.odor0Valve5RadioButton)
        self.odor0_valve_group.addButton(self.odor0Valve6RadioButton)
        self.odor0_valve_group.addButton(self.odor0Valve7RadioButton)
        self.odor0_valve_group.setExclusive(False)
        self.odor0Valve0RadioButton.setChecked(False)
        self.odor0Valve1RadioButton.setChecked(False)
        self.odor0Valve2RadioButton.setChecked(False)
        self.odor0Valve3RadioButton.setChecked(False)
        self.odor0Valve4RadioButton.setChecked(False)
        self.odor0Valve5RadioButton.setChecked(False)
        self.odor0Valve6RadioButton.setChecked(False)
        self.odor0Valve7RadioButton.setChecked(False)
        self.odor0_valve_group.setExclusive(True)

        self.odor1_valve_group = QtGui.QButtonGroup()
        self.odor1_valve_group.addButton(self.odor1Valve0RadioButton)
        self.odor1_valve_group.addButton(self.odor1Valve1RadioButton)
        self.odor1_valve_group.addButton(self.odor1Valve2RadioButton)
        self.odor1_valve_group.addButton(self.odor1Valve3RadioButton)
        self.odor1_valve_group.addButton(self.odor1Valve4RadioButton)
        self.odor1_valve_group.addButton(self.odor1Valve5RadioButton)
        self.odor1_valve_group.addButton(self.odor1Valve6RadioButton)
        self.odor1_valve_group.addButton(self.odor1Valve7RadioButton)
        self.odor1_valve_group.setExclusive(False)
        self.odor1Valve0RadioButton.setChecked(False)
        self.odor1Valve1RadioButton.setChecked(False)
        self.odor1Valve2RadioButton.setChecked(False)
        self.odor1Valve3RadioButton.setChecked(False)
        self.odor1Valve4RadioButton.setChecked(False)
        self.odor1Valve5RadioButton.setChecked(False)
        self.odor1Valve6RadioButton.setChecked(False)
        self.odor1Valve7RadioButton.setChecked(False)
        self.odor1_valve_group.setExclusive(True)

    def makeCheckBoxArrays(self):
        self.t0lxCheckBox = [self.t0l0CheckBox,self.t0l1CheckBox,self.t0laCheckBox]
        self.t1lxCheckBox = [self.t1l0CheckBox,self.t1l1CheckBox,self.t1laCheckBox]
        self.t2lxCheckBox = [self.t2l0CheckBox,self.t2l1CheckBox,self.t2laCheckBox]
        self.t3lxCheckBox = [self.t3l0CheckBox,self.t3l1CheckBox,self.t3laCheckBox]
        self.t4lxCheckBox = [self.t4l0CheckBox,self.t4l1CheckBox,self.t4laCheckBox]
        self.t5lxCheckBox = [self.t5l0CheckBox,self.t5l1CheckBox,self.t5laCheckBox]
        self.talxCheckBox = [self.tal0CheckBox,self.tal1CheckBox,self.talaCheckBox]
        self.txlxCheckBox = [self.t0lxCheckBox,self.t1lxCheckBox,self.t2lxCheckBox,self.t3lxCheckBox,self.t4lxCheckBox,self.t5lxCheckBox,self.talxCheckBox]

        self.t0gxCheckBox = [self.t0g0CheckBox,self.t0g1CheckBox,self.t0g2CheckBox,self.t0gaCheckBox]
        self.t1gxCheckBox = [self.t1g0CheckBox,self.t1g1CheckBox,self.t1g2CheckBox,self.t1gaCheckBox]
        self.t2gxCheckBox = [self.t2g0CheckBox,self.t2g1CheckBox,self.t2g2CheckBox,self.t2gaCheckBox]
        self.t3gxCheckBox = [self.t3g0CheckBox,self.t3g1CheckBox,self.t3g2CheckBox,self.t3gaCheckBox]
        self.t4gxCheckBox = [self.t4g0CheckBox,self.t4g1CheckBox,self.t4g2CheckBox,self.t4gaCheckBox]
        self.t5gxCheckBox = [self.t5g0CheckBox,self.t5g1CheckBox,self.t5g2CheckBox,self.t5gaCheckBox]
        self.tagxCheckBox = [self.tag0CheckBox,self.tag1CheckBox,self.tag2CheckBox,self.tagaCheckBox]
        self.txgxCheckBox = [self.t0gxCheckBox,self.t1gxCheckBox,self.t2gxCheckBox,self.t3gxCheckBox,self.t4gxCheckBox,self.t5gxCheckBox,self.tagxCheckBox]

    def connectActions(self):
        self.t0l0CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=0,led=0))
        self.t1l0CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=1,led=0))
        self.t2l0CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=2,led=0))
        self.t3l0CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=3,led=0))
        self.t4l0CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=4,led=0))
        self.t5l0CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=5,led=0))
        self.tal0CheckBox.clicked.connect(functools.partial(self.talxClicked_Callback,led=0))

        self.t0l1CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=0,led=1))
        self.t1l1CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=1,led=1))
        self.t2l1CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=2,led=1))
        self.t3l1CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=3,led=1))
        self.t4l1CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=4,led=1))
        self.t5l1CheckBox.clicked.connect(functools.partial(self.txlxClicked_Callback,tunnel=5,led=1))
        self.tal1CheckBox.clicked.connect(functools.partial(self.talxClicked_Callback,led=1))

        self.t0laCheckBox.clicked.connect(functools.partial(self.txlaClicked_Callback,tunnel=0))
        self.t1laCheckBox.clicked.connect(functools.partial(self.txlaClicked_Callback,tunnel=1))
        self.t2laCheckBox.clicked.connect(functools.partial(self.txlaClicked_Callback,tunnel=2))
        self.t3laCheckBox.clicked.connect(functools.partial(self.txlaClicked_Callback,tunnel=3))
        self.t4laCheckBox.clicked.connect(functools.partial(self.txlaClicked_Callback,tunnel=4))
        self.t5laCheckBox.clicked.connect(functools.partial(self.txlaClicked_Callback,tunnel=5))
        self.talaCheckBox.clicked.connect(self.talaClicked_Callback)

        self.t0g0CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=0,gate=0))
        self.t1g0CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=1,gate=0))
        self.t2g0CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=2,gate=0))
        self.t3g0CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=3,gate=0))
        self.t4g0CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=4,gate=0))
        self.t5g0CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=5,gate=0))
        self.tag0CheckBox.clicked.connect(functools.partial(self.tagxClicked_Callback,gate=0))

        self.t0g1CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=0,gate=1))
        self.t1g1CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=1,gate=1))
        self.t2g1CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=2,gate=1))
        self.t3g1CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=3,gate=1))
        self.t4g1CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=4,gate=1))
        self.t5g1CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=5,gate=1))
        self.tag1CheckBox.clicked.connect(functools.partial(self.tagxClicked_Callback,gate=1))

        self.t0g2CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=0,gate=2))
        self.t1g2CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=1,gate=2))
        self.t2g2CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=2,gate=2))
        self.t3g2CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=3,gate=2))
        self.t4g2CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=4,gate=2))
        self.t5g2CheckBox.clicked.connect(functools.partial(self.txgxClicked_Callback,tunnel=5,gate=2))
        self.tag2CheckBox.clicked.connect(functools.partial(self.tagxClicked_Callback,gate=2))

        self.t0gaCheckBox.clicked.connect(functools.partial(self.txgaClicked_Callback,tunnel=0))
        self.t1gaCheckBox.clicked.connect(functools.partial(self.txgaClicked_Callback,tunnel=1))
        self.t2gaCheckBox.clicked.connect(functools.partial(self.txgaClicked_Callback,tunnel=2))
        self.t3gaCheckBox.clicked.connect(functools.partial(self.txgaClicked_Callback,tunnel=3))
        self.t4gaCheckBox.clicked.connect(functools.partial(self.txgaClicked_Callback,tunnel=4))
        self.t5gaCheckBox.clicked.connect(functools.partial(self.txgaClicked_Callback,tunnel=5))
        self.tagaCheckBox.clicked.connect(self.tagaClicked_Callback)

        self.analogGateOpenPushButton.clicked.connect(self.analogGateOpenClicked_Callback)
        self.analogGateClosePushButton.clicked.connect(self.analogGateCloseClicked_Callback)
        self.analogLedOnPushButton.clicked.connect(self.analogLedOnClicked_Callback)

        self.analogGateHorizontalSlider.valueChanged.connect(self.analogGateHorizontalSliderValueChanged_Callback)
        self.analogGateSpinBox.editingFinished.connect(self.analogGateSpinBoxEditingFinished_Callback)
        self.analogGateTunnelComboBox.currentIndexChanged.connect(self.analogGateTunnelComboBoxCurrentIndexChanged_Callback)
        self.analogGateComboBox.currentIndexChanged.connect(self.analogGateComboBoxCurrentIndexChanged_Callback)

        self.analogLedHorizontalSlider.valueChanged.connect(self.analogLedHorizontalSliderValueChanged_Callback)
        self.analogLedSpinBox.editingFinished.connect(self.analogLedSpinBoxEditingFinished_Callback)
        self.analogLedTunnelComboBox.currentIndexChanged.connect(self.analogLedTunnelComboBoxCurrentIndexChanged_Callback)
        self.analogLedComboBox.currentIndexChanged.connect(self.analogLedComboBoxCurrentIndexChanged_Callback)

        self.ethanolOffPushButton.clicked.connect(functools.partial(self.valveOffClicked_Callback,olfactometer=ETHANOL))
        self.ethanolValve0RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ETHANOL,valve=0))
        self.ethanolValve1RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ETHANOL,valve=1))
        self.ethanolValve2RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ETHANOL,valve=2))
        self.ethanolValve3RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ETHANOL,valve=3))
        self.ethanolValve4RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ETHANOL,valve=4))
        self.ethanolValve5RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ETHANOL,valve=5))
        self.ethanolValve6RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ETHANOL,valve=6))
        self.ethanolValve7RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ETHANOL,valve=7))
        self.ethanolMfc0HorizontalSlider.valueChanged.connect(self.ethanolMfc0ValueChanged_Callback)
        self.ethanolMfc1HorizontalSlider.valueChanged.connect(self.ethanolMfc1ValueChanged_Callback)

        self.odor0OffPushButton.clicked.connect(functools.partial(self.valveOffClicked_Callback,olfactometer=ODOR_0))
        self.odor0Valve0RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_0,valve=0))
        self.odor0Valve1RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_0,valve=1))
        self.odor0Valve2RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_0,valve=2))
        self.odor0Valve3RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_0,valve=3))
        self.odor0Valve4RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_0,valve=4))
        self.odor0Valve5RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_0,valve=5))
        self.odor0Valve6RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_0,valve=6))
        self.odor0Valve7RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_0,valve=7))
        self.odor0Mfc0HorizontalSlider.valueChanged.connect(self.odor0Mfc0ValueChanged_Callback)
        self.odor0Mfc1HorizontalSlider.valueChanged.connect(self.odor0Mfc1ValueChanged_Callback)

        self.odor1OffPushButton.clicked.connect(functools.partial(self.valveOffClicked_Callback,olfactometer=ODOR_1))
        self.odor1Valve0RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_1,valve=0))
        self.odor1Valve1RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_1,valve=1))
        self.odor1Valve2RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_1,valve=2))
        self.odor1Valve3RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_1,valve=3))
        self.odor1Valve4RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_1,valve=4))
        self.odor1Valve5RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_1,valve=5))
        self.odor1Valve6RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_1,valve=6))
        self.odor1Valve7RadioButton.clicked.connect(functools.partial(self.valveClicked_Callback,olfactometer=ODOR_1,valve=7))
        self.odor1Mfc0HorizontalSlider.valueChanged.connect(self.odor1Mfc0ValueChanged_Callback)
        self.odor1Mfc1HorizontalSlider.valueChanged.connect(self.odor1Mfc1ValueChanged_Callback)

        self.percentCapacityHorizontalSlider.valueChanged.connect(self.percentCapacityValueChanged_Callback)
        self.durationOnHorizontalSlider.valueChanged.connect(self.durationOnValueChanged_Callback)
        self.durationOffHorizontalSlider.valueChanged.connect(self.durationOffValueChanged_Callback)
        self.startPwmPushButton.clicked.connect(self.startPwmClicked_Callback)
        self.stopPwmPushButton.clicked.connect(self.stopPwmClicked_Callback)

    def closeEvent(self, event):
        pass

    def updateDeviceInfoLabels(self):
        dev_info = self.get_pwm_controller_info()
        self.gateOpenValueLabel.setNum(dev_info.gate_open_servo_angle)
        self.gateCloseValueLabel.setNum(dev_info.gate_close_servo_angle)
        self.ledOnValueLabel.setNum(dev_info.led_on_duty_cycle)
        self.ledOffValueLabel.setNum(0)
        self.gateOpenValueLabel_2.setNum(dev_info.gate_open_servo_angle)
        self.gateCloseValueLabel_2.setNum(dev_info.gate_close_servo_angle)
        self.ledOnValueLabel_2.setNum(dev_info.led_on_duty_cycle)

    def analogGateOpenClicked_Callback(self):
        angle = self.analogGateSpinBox.value()
        self.setGateOpenServoAngle(angle)
        self.updateDeviceInfoLabels()

    def analogGateCloseClicked_Callback(self):
        angle = self.analogGateSpinBox.value()
        self.setGateCloseServoAngle(angle)
        self.updateDeviceInfoLabels()

    def analogLedOnClicked_Callback(self):
        duty_cycle = self.analogLedSpinBox.value()
        self.setLedOnDutyCycle(duty_cycle)
        self.updateDeviceInfoLabels()

    def main(self):
        self.show()

    def isCheckBoxOn(self,checkBox):
        return checkBox.isChecked() == On

    def initializeLedCheckBoxes(self,state):
        for tunnel in range(0,NUM_TUNNELS+1):
            for led in range(0,NUM_LEDS_PER_TUNNEL+1):
                self.txlxCheckBox[tunnel][led].setChecked(state)

    def talaClicked_Callback(self):
        if self.isCheckBoxOn(self.talaCheckBox):
            self.turnOnAllLeds()
            for tunnel in range(0,NUM_TUNNELS+1):
                for led in range(0,NUM_LEDS_PER_TUNNEL+1):
                    self.txlxCheckBox[tunnel][led].setChecked(On)
        else:
            self.turnOffAllLeds()
            for tunnel in range(0,NUM_TUNNELS+1):
                for led in range(0,NUM_LEDS_PER_TUNNEL+1):
                    self.txlxCheckBox[tunnel][led].setChecked(Off)

    def talxClicked_Callback(self,led):
        checkBox = self.txlxCheckBox[NUM_TUNNELS][led]
        if self.isCheckBoxOn(checkBox):
            self.turnOnAllLedsByLed(led)
            for tunnel in range(0,NUM_TUNNELS):
                self.txlxCheckBox[tunnel][led].setChecked(On)
        else:
            self.turnOffAllLedsByLed(led)
            for tunnel in range(0,NUM_TUNNELS):
                self.txlxCheckBox[tunnel][led].setChecked(Off)
        for tunnel in range(0,NUM_TUNNELS):
            self.txlaUpdate(tunnel)

    def txlaClicked_Callback(self,tunnel):
        checkBox = self.txlxCheckBox[tunnel][NUM_LEDS_PER_TUNNEL]
        if self.isCheckBoxOn(checkBox):
            self.turnOnAllLedsByTunnel(tunnel)
            for led in range(0,NUM_LEDS_PER_TUNNEL):
                self.txlxCheckBox[tunnel][led].setChecked(On)
        else:
            self.turnOffAllLedsByTunnel(tunnel)
            for led in range(0,NUM_LEDS_PER_TUNNEL):
                self.txlxCheckBox[tunnel][led].setChecked(Off)
        for led in range(0,NUM_LEDS_PER_TUNNEL):
            self.talxUpdate(led)

    def talaUpdate(self):
        flag = True
        for tunnel in range(0,NUM_TUNNELS):
            flag = flag and self.txlxCheckBox[tunnel][NUM_LEDS_PER_TUNNEL].isChecked()
        for led in range(0,NUM_LEDS_PER_TUNNEL):
            flag = flag and self.txlxCheckBox[NUM_LEDS_PER_TUNNEL][led].isChecked()
        self.txlxCheckBox[NUM_TUNNELS][NUM_LEDS_PER_TUNNEL].setChecked(flag)

    def txlaUpdate(self,tunnel):
        flag = True
        for led in range(0,NUM_LEDS_PER_TUNNEL):
            flag = flag and self.txlxCheckBox[tunnel][led].isChecked()
        self.txlxCheckBox[tunnel][NUM_LEDS_PER_TUNNEL].setChecked(flag)
        self.talaUpdate()

    def talxUpdate(self,led):
        flag = True
        for tunnel in range(0,NUM_TUNNELS):
            flag = flag and self.txlxCheckBox[tunnel][led].isChecked()
        self.txlxCheckBox[NUM_TUNNELS][led].setChecked(flag)
        self.talaUpdate()

    def txlxClicked_Callback(self,tunnel,led):
        checkBox = self.txlxCheckBox[tunnel][led]
        if self.isCheckBoxOn(checkBox):
            self.turnOnLed(tunnel,led)
        else:
            self.turnOffLed(tunnel,led)
        self.txlaUpdate(led)
        self.talxUpdate(led)

    def isCheckBoxOpen(self,checkBox):
        return checkBox.isChecked() == Open

    def initializeGateCheckBoxes(self,state):
        for tunnel in range(0,NUM_TUNNELS+1):
            for gate in range(0,NUM_GATES_PER_TUNNEL+1):
                self.txgxCheckBox[tunnel][gate].setChecked(state)

    def tagaClicked_Callback(self):
        if self.isCheckBoxOpen(self.tagaCheckBox):
            self.openAllGates()
            for tunnel in range(0,NUM_TUNNELS+1):
                for gate in range(0,NUM_GATES_PER_TUNNEL+1):
                    self.txgxCheckBox[tunnel][gate].setChecked(Open)
        else:
            self.closeAllGates()
            for tunnel in range(0,NUM_TUNNELS+1):
                for gate in range(0,NUM_GATES_PER_TUNNEL+1):
                    self.txgxCheckBox[tunnel][gate].setChecked(Close)

    def tagaUpdate(self):
        flag = True
        for tunnel in range(0,NUM_TUNNELS):
            flag = flag and self.txgxCheckBox[tunnel][NUM_GATES_PER_TUNNEL].isChecked()
        for gate in range(0,NUM_GATES_PER_TUNNEL):
            flag = flag and self.txgxCheckBox[NUM_GATES_PER_TUNNEL][gate].isChecked()
        self.txgxCheckBox[NUM_TUNNELS][NUM_GATES_PER_TUNNEL].setChecked(flag)

    def tagxUpdate(self,gate):
        flag = True
        for tunnel in range(0,NUM_TUNNELS):
            flag = flag and self.txgxCheckBox[tunnel][gate].isChecked()
        self.txgxCheckBox[NUM_TUNNELS][gate].setChecked(flag)
        self.tagaUpdate()

    def txgaUpdate(self,tunnel):
        flag = True
        for gate in range(0,NUM_GATES_PER_TUNNEL):
            flag = flag and self.txgxCheckBox[tunnel][gate].isChecked()
        self.txgxCheckBox[tunnel][NUM_GATES_PER_TUNNEL].setChecked(flag)
        self.tagaUpdate()

    def tagxClicked_Callback(self,gate):
        checkBox = self.txgxCheckBox[NUM_TUNNELS][gate]
        if self.isCheckBoxOpen(checkBox):
            self.openAllGatesByGate(gate)
            for tunnel in range(0,NUM_TUNNELS):
                self.txgxCheckBox[tunnel][gate].setChecked(Open)
        else:
            self.closeAllGatesByGate(gate)
            for tunnel in range(0,NUM_TUNNELS):
                self.txgxCheckBox[tunnel][gate].setChecked(Close)
        for tunnel in range(0,NUM_TUNNELS):
            self.txgaUpdate(tunnel)

    def txgaClicked_Callback(self,tunnel):
        checkBox = self.txgxCheckBox[tunnel][NUM_GATES_PER_TUNNEL]
        if self.isCheckBoxOpen(checkBox):
            self.openAllGatesByTunnel(tunnel)
            for gate in range(0,NUM_GATES_PER_TUNNEL):
                self.txgxCheckBox[tunnel][gate].setChecked(Open)
        else:
            self.closeAllGatesByTunnel(tunnel)
            for gate in range(0,NUM_GATES_PER_TUNNEL):
                self.txgxCheckBox[tunnel][gate].setChecked(Close)
        for gate in range(0,NUM_GATES_PER_TUNNEL):
            self.tagxUpdate(gate)

    def txgxClicked_Callback(self,tunnel,gate):
        checkBox = self.txgxCheckBox[tunnel][gate]
        if self.isCheckBoxOpen(checkBox):
            self.openGate(tunnel,gate)
        else:
            self.closeGate(tunnel,gate)
        self.txgaUpdate(tunnel)
        self.tagxUpdate(gate)


    def updateGateServoAngle(self,angle):
        if self.tabWidget.currentIndex() == GATE_LED_ANALOG_TAB_INDEX:
            if (self.analogGateTunnel == NUM_TUNNELS) and (self.analogGate == NUM_GATES_PER_TUNNEL):
                self.setAllGatesServoAngle(angle)
            elif self.analogGateTunnel == NUM_TUNNELS:
                self.setAllGatesServoAngleByGate(self.analogGate,angle)
            elif self.analogGate == NUM_GATES_PER_TUNNEL:
                self.setAllGatesServoAngleByTunnel(self.analogGateTunnel,angle)
            else:
                self.setGateServoAngle(self.analogGateTunnel,self.analogGate,angle)

    def analogGateHorizontalSliderValueChanged_Callback(self,angle):
        self.analogGateSpinBox.setValue(angle)
        self.updateGateServoAngle(angle)

    def analogGateSpinBoxEditingFinished_Callback(self):
        angle = self.analogGateSpinBox.value()
        self.analogGateHorizontalSlider.setValue(angle)
        self.updateGateServoAngle(angle)

    def analogGateTunnelComboBoxCurrentIndexChanged_Callback(self,index):
        text = self.analogGateTunnelComboBox.currentText()
        if text == 'All':
            self.analogGateTunnel = NUM_TUNNELS
        else:
            self.analogGateTunnel = int(text)-1

    def analogGateComboBoxCurrentIndexChanged_Callback(self,index):
        text = self.analogGateComboBox.currentText()
        if text == 'All':
            self.analogGate = NUM_GATES_PER_TUNNEL
        else:
            self.analogGate = int(text)-1

    def updateLedDutyCycle(self,duty_cycle):
        if self.tabWidget.currentIndex() == GATE_LED_ANALOG_TAB_INDEX:
            if (self.analogLedTunnel == NUM_TUNNELS) and (self.analogLed == NUM_LEDS_PER_TUNNEL):
                self.setAllLedsDutyCycle(duty_cycle)
            elif self.analogLedTunnel == NUM_TUNNELS:
                self.setAllLedsDutyCycleByLed(self.analogLed,duty_cycle)
            elif self.analogLed == NUM_LEDS_PER_TUNNEL:
                self.setAllLedsDutyCycleByTunnel(self.analogLedTunnel,duty_cycle)
            else:
                self.setLedDutyCycle(self.analogLedTunnel,self.analogLed,duty_cycle)

    def analogLedHorizontalSliderValueChanged_Callback(self,duty_cycle):
        self.analogLedSpinBox.setValue(duty_cycle)
        self.updateLedDutyCycle(duty_cycle)

    def analogLedSpinBoxEditingFinished_Callback(self):
        duty_cycle = self.analogLedSpinBox.value()
        self.analogLedHorizontalSlider.setValue(duty_cycle)
        self.updateLedDutyCycle(duty_cycle)

    def analogLedTunnelComboBoxCurrentIndexChanged_Callback(self,index):
        text = self.analogLedTunnelComboBox.currentText()
        if text == 'All':
            self.analogLedTunnel = NUM_TUNNELS
        else:
            self.analogLedTunnel = int(text)-1

    def analogLedComboBoxCurrentIndexChanged_Callback(self,index):
        text = self.analogLedComboBox.currentText()
        if text == 'All':
            self.analogLed = NUM_LEDS_PER_TUNNEL
        else:
            self.analogLed = int(text)-1


    def valveOffClicked_Callback(self,olfactometer):
        self.set_odor_valves_off(olfactometer)
        if olfactometer == ETHANOL:
            self.ethanol_valve_group.setExclusive(False)
            self.ethanolValve0RadioButton.setChecked(False)
            self.ethanolValve1RadioButton.setChecked(False)
            self.ethanolValve2RadioButton.setChecked(False)
            self.ethanolValve3RadioButton.setChecked(False)
            self.ethanolValve4RadioButton.setChecked(False)
            self.ethanolValve5RadioButton.setChecked(False)
            self.ethanolValve6RadioButton.setChecked(False)
            self.ethanolValve7RadioButton.setChecked(False)
            self.ethanol_valve_group.setExclusive(True)
        elif olfactometer == ODOR_1:
            self.odor1_valve_group.setExclusive(False)
            self.odor1Valve0RadioButton.setChecked(False)
            self.odor1Valve1RadioButton.setChecked(False)
            self.odor1Valve2RadioButton.setChecked(False)
            self.odor1Valve3RadioButton.setChecked(False)
            self.odor1Valve4RadioButton.setChecked(False)
            self.odor1Valve5RadioButton.setChecked(False)
            self.odor1Valve6RadioButton.setChecked(False)
            self.odor1Valve7RadioButton.setChecked(False)
            self.odor1_valve_group.setExclusive(True)
        elif olfactometer == ODOR_0:
            self.odor0_valve_group.setExclusive(False)
            self.odor0Valve0RadioButton.setChecked(False)
            self.odor0Valve1RadioButton.setChecked(False)
            self.odor0Valve2RadioButton.setChecked(False)
            self.odor0Valve3RadioButton.setChecked(False)
            self.odor0Valve4RadioButton.setChecked(False)
            self.odor0Valve5RadioButton.setChecked(False)
            self.odor0Valve6RadioButton.setChecked(False)
            self.odor0Valve7RadioButton.setChecked(False)
            self.odor0_valve_group.setExclusive(True)

    def valveClicked_Callback(self,olfactometer,valve):
        self.set_odor_valve_on(olfactometer,valve)

    def ethanolMfc0ValueChanged_Callback(self,value):
        self.set_mfc_flow_rate(ETHANOL,0,value)
        self.ethanolMfc0ValueLabel.setNum(value)

    def ethanolMfc1ValueChanged_Callback(self,value):
        self.set_mfc_flow_rate(ETHANOL,1,value)
        self.ethanolMfc1ValueLabel.setNum(value)

    def odor0Mfc0ValueChanged_Callback(self,value):
        self.set_mfc_flow_rate(ODOR_0,0,value)
        self.odor0Mfc0ValueLabel.setNum(value)

    def odor0Mfc1ValueChanged_Callback(self,value):
        self.set_mfc_flow_rate(ODOR_0,1,value)
        self.odor0Mfc1ValueLabel.setNum(value)

    def odor1Mfc0ValueChanged_Callback(self,value):
        self.set_mfc_flow_rate(ODOR_1,0,value)
        self.odor1Mfc0ValueLabel.setNum(value)

    def odor1Mfc1ValueChanged_Callback(self,value):
        self.set_mfc_flow_rate(ODOR_1,1,value)
        self.odor1Mfc1ValueLabel.setNum(value)

    def updateMfcValues(self):
        olfactometer = ETHANOL
        mfc = 0
        value = self.get_mfc_flow_rate_setting(device=olfactometer,mfc=mfc).percent_capacity
        self.ethanolMfc0ValueLabel.setNum(value)
        self.ethanolMfc0HorizontalSlider.setValue(value)
        mfc = 1
        value = self.get_mfc_flow_rate_setting(device=olfactometer,mfc=mfc).percent_capacity
        self.ethanolMfc1ValueLabel.setNum(value)
        self.ethanolMfc1HorizontalSlider.setValue(value)

        olfactometer = ODOR_1
        mfc = 0
        value = self.get_mfc_flow_rate_setting(device=olfactometer,mfc=mfc).percent_capacity
        self.odor1Mfc0ValueLabel.setNum(value)
        self.odor1Mfc0HorizontalSlider.setValue(value)
        mfc = 1
        value = self.get_mfc_flow_rate_setting(device=olfactometer,mfc=mfc).percent_capacity
        self.odor1Mfc1ValueLabel.setNum(value)
        self.odor1Mfc1HorizontalSlider.setValue(value)

        olfactometer = ODOR_0
        mfc = 0
        value = self.get_mfc_flow_rate_setting(device=olfactometer,mfc=mfc).percent_capacity
        self.odor0Mfc0ValueLabel.setNum(value)
        self.odor0Mfc0HorizontalSlider.setValue(value)
        mfc = 1
        value = self.get_mfc_flow_rate_setting(device=olfactometer,mfc=mfc).percent_capacity
        self.odor0Mfc1ValueLabel.setNum(value)
        self.odor0Mfc1HorizontalSlider.setValue(value)

    def percentCapacityValueChanged_Callback(self,value):
        self.pwmValues['percent_capacity'] = value
        self.percentCapacityLabel.setNum(value)

    def durationOnValueChanged_Callback(self,value):
        self.pwmValues['duration_on'] = value
        self.durationOnLabel.setNum(value)

    def durationOffValueChanged_Callback(self,value):
        self.pwmValues['duration_off'] = value
        self.durationOffLabel.setNum(value)

    def startPwmClicked_Callback(self):
        self.start_current_controller_pwm(self.pwmValues['percent_capacity'],
                                          self.pwmValues['duration_on'],
                                          self.pwmValues['duration_off'])

    def stopPwmClicked_Callback(self):
        self.stop_current_controller_pwm()


def manualControlGui():
    app = QtGui.QApplication(sys.argv)
    mainWindow = ManualControlMainWindow()
    mainWindow.main()
    app.exec_()

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    manualControlGui()
