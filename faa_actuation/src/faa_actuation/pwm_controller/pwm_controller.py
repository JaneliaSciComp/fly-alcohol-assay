from __future__ import print_function
import atexit
import time
import argparse

from arduino_device import ArduinoDevice, ArduinoDevices, findArduinoDevicePorts


DEBUG = False
BAUDRATE = 'default'
DEVICE_MODEL_NUMBER = 1154

class PwmController(ArduinoDevice):

    TIMEOUT = 1

    def __init__(self,*args,**kwargs):
        kwargs.update({'model_number': DEVICE_MODEL_NUMBER})
        serial_number = None
        if 'debug' not in kwargs:
            kwargs.update({'debug': DEBUG})
        if 'baudrate' not in kwargs:
            kwargs.update({'baudrate': BAUDRATE})
        elif (kwargs['baudrate'] is None) or (kwargs['baudrate'].lower() == 'default'):
            kwargs.update({'baudrate': BAUDRATE})
        if 'timeout' not in kwargs:
            kwargs.update({'timeout': self.TIMEOUT})
        if 'serial_number' in kwargs:
            serial_number = kwargs.pop('serial_number')
        if ('port' not in kwargs) or (kwargs['port'] is None):
            port =  findPwmControllerPort(baudrate=kwargs['baudrate'],
                                          serial_number=serial_number)
            kwargs.update({'port': port})
        super(PwmController,self).__init__(*args,**kwargs)
        atexit.register(self._exitPwmController)

    def _exitPwmController(self):
        pass

    def getPwmControllerInfo(self):
        controller_info = self.getArduinoDeviceInfo()
        dev_info = self.getDevInfo()
        controller_info.update(dev_info)
        return controller_info

    def updateLed(self,tunnel='all',led='all',duty_cycle='off'):
        tunnel = str(tunnel).lower()
        led = str(led).lower()
        duty_cycle = str(duty_cycle).lower()
        if self.debug:
            print('inside updateLed: tunnel={0}, led={1}, duty_cycle={2}'.format(tunnel,led,duty_cycle))
        if (tunnel != 'all') and (led != 'all'):
            tunnel = int(tunnel)
            led = int(led)
            if duty_cycle == 'on':
                self.turnOnLed(tunnel,led)
            elif duty_cycle == 'off':
                self.turnOffLed(tunnel,led)
            else:
                duty_cycle = int(duty_cycle)
                self.setLedDutyCycle(tunnel,led,duty_cycle)
        elif tunnel != 'all':
            tunnel = int(tunnel)
            if duty_cycle == 'on':
                self.turnOnAllLedsByTunnel(tunnel)
            elif duty_cycle == 'off':
                self.turnOffAllLedsByTunnel(tunnel)
            else:
                duty_cycle = int(duty_cycle)
                self.setAllLedsDutyCycleByTunnel(tunnel,duty_cycle)
        elif led != 'all':
            led = int(led)
            if duty_cycle == 'on':
                self.turnOnAllLedsByLed(led)
            elif duty_cycle == 'off':
                self.turnOffAllLedsByLed(led)
            else:
                duty_cycle = int(duty_cycle)
                self.setAllLedsDutyCycleByLed(led,duty_cycle)
        elif (tunnel == 'all') and (led == 'all'):
            if duty_cycle == 'on':
                self.turnOnAllLeds()
            elif duty_cycle == 'off':
                self.turnOffAllLeds()
            else:
                duty_cycle = int(duty_cycle)
                self.setAllLedsDutyCycle(duty_cycle)

    def updateGate(self,tunnel='all',gate='all',angle='close'):
        tunnel = str(tunnel).lower()
        gate = str(gate).lower()
        angle = str(angle).lower()
        if self.debug:
            print('inside updateGate: tunnel={0}, gate={1}, angle={2}'.format(tunnel,gate,angle))
        if (tunnel != 'all') and (gate != 'all'):
            tunnel = int(tunnel)
            gate = int(gate)
            if angle == 'open':
                self.openGate(tunnel,gate)
            elif angle == 'close':
                self.closeGate(tunnel,gate)
            else:
                angle = int(angle)
                self.setGateServoAngle(tunnel,gate,angle)
        elif tunnel != 'all':
            tunnel = int(tunnel)
            if angle == 'open':
                self.openAllGatesByTunnel(tunnel)
            elif angle == 'close':
                self.closeAllGatesByTunnel(tunnel)
            else:
                angle = int(angle)
                self.setAllGatesServoAngleByTunnel(tunnel,angle)
        elif (gate != 'all'):
            gate = int(gate)
            if angle == 'open':
                self.openAllGatesByGate(gate)
            elif angle == 'close':
                self.closeAllGatesByGate(gate)
            else:
                angle = int(angle)
                self.setAllGatesServoAngleByGate(gate,angle)
        elif (tunnel == 'all') and (gate == 'all'):
            if angle == 'open':
                self.openAllGates()
            elif angle == 'close':
                self.closeAllGates()
            else:
                angle = int(angle)
                self.setAllGatesServoAngle(angle)


# device_names example:
# [{'port':'/dev/ttyACM0',
#   'device_name':'controller0'},
#  {'serial_number':3,
#   'device_name':'controller1'}]
class PwmControllers(ArduinoDevices):

    def __init__(self,*args,**kwargs):
        if ('use_ports' not in kwargs) or (kwargs['use_ports'] is None):
            kwargs['use_ports'] = findPwmControllerPorts(*args,**kwargs)
        super(PwmControllers,self).__init__(*args,**kwargs)

    def appendDevice(self,*args,**kwargs):
        self.append(PwmController(*args,**kwargs))

    def getPwmControllerInfo(self):
        controller_info = []
        for device_index in range(len(self)):
            dev = self[device_index]
            controller_info.append(dev.getPwmControllerInfo())
        return controller_info


def findPwmControllerPorts(baudrate=None, serial_number=None, try_ports=None, debug=DEBUG):
    arduino_device_ports = findArduinoDevicePorts(baudrate=baudrate,
                                                  model_number=DEVICE_MODEL_NUMBER,
                                                  serial_number=serial_number,
                                                  try_ports=try_ports,
                                                  debug=debug)

    if type(serial_number) is int:
        serial_number = [serial_number]

    controller_ports = {}
    for port in arduino_device_ports:
        try:
            dev_serial_number = arduino_device_ports[port]['serial_number']
        except KeyError:
            break
        if (serial_number is None) or (dev_serial_number in serial_number):
            controller_ports[port] = {'serial_number': dev_serial_number}
    return controller_ports

def findPwmControllerPort(baudrate=None, serial_number=None, try_ports=None, debug=DEBUG):
    controller_ports = findPwmControllerPorts(baudrate=baudrate,
                                              serial_number=serial_number,
                                              try_ports=try_ports,
                                              debug=debug)
    if len(controller_ports) == 1:
        return controller_ports.keys()[0]
    elif len(controller_ports) == 0:
        arduino_device_ports = findArduinoDevicePorts(baudrate=baudrate,
                                                      model_number=DEVICE_MODEL_NUMBER,
                                                      serial_number=serial_number,
                                                      try_ports=try_ports,
                                                      debug=debug)
        err_str = 'Could not find pwm controller. Check connections and permissions.\n'
        err_str += 'Tried ports: ' + str(arduino_device_ports)
        raise RuntimeError(err_str)
    else:
        err_str = 'Found more than one pwm controller. Specify port or serial_number.\n'
        err_str += 'Matching ports: ' + str(controller_ports)
        raise RuntimeError(err_str)


def isPwmControllerPortInfo(port_info):
    return port_info['model_number'] == DEVICE_MODEL_NUMBER


def pwmControllerCli():
    debug = False
    dev = PwmController(debug=debug)
    # dev.printCommands()

    parser = argparse.ArgumentParser(description='Pwm Controller')
    subparsers = parser.add_subparsers(dest='subparser_name',help='sub-command help')

    # create the parser for the "info" command
    parser_info = subparsers.add_parser('info', help='info help')
    parser_info.add_argument('-c','--commands',action="store_true",
                             help='print device commands')
    parser_info.add_argument('-i','--device-info',action="store_true",
                             help='print device info')

    # create the parser for the "gate" command
    parser_gate = subparsers.add_parser('gate', help='gate help')
    parser_gate.add_argument('-o','--open-gates',action="store_true",
                             help='open gates')
    parser_gate.add_argument('-c','--close-gates',action="store_true",
                             help='close gates')
    parser_gate.add_argument('-a','--servo-angle',
                             help='gate servo angle degrees: 0-180')
    parser_gate.add_argument('-t','--tunnel',nargs=1,default='all',
                             help='tunnel number: integer from 0 to (numTunnels-1), default=all')
    parser_gate.add_argument('-g','--gate',nargs=1,default='all',
                             help='gate number: integer from 0 to (numGatesPerTunnel-1), default=all')

    # create the parser for the "led" command
    parser_led = subparsers.add_parser('led', help='led help')
    parser_led.add_argument('--on',action="store_true",
                            help='turn on leds')
    parser_led.add_argument('--off',action="store_true",
                            help='turn off leds')
    parser_led.add_argument('-d','--duty-cycle',
                            help='set led pwm duty cycle %: 0-100')
    parser_led.add_argument('-t','--tunnel',nargs=1,default='all',
                            help='tunnel number: integer from 0 to (numTunnels-1), default=all')
    parser_led.add_argument('-l','--led',nargs=1,default='all',
                            help='led number: integer from 0 to (numLedsPerTunnel-1), default=all')

    # create the parser for the "set" command
    parser_set = subparsers.add_parser('set', help='set help')
    parser_set.add_argument('-s','--serial-number',
                            help='set device serial number: 0-255')
    parser_set.add_argument('-o','--open',
                            help='"Open" gate servo angle degrees: 20-165')
    parser_set.add_argument('-c','--close',
                            help='"Close" gate servo angle: 20-165')
    parser_set.add_argument('--on',
                            help='"On" led duty cycle %: 0-100')

    args = parser.parse_args()

    if (args.subparser_name == 'gate') or (args.subparser_name == 'led'):
        tunnel = args.tunnel
        if tunnel != 'all':
            tunnel = int(args.tunnel[0])
        if args.subparser_name == 'gate':
            gate = args.gate
            if gate != 'all':
                gate = int(args.gate[0])
            if args.open_gates:
                dev.updateGate(tunnel,gate,'open')
            elif args.close_gates:
                dev.updateGate(tunnel,gate,'close')
            elif args.servo_angle:
                dev.updateGate(tunnel,gate,args.servo_angle)
        elif args.subparser_name == 'led':
            led = args.led
            if led != 'all':
                led = int(args.led[0])
            if args.on:
                dev.updateLed(tunnel,led,'on')
            elif args.off:
                dev.updateLed(tunnel,led,'off')
            elif args.duty_cycle:
                dev.updateLed(tunnel,led,args.duty_cycle)
    elif args.subparser_name == 'info':
        if args.commands:
            dev.printCommands()
        if args.device_info:
            print(str(dev.getDevInfo()))
    elif args.subparser_name == 'set':
        if args.serial_number is not None:
            dev.setSerialNumber(args.serial_number)
        if args.open is not None:
            dev.setGateOpenServoAngle(args.open)
        if args.close is not None:
            dev.setGateCloseServoAngle(args.close)
        if args.on is not None:
            dev.setLedOnDutyCycle(args.on)

# -----------------------------------------------------------------------------------------
if __name__ == '__main__':
    pwmControllerCli()
