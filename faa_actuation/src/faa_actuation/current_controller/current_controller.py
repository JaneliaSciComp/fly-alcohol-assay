from __future__ import print_function, division
import atexit
import time
import argparse

from arduino_device import ArduinoDevice, ArduinoDevices, findArduinoDevicePorts


DEBUG = False
BAUDRATE = 'default'
DEVICE_MODEL_NUMBER = 2346

class CurrentController(ArduinoDevice):

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
            port =  findCurrentControllerPort(baudrate=kwargs['baudrate'],
                                              serial_number=serial_number)
            kwargs.update({'port': port})
        super(CurrentController,self).__init__(*args,**kwargs)
        atexit.register(self._exitCurrentController)

    def _exitCurrentController(self):
        pass

    def getCurrentControllerInfo(self):
        controller_info = self.getArduinoDeviceInfo()
        dev_info = self.getDevInfo()
        controller_info.update(dev_info)
        return controller_info


# device_names example:
# [{'port':'/dev/ttyACM0',
#   'device_name':'controller0'},
#  {'serial_number':3,
#   'device_name':'controller1'}]
class CurrentControllers(ArduinoDevices):

    def __init__(self,*args,**kwargs):
        if ('use_ports' not in kwargs) or (kwargs['use_ports'] is None):
            kwargs['use_ports'] = findCurrentControllerPorts(*args,**kwargs)
        super(CurrentControllers,self).__init__(*args,**kwargs)

    def appendDevice(self,*args,**kwargs):
        self.append(CurrentController(*args,**kwargs))

    def getCurrentControllerInfo(self):
        controller_info = []
        for device_index in range(len(self)):
            dev = self[device_index]
            controller_info.append(dev.getCurrentControllerInfo())
        return controller_info


def findCurrentControllerPorts(baudrate=None, serial_number=None, try_ports=None, debug=DEBUG):
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

def findCurrentControllerPort(baudrate=None, serial_number=None, try_ports=None, debug=DEBUG):
    controller_ports = findCurrentControllerPorts(baudrate=baudrate,
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
        err_str = 'Could not find current controller. Check connections and permissions.\n'
        err_str += 'Tried ports: ' + str(arduino_device_ports)
        raise RuntimeError(err_str)
    else:
        err_str = 'Found more than one current controller. Specify port or serial_number.\n'
        err_str += 'Matching ports: ' + str(controller_ports)
        raise RuntimeError(err_str)


def isCurrentControllerPortInfo(port_info):
    return port_info['model_number'] == DEVICE_MODEL_NUMBER


def currentControllerCli():
    debug = False
    dev = CurrentController(debug=debug)
    # dev.printCommands()

    parser = argparse.ArgumentParser(description='Current Controller')
    subparsers = parser.add_subparsers(dest='subparser_name',help='sub-command help')

    # create the parser for the "info" command
    parser_info = subparsers.add_parser('info', help='info help')
    parser_info.add_argument('-c','--commands',action="store_true",
                             help='print device commands')
    parser_info.add_argument('-i','--device-info',action="store_true",
                             help='print device info')

    args = parser.parse_args()

    if args.subparser_name == 'info':
        if args.commands:
            dev.printCommands()
        if args.device_info:
            print(str(dev.getDevInfo()))

# -----------------------------------------------------------------------------------------
if __name__ == '__main__':
    currentControllerCli()
