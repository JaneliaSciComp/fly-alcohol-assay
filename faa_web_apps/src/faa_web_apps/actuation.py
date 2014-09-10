#!python
from __future__ import print_function
import flask,flask_sijax
import argparse
import os
import signal
import json

from faa_actuation import Actuation


network_port = 5000

# Setup application
app = flask.Flask(__name__)
# Setup application w/ sijax
path = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_STATIC_PATH"] = path
app.config["SIJAX_JSON_URI"] = '/static/js/sijax/json2.js'
flask_sijax.Sijax(app)

class SijaxHandler(object):
    """A container class for all Sijax handlers.

    Grouping all Sijax handler functions in a class
    (or a Python module) allows them all to be registered with
    a single line of code.
    """

    @staticmethod
    def updateLed(obj_response,tunnel,led,duty_cycle):
        print('updateLed: tunnel={0},led={1},duty_cycle={2}'.format(tunnel,led,duty_cycle))
        if not test:
            actuation.controller.updateLed(tunnel,led,duty_cycle)

    @staticmethod
    def updateGate(obj_response,tunnel,gate,angle):
        print('updateGate: tunnel={0},gate={1},angle={2}'.format(tunnel,gate,angle))
        if not test:
            actuation.controller.updateGate(tunnel,gate,angle)

    @staticmethod
    def setOdorValveOn(obj_response,device,valve):
        print('setOdorValveOn: device={0},valve={1}'.format(device,valve))
        if not test:
            actuation.olfactometers[device].setOdorValveOn(valve)

    @staticmethod
    def setOdorValvesOff(obj_response,device):
        print('setOdorValvesOff: device={0}'.format(device))
        if not test:
            actuation.olfactometers[device].setOdorValvesOff()

    @staticmethod
    def setMfcFlowRate(obj_response,device,mfc,percent_capacity):
        print('setMfcFlowRate: device={0},mfc={1},percent_capacity={2}'.format(device,mfc,percent_capacity))
        if not test:
            actuation.olfactometers[device].setMfcFlowRate(mfc,percent_capacity)

    @staticmethod
    def updateMfcFlowRateSettings(obj_response,device,mfc):
        if not test:
            percent_capacity = actuation.olfactometers[device].getMfcFlowRateSetting(mfc)
        else:
            percent_capacity = (device + 10) + 2*(mfc + 2)
            print('updateMfcFlowRateSettings: device={0},mfc={1},percent_capacity={2}'.format(device,mfc,percent_capacity))
        obj_response.script("update_mfc_flow_rate_settings({0},{1},{2});".format(device,mfc,percent_capacity))

    @staticmethod
    def updateInfo(obj_response):
        print("Getting info!")
        for device in range(4):
            if not test:
                if device == 3:
                    info_dict = actuation.controller.getControllerInfo()
                else:
                    info_dict = actuation.olfactometers[device].getArduinoOlfactometerInfo()
            else:
                info_dict = {'a':1, 'b':'foo', 'c':[False,None, {'d':{'e':1.3e5}}]}
            info_str = json.dumps(info_dict,sort_keys=True,separators=(',', ':'))
            obj_response.script("display_device_info({0},{1});".format(device,info_str))

    # @staticmethod
    # def getMfcFlowRateSetting(obj_response,device,mfc):
    #     obj_response.script("update_odor0mfc0(78);")
    #     if not test:
    #         percent_capacity = actuation.olfactometers[device].getMfcFlowRateSetting(mfc)
    #     else:
    #         percent_capacity = '?'
    #         print('getMfcFlowRateSetting: device={0},mfc={1},percent_capacity={2}'.format(device,mfc,percent_capacity))

    # @staticmethod
    # def getMfcFlowRateMeasure(obj_response,device,mfc):
    #     if not test:
    #         percent_capacity = actuation.olfactometers[device].getMfcFlowRateMeasure(mfc)
    #     else:
    #         percent_capacity = '?'
    #         print('getMfcFlowRateMeasure: device={0},mfc={1},percent_capacity={2}'.format(device,mfc,percent_capacity))


@flask_sijax.route(app, "/")
def index():
    if flask.g.sijax.is_sijax_request:
        # The request looks like a valid Sijax request
        # Let's register the handlers and tell Sijax to process it
        flask.g.sijax.register_object(SijaxHandler)
        return flask.g.sijax.process_request()

    return flask.render_template('actuation.html')

@app.route('/kill')
def abort():
    actuation.controller.close()
    pid = os.getpid()
    os.kill(pid,signal.SIGTERM)

@app.route('/controller/updateLed')
def updateLed():
    try:
        tunnel = int(flask.request.args['tunnel'])
    except KeyError:
        tunnel = 'all'
    try:
        led = int(flask.request.args['led'])
    except KeyError:
        led = 'all'
    try:
        duty_cycle = int(flask.request.args['duty_cycle'])
    except KeyError:
        duty_cycle = 'off'
    actuation.controller.updateLed(tunnel,led,duty_cycle)
    return 'tunnel: {tunnel}, led: {led}, duty_cycle: {duty_cycle}'.format(tunnel=tunnel,
                                                                           led=led,
                                                                           duty_cycle=duty_cycle)

@app.route('/controller/updateGate')
def updateGate():
    try:
        tunnel = int(flask.request.args['tunnel'])
    except KeyError:
        tunnel = 'all'
    try:
        gate = int(flask.request.args['gate'])
    except KeyError:
        gate = 'all'
    try:
        angle = int(flask.request.args['angle'])
    except KeyError:
        angle = 'close'
    actuation.controller.updateGate(tunnel,gate,angle)
    return 'tunnel: {tunnel}, gate: {gate}, angle: {angle}'.format(tunnel=tunnel,
                                                                   gate=gate,
                                                                   angle=angle)

@app.route('/olfactometer/setOdorValveOn')
def setOdorValveOn():
    try:
        device = int(flask.request.args['device'])
    except KeyError:
        device = 0
    try:
        valve = int(flask.request.args['valve'])
        actuation.olfactometers[device].setOdorValveOn(valve)
        return 'device: {device} valve: {valve}, set: on'.format(device=device,
                                                                 valve=valve)
    except KeyError:
        pass

@app.route('/olfactometer/setOdorValvesOff')
def setOdorValvesOff():
    try:
        device = int(flask.request.args['device'])
    except KeyError:
        device = 0
    try:
        actuation.olfactometers[device].setOdorValvesOff()
        return 'device: {device} valves, set: off'.format(device=device)
    except KeyError:
        pass

@app.route('/olfactometer/setMfcFlowRate')
def setMfcFlowRate():
    try:
        device = int(flask.request.args['device'])
    except KeyError:
        device = 0
    try:
        mfc = int(flask.request.args['mfc'])
        percent_capacity = int(flask.request.args['percent_capacity'])
        actuation.olfactometers[device].setMfcFlowRate(mfc,percent_capacity)
        return 'device: {device}, mfc: {mfc}, percent_capacity: {percent_capacity}'.format(device=device,
                                                                                           mfc=mfc,
                                                                                           percent_capacity=percent_capacity)
    except KeyError:
        pass

@app.route('/olfactometer/getMfcFlowRateSetting')
def getMfcFlowRateSetting():
    try:
        device = int(flask.request.args['device'])
    except KeyError:
        device = 0
    try:
        mfc = int(flask.request.args['mfc'])
        percent_capacity = actuation.olfactometers[device].getMfcFlowRateSetting(mfc)
        return 'device: {device}, mfc: {mfc}, percent_capacity: {percent_capacity}'.format(device=device,
                                                                                           mfc=mfc,
                                                                                           percent_capacity=percent_capacity)
    except KeyError:
        pass

@app.route('/olfactometer/getMfcFlowRateMeasure')
def getMfcFlowRateMeasure():
    try:
        device = int(flask.request.args['device'])
    except KeyError:
        device = 0
    try:
        mfc = int(flask.request.args['mfc'])
        percent_capacity = actuation.olfactometers[device].getMfcFlowRateMeasure(mfc)
        return 'device: {device}, mfc: {mfc}, percent_capacity: {percent_capacity}'.format(device=device,
                                                                                           mfc=mfc,
                                                                                           percent_capacity=percent_capacity)
    except KeyError:
        pass

def actuationWebserver():
    global actuation
    actuation = None

    global test
    test = False

    parser = argparse.ArgumentParser(description='Actuation Webserver')
    parser.add_argument('-l', '--local',
                        help='start the server in local-only debug mode',
                        action='store_true')
    parser.add_argument('-t', '--test',
                        help='test by printing commands rather than sending them to hardware',
                        action='store_true')
    parser.add_argument('-d', '--debug',
                        help='debug mode for controller and olfactometer',
                        action='store_true')
    args = parser.parse_args()

    server = 'remote'
    debug = False
    if args.debug:
        debug = True
    if args.test:
        test = True
    if args.local:
        server = 'local'

    # Open connection to actuation
    if not test:
        actuation = Actuation(debug=debug)

    if server == 'local':
        print(' * using debug server - localhost only')
        app.run(debug=True)
    else:
        print(' * using builtin server - remote access possible')
        app.run(host='0.0.0.0')


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    actuationWebserver()
